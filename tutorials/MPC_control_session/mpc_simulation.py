"""
MPC with Linearized Kinematic Bicycle Model + OSQP
===================================================
Fast QP-based MPC — solves in ~1ms vs SLSQP's seconds.

Key ideas:
  - Linearize the bicycle model around current state → get A, B matrices
  - Write MPC as a standard QP:  min 0.5 z'Pz + q'z   s.t.  l ≤ Az ≤ u
  - Solve with OSQP (active-set QP solver, written in C)
  - Animate with matplotlib: receding horizon, predicted path, applied action

State:   x = [X, Y, psi, v]        (4D)
Control: u = [delta, a]             (2D)

Run:
    python mpc_osqp.py

SPACE — pause/resume   R — restart   Q — quit
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
import osqp
import scipy.sparse as sp
import warnings
warnings.filterwarnings('ignore')


# ═══════════════════════════════════════════════════════
# 1.  PARAMETERS
# ═══════════════════════════════════════════════════════

# Vehicle
L        = 2.8       # wheelbase [m]
V_REF    = 7.0       # reference speed [m/s]
DT       = 0.1       # time step [s]

# MPC horizon
N  = 15              # prediction steps  (N × DT = 1.5s lookahead)
NX = 4               # state dim:    [X, Y, psi, v]
NU = 2               # control dim:  [delta, a]

# State cost  Q  (penalize: X, Y, heading, speed error)
Q_diag  = np.array([4., 4., 2., 10.])
# Terminal cost  Qf = 5 × Q  (arrive cleanly)
Qf_diag = 5 * Q_diag
# Control cost  R
R_diag  = np.array([10., 1.])
# Control rate cost  Rd  (smoothness)
Rd_diag = np.array([30., 3.])

# Constraints
DELTA_MAX =  0.45     # max steering [rad]
A_MAX     =  3.0      # max accel [m/s²]
A_MIN     = -5.0      # max brake [m/s²]
V_MAX     = 14.0      # max speed [m/s]
V_MIN     =  1.0      # min speed [m/s]

# Simulation
N_SIM  = 120          # total simulation steps
X0     = np.array([0., 1.5, 0.08, 5.0])   # slightly off-track start


# ═══════════════════════════════════════════════════════
# 2.  KINEMATIC BICYCLE MODEL
# ═══════════════════════════════════════════════════════

def bicycle_step(x: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """
    Nonlinear kinematic bicycle model — used for TRUE simulation.
    (MPC linearizes this; simulation uses the real nonlinear version)

    Equations:
        Ẋ   = v · cos(ψ)
        Ẏ   = v · sin(ψ)
        ψ̇   = v / L · tan(δ)
        v̇   = a
    """
    X, Y, psi, v = x
    delta, a = u
    v = np.clip(v, V_MIN, V_MAX)

    X_n   = X   + v * np.cos(psi) * dt
    Y_n   = Y   + v * np.sin(psi) * dt
    psi_n = psi + v / L * np.tan(delta) * dt
    v_n   = np.clip(v + a * dt, V_MIN, V_MAX)

    return np.array([X_n, Y_n, psi_n, v_n])


def linearize(x_op: np.ndarray, u_op: np.ndarray, dt: float):
    """
    Linearize bicycle model around operating point (x_op, u_op).

    Returns discrete-time A, B matrices such that:
        x_{k+1} ≈ A @ x_k + B @ u_k  +  (nonlinear remainder)

    A = I + dt · ∂f/∂x  evaluated at (x_op, u_op)
    B =     dt · ∂f/∂u  evaluated at (x_op, u_op)
    """
    X, Y, psi, v = x_op
    delta, a = u_op
    v = max(v, V_MIN)

    # Jacobian ∂f/∂x  (continuous time)
    Ac = np.array([
        [0, 0, -v*np.sin(psi),  np.cos(psi)],
        [0, 0,  v*np.cos(psi),  np.sin(psi)],
        [0, 0,  0,               np.tan(delta)/L],
        [0, 0,  0,               0            ],
    ])

    # Jacobian ∂f/∂u  (continuous time)
    Bc = np.array([
        [0,  0],
        [0,  0],
        [v / (L * np.cos(delta)**2),  0],
        [0,  1],
    ])

    # Euler discretization
    A = np.eye(NX) + dt * Ac
    B = dt * Bc
    return A, B


# ═══════════════════════════════════════════════════════
# 3.  REFERENCE TRAJECTORY
# ═══════════════════════════════════════════════════════

def build_ref(n_pts: int = 400):
    """S-curve reference path."""
    t  = np.linspace(0, 50, n_pts)
    rx = t * 1.6
    ry = 5.0 * np.sin(0.18 * rx)
    rpsi = np.arctan2(np.gradient(ry), np.gradient(rx))
    rv   = np.full(n_pts, V_REF)
    return rx, ry, rpsi, rv

RX, RY, RPSI, RV = build_ref()

def get_ref_window(x: np.ndarray):
    """Find closest ref point and return N+1 ahead."""
    dists = np.hypot(RX - x[0], RY - x[1])
    idx   = int(np.argmin(dists))
    idxs  = [min(idx + k, len(RX)-1) for k in range(N+1)]
    return (RX[idxs], RY[idxs], RPSI[idxs], RV[idxs])


# ═══════════════════════════════════════════════════════
# 4.  OSQP MPC SOLVER
# ═══════════════════════════════════════════════════════

def build_qp(x0: np.ndarray, ref_x, ref_y, ref_psi, ref_v,
             u_prev: np.ndarray):
    """
    Build and solve the MPC QP with OSQP.

    Decision variable z = [x_1,...,x_N, u_0,...,u_{N-1}]
    Size: N*NX + N*NU

    Cost (expanded, no cross terms):
        J = Σ_k  (x_k - xr_k)' Q (x_k - xr_k)
              +  u_k' R u_k
              +  (u_k - u_{k-1})' Rd (u_k - u_{k-1})
          + terminal Qf term

    Rate cost Rd contribution:
        Σ u_k' Rd u_k  - 2 u_k' Rd u_{k-1}  + const
        → adds Rd to each R_k block in P, and cross-terms in q
    """
    nz = N*NX + N*NU

    # ── Linearize along reference (not just x0) ───────────────
    A_list, B_list, d_list = [], [], []
    x_lin = x0.copy()
    for k in range(N):
        # operating point: reference state at step k
        x_op = np.array([ref_x[k], ref_y[k], ref_psi[k], ref_v[k]])
        u_op = np.zeros(NU)   # assume zero control at op point
        A_k, B_k = linearize(x_op, u_op, DT)
        # affine term: d_k = f(x_op,u_op) - A_k x_op - B_k u_op
        d_k = bicycle_step(x_op, u_op, DT) - A_k @ x_op - B_k @ u_op
        A_list.append(A_k)
        B_list.append(B_k)
        d_list.append(d_k)

    # ── Cost matrix P ─────────────────────────────────────────
    Q_blk  = np.diag(Q_diag)
    Qf_blk = np.diag(Qf_diag)
    R_blk  = np.diag(R_diag)
    Rd_blk = np.diag(Rd_diag)

    # State blocks: Q for k<N-1, Qf for k=N-1
    P_state = sp.block_diag(
        [Qf_blk if k == N-1 else Q_blk for k in range(N)],
        format='csc'
    )
    # Control blocks: R + 2Rd (Rd appears twice: u_k and u_{k+1}-u_k terms)
    P_ctrl  = sp.block_diag(
        [R_blk + 2*Rd_blk for _ in range(N)],
        format='csc'
    )
    P = sp.block_diag([P_state, P_ctrl], format='csc')
    # Scale by 2 for OSQP convention (minimizes 0.5 z'Pz + q'z)
    P = 2 * P

    # ── Linear cost q ─────────────────────────────────────────
    q = np.zeros(nz)
    # State tracking terms
    for k in range(N):
        x_ref_k = np.array([ref_x[k+1], ref_y[k+1], ref_psi[k+1], ref_v[k+1]])
        Qk = Qf_blk if k == N-1 else Q_blk
        q[k*NX:(k+1)*NX] = -2 * Qk @ x_ref_k
    # Rate cost terms: -2 Rd u_{k-1} in q for u_k block
    for k in range(N):
        u_km1 = u_prev if k == 0 else np.zeros(NU)  # only first matters
        q[N*NX + k*NU : N*NX + (k+1)*NU] += -2 * Rd_blk @ u_km1

    # ── Equality constraints: dynamics ────────────────────────
    # x_{k+1} = A_k x_k + B_k u_k + d_k
    # Rearranged: x_{k+1} - B_k u_k = A_k x_k + d_k
    Aeq = np.zeros((N*NX, nz))
    beq = np.zeros(N*NX)

    for k in range(N):
        # x_{k+1} part
        Aeq[k*NX:(k+1)*NX,  k*NX:(k+1)*NX] = np.eye(NX)
        # u_k part
        Aeq[k*NX:(k+1)*NX,  N*NX + k*NU : N*NX + (k+1)*NU] = -B_list[k]
        # rhs
        if k == 0:
            beq[0:NX] = A_list[0] @ x0 + d_list[0]
        else:
            # x_k appears on rhs via A_{k-1}: handled by equality chain
            Aeq[k*NX:(k+1)*NX, (k-1)*NX:k*NX] = -A_list[k]
            beq[k*NX:(k+1)*NX] = d_list[k]

    # ── Inequality constraints ────────────────────────────────
    n_ineq = N * (NU + 1)   # NU control + 1 speed per step
    Aineq  = np.zeros((n_ineq, nz))
    l_in   = np.zeros(n_ineq)
    u_in   = np.zeros(n_ineq)
    row = 0
    for k in range(N):
        # delta
        Aineq[row, N*NX + k*NU]     = 1.; l_in[row] = -DELTA_MAX; u_in[row] = DELTA_MAX; row+=1
        # accel
        Aineq[row, N*NX + k*NU + 1] = 1.; l_in[row] = A_MIN;      u_in[row] = A_MAX;     row+=1
        # speed
        Aineq[row, k*NX + 3]        = 1.; l_in[row] = V_MIN;       u_in[row] = V_MAX;     row+=1

    # ── Stack and solve ───────────────────────────────────────
    A_full = sp.vstack([sp.csc_matrix(Aeq),
                        sp.csc_matrix(Aineq)], format='csc')
    l_full = np.concatenate([beq, l_in])
    u_full = np.concatenate([beq, u_in])

    prob = osqp.OSQP()
    prob.setup(P, q, A_full, l_full, u_full,
               warm_starting=True, verbose=False,
               eps_abs=1e-4, eps_rel=1e-4,
               max_iter=4000, polish=True)

    res = prob.solve()

    z   = res.x if res.x is not None else np.zeros(nz)

    X_pred = np.vstack([x0, z[:N*NX].reshape(N, NX)])
    U_opt  = z[N*NX:].reshape(N, NU)
    u0     = np.clip(U_opt[0], [- DELTA_MAX, A_MIN], [DELTA_MAX, A_MAX])
    return u0, X_pred, U_opt


# ═══════════════════════════════════════════════════════
# 5.  PRE-COMPUTE SIMULATION  (fast — should be < 5s)
# ═══════════════════════════════════════════════════════

print("Pre-computing simulation ...")
import time

states   = [X0.copy()]
controls = []
horizons = []
ref_wins = []
u_prev   = np.zeros(NU)

t0 = time.time()
for step in range(N_SIM):
    x     = states[-1]
    rw    = get_ref_window(x)
    u0, X_pred, _ = build_qp(x, *rw, u_prev)
    u_prev = u0.copy()

    states.append(bicycle_step(x, u0, DT))
    controls.append(u0.copy())
    horizons.append(X_pred.copy())
    ref_wins.append(rw)

    if step % 20 == 0:
        err = np.hypot(x[0]-rw[0][0], x[1]-rw[1][0])
        print(f"  step {step:3d} | "
              f"pos=({x[0]:.1f}, {x[1]:.1f}) | "
              f"v={x[3]:.2f} m/s | "
              f"δ={u0[0]:+.3f} rad | "
              f"a={u0[1]:+.2f} m/s² | "
              f"err={err:.3f} m")

elapsed = time.time() - t0
states   = np.array(states)
controls = np.array(controls)
t_arr    = np.arange(N_SIM+1) * DT
print(f"\nDone. {N_SIM} steps in {elapsed:.2f}s  "
      f"({elapsed/N_SIM*1000:.1f} ms/step)\n")


# ═══════════════════════════════════════════════════════
# 6.  ANIMATED VISUALIZATION
# ═══════════════════════════════════════════════════════

# ── Palette ──────────────────────────────────────────
BG      = '#0b0f1a'
SURF    = '#111827'
BORDER  = '#1f2d45'
C_REF   = '#475569'
C_WIN   = '#94a3b8'
C_PRED  = '#f59e0b'
C_ACT   = '#60a5fa'
C_CAR   = '#34d399'
C_HEND  = '#a78bfa'
C_STEER = '#f472b6'
C_ACCEL = '#34d399'
C_ERR   = '#f87171'
C_SPD   = '#60a5fa'
C_YAW   = '#a78bfa'

plt.rcParams.update({
    'figure.facecolor': BG, 'axes.facecolor': SURF,
    'axes.edgecolor': BORDER, 'axes.labelcolor': '#94a3b8',
    'xtick.color': '#64748b', 'ytick.color': '#64748b',
    'text.color': '#e2e8f0', 'grid.color': BORDER,
    'grid.linewidth': 0.5, 'font.family': 'monospace',
    'font.size': 9,
})

fig = plt.figure(figsize=(16, 9), facecolor=BG)
fig.suptitle(
    'MPC Receding Horizon  ·  Kinematic Bicycle Model  ·  OSQP Solver',
    fontsize=12, color='#e2e8f0', fontweight='bold', y=0.98
)

gs = gridspec.GridSpec(3, 3, figure=fig,
                        left=0.05, right=0.97,
                        top=0.93, bottom=0.07,
                        hspace=0.55, wspace=0.42)

ax_w = fig.add_subplot(gs[0:2, 0:2])   # world view
ax_s = fig.add_subplot(gs[0,   2])     # steering
ax_a = fig.add_subplot(gs[1,   2])     # acceleration
ax_v = fig.add_subplot(gs[2,   0])     # speed
ax_e = fig.add_subplot(gs[2,   1])     # error
ax_y = fig.add_subplot(gs[2,   2])     # yaw

# ── World view static elements ────────────────────────
ax_w.set_facecolor('#0d1117')
ax_w.plot(RX, RY, color=C_REF, lw=1.2, ls='--', alpha=0.5,
          label='Reference', zorder=1)
# Lane markings ±3m
ax_w.plot(RX - 3*np.sin(RPSI), RY + 3*np.cos(RPSI),
          color=BORDER, lw=0.8, ls=':', alpha=0.4)
ax_w.plot(RX + 3*np.sin(RPSI), RY - 3*np.cos(RPSI),
          color=BORDER, lw=0.8, ls=':', alpha=0.4)
ax_w.set_aspect('equal')
ax_w.set_xlabel('X [m]'); ax_w.set_ylabel('Y [m]')
ax_w.set_title('World View', color='#e2e8f0', pad=5)
ax_w.grid(True, alpha=0.12)

# Animated lines — world
ln_actual,  = ax_w.plot([], [], color=C_ACT,   lw=2,    label='Driven path', zorder=3)
ln_pred,    = ax_w.plot([], [], color=C_PRED,  lw=2.5,  label=f'MPC horizon (N={N})', zorder=5, alpha=0.9)
ln_pred_pt, = ax_w.plot([], [], 'o', color=C_PRED,  ms=4,    zorder=6, alpha=0.6)
ln_refwin,  = ax_w.plot([], [], color=C_WIN,   lw=1.8,  label='Ref window', zorder=4, alpha=0.7)
sc_curr     = ax_w.scatter([], [], s=130, color=C_CAR,   zorder=8, label='Vehicle')
sc_hend     = ax_w.scatter([], [], s=80,  color=C_HEND,  marker='D', zorder=7, label='Horizon end')

# Applied action annotation
txt_action = ax_w.text(0, 0, '', fontsize=8.5, color='#e2e8f0', zorder=11,
                        bbox=dict(facecolor='#0d1117', edgecolor=C_PRED,
                                  boxstyle='round,pad=0.4', alpha=0.92))
# Step info
txt_info = ax_w.text(0.02, 0.97, '', transform=ax_w.transAxes,
                      fontsize=8.5, color='#e2e8f0', va='top',
                      bbox=dict(facecolor='#0d1117', edgecolor=BORDER,
                                boxstyle='round,pad=0.4', alpha=0.88))
# Horizon bracket annotation  (drawn once, updated position via transform)
ax_w.legend(loc='lower right', fontsize=8, facecolor='#0d1117',
            edgecolor=BORDER, labelcolor='#94a3b8', framealpha=0.9)

# ── Dashboard setup helper ─────────────────────────────
def dash(ax, title, ylabel, color, ylim, hlines=None):
    ax.set_title(title, color='#e2e8f0', pad=4, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_xlabel('t [s]', fontsize=8)
    ax.grid(True, alpha=0.2)
    ax.set_xlim(0, t_arr[-1])
    ax.set_ylim(*ylim)
    ax.axhline(0, color=BORDER, lw=0.8)
    if hlines:
        for y, c, lbl in hlines:
            ax.axhline(y, color=c, lw=0.8, ls='--', alpha=0.5)
            ax.text(t_arr[-1]*0.98, y, lbl, color=c,
                    fontsize=7, ha='right', va='bottom')
    ln,  = ax.plot([], [], color=color, lw=1.8)
    pt,  = ax.plot([], [], 'o', color=color, ms=6, zorder=5)
    return ln, pt

ln_s, pt_s = dash(ax_s, 'Steering  δ', 'rad', C_STEER,
                   (-DELTA_MAX*1.4, DELTA_MAX*1.4),
                   [(DELTA_MAX, C_STEER, '+δmax'),
                    (-DELTA_MAX, C_STEER, '-δmax')])
ln_a, pt_a = dash(ax_a, 'Acceleration  a', 'm/s²', C_ACCEL,
                   (A_MIN*1.2, A_MAX*1.3),
                   [(A_MAX, C_ACCEL, 'amax'),
                    (A_MIN, '#f87171', 'amin')])
ln_v, pt_v = dash(ax_v, 'Longitudinal Speed  v', 'm/s', C_SPD,
                   (0, V_MAX*1.15),
                   [(V_REF, C_WIN, 'v_ref'),
                    (V_MAX, C_ERR, 'vmax')])
ln_e, pt_e = dash(ax_e, 'Tracking Error', 'm', C_ERR,
                   (0, 3.0),
                   [(0.3, C_ERR, '0.3m')])
ln_y, pt_y = dash(ax_y, 'Yaw  ψ', 'rad', C_YAW, (-0.5, 0.5))

# vehicle polygon handle
_vpoly = [None]
_arrow = [None]


# ═══════════════════════════════════════════════════════
# 7.  ANIMATION
# ═══════════════════════════════════════════════════════

paused = [False]

def on_key(ev):
    if ev.key == ' ':   paused[0] = not paused[0]
    elif ev.key == 'r': ani.frame_seq = ani.new_frame_seq()
    elif ev.key == 'q': plt.close()
fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    if paused[0]:
        return
    k = frame % N_SIM

    x       = states[k]
    hist    = states[:k+1]
    X_pred  = horizons[k]            # (N+1, 4)
    rw      = ref_wins[k]
    ctrl    = controls[:max(k,1)]
    t_hist  = t_arr[:k+1]
    t_ctrl  = t_arr[:max(k,1)]

    u0_d = controls[k][0] if k < len(controls) else 0.
    u0_a = controls[k][1] if k < len(controls) else 0.
    err  = np.hypot(x[0]-rw[0][0], x[1]-rw[1][0])

    # ── World view ─────────────────────────────────────
    ln_actual.set_data(hist[:,0], hist[:,1])
    ln_pred.set_data(X_pred[:,0], X_pred[:,1])
    ln_pred_pt.set_data(X_pred[:,0], X_pred[:,1])
    ln_refwin.set_data(rw[0], rw[1])
    sc_curr.set_offsets([[x[0], x[1]]])
    sc_hend.set_offsets([[X_pred[-1,0], X_pred[-1,1]]])

    # Vehicle body
    if _vpoly[0] is not None:
        try: _vpoly[0].remove()
        except: pass
    psi  = x[2]
    LV, WV = 4.5, 2.0
    pts  = np.array([[-LV/2,-WV/2],[LV/2,-WV/2],
                     [LV/2, WV/2],[-LV/2, WV/2]])
    R_   = np.array([[np.cos(psi),-np.sin(psi)],
                     [np.sin(psi), np.cos(psi)]])
    rot  = (R_ @ pts.T).T + np.array([x[0], x[1]])
    poly = Polygon(rot, closed=True, facecolor='#1a2235',
                   edgecolor=C_CAR, lw=2.0, zorder=9, alpha=0.92)
    ax_w.add_patch(poly)
    _vpoly[0] = poly

    # Steering direction arrow
    if _arrow[0] is not None:
        try: _arrow[0].remove()
        except: pass
    aL = 5.5
    _arrow[0] = ax_w.annotate(
        '', xy=(x[0]+aL*np.cos(psi+u0_d), x[1]+aL*np.sin(psi+u0_d)),
        xytext=(x[0], x[1]),
        arrowprops=dict(arrowstyle='->', color=C_STEER, lw=2.2),
        zorder=10
    )

    # Action label
    txt_action.set_position((x[0]+3, x[1]+3.5))
    txt_action.set_text(f'u₀:  δ={u0_d:+.3f}  a={u0_a:+.2f}')

    # Step info box
    txt_info.set_text(
        f'step {k}/{N_SIM}  ·  t={k*DT:.1f}s\n'
        f'N={N}  ·  dt={DT}s  →  {N*DT:.1f}s lookahead\n'
        f'err={err:.3f}m   v={x[3]:.2f}m/s   ψ={x[2]:.3f}rad'
    )

    # Auto-pan
    vw, vh = 60, 38
    ax_w.set_xlim(x[0]-vw/2, x[0]+vw/2)
    ax_w.set_ylim(x[1]-vh/2, x[1]+vh/2)

    # ── Dashboard panels ────────────────────────────────
    if k > 0:
        ln_s.set_data(t_ctrl, ctrl[:,0])
        pt_s.set_data([t_ctrl[-1]], [ctrl[-1,0]])
        ln_a.set_data(t_ctrl, ctrl[:,1])
        pt_a.set_data([t_ctrl[-1]], [ctrl[-1,1]])
        ln_v.set_data(t_hist, hist[:,3])
        pt_v.set_data([t_hist[-1]], [x[3]])
        e_h = [np.hypot(states[i,0]-ref_wins[min(i,len(ref_wins)-1)][0][0],
                        states[i,1]-ref_wins[min(i,len(ref_wins)-1)][1][0])
               for i in range(k+1)]
        ln_e.set_data(t_hist, e_h)
        pt_e.set_data([t_hist[-1]], [err])
        ln_y.set_data(t_hist, hist[:,2])
        pt_y.set_data([t_hist[-1]], [x[2]])


ani = FuncAnimation(fig, update, frames=N_SIM,
                    interval=80, blit=False, repeat=True)

fig.text(0.5, 0.005,
         'SPACE — pause/resume   ·   R — restart   ·   Q — quit',
         ha='center', color='#475569', fontsize=8)

plt.show()