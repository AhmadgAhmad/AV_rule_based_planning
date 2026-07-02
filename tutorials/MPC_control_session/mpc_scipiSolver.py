"""
MPC Receding Horizon — Animated Simulation
==========================================
Dynamic Bicycle Model  ·  Quintic Poly Reference  ·  SLSQP Optimizer

This file is a self-contained teaching demo that animates:
  1. The reference trajectory (what the MPC is chasing)
  2. The prediction horizon (what MPC thinks will happen over N steps)
  3. The actual driven path (what really happened after applying u₀)
  4. Live dashboards: state variables, control inputs, tracking error

Run:
    python mpc_simulation.py

Controls:
    SPACE  — pause / resume
    R      — restart
    Q      — quit

Author: Nidhi  (mentored by Ahmad Ahmad, Boston University)
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.patches import FancyArrowPatch
from scipy.optimize import minimize
from dataclasses import dataclass, field
from typing import Optional
import warnings
warnings.filterwarnings('ignore')


# ═══════════════════════════════════════════════════════════════════
# 1.  VEHICLE & MPC PARAMETERS
# ═══════════════════════════════════════════════════════════════════

@dataclass
class VehicleParams:
    m:  float = 1500.0      # mass [kg]
    Iz: float = 2500.0      # yaw inertia [kg·m²]
    lf: float = 1.2         # front axle to CoM [m]
    lr: float = 1.6         # rear axle to CoM [m]
    Cf: float = 80000.0     # front cornering stiffness [N/rad]
    Cr: float = 80000.0     # rear cornering stiffness [N/rad]
    v_min:     float = 1.0
    v_max:     float = 14.0
    delta_max: float = 0.45
    a_max:     float = 3.0
    a_min:     float = -5.0

@dataclass
class MPCParams:
    N:  int   = 12          # prediction horizon
    dt: float = 0.1         # time step [s]
    Q:  np.ndarray = field(default_factory=lambda: np.diag([15., 15., 8., 2., 0.5, 0.5]))
    Qf: np.ndarray = field(default_factory=lambda: np.diag([75., 75., 40., 10., 2.5, 2.5]))
    R:  np.ndarray = field(default_factory=lambda: np.diag([8., 1.]))
    Rd: np.ndarray = field(default_factory=lambda: np.diag([40., 4.]))

VP  = VehicleParams()
MPC = MPCParams()


# ═══════════════════════════════════════════════════════════════════
# 2.  DYNAMIC BICYCLE MODEL
# ═══════════════════════════════════════════════════════════════════

def tire_forces(state: np.ndarray, delta: float):
    """Linear tire model → cornering forces."""
    x, y, psi, vx, vy, r = state
    vx = max(vx, VP.v_min)
    alpha_f = delta - np.arctan2(vy + VP.lf * r, vx)
    alpha_r =       - np.arctan2(vy - VP.lr * r, vx)
    return VP.Cf * alpha_f, VP.Cr * alpha_r

def f_cont(state: np.ndarray, u: np.ndarray) -> np.ndarray:
    """Continuous-time equations of motion."""
    x, y, psi, vx, vy, r = state
    delta, a = u
    Ff, Fr = tire_forces(state, delta)
    return np.array([
        vx * np.cos(psi) - vy * np.sin(psi),   # ẋ
        vx * np.sin(psi) + vy * np.cos(psi),   # ẏ
        r,                                       # ψ̇
        a + r * vy,                              # v̇x
        (Ff + Fr) / VP.m - r * vx,              # v̇y
        (VP.lf * Ff - VP.lr * Fr) / VP.Iz,     # ṙ
    ])

def rk4(state: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """4th-order Runge-Kutta integration."""
    k1 = f_cont(state,            u)
    k2 = f_cont(state + dt/2*k1, u)
    k3 = f_cont(state + dt/2*k2, u)
    k4 = f_cont(state + dt  *k3, u)
    return state + (dt/6)*(k1 + 2*k2 + 2*k3 + k4)

def rollout(x0: np.ndarray, U: np.ndarray, dt: float) -> np.ndarray:
    """Predict trajectory: returns (N+1, 6) state array."""
    N = len(U)
    X = np.zeros((N+1, 6))
    X[0] = x0
    for k in range(N):
        X[k+1] = rk4(X[k], U[k], dt)
    return X


# ═══════════════════════════════════════════════════════════════════
# 3.  REFERENCE TRAJECTORY  (figure-8 / S-curve for demo variety)
# ═══════════════════════════════════════════════════════════════════

def build_reference(shape: str = 'scurve', n_pts: int = 300, v_ref: float = 7.0):
    """Build a smooth reference path."""
    t = np.linspace(0, 2*np.pi, n_pts)
    if shape == 'scurve':
        rx = np.linspace(0, 80, n_pts)
        ry = 6.0 * np.sin(0.12 * rx)
    elif shape == 'figure8':
        rx = 30 * np.sin(t)
        ry = 15 * np.sin(2*t)
    elif shape == 'oval':
        rx = 40 * np.cos(t) - 40
        ry = 18 * np.sin(t)
    else:  # straight
        rx = np.linspace(0, 80, n_pts)
        ry = np.zeros(n_pts)

    psi = np.arctan2(np.gradient(ry), np.gradient(rx))
    vx  = np.full(n_pts, v_ref)
    return rx, ry, psi, vx


# ═══════════════════════════════════════════════════════════════════
# 4.  MPC OPTIMIZER
# ═══════════════════════════════════════════════════════════════════

def get_ref_window(rx, ry, rpsi, rvx, state: np.ndarray, N: int):
    """Find closest reference point and return N+1 step window."""
    dists = np.hypot(rx - state[0], ry - state[1])
    idx   = int(np.argmin(dists))
    idxs  = [min(idx + k, len(rx)-1) for k in range(N+1)]
    return (rx[idxs], ry[idxs], rpsi[idxs], rvx[idxs])

def mpc_cost(U_flat, x0, ref_x, ref_y, ref_psi, ref_vx, u_prev):
    N  = MPC.N
    dt = MPC.dt
    U  = U_flat.reshape(N, 2)
    X  = rollout(x0, U, dt)
    cost = 0.0
    for k in range(N):
        x_ref = np.array([ref_x[k], ref_y[k], ref_psi[k], ref_vx[k], 0., 0.])
        e     = X[k] - x_ref
        cost += e @ MPC.Q @ e + U[k] @ MPC.R @ U[k]
        u_p   = U[k-1] if k > 0 else u_prev
        du    = U[k] - u_p
        cost += du @ MPC.Rd @ du
    e_N = X[N] - np.array([ref_x[N], ref_y[N], ref_psi[N], ref_vx[N], 0., 0.])
    cost += e_N @ MPC.Qf @ e_N
    return cost

def mpc_solve(x0, rx, ry, rpsi, rvx, U_warm, u_prev):
    """Run one MPC solve. Returns (optimal U, predicted X, info)."""
    ref_x, ref_y, ref_psi, ref_vx = get_ref_window(rx, ry, rpsi, rvx, x0, MPC.N)
    N = MPC.N

    bounds = []
    for _ in range(N):
        bounds += [(-VP.delta_max, VP.delta_max), (VP.a_min, VP.a_max)]

    # Speed constraints
    def speed_lb(U_flat, k):
        X = rollout(x0, U_flat.reshape(N,2), MPC.dt)
        return X[k+1, 3] - VP.v_min
    def speed_ub(U_flat, k):
        X = rollout(x0, U_flat.reshape(N,2), MPC.dt)
        return VP.v_max - X[k+1, 3]

    constraints = []
    for k in range(N):
        constraints += [
            {'type':'ineq','fun': lambda u,k=k: speed_lb(u,k)},
            {'type':'ineq','fun': lambda u,k=k: speed_ub(u,k)},
        ]

    res = minimize(
        mpc_cost, U_warm.flatten(),
        args=(x0, ref_x, ref_y, ref_psi, ref_vx, u_prev),
        method='SLSQP', bounds=bounds, constraints=constraints,
        options={'ftol':1e-4, 'maxiter':80, 'disp':False}
    )

    U_opt  = res.x.reshape(N, 2)
    X_pred = rollout(x0, U_opt, MPC.dt)
    # Shift warm-start
    U_next = np.vstack([U_opt[1:], U_opt[-1:]])
    return U_opt, X_pred, U_next, ref_x, ref_y


# ═══════════════════════════════════════════════════════════════════
# 5.  PRE-COMPUTE SIMULATION  (so animation is smooth)
# ═══════════════════════════════════════════════════════════════════

SHAPE   = 'scurve'
V_REF   = 7.0
N_STEPS = 70

print("Pre-computing MPC simulation ...")
rx, ry, rpsi, rvx = build_reference(SHAPE, v_ref=V_REF)

# Initial state: slightly off-track
x0 = np.array([0.0, 1.2, 0.05, 5.5, 0.0, 0.0])

states   = [x0.copy()]
controls = []
horizons = []   # predicted X at each step
ref_wins = []   # reference window at each step
u_warm   = np.zeros((MPC.N, 2))
u_prev   = np.zeros(2)

for step in range(N_STEPS):
    state = states[-1]
    U_opt, X_pred, u_warm, rw_x, rw_y = mpc_solve(
        state, rx, ry, rpsi, rvx, u_warm, u_prev
    )
    u0     = U_opt[0]
    u_prev = u0.copy()
    next_s = rk4(state, u0, MPC.dt)

    states.append(next_s)
    controls.append(u0)
    horizons.append(X_pred.copy())
    ref_wins.append((rw_x.copy(), rw_y.copy()))

    if step % 10 == 0:
        err = np.hypot(state[0]-rw_x[0], state[1]-rw_y[0])
        print(f"  step {step:3d} | pos=({state[0]:.1f},{state[1]:.1f}) "
              f"| vx={state[3]:.2f} | δ={u0[0]:.3f} | a={u0[1]:.2f} "
              f"| err={err:.3f}m")

states   = np.array(states)
controls = np.array(controls)
t_arr    = np.arange(N_STEPS+1) * MPC.dt
print(f"Done. {N_STEPS} steps simulated.\n")


# ═══════════════════════════════════════════════════════════════════
# 6.  FIGURE LAYOUT
# ═══════════════════════════════════════════════════════════════════

# ── Color palette (dark robotics theme) ──────────────────────────
BG       = '#0b0f1a'
SURFACE  = '#111827'
BORDER   = '#1f2d45'
C_REF    = '#94a3b8'   # reference path
C_PRED   = '#f59e0b'   # MPC predicted horizon
C_ACTUAL = '#60a5fa'   # actual driven path
C_CURR   = '#34d399'   # current vehicle position
C_HORIZ  = '#a78bfa'   # horizon end marker
C_STEER  = '#f472b6'   # steering
C_ACCEL  = '#34d399'   # acceleration
C_ERR    = '#f87171'   # tracking error
C_VX     = '#60a5fa'   # speed
C_VY     = '#2dd4bf'   # lateral vel
C_YAW    = '#a78bfa'   # yaw rate

plt.rcParams.update({
    'figure.facecolor':  BG,
    'axes.facecolor':    SURFACE,
    'axes.edgecolor':    BORDER,
    'axes.labelcolor':   '#94a3b8',
    'xtick.color':       '#64748b',
    'ytick.color':       '#64748b',
    'text.color':        '#e2e8f0',
    'grid.color':        BORDER,
    'grid.linewidth':    0.5,
    'font.family':       'monospace',
    'font.size':         9,
})

fig = plt.figure(figsize=(16, 9), facecolor=BG)
fig.suptitle(
    'MPC  ·  Receding Horizon  ·  Dynamic Bicycle Model',
    fontsize=13, color='#e2e8f0', fontweight='bold',
    x=0.5, y=0.98
)

gs = gridspec.GridSpec(
    3, 3,
    figure=fig,
    left=0.05, right=0.97,
    top=0.94,  bottom=0.07,
    hspace=0.55, wspace=0.40
)

# Main world view spans top 2 rows, left 2 cols
ax_world = fig.add_subplot(gs[0:2, 0:2])
# Right column: 3 small panels
ax_steer = fig.add_subplot(gs[0, 2])
ax_accel = fig.add_subplot(gs[1, 2])
# Bottom row: 3 panels
ax_speed = fig.add_subplot(gs[2, 0])
ax_err   = fig.add_subplot(gs[2, 1])
ax_yaw   = fig.add_subplot(gs[2, 2])


# ── World view setup ──────────────────────────────────────────────
ax_world.set_facecolor('#0d1117')
ax_world.plot(rx, ry, color=C_REF, lw=1.2, alpha=0.4,
              linestyle='--', label='Reference path', zorder=1)
# lane boundaries (mock ±2m)
ax_world.plot(rx - 2*np.sin(rpsi), ry + 2*np.cos(rpsi),
              color=BORDER, lw=0.7, ls=':', alpha=0.5)
ax_world.plot(rx + 2*np.sin(rpsi), ry - 2*np.cos(rpsi),
              color=BORDER, lw=0.7, ls=':', alpha=0.5)
ax_world.set_aspect('equal')
ax_world.set_xlabel('X  [m]')
ax_world.set_ylabel('Y  [m]')
ax_world.set_title('World View', color='#e2e8f0', pad=6)
ax_world.grid(True, alpha=0.15)

# Set world limits with padding
pad = 6
ax_world.set_xlim(rx.min()-pad, rx.max()+pad)
ax_world.set_ylim(ry.min()-pad*2, ry.max()+pad*2)

# Animated artists — world
line_actual,   = ax_world.plot([], [], color=C_ACTUAL, lw=2,
                               label='Driven path', zorder=3)
line_pred,     = ax_world.plot([], [], color=C_PRED, lw=2.2,
                               label=f'MPC horizon (N={MPC.N})',
                               zorder=5, alpha=0.9)
line_pred_pts, = ax_world.plot([], [], 'o', color=C_PRED,
                               ms=3.5, zorder=6, alpha=0.6)
scat_curr      = ax_world.scatter([], [], s=120, color=C_CURR,
                                  zorder=8, label='Vehicle')
scat_horiz_end = ax_world.scatter([], [], s=70, color=C_HORIZ,
                                  zorder=7, marker='D',
                                  label='Horizon end')
line_refwin,   = ax_world.plot([], [], color=C_REF, lw=1.8,
                               alpha=0.7, zorder=4, label='Ref window')
# Vehicle body rectangle placeholder
vehicle_patch = mpatches.FancyBboxPatch(
    (0, 0), 4, 2, boxstyle='round,pad=0.1',
    linewidth=1.5, edgecolor=C_CURR, facecolor='#1a2235',
    zorder=9
)
ax_world.add_patch(vehicle_patch)

# Step counter text
step_text = ax_world.text(
    0.02, 0.95, '', transform=ax_world.transAxes,
    color='#e2e8f0', fontsize=9, va='top',
    bbox=dict(facecolor='#0d1117', edgecolor=BORDER,
              boxstyle='round,pad=0.4', alpha=0.85)
)

# Legend
leg = ax_world.legend(
    loc='lower right', fontsize=8,
    facecolor='#0d1117', edgecolor=BORDER,
    labelcolor='#94a3b8', framealpha=0.9
)


# ── Dashboard panel helper ────────────────────────────────────────
def setup_dash(ax, title, ylabel, color, ylim=None):
    ax.set_title(title, color='#e2e8f0', pad=4, fontsize=9)
    ax.set_ylabel(ylabel, fontsize=8)
    ax.set_xlabel('Time [s]', fontsize=8)
    ax.grid(True, alpha=0.2)
    ax.axhline(0, color=BORDER, lw=0.8)
    if ylim:
        ax.set_ylim(*ylim)
    ax.set_xlim(0, t_arr[-1])
    line, = ax.plot([], [], color=color, lw=1.8)
    # "applied action" marker
    pt,   = ax.plot([], [], 'o', color=color, ms=6, zorder=5)
    return line, pt

line_steer, pt_steer = setup_dash(ax_steer, 'Steering  δ', 'rad',
                                   C_STEER,
                                   ylim=(-VP.delta_max*1.3, VP.delta_max*1.3))
ax_steer.axhline( VP.delta_max, color=C_STEER, lw=0.6, ls='--', alpha=0.4)
ax_steer.axhline(-VP.delta_max, color=C_STEER, lw=0.6, ls='--', alpha=0.4)

line_accel, pt_accel = setup_dash(ax_accel, 'Acceleration  a', 'm/s²',
                                   C_ACCEL,
                                   ylim=(VP.a_min*1.2, VP.a_max*1.2))
ax_accel.axhline(VP.a_max, color=C_ACCEL, lw=0.6, ls='--', alpha=0.4)
ax_accel.axhline(VP.a_min, color=C_ACCEL, lw=0.6, ls='--', alpha=0.4)

line_speed, pt_speed = setup_dash(ax_speed, 'Longitudinal Speed  vₓ', 'm/s',
                                   C_VX,
                                   ylim=(0, VP.v_max*1.2))
ax_speed.axhline(V_REF,    color=C_REF, lw=1, ls='--', alpha=0.5)
ax_speed.axhline(VP.v_max, color=C_ERR, lw=0.6, ls='--', alpha=0.4)
ax_speed.axhline(VP.v_min, color=C_ERR, lw=0.6, ls='--', alpha=0.4)
ax_speed.text(t_arr[-1]*0.98, V_REF+0.2, 'v_ref', color=C_REF,
              fontsize=7, ha='right')

# Tracking error
ax_err.set_title('Tracking Error', color='#e2e8f0', pad=4, fontsize=9)
ax_err.set_ylabel('m', fontsize=8)
ax_err.set_xlabel('Time [s]', fontsize=8)
ax_err.grid(True, alpha=0.2)
ax_err.set_xlim(0, t_arr[-1])
ax_err.set_ylim(0, 3.5)
line_err, = ax_err.plot([], [], color=C_ERR, lw=1.8)
pt_err,   = ax_err.plot([], [], 'o', color=C_ERR, ms=6, zorder=5)
ax_err.axhline(0.3, color=C_ERR, lw=0.6, ls='--', alpha=0.4)
ax_err.text(t_arr[-1]*0.98, 0.35, '0.3m target', color=C_ERR,
            fontsize=7, ha='right')

# Yaw rate
line_yaw, pt_yaw = setup_dash(ax_yaw, 'Yaw Rate  r', 'rad/s',
                               C_YAW, ylim=(-0.6, 0.6))


# ── Overlay label for "applied u₀" ───────────────────────────────
applied_label = ax_world.text(
    0, 0, '', color=C_CURR, fontsize=8, zorder=10,
    bbox=dict(facecolor='#0d1117', edgecolor=C_CURR,
              boxstyle='round,pad=0.3', alpha=0.9)
)


# ═══════════════════════════════════════════════════════════════════
# 7.  ANIMATION
# ═══════════════════════════════════════════════════════════════════

paused = [False]
frame_idx = [0]

def on_key(event):
    if event.key == ' ':
        paused[0] = not paused[0]
    elif event.key == 'r':
        frame_idx[0] = 0
    elif event.key == 'q':
        plt.close()

fig.canvas.mpl_connect('key_press_event', on_key)

def update(frame):
    if paused[0]:
        return

    k = frame % N_STEPS

    # ── Precomputed data for this frame ──────────────────────
    state   = states[k]
    hist    = states[:k+1]
    X_pred  = horizons[k]            # (N+1, 6) predicted states
    rw_x, rw_y = ref_wins[k]        # reference window

    ctrl_hist = controls[:k+1] if k > 0 else np.zeros((1, 2))
    t_hist    = t_arr[:k+1]

    u0_delta  = controls[k][0] if k < len(controls) else 0.0
    u0_a      = controls[k][1] if k < len(controls) else 0.0

    err = np.hypot(state[0] - rw_x[0], state[1] - rw_y[0])

    # ── World view ────────────────────────────────────────────
    # Actual path
    line_actual.set_data(hist[:, 0], hist[:, 1])

    # MPC predicted horizon
    line_pred.set_data(X_pred[:, 0], X_pred[:, 1])
    line_pred_pts.set_data(X_pred[:, 0], X_pred[:, 1])

    # Reference window
    line_refwin.set_data(rw_x, rw_y)

    # Current position dot
    scat_curr.set_offsets([[state[0], state[1]]])

    # Horizon end marker
    scat_horiz_end.set_offsets([[X_pred[-1, 0], X_pred[-1, 1]]])

    # Vehicle body rectangle (rotated)
    psi   = state[2]
    L, W  = 4.5, 2.0
    cx, cy = state[0], state[1]
    # corners in body frame
    corners = np.array([
        [-L/2, -W/2], [L/2, -W/2],
        [L/2,  W/2],  [-L/2,  W/2]
    ])
    R_mat = np.array([[np.cos(psi), -np.sin(psi)],
                      [np.sin(psi),  np.cos(psi)]])
    rot   = (R_mat @ corners.T).T
    xs    = rot[:, 0] + cx
    ys    = rot[:, 1] + cy
    # update patch — redraw as polygon
    vehicle_patch.set_visible(False)

    # Draw vehicle as filled polygon
    if hasattr(update, '_vpoly') and update._vpoly in ax_world.patches:
        update._vpoly.remove()
    from matplotlib.patches import Polygon
    poly = Polygon(list(zip(xs, ys)), closed=True,
                   facecolor='#1a2235', edgecolor=C_CURR,
                   linewidth=1.8, zorder=9, alpha=0.9)
    ax_world.add_patch(poly)
    update._vpoly = poly

    # Steering arrow from vehicle center
    arrow_len = 5.0
    arrow_dx  = arrow_len * np.cos(psi + u0_delta)
    arrow_dy  = arrow_len * np.sin(psi + u0_delta)
    if hasattr(update, '_arrow'):
        update._arrow.remove()
    update._arrow = ax_world.annotate(
        '', xy=(cx + arrow_dx, cy + arrow_dy),
        xytext=(cx, cy),
        arrowprops=dict(arrowstyle='->', color=C_STEER,
                        lw=2.0, connectionstyle='arc3,rad=0')
    )

    # Applied action label
    applied_label.set_position((cx + 3, cy + 3))
    applied_label.set_text(
        f'u₀ applied\nδ = {u0_delta:+.3f} rad\na = {u0_a:+.2f} m/s²'
    )

    # Step info
    step_text.set_text(
        f'Step {k}/{N_STEPS}   t = {k*MPC.dt:.1f}s\n'
        f'Horizon N = {MPC.N}  ·  dt = {MPC.dt}s\n'
        f'Track err = {err:.3f} m'
    )

    # ── Dashboard panels ──────────────────────────────────────
    if k > 0:
        t_ctrl = t_arr[:len(ctrl_hist)]
        line_steer.set_data(t_ctrl, ctrl_hist[:, 0])
        pt_steer.set_data([t_ctrl[-1]], [ctrl_hist[-1, 0]])

        line_accel.set_data(t_ctrl, ctrl_hist[:, 1])
        pt_accel.set_data([t_ctrl[-1]], [ctrl_hist[-1, 1]])

        line_speed.set_data(t_hist, hist[:, 3])
        pt_speed.set_data([t_hist[-1]], [state[3]])

        err_hist = np.array([
            np.hypot(states[i, 0] - ref_wins[min(i, len(ref_wins)-1)][0][0],
                     states[i, 1] - ref_wins[min(i, len(ref_wins)-1)][1][0])
            for i in range(k+1)
        ])
        line_err.set_data(t_hist, err_hist)
        pt_err.set_data([t_hist[-1]], [err])

        line_yaw.set_data(t_hist, hist[:, 5])
        pt_yaw.set_data([t_hist[-1]], [state[5]])

    # Auto-pan world view to follow vehicle
    view_w, view_h = 55, 35
    ax_world.set_xlim(cx - view_w/2, cx + view_w/2)
    ax_world.set_ylim(cy - view_h/2, cy + view_h/2)


ani = FuncAnimation(
    fig, update,
    frames=N_STEPS,
    interval=120,     # ms between frames
    blit=False,
    repeat=True
)

# ── Legend / annotation overlays ─────────────────────────────────
# Horizon explanation annotation on first frame
ax_world.annotate(
    f'← Predicted horizon\n   ({MPC.N} steps × {MPC.dt}s = {MPC.N*MPC.dt:.1f}s)',
    xy=(8, 3), xytext=(12, 8),
    color=C_PRED, fontsize=8,
    arrowprops=dict(arrowstyle='->', color=C_PRED, lw=1.2),
    bbox=dict(facecolor='#0d1117', edgecolor=C_PRED,
              boxstyle='round,pad=0.3', alpha=0.8)
)

# ── Keyboard hint ─────────────────────────────────────────────────
fig.text(0.5, 0.01,
         'SPACE — pause/resume   ·   R — restart   ·   Q — quit',
         ha='center', color='#475569', fontsize=8)

plt.show()