"""
context_model.py — Learned Context Detector (GRU sequence classifier)
==========================================================================
Replaces ContextDetector.detect() (planner.py) — currently a hand-written
if/elif on speed limit + junction proximity — with a small sequence model
trained on CARLA telemetry.

Problem framing:
    Multi-class classification over a short window of past feature vectors
    → probability distribution over {INTERSECTION, HIGHWAY, CITY, PARKING}.
    A window, not a single frame, because context is a slowly varying mode:
    one noisy tick of deceleration shouldn't flip the label.

Architecture:
    GRU(feature_dim → hidden_size) over the last SEQ_LEN ticks
        → final hidden state
        → 2-layer MLP head
        → softmax over NUM_CLASSES

See context_data_logger.py for how the input features are collected from
CARLA, and train_context_model.py for the training loop.

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import torch
import torch.nn as nn
import torch.nn.functional as F

# ─── Feature / label schema — MUST match context_data_logger.py exactly ────

FEATURE_NAMES = [
    'speed_mps',
    'accel_long',
    'yaw_rate_dps',
    'speed_limit_kmh',
    'lane_width',
    'lane_type',            # encoded: Driving=0, Parking=1, Shoulder=2, Bidirectional=3, other=-1
    'is_junction',           # 0/1
    'dist_to_junction_m',    # capped at lookahead distance
    'road_curvature',        # heading change (deg) over a short lookahead
    'dist_to_light_m',       # capped; large sentinel if no light nearby
    'light_state',           # none=0, red=1, yellow=2, green=3
    'num_nearby_vehicles',
    'num_nearby_pedestrians',
    'avg_rel_speed_nearby',
]
FEATURE_DIM = len(FEATURE_NAMES)

CONTEXT_CLASSES = ['INTERSECTION', 'HIGHWAY', 'CITY', 'PARKING']
NUM_CLASSES = len(CONTEXT_CLASSES)

SEQ_LEN = 15   # ticks of history fed to the GRU (0.75s at 20 Hz — matches REPLAN_INTERVAL scale)


class ContextGRU(nn.Module):
    """GRU sequence classifier. Input: (batch, seq_len, FEATURE_DIM).
    Output: (batch, NUM_CLASSES) raw logits — apply softmax yourself
    (kept as logits so this composes cleanly with F.cross_entropy)."""

    def __init__(self, feature_dim: int = FEATURE_DIM, hidden_size: int = 48,
                 num_layers: int = 1, num_classes: int = NUM_CLASSES, dropout: float = 0.1):
        super().__init__()
        self.hidden_size = hidden_size
        self.gru = nn.GRU(
            input_size=feature_dim,
            hidden_size=hidden_size,
            num_layers=num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0,
        )
        self.head = nn.Sequential(
            nn.Linear(hidden_size, hidden_size // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_size // 2, num_classes),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """x: (batch, seq_len, feature_dim) → logits (batch, num_classes)."""
        out, _ = self.gru(x)          # out: (batch, seq_len, hidden_size)
        last_hidden = out[:, -1, :]   # final timestep's hidden state
        return self.head(last_hidden)

    @torch.no_grad()
    def predict_proba(self, x: torch.Tensor) -> torch.Tensor:
        self.eval()
        return F.softmax(self.forward(x), dim=-1)

    @torch.no_grad()
    def predict_context(self, x: torch.Tensor):
        """Convenience wrapper for inference: returns (label_str, prob_dict)
        for a single sequence, x shaped (seq_len, feature_dim) or
        (1, seq_len, feature_dim)."""
        self.eval()
        if x.dim() == 2:
            x = x.unsqueeze(0)
        probs = self.predict_proba(x)[0]
        label = CONTEXT_CLASSES[int(torch.argmax(probs))]
        prob_dict = {c: float(p) for c, p in zip(CONTEXT_CLASSES, probs)}
        return label, prob_dict


def context_loss(logits: torch.Tensor, targets: torch.Tensor,
                  class_weights: torch.Tensor = None,
                  prev_logits: torch.Tensor = None,
                  temporal_weight: float = 0.1,
                  label_smoothing: float = 0.05) -> torch.Tensor:
    """Cross-entropy classification loss, with two optional add-ons:

    class_weights   — up-weight rare classes (PARKING is almost always the
                       minority class in naturally-driven CARLA logs).
    prev_logits      — if you construct batches of *consecutive* windows
                       (window at tick t and the same route's window at
                       tick t-1), penalizing large jumps in the predicted
                       distribution when nothing dramatic happened keeps
                       the context signal from flickering and causing
                       eta_p2's weights to jump every replan. Off by
                       default — see train_context_model.py's
                       `--temporal-consistency` flag.
    """
    ce = F.cross_entropy(logits, targets, weight=class_weights, label_smoothing=label_smoothing)
    loss = ce
    if prev_logits is not None:
        p = F.softmax(logits, dim=-1)
        p_prev = F.softmax(prev_logits, dim=-1).detach()
        consistency = F.mse_loss(p, p_prev)
        loss = loss + temporal_weight * consistency
    return loss


def blended_p2_weights(probs: dict, weight_table: dict) -> dict:
    """Turn a soft context distribution into a blended P2 weight vector,
    instead of hard-switching on argmax. Drop-in alternative to indexing
    TWTLEvaluator.WEIGHTS by a single Context enum value — smooths out
    weight jumps right at context boundaries (e.g. 20m before a junction,
    where CITY and INTERSECTION are both plausible).

    probs:        {'INTERSECTION': 0.6, 'HIGHWAY': 0.05, 'CITY': 0.3, 'PARKING': 0.05}
    weight_table: TWTLEvaluator.WEIGHTS keyed by Context enum
    """
    keys = ('comfort', 'efficiency', 'safety_margin')
    blended = {k: 0.0 for k in keys}
    for context_name, p in probs.items():
        w = weight_table_lookup(weight_table, context_name)
        for k in keys:
            blended[k] += p * w[k]
    return blended


def weight_table_lookup(weight_table, context_name: str):
    """weight_table is keyed by the Context enum (planner.py); this looks
    it up by the enum's string value so this module doesn't need to import
    planner.py and create a circular dependency."""
    for enum_key, w in weight_table.items():
        if getattr(enum_key, 'value', str(enum_key)).lower() == context_name.lower():
            return w
    raise KeyError(f"No weight entry for context '{context_name}'")
