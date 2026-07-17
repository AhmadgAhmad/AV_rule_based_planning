"""
train_context_model.py — Train the GRU Context Classifier
================================================================
Loads one or more CSV logs written by context_data_logger.py, builds
sliding windows of SEQ_LEN ticks (never crossing a route boundary), trains
ContextGRU, and saves a checkpoint you can load back into
predict_context().

Usage:
    python3 train_context_model.py \
        --data-dir output/context_data \
        --epochs 30 \
        --out context_model.pt

Split strategy:
    Whole CSV files (routes) are assigned to train/val/test — never
    individual rows. Row-level random splitting on a continuous drive
    leaks near-duplicate adjacent frames across splits and inflates
    validation accuracy; see the HTML guide for why this matters.

Author: Ahmad Ahmad | For: Nidhi's Autonomous Driving Course
"""

import argparse
import csv
import glob
import os
import random

import numpy as np
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader

from context_model import ContextGRU, FEATURE_NAMES, CONTEXT_CLASSES, SEQ_LEN, context_loss


# ═══════════════════════════════════════════════════════════════════════════
# DATA LOADING
# ═══════════════════════════════════════════════════════════════════════════

def load_route_csv(path):
    """Returns (features: np.ndarray [T, FEATURE_DIM], labels: List[str])."""
    rows, labels = [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append([float(row[name]) for name in FEATURE_NAMES])
            labels.append(row['label'])
    return np.array(rows, dtype=np.float32), labels


def make_windows(features, labels, seq_len=SEQ_LEN):
    """Slide a length-seq_len window over one route's ticks. The label for
    a window is the label at its LAST tick (predicting "what's the context
    right now" given the recent past, not the future)."""
    X, y = [], []
    for i in range(len(features) - seq_len + 1):
        X.append(features[i:i + seq_len])
        y.append(labels[i + seq_len - 1])
    return X, y


class ContextWindowDataset(Dataset):
    def __init__(self, windows, labels, mean, std):
        self.X = np.stack(windows).astype(np.float32)
        self.X = (self.X - mean) / std
        self.y = np.array([CONTEXT_CLASSES.index(l) for l in labels], dtype=np.int64)

    def __len__(self):
        return len(self.y)

    def __getitem__(self, idx):
        return torch.from_numpy(self.X[idx]), int(self.y[idx])


def split_routes(csv_paths, val_frac=0.15, test_frac=0.15, seed=0):
    """Assign WHOLE files to train/val/test so no route straddles a split."""
    paths = list(csv_paths)
    rng = random.Random(seed)
    rng.shuffle(paths)
    n = len(paths)
    n_val = max(1, int(n * val_frac)) if n > 2 else 0
    n_test = max(1, int(n * test_frac)) if n > 2 else 0
    test_paths = paths[:n_test]
    val_paths = paths[n_test:n_test + n_val]
    train_paths = paths[n_test + n_val:]
    if not train_paths:  # tiny dataset fallback (e.g. a single demo file)
        train_paths, val_paths, test_paths = paths, [], []
    return train_paths, val_paths, test_paths


def build_dataset(paths, seq_len, mean=None, std=None):
    all_windows, all_labels = [], []
    for p in paths:
        feats, labels = load_route_csv(p)
        if len(feats) < seq_len:
            print(f"[data] skipping {p}: only {len(feats)} ticks, need >= {seq_len}")
            continue
        X, y = make_windows(feats, labels, seq_len)
        all_windows.extend(X)
        all_labels.extend(y)

    if mean is None:  # only fit normalization stats on the TRAIN split
        stacked = np.stack(all_windows).astype(np.float32)
        mean = stacked.mean(axis=(0, 1))
        std = stacked.std(axis=(0, 1))
        std[std < 1e-6] = 1e-6

    dataset = ContextWindowDataset(all_windows, all_labels, mean, std)
    return dataset, mean, std


# ═══════════════════════════════════════════════════════════════════════════
# TRAIN / EVAL LOOPS
# ═══════════════════════════════════════════════════════════════════════════

def class_weights_from_labels(dataset, device):
    counts = np.bincount(dataset.y, minlength=len(CONTEXT_CLASSES)).astype(np.float32)
    counts[counts == 0] = 1.0
    weights = counts.sum() / (len(CONTEXT_CLASSES) * counts)
    return torch.tensor(weights, device=device)


def run_epoch(model, loader, optimizer, device, class_weights=None, train=True):
    model.train(train)
    total_loss, correct, n = 0.0, 0, 0
    for xb, yb in loader:
        xb, yb = xb.to(device), yb.to(device)
        if train:
            optimizer.zero_grad()
        logits = model(xb)
        loss = context_loss(logits, yb, class_weights=class_weights)
        if train:
            loss.backward()
            optimizer.step()
        total_loss += loss.item() * len(yb)
        correct += (logits.argmax(dim=-1) == yb).sum().item()
        n += len(yb)
    return total_loss / max(n, 1), correct / max(n, 1)


def main(args):
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"[train] device: {device}")

    csv_paths = sorted(glob.glob(os.path.join(args.data_dir, '*.csv')))
    if not csv_paths:
        raise SystemExit(f"No CSV files found in {args.data_dir}. "
                          f"Run a scenario with ContextDataLogger attached first.")
    print(f"[train] found {len(csv_paths)} route CSVs")

    train_paths, val_paths, test_paths = split_routes(csv_paths, seed=args.seed)
    print(f"[train] split: {len(train_paths)} train / {len(val_paths)} val / {len(test_paths)} test routes")

    train_ds, mean, std = build_dataset(train_paths, args.seq_len)
    val_ds, _, _ = build_dataset(val_paths, args.seq_len, mean=mean, std=std) if val_paths else (None, None, None)
    test_ds, _, _ = build_dataset(test_paths, args.seq_len, mean=mean, std=std) if test_paths else (None, None, None)

    print(f"[train] windows: {len(train_ds)} train"
          + (f" / {len(val_ds)} val" if val_ds else "")
          + (f" / {len(test_ds)} test" if test_ds else ""))

    train_loader = DataLoader(train_ds, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=args.batch_size) if val_ds else None

    model = ContextGRU(hidden_size=args.hidden_size, num_layers=args.num_layers).to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=args.lr)
    class_weights = class_weights_from_labels(train_ds, device) if args.class_weighted else None
    if class_weights is not None:
        print(f"[train] class weights: "
              f"{dict(zip(CONTEXT_CLASSES, class_weights.cpu().numpy().round(2)))}")

    best_val_acc = -1.0
    for epoch in range(1, args.epochs + 1):
        train_loss, train_acc = run_epoch(model, train_loader, optimizer, device,
                                           class_weights=class_weights, train=True)
        msg = f"[epoch {epoch:3d}] train_loss={train_loss:.4f} train_acc={train_acc:.3f}"

        if val_loader:
            with torch.no_grad():
                val_loss, val_acc = run_epoch(model, val_loader, optimizer, device,
                                               class_weights=class_weights, train=False)
            msg += f" | val_loss={val_loss:.4f} val_acc={val_acc:.3f}"
            if val_acc > best_val_acc:
                best_val_acc = val_acc
                save_checkpoint(model, mean, std, args, path=args.out)
                msg += "  (saved)"
        print(msg)

    if not val_loader:
        save_checkpoint(model, mean, std, args, path=args.out)

    if test_ds:
        test_loader = DataLoader(test_ds, batch_size=args.batch_size)
        with torch.no_grad():
            test_loss, test_acc = run_epoch(model, test_loader, optimizer, device,
                                             class_weights=class_weights, train=False)
        print(f"[test] loss={test_loss:.4f} acc={test_acc:.3f}")


def save_checkpoint(model, mean, std, args, path):
    torch.save({
        'model_state_dict': model.state_dict(),
        'feature_names': FEATURE_NAMES,
        'context_classes': CONTEXT_CLASSES,
        'seq_len': args.seq_len,
        'hidden_size': args.hidden_size,
        'num_layers': args.num_layers,
        'norm_mean': mean,
        'norm_std': std,
    }, path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir', default='output/context_data')
    parser.add_argument('--out', default='context_model.pt')
    parser.add_argument('--seq-len', type=int, default=SEQ_LEN)
    parser.add_argument('--hidden-size', type=int, default=48)
    parser.add_argument('--num-layers', type=int, default=1)
    parser.add_argument('--batch-size', type=int, default=64)
    parser.add_argument('--lr', type=float, default=1e-3)
    parser.add_argument('--epochs', type=int, default=30)
    parser.add_argument('--class-weighted', action='store_true',
                         help='up-weight rare classes (PARKING is usually scarce)')
    parser.add_argument('--seed', type=int, default=0)
    main(parser.parse_args())
