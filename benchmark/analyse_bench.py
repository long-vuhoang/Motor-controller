#!/usr/bin/env python3
"""
analyse_bench.py – Visualise Robstride SocketCAN driver benchmark results.

Usage:
    python3 analyse_bench.py bench_results.csv
    python3 analyse_bench.py bench_results.csv --save plots/

Produces:
    1. Latency comparison bar chart (mean ± stddev, p99 markers)
    2. Per-test CDF (cumulative distribution function)
    3. Jitter timeline (T2 period deviation vs. sample number)
    4. Multi-bus RTL comparison (T4 bus0–bus3 p50/p99 side-by-side)
    5. Summary pass/warn/fail heatmap
"""

import sys
import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np


# ── Threshold definitions (must match report.h) ──────────────────────────────

RTL_PASS_US   =  500.0
RTL_WARN_US   = 1000.0
JITTER_PASS_US =  50.0
JITTER_WARN_US = 100.0
MUTEX_PASS_US  =  10.0
MUTEX_WARN_US  =  50.0

THRESHOLDS = {
    "rtl":    (RTL_PASS_US,    RTL_WARN_US),
    "jitter": (JITTER_PASS_US, JITTER_WARN_US),
    "mutex":  (MUTEX_PASS_US,  MUTEX_WARN_US),
}


def classify(row):
    name = row["test_name"].lower()
    if "jitter" in name or "period" in name:
        p, w = THRESHOLDS["jitter"]
    elif "mutex" in name or "lock" in name:
        p, w = THRESHOLDS["mutex"]
    else:
        p, w = THRESHOLDS["rtl"]

    if row["p99_us"] < p:
        return "PASS", "#2ecc71"   # green
    elif row["p99_us"] < w:
        return "WARN", "#f39c12"   # orange
    else:
        return "FAIL", "#e74c3c"   # red


# ── Plot 1: Latency bar chart ─────────────────────────────────────────────────

def plot_bar_chart(df: pd.DataFrame, save_dir=None):
    fig, ax = plt.subplots(figsize=(14, 6))

    x = np.arange(len(df))
    colours = [classify(row)[1] for _, row in df.iterrows()]

    bars = ax.bar(x, df["mean_us"], yerr=df["stddev_us"],
                  color=colours, alpha=0.8, capsize=4,
                  label="Mean ± Stddev (µs)")

    # p99 markers
    ax.scatter(x, df["p99_us"], marker="D", color="navy",
               zorder=5, label="p99 (µs)", s=40)

    # p99.9 markers
    ax.scatter(x, df["p999_us"], marker="^", color="purple",
               zorder=5, label="p99.9 (µs)", s=40)

    ax.set_xticks(x)
    ax.set_xticklabels(df["test_name"], rotation=35, ha="right", fontsize=8)
    ax.set_ylabel("Latency (µs)")
    ax.set_title("Robstride SocketCAN Driver – Latency Overview")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    _save_or_show(fig, save_dir, "01_bar_chart.png")


# ── Plot 2: CDF per test ──────────────────────────────────────────────────────

def plot_cdf(df: pd.DataFrame, save_dir=None):
    """
    Approximate CDF from available percentile points.
    We reconstruct the CDF from p50/p90/p99/p99.9/max quantiles.
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    for _, row in df.iterrows():
        pts_x = [row["min_us"], row["p50_us"], row["p90_us"],
                 row["p99_us"], row["p999_us"], row["max_us"]]
        pts_y = [0.0, 0.50, 0.90, 0.99, 0.999, 1.0]
        verdict, colour = classify(row)
        ax.plot(pts_x, pts_y, marker="o", label=f"{row['test_name']} [{verdict}]",
                color=colour, alpha=0.8, linewidth=1.5)

    ax.axvline(RTL_PASS_US,    color="#2ecc71", linestyle="--", alpha=0.5, label="RTL PASS (500µs)")
    ax.axvline(RTL_WARN_US,    color="#f39c12", linestyle="--", alpha=0.5, label="RTL WARN (1ms)")
    ax.axvline(JITTER_PASS_US, color="#2ecc71", linestyle=":",  alpha=0.5, label="Jitter PASS (50µs)")

    ax.set_xlabel("Latency (µs)")
    ax.set_ylabel("Cumulative Probability")
    ax.set_title("Latency CDF (reconstructed from percentile points)")
    ax.set_xlim(left=0)
    ax.set_ylim(0, 1.02)
    ax.legend(fontsize=7, ncol=2)
    ax.grid(alpha=0.3)

    plt.tight_layout()
    _save_or_show(fig, save_dir, "02_cdf.png")


# ── Plot 3: RTL p50 / p99 comparison ─────────────────────────────────────────

def plot_rtl_comparison(df: pd.DataFrame, save_dir=None):
    rtl = df[df["test_name"].str.contains("rtl|RTL|round", case=False, na=False)]
    if rtl.empty:
        print("[analyse] No RTL rows found, skipping plot 3.")
        return

    fig, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(rtl))
    w = 0.35

    ax.bar(x - w/2, rtl["p50_us"], width=w, label="p50", color="#3498db", alpha=0.8)
    ax.bar(x + w/2, rtl["p99_us"], width=w, label="p99", color="#e74c3c", alpha=0.8)

    ax.axhline(RTL_PASS_US, color="#2ecc71", linestyle="--", label=f"PASS limit {RTL_PASS_US}µs")
    ax.axhline(RTL_WARN_US, color="#f39c12", linestyle="--", label=f"WARN limit {RTL_WARN_US}µs")

    ax.set_xticks(x)
    ax.set_xticklabels(rtl["test_name"], rotation=25, ha="right", fontsize=8)
    ax.set_ylabel("Latency (µs)")
    ax.set_title("Round-Trip Latency: p50 vs p99")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    plt.tight_layout()
    _save_or_show(fig, save_dir, "03_rtl_p50_p99.png")


# ── Plot 4: Jitter comparison (T2a SCHED_OTHER vs T2b SCHED_FIFO) ─────────────

def plot_jitter_comparison(df: pd.DataFrame, save_dir=None):
    jitter = df[df["test_name"].str.contains("jitter", case=False, na=False)]
    if jitter.empty:
        print("[analyse] No jitter rows found, skipping plot 4.")
        return

    fig, ax = plt.subplots(figsize=(8, 5))
    labels = jitter["test_name"].tolist()
    metrics = ["mean_us", "p50_us", "p90_us", "p99_us", "p999_us", "max_us"]
    x = np.arange(len(metrics))

    for i, (_, row) in enumerate(jitter.iterrows()):
        vals = [row[m] for m in metrics]
        ax.plot(x, vals, marker="o", label=labels[i], linewidth=2)

    ax.axhline(JITTER_PASS_US, color="#2ecc71", linestyle="--",
               label=f"PASS limit {JITTER_PASS_US}µs")
    ax.axhline(JITTER_WARN_US, color="#f39c12", linestyle="--",
               label=f"WARN limit {JITTER_WARN_US}µs")

    ax.set_xticks(x)
    ax.set_xticklabels(["Mean", "p50", "p90", "p99", "p99.9", "Max"])
    ax.set_ylabel("Jitter (µs)")
    ax.set_title("Control-Loop Wakeup Jitter: SCHED_OTHER vs SCHED_FIFO")
    ax.legend()
    ax.grid(alpha=0.3)

    plt.tight_layout()
    _save_or_show(fig, save_dir, "04_jitter_comparison.png")


# ── Plot 5: Pass/Warn/Fail heatmap ────────────────────────────────────────────

def plot_heatmap(df: pd.DataFrame, save_dir=None):
    metrics = ["mean_us", "p50_us", "p90_us", "p99_us", "p999_us", "max_us"]
    labels  = ["Mean",    "p50",    "p90",    "p99",    "p99.9",   "Max"]

    data = df[metrics].values
    norm_data = np.zeros_like(data)

    # Normalise each row to its WARN threshold for colour mapping
    for i, (_, row) in enumerate(df.iterrows()):
        _, colour = classify(row)
        _, w = THRESHOLDS.get("jitter" if "jitter" in row["test_name"].lower()
                               else ("mutex" if "mutex" in row["test_name"].lower()
                               else "rtl"), (RTL_PASS_US, RTL_WARN_US))
        norm_data[i] = data[i] / w

    fig, ax = plt.subplots(figsize=(10, max(4, len(df) * 0.4)))
    im = ax.imshow(norm_data, aspect="auto", cmap="RdYlGn_r",
                   vmin=0.0, vmax=2.0)

    ax.set_xticks(np.arange(len(labels)))
    ax.set_xticklabels(labels)
    ax.set_yticks(np.arange(len(df)))
    ax.set_yticklabels(df["test_name"], fontsize=8)
    ax.set_title("Latency Heatmap (green = well within threshold, red = over WARN)")

    for i in range(len(df)):
        for j, m in enumerate(metrics):
            ax.text(j, i, f"{df[m].iloc[i]:.0f}",
                    ha="center", va="center", fontsize=7, color="black")

    plt.colorbar(im, ax=ax, label="Normalised to WARN threshold (1.0 = WARN)")
    plt.tight_layout()
    _save_or_show(fig, save_dir, "05_heatmap.png")


# ── Summary table ─────────────────────────────────────────────────────────────

def print_summary(df: pd.DataFrame):
    print("\n╔══════════════════════════════════════════════════════╗")
    print("║          Analysis Summary                            ║")
    print("╠══════════════════════════════════════════════════════╣")
    counts = {"PASS": 0, "WARN": 0, "FAIL": 0}
    for _, row in df.iterrows():
        verdict, _ = classify(row)
        counts[verdict] += 1
        icon = {"PASS": "✔", "WARN": "⚠", "FAIL": "✘"}[verdict]
        print(f"║  {icon} {row['test_name']:<38} p99={row['p99_us']:7.1f}µs {verdict}")
    print("╠══════════════════════════════════════════════════════╣")
    print(f"║  PASS: {counts['PASS']}  WARN: {counts['WARN']}  FAIL: {counts['FAIL']}")
    overall = "PASS" if counts["FAIL"] == 0 and counts["WARN"] == 0 else \
              ("WARN" if counts["FAIL"] == 0 else "FAIL")
    print(f"║  Overall: {overall}")
    print("╚══════════════════════════════════════════════════════╝\n")


# ── Helpers ───────────────────────────────────────────────────────────────────

def _save_or_show(fig, save_dir, filename):
    if save_dir:
        Path(save_dir).mkdir(parents=True, exist_ok=True)
        path = Path(save_dir) / filename
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"  Saved → {path}")
        plt.close(fig)
    else:
        plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Analyse bench_results.csv")
    parser.add_argument("csv",            help="Path to bench_results.csv")
    parser.add_argument("--save",         metavar="DIR", default=None,
                        help="Save plots to directory instead of showing")
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    print(f"[analyse] Loaded {len(df)} rows from {args.csv}")

    print_summary(df)
    plot_bar_chart(df, args.save)
    plot_cdf(df, args.save)
    plot_rtl_comparison(df, args.save)
    plot_jitter_comparison(df, args.save)
    plot_heatmap(df, args.save)

    print("[analyse] Done.")


if __name__ == "__main__":
    main()