#!/usr/bin/env python3
"""Sweep IVE thresholds and compute P/R/F1 at each point.

Re-runs test-ive-video-mpi on all eval clips with different threshold
combinations, then computes metrics for each. Outputs a sweep results
table and identifies the best operating point.

Usage:
    python3 scripts/eval_threshold_sweep.py [--mode abandoned|md] [--quick]
"""
import json, os, sys, subprocess, re
from collections import defaultdict

DATASET = "/mnt/data/datasets/meva"
Y4M_DIR = os.path.join(DATASET, "y4m")
INDEX_DIR = os.path.join(DATASET, "index")
NFS_DIR = "/mnt/noc/ive-test"
BOARD = "root@IVG85HG50PYA-S.dlab.doty.ru"
BOARD_DIR = "/utils/ive-test"


def parse_detections(text):
    count = 0
    for line in text.split("\n"):
        if line.startswith("FRAME"):
            count += 1
    return count


def run_ive_on_clip(y4m_path, mode, extra_args):
    """Run IVE on a single clip, return detection count."""
    # Copy to NFS
    subprocess.run(["cp", y4m_path, os.path.join(NFS_DIR, "sweep_clip.y4m")],
                   capture_output=True, timeout=30)
    # Run on board
    cmd = (f"timeout 300 {BOARD_DIR}/test-ive-video-mpi "
           f"{BOARD_DIR}/sweep_clip.y4m {mode} {extra_args}")
    result = subprocess.run(
        ["ssh", BOARD, cmd],
        capture_output=True, text=True, timeout=360
    )
    return parse_detections(result.stdout), result.stdout


def eval_abandoned_sweep(entries, y4m_clips, extra_args):
    """Run abandoned detection with given args and compute metrics."""
    tp = fp = fn = tn = 0
    for clip, label in y4m_clips:
        y4m = os.path.join(Y4M_DIR, f"{clip}.abandoned.y4m")
        if not os.path.exists(y4m):
            continue
        n_det, _ = run_ive_on_clip(y4m, "abandoned", extra_args)
        has_det = n_det > 0

        if label == "positive":
            if has_det: tp += 1
            else: fn += 1
        else:
            if has_det: fp += 1
            else: tn += 1

    p = tp / (tp + fp) if (tp + fp) > 0 else 0
    r = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1 = 2 * p * r / (p + r) if (p + r) > 0 else 0
    return {"tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "precision": round(p, 3), "recall": round(r, 3), "f1": round(f1, 3)}


def main():
    mode = "abandoned"
    quick = "--quick" in sys.argv

    # Load eval index
    entries = json.load(open(os.path.join(INDEX_DIR, "eval_abandoned.json")))

    # Build clip list with labels (deduplicated by clip name)
    clips_seen = set()
    y4m_clips = []
    for e in entries:
        clip = e["clip"]
        if clip in clips_seen:
            continue
        clips_seen.add(clip)
        y4m = os.path.join(Y4M_DIR, f"{clip}.abandoned.y4m")
        if os.path.exists(y4m):
            y4m_clips.append((clip, e["label"]))

    pos = sum(1 for _, l in y4m_clips if l == "positive")
    neg = sum(1 for _, l in y4m_clips if l != "positive")
    print(f"Eval clips: {len(y4m_clips)} ({pos} positive, {neg} negative)")

    if quick:
        # Subsample for faster sweep
        y4m_clips = y4m_clips[:20]
        print(f"Quick mode: using {len(y4m_clips)} clips")

    # Stop majestic
    subprocess.run(["ssh", BOARD, "killall majestic 2>/dev/null; sleep 1"],
                   capture_output=True, timeout=10)

    # Define sweep parameters
    sweep_params = [
        # (diff_thr, sad_thr, area_thr)
        (30, 200, 4),   # baseline lower diff
        (40, 200, 4),   # current default
        (50, 200, 4),   # higher diff
        (60, 200, 4),   # aggressive diff
        (40, 200, 8),   # larger min area
        (40, 200, 16),  # much larger min area
        (40, 200, 32),  # very large min area
        (50, 200, 8),   # combined
        (50, 200, 16),  # combined aggressive
        (60, 200, 16),  # most aggressive
        (40, 100, 4),   # lower SAD (more stationary = more detections)
        (40, 300, 4),   # higher SAD (stricter stationarity)
        (40, 300, 16),  # high SAD + large area
    ]

    results = []
    for i, (diff, sad, area) in enumerate(sweep_params):
        args = f"--diff-thr={diff} --sad-thr={sad} --area-thr={area}"
        print(f"\n[{i+1}/{len(sweep_params)}] {args}")
        metrics = eval_abandoned_sweep(entries, y4m_clips, args)
        metrics["diff_thr"] = diff
        metrics["sad_thr"] = sad
        metrics["area_thr"] = area
        results.append(metrics)
        print(f"  P={metrics['precision']:.3f} R={metrics['recall']:.3f} "
              f"F1={metrics['f1']:.3f} TP={metrics['tp']} FP={metrics['fp']} "
              f"FN={metrics['fn']} TN={metrics['tn']}")

    # Restart majestic
    subprocess.run(["ssh", BOARD, "majestic &"],
                   capture_output=True, timeout=10)

    # Print summary table
    print("\n" + "=" * 90)
    print(f"{'diff':>4s} {'sad':>4s} {'area':>4s}  {'P':>6s} {'R':>6s} {'F1':>6s}  "
          f"{'TP':>3s} {'FP':>3s} {'FN':>3s} {'TN':>3s}")
    print("-" * 90)
    for r in sorted(results, key=lambda x: -x["f1"]):
        marker = " <<<" if r["f1"] == max(x["f1"] for x in results) else ""
        print(f"{r['diff_thr']:4d} {r['sad_thr']:4d} {r['area_thr']:4d}  "
              f"{r['precision']:6.3f} {r['recall']:6.3f} {r['f1']:6.3f}  "
              f"{r['tp']:3d} {r['fp']:3d} {r['fn']:3d} {r['tn']:3d}{marker}")

    # Save results
    out_path = os.path.join(DATASET, "results", "threshold_sweep.json")
    with open(out_path, "w") as f:
        json.dump(results, f, indent=2)
    print(f"\nSaved: {out_path}")


if __name__ == "__main__":
    main()
