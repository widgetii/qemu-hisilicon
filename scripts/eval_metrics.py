#!/usr/bin/env python3
"""Compute P/R/F1 metrics for IVE detection against MEVA/VIRAT ground truth.

Usage:
    python3 scripts/eval_metrics.py [--mode abandoned|md|both]
"""
import json, os, sys, re
from collections import defaultdict

DATASET = "/mnt/data/datasets/meva"
INDEX_DIR = os.path.join(DATASET, "index")
RESULTS_DIR = os.path.join(DATASET, "results")


def parse_ive_output(path):
    """Parse IVE detection output file.
    Returns list of (frame, x1, y1, x2, y2, area, dur) tuples.
    """
    detections = []
    if not os.path.exists(path):
        return detections
    with open(path) as f:
        for line in f:
            m = re.match(r'FRAME (\d+): (?:ABANDONED )?\((\d+),(\d+)\)-\((\d+),(\d+)\) area=(\d+)(?: dur=(\d+))?', line.strip())
            if not m:
                continue
            frame = int(m.group(1))
            x1, y1 = int(m.group(2)), int(m.group(3))
            x2, y2 = int(m.group(4)), int(m.group(5))
            area = int(m.group(6))
            dur = int(m.group(7)) if m.group(7) else 0
            is_abandoned = "ABANDONED" in line
            detections.append({
                "frame": frame, "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                "area": area, "dur": dur, "is_abandoned": is_abandoned
            })
    return detections


def parse_benchmark(path):
    """Parse IVE benchmark output (stderr)."""
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        text = f.read()
    bench = {}
    m = re.search(r'IVE hardware:\s+(\d+) ms \((\S+) ms/frame\)', text)
    if m:
        bench["ive_total_ms"] = int(m.group(1))
        bench["ive_ms_per_frame"] = float(m.group(2))
    m = re.search(r'I/O.*?:\s+(\d+) ms \((\S+) ms/frame\)', text)
    if m:
        bench["io_ms_per_frame"] = float(m.group(2))
    m = re.search(r'(\d+) frames,', text)
    if m:
        bench["num_frames"] = int(m.group(1))
    return bench


def iou(box1, box2):
    """Compute IoU between two boxes (x1,y1,x2,y2)."""
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0


def eval_abandoned(entries):
    """Evaluate abandoned object detection."""
    tp = fp = fn = tn = 0
    latencies = []
    ious = []
    per_clip = []

    for entry in entries:
        clip = entry["clip"]
        label = entry["label"]
        gt_frames = entry.get("frames", [])
        gt_tracks = entry.get("tracks", {})

        result_file = os.path.join(RESULTS_DIR, f"{clip}.abandoned.txt")
        detections = parse_ive_output(result_file)
        abandoned_dets = [d for d in detections if d["is_abandoned"]]

        bench_file = os.path.join(RESULTS_DIR, f"{clip}.abandoned.bench")
        bench = parse_benchmark(bench_file)

        clip_result = {
            "clip": clip, "label": label, "gt_frames": gt_frames,
            "num_detections": len(abandoned_dets),
            "benchmark": bench
        }

        if label == "positive":
            if abandoned_dets:
                tp += 1
                clip_result["detected"] = True
                clip_result["first_det_frame"] = min(d["frame"] for d in abandoned_dets)
            else:
                fn += 1
                clip_result["detected"] = False
        elif label == "negative":
            if abandoned_dets:
                fp += 1
                clip_result["false_alarm"] = True
            else:
                tn += 1
                clip_result["false_alarm"] = False
        elif label == "true_negative":
            if abandoned_dets:
                fp += 1
                clip_result["false_alarm"] = True
            else:
                tn += 1
                clip_result["false_alarm"] = False

        per_clip.append(clip_result)

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    return {
        "event_level": {
            "tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "precision": round(precision, 3),
            "recall": round(recall, 3),
            "f1": round(f1, 3),
            "total_positive": tp + fn,
            "total_negative": fp + tn
        },
        "per_clip": per_clip
    }


def eval_motion(entries):
    """Evaluate motion detection."""
    tp = fp = fn = tn = 0
    per_clip = []

    for entry in entries:
        clip = entry["clip"]
        label = entry["label"]
        gt_frames = entry.get("frames", [])

        result_file = os.path.join(RESULTS_DIR, f"{clip}.md.txt")
        detections = parse_ive_output(result_file)

        bench_file = os.path.join(RESULTS_DIR, f"{clip}.md.bench")
        bench = parse_benchmark(bench_file)

        clip_result = {
            "clip": clip, "label": label,
            "num_detections": len(detections),
            "benchmark": bench
        }

        if label == "positive":
            # Event-level: any motion detection in this clip = TP
            if detections:
                tp += 1
                clip_result["detected"] = True
            else:
                fn += 1
                clip_result["detected"] = False
        elif label == "true_negative":
            if detections:
                fp += 1
                clip_result["false_alarm"] = True
            else:
                tn += 1
                clip_result["false_alarm"] = False

        per_clip.append(clip_result)

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    return {
        "event_level": {
            "tp": tp, "fp": fp, "fn": fn, "tn": tn,
            "precision": round(precision, 3),
            "recall": round(recall, 3),
            "f1": round(f1, 3),
            "total_positive": tp + fn,
            "total_negative": fp + tn
        },
        "per_clip": per_clip
    }


def main():
    mode = "both"
    for arg in sys.argv[1:]:
        if arg.startswith("--mode="):
            mode = arg.split("=")[1]

    os.makedirs(RESULTS_DIR, exist_ok=True)
    metrics = {}

    if mode in ("abandoned", "both"):
        idx_path = os.path.join(INDEX_DIR, "eval_abandoned.json")
        if os.path.exists(idx_path):
            entries = json.load(open(idx_path))
            # Filter to clips that have results
            with_results = [e for e in entries
                           if os.path.exists(os.path.join(RESULTS_DIR, f"{e['clip']}.abandoned.txt"))]
            print(f"Abandoned: {len(with_results)}/{len(entries)} clips have results")
            if with_results:
                metrics["abandoned"] = eval_abandoned(with_results)
                ev = metrics["abandoned"]["event_level"]
                print(f"  P={ev['precision']:.3f} R={ev['recall']:.3f} F1={ev['f1']:.3f}")
                print(f"  TP={ev['tp']} FP={ev['fp']} FN={ev['fn']} TN={ev['tn']}")

    if mode in ("md", "both"):
        idx_path = os.path.join(INDEX_DIR, "eval_motion.json")
        if os.path.exists(idx_path):
            entries = json.load(open(idx_path))
            with_results = [e for e in entries
                           if os.path.exists(os.path.join(RESULTS_DIR, f"{e['clip']}.md.txt"))]
            print(f"\nMotion: {len(with_results)}/{len(entries)} clips have results")
            if with_results:
                metrics["motion"] = eval_motion(with_results)
                ev = metrics["motion"]["event_level"]
                print(f"  P={ev['precision']:.3f} R={ev['recall']:.3f} F1={ev['f1']:.3f}")
                print(f"  TP={ev['tp']} FP={ev['fp']} FN={ev['fn']} TN={ev['tn']}")

    # Save metrics
    output_path = os.path.join(RESULTS_DIR, "metrics.json")
    with open(output_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"\nMetrics saved: {output_path}")


if __name__ == "__main__":
    main()
