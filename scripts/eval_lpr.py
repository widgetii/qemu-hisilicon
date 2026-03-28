#!/usr/bin/env python3
"""Evaluate IVE License Plate Region detection against CCPD ground truth.

CCPD embeds annotations in filenames:
  025-95_113-154&383_386&473-...-0_0_22_27_27_33_16-37-15.jpg
  Field 3 (0-indexed): "154&383_386&473" = left&top_right&bottom

Pipeline:
  1. Sample N images from CCPD
  2. Convert each to Y4M (grayscale, MD_SIZE resolution)
  3. Run IVE LPR on real board
  4. Match detections against GT plate bbox (IoU threshold)
  5. Compute P/R/F1

Usage:
    python3 scripts/eval_lpr.py [--max-images 200] [--iou-thr 0.3]
                                [--diff-thr 40] [--area-thr 4]
"""
import json, os, sys, re, subprocess, random
from collections import defaultdict

CCPD_DIR = "/mnt/data/datasets/ccpd"
RESULTS_DIR = os.path.join(CCPD_DIR, "results")
NFS_DIR = "/mnt/noc/ive-test"
BOARD = "root@IVG85HG50PYA-S.dlab.doty.ru"
BOARD_DIR = "/utils/ive-test"


def md_size(x):
    return (x // 2) & ~0xF


def parse_ccpd_filename(fname):
    """Parse CCPD filename to extract plate bounding box.
    Returns (left, top, right, bottom) or None.
    """
    base = os.path.splitext(fname)[0]
    fields = base.split("-")
    if len(fields) < 4:
        return None
    # Field 2 (0-indexed): bbox "154&383_386&473"
    bbox_str = fields[2]
    parts = bbox_str.split("_")
    if len(parts) != 2:
        return None
    lt = parts[0].split("&")
    rb = parts[1].split("&")
    if len(lt) != 2 or len(rb) != 2:
        return None
    try:
        left, top = int(lt[0]), int(lt[1])
        right, bottom = int(rb[0]), int(rb[1])
        return (left, top, right, bottom)
    except ValueError:
        return None


def iou(box1, box2):
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    inter = max(0, x2 - x1) * max(0, y2 - y1)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union = area1 + area2 - inter
    return inter / union if union > 0 else 0


def convert_image_to_y4m(img_path, y4m_path, proc_w, proc_h):
    """Convert a single CCPD image to a 1-frame Y4M."""
    # Duplicate image to 10 frames (IVE needs at least 2 frames for SAD)
    cmd = [
        "ffmpeg", "-y", "-loop", "1", "-i", img_path,
        "-t", "1.0", "-r", "10",
        "-vf", f"scale={proc_w}:{proc_h},format=gray",
        "-pix_fmt", "gray", "-f", "yuv4mpegpipe", y4m_path
    ]
    r = subprocess.run(cmd, capture_output=True, timeout=30)
    return r.returncode == 0


def run_ive_lpr(y4m_path, extra_args=""):
    """Run IVE LPR on a Y4M file. Returns list of (x1,y1,x2,y2,area,ratio) detections."""
    subprocess.run(["cp", y4m_path, os.path.join(NFS_DIR, "lpr_eval.y4m")],
                   capture_output=True, timeout=30)
    cmd = f"timeout 30 {BOARD_DIR}/test-ive-video-mpi {BOARD_DIR}/lpr_eval.y4m lpr {extra_args}"
    r = subprocess.run(["ssh", BOARD, cmd], capture_output=True, text=True, timeout=60)

    detections = []
    for line in r.stdout.split("\n"):
        m = re.match(r'FRAME \d+: PLATE \((\d+),(\d+)\)-\((\d+),(\d+)\) area=(\d+) ratio=(\S+)', line)
        if m:
            x1, y1, x2, y2 = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
            area = int(m.group(5))
            ratio = float(m.group(6))
            detections.append((x1, y1, x2, y2, area, ratio))
    return detections


def main():
    max_images = 200
    iou_thr = 0.3
    extra_args = ""

    for arg in sys.argv[1:]:
        if arg.startswith("--max-images="):
            max_images = int(arg.split("=")[1])
        elif arg.startswith("--iou-thr="):
            iou_thr = float(arg.split("=")[1])
        elif arg.startswith("--diff-thr=") or arg.startswith("--area-thr=") or arg.startswith("--sad-thr="):
            extra_args += f" {arg}"

    os.makedirs(RESULTS_DIR, exist_ok=True)

    # Find CCPD images
    ccpd_base = os.path.join(CCPD_DIR, "CCPD2019", "ccpd_base")
    if not os.path.isdir(ccpd_base):
        # Try alternative paths
        for subdir in ["ccpd_base", "CCPD2019/ccpd_base", "ccpd_base"]:
            test = os.path.join(CCPD_DIR, subdir)
            if os.path.isdir(test):
                ccpd_base = test
                break
        else:
            print(f"CCPD base directory not found. Checked: {CCPD_DIR}")
            print("Run: tar xf CCPD2019.tar.xz in /mnt/data/datasets/ccpd/")
            sys.exit(1)

    images = [f for f in os.listdir(ccpd_base) if f.endswith(".jpg")]
    print(f"CCPD images found: {len(images)}")

    # Sample
    random.seed(42)
    if len(images) > max_images:
        images = random.sample(images, max_images)
    print(f"Evaluating {len(images)} images (iou_thr={iou_thr})")

    # Get resolution from first image
    first_img = os.path.join(ccpd_base, images[0])
    r = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height", "-of", "csv=p=0", first_img],
        capture_output=True, text=True, timeout=10
    )
    orig_w, orig_h = [int(x) for x in r.stdout.strip().split(",")]
    proc_w, proc_h = md_size(orig_w), md_size(orig_h)
    sx = orig_w / proc_w
    sy = orig_h / proc_h
    print(f"Resolution: {orig_w}x{orig_h} → {proc_w}x{proc_h} (scale {sx:.2f}x{sy:.2f})")

    # Stop majestic
    subprocess.run(["ssh", BOARD, "killall majestic 2>/dev/null; sleep 1"],
                   capture_output=True, timeout=10)

    tp = fp = fn = 0
    ious_list = []
    per_image = []

    for idx, fname in enumerate(images):
        gt_bbox = parse_ccpd_filename(fname)
        if gt_bbox is None:
            continue

        img_path = os.path.join(ccpd_base, fname)
        y4m_path = os.path.join(RESULTS_DIR, "eval_tmp.y4m")

        # Convert to Y4M
        if not convert_image_to_y4m(img_path, y4m_path, proc_w, proc_h):
            continue

        # Run IVE
        detections = run_ive_lpr(y4m_path, extra_args)

        # Deduplicate detections (same region across multiple frames)
        unique_dets = {}
        for det in detections:
            key = (det[0]//8, det[1]//8, det[2]//8, det[3]//8)  # quantize
            if key not in unique_dets or det[4] > unique_dets[key][4]:
                unique_dets[key] = det
        detections = list(unique_dets.values())

        # Scale IVE detections to original coordinates
        scaled_dets = []
        for x1, y1, x2, y2, area, ratio in detections:
            scaled_dets.append((int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy), area, ratio))

        # Match: find best IoU detection vs GT
        best_iou = 0
        best_det = None
        for det in scaled_dets:
            val = iou(gt_bbox, det[:4])
            if val > best_iou:
                best_iou = val
                best_det = det

        if best_iou >= iou_thr:
            tp += 1
        else:
            fn += 1
        # FP: detections that don't match GT (all except best match if it matched)
        fp += max(0, len(scaled_dets) - (1 if best_iou >= iou_thr else 0))

        if best_iou > 0:
            ious_list.append(best_iou)

        result = {
            "image": fname,
            "gt_bbox": gt_bbox,
            "num_detections": len(scaled_dets),
            "best_iou": round(best_iou, 3),
            "matched": best_iou >= iou_thr
        }
        per_image.append(result)

        if (idx + 1) % 50 == 0 or idx < 5:
            p = tp / (tp + fp) if (tp + fp) > 0 else 0
            r = tp / (tp + fn) if (tp + fn) > 0 else 0
            print(f"  [{idx+1}/{len(images)}] TP={tp} FP={fp} FN={fn} "
                  f"P={p:.3f} R={r:.3f} best_iou={best_iou:.3f}")

    # Restart majestic
    subprocess.run(["ssh", BOARD, "majestic &"], capture_output=True, timeout=10)

    # Final metrics
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0

    print(f"\n=== LPR Evaluation ({len(per_image)} images, IoU≥{iou_thr}) ===")
    print(f"  Precision: {precision:.3f}")
    print(f"  Recall:    {recall:.3f}")
    print(f"  F1:        {f1:.3f}")
    print(f"  TP={tp} FP={fp} FN={fn}")
    if ious_list:
        print(f"  Mean IoU (matched): {sum(ious_list)/len(ious_list):.3f}")
        print(f"  Median IoU: {sorted(ious_list)[len(ious_list)//2]:.3f}")

    # Save
    metrics = {
        "precision": round(precision, 3),
        "recall": round(recall, 3),
        "f1": round(f1, 3),
        "tp": tp, "fp": fp, "fn": fn,
        "iou_threshold": iou_thr,
        "num_images": len(per_image),
        "extra_args": extra_args.strip(),
        "mean_iou": round(sum(ious_list)/len(ious_list), 3) if ious_list else 0,
        "per_image": per_image
    }
    out_path = os.path.join(RESULTS_DIR, "lpr_metrics.json")
    with open(out_path, "w") as f:
        json.dump(metrics, f, indent=2)
    print(f"  Saved: {out_path}")


if __name__ == "__main__":
    main()
