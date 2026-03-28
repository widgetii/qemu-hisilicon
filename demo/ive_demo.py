#!/usr/bin/env python3
"""
Host-side IVE motion detection reference implementation.

Implements the exact same SAD+CCL algorithm as QEMU's hisi-ive.c.
Outputs bounding box coordinates and optionally draws them on frames.

Usage:
  python3 ive_demo.py                    # print bboxes to stdout
  python3 ive_demo.py --visualize        # also draw boxes + make MP4
  python3 ive_demo.py --frames-dir DIR   # custom frames directory
"""
import os, sys, json, argparse, subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H = 320, 240
BLOCK = 4
BW, BH = W // BLOCK, H // BLOCK
SAD_THRESHOLD = 200
CCL_AREA_THR = 16


def sad_4x4(frame1, frame2, threshold=SAD_THRESHOLD):
    """
    4×4 block Sum of Absolute Differences with binary threshold.
    Exact match of hisi-ive.c ive_op_sad().
    """
    out = np.zeros((BH, BW), dtype=np.uint8)
    for by in range(BH):
        for bx in range(BW):
            y0, x0 = by * BLOCK, bx * BLOCK
            b1 = frame1[y0:y0+BLOCK, x0:x0+BLOCK].astype(np.int32)
            b2 = frame2[y0:y0+BLOCK, x0:x0+BLOCK].astype(np.int32)
            sad = int(np.sum(np.abs(b1 - b2)))
            out[by, bx] = 255 if sad > threshold else 0
    return out


def ccl_4connected(binary, area_thr=CCL_AREA_THR):
    """
    4-connected Component Labeling with union-find.
    Exact match of hisi-ive.c ive_op_ccl().
    Returns list of dicts: {area, left, top, right, bottom}
    """
    h, w = binary.shape
    labels = np.zeros((h, w), dtype=np.int32)
    parent = list(range(1000))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

    next_label = 1

    # Pass 1: label
    for y in range(h):
        for x in range(w):
            if binary[y, x] == 0:
                continue
            left = labels[y, x-1] if x > 0 else 0
            up = labels[y-1, x] if y > 0 else 0

            if left and up:
                labels[y, x] = left
                union(left, up)
            elif left:
                labels[y, x] = left
            elif up:
                labels[y, x] = up
            else:
                if next_label < 254:
                    labels[y, x] = next_label
                    next_label += 1

    # Pass 2: bounding boxes
    areas = {}
    bboxes = {}
    for y in range(h):
        for x in range(w):
            l = labels[y, x]
            if l == 0:
                continue
            l = find(l)
            if l not in areas:
                areas[l] = 0
                bboxes[l] = [x, y, x, y]
            areas[l] += 1
            bb = bboxes[l]
            if x < bb[0]: bb[0] = x
            if y < bb[1]: bb[1] = y
            if x > bb[2]: bb[2] = x
            if y > bb[3]: bb[3] = y

    # Filter by area threshold
    regions = []
    for l in sorted(areas.keys()):
        if areas[l] >= area_thr:
            bb = bboxes[l]
            regions.append({
                "area": areas[l],
                "left": bb[0], "top": bb[1],
                "right": bb[2], "bottom": bb[3]
            })

    return regions


def scale_bbox(region, src_w, src_h, dst_w, dst_h):
    """Scale bbox from block coordinates to pixel coordinates."""
    return {
        "left": region["left"] * BLOCK,
        "top": region["top"] * BLOCK,
        "right": (region["right"] + 1) * BLOCK,
        "bottom": (region["bottom"] + 1) * BLOCK,
        "area": region["area"]
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames-dir", default="demo/frames")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--output-dir", default="demo/output")
    parser.add_argument("--small", action="store_true",
                        help="Use 64×48 frames (for QEMU comparison)")
    args = parser.parse_args()

    if args.small:
        global W, H, BW, BH, SAD_THRESHOLD, CCL_AREA_THR
        W, H = 64, 64
        BW, BH = W // BLOCK, H // BLOCK
        SAD_THRESHOLD = 50   # lower for small frames (less pixel diff)
        CCL_AREA_THR = 4     # smaller regions at lower resolution

    raw_dir = os.path.join(args.frames_dir, "small" if args.small else "raw")
    frames = sorted(f for f in os.listdir(raw_dir) if f.endswith(".raw"))

    if args.visualize:
        os.makedirs(args.output_dir, exist_ok=True)

    results = []
    prev_frame = None

    for idx, fname in enumerate(frames):
        raw = np.fromfile(os.path.join(raw_dir, fname), dtype=np.uint8).reshape(H, W)

        if prev_frame is None:
            prev_frame = raw
            results.append({"frame": idx, "regions": []})
            if args.visualize:
                img = Image.open(os.path.join(args.frames_dir, f"frame_{idx:04d}.png")).convert("RGB")
                draw = ImageDraw.Draw(img)
                draw.text((5, 5), f"Frame {idx}: no motion", fill=(255, 255, 0))
                img.save(os.path.join(args.output_dir, f"frame_{idx:04d}.png"))
            continue

        # SAD
        motion_mask = sad_4x4(raw, prev_frame)

        # CCL
        regions = ccl_4connected(motion_mask)

        # Scale to pixel coordinates
        pixel_regions = [scale_bbox(r, BW, BH, W, H) for r in regions]

        # Output
        entry = {"frame": idx, "regions": pixel_regions}
        results.append(entry)

        # Print to stdout (same format as ARM binary)
        if pixel_regions:
            parts = []
            for r in pixel_regions:
                parts.append(f"({r['left']},{r['top']})-({r['right']},{r['bottom']}) area={r['area']}")
            print(f"FRAME {idx}: {' '.join(parts)}")

        # Visualize
        if args.visualize:
            img = Image.open(os.path.join(args.frames_dir, f"frame_{idx:04d}.png")).convert("RGB")
            draw = ImageDraw.Draw(img)
            for r in pixel_regions:
                draw.rectangle([r["left"], r["top"], r["right"], r["bottom"]],
                              outline=(255, 0, 0), width=2)
            label = f"Frame {idx}: {len(pixel_regions)} region(s)"
            draw.text((5, 5), label, fill=(255, 255, 0))
            img.save(os.path.join(args.output_dir, f"frame_{idx:04d}.png"))

        prev_frame = raw

    # Write JSON
    json_path = os.path.join(args.output_dir if args.visualize else "demo",
                             "host_bboxes.json")
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    with open(json_path, "w") as f:
        json.dump(results, f, indent=2)

    # Make MP4 if visualizing
    if args.visualize:
        mp4_path = os.path.join(args.output_dir, "demo_output.mp4")
        subprocess.run([
            "ffmpeg", "-y", "-r", "10",
            "-i", os.path.join(args.output_dir, "frame_%04d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            mp4_path
        ], capture_output=True)
        print(f"\nDemo video: {mp4_path}", file=sys.stderr)

    print(f"\nBboxes: {json_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
