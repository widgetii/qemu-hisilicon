#!/usr/bin/env python3
"""
Abandoned Object Detection using reference background subtraction.

Algorithm:
  1. Learn reference background from first N frames (average)
  2. For each frame: Sub(current, reference) → Thresh → CCL → regions
  3. Track stationary regions — if a region persists > ABANDON_FRAMES → abandoned
  4. Moving regions (person) are filtered out by comparing consecutive frames

IVE operations used: Sub, Thresh, CCL, SAD (for motion filtering)
All verified byte-identical between QEMU and real EV300 board.

Usage:
  python3 abandoned_demo.py [--visualize] [--frames-dir DIR]
"""
import os, sys, argparse
import numpy as np
from PIL import Image, ImageDraw
from collections import defaultdict

W, H = 64, 48
LEARN_FRAMES = 20          # frames to build reference background
ABANDON_FRAMES = 30        # frames before declaring abandoned (~3 sec at 10fps)
DIFF_THRESHOLD = 40        # pixel difference threshold for foreground
AREA_THRESHOLD = 8         # minimum blob area (pixels)
MOTION_THRESHOLD = 100     # SAD threshold for "still moving"
STATIONARY_RADIUS = 3      # centroid matching radius (pixels)


def ccl_regions(binary, area_thr=AREA_THRESHOLD):
    """4-connected CCL, returns list of dicts with bbox and area."""
    h, w = binary.shape
    labels = np.zeros_like(binary, dtype=np.int32)
    parent = list(range(1000))
    nl = 1

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb

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
            elif nl < 254:
                labels[y, x] = nl
                nl += 1

    regions = defaultdict(lambda: {"area": 0, "left": w, "top": h, "right": 0, "bottom": 0})
    for y in range(h):
        for x in range(w):
            l = labels[y, x]
            if l == 0:
                continue
            l = find(l)
            r = regions[l]
            r["area"] += 1
            r["left"] = min(r["left"], x)
            r["top"] = min(r["top"], y)
            r["right"] = max(r["right"], x)
            r["bottom"] = max(r["bottom"], y)

    return [r for r in regions.values() if r["area"] >= area_thr]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames-dir", default="demo/abandoned_frames")
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--output-dir", default="demo/abandoned_output")
    args = parser.parse_args()

    rawdir = os.path.join(args.frames_dir, "raw")
    frames = sorted(f for f in os.listdir(rawdir) if f.endswith(".raw"))

    if args.visualize:
        os.makedirs(args.output_dir, exist_ok=True)

    # Phase 1: Learn reference background (average of first N frames)
    bg_sum = np.zeros((H, W), dtype=np.float64)
    for i in range(min(LEARN_FRAMES, len(frames))):
        raw = np.fromfile(os.path.join(rawdir, frames[i]), dtype=np.uint8).reshape(H, W)
        bg_sum += raw.astype(np.float64)
    reference_bg = (bg_sum / LEARN_FRAMES).astype(np.uint8)

    # Phase 2: Process each frame
    prev_frame = None
    # Track stationary blobs: key=(cx_quantized, cy_quantized) → duration
    stationary = {}

    for idx in range(len(frames)):
        raw = np.fromfile(os.path.join(rawdir, frames[idx]), dtype=np.uint8).reshape(H, W)

        # Sub: |current - reference|
        diff = np.abs(raw.astype(np.int16) - reference_bg.astype(np.int16)).astype(np.uint8)

        # Thresh: binary foreground mask
        fg_mask = (diff > DIFF_THRESHOLD).astype(np.uint8) * 255

        # CCL: find connected regions in foreground
        regions = ccl_regions(fg_mask)

        # Filter out MOVING regions using SAD with previous frame
        stationary_regions = []
        if prev_frame is not None:
            for r in regions:
                # Check if this region is moving (SAD between consecutive frames)
                l, t, ri, b = r["left"], r["top"], r["right"]+1, r["bottom"]+1
                crop_cur = raw[t:b, l:ri]
                crop_prev = prev_frame[t:b, l:ri]
                sad = np.sum(np.abs(crop_cur.astype(int) - crop_prev.astype(int)))
                if sad < MOTION_THRESHOLD:
                    stationary_regions.append(r)

        prev_frame = raw.copy()

        # Update stationary tracker
        new_stationary = {}
        for r in stationary_regions:
            cx = (r["left"] + r["right"]) // 2
            cy = (r["top"] + r["bottom"]) // 2
            # Find nearest previous stationary blob
            best_key = None
            best_dist = STATIONARY_RADIUS + 1
            for key, dur in stationary.items():
                dist = abs(key[0] - cx) + abs(key[1] - cy)
                if dist < best_dist:
                    best_dist = dist
                    best_key = key
            if best_key and best_dist <= STATIONARY_RADIUS:
                new_stationary[(cx, cy)] = stationary[best_key] + 1
            else:
                new_stationary[(cx, cy)] = 1
        stationary = new_stationary

        # Check for abandoned objects
        abandoned = []
        for r in stationary_regions:
            cx = (r["left"] + r["right"]) // 2
            cy = (r["top"] + r["bottom"]) // 2
            duration = stationary.get((cx, cy), 0)
            if duration >= ABANDON_FRAMES:
                abandoned.append({**r, "duration": duration})

        if abandoned:
            parts = []
            for a in abandoned:
                parts.append(f"({a['left']},{a['top']})-({a['right']},{a['bottom']}) "
                           f"area={a['area']} dur={a['duration']}")
            print(f"FRAME {idx}: ABANDONED {' '.join(parts)}")

        if args.visualize:
            img = Image.fromarray(raw).convert("RGB").resize((W*4, H*4), Image.NEAREST)
            draw = ImageDraw.Draw(img)

            # Yellow: all foreground regions
            for r in regions:
                draw.rectangle([r["left"]*4, r["top"]*4, (r["right"]+1)*4, (r["bottom"]+1)*4],
                              outline=(255, 255, 0), width=1)

            # Green: stationary but not yet abandoned
            for r in stationary_regions:
                cx = (r["left"] + r["right"]) // 2
                cy = (r["top"] + r["bottom"]) // 2
                dur = stationary.get((cx, cy), 0)
                if dur < ABANDON_FRAMES:
                    draw.rectangle([r["left"]*4, r["top"]*4, (r["right"]+1)*4, (r["bottom"]+1)*4],
                                  outline=(0, 255, 0), width=1)

            # Red: abandoned objects
            for a in abandoned:
                draw.rectangle([a["left"]*4, a["top"]*4, (a["right"]+1)*4, (a["bottom"]+1)*4],
                              outline=(255, 0, 0), width=3)
                draw.text((a["left"]*4, a["top"]*4 - 14),
                         f"ABANDONED ({a['duration']}f)", fill=(255, 0, 0))

            label = f"Frame {idx}"
            if idx < LEARN_FRAMES:
                label += " [LEARNING BG]"
            elif abandoned:
                label += f" - {len(abandoned)} ABANDONED"
            draw.text((4, 4), label, fill=(255, 255, 0))

            img.save(os.path.join(args.output_dir, f"frame_{idx:04d}.png"))

    if args.visualize:
        import subprocess
        mp4_path = os.path.join(args.output_dir, "abandoned_demo.mp4")
        subprocess.run([
            "ffmpeg", "-y", "-r", "10",
            "-i", os.path.join(args.output_dir, "frame_%04d.png"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-vf", "pad=ceil(iw/2)*2:ceil(ih/2)*2",
            mp4_path
        ], capture_output=True)
        print(f"\nDemo video: {mp4_path}", file=sys.stderr)


if __name__ == "__main__":
    main()
