#!/usr/bin/env python3
"""
Generate a synthetic CCTV scene for motion detection demo.

Output: 150 grayscale frames (320×240) showing a static room
with a "person" blob walking left→right and a "cat" blob
moving right→left. Frames saved as raw Y8 and PNG.

Usage: python3 generate_scene.py [output_dir]
"""
import os, sys
import numpy as np
from PIL import Image, ImageDraw

W, H = 320, 240
SMALL_W, SMALL_H = 64, 64
NFRAMES = 150

def draw_background(img):
    """Static CCTV room: floor, walls, furniture."""
    draw = ImageDraw.Draw(img)
    # Floor (slightly lighter gray)
    draw.rectangle([0, 160, W, H], fill=140)
    # Wall
    draw.rectangle([0, 0, W, 160], fill=120)
    # Window
    draw.rectangle([130, 20, 190, 80], fill=170)
    draw.rectangle([135, 25, 185, 75], fill=190)
    # Table
    draw.rectangle([20, 170, 90, 200], fill=95)
    draw.rectangle([25, 200, 35, H], fill=90)
    draw.rectangle([75, 200, 85, H], fill=90)
    # Shelf on wall
    draw.rectangle([230, 50, 300, 60], fill=100)
    draw.rectangle([240, 30, 260, 50], fill=105)
    draw.rectangle([270, 35, 290, 50], fill=108)
    # Door frame
    draw.rectangle([0, 40, 15, 160], fill=110)
    draw.rectangle([0, 40, 50, 50], fill=110)

def draw_person(img, x, y):
    """Simple person silhouette (dark blob, ~30×60 pixels)."""
    draw = ImageDraw.Draw(img)
    # Head
    cx = x + 15
    draw.ellipse([cx-8, y, cx+8, y+16], fill=60)
    # Body
    draw.rectangle([cx-12, y+16, cx+12, y+45], fill=55)
    # Legs
    draw.rectangle([cx-12, y+45, cx-3, y+60], fill=50)
    draw.rectangle([cx+3, y+45, cx+12, y+60], fill=50)

def draw_cat(img, x, y):
    """Cat blob (~40×25 pixels, facing left, high contrast for detection)."""
    draw = ImageDraw.Draw(img)
    # Body
    draw.ellipse([x+10, y+6, x+40, y+22], fill=45)
    # Head (on the left side — cat faces left since it walks left)
    draw.ellipse([x, y+2, x+16, y+18], fill=40)
    # Ears
    draw.polygon([(x+4, y+2), (x+6, y-4), (x+8, y+2)], fill=40)
    draw.polygon([(x+9, y+2), (x+11, y-4), (x+13, y+2)], fill=40)
    # Tail (right side, curving up)
    draw.arc([x+32, y, x+52, y+14], 220, 360, fill=45, width=3)
    # Legs
    draw.rectangle([x+14, y+20, x+20, y+25], fill=42)
    draw.rectangle([x+30, y+20, x+36, y+25], fill=42)

def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "demo/frames"
    os.makedirs(outdir, exist_ok=True)
    rawdir = os.path.join(outdir, "raw")
    os.makedirs(rawdir, exist_ok=True)

    # Pre-render background (static across all frames)
    bg = Image.new("L", (W, H), 128)
    draw_background(bg)
    bg_arr = np.array(bg)

    print(f"Generating {NFRAMES} frames ({W}×{H}) in {outdir}/")

    for i in range(NFRAMES):
        frame = bg.copy()

        # Person walks left→right (enters at frame 10, exits at frame 140)
        if 10 <= i <= 140:
            px = int(-30 + (i - 10) * (W + 30) / 130)
            py = 100  # stands on floor
            if 0 <= px < W:
                draw_person(frame, px, py)

        # Cat walks right→left (enters at frame 20, exits at frame 100)
        # Faster movement (~4 px/frame) for reliable detection
        if 20 <= i <= 100:
            cx = int(W + 10 - (i - 20) * (W + 50) / 80)
            cy = 185  # on floor level
            if -10 <= cx < W:
                draw_cat(frame, cx, cy)

        # Save as raw Y8
        raw_path = os.path.join(rawdir, f"frame_{i:04d}.raw")
        np.array(frame, dtype=np.uint8).tofile(raw_path)

        # Save as PNG for visualization
        png_path = os.path.join(outdir, f"frame_{i:04d}.png")
        frame.save(png_path)

    # Concatenate all raw frames into one binary file
    all_frames = os.path.join(outdir, "frames.bin")
    with open(all_frames, "wb") as f:
        for i in range(NFRAMES):
            raw = np.fromfile(os.path.join(rawdir, f"frame_{i:04d}.raw"), dtype=np.uint8)
            f.write(raw.tobytes())

    # Create small (64×48) versions for QEMU test (must fit in one 4KB page)
    # Re-render at native small resolution (not downscaled) so objects are sharp
    smalldir = os.path.join(outdir, "small")
    os.makedirs(smalldir, exist_ok=True)
    small_bin = os.path.join(outdir, "frames_small.bin")
    bg_small = Image.new("L", (SMALL_W, SMALL_H), 128)
    # Simple small background
    draw = ImageDraw.Draw(bg_small)
    draw.rectangle([0, 32, SMALL_W, SMALL_H], fill=140)  # floor
    draw.rectangle([4, 34, 16, 40], fill=95)              # table

    with open(small_bin, "wb") as f:
        for i in range(NFRAMES):
            frame = bg_small.copy()
            draw = ImageDraw.Draw(frame)
            # Person: 8×16 blob, moves 2px/frame (visible motion at this scale)
            if 5 <= i <= 35:
                px = int(-8 + (i - 5) * 2.5)
                if 0 <= px < SMALL_W:
                    draw.rectangle([px, 16, px+8, 32], fill=50)
                    draw.ellipse([px+1, 12, px+7, 18], fill=55)
            # Cat: 6×4 blob moves right→left
            if 10 <= i <= 30:
                cx = int(SMALL_W - (i - 10) * 3)
                if 0 <= cx < SMALL_W:
                    draw.ellipse([cx, 36, cx+6, 40], fill=70)
            arr = np.array(frame, dtype=np.uint8)
            arr.tofile(os.path.join(smalldir, f"frame_{i:04d}.raw"))
            f.write(arr.tobytes())

    print(f"Done: {NFRAMES} frames")
    print(f"  Full ({W}×{H}): {outdir}/frame_NNNN.png, {all_frames}")
    print(f"  Small ({SMALL_W}×{SMALL_H}): {smalldir}/, {small_bin}")

if __name__ == "__main__":
    main()
