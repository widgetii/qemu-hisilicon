#!/usr/bin/env python3
"""
Generate synthetic CCTV scene for abandoned object detection demo.

Scene: static room, person walks in (frame 20-50), places a bag
on the floor (frame 40), walks away (frame 50-80). The bag remains
stationary from frame 40 onward → abandoned object after ~3 seconds.

Output: 200 frames at 64×48 (fits in one 4KB page for QEMU test).
"""
import os, sys
import numpy as np
from PIL import Image, ImageDraw

W, H = 64, 64
NFRAMES = 200

def draw_background(img):
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 32, W, H], fill=140)      # floor
    draw.rectangle([0, 0, W, 32], fill=120)       # wall
    draw.rectangle([4, 34, 14, 40], fill=95)      # table leg
    draw.rectangle([50, 10, 60, 20], fill=110)    # window

def draw_person(img, x, y):
    draw = ImageDraw.Draw(img)
    draw.ellipse([x+2, y, x+8, y+6], fill=50)    # head
    draw.rectangle([x, y+6, x+10, y+18], fill=45) # body
    draw.rectangle([x, y+18, x+4, y+24], fill=42) # legs
    draw.rectangle([x+6, y+18, x+10, y+24], fill=42)

def draw_bag(img, x, y):
    """A dark rectangular bag/box on the floor — high contrast vs floor."""
    draw = ImageDraw.Draw(img)
    draw.rectangle([x, y, x+10, y+7], fill=30)    # dark bag body
    draw.rectangle([x+3, y-2, x+7, y], fill=25)   # handle

def main():
    outdir = sys.argv[1] if len(sys.argv) > 1 else "demo/abandoned_frames"
    os.makedirs(outdir, exist_ok=True)
    rawdir = os.path.join(outdir, "raw")
    os.makedirs(rawdir, exist_ok=True)

    bg = Image.new("L", (W, H), 128)
    draw_background(bg)

    bag_x, bag_y = 28, 35  # bag placement position

    print(f"Generating {NFRAMES} frames ({W}×{H}) in {outdir}/")

    with open(os.path.join(outdir, "frames.bin"), "wb") as fbin:
        for i in range(NFRAMES):
            frame = bg.copy()

            # Person walks in from left (frames 20-50)
            if 20 <= i <= 50:
                px = int(-10 + (i - 20) * 2)
                py = 16
                if 0 <= px < W:
                    draw_person(frame, px, py)

            # Person walks away to right (frames 50-80)
            if 50 < i <= 80:
                px = int(50 + (i - 50) * 2)
                py = 16
                if px < W:
                    draw_person(frame, px, py)

            # Bag appears at frame 40 and stays forever
            if i >= 40:
                draw_bag(frame, bag_x, bag_y)

            arr = np.array(frame, dtype=np.uint8)
            arr.tofile(os.path.join(rawdir, f"frame_{i:04d}.raw"))
            frame.save(os.path.join(outdir, f"frame_{i:04d}.png"))
            fbin.write(arr.tobytes())

    print(f"Done: {NFRAMES} frames")
    print(f"  Scenario: person enters (20-50), places bag at ({bag_x},{bag_y}) at frame 40,")
    print(f"  walks away (50-80). Bag stays from frame 40 onward.")
    print(f"  Expected: abandoned object detected after bag is alone for ~30 frames")

if __name__ == "__main__":
    main()
