#!/usr/bin/env python3
"""Overlay IVE detection bboxes on original video frames using PIL.

Can auto-detect source video, resolution, and time offset from .meta.json
files saved by eval_build_dataset.py.

Usage:
    # Auto mode: reads .meta.json next to the result file
    python3 overlay_detections.py <detections.txt> <output.mp4>

    # Manual mode: specify all parameters
    python3 overlay_detections.py <detections.txt> <source.avi> <output.mp4> <proc_w> <proc_h> <time_offset>
"""
import json, re, sys, os, subprocess
from PIL import Image, ImageDraw


Y4M_DIR = "/mnt/data/datasets/meva/y4m"


def parse_detections(path):
    dets = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'FRAME (\d+): (?:ABANDONED )?\((\d+),(\d+)\)-\((\d+),(\d+)\) area=(\d+)(?: dur=(\d+))?', line.strip())
            if not m:
                continue
            frame = int(m.group(1))
            x1, y1, x2, y2 = int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
            area = int(m.group(6))
            dur = int(m.group(7)) if m.group(7) else 0
            is_abandoned = "ABANDONED" in line
            dets.setdefault(frame, []).append((x1, y1, x2, y2, area, dur, is_abandoned))
    return dets


def find_meta(det_file):
    """Find .meta.json matching a detection result file.

    Result files are like: results/CLIP.MODE.txt
    Meta files are like:   y4m/CLIP.MODE.meta.json
    """
    base = os.path.basename(det_file)  # CLIP.MODE.txt
    stem = base.rsplit(".", 1)[0]      # CLIP.MODE
    meta_path = os.path.join(Y4M_DIR, stem + ".meta.json")
    if os.path.exists(meta_path):
        return json.load(open(meta_path))
    return None


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <detections.txt> <output.mp4>")
        print(f"       {sys.argv[0]} <detections.txt> <source> <output.mp4> <proc_w> <proc_h> <time_offset>")
        sys.exit(1)

    det_file = sys.argv[1]

    if len(sys.argv) >= 7:
        # Manual mode
        source = sys.argv[2]
        output = sys.argv[3]
        proc_w, proc_h = int(sys.argv[4]), int(sys.argv[5])
        time_offset = float(sys.argv[6])
    elif len(sys.argv) >= 3:
        # Auto mode: read from .meta.json
        output = sys.argv[2]
        meta = find_meta(det_file)
        if not meta:
            print(f"No .meta.json found for {det_file}. Use manual mode.")
            sys.exit(1)
        source = meta["source_video"]
        proc_w = meta["proc_w"]
        proc_h = meta["proc_h"]
        time_offset = meta["time_offset"]
        print(f"Auto: source={os.path.basename(source)} {proc_w}x{proc_h} offset={time_offset}s")

    dets = parse_detections(det_file)
    max_frame = max(dets.keys()) if dets else 0
    duration = max_frame / 10.0 + 1.0
    nframes = int(duration * 10)
    print(f"Detections: {sum(len(v) for v in dets.values())} across {len(dets)} frames, max_frame={max_frame}")

    tmpdir = "/tmp/ive_overlay_frames"
    os.makedirs(tmpdir, exist_ok=True)

    # Extract frames from original at 10fps
    print(f"Extracting {nframes} frames from original at 10fps (offset={time_offset}s)...")
    subprocess.run([
        "ffmpeg", "-y", "-ss", str(time_offset), "-i", source,
        "-t", str(duration), "-r", "10",
        "-q:v", "2", f"{tmpdir}/frame_%04d.png"
    ], capture_output=True, timeout=120)

    frames_drawn = 0
    for fi in range(1, nframes + 2):
        path = f"{tmpdir}/frame_{fi:04d}.png"
        if not os.path.exists(path):
            continue
        img = Image.open(path)
        orig_w, orig_h = img.size
        sx = orig_w / proc_w
        sy = orig_h / proc_h
        draw = ImageDraw.Draw(img)

        frame_idx = fi - 1
        label = f"Frame {frame_idx}"

        boxes = dets.get(frame_idx, [])
        abandoned_count = sum(1 for b in boxes if b[6] and b[5] >= 30)

        if abandoned_count:
            label += f"  |  {abandoned_count} ABANDONED"
        elif boxes:
            label += f"  |  {len(boxes)} motion"

        draw.rectangle([0, 0, orig_w, 32], fill=(0, 0, 0, 180))
        draw.text((10, 6), label, fill=(0, 255, 0))

        for x1, y1, x2, y2, area, dur, is_ab in boxes:
            if area < 4:
                continue
            ox1 = int(x1 * sx)
            oy1 = int(y1 * sy)
            ox2 = int(x2 * sx)
            oy2 = int(y2 * sy)

            # Expand small boxes for visibility (min 20px)
            if ox2 - ox1 < 20:
                cx = (ox1 + ox2) // 2
                ox1, ox2 = cx - 10, cx + 10
            if oy2 - oy1 < 20:
                cy = (oy1 + oy2) // 2
                oy1, oy2 = cy - 10, cy + 10

            if is_ab and dur >= 30:
                color = (255, 0, 0)
                width = 3
                txt = f"ABANDONED dur={dur}"
            else:
                color = (255, 255, 0)
                width = 2
                txt = f"motion area={area}"

            draw.rectangle([ox1, oy1, ox2, oy2], outline=color, width=width)
            draw.text((ox1, max(34, oy1 - 18)), txt, fill=color)

        img.save(path)
        frames_drawn += 1

    print(f"Drew boxes on {frames_drawn} frames, encoding...")

    subprocess.run([
        "ffmpeg", "-y", "-r", "10",
        "-i", f"{tmpdir}/frame_%04d.png",
        "-c:v", "libx264", "-preset", "fast", "-crf", "18",
        "-pix_fmt", "yuv420p", output
    ], capture_output=True, timeout=120)

    for f in os.listdir(tmpdir):
        os.remove(os.path.join(tmpdir, f))

    sz = os.path.getsize(output) / 1024 / 1024
    print(f"Output: {output} ({sz:.1f} MB)")


if __name__ == "__main__":
    main()
