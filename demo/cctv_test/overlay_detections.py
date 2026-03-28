#!/usr/bin/env python3
"""Overlay IVE detection bboxes on original 1080p video frames using PIL."""
import re, sys, os, subprocess
from PIL import Image, ImageDraw, ImageFont

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
            if frame not in dets:
                dets[frame] = []
            dets[frame].append((x1, y1, x2, y2, area, dur, is_abandoned))
    return dets

def main():
    if len(sys.argv) < 7:
        print(f"Usage: {sys.argv[0]} <detections.txt> <source.avi> <output.mp4> <proc_w> <proc_h> <time_offset>")
        sys.exit(1)

    det_file, source, output = sys.argv[1], sys.argv[2], sys.argv[3]
    proc_w, proc_h = int(sys.argv[4]), int(sys.argv[5])
    time_offset = float(sys.argv[6])

    dets = parse_detections(det_file)
    max_frame = max(dets.keys()) if dets else 0
    duration = max_frame / 10.0 + 1.0
    nframes = int(duration * 10)
    print(f"Detections: {sum(len(v) for v in dets.values())} across {len(dets)} frames, max_frame={max_frame}")

    tmpdir = "/tmp/ive_overlay_frames"
    os.makedirs(tmpdir, exist_ok=True)

    # Extract frames from original at 10fps
    print(f"Extracting {nframes} frames from original at 10fps...")
    subprocess.run([
        "ffmpeg", "-y", "-ss", str(time_offset), "-i", source,
        "-t", str(duration), "-r", "10",
        "-q:v", "2", f"{tmpdir}/frame_%04d.png"
    ], capture_output=True, timeout=120)

    orig_w, orig_h = 1920, 1080
    sx = orig_w / proc_w
    sy = orig_h / proc_h

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

        # Frame number label
        frame_idx = fi - 1  # 0-based
        label = f"Frame {frame_idx}"

        boxes = dets.get(frame_idx, [])
        abandoned_count = sum(1 for b in boxes if b[6] and b[5] >= 30)

        if abandoned_count:
            label += f"  |  {abandoned_count} ABANDONED"
        elif boxes:
            label += f"  |  {len(boxes)} motion"

        # Draw status bar
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

    # Cleanup
    for f in os.listdir(tmpdir):
        os.remove(os.path.join(tmpdir, f))

    sz = os.path.getsize(output) / 1024 / 1024
    print(f"Output: {output} ({sz:.1f} MB)")

if __name__ == "__main__":
    main()
