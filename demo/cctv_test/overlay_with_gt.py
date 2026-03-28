#!/usr/bin/env python3
"""Overlay IVE detections + MEVA ground truth on original 1080p video."""
import re, sys, os, subprocess
from PIL import Image, ImageDraw

def parse_detections(path):
    dets = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'FRAME (\d+): (?:ABANDONED )?\((\d+),(\d+)\)-\((\d+),(\d+)\) area=(\d+)(?: dur=(\d+))?', line.strip())
            if not m: continue
            frame = int(m.group(1))
            x1, y1, x2, y2 = int(m.group(2)), int(m.group(3)), int(m.group(4)), int(m.group(5))
            area, dur = int(m.group(6)), int(m.group(7)) if m.group(7) else 0
            is_ab = "ABANDONED" in line
            dets.setdefault(frame, []).append((x1, y1, x2, y2, area, dur, is_ab))
    return dets

def parse_geom(path):
    """Parse MEVA geom.yml into {id1: {frame: (x1,y1,x2,y2)}}"""
    tracks = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'.*id1: (\d+).*ts0: (\d+).*g0: (\d+) (\d+) (\d+) (\d+)', line)
            if not m: continue
            id1, frame = int(m.group(1)), int(m.group(2))
            x1, y1, x2, y2 = int(m.group(3)), int(m.group(4)), int(m.group(5)), int(m.group(6))
            tracks.setdefault(id1, {})[frame] = (x1, y1, x2, y2)
    return tracks

def parse_types(path):
    types = {}
    with open(path) as f:
        for line in f:
            m = re.match(r'.*id1: (\d+).*cset3: \{ (\w+):', line)
            if m:
                types[int(m.group(1))] = m.group(2)
    return types

def main():
    det_file = sys.argv[1]    # detections.txt
    geom_file = sys.argv[2]   # .geom.yml
    types_file = sys.argv[3]  # .types.yml
    source = sys.argv[4]      # source.avi
    output = sys.argv[5]      # output.mp4
    proc_w, proc_h = int(sys.argv[6]), int(sys.argv[7])
    time_offset = float(sys.argv[8])  # -ss offset used for Y4M extraction

    dets = parse_detections(det_file)
    gt_tracks = parse_geom(geom_file)
    gt_types = parse_types(types_file)

    max_frame = max(dets.keys()) if dets else 0
    duration = max_frame / 10.0 + 1.0
    print(f"Detections: {sum(len(v) for v in dets.values())} across {len(dets)} frames")
    track_desc = ', '.join(f'{gt_types.get(k,"?")}(id={k})' for k in gt_tracks)
    print(f"GT tracks: {len(gt_tracks)} ({track_desc})")

    tmpdir = "/tmp/ive_gt_frames"
    os.makedirs(tmpdir, exist_ok=True)

    # Extract frames
    print(f"Extracting frames at 10fps for {duration:.1f}s...")
    subprocess.run([
        "ffmpeg", "-y", "-ss", str(time_offset), "-i", source,
        "-t", str(duration), "-r", "10", "-q:v", "2",
        f"{tmpdir}/frame_%04d.png"
    ], capture_output=True, timeout=120)

    orig_w, orig_h = 1920, 1080
    sx = orig_w / proc_w
    sy = orig_h / proc_h
    # GT frames are at 30fps, our processing at 10fps, video starts at time_offset
    gt_frame_offset = int(time_offset * 30)

    for fi in range(1, int(duration * 10) + 2):
        path = f"{tmpdir}/frame_{fi:04d}.png"
        if not os.path.exists(path): continue
        img = Image.open(path)
        draw = ImageDraw.Draw(img)
        frame_idx = fi - 1  # 0-based processing frame
        src_frame = gt_frame_offset + frame_idx * 3  # approximate 30fps source frame

        # Draw GT boxes (green for person, cyan for bag)
        gt_labels = []
        for tid, track in gt_tracks.items():
            tname = gt_types.get(tid, "?")
            # Find nearest GT frame
            best = None
            for gf in [src_frame, src_frame-1, src_frame+1, src_frame-2, src_frame+2]:
                if gf in track:
                    best = track[gf]
                    break
            if not best: continue
            gx1, gy1, gx2, gy2 = best
            color = (0, 255, 0) if tname == "Person" else (0, 255, 255)
            draw.rectangle([gx1, gy1, gx2, gy2], outline=color, width=2)
            draw.text((gx1, max(0, gy1 - 18)), f"GT:{tname}", fill=color)
            gt_labels.append(tname)

        # Draw IVE detections (red=abandoned, yellow=other)
        boxes = dets.get(frame_idx, [])
        det_labels = []
        for x1, y1, x2, y2, area, dur, is_ab in boxes:
            if area < 4: continue
            ox1, oy1 = int(x1 * sx), int(y1 * sy)
            ox2, oy2 = int(x2 * sx), int(y2 * sy)
            # Expand small boxes
            if ox2 - ox1 < 20:
                cx = (ox1 + ox2) // 2; ox1, ox2 = cx - 10, cx + 10
            if oy2 - oy1 < 20:
                cy = (oy1 + oy2) // 2; oy1, oy2 = cy - 10, cy + 10
            if is_ab and dur >= 30:
                draw.rectangle([ox1, oy1, ox2, oy2], outline=(255, 0, 0), width=3)
                draw.text((ox1, max(34, oy1 - 18)), f"IVE:ABANDONED d={dur}", fill=(255, 0, 0))
                det_labels.append(f"AB d={dur}")
            else:
                draw.rectangle([ox1, oy1, ox2, oy2], outline=(255, 255, 0), width=2)

        # Status bar
        draw.rectangle([0, 0, orig_w, 32], fill=(0, 0, 0))
        status = f"Frame {frame_idx}"
        if gt_labels:
            status += f"  |  GT: {', '.join(gt_labels)}"
        if det_labels:
            status += f"  |  IVE: {len(det_labels)} abandoned"
        draw.text((10, 6), status, fill=(0, 255, 0))

        # Legend
        draw.rectangle([orig_w - 300, orig_h - 60, orig_w, orig_h], fill=(0, 0, 0))
        draw.text((orig_w - 290, orig_h - 55), "Green/Cyan = GT", fill=(0, 255, 0))
        draw.text((orig_w - 290, orig_h - 35), "Red = IVE detected", fill=(255, 0, 0))

        img.save(path)

    print("Encoding...")
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
