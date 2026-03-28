#!/usr/bin/env python3
"""Build Y4M evaluation dataset from MEVA examples + VIRAT Ground 2.0 videos.

Converts clips to Y4M at Majestic-equivalent resolution (MD_SIZE = half, align 16).
For abandoned clips, includes pre-event context for background learning.

Usage:
    python3 scripts/eval_build_dataset.py [--mode abandoned|md|both] [--max-clips N]
"""
import json, os, sys, subprocess, re

DATASET = "/mnt/data/datasets/meva"
EXAMPLES_DIR = os.path.join(DATASET, "examples/videos")
VIRAT_DIR = os.path.join(DATASET, "virat-videos")
Y4M_DIR = os.path.join(DATASET, "y4m")
INDEX_DIR = os.path.join(DATASET, "index")


def md_size(x):
    """Majestic's MD_SIZE: half, aligned to 16."""
    return (x // 2) & ~0xF


def get_resolution(video_path):
    """Get video resolution via ffprobe."""
    result = subprocess.run(
        ["ffprobe", "-v", "error", "-select_streams", "v:0",
         "-show_entries", "stream=width,height,r_frame_rate",
         "-of", "csv=p=0", video_path],
        capture_output=True, text=True, timeout=10
    )
    parts = result.stdout.strip().split(",")
    if len(parts) >= 2:
        return int(parts[0]), int(parts[1])
    return None, None


def find_video(clip_name, source):
    """Find video file for a clip."""
    if source == "examples":
        # Try example videos (pre-trimmed clips with activity names)
        for f in os.listdir(EXAMPLES_DIR):
            if f.endswith(".mp4"):
                return os.path.join(EXAMPLES_DIR, f), "example"
        # Try matching by clip name pattern in drops
        return None, None
    else:
        # VIRAT clip — look in virat-videos/
        mp4 = os.path.join(VIRAT_DIR, clip_name + ".mp4")
        if os.path.exists(mp4):
            return mp4, "virat"
        return None, None


def find_example_video_for_clip(clip_name, activity):
    """Find the pre-trimmed example clip for a MEVA example annotation."""
    # Example clips are named ex000-activity-name.mp4
    act_slug = activity.lower().replace("_", "-")
    for f in sorted(os.listdir(EXAMPLES_DIR)):
        if act_slug in f and f.endswith(".mp4"):
            return os.path.join(EXAMPLES_DIR, f)
    return None


def convert_to_y4m(video_path, output_path, proc_w, proc_h, ss=None, duration=None):
    """Convert video to Y4M Cmono at given resolution."""
    cmd = ["ffmpeg", "-y"]
    if ss is not None:
        cmd += ["-ss", str(ss)]
    cmd += ["-i", video_path]
    if duration is not None:
        cmd += ["-t", str(duration)]
    cmd += [
        "-vf", f"scale={proc_w}:{proc_h},format=gray",
        "-r", "10",
        "-pix_fmt", "gray",
        "-f", "yuv4mpegpipe",
        output_path
    ]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
    return result.returncode == 0


def main():
    mode = "both"
    max_clips = 9999
    for arg in sys.argv[1:]:
        if arg.startswith("--mode="):
            mode = arg.split("=")[1]
        elif arg.startswith("--max-clips="):
            max_clips = int(arg.split("=")[1])

    os.makedirs(Y4M_DIR, exist_ok=True)

    # Load eval indexes
    eval_files = []
    if mode in ("abandoned", "both"):
        eval_files.append(("eval_abandoned.json", "abandoned"))
    if mode in ("md", "both"):
        eval_files.append(("eval_motion.json", "md"))

    converted = 0
    skipped = 0
    failed = 0

    for idx_file, eval_mode in eval_files:
        entries = json.load(open(os.path.join(INDEX_DIR, idx_file)))
        print(f"\n=== {eval_mode}: {len(entries)} entries ===")

        # Group by clip to avoid duplicate conversions
        clips = {}
        for e in entries:
            clip = e["clip"]
            if clip not in clips:
                clips[clip] = e

        for clip_name, entry in sorted(clips.items()):
            if converted + skipped >= max_clips:
                break

            out_name = f"{clip_name}.{eval_mode}.y4m"
            out_path = os.path.join(Y4M_DIR, out_name)

            if os.path.exists(out_path) and os.path.getsize(out_path) > 1000:
                skipped += 1
                continue

            source = entry["source"]
            activity = entry.get("activity")
            frames = entry.get("frames", [0, 300])

            # Find video
            if source == "examples":
                video_path = find_example_video_for_clip(clip_name, activity or "")
                if not video_path:
                    # Try to find any matching example
                    for f in os.listdir(EXAMPLES_DIR):
                        if f.endswith(".mp4"):
                            # Check if this example's annotation matches our clip
                            pass
                    video_path = None
            else:
                video_path = os.path.join(VIRAT_DIR, clip_name + ".mp4")
                if not os.path.exists(video_path):
                    video_path = None

            if not video_path or not os.path.exists(video_path):
                failed += 1
                if failed <= 10:
                    print(f"  SKIP {clip_name}: no video (source={source})")
                continue

            # Get resolution
            orig_w, orig_h = get_resolution(video_path)
            if not orig_w:
                failed += 1
                continue

            proc_w = md_size(orig_w)
            proc_h = md_size(orig_h)

            # Calculate seek and duration
            fps = 30  # VIRAT/MEVA standard
            if not frames or frames[0] is None:
                # No frame info — use full clip (first 30s)
                ss = None
                duration = 30
            elif eval_mode == "abandoned" and source != "examples":
                # Need pre-event context for background learning
                event_start_sec = frames[0] / fps
                event_end_sec = frames[1] / fps if len(frames) > 1 else event_start_sec + 10
                ss = max(0, event_start_sec - 10)  # 10s before event
                duration = (event_end_sec - ss) + 5  # event + 5s after
            elif source == "examples":
                # Example clips are already trimmed
                ss = None
                duration = None
            else:
                event_start_sec = frames[0] / fps
                event_end_sec = frames[1] / fps if len(frames) > 1 else event_start_sec + 10
                ss = max(0, event_start_sec - 5)
                duration = (event_end_sec - ss) + 5

            ok = convert_to_y4m(video_path, out_path, proc_w, proc_h, ss, duration)
            if ok and os.path.exists(out_path) and os.path.getsize(out_path) > 1000:
                # Save extraction metadata for overlay scripts
                meta = {
                    "source_video": video_path,
                    "clip": clip_name,
                    "mode": eval_mode,
                    "proc_w": proc_w, "proc_h": proc_h,
                    "orig_w": orig_w, "orig_h": orig_h,
                    "time_offset": ss if ss is not None else 0,
                    "duration": duration,
                    "gt_frames": frames,
                    "label": entry.get("label"),
                    "activity": entry.get("activity"),
                }
                meta_path = out_path.replace(".y4m", ".meta.json")
                with open(meta_path, "w") as mf:
                    json.dump(meta, mf, indent=2)

                converted += 1
                if converted <= 20 or converted % 50 == 0:
                    sz_mb = os.path.getsize(out_path) / 1024 / 1024
                    print(f"  [{converted}] {out_name} ({proc_w}x{proc_h}, {sz_mb:.0f} MB)")
            else:
                failed += 1
                if os.path.exists(out_path):
                    os.remove(out_path)

    print(f"\nDone: {converted} converted, {skipped} skipped, {failed} failed")
    print(f"Y4M directory: {Y4M_DIR}")

    # Summary
    y4m_files = [f for f in os.listdir(Y4M_DIR) if f.endswith(".y4m")]
    total_sz = sum(os.path.getsize(os.path.join(Y4M_DIR, f)) for f in y4m_files)
    print(f"Total Y4M files: {len(y4m_files)} ({total_sz / 1024**3:.1f} GB)")


if __name__ == "__main__":
    main()
