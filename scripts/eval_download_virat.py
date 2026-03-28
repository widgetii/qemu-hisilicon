#!/usr/bin/env python3
"""Download VIRAT Ground 2.0 videos needed for IVE evaluation.

Queries Kitware Girder API, matches against clip names in eval indexes,
downloads only the needed video files.

Usage:
    python3 scripts/eval_download_virat.py [--dry-run]
"""
import json, os, sys, urllib.request, subprocess

GIRDER_API = "https://data.kitware.com/api/v1"
FOLDER_ID = "56f581ce8d777f753209ca43"  # VIRAT Ground / videos_original
DEST = "/mnt/data/datasets/meva/virat-videos"
INDEX_DIR = "/mnt/data/datasets/meva/index"


def girder_list_items(folder_id, limit=500):
    """List all items in a Girder folder."""
    items = []
    offset = 0
    while True:
        url = f"{GIRDER_API}/item?folderId={folder_id}&limit={limit}&offset={offset}"
        with urllib.request.urlopen(url) as resp:
            batch = json.load(resp)
        if not batch:
            break
        items.extend(batch)
        offset += len(batch)
        if len(batch) < limit:
            break
    return items


def get_needed_clips():
    """Get set of clip names needed from eval indexes."""
    clips = set()
    for idx_file in ["eval_abandoned.json", "eval_motion.json"]:
        path = os.path.join(INDEX_DIR, idx_file)
        if not os.path.exists(path):
            continue
        entries = json.load(open(path))
        for e in entries:
            if "virat" in e.get("source", ""):
                clips.add(e["clip"])
    return clips


def match_clips_to_videos(needed_clips, available_items):
    """Match clip names to downloadable video items."""
    matched = {}  # clip_name -> (item_name, item_id, size)
    for item in available_items:
        name = item["name"].replace(".mp4", "")
        if name in needed_clips:
            matched[name] = (item["name"], item["_id"], item["size"])
    return matched


def download_file(item_id, dest_path):
    """Download a file from Girder."""
    url = f"{GIRDER_API}/item/{item_id}/download"
    cmd = ["curl", "-L", "-o", dest_path, "-#", url]
    subprocess.run(cmd, check=True)


def main():
    dry_run = "--dry-run" in sys.argv

    print("Loading eval indexes...")
    needed = get_needed_clips()
    print(f"Need {len(needed)} unique VIRAT clips")

    print("Querying Kitware Girder API...")
    items = girder_list_items(FOLDER_ID)
    print(f"Found {len(items)} videos in VIRAT Ground 2.0")

    matched = match_clips_to_videos(needed, items)
    unmatched = needed - set(matched.keys())

    total_size = sum(v[2] for v in matched.values())
    print(f"\nMatched: {len(matched)} clips ({total_size / 1024**3:.1f} GB)")
    if unmatched:
        print(f"Unmatched: {len(unmatched)} clips (may be segments of larger files)")
        # Try matching by base name (VIRAT_S_XXXXXX prefix)
        base_needed = set()
        for clip in unmatched:
            parts = clip.split("_")
            if len(parts) >= 3:
                base_needed.add("_".join(parts[:3]))
        base_matched = {}
        for item in items:
            name = item["name"].replace(".mp4", "")
            base = "_".join(name.split("_")[:3])
            if base in base_needed and name not in matched:
                base_matched[name] = (item["name"], item["_id"], item["size"])
        if base_matched:
            extra_size = sum(v[2] for v in base_matched.values())
            print(f"Additional by base name: {len(base_matched)} clips ({extra_size / 1024**3:.1f} GB)")
            matched.update(base_matched)
            total_size += extra_size

    print(f"\nTotal to download: {len(matched)} files ({total_size / 1024**3:.1f} GB)")

    if dry_run:
        for name in sorted(matched.keys()):
            fname, fid, sz = matched[name]
            print(f"  {fname:60s} {sz/1024**2:8.1f} MB")
        print("\n(dry run — use without --dry-run to download)")
        return

    os.makedirs(DEST, exist_ok=True)

    downloaded = 0
    skipped = 0
    for name in sorted(matched.keys()):
        fname, fid, sz = matched[name]
        dest_path = os.path.join(DEST, fname)
        if os.path.exists(dest_path) and os.path.getsize(dest_path) == sz:
            skipped += 1
            continue
        print(f"\n[{downloaded+skipped+1}/{len(matched)}] Downloading {fname} ({sz/1024**2:.0f} MB)...")
        download_file(fid, dest_path)
        downloaded += 1

    print(f"\nDone: {downloaded} downloaded, {skipped} skipped (already exist)")

    # Verify with ffprobe
    print("\nVerifying resolutions...")
    resolutions = {}
    for fname in sorted(os.listdir(DEST)):
        if not fname.endswith(".mp4"):
            continue
        path = os.path.join(DEST, fname)
        result = subprocess.run(
            ["ffprobe", "-v", "error", "-select_streams", "v:0",
             "-show_entries", "stream=width,height", "-of", "csv=p=0", path],
            capture_output=True, text=True
        )
        res = result.stdout.strip()
        resolutions[res] = resolutions.get(res, 0) + 1
    for res, count in sorted(resolutions.items()):
        print(f"  {res}: {count} clips")


if __name__ == "__main__":
    main()
