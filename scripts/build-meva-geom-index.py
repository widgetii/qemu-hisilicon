#!/usr/bin/env python3
"""Build geometry index from MEVA + VIRAT geom.yml files.

For each annotated clip, extracts per-track summary stats (frame count,
enclosing bbox) and merges actor types from types.yml.

Output: $DEST/index/geometry_index.json
"""

import json
import re
import sys
from collections import defaultdict
from pathlib import Path

DEST = Path("/mnt/data/datasets/meva")

GEOM_RE = re.compile(
    r'id1:\s*(\d+).*?ts0:\s*(\d+).*?g0:\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)'
)


def parse_types(types_path):
    """Parse a types.yml file, return {id1: type_name}."""
    types = {}
    with open(types_path) as f:
        for line in f:
            m = re.search(r'id1:\s*(\d+)\s*,\s*cset3:\s*\{\s*(\w+):', line)
            if m:
                types[int(m.group(1))] = m.group(2)
    return types


def parse_geom(geom_path):
    """Stream-parse a geom.yml, return per-track summary.

    Returns dict: {id1: {"frames": count, "bbox_range": [x1_min, y1_min, x2_max, y2_max],
                         "frame_range": [first_frame, last_frame]}}
    """
    tracks = {}
    for line in open(geom_path):
        m = GEOM_RE.search(line)
        if not m:
            continue
        tid = int(m.group(1))
        frame = int(m.group(2))
        x1, y1, x2, y2 = int(m.group(3)), int(m.group(4)), int(m.group(5)), int(m.group(6))

        if tid not in tracks:
            tracks[tid] = {
                "frames": 0,
                "bbox_range": [x1, y1, x2, y2],
                "frame_range": [frame, frame],
            }
        t = tracks[tid]
        t["frames"] += 1
        b = t["bbox_range"]
        b[0] = min(b[0], x1)
        b[1] = min(b[1], y1)
        b[2] = max(b[2], x2)
        b[3] = max(b[3], y2)
        fr = t["frame_range"]
        fr[0] = min(fr[0], frame)
        fr[1] = max(fr[1], frame)

    return tracks


def main():
    geometry_index = []

    all_sources = [
        ("examples", DEST / "examples" / "annotations"),
        ("virat-train", DEST / "virat-annotations" / "train"),
        ("virat-validate", DEST / "virat-annotations" / "validate"),
    ]

    total_files = 0
    total_tracks = 0
    total_detections = 0

    for source_label, ann_dir in all_sources:
        if not ann_dir.exists():
            print(f"  Skipping {source_label}: {ann_dir} not found")
            continue

        geom_files = sorted(ann_dir.rglob("*.geom.yml"))
        print(f"  {source_label}: {len(geom_files)} geometry files")

        for geom_path in geom_files:
            total_files += 1
            clip = geom_path.stem.replace(".geom", "")

            # Load types
            types_path = geom_path.with_name(geom_path.name.replace(".geom.yml", ".types.yml"))
            actor_types = parse_types(types_path) if types_path.exists() else {}

            tracks = parse_geom(geom_path)
            if not tracks:
                continue

            track_summaries = {}
            for tid, info in sorted(tracks.items()):
                total_tracks += 1
                total_detections += info["frames"]
                track_summaries[str(tid)] = {
                    "type": actor_types.get(tid, "Unknown"),
                    "frames": info["frames"],
                    "frame_range": info["frame_range"],
                    "bbox_range": info["bbox_range"],
                }

            geometry_index.append({
                "clip": clip,
                "source": source_label,
                "tracks": track_summaries,
            })

            if total_files % 20 == 0:
                print(f"    processed {total_files} files, {total_detections} detections...",
                      file=sys.stderr, flush=True)

    # Sort by clip name
    geometry_index.sort(key=lambda e: e["clip"])

    out_dir = DEST / "index"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "geometry_index.json"
    with open(out_path, "w") as f:
        json.dump(geometry_index, f, indent=2)

    print(f"\nParsed {total_files} geometry files")
    print(f"Total tracks: {total_tracks}")
    print(f"Total detections: {total_detections}")
    print(f"Written to {out_path} ({out_path.stat().st_size / 1024 / 1024:.1f} MB)")


if __name__ == "__main__":
    main()
