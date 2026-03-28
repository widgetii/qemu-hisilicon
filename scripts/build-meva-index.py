#!/usr/bin/env python3
"""Build activity index from MEVA + VIRAT annotation files.

Parses all *.activities.yml and *.types.yml under the dataset directory,
produces $DEST/index/activity_index.json with per-activity event lists.
"""

import json
import os
import re
import sys
from collections import defaultdict
from pathlib import Path

DEST = Path("/mnt/data/datasets/meva")


def parse_types(types_path):
    """Parse a types.yml file, return {id1: type_name}."""
    types = {}
    with open(types_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Match: { types: { id1: 0 , cset3: { Person: 1.0 } } }
            m = re.search(r'id1:\s*(\d+)\s*,\s*cset3:\s*\{\s*(\w+):', line)
            if m:
                types[int(m.group(1))] = m.group(2)
    return types


def parse_activities(act_path):
    """Parse an activities.yml file, return list of activity dicts."""
    activities = []
    with open(act_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Skip meta lines
            if "{ meta:" in line:
                continue
            # Activity type — try act3 (MEVA) then act2 (VIRAT)
            act_match = re.search(r'act[23]:\s*\{\s*(\w+):\s*[\d.]+\s*\}', line)
            if not act_match:
                continue
            activity_type = act_match.group(1)

            # Activity ID
            id_match = re.search(r'id2:\s*(\d+)', line)
            act_id = int(id_match.group(1)) if id_match else None

            # Frame range from timespan (first tsr0 at activity level)
            frames_match = re.search(r'timespan:\s*\[\{tsr0:\s*\[(\d+),\s*(\d+)\]\}', line)
            frames = [int(frames_match.group(1)), int(frames_match.group(2))] if frames_match else None

            # Actor IDs
            actor_ids = [int(x) for x in re.findall(r'id1:\s*(\d+)', line)]

            activities.append({
                "type": activity_type,
                "id": act_id,
                "frames": frames,
                "actor_ids": actor_ids,
            })
    return activities


def clip_name_from_path(act_path):
    """Extract clip name from annotation file path."""
    return act_path.stem.replace(".activities", "")


def find_source_video(clip_name):
    """Try to find the source video path for a clip."""
    # MEVA examples: check if there's a matching example video
    # Example videos are named ex000-abandon-package.mp4, not by clip name
    # For drops-123-r13 clips, construct the expected path
    # e.g. "2018-03-15.15-15-00.15-20-00.bus.G331" → drops-123-r13/2018-03-15/15/...r13.avi
    m = re.match(r'(\d{4}-\d{2}-\d{2})\.(\d{2})-', clip_name)
    if m:
        date, hour = m.group(1), m.group(2)
        return f"drops-123-r13/{date}/{hour}/{clip_name}.r13.avi"
    return None


def find_example_clip(clip_name, example_videos):
    """Check if any example video corresponds to this clip's activities."""
    # The example videos dir has all clips; we can't map by name easily
    # but we can check if an annotation came from the examples dir
    return None


def main():
    index = defaultdict(list)
    all_sources = [
        ("examples", DEST / "examples" / "annotations"),
        ("virat-train", DEST / "virat-annotations" / "train"),
        ("virat-validate", DEST / "virat-annotations" / "validate"),
    ]

    total_files = 0
    total_events = 0

    for source_label, ann_dir in all_sources:
        if not ann_dir.exists():
            print(f"  Skipping {source_label}: {ann_dir} not found")
            continue

        act_files = sorted(ann_dir.rglob("*.activities.yml"))
        print(f"  {source_label}: {len(act_files)} activity files")

        for act_path in act_files:
            total_files += 1
            clip = clip_name_from_path(act_path)

            # Load types from sibling .types.yml
            types_path = act_path.with_name(act_path.name.replace(".activities.yml", ".types.yml"))
            actor_types = parse_types(types_path) if types_path.exists() else {}

            activities = parse_activities(act_path)
            source_video = find_source_video(clip)

            for act in activities:
                total_events += 1
                actors = {}
                for aid in act["actor_ids"]:
                    actors[str(aid)] = actor_types.get(aid, "Unknown")

                entry = {
                    "clip": clip,
                    "source": source_label,
                    "frames": act["frames"],
                    "actors": actors,
                }
                if source_video:
                    entry["source_video"] = source_video

                index[act["type"]].append(entry)

    # Sort activities by name, entries by clip
    index = {k: sorted(v, key=lambda e: e["clip"]) for k, v in sorted(index.items())}

    # Write index
    out_dir = DEST / "index"
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "activity_index.json"
    with open(out_path, "w") as f:
        json.dump(index, f, indent=2)

    # Print summary
    print(f"\nParsed {total_files} annotation files, {total_events} events")
    print(f"Activity types: {len(index)}")
    print()
    for act_type, entries in index.items():
        sources = defaultdict(int)
        for e in entries:
            sources[e["source"]] += 1
        src_str = ", ".join(f"{k}={v}" for k, v in sorted(sources.items()))
        print(f"  {act_type:30s} {len(entries):4d}  ({src_str})")
    print(f"\nWritten to {out_path}")


if __name__ == "__main__":
    main()
