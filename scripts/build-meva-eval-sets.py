#!/usr/bin/env python3
"""Build evaluation clip selection files from the activity + geometry indices.

Produces:
  $DEST/index/eval_abandoned.json  — clips for abandoned object detection eval
  $DEST/index/eval_motion.json     — clips for motion detection eval

Selection criteria follow docs/meva-dataset-spec.md.
"""

import json
from collections import defaultdict
from pathlib import Path

DEST = Path("/mnt/data/datasets/meva")


def load_indices():
    act_index = json.load(open(DEST / "index" / "activity_index.json"))
    geom_index = json.load(open(DEST / "index" / "geometry_index.json"))
    # Build clip→geometry lookup
    geom_by_clip = {e["clip"]: e for e in geom_index}
    # Build clip→activities lookup
    clip_activities = defaultdict(list)
    for act_type, events in act_index.items():
        for e in events:
            clip_activities[e["clip"]].append({
                "activity": act_type,
                "frames": e["frames"],
                "actors": e["actors"],
                "source": e["source"],
            })
    return act_index, geom_by_clip, clip_activities


def get_track_info(geom_by_clip, clip, actor_ids):
    """Get geometry summary for specific actors in a clip."""
    geom = geom_by_clip.get(clip)
    if not geom:
        return {}
    tracks = {}
    for aid in actor_ids:
        if aid in geom["tracks"]:
            tracks[aid] = geom["tracks"][aid]
    return tracks


def build_abandoned_eval(act_index, geom_by_clip, clip_activities):
    """Select clips for abandoned object detection evaluation.

    Positive (object placed/appears and becomes static):
      Abandon_Package, Set_Down_Object, SetDown, Drop,
      Unloading, Unload_Vehicle
    Negative (object removed/transferred):
      Pick_Up_Object, PickUp, Theft, Object_Transfer,
      Loading, Load_Vehicle
    True negative: clips with only vehicle/walking activities (no object interaction)
    """
    positive_types = {
        "Abandon_Package", "Set_Down_Object", "SetDown", "Drop",
        "Unloading", "Unload_Vehicle",
    }
    negative_types = {
        "Pick_Up_Object", "PickUp", "Theft", "Object_Transfer",
        "Loading", "Load_Vehicle",
    }
    # Activities that indicate NO object abandonment (for true negatives)
    motion_only_types = {
        "vehicle_moving", "vehicle_starting", "vehicle_stopping",
        "vehicle_turning_left", "vehicle_turning_right", "vehicle_u_turn",
        "activity_walking", "activity_standing", "activity_running",
        "activity_sitting", "activity_crouching",
        "Vehicle_Starting", "Vehicle_Stopping", "Vehicle_Reversing",
        "Vehicle_Turning_Left", "Vehicle_Turning_Right", "Vehicle_UTurn",
        "Riding",
    }

    entries = []
    seen_clips = set()

    # Collect positive clips
    for act_type in sorted(positive_types):
        for event in act_index.get(act_type, []):
            clip = event["clip"]
            key = (clip, act_type, tuple(event["frames"] or []))
            if key in seen_clips:
                continue
            seen_clips.add(key)
            tracks = get_track_info(geom_by_clip, clip, list(event["actors"].keys()))
            entries.append({
                "clip": clip,
                "source": event["source"],
                "label": "positive",
                "activity": act_type,
                "frames": event["frames"],
                "actors": event["actors"],
                "tracks": tracks,
                "source_video": event.get("source_video"),
            })

    # Collect negative clips (object picked up / stolen / transferred)
    for act_type in sorted(negative_types):
        for event in act_index.get(act_type, []):
            clip = event["clip"]
            key = (clip, act_type, tuple(event["frames"] or []))
            if key in seen_clips:
                continue
            seen_clips.add(key)
            tracks = get_track_info(geom_by_clip, clip, list(event["actors"].keys()))
            entries.append({
                "clip": clip,
                "source": event["source"],
                "label": "negative",
                "activity": act_type,
                "frames": event["frames"],
                "actors": event["actors"],
                "tracks": tracks,
                "source_video": event.get("source_video"),
            })

    # Collect true negatives: clips where ALL activities are motion-only
    positive_negative_clips = {e["clip"] for e in entries}
    for clip, acts in clip_activities.items():
        if clip in positive_negative_clips:
            continue
        act_types_in_clip = {a["activity"] for a in acts}
        if act_types_in_clip.issubset(motion_only_types):
            entries.append({
                "clip": clip,
                "source": acts[0]["source"],
                "label": "true_negative",
                "activity": None,
                "frames": None,
                "actors": {},
                "tracks": {},
                "source_video": None,
            })

    return entries


def build_motion_eval(act_index, geom_by_clip, clip_activities):
    """Select clips for motion detection evaluation.

    Vehicle motion: vehicle_moving, vehicle_starting, vehicle_stopping,
                    vehicle_turning_*, vehicle_u_turn, Vehicle_Starting, etc.
    Person motion:  activity_walking, activity_running, Enter/Exit_Vehicle/Facility,
                    Heavy_Carry, Transport_HeavyCarry, Riding
    True negative:  clips with only static activities (standing, sitting, talking)
    """
    vehicle_types = {
        "vehicle_moving", "vehicle_starting", "vehicle_stopping",
        "vehicle_turning_left", "vehicle_turning_right", "vehicle_u_turn",
        "Vehicle_Starting", "Vehicle_Stopping", "Vehicle_Reversing",
        "Vehicle_Turning_Left", "Vehicle_Turning_Right", "Vehicle_UTurn",
    }
    person_motion_types = {
        "activity_walking", "activity_running",
        "Enter_Vehicle", "Exit_Vehicle", "Enter_Facility", "Exit_Facility",
        "Entering", "Exiting",
        "Heavy_Carry", "Transport_HeavyCarry",
        "Object_Transfer", "Riding",
    }
    static_types = {
        "activity_standing", "activity_sitting", "activity_crouching",
        "Talking", "People_Talking", "Talk_on_Phone",
        "specialized_talking_phone", "specialized_texting_phone",
        "Text_On_Phone", "Read_Document", "Laptop_Interaction",
        "activity_gesturing", "Interacts", "Person_Person_Interaction",
    }

    all_motion = vehicle_types | person_motion_types
    entries = []
    seen_clips = set()

    # Collect motion clips
    for act_type in sorted(all_motion):
        for event in act_index.get(act_type, []):
            clip = event["clip"]
            key = (clip, act_type, tuple(event["frames"] or []))
            if key in seen_clips:
                continue
            seen_clips.add(key)

            motion_class = "vehicle" if act_type in vehicle_types else "person"
            tracks = get_track_info(geom_by_clip, clip, list(event["actors"].keys()))
            entries.append({
                "clip": clip,
                "source": event["source"],
                "label": "positive",
                "motion_class": motion_class,
                "activity": act_type,
                "frames": event["frames"],
                "actors": event["actors"],
                "tracks": tracks,
                "source_video": event.get("source_video"),
            })

    # True negatives: clips where ALL activities are static
    motion_clips = {e["clip"] for e in entries}
    for clip, acts in clip_activities.items():
        if clip in motion_clips:
            continue
        act_types_in_clip = {a["activity"] for a in acts}
        if act_types_in_clip.issubset(static_types):
            entries.append({
                "clip": clip,
                "source": acts[0]["source"],
                "label": "true_negative",
                "motion_class": "static",
                "activity": None,
                "frames": None,
                "actors": {},
                "tracks": {},
                "source_video": None,
            })

    return entries


def print_summary(name, entries):
    print(f"\n=== {name} ===")
    by_label = defaultdict(int)
    by_activity = defaultdict(int)
    by_source = defaultdict(int)
    clips = set()
    for e in entries:
        by_label[e["label"]] += 1
        if e["activity"]:
            by_activity[e["activity"]] += 1
        by_source[e["source"]] += 1
        clips.add(e["clip"])

    print(f"  Total events: {len(entries)}, unique clips: {len(clips)}")
    print(f"  By label: {dict(sorted(by_label.items()))}")
    print(f"  By source: {dict(sorted(by_source.items()))}")
    print(f"  By activity:")
    for act, cnt in sorted(by_activity.items(), key=lambda x: -x[1]):
        print(f"    {act:30s} {cnt:4d}")


def main():
    act_index, geom_by_clip, clip_activities = load_indices()

    abandoned = build_abandoned_eval(act_index, geom_by_clip, clip_activities)
    motion = build_motion_eval(act_index, geom_by_clip, clip_activities)

    out_dir = DEST / "index"
    out_dir.mkdir(parents=True, exist_ok=True)

    with open(out_dir / "eval_abandoned.json", "w") as f:
        json.dump(abandoned, f, indent=2)

    with open(out_dir / "eval_motion.json", "w") as f:
        json.dump(motion, f, indent=2)

    print_summary("eval_abandoned", abandoned)
    print_summary("eval_motion", motion)

    print(f"\nWritten to {out_dir / 'eval_abandoned.json'}")
    print(f"Written to {out_dir / 'eval_motion.json'}")


if __name__ == "__main__":
    main()
