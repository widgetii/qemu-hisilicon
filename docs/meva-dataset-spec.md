# MEVA Dataset Download & Evaluation Index Spec

## Goal

Download and index the MEVA KF1 dataset for quantitative evaluation of our
IVE hardware pipeline (motion detection + abandoned object detection).
Produce precision/recall/F1 metrics against ground truth annotations.

## Dataset Overview

**MEVA (Multiview Extended Video with Activities)** — CC BY 4.0, IARPA-funded.
- S3 bucket: `s3://mevadata-public-01/` (public, no credentials needed)
- Ground cameras: 1920×1080 H.264 @ 30fps
- Activity vocabulary: 36 types including `Abandon_Package`

### S3 Structure

```
s3://mevadata-public-01/
├── examples/                          # Annotated subset (~900 MB)
│   ├── videos/                        # 121 example clips (named by activity)
│   │   ├── ex000-abandon-package.mp4  # Pre-trimmed clips around events
│   │   ├── ex001-close-trunk.mp4
│   │   └── ...
│   └── annotations/                   # Per-clip YML annotations (~6.3 MB)
│       └── YYYY-MM-DD/HH/
│           ├── *.activities.yml       # Activity type + temporal bounds
│           ├── *.geom.yml             # Per-frame bounding boxes for actors/props
│           └── *.types.yml            # Object type labels (Person, Bag, Vehicle)
├── drops-123-r13/                     # Full 5-minute raw clips (~443 GB, 3956 files)
│   └── YYYY-MM-DD/HH/*.r13.avi
├── viratannotations.snapshot.23jul2025.zip  # 175 MB — likely full annotation set
├── drop-4-hadcv22/                    # Additional clips (no annotations)
└── drop-5-mevid/                      # Re-ID subset
```

### Annotation Format

**activities.yml** — temporal bounds per activity:
```yaml
- { act: { act3: { Abandon_Package: 1.0 }, id2: 1,
    timespan: [{tsr0: [5619, 5732]}],  # frame range at 30fps
    actors: [{ id1: 1, timespan: [...] }, { id1: 0, timespan: [...] }] } }
```

**types.yml** — actor/object classification:
```yaml
- { types: { id1: 0, cset3: { Person: 1.0 } } }
- { types: { id1: 1, cset3: { Bag: 1.0 } } }
```

**geom.yml** — per-frame bounding boxes (1080p pixel coordinates):
```yaml
- { geom: { id1: 1, id0: 227, ts0: 5732, ts1: 191.07,
    g0: 1611 425 1703 522, src: truth } }
#        x1   y1   x2   y2
```

## What to Download

### Phase 1: Annotated examples (required, ~900 MB)

```bash
DEST=/mnt/data/datasets/meva
aws s3 sync s3://mevadata-public-01/examples/ $DEST/examples/ --no-sign-request
```

Contents: 121 example clip videos + 64 annotation triplets (activities + geom + types).

### Phase 2: Full annotation archive (required, ~175 MB)

```bash
aws s3 cp s3://mevadata-public-01/viratannotations.snapshot.23jul2025.zip \
    $DEST/ --no-sign-request
unzip $DEST/viratannotations.snapshot.23jul2025.zip -d $DEST/virat-annotations/
```

This likely contains annotations for the full `drops-123-r13/` clips beyond the
64 example triplets. Index it to find additional `Abandon_Package` events and
motion-heavy scenes.

### Phase 3: Selective full clips (on demand, per-clip ~100 MB)

Only download specific 5-minute clips from `drops-123-r13/` when the annotation
index identifies them as containing target activities. Example:

```bash
aws s3 cp "s3://mevadata-public-01/drops-123-r13/2018-03-15/15/\
2018-03-15.15-15-00.15-20-00.bus.G331.r13.avi" $DEST/clips/ --no-sign-request
```

## Indexing

### Step 1: Build activity index

Parse all `*.activities.yml` files and build a JSON index:

```json
{
  "Abandon_Package": [
    {
      "clip": "2018-03-15.15-15-00.15-20-00.bus.G331",
      "frames": [5619, 5732],
      "actors": {"0": "Person", "1": "Bag"},
      "source_video": "drops-123-r13/2018-03-15/15/...r13.avi",
      "example_clip": "examples/videos/ex000-abandon-package.mp4"
    }
  ],
  "Vehicle_Stopping": [...],
  ...
}
```

Output: `$DEST/index/activity_index.json`

### Step 2: Build geometry index

For each annotated clip, extract summary stats from `*.geom.yml`:

```json
{
  "clip": "2018-03-15.15-15-00.15-20-00.bus.G331",
  "tracks": {
    "0": {"type": "Person", "frames": 114, "bbox_range": [1611,239,1919,670]},
    "1": {"type": "Bag",    "frames": 114, "bbox_range": [1611,425,1703,598]}
  }
}
```

Output: `$DEST/index/geometry_index.json`

### Step 3: Select evaluation clips

For each evaluation task, select clips from the index:

**Abandoned object detection:**
- All clips with `Abandon_Package` activity
- Also `Set_Down_Object` + person leaves (derivable abandonment)
- Also `Theft` (negative — object is taken, not abandoned)
- Need clips WITHOUT any activity as true negatives

**Motion detection:**
- All clips with `Vehicle_Starting`, `Vehicle_Stopping`, `Vehicle_Reversing`
- `Enter_Vehicle`, `Exit_Vehicle` (person motion)
- `Heavy_Carry`, `Object_Transfer` (person + object motion)
- Static clips (no activities) as true negatives

Output: `$DEST/index/eval_abandoned.json`, `$DEST/index/eval_motion.json`

## Evaluation Pipeline

### Y4M Conversion

For each selected clip, convert to Y4M at Majestic-equivalent resolution:

```bash
# 1080p → 960×528 grayscale at 10fps
ffmpeg -y -i clip.avi \
    -ss $EVENT_START_SEC -t $DURATION \
    -vf "scale=960:528,format=gray" -r 10 \
    -pix_fmt gray -f yuv4mpegpipe output.y4m
```

Store: `$DEST/y4m/{clip_name}.y4m`

### Run IVE on Real Board

```bash
# Copy Y4M to NFS, run on EV300
cp $DEST/y4m/*.y4m /mnt/noc/ive-test/
ssh root@ev300-board 'killall majestic; \
    for f in /utils/ive-test/*.y4m; do \
        echo "=== $f ===" ; \
        /utils/ive-test/test-ive-video-mpi "$f" abandoned ; \
    done' > $DEST/results/ive_output.txt 2>&1
```

### Coordinate Mapping

IVE outputs block-level coordinates at processing resolution (960×528).
Ground truth is at original resolution (1920×1080).

```
GT → processing:  x_proc = x_gt * 960/1920 = x_gt / 2
                   y_proc = y_gt * 528/1080 = y_gt * 0.489

Block level:       bx = x_proc / 4
                   by = y_proc / 4
```

For IoU comparison, convert both to the same coordinate space (original 1080p).

### Metrics

**Abandoned Object Detection:**

Per-frame evaluation at 10fps:
- **True Positive (TP)**: IVE detects ABANDONED and IoU ≥ 0.1 with GT bag bbox
  (low IoU threshold because IVE operates at block level — 4×4 pixel granularity)
- **False Positive (FP)**: IVE detects ABANDONED but no GT object within IoU threshold
- **False Negative (FN)**: GT has abandoned object but IVE doesn't detect it
- **True Negative (TN)**: No GT object and no IVE detection

Aggregate metrics:
- Precision = TP / (TP + FP)
- Recall = TP / (TP + FN)
- F1 = 2 * P * R / (P + R)
- Detection latency = first IVE ABANDONED frame − GT abandonment start frame

Per-clip:
- Event-level: did IVE detect the abandonment event at all? (any frame overlap)
- Temporal accuracy: how close is first detection to GT event start?

**Motion Detection:**

Per-frame evaluation at 10fps:
- **TP**: IVE reports motion AND GT has an active track with frame-to-frame displacement
- **FP**: IVE reports motion but no GT track is moving (or no GT at all)
- **FN**: GT track is moving but IVE reports no motion
- Evaluate against GT tracks that have bbox displacement > threshold between frames

Additional:
- Spatial accuracy: IoU between IVE motion bbox and GT track bbox
- False alarm rate on static clips (no GT activities)

### Output Format

```
$DEST/
├── examples/          # Downloaded from S3
│   ├── videos/        # 121 example clips
│   └── annotations/   # 64 annotation triplets
├── virat-annotations/ # Full annotation archive
├── clips/             # Selected full 5-min clips from drops-123-r13
├── y4m/               # Converted Y4M files for IVE processing
├── index/
│   ├── activity_index.json
│   ├── geometry_index.json
│   ├── eval_abandoned.json
│   └── eval_motion.json
└── results/
    ├── ive_output.txt
    └── metrics.json     # P/R/F1 per clip and aggregate
```

## Activity Types Relevant to Our Tests

Sources: MEVA examples (64 clips), VIRAT train (64 clips), VIRAT validate (55 clips).
MEVA uses `act3` keys (e.g. `Abandon_Package`), VIRAT uses `act2` (e.g. `vehicle_moving`).

### For abandoned detection evaluation (255 events, 89 clips)

**Positives — object placed/appears and becomes static (106 events):**

| Activity | Count | Sources | Use |
|----------|-------|---------|-----|
| `Unloading` | 76 | virat | Object removed from vehicle → placed down |
| `SetDown` | 22 | virat | Object set down |
| `Set_Down_Object` | 3 | examples | Object set down (person may leave) |
| `Unload_Vehicle` | 3 | examples | Object unloaded |
| `Abandon_Package` | 1 | examples | Primary — object abandoned |
| `Drop` | 1 | virat | Object dropped |

**Negatives — object removed/transferred (132 events):**

| Activity | Count | Sources | Use |
|----------|-------|---------|-----|
| `Loading` | 76 | virat | Object loaded into vehicle → removed |
| `PickUp` | 30 | virat | Object picked up |
| `Object_Transfer` | 19 | virat+examples | Hand-to-hand transfer |
| `Load_Vehicle` | 3 | examples | Object loaded into vehicle |
| `Pick_Up_Object` | 3 | examples | Object picked up |
| `Theft` | 1 | examples | Object stolen |

**True negatives (17 clips):** motion-only clips with no object interaction.

Statistical power: 106 positives → ±9% CI on recall at 95% confidence.

### For motion detection evaluation (5049 events, 158 clips)

**Positives — vehicle motion (3060 events):**

| Activity | Count | Sources | Use |
|----------|-------|---------|-----|
| `vehicle_moving` | 1414 | virat | Vehicle in motion |
| `vehicle_stopping` | 566 | virat | Vehicle decelerating |
| `vehicle_starting` | 449 | virat | Vehicle accelerating |
| `vehicle_turning_right` | 307 | virat | Vehicle turning |
| `vehicle_turning_left` | 294 | virat | Vehicle turning |
| `vehicle_u_turn` | 22 | virat | Vehicle U-turn |
| `Vehicle_Starting` | 4 | examples | Vehicle starting |
| `Vehicle_Stopping` | 4 | examples | Vehicle stopping |
| `Vehicle_Reversing` | 4 | examples | Vehicle reversing |
| `Vehicle_Turning_Left/Right` | 8 | examples | Vehicle turning |
| `Vehicle_UTurn` | 4 | examples | Vehicle U-turn |

**Positives — person motion (1983 events):**

| Activity | Count | Sources | Use |
|----------|-------|---------|-----|
| `activity_walking` | 1495 | virat | Person walking |
| `Entering` | 146 | virat | Person entering |
| `Exiting` | 138 | virat | Person exiting |
| `Transport_HeavyCarry` | 75 | virat | Person carrying object |
| `Riding` | 46 | virat+examples | Bicycle/vehicle |
| `activity_running` | 30 | virat | Person running |
| `Object_Transfer` | 19 | virat+examples | Hand-to-hand (motion) |
| `Enter/Exit_Vehicle` | 7 | examples | Person walking to/from vehicle |
| `Enter/Exit_Facility` | 7 | examples | Person walking to/from building |
| `Heavy_Carry` | 4 | examples | Person carrying heavy object |

**True negatives (6 clips):** static-only clips (standing, sitting, talking).

## Implementation Notes

- The `examples/videos/` clips are pre-trimmed around events — useful for quick testing
  but may not have enough background frames for abandoned detection (needs 50 frames
  of static background to learn reference)
- The `drops-123-r13/` clips are 5 minutes long at 30fps = 9000 frames — plenty of
  background, but need to seek to event timestamps from annotations
- `viratannotations.snapshot.23jul2025.zip` contains 119 annotation triplets across
  train (64) and validate (55) splits. Uses VIRAT vocabulary (`act2` keys) which
  differs from MEVA examples (`act3` keys) — both are indexed
- Only 1 explicit `Abandon_Package` event exists. Abandoned detection eval uses
  `Unloading`/`SetDown`/`Drop` as positives (object becomes static) and
  `Loading`/`PickUp`/`Theft` as negatives (object removed), totaling 238 labeled events
- VIRAT clips reference external video files (VIRAT_S_*) not present in the S3 bucket;
  only MEVA `examples/videos/` and `drops-123-r13/` have downloadable video
- All evaluation should use the same IVE binary (`test-ive-video-mpi`) to ensure
  we measure real hardware performance, not a simulation
