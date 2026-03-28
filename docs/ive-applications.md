# IVE-Based Video Analytics Applications for OpenIPC

Practical CV applications that can be built on EV300 (Cortex-A7, 64MB RAM)
using IVE hardware acceleration. Grouped by implementation complexity.

## Tier 1: Ready to build (existing 17 IVE ops)

### Scene Change / Tamper Detection
**IVE ops:** Hist + Sub + Thresh
**Algorithm:** Compare current frame's histogram against a reference.
Camera covered = all dark (histogram collapses to low bins).
Camera moved = completely different distribution.
Camera defocused = reduced high-frequency content (Sobel energy drops).
**Metadata output:**
```json
{"sceneChange": {"type": "covered", "confidence": 0.95, "time": "..."}}
```
**Value:** Every security system needs tamper detection. Currently missing from majestic.
**Effort:** ~1 day

### Line Crossing / Tripwire
**IVE ops:** SAD + CCL + software tracking (CPU)
**Algorithm:** Define a virtual line in the scene. Track motion blob centroids
across frames. When a centroid crosses the line, fire event with direction.
Direction determined by which side the centroid was on in the previous frame.
**Metadata output:**
```json
{"lineCross": {"line": "entrance", "direction": "in", "objectId": 3}}
```
**Value:** People counting in retail, building access monitoring. One of the
most requested CCTV analytics features.
**Effort:** ~2 days

### Zone Intrusion
**IVE ops:** SAD + CCL + And (zone mask)
**Algorithm:** Define forbidden zones as binary bitmaps (one per zone).
After SAD produces the motion binary map, use IVE And to mask it with
each zone bitmap. Any remaining non-zero pixels = intrusion in that zone.
**Metadata output:**
```json
{"intrusion": {"zone": "parking_lot", "regions": [{"bbox": [x,y,w,h]}]}}
```
**Value:** Perimeter security. Majestic has basic ROI filtering but this
would be proper multi-zone alerting with per-zone sensitivity.
**Effort:** ~1 day

## Tier 2: Moderate effort (needs 1-2 new IVE ops or CPU algorithms)

### Loitering Detection
**IVE ops:** SAD + CCL + software trajectory tracking
**Algorithm:** Track motion blobs over time using centroid matching.
If a blob stays within a small area (< N pixels movement) for longer
than a threshold (e.g., 60 seconds), flag as loitering.
**Metadata output:**
```json
{"loitering": {"bbox": [x,y,w,h], "duration": 72, "zone": "ATM_area"}}
```
**Value:** ATM security, retail loss prevention, public safety.
**Effort:** ~3 days (CPU-side tracking logic, no new IVE ops)

### Edge-based Object Classification
**IVE ops:** Sobel + Hist + Thresh + Integ
**Algorithm:** After motion detection finds a blob, extract its bounding box.
Run Sobel on the region to get edge map. Compute histogram of edge
orientations (simplified HOG). Classify by edge distribution:
- Person: dominant vertical edges, aspect ratio > 1.5
- Vehicle: dominant horizontal edges, large area, aspect ratio < 1.0
- Animal: small area, irregular edge distribution
**Metadata output:**
```json
{"objects": [{"type": "person", "bbox": [x,y,w,h], "confidence": 0.7}]}
```
**Value:** Reduces false alarms from animals/vehicles/trees.
**New IVE ops needed:** HOG would help but simplified version works with Sobel+Hist.
**Effort:** ~1 week

### Abandoned Object Detection
**IVE ops:** SAD + CCL + Sub + Thresh + Integ + GMM2 (needs implementation)
**Algorithm:** Build a background model with GMM2 (Gaussian Mixture Model).
Compare current frame against background. Objects that:
1. Are NOT part of the learned background
2. Remain stationary for > N seconds (centroid doesn't move)
3. Were not present in the original background
...are flagged as abandoned.
**Metadata output:**
```json
{"abandonedObject": {"bbox": [x,y,w,h], "duration": 45, "zone": "lobby"}}
```
**Value:** Airport/train station security. Unattended luggage detection
is a regulatory requirement in many countries.
**New IVE ops needed:** GMM2, UpdateBgModel (complex — stateful across frames)
**Effort:** ~2 weeks

### License Plate Region Detection
**IVE ops:** Sobel + Dilate + Erode + CCL + Resize + Integ
**Algorithm:** Find rectangular regions with dense edges:
1. Sobel → edge map
2. Threshold → binary edges
3. Dilate → close gaps between characters
4. Erode → remove noise
5. CCL → find connected blobs
6. Filter by: aspect ratio (~3:1), edge density (> 50%), size range
Not full LPR (needs OCR/NNIE) but detects WHERE the plate is.
**Metadata output:**
```json
{"plateRegion": {"bbox": [x,y,w,h], "confidence": 0.8}}
```
**Value:** Pre-filter for cloud-based LPR, trigger high-res snapshot.
**New IVE ops needed:** Resize (for multi-scale detection)
**Effort:** ~1 week

## Tier 3: Significant effort (complex algorithms or many new IVE ops)

### Optical Flow / Speed Estimation
**IVE ops:** LKOpticalFlowPyr + STCandiCorner + STCorner
**Algorithm:** Detect feature points (Shi-Tomasi corners), track with
Lucas-Kanade optical flow across frames, compute velocity vectors.
Aggregate per-object speeds from tracked feature clusters.
**Metadata output:**
```json
{"flow": {"avgSpeed": 2.3, "direction": 135, "movingObjects": 3}}
```
**Value:** Traffic monitoring (vehicle speed), crowd density estimation.
**New IVE ops needed:** LKOpticalFlowPyr, STCandiCorner, STCorner
**Effort:** ~2 weeks

### Persistent Object Tracking (KCF)
**IVE ops:** KCF (11 functions) + SAD + CCL
**Algorithm:** After detecting an object via motion detection, initialize
a KCF tracker for it. KCF uses correlation filters in the frequency
domain to track the object robustly, even through brief occlusions.
Each tracked object gets a persistent ID across frames.
**Metadata output:**
```json
{"tracks": [{"id": 1, "bbox": [x,y,w,h], "age": 45, "velocity": [2,1]}]}
```
**Value:** Enables counting (how many people entered), trajectory analysis,
and behavioral analytics. Required for most advanced CCTV features.
**New IVE ops needed:** All 11 KCF functions (complex state management)
**Effort:** ~3 weeks

## Implementation Priority Matrix

| Feature | Business Value | IVE Ready | New Ops | Effort | Priority |
|---------|---------------|-----------|---------|--------|----------|
| Tamper detection | High (every camera) | ✓ | None | 1 day | **P0** |
| Line crossing | High (retail/access) | ✓ | None | 2 days | **P0** |
| Zone intrusion | High (security) | ✓ | None | 1 day | **P0** |
| Loitering | Medium (ATM/retail) | ✓ | None | 3 days | **P1** |
| Object classification | Medium (false alarms) | Mostly | HOG helps | 1 week | **P1** |
| Abandoned object | Medium (transport) | Partial | GMM2 | 2 weeks | **P2** |
| Plate detection | Medium (parking) | Partial | Resize | 1 week | **P2** |
| Optical flow | Low-Medium (traffic) | No | LK+ST | 2 weeks | **P3** |
| KCF tracking | Medium (persistent IDs) | No | KCF×11 | 3 weeks | **P3** |

## Metadata Integration

All analytics produce JSON metadata that can be:
- Embedded in RTSP stream as ONVIF-compatible events
- Sent via webhook to external systems (NVR, home automation)
- Stored alongside video recordings for search/playback
- Consumed by MQTT for IoT integration

Compatible NVR software: Milestone, Genetec, ZoneMinder, Frigate,
Shinobi, or any system supporting ONVIF analytics events.

## IVE Operation Coverage

| Application | DMA | SAD | CCL | Sub | Add | And | Thresh | Hist | Sobel | Dilate | Erode | Integ | Map | NCC | GMM2 | LK | KCF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Motion detect | ✓ | ✓ | ✓ | | | | | | | | | | | | | | |
| Tamper detect | | | | ✓ | | | ✓ | ✓ | | | | | | | | | |
| Line crossing | ✓ | ✓ | ✓ | | | | | | | | | | | | | | |
| Zone intrusion | ✓ | ✓ | ✓ | | | ✓ | | | | | | | | | | | |
| Loitering | ✓ | ✓ | ✓ | | | | | | | | | | | | | | |
| Object class. | | | | | | | ✓ | ✓ | ✓ | | | ✓ | | | | | |
| Abandoned obj | ✓ | ✓ | ✓ | ✓ | | | ✓ | | | | | ✓ | | | ✓ | | |
| Plate region | | | ✓ | | | | ✓ | | ✓ | ✓ | ✓ | | | | | | |
| Optical flow | | | | | | | | | | | | | | | | ✓ | |
| KCF tracking | ✓ | ✓ | ✓ | | | | | | | | | | | | | | ✓ |

## References

- HiSilicon IVE API Reference (SDK documentation)
- `docs/ive-registers.md` — hardware register map
- `docs/nnie-vs-npu.md` — NNIE/NPU architecture comparison
- `qemu/hw/misc/hisi-ive.c` — QEMU IVE implementation (17 ops)
- `qemu-boot/test-ive-ops.c` — operation test suite
- Majestic motion detection: `~/git/majestic/src/hisi/mdetect.c`
