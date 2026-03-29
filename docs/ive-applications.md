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
**IVE ops:** Sub + Thresh + Erode + Dilate + SAD + And + CCL (all implemented)
**Algorithm (implemented, 9-step full-hardware pipeline):**
```
Sub(cur,ref) → Thresh → Erode → Dilate → SAD(mask,zero) → blockify
→ SAD(cur,prev) → Thresh(invert) → AND(fg,stationary) → CCL → blob regions
```

1. Learn reference background from first 50 frames (average)
2. Sub(current, reference) → foreground diff
3. Thresh → binary foreground mask
4. Erode → remove noise pixels
5. Dilate → restore object shapes
6. SAD(mask, zero_image) → reduce to block-level (bypasses CCL 720px width limit)
7. SAD(current, previous) → detect inter-frame motion
8. Invert motion mask → stationary blocks
9. AND(foreground, stationary) → stationary foreground only
10. CCL → blob regions with bboxes

CPU only reads the CCL blob struct (~3 KB) per frame — zero pixel scanning.
IVE hardware time: **6.7 ms/frame** at 960×528 (149 fps capacity).

**Evaluated on MEVA + VIRAT Ground Truth (131 events):**
- Recall = 0.67 (detects 2/3 of abandoned events)
- Precision = 0.36 (many false positives)
- F1 = 0.47

**Inherent limitation of background subtraction:**
The algorithm detects "anything that appeared and stayed" — it cannot distinguish
an abandoned bag from a parked car, an opened door, a person sitting down, or
furniture rearranged after the reference was learned. All of these produce a
foreground region that is stationary, triggering a false ABANDONED alert.

This is not a threshold tuning problem. Sweeping diff_thr (30-60), sad_thr (100-300),
and area_thr (4-16) shifts the precision/recall tradeoff but cannot eliminate the
fundamental ambiguity. On a 5-clip sweep:

| diff | sad | area | P | R | F1 |
|------|-----|------|-------|-------|-------|
| 40 | 200 | 4 | 0.60 | 0.75 | 0.67 |
| 50 | 200 | 4 | 0.75 | 0.75 | 0.75 |
| 60 | 200 | 4 | 0.75 | 0.75 | 0.75 |
| 40 | 200 | 16 | 0.67 | 0.50 | 0.57 |
| 60 | 300 | 16 | 0.75 | 0.75 | 0.75 |

To reach production-grade precision (>0.9), abandoned detection needs one of:
- **Owner-object association**: Track the person who placed the object, detect
  when they leave without retrieving it. Requires persistent object tracking
  (KCF or similar) — Tier 3 complexity.
- **Semantic classification**: Use NNIE/NPU to classify the foreground region
  as "bag" vs "person" vs "vehicle". Requires neural network inference.
- **Scene model update**: Adaptive background (GMM2) that learns parked cars
  and furniture into the model over time, so only truly new objects trigger.
  IVE has GMM2 but stateful per-pixel models are complex to implement.

The current IVE-only pipeline is suitable for **alerting with human verification**
(operator reviews flagged events) but not autonomous decision-making.

**Metadata output:**
```json
{"abandonedObject": {"bbox": [x,y,w,h], "duration": 45, "zone": "lobby"}}
```
**Value:** Airport/train station security. Unattended luggage detection
is a regulatory requirement in many countries.
**Effort:** Implemented (basic pipeline). Production-grade: ~2-3 weeks (needs tracking).

### License Plate Region Detection
**IVE ops:** Sobel + 16BitTo8Bit + Thresh + Dilate + Erode + CCL (all implemented)
**Algorithm (implemented, 6-step full-hardware pipeline):**
```
Frame → Sobel(vertical edges) → 16BitTo8Bit(S16→U8 abs) → Thresh → Dilate → Erode → CCL
                                                                                      ↓
                                                        CPU: filter by aspect ratio 2.0-6.0
```

1. Sobel (vertical kernel) → S16C1 horizontal edge response
2. 16BitTo8Bit (S16→U8 absolute value) → edge magnitude
3. Thresh → binary edge map
4. Dilate (5×5) → close gaps between characters, merge into plate blob
5. Erode (5×5) → remove noise, tighten boundaries
6. CCL → blob regions with bounding boxes
7. CPU: filter by aspect ratio (2.0-6.0), minimum area, position (bottom 80%)

IVE hardware time: **9.1 ms/frame** at 960×528 (110 fps capacity).
Not full LPR/OCR — detects WHERE the plate region is.

**Evaluated on CCPD (Chinese City Parking Dataset, 100 images, IoU≥0.3):**

| Parameters | P | R | F1 | FP/image |
|------------|-------|-------|-------|---------|
| diff=40, area=4 (baseline) | 0.007 | 0.260 | 0.014 | ~36 |
| diff=60, area=16 (tuned) | 0.065 | 0.560 | 0.116 | ~8 |

**Inherent limitation of edge-based detection:**
Sobel + morphology + CCL detects *any* rectangular region with dense edges — bumpers,
window frames, signs, text on buildings, and license plates all look identical in edge
space. The aspect ratio filter reduces FP somewhat but cannot provide the selectivity
needed for production use.

When the algorithm does detect a plate (26-56% recall), the spatial accuracy is decent
(median IoU 0.43-0.67). The problem is purely precision — too many false positives.

To reach production-grade plate detection (>50% precision), needs either:
- **Multi-scale + texture analysis**: Histogram of edge orientations within candidate
  regions to distinguish plate characters from other dense-edge textures
- **Neural network**: NNIE/NPU-based plate detector (e.g., YOLO) with IVE as
  preprocessing (Sobel for edge enhancement before NN inference)
- **Motion-gated**: Only run plate detection within motion-detected vehicle bounding
  boxes (IVE SAD→CCL first, then Sobel on cropped region)

**Metadata output:**
```json
{"plateRegion": {"bbox": [x,y,w,h], "ratio": 3.2, "area": 25}}
```
**Value:** Research/prototype only. Not production-ready without NN assistance.
**Effort:** Implemented. Production-grade: needs NNIE/NPU.

## Tier 3: Significant effort (complex algorithms or many new IVE ops)

### Optical Flow / Speed Estimation
**IVE ops:** LKOpticalFlowPyr + STCandiCorner + STCorner (all implemented)
**Algorithm:** Detect feature points (Shi-Tomasi corners), track with
Lucas-Kanade optical flow across frames, compute velocity vectors.
Aggregate per-object speeds from tracked feature clusters.
**Metadata output:**
```json
{"flow": {"avgSpeed": 2.3, "direction": 135, "movingObjects": 3}}
```
**Value:** Traffic monitoring (vehicle speed), crowd density estimation.
**Effort:** Implemented in QEMU. Tested on real EV300: 4/5 points tracked
with correct flow direction on 2px-shift test. LK flow vectors within
±0.016 pixels of HW output (sub-pixel precision difference from HW
fixed-point arithmetic).

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
| Abandoned object | Medium (transport) | ✓ (basic) | Tracking for prod | Done+2w | **P2** |
| Plate detection | Medium (parking) | ✓ | None | Done | **P2** |
| Optical flow | Low-Medium (traffic) | ✓ | None | Done | **P3** |
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

### QEMU Emulation Status (39 ops implemented)

**Byte-identical to real EV300 hardware (22 ops, verified via full pixel capture):**

| Category | Ops | Pixels verified |
|----------|-----|----------------|
| Pixel-wise | Sub, Add, And, Or, Xor | 4096/4096 each |
| Threshold | Thresh, Thresh_S16, Thresh_U16, 16BitTo8Bit | 4096/4096 each |
| Convolution | Filter, Sobel (S16C1 output) | 4096/4096 each |
| Morphology | Dilate (bitwise OR), Erode (bitwise AND), OrdStatFilter | 4096/4096 each |
| Edge | CannyEdge (5×5 mask + NMS + hysteresis), MagAndAng, NormGrad | 3968-4096 each |
| Histogram | Hist, EqualizeHist | 256/256 bins, 4096/4096 px |
| Analysis | Integ (COMBINE: sqsum<<28\|sum), Map | 4096/4096 each |
| Background | GMM2 (30-frame stateful test) | 256/256 fg pixels |

**Functionally correct, sub-pixel precision gap (2 ops):**

| Op | Gap | Detail |
|----|-----|--------|
| STCandiCorner | Eigenvalue scaling | HW internal fixed-point normalization unknown; 46 corners detected correctly |
| LKOpticalFlowPyr | ±0.016 px flow | HW fixed-point matrix solve rounding; 4/5 points tracked with correct direction |

**NOT_SUPPORT on EV300 (8 ops — API exists but HW doesn't implement):**
LBP, Resize, NCC, CSC, GradFg, GMM(v1), FilterAndCSC, MatchBgModel

**Not in SDK library (2 ops — headers only, no libive.so implementation):**
PerspTrans, Hog

### ML Inference Subsystem (CPU-only, in libive.so)

EV300 has **both** IVE hardware and NNIE (0.5 TOPS) — these are completely
separate subsystems. The IVE ML functions below run entirely on the CPU via
`libive.so` internal `mpi_ive_xnn_*` functions. They do NOT use the NNIE
hardware accelerator (`/dev/nnie`, separate `HI_MPI_SVP_NNIE_*` API).

| Function group | Functions | Status on EV300 | Error code |
|---------------|-----------|----------------|------------|
| ANN_MLP | LoadModel, UnloadModel, Predict | **Callable** (CPU) | 0xa01d8006 for NULL file |
| SVM | LoadModel, UnloadModel, Predict | **Callable** (CPU) | 0xa01d8006 for NULL file |
| CNN | LoadModel, UnloadModel, Predict, GetResult | **Callable** (CPU) | 0xa01d8006 for NULL file |
| KCF | GetMemSize, CreateObjList, etc. (10 functions) | **Not in libive.so** | Linker error |

- ANN_MLP: Fixed-point MLP, up to 8 layers, 1024D input, sigmoid/gaussian activation
- SVM: C-SVC/NU-SVC with Linear/RBF/Poly/Sigmoid kernels
- CNN: Conv3×3 → ReLU → Pool2×2 pipeline, up to 8 conv layers + FC
- KCF: Declared in headers but never implemented on EV200/EV300

**QEMU implications:** No changes needed to `hisi-ive.c`. These functions
execute natively on the emulated ARM CPU via `libive.so`. The IVE hardware
device (`/dev/ive`) is only used for the 39 pixel/image ops above.

### Key reverse-engineering discoveries

- **Dilate/Erode**: NOT standard max/min morphology. Uses bitwise OR/AND:
  `dilate = OR(src[i] & mask[i])`, `erode = AND(src[i] | ~mask[i])`
- **Canny NMS**: Asymmetric comparison `(m > n1 && m >= n2)` with boundary
  clamping where n1 is the "backward" neighbor
- **Sobel**: Outputs S16C1 (not U8 magnitude), uses configurable 5×5 mask
  with clamped border access for rows 1..H-2
- **Integ COMBINE**: U64 = `(sqsum << 28) | sum` — 28-bit sum + 36-bit sqsum

### Application coverage

| Application | DMA | SAD | CCL | Sub | Add | And | Thresh | Hist | Sobel | Dilate | Erode | Integ | Map | GMM2 | LK | ST |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| Motion detect | ✓ | ✓ | ✓ | | | | | | | | | | | | | |
| Tamper detect | | | | ✓ | | | ✓ | ✓ | | | | | | | | |
| Line crossing | ✓ | ✓ | ✓ | | | | | | | | | | | | | |
| Zone intrusion | ✓ | ✓ | ✓ | | | ✓ | | | | | | | | | | |
| Loitering | ✓ | ✓ | ✓ | | | | | | | | | | | | | |
| Object class. | | | | | | | ✓ | ✓ | ✓ | | | ✓ | | | | |
| Abandoned obj | ✓ | ✓ | ✓ | ✓ | | ✓ | ✓ | | | ✓ | ✓ | | | | | |
| Plate region | | | ✓ | | | | ✓ | | ✓ | ✓ | ✓ | | | | | |
| Optical flow | | | | | | | | | | | | | | | ✓ | ✓ |
| KCF tracking | ✓ | ✓ | ✓ | | | | | | | | | | | | | |

### Real hardware test coverage

```
QEMU register-level test (test-ive-ops.c):  19/19 pass
Real board MPI test (test-ive-mpi.c):       34/34 pass
  - 24 pixel/image ops verified on real EV300 silicon
  - 3 ML functions tested (ANN_MLP, SVM, CNN — CPU-only, callable)
  - 8 ops NOT_SUPPORT (API exists, HW doesn't implement)
  - 3 N/A (PerspTrans, Hog, KCF — not in libive.so)
```

## References

- HiSilicon IVE API Reference (SDK documentation)
- `docs/ive-registers.md` — hardware register map
- `docs/ive-eval-methodology.md` — evaluation pipeline, metrics, reproducing results
- `docs/meva-dataset-spec.md` — dataset download and indexing spec
- `qemu/hw/misc/hisi-ive.c` — QEMU IVE implementation (39 ops, 22 byte-identical)
- `qemu-boot/test-ive-ops.c` — QEMU register-level test suite (19 tests)
- `qemu-boot/test-ive-mpi.c` — real board MPI test suite (34 tests, incl. ML inference)
- `qemu-boot/test-ive-video-mpi.c` — real board video pipeline (MD + abandoned + LPR)
- `qemu-boot/test-ive-bytecmp.c` — byte-exact capture for differential testing
- `qemu-boot/test-canny-capture.c` — Canny byte-identical verification tool
- Majestic motion detection: `~/git/majestic/src/hisi/mdetect.c`
- MEVA dataset: [mevadata.org](https://mevadata.org/) (CC BY 4.0)
- VIRAT Ground 2.0: [viratdata.org](https://viratdata.org/)
- CCPD dataset: [GitHub](https://github.com/detectRecog/CCPD) (MIT license)
