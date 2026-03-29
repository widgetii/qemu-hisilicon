# HiSilicon Neural Network Accelerator Architecture: NNIE vs SVP_NPU

Research findings from SDK analysis, vendor documentation, kernel drivers,
and public sources. Last updated: 2026-03-27.

## Official Taxonomy

HiSilicon defines 4 generations of "Image Analysis Engines"
(source: `Hi3516CV610R001C01SPC020/ReleaseDoc/en/01.software/pc/SVP_NPU/Image Analysis Engine Differences.pdf`):

| Engine | API | Chips | HW Block |
|--------|-----|-------|----------|
| Engine 1 | ACL (Ascend) | SD3403V100 | NPU |
| Engine 2 | SVP ACL | SD3403V100 SVP, Hi3536AV100 | SVP_NPU |
| Engine 3 | SVP ACL | Hi3519DV500, Hi3516DV500 | SVP_NPU |
| Engine 4 | SVP ACL | Hi3516CV610, Hi3516CV608 | SVP_NPU |

These are all **post-NNIE** designs. The older NNIE (Hi3516CV500, EV300, 3519V101)
is a completely separate architecture not covered by the engine numbering.

## NNIE (Neural Network Inference Engine)

### Chips
Hi3516CV500, Hi3516EV300/EV200, Hi3516DV200, Hi3519V101,
Goke GK7205V200/V300, GK7202V300, GK7605V100

### Architecture
- Dedicated fixed-function CNN accelerator (not programmable)
- Single core per chip (`SVP_NNIE_ID_0`)
- Caffe-only framework support
- Model format: `.wk` (compiled by `nnie_mapper`)
- Hardware register space: ~64 KB
- INT8/INT16 quantization, per-layer (not per-channel)
- DDR-only memory access (no on-chip NNIE SRAM)

### Network Types
```c
SVP_NNIE_NET_TYPE_CNN       = 0x0   // Standard CNN (no ROI)
SVP_NNIE_NET_TYPE_ROI       = 0x1   // With ROI/PSROI Pooling
SVP_NNIE_NET_TYPE_RECURRENT = 0x2   // RNN or LSTM
```

### Constraints
- Max 8 segments per network
- Max 16 input/output nodes per segment
- Max 4 ROI pooling layers total
- Node name length: 32 chars

### Supported Operators
- Convolution (N×N kernels)
- Pooling (Max, Average)
- Activation (ReLU, Sigmoid, TanH)
- ROI Pooling, PSROI Pooling
- Reshape (first dim must be 0)
- Slice (C/H/W only, not batch)
- LSTM, RNN
- Fully Connected

### Known Limitations
- No depthwise convolution
- Only 2×2 nearest-neighbor upsample
- Batch normalization must be fused into convolution
- No Transformer / attention support
- No dynamic shapes
- Per-layer quantization only (not per-channel)
- Unsupported layers require network segmentation (NNIE→CPU→NNIE)

### Programming Model
```
hi_mpi_svp_nnie_load_model()
  → hi_mpi_svp_nnie_get_tsk_buf_size()
  → hi_mpi_svp_nnie_add_tsk_buf()
  → hi_mpi_svp_nnie_forward() / hi_mpi_svp_nnie_forward_with_bbox()
  → hi_mpi_svp_nnie_query()           // poll for completion
  → hi_mpi_svp_nnie_unload_model()
```

### Performance
| Chip | TOPS | Notes |
|------|------|-------|
| Hi3516EV200 | ~0.5 | Economy |
| Hi3516EV300 | ~0.5 | Standard |
| Hi3516CV500 | ~0.5 | Mid-range |
| Hi3516DV200 | ~0.5 | Higher res |
| Hi3519V101 | ~1.0 | With A17 big core |

### Model Compilation Pipeline
```
PyTorch/TF → export to Caffe (.prototxt + .caffemodel)
  → nnie_mapper (quantize + compile, needs calibration images)
  → .wk file
  → deploy to board
```

### Tested Networks (from SDK samples)
RFCN, Segnet, FasterRCNN (Alexnet, VGG16), SSD, YOLOv1/v2/v3,
LSTM, PVANET. Community has also run YOLOv5.

### Key SDK Files (CV500)
```
smp/a7_linux/mpp/include/hi_nnie.h          # NNIE data structures
smp/a7_linux/mpp/include/mpi_nnie.h         # NNIE API declarations
smp/a7_linux/mpp/include/hi_comm_svp.h      # SVP common types (blobs, memory)
smp/a7_linux/mpp/ko/hi3516cv500_nnie.ko     # Kernel module
smp/a7_linux/mpp/init/nnie_init.c           # Module init source
smp/a7_linux/mpp/sample/svp/nnie/           # 9 sample implementations
```

## SVP_NPU (Smart Vision Platform NPU)

### Chips
Hi3516CV610/CV608/CV613, Hi3516DV500, Hi3519DV500,
SD3403V100, Hi3536AV100

### Architecture
- Programmable graph-execution neural processing unit
- Derived from Huawei Ascend AI stack
- Multi-framework: Caffe + ONNX natively (TF/PyTorch via ONNX export)
- Model format: `.om` (compiled by `svp_atc`, Automatic Tensor Compiler)
- Hardware register space: 8 MB at 0x14000000 (CV610)
- INT4/INT8 quantization, per-layer + automatic mixed precision
- Transformer feature acceleration (Engine 1+)

### Key Differences from NNIE

| Feature | NNIE | SVP_NPU |
|---------|------|---------|
| Type | Fixed-function CNN | Programmable graph engine |
| Framework | Caffe only | Caffe + ONNX (+TF/PyTorch) |
| Model compiler | `nnie_mapper` | `svp_atc` (graph optimizer) |
| Model format | `.wk` | `.om` |
| Quantization | INT8/INT16 per-layer | INT4/INT8 mixed precision |
| Dynamic batch | No | Yes |
| Dynamic image size | No | Yes (Engine 3,4) |
| Custom operators | No | Yes |
| Transformer | No | Yes |
| Task preemption | No | Yes (Engine 3,4) |
| Async events | No | Yes (Engine 3,4) |
| AIPP preprocessing | Static only | Static + Dynamic |
| Weight compression | No | Yes (Engine 1,3) |
| Data formats | NCHW only | NCHW + NHWC |
| Priority config | No | Yes (Engine 3,4) |
| Quant tools | Built into nnie_mapper | Separate AMCT (Caffe+PyTorch) |

### Programming Model (SVP ACL)
```
svp_acl_init(config_path)
  → svp_acl_rt_set_device(0)
  → svp_acl_mdl_load_from_mem(model_data, size, &model_id)
  → svp_acl_mdl_execute(model_id, input_dataset, output_dataset)
     // or svp_acl_mdl_execute_async() with stream
  → svp_acl_mdl_unload(model_id)
  → svp_acl_finalize()
```

### Performance
| Chip | TOPS | Notes |
|------|------|-------|
| Hi3516CV608 | 0.2 | Consumer |
| Hi3516CV610 (10B) | 0.5 | Entry professional |
| Hi3516CV613 (20S) | 1.0 | Professional |
| Hi3516CV610 (00S/20G/00G) | 1.0 | Higher models |
| Hi3516DV500 | TBD | |
| Hi3519DV500 | TBD | |

### Model Compilation Pipeline
```
PyTorch/TF → export to ONNX (.onnx) or Caffe
  → AMCT quantization (optional: INT4/INT8/mixed, calibration or QAT)
  → svp_atc compilation (graph optimization, operator fusion)
  → .om file
  → deploy to board
```

### Evolution Across SVP_NPU Engines

| Feature | Engine 2 (SD3403) | Engine 3 (DV500) | Engine 4 (CV610) |
|---------|-------------------|------------------|------------------|
| Dynamic AIPP | No | Yes | Yes |
| Dynamic image size | No | Yes | Yes |
| Task preemption | No | Yes | Yes |
| Event sync | No | Yes | Yes |
| PyTorch AMCT | 1.11 | 1.11 | **2.1.0** |
| Mixed precision | INT4+INT8 | INT4+INT8 | INT4+INT8 + **auto search** |
| Quant error analysis | No | No | **Yes** (QuantAnalyzer) |
| Weight compression | No | Yes | No (removed) |
| Custom operators | Full | Same | Custom AI CPUs only |

### Key SDK Files (CV610)
```
ReleaseDoc/en/01.software/pc/SVP_NPU/
  Application Development Guide.pdf      # Runtime API reference
  ATC Instructions.pdf                    # Model compiler usage
  ATC Graph Development Guide.pdf         # Graph-level programming
  ATC Custom Operator Developer Guide.pdf # Custom ops
  AMCT Instructions (PyTorch).pdf         # Quantization (PyTorch)
  AMCT Instructions (Caffe).pdf           # Quantization (Caffe)
  Image Analysis Engine Differences.pdf   # Engine 1-4 comparison
  Quick Start Guide.pdf                   # Getting started
```

## IVE (Intelligent Video Engine)

Present alongside both NNIE and NPU in all generations.
Fixed-function accelerator for traditional computer vision:

- Motion detection (GMM/GMM2)
- Edge detection (Canny, Sobel)
- Optical flow
- Histogram, threshold, morphology, dilate/erode
- Object tracking
- Background subtraction

### Register Space
| Generation | Address | Size |
|------------|---------|------|
| V4 (EV300, Goke) | ~0x11320000 | 64 KB |
| V5 (CV610) | 0x14000000 | Shared with NPU |

### IVE Versions
- IVE 1.0: V3.5/V4 (basic ops)
- IVE 2.5: V5/CV610 (added: perimeter protection, perspective transform,
  video diagnosis, multiple intelligent analysis)

### IVE XNN — Hidden CNN Accelerator Inside IVE

The IVE hardware on EV200/EV300 contains a **built-in CNN inference engine** (XNN)
that is separate from NNIE. This is not documented in public API headers — the
`mpi_ive_xnn_*` functions are private symbols in the static `libive.a`, accessed
via ioctl on the same `/dev/ive` device.

**Discovery source:** `/proc/umap/ive` on EV300 lists `XNN` as the last instruction
type in the invoke counters. The kernel module `hi3516ev200_ive.ko` contains the
full XNN layer parser and execution engine.

#### Hardware-Accelerated Layer Types

| Layer | Key Parameters | Purpose |
|-------|---------------|---------|
| Conv | kernel_size, input_c/h/w, output_c/h/w, pool_mode, af_mode, pad | Convolution + fused pooling + activation |
| FC | input_w, output_w, batch_num, af_mode | Fully connected |
| Eltwise | eltwise_op, in_num, af_mode | Element-wise ops (residual add/mul) |
| Flatten | input_c/h/w | Reshape tensor for FC input |
| Preproc | (VGS-based) | Input resize + color space conversion |
| DMA | | Inter-layer data movement |
| Unpack | | Data format conversion |

- `af_mode` = activation function (ReLU, etc.)
- `pool_mode` = pooling type (max, average), fused into conv layer
- `in_fmt`/`out_fmt` = quantization format
- Conv `kernel_size` supports at least two sizes (error: "must be {%d, %d}")
- Eltwise supports residual connections (`in_num` up to some max)

#### XNN API (Private, in static libive.a)

```
mpi_ive_xnn_loadmodel(model_data, ...)     → parse OMS, allocate HW resources
mpi_ive_xnn_preproc(input_frame, ...)      → VGS-based input preprocessing
mpi_ive_xnn_forward(model, input, output)  → run full network inference in HW
mpi_ive_xnn_forward_slice(...)             → sliced execution for large inputs
mpi_ive_xnn_unloadmodel(...)               → free resources
mpi_ive_xnn_get_tmpbuf_size(...)           → query temp buffer requirements
```

These are called by `libivp.a` (Intelligent Video Processing) which provides
the user-facing `hi_ivp_process_ex()` API.

#### OMS Model Format (Reverse-Engineered Structure)

The `.oms` files in `mpp/sample/ivp/res/` are the XNN model format.
Analyzed: `ivp_re_allday_f1y2f2m1_640x360_v1003.oms` (1010 KB).

**File layout:**
```
0x000-0x01F   Global header (magic, total size, segment count)
0x020-0x06F   Segment descriptors (offsets, sizes, checksums)
0x070-0x07F   Build timestamp: "1904291011231421" (2019-04-29)
0x080-0x09F   Input descriptor (dimensions, strides, format)
0x0A0-0x0CF   Network I/O names (32-byte padded): "data", "layer69-conv_report0"
0x0F0-0x926FF Per-layer binary descriptors + quantized weights (586 KB)
0x92700       Layer name table: 46 entries × 0x40 bytes (network 1)
0x93290       Second network weights + descriptors (421 KB)
0xFC5C0       Layer name table: 9 entries × 0x40 bytes (network 2)
```

Layer name table entry format: 32-byte name (null-padded) + 32-byte metadata
(layer index at offset +0x30).

**Input dimensions** at offsets 0xF6-0xF9: height=360, width=640 (LE uint16).

#### Two-Stage Detection Architecture in OMS

The model contains two cascaded networks:

**Network 1 — YOLO detection backbone (586 KB, 46 conv layers):**
```
Input: 640×360 grayscale
layer1-conv → layer2-conv → layer4-conv → ... → layer69-conv
                                                       ↓
                                              layer69-conv_report0 (proposals)
```
- Layers numbered 1-69 with gaps (3,6,9,10,13,14...) = implicit BatchNorm+ReLU
- Architecture: compact YOLO variant (confirmed by `svp_alg_yoloonet_process` symbol)
- Post-processing: `svp_alg_rpn_process()` filters proposals

**Network 2 — Classification refinement head (421 KB, 9 layers):**
```
Input: cropped proposals from network 1
conv1 → conv2 → conv3 → conv4 → conv4_up → fc5_flatten → fc5 → fc6_det_score
```
- 4 conv + 1 deconv (upsample) + flatten + 2 FC layers
- Output: `fc6_det_score` = per-detection confidence
- Two-stage pattern: backbone proposes, head refines (like Faster R-CNN)

#### IVP Call Chain (How It All Connects)

```
hi_ivp_process_ex()                         [libivp.a — user API]
  → ivp_process()
    → mpi_ive_xnn_preproc()                 [libive.a — VGS resize/CSC]
    → mpi_ive_xnn_forward()                 [libive.a — HW CNN inference]
      → ioctl(/dev/ive, XNN_FORWARD_CMD)    [kernel: ive_xnn_fill_forward_node_info]
        → IVE hardware XNN engine           [silicon: conv/fc/eltwise/flatten]
    → svp_alg_yoloonet_process()            [libivp.a — YOLO decode, CPU]
    → svp_alg_rpn_process()                 [libivp.a — NMS/filtering, CPU]
  → ivp_get_obj_array()                     [libivp.a — extract bboxes]
```

#### Implications

1. **EV200/EV300 can run CNNs without NNIE** — the IVE XNN engine is a separate
   CNN accelerator embedded in the IVE block. The "0.5 TOPS" attributed to NNIE
   on these chips may actually be the IVE XNN engine (there is no separate NNIE
   kernel module — only `hi3516ev200_ive.ko` exists).

2. **The XNN API is private but usable** — symbols are in `libive.a` (static lib).
   Custom models could be loaded via `mpi_ive_xnn_loadmodel()` if the OMS format
   is fully reverse-engineered.

3. **IVP is a thin wrapper** — the "Intelligent Video Processing" framework just
   orchestrates XNN inference + YOLO post-processing + ISP/VENC integration.
   The actual HW acceleration is all IVE XNN.

4. **The vendor QR library (`libqr.a`)** also uses `HI_MPI_SYS_Mmap` and
   `detect_proc` — it likely runs a small CNN via IVE XNN for QR finder
   pattern detection, not just classical image processing.

## QEMU Emulation Status

| Block | Address (example) | QEMU Device | Functional |
|-------|-------------------|-------------|------------|
| NNIE (V4) | 0x11320000 | `hisi-ive` regbank | Stub only |
| NPU (V5) | 0x14000000 | `hisi-npu` regbank | Stub only |

Kernel modules load but inference operations are no-ops.
Register reads return 0 (regbank default).

## References

### SDK Sources
- NNIE headers: `Hi3516CV500_SDK_V2.0.2.1/smp/a7_linux/mpp/include/hi_nnie.h`
- NNIE samples: `Hi3516CV500_SDK_V2.0.2.1/smp/a7_linux/mpp/sample/svp/nnie/`
- NPU docs: `Hi3516CV610R001C01SPC020/ReleaseDoc/en/01.software/pc/SVP_NPU/`
- Engine diffs: `Image Analysis Engine Differences.pdf` (2025-03-28)
- IVE XNN symbols: `Hi3516EV200_SDK_V1.0.1.2/mpp/lib/libive.a` (static, private API)
- IVP framework: `Hi3516EV200_SDK_V1.0.1.2/mpp/lib/libivp.a` + `mpp/include/hi_ivp.h`
- IVP sample: `Hi3516EV200_SDK_V1.0.1.2/mpp/sample/ivp/sample_ivp.c`
- IVP models: `mpp/sample/ivp/res/ivp_re_allday_f1y2f2m1_{640x360,360x640}_v1003.oms`
- IVE QR sample: `Hi3516EV200_SDK_V1.0.1.2/mpp/sample/ive/sample/sample_ive_qr.c`
- QR library: `Hi3516EV200_SDK_V1.0.1.2/mpp/lib/libqr.a` (static, closed-source)
- IVE kernel module: `Hi3516EV200_SDK_V1.0.1.2/mpp/ko/hi3516ev200_ive.ko`

### Public Sources
- [NNIE Development Notes - Huawei Cloud](https://bbs.huaweicloud.com/blogs/395170)
- [YOLOv5-NNIE on GitHub](https://github.com/tevcam2020/yolov5-nnie)
- [NNIE Quantization Aware Training](https://github.com/aovoc/nnieqat-pytorch)
- [MindSpore Lite NNIE Backend](https://www.mindspore.cn/lite/docs/en/r1.8/use/nnie.html)
- [MQBench NNIE Backend](https://mqbench.readthedocs.io/en/latest/user_guide/hardware/nnie.html)
- [HiSilicon SVP Platform](https://www.hisilicon.com/en/products/developer-platform/perceptual-computing)
