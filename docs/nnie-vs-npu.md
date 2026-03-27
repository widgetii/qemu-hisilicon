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

### Public Sources
- [NNIE Development Notes - Huawei Cloud](https://bbs.huaweicloud.com/blogs/395170)
- [YOLOv5-NNIE on GitHub](https://github.com/tevcam2020/yolov5-nnie)
- [NNIE Quantization Aware Training](https://github.com/aovoc/nnieqat-pytorch)
- [MindSpore Lite NNIE Backend](https://www.mindspore.cn/lite/docs/en/r1.8/use/nnie.html)
- [MQBench NNIE Backend](https://mqbench.readthedocs.io/en/latest/user_guide/hardware/nnie.html)
- [HiSilicon SVP Platform](https://www.hisilicon.com/en/products/developer-platform/perceptual-computing)
