# XNN Ioctl RE Specification

Spec for reverse-engineering the IVE XNN ioctl handlers from the kernel module
binary to understand exact buffer layouts, validation logic, and execution flow.

## Source Material

Kernel module: `~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp/ko/hi3516ev200_ive.ko`

This single .ko serves all `/dev/ive` requests — both standard IVE pixel ops
(Thresh, Sobel, CCL) and XNN CNN inference (loadmodel, forward, forward_slice).

Toolchain for disassembly: `arm-none-eabi-objdump -d` or Ghidra/IDA.

## Known Ioctl Commands (from userspace RE)

| Command | Code | Dir | Size | Nr | Handler to find |
|---------|------|-----|------|----|----------------|
| LOADMODEL | `0xc8a04636` | R/W | 2208 | 0x36 | `ive_xnn_parse_model` |
| FORWARD | `0xc620463a` | R/W | 1568 | 0x3a | `ive_xnn_fill_forward_node_info` |
| FORWARD_SLICE | `0xc9704638` | R/W | 2416 | 0x38 | `ive_xnn_slice_process` |
| SVP_INIT | `0x8010463b` | R | 16 | 0x3b | maps HW registers |
| QUERY | `0x463c` | - | 0 | 0x3c | completion check |
| FINISH | `0x463d` | - | 0 | 0x3d | sync/finalize |
| OPEN_DEV | `0x801046c8` | R | 16 | 0xc8 | context init |
| CLOSE | `0x46c9` | - | 0 | 0xc9 | sync/reset |
| ENABLE | `0x46ca` | - | 0 | 0xca | arm engine |

All use ioctl magic `'F'` (0x46).

## What to Extract from the .ko

### 1. Ioctl Dispatch Table

Find the `ioctl` or `unlocked_ioctl` handler registered in `file_operations`.
It will contain a switch/jump table keyed on the ioctl command number (nr byte).
Map each nr (0x36, 0x38, 0x3a, 0x3b, 0x3c, 0x3d, 0xc8, 0xc9, 0xca) to its
handler function address.

**Strings to search for**: `ive_xnn_`, `ive_ioctl`, `IVE_XNN`, `file_operations`.

### 2. LOADMODEL Handler (nr=0x36, size=2208)

Find the handler for ioctl `0xc8a04636`. It receives a 2208-byte user buffer
via `copy_from_user` / `copy_to_user`.

**Extract**:
- The exact struct layout the kernel expects in the 2208-byte buffer
- Which fields are input (read by kernel) vs output (written by kernel)
- The buffer layout: we know `[0..23]=model_mem, [24..47]=tmp_mem, [48..2207]=model_params`
  but need field-level detail within model_params (2160 bytes)
- What `ive_xnn_parse_model` does: how it reads the OMS binary data,
  what it validates, what it writes back into model_params
- The model_params output struct — specifically:
  - Where are the output layer descriptors stored?
  - What is the output tensor dimensions/format?
  - How many output layers does the model declare?

**Key function**: `ive_xnn_parse_model` — called from the loadmodel ioctl handler.
It parses the OMS binary in the model_mem buffer and fills model_params.

### 3. FORWARD_SLICE Handler (nr=0x38, size=2416)

This is the **critical path** — the actual CNN inference ioctl.

**Extract**:
- The exact 2416-byte buffer layout: which offsets contain src blobs, dst blobs,
  model reference, control params, etc.
- How the handler maps the userspace blob descriptors (phys/virt addresses)
  to hardware DMA descriptors
- The "preproc ctrl" struct that `ive_xnn_check_forward_ds_comm` validates at
  line ~1385 (from error strings). What fields does it check? What are valid values?
- The report/output blob: how does the kernel know where to write detection results?
  Does it read the output address from model_params, from the dst_blob, or from
  the report_blob argument?
- The slice logic: `ive_xnn_handle_slice` — how does the kernel tile the input
  for large networks? What determines slice boundaries?

**Key functions in .ko** (from strings):
```
ive_xnn_fill_forward_node_info
ive_xnn_slice_process
ive_xnn_handle_slice
ive_xnn_handle_pic
ive_xnn_handle_roi
ive_xnn_pre_dst_data
ive_xnn_begin_vgs_job
ive_xnn_dma_layer_case
ive_xnn_layer_conv_case
ive_xnn_layer_fc_case
ive_xnn_layer_eltwise_case
ive_xnn_layer_flatten_case
ive_xnn_layer_preproc_case
ive_xnn_layer_unpack_case
```

### 4. Conv Layer Execution (`ive_xnn_layer_conv_case`)

**Extract** the conv layer descriptor struct:
- `kernel_size` — what sizes are supported? (error says "must be {%d, %d}")
- `input_c`, `output_c` — channel counts
- `input_h`, `input_w`, `output_h`, `output_w` — spatial dims
- `pool_mode` — fused pooling types
- `af_mode` — activation function types (ReLU, etc.)
- `is_pad` — padding flag
- `in_fmt`, `out_fmt` — quantization format enum values
- `in_stride_c`, `out_stride_c` — channel stride
- `arg_offset`, `arg_len` — weight data location within model buffer

**Also extract**: `ive_xnn_parse_conv` — how conv layer descriptors are read
from the OMS binary during model loading.

### 5. FC Layer (`ive_xnn_layer_fc_case`)

**Extract**:
- `input_w`, `output_w` — dimensions
- `batch_num` — must be specific value
- `af_mode` — activation
- `is_bott_from_usr` — "must be {%d, %d}"
- `in_fmt`, `out_fmt` — format enums

### 6. Output Tensor Handling

The key missing piece: how does the kernel write CNN output back to userspace?

**Find in the .ko**:
- After the last layer executes, where does the output go?
- Does `ive_xnn_pre_dst_data` set up the output DMA?
- Is the output written to `dst_blob.phys_addr`, `report_blob.phys_addr`,
  or somewhere inside `tmp_mem`?
- What format is the raw output? (float, int8, int16?)

### 7. Preprocessing (`ive_xnn_layer_preproc_case`)

**Extract**:
- What does the preproc layer do? (resize, CSC, normalize?)
- The preproc control struct layout — what fields does
  `ive_xnn_check_forward_ds_comm` validate?
- VGS vs CPU preproc paths — what flags select each?

## Approach

1. Load `hi3516ev200_ive.ko` into Ghidra or disassemble with objdump
2. Find the ioctl dispatch function (search for the magic/nr constants)
3. For each handler: trace `copy_from_user` calls to map buffer offsets to struct fields
4. For each handler: trace `copy_to_user` calls to find what kernel writes back
5. Document each struct with field offsets, types, and valid ranges
6. Pay special attention to the forward_slice handler — trace from ioctl entry
   through layer execution to output writeback

## Already Known (from userspace RE)

### Blob struct (48 bytes) — confirmed from LD_PRELOAD capture
```
+0x00: u32 blob_type   (2=YUV420SP)
+0x04: u32 width
+0x08: u32 virt_addr
+0x0C: u32 reserved
+0x10: u64 phys_addr
+0x18: u32 num         (batch, usually 1)
+0x1C: u32 reserved
+0x20: u32 stride
+0x24: u32 height
+0x28: u32 channels
+0x2C: u32 pad
```

### Model params (2160 bytes) — partial, from loadmodel output
```
+0x00: u32 ??? (0 after load)
+0x04: u32 tmp_buf_size
+0x08: u32 src_num
+0x0C: u32 flags
+0x10: u32 blob_type
+0x14: u32 input_w
+0x18: u32 input_h
+0x1C: u32 input_c
+0x20: u32 reserved
+0x24: char[32] input_name ("data")
+0x44: u32 output_layer_info_start
+0x48..0x86F: layer descriptors + output node info (UNKNOWN)
```

### Forward_slice arguments (8 total)
```
r0: int32_t *handle
r1: blob_t *src_blobs
r2: blob_t *dst_blobs
r3: model_params_t *model
[sp+0]: mem_t *tmp_buf       (24B: phys+virt+size)
[sp+4]: blob_t *report_blob  (48B: output detection tensor)
[sp+8]: void *ctrl           (128B: forward control — NEEDS RE)
[sp+12]: int instant         (1=sync)
```

## Findings from Decompiled Kernel Source

Reference: `/tmp/ive.c` (Ghidra decompilation of hi3516cv500_ive, 17K lines)
Spec update: `/tmp/xnn-ioctl-trace-spec.md` (603 lines, complete RE results)

### Key Answers

1. **ctrl struct**: Just `{u32 src_num, u32 dst_num}` — 8 bytes.
   Kernel derives everything else from model_ctx loaded during LOADMODEL.

2. **Output location**: dst_blobs_out at forward_slice buffer +0x640.
   Physical addresses computed as `tmp_buf_phys + output_offset_table[dst_idx]`.
   The output_offset_table is at model_ctx+0x3A0 (kernel memory only).

3. **Output format**: Always IVE_BLOB_TYPE_S8 (signed 8-bit, enum value 7).
   Stride = width aligned to 16 bytes. Dimensions from model's dst_node array.

4. **The gap**: model_params (2160B returned to userspace) does NOT contain
   dst_node descriptors or output_offset_table. Those stay in kernel's model_ctx
   (1080B per model at `g_ive_xnn_ctx + model_id * 0x438`).
   The vendor's `svp_alg_load_model` reads these via additional queries.

### FORWARD_SLICE Buffer Layout (confirmed, 2416 bytes)

```
+0x000..0x007: reserved/handle
+0x008..0x307: src_blobs[]         — IVE_BLOB_S[16] (input)
+0x308..0x337: roi_info[]
+0x338:        u32 model_id
+0x340..0x63F: dst_blobs_template  — IVE_BLOB_S[16] (preproc fills from this)
+0x640..0x93F: dst_blobs_out[]     — IVE_BLOB_S[16] (actual output destination)
+0x940:        u64 tmp_buf_phys
+0x948:        u64 report_buf_phys (VGS path virtual addr base)
+0x950:        u32 tmp_buf_size
+0x958:        ctrl {u32 src_num, u32 dst_num}
+0x960:        u32 has_roi         (0=picture, 1=ROI)
+0x968:        u32 is_instant      (1=sync)
```

### IVE_BLOB_S Corrected Layout (48 bytes)

```c
typedef struct {
    u32 enType;        // +0x00: IVE_BLOB_TYPE_E
    u32 u32Stride;     // +0x04: row stride (aligned to 16)
    u64 u64VirAddr;    // +0x08: virtual address
    u64 u64PhyAddr;    // +0x10: physical address
    u32 u32Num;        // +0x18: batch count [1,32]
    u32 u32Width;      // +0x1C: width
    u32 u32Height;     // +0x20: height
    u32 u32Chn;        // +0x24: channels
    u32 pad[2];        // +0x28: alignment padding
} IVE_BLOB_S;          // 0x30 = 48 bytes
```

### Remaining Work

To make direct forward_slice work, we need the output blob descriptors
(dst_blobs_out at +0x640) pre-filled with correct phys addresses pointing
into tmp_buf. The kernel's `ive_xnn_pre_dst_data` does this, but only
from model_ctx data that's not returned to userspace.

**Two approaches**:
a) Capture the complete 2416-byte ioctl buffer from a working IVP call
   (via our LD_PRELOAD hook) and replay it with our own frame data
b) RE the `svp_alg_load_model` in libivp.a to find how it queries
   dst_node info and output offsets from the loaded model

## OMS Model Format (fully decoded from ive_xnn_loop_parse_every_layer)

### Outer File Structure
```
+0x00..0x4F: OMS outer header (80 bytes)
  +0x04: u32 total_file_size
  +0x24: u32 payload_size (file_size - 0x20)
  +0x38: u32 tmp_buf_size_hint
  +0x48: u32 segment2_offset (from file start)
+0x50: Segment 1 start
+0x50+seg2_off: Segment 2 start (if multi-network model)
```

### Segment Header
```
+0x00: u32 crc32            — ~CRC32 of bytes [4..end]
+0x04: u32 segment_size     — total segment size
+0x08: u32 model_buf_offset — weight data start (also = layer descriptor region size + header)
+0x0C: u32 tmp_buf_size     — required temp buffer for inference
+0x14: u8  data_slice       — 0=normal, 1=slice mode
+0x16: u8  roi_batch_num    — [0, 32]
+0x30: u16 total_scale_num  — scales [1, 16]
+0x32: u16 total_layer_num  — layers [1, 65535/scale_num]
+0x34: u8  src_num          — input nodes [1, 16]
+0x35: u8  dst_num          — output nodes [1, 16]
+0x36: u8  top_layer_num    — [1, 16]
+0x38: u32 weight_data_size — weight blob size
+0x3C: u32 weight_data_off  — offset from segment start to weights
+0x40: u32 layer_name_off   — offset from segment start to name table
+0x50: u16[] src_node_ids   — src_num entries (layer IDs for input nodes)
+0x50+2*src_num: u16[] dst_node_ids — dst_num entries
```

### Layer Descriptor Start Offset
```
v63 = align16(2*(src_num + dst_num)) + 80
v61 = 32 * src_num + v63          — end of src node data
v70 = 32 * dst_num + v61          — layer descriptors start at seg + v70
For src=1, dst=1: v70 = 0xA0 (160 bytes from segment start)
```

### Layer Descriptors (sequential, variable-length)
First byte of each descriptor = layer type:

| Type | Name | Size | Key Fields |
|------|------|------|-----------|
| 0 | Conv | 80B | +0x04:in_fmt, +0x05:out_fmt, +0x06:pool_mode, +0x07:af_mode, +0x08:is_pad, +0x0A:in_c(u16), +0x0C:in_h, +0x0E:in_w, +0x10:out_c, +0x12:out_h, +0x14:out_w, +0x18:arg_len, +0x1C:arg_offset, +0x2C:in_tmp_off, +0x30:out_tmp_off |
| 1 | Flatten | 48B | +0x08:in_c, +0x0A:in_h, +0x0C:in_w, +0x10:in_tmp_off, +0x14:out_tmp_off |
| 2 | FC | 64B | +0x0A:in_w, +0x0C:out_w, +0x14:arg_len, +0x18:arg_offset |
| 3 | Eltwise | 96B | +0x06:in_num, +0x07:eltwise_op, +0x08:out_c, +0x0A:out_h, +0x0C:out_w |
| 4 | Unpack | 64B | +0x08:out_c, +0x0A:out_h, +0x0C:out_w |
| 5 | Preproc | 48B | +0x04:need_vgs, +0x06:in_c, +0x08:in_h, +0x0A:in_w, +0x2C:blob_type |
| 6 | DMA | 32B | +0x06:in_w, +0x08:in_h |

Weight data is referenced by `arg_offset` + `arg_len` within each Conv/FC layer,
pointing into the weight blob at segment + `weight_data_off`.

### Layer Name Table
At segment + `layer_name_off`. Each entry is 0x40 bytes:
32 bytes name (null-padded) + 32 bytes metadata (layer index at +0x30).

### Vendor YOLO Model Architecture (parsed from OMS)
```
[0]  Preproc VGS: 360×640×3 input
[1]  Conv: 3→8ch, 360×640    (first conv, likely 3×3)
[2]  Conv: 8→8ch, 360→180    (stride-2 downscale)
[3]  Conv: 16ch, 180×320
...
[20] Conv: 64ch, 23×40       (final spatial resolution)
...
[42] Conv: 512→512ch, 23×40
[43] Conv: 512→63ch, 23×40   (detection head, no pool/relu)
[44] Unpack                   (output format conversion)
```
45 layers total, 42 conv + 1 preproc + 1 unpack + 1 output conv.
Output: 40×23×63 = 9 anchors × (4 bbox + 1 conf + 2 class).

## Success Criteria

The RE is complete when we can:
1. ~~Define the complete ctrl struct~~ — DONE (just src_num + dst_num)
2. ~~Know where kernel writes CNN output~~ — DONE (dst_blobs_out at +0x640)
3. ~~Know output tensor format~~ — DONE (S8, stride aligned to 16)
4. Populate dst_blobs_out with correct phys addresses → get non-zero detection output
