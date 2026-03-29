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

## Success Criteria

The RE is complete when we can:
1. Define the complete `ctrl` struct (arg [sp+8]) with all field offsets and valid values
2. Know exactly where the kernel writes CNN output (which blob, which phys_addr)
3. Know the output tensor format (data type, dimensions, layout)
4. Populate all forward_slice arguments correctly to get non-zero detection output
   from `test-xnn-direct.c`
