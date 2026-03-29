# XNN Ioctl Trace Specification

Spec for an LD_PRELOAD hook that captures the exact ioctl traffic needed
to replay direct IVE XNN CNN inference without any vendor library.

## Goal

Capture a complete, byte-exact recording of one IVP inference cycle
(init → load model → forward N frames → cleanup) so we can replay it
from our own code using raw `ioctl()` calls to `/dev/ive`.

## Device

All ioctls go to `/dev/ive` (char device, major 218, minor 17).
The fd is opened once during `HI_MPI_SYS_Init()` and reused for
both standard IVE pixel ops and XNN CNN inference.

## Ioctls to Capture

### Phase 1: Initialization (once per session)

#### SVP_INIT — `ioctl(fd, 0x8010463b, buf)`
- Direction: Read (kernel writes to buf)
- Buffer: 16 bytes
- **Capture**: full 16 bytes AFTER the call
- **We care about**: physical address of mapped XNN HW registers (offset +0x08, u32)
- **Purpose**: maps XNN hardware register space via /dev/mem internally

#### OPEN_DEV — `ioctl(fd, 0x801046c8, buf)`
- Direction: Read
- Buffer: 16 bytes
- **Capture**: full 16 bytes AFTER the call
- **We care about**: session handle and mapped address
- **Purpose**: opens IVE/XNN execution context

#### CLOSE — `ioctl(fd, 0x46c9)`
- No buffer (size=0)
- **Capture**: return value only
- **Purpose**: sync/reset between phases

#### ENABLE — `ioctl(fd, 0x46ca)`
- No buffer (size=0)
- **Capture**: return value only
- **Purpose**: arm XNN engine for operations

### Phase 2: Model Loading (once per model segment)

#### LOADMODEL — `ioctl(fd, 0xc8a04636, buf)`
- Direction: Read/Write
- Buffer: 2208 bytes
- Layout:
  ```
  [+0x000] model_mem  (24B): {u64 phys, u64 virt, u32 size, u32 pad}
  [+0x018] tmp_mem    (24B): {u64 phys, u64 virt, u32 size, u32 pad}
  [+0x028] model_params (2160B): zeros on input, kernel fills on output
  ```
- **Capture BEFORE call**:
  - `model_mem.phys_addr`, `model_mem.size` (where the OMS segment is in MMZ)
  - `tmp_mem.phys_addr`, `tmp_mem.size` (temp working buffer)
- **Capture AFTER call**:
  - Full `model_params` (2160 bytes) — this is the kernel-parsed model descriptor
  - Return value
- **We care about** (in model_params output):
  - `[+0x04]` u32: tmp_buf_size needed
  - `[+0x08]` u32: src_num (number of input blobs)
  - `[+0x0C]` u32: flags
  - `[+0x10]` u32: blob_type (2=YUV420SP)
  - `[+0x14]` u32: input width
  - `[+0x18]` u32: input height
  - `[+0x1C]` u32: input channels
  - `[+0x24]` char[32]: input node name ("data")
  - `[+0x44]` u32: output layer count
  - `[+0x48]` onwards: output layer descriptors (dimensions, types)
- **Called twice** for the vendor model: once for backbone (seg1), once for head (seg2)
- **Key**: record which OMS file offset each segment came from

### Phase 3: Per-Frame Inference (repeated)

This is the critical phase. Two related ioctls that must be captured together.

#### FORWARD_SLICE — `ioctl(fd, 0xc9704638, buf)`
- Direction: Read/Write
- Buffer: 2416 bytes
- **This is called via `mpi_ive_xnn_forward_slice()` which has 8 arguments.**
  The function builds the 2416-byte ioctl buffer from those args internally.

**To capture the function arguments** (more useful than raw ioctl buffer):

Hook `mpi_ive_xnn_forward_slice` directly (exported from libive.so):
```c
int mpi_ive_xnn_forward_slice(
    int32_t *handle,      // r0: output IVE handle for query
    void    *src_blobs,   // r1: input blob array (48 bytes per blob)
    void    *dst_blobs,   // r2: output blob array (kernel fills)
    void    *model_params,// r3: loaded model params (2160B)
    void    *tmp_mem,     // [sp+0]: temp buffer descriptor (24B)
    void    *report_blob, // [sp+4]: output report/detection blob (48B)
    void    *ctrl,        // [sp+8]: forward control struct
    int      instant      // [sp+12]: 1=synchronous, 0=async
);
```

**Capture for each argument BEFORE the call:**

1. **handle** (r0): just the pointer address (for correlation)

2. **src_blobs** (r1): dump 48 bytes per blob. Blob layout:
   ```
   [+0x00] u32 blob_type   (2=YUV420SP)
   [+0x04] u32 width
   [+0x08] u32 virt_addr   (32-bit userspace pointer)
   [+0x0C] u32 virt_hi     (upper bits or secondary addr)
   [+0x10] u64 phys_addr
   [+0x18] u32 num         (batch count, usually 1)
   [+0x1C] u32 reserved
   [+0x20] u32 stride
   [+0x24] u32 height
   [+0x28] u32 channels
   [+0x2C] u32 pad
   ```
   **We care about**: all fields. This tells us the input tensor layout.

3. **dst_blobs** (r2): dump 48 bytes. Usually all zeros BEFORE call.
   **Capture AFTER call too** — kernel may fill with output tensor descriptor.

4. **model_params** (r3): dump first 128 bytes (to identify which model).
   **We care about**: `[+0x14]` w, `[+0x18]` h to confirm which segment is running.

5. **tmp_mem** ([sp+0]): dump 24 bytes.
   ```
   [+0x00] u64 phys_addr
   [+0x08] u64 virt_addr
   [+0x10] u32 size
   ```
   **We care about**: phys_addr and size (must match what was allocated).

6. **report_blob** ([sp+4]): dump 48 bytes BEFORE and AFTER.
   This is the **detection output tensor**. Layout same as src_blob.
   **We care about**: ALL fields — this tells us the output dimensions and where
   the detection results are written. After the call, the data at `phys_addr`
   contains the raw detection output (bbox proposals, scores, etc.)
   **Also dump first 256 bytes of the report data buffer** (at report_blob.virt_addr)
   AFTER the call to see actual detection values.

7. **ctrl** ([sp+8]): dump 128 bytes.
   Known layout (partial):
   ```
   [+0x00] u32 src_num    (1)
   [+0x04] u32 dst_num    (1)
   [+0x08] u32 ???        (0)
   [+0x0C] u32 ???        (1)
   [+0x10] ... copy of src_blob (48 bytes)
   ```
   **We care about**: full 128 bytes. The first 16 bytes are control flags,
   the rest may contain additional blob descriptors or slice parameters.

8. **instant** ([sp+12]): just the integer value (1 or 0).

**Capture AFTER the call:**
- Return value (0=success)
- `*handle` value (IVE task handle, used for query)
- `dst_blobs` (48 bytes) — did kernel fill output descriptor?
- `report_blob` (48 bytes) — updated output descriptor?
- First 256 bytes of report data buffer (actual detection output)

#### QUERY — `ioctl(fd, 0x463c)`
- No buffer
- **Capture**: return value, timing (how long after forward_slice)
- **Purpose**: polls for hardware completion (may be called in loop)

#### FINISH — `ioctl(fd, 0x463d)`
- No buffer
- **Capture**: return value
- **Purpose**: finalizes the inference, results are now readable

### Phase 4: Cleanup

#### CLOSE — `ioctl(fd, 0x46c9)` — between frames
#### ENABLE — `ioctl(fd, 0x46ca)` — re-arm for next frame

These bracket each frame. Capture call order and return values.

## Capture Format

Output as structured text to stderr. For each ioctl/function call:

```
[TRACE] <timestamp_us> <function_name> BEFORE
  arg0: <hex dump or value>
  arg1: <hex dump>
  ...
[TRACE] <timestamp_us> <function_name> AFTER ret=<value>
  out0: <hex dump>
  ...
[DATA] <label> <offset> <length>
  <hex bytes>
```

## What We Need to Extract

From a single successful inference cycle (3-5 frames), extract:

1. **The exact ioctl sequence** with commands, ordering, and timing
2. **The LOADMODEL output** (model_params 2160 bytes) for each segment
3. **The FORWARD_SLICE report blob** BEFORE and AFTER — this is the missing piece.
   We need to know the exact output blob dimensions, phys/virt addresses, and
   where the kernel writes detection results
4. **The ctrl struct** (128 bytes) — we need the full contents, not just src_num/dst_num
5. **The report data buffer** (first 256+ bytes) after successful forward — this
   contains the raw YOLO output (bboxes, scores) that svp_alg_yoloonet_process
   decodes on CPU

## Test Setup

```bash
# On host: build hook
CC=~/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-gcc
SDK=~/projects/cameras/sdk/Hi3516EV200_SDK_V1.0.1.2/mpp
$CC -shared -fPIC -o xnn-hook.so xnn-trace.c -I$SDK/include -ldl
sudo cp xnn-hook.so /mnt/noc/

# On board:
killall majestic
LD_PRELOAD=/utils/xnn-hook.so /utils/test-ivp \
    /utils/ivp_re_allday_f1y2f2m1_640x360_v1003.oms \
    /utils/test-ivp-people.y4m 2>/utils/xnn-trace.log

# Collect trace:
# On host: cp /mnt/noc/xnn-trace.log .
```

## Success Criteria

The trace is complete when we can:
1. Reconstruct the full `report_blob` layout (all 48 bytes, with correct phys/virt/size)
2. See non-zero data in the report buffer after forward_slice (actual detection output)
3. Know the exact `ctrl` struct contents (128 bytes)
4. Replay the sequence from test-xnn-direct.c and get matching non-zero output
