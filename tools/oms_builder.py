#!/usr/bin/env python3
"""
OMS Model Builder — Create HiSilicon IVE XNN .oms model files

Converts a PyTorch model with int8-quantized weights into the .oms format
that can be loaded by mpi_ive_xnn_loadmodel() on Hi3516EV200/EV300.

Format details reverse-engineered from:
  - Ghidra decompilation of hi3516ev200_ive.ko
  - Byte-level analysis of vendor OMS models
  - CRC32 verification (standard polynomial, init=0, stored as ~crc)

Supported layers: Preproc(VGS), Flatten, FC, Unpack
Limitations: single-input, single-output, single-scale, no Conv yet

Usage:
  python3 oms_builder.py model.pt output.oms    # convert PyTorch model
  python3 oms_builder.py --test output.oms       # generate test model
"""

import struct
import argparse
import numpy as np
from pathlib import Path


def align16(n):
    return (n + 15) & ~15


def crc32_oms(data: bytes) -> int:
    """CRC32 with standard polynomial 0xEDB88320, init=0, no final XOR."""
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (0xEDB88320 ^ (c >> 1)) if (c & 1) else (c >> 1)
        table.append(c)
    crc = 0
    for b in data:
        crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc


def make_preproc(input_h, input_w, input_c, out_tmp_offset, need_vgs=1):
    """Build a 48-byte Preproc layer descriptor.

    Uses the exact byte pattern from vendor v2001 32×32 model as reference,
    with input dimensions and offsets patched. The Preproc descriptor has
    many undocumented fields (stride constants, normalization params) that
    must match the kernel's expectations.
    """
    # Reference: v2001 segment 2, Preproc for 32×32×3 VGS input
    # 05 00 00 01 03 00 20 00 20 00 04 84 04 84 04 84
    # 00 fe 00 fe 00 fe 20 00 00 04 00 00 e0 3f 02 00
    # f0 ff ff ff 00 10 00 00 02 00 00 00 00 00 00 00
    d = bytearray(48)
    d[0] = 5   # type = Preproc
    d[1] = 0   # layer_index (filled by caller if needed)
    d[2] = 0
    d[3] = 1   # CRITICAL: must be 1 (unknown flag, validated by kernel)
    d[4] = 3   # need_vgs = 3 (VGS mode with CSC)
    d[5] = 0
    struct.pack_into('<H', d, 6, input_h)
    struct.pack_into('<H', d, 8, input_w)
    # Stride/normalization constants (copied from reference)
    struct.pack_into('<H', d, 0x0A, 0x8404)
    struct.pack_into('<H', d, 0x0C, 0x8404)
    struct.pack_into('<H', d, 0x0E, 0x8404)
    struct.pack_into('<H', d, 0x10, 0xFE00)
    struct.pack_into('<H', d, 0x12, 0xFE00)
    struct.pack_into('<H', d, 0x14, 0xFE00)
    # Bytes 10-47: stride/normalization constants
    # These are architecture-dependent and computed by the vendor's model compiler.
    # For 32×32×3 VGS input, use exact bytes from working v2001 reference:
    ref_tail = bytes.fromhex(
        '04 84 04 84 04 84'     # [10..15]: mean/norm constants
        '00 fe 00 fe 00 fe'     # [16..21]: more norm constants
        '20 00'                 # [22..23]: out_stride_w = 32
        '00 04 00 00'           # [24..27]: out_stride_c = 1024
        'e0 3f 02 00'           # [28..31]: total_output_size = 147424
        'f0 ff ff ff'           # [32..35]: unknown (-16)
        '00 10 00 00'           # [36..39]: unknown (4096)
        '02 00 00 00'           # [40..43]: blob_type = 2 (YVU420SP)
        '00 00 00 00'           # [44..47]: padding
    )
    d[10:48] = ref_tail
    # Patch stride for actual dimensions if not 32×32
    if input_w != 32 or input_h != 32:
        out_stride_w = align16(input_w)
        struct.pack_into('<H', d, 22, out_stride_w)
        struct.pack_into('<I', d, 24, out_stride_w * input_h)
    return bytes(d)


def make_conv(input_c, input_h, input_w, output_c, output_h, output_w,
              kernel_size, pool_mode, af_mode, arg_len_cumulative, arg_offset,
              in_tmp_offset, out_tmp_offset, layer_index,
              in_fmt=0, out_fmt=1, in_bond_num=1):
    """Build an 80-byte Conv layer descriptor.

    Conv descriptor layout (from IDA RE of ive_xnn_check_conv_layer):
      d[0]  = type (0=Conv)
      d[1]  = 1 (always, unknown flag)
      d[4]  = in_fmt {0,2,4}
      d[5]  = out_fmt {0,1}
      d[6]  = pool_mode [0,5) — 0=none, 1=max2x2, 2=avg2x2
      d[7]  = af_mode [0,4) — 0=none, 1=relu, 2=sigmoid, 3=tanh
      d[8]  = is_pad [0,2) — must be 0 for kernel_size=3
      d[10..11] = input_c  [1,1024]
      d[12..13] = input_h  [2,720]
      d[14..15] = input_w  [2,1280]
      d[16..17] = output_c [1,1024]
      d[18..19] = output_h
      d[20..21] = output_w
      d[24..27] = arg_len (cumulative)
      d[28..31] = arg_offset (per-layer)
      d[44..47] = in_tmp_offset
      d[48..51] = out_tmp_offset
      d[52..53] = in_stride_w (aligned 16)
      d[54..55] = out_stride_w (aligned 16)
      d[56..59] = in_stride_c (= input_h × in_stride_w)
      d[60..63] = out_stride_c (= output_h × out_stride_w)
      d[61]     = kernel_size {1,3}
      d[62]     = in_bond_num {1,2,8}
    """
    d = bytearray(80)
    d[0] = 0  # type = Conv
    d[1] = 1  # always 1 in vendor models
    # Fields matched to vendor s[] packing → check function a1[offset]:
    # s[1] = {desc[3], desc[4..5] as word, desc[6]}
    # s[2] = {desc[7], padding, desc[8..9] as word}
    d[3] = in_fmt       # a1+4 = LOBYTE(s[1])
    d[4] = out_fmt      # a1+5 = BYTE1(s[1])
    d[5] = pool_mode    # a1+6 = BYTE2(s[1])
    d[6] = af_mode      # a1+7 = HIBYTE(s[1])
    d[7] = 0            # a1+8 = LOBYTE(s[2]) = is_pad (0 for 3×3)
    struct.pack_into('<H', d, 8, input_c)    # a1+10 = HIWORD(s[2])
    struct.pack_into('<H', d, 10, input_h)   # a1+12 = LOWORD(s[3])
    struct.pack_into('<H', d, 12, input_w)   # a1+14 = HIWORD(s[3])
    struct.pack_into('<H', d, 14, output_c)  # a1+16 = LOWORD(s[4])
    struct.pack_into('<H', d, 16, output_h)  # a1+18 = HIWORD(s[4])
    struct.pack_into('<H', d, 18, output_w)  # a1+20 = LOWORD(s[5])
    # s[6..15] = desc[20..59] as u32 words
    struct.pack_into('<I', d, 20, arg_len_cumulative)   # s[6] = a1+24
    struct.pack_into('<I', d, 24, arg_offset)           # s[7] = a1+28
    # s[8]=desc[28..31], s[9]=desc[32..35], s[10]=desc[36..39] — reserved
    struct.pack_into('<I', d, 40, in_tmp_offset)        # s[11] = a1+44
    struct.pack_into('<I', d, 44, out_tmp_offset)       # s[12] = a1+48
    # in_stride must be exactly align16(input_w) (vendor check: must equal X)
    # out_stride minimum is 240 (vendor check: ive_xnn_check_stride [240,65535])
    in_stride_w = align16(input_w)
    out_stride_w = max(align16(output_w), 240)
    # s[13] = desc[48..51] = {in_stride_w, out_stride_w} packed
    struct.pack_into('<H', d, 48, in_stride_w)          # a1+52 LOWORD(s[13])
    struct.pack_into('<H', d, 50, out_stride_w)         # a1+54 HIWORD(s[13])
    struct.pack_into('<I', d, 52, input_h * in_stride_w)  # s[14] = in_stride_c
    struct.pack_into('<I', d, 56, output_h * out_stride_w) # s[15] = out_stride_c
    # s[16] = {desc[60..61] as u16, desc[62] as byte}
    d[61] = kernel_size  # BYTE1 of LOWORD(s[16]) — MUST be 1 or 3
    d[62] = in_bond_num  # BYTE2(s[16])
    # s[19] = {desc[72] as byte, desc[73..74] as u16}
    # desc[72] = is_bottom_from_user (1 if this Conv reads from model input)
    # desc[73..74] = input_node_name_id (which src_node this references)
    d[72] = 1  # first Conv reads from user input
    struct.pack_into('<H', d, 73, 0)  # src_node 0 (= 'data')
    return bytes(d)


def make_flatten(in_c, in_h, in_w, in_tmp_offset, out_tmp_offset,
                 layer_index):
    """Build a 48-byte Flatten layer descriptor.

    Byte-packed format (from kernel ive_xnn_check_flatten_layer):
      d[0]      = layer_type (1)
      d[1..2]   = name_id (u16 LE)
      d[3]      = out_fmt (must be 1 or 2)
      d[4..5]   = reserved/flags (u16)
      d[6..7]   = input_c (u16 LE, [1,1024])
      d[8..9]   = input_h (u16 LE, [1,720])
      d[10..11]  = input_w (u16 LE, [1,1280])
      d[12..15]  = in_tmp_offset (u32, < tmp_buf_size)
      d[16..19]  = out_tmp_offset (u32)
      d[20..21]  = in_stride_w (u16, aligned to 16)
      d[22..23]  = out_stride_w (u16)
      d[24..27]  = in_stride_c (u32, = input_h * in_stride_w)
    """
    d = bytearray(48)
    d[0] = 1  # type = Flatten
    struct.pack_into('<H', d, 1, layer_index)  # name_id
    d[3] = 1  # out_fmt
    struct.pack_into('<H', d, 6, in_c)
    struct.pack_into('<H', d, 8, in_h)
    struct.pack_into('<H', d, 10, in_w)
    struct.pack_into('<I', d, 12, in_tmp_offset)
    struct.pack_into('<I', d, 16, out_tmp_offset)
    in_stride_w = align16(in_w)
    out_total = in_c * in_h * in_w
    out_stride_w = align16(out_total)
    struct.pack_into('<H', d, 20, in_stride_w)
    struct.pack_into('<H', d, 22, out_stride_w)
    struct.pack_into('<I', d, 24, in_h * in_stride_w)
    return bytes(d)


def make_fc(input_w, output_w, arg_len_cumulative, arg_offset,
            in_tmp_offset, out_tmp_offset, layer_index,
            af_mode=0, in_fmt=2, out_fmt=1):
    """Build a 64-byte FC layer descriptor.

    Byte-packed format (from kernel ive_xnn_check_fc_layer + parse):
      d[0]      = layer_type (2)
      d[1..2]   = name_id (u16 LE)
      d[3]      = in_fmt ({0,2,4})
      d[4]      = out_fmt ({1,2})
      d[5]      = batch_num (must be 0)
      d[6]      = af_mode ([0,3))
      d[7]      = reserved
      d[8..9]   = input_w (u16, [1,8192])
      d[10..11]  = output_w (u16, [1,8192])
      d[12..13]  = reserved
      d[14..15]  = in_stride_w (u16, aligned to 16)
      d[16..19]  = arg_len (u32, cumulative)
      d[20..23]  = arg_offset (u32, offset into weight blob)
      d[24..27]  = in_tmp_offset (u32, < tmp_buf_size)
      d[28..31]  = out_tmp_offset (u32, < tmp_buf_size)
      d[52]     = is_bott_from_usr (0 or 1)
      d[53..54]  = input node name_id (u16, for cross-ref)
    """
    d = bytearray(64)
    d[0] = 2  # type = FC
    struct.pack_into('<H', d, 1, layer_index)  # name_id
    d[3] = in_fmt
    d[4] = out_fmt
    d[5] = 1  # batch_num (must be 1)
    d[6] = af_mode
    struct.pack_into('<H', d, 8, input_w)
    struct.pack_into('<H', d, 10, output_w)
    in_stride = align16(input_w)
    # out_stride = align16((out_fmt_size * output_w) >> 3), minimum 16
    # out_fmt=1 → size=1 byte; out_fmt=2 → size=4 bytes
    out_fmt_sz = 1 if out_fmt == 1 else 4
    out_stride = align16(max((out_fmt_sz * output_w) >> 3, 16))
    struct.pack_into('<H', d, 12, in_stride)   # v178+14 reads from d[12..13]
    struct.pack_into('<H', d, 14, out_stride)  # v178+16 reads from d[14..15]
    struct.pack_into('<I', d, 16, arg_len_cumulative)
    struct.pack_into('<I', d, 20, arg_offset)
    struct.pack_into('<I', d, 24, in_tmp_offset)
    struct.pack_into('<I', d, 28, out_tmp_offset)
    d[52] = 1  # is_bott_from_usr (1 = reads from user input, sets InNode flag)
    struct.pack_into('<H', d, 53, 0)  # src_node name_id (0 = matches preproc)
    return bytes(d)


def make_unpack(out_c, out_h, out_w, in_tmp_offset, out_tmp_offset,
                layer_index):
    """Build a 64-byte Unpack layer descriptor.

    Byte-packed format (same pattern as other layers):
      d[0]     = layer_type (4)
      d[1..2]  = name_id (u16)
      d[3]     = in_fmt ({0,1})
      d[4]     = out_fmt ({1,2})
      d[5]     = reserved
      d[6..7]  = output_c (u16, [1,1024])
      d[8..9]  = output_h (u16, [1,720])
      d[10..11] = output_w (u16, [1,1280])
      d[12..13] = in_stride_w (u16)
      d[14..15] = out_stride_w (u16)
      d[16..19] = in_stride_c (u32)
      d[20..23] = out_stride_c (u32)
      d[24..27] = in_tmp_offset (u32)
      d[28..31] = out_tmp_offset (u32)
    """
    d = bytearray(64)
    d[0] = 4  # type = Unpack
    struct.pack_into('<H', d, 1, layer_index)  # name_id
    d[3] = 1  # in_fmt
    d[4] = 1  # out_fmt (must be {1,2})
    struct.pack_into('<H', d, 6, out_c)
    struct.pack_into('<H', d, 8, out_h)
    struct.pack_into('<H', d, 10, out_w)
    # in_stride: internal format (int8/int32), needs >= out_w * elem_size
    in_stride = align16(max(out_w * 4, 16))
    # out_stride: external format, computed as align16((out_fmt_sz * out_w) >> 3), min 16
    # out_fmt=1 → 1 byte per elem → (1 * out_w) >> 3 → small; min is 16
    out_stride = align16(max(out_w, 16))  # vendor minimum is out_w, 16-byte aligned
    struct.pack_into('<H', d, 12, in_stride)
    struct.pack_into('<H', d, 14, out_stride)
    struct.pack_into('<I', d, 16, out_h * in_stride)   # in_stride_c
    struct.pack_into('<I', d, 20, out_h * out_stride)  # out_stride_c
    struct.pack_into('<I', d, 24, in_tmp_offset)
    struct.pack_into('<I', d, 28, out_tmp_offset)
    # Unpack also needs arg_offset and arg_len (from parse: v178[13,14])
    # v178[13] = *(u32*)(v174+48) → arg_len at d[48..51]
    # v178[14] = *(u32*)(v174+52) → arg_offset at d[52..55]
    struct.pack_into('<I', d, 48, 1)  # arg_len (minimal)
    struct.pack_into('<I', d, 52, 1)  # arg_offset >= 1
    return bytes(d)


def make_name_entry(name: str, index: int) -> bytes:
    """Build a 0x40-byte layer name table entry."""
    entry = bytearray(0x40)
    name_bytes = name.encode('ascii')[:31]
    entry[:len(name_bytes)] = name_bytes
    struct.pack_into('<I', entry, 0x30, index)
    return bytes(entry)


def quantize_linear(weight, bias, weight_scale=None):
    """Quantize float32 weight/bias to int8/int32 for XNN.

    Returns (weight_int8, bias_int32, scale) where:
      weight_int8 = clamp(round(weight / scale), -128, 127)
      bias_int32 = round(bias / scale)
    """
    if weight_scale is None:
        w_abs_max = max(abs(weight.max()), abs(weight.min()), 1e-8)
        weight_scale = w_abs_max / 127.0

    w_q = np.clip(np.round(weight / weight_scale), -128, 127).astype(np.int8)
    b_q = np.round(bias / weight_scale).astype(np.int32)
    return w_q, b_q, weight_scale


def build_segment(layers_desc: bytes, weight_data: bytes,
                  name_entries: bytes, src_num: int, dst_num: int,
                  total_layers: int, tmp_buf_size: int,
                  src_names: list, dst_names: list,
                  data_slice=0, roi_batch=0, top_layer_num=1,
                  timestamp='0000000000000000') -> bytes:
    """Build a complete OMS segment with header, descriptors, weights, names."""

    # Calculate layout
    v63 = ((-2 * (src_num + dst_num)) & 0xF) + 2 * (src_num + dst_num) + 80
    v61 = 32 * src_num + v63
    v70 = 32 * dst_num + v61  # layer desc start

    layer_desc_end = v70 + len(layers_desc)
    weight_data_off = max(align16(layer_desc_end + 16), 1)  # must be >= 1
    weight_data_end = weight_data_off + len(weight_data)
    name_table_off = align16(weight_data_end + 16)
    segment_end = name_table_off + len(name_entries)
    segment_size = align16(segment_end)

    seg = bytearray(segment_size)

    # Header
    # +0x00: CRC32 (filled last)
    struct.pack_into('<I', seg, 4, segment_size)  # segment_size
    struct.pack_into('<I', seg, 8, 0xA0)  # model_buf_offset (= v70 typically)
    struct.pack_into('<I', seg, 12, tmp_buf_size)
    # +0x10..0x13: unknown flags
    struct.pack_into('<I', seg, 0x10, 0x04010201)  # from reference
    seg[20] = data_slice
    seg[21] = 1   # unknown byte at +0x15, always 1 in working models
    seg[22] = roi_batch if roi_batch else 32  # default to 32 like vendor
    # +0x18..0x1F: padding/unknown
    struct.pack_into('<I', seg, 0x18, 0x000780)  # from reference
    struct.pack_into('<I', seg, 0x1C, 0x000280)  # from reference
    # Timestamp at +0x20
    ts_bytes = timestamp.encode('ascii')[:16]
    seg[0x20:0x20 + len(ts_bytes)] = ts_bytes
    # Layer/node counts
    struct.pack_into('<H', seg, 48, 1)  # total_scale_num
    struct.pack_into('<H', seg, 50, total_layers)
    seg[52] = src_num
    seg[53] = dst_num
    seg[54] = top_layer_num
    # Weight info
    struct.pack_into('<I', seg, 56, len(weight_data))
    struct.pack_into('<I', seg, 60, weight_data_off)
    struct.pack_into('<I', seg, 64, name_table_off)

    # Src node name IDs at +0x50 (u16 per src)
    # These are layer indices that the src nodes reference
    for i, name in enumerate(src_names):
        struct.pack_into('<H', seg, 80 + i * 2, 0)  # typically 0
    # Dst node name IDs — immediately after src_node name IDs (no alignment)
    dst_name_off = 80 + 2 * src_num
    for i, name in enumerate(dst_names):
        struct.pack_into('<H', seg, dst_name_off + i * 2, total_layers - 1)

    # Src node data (32 bytes each) — just the name string
    for i, name in enumerate(src_names):
        name_b = name.encode('ascii')[:31]
        seg[v63 + i * 32:v63 + i * 32 + len(name_b)] = name_b

    # Dst node data (32 bytes each) — just the name string
    for i, name in enumerate(dst_names):
        name_b = name.encode('ascii')[:31]
        seg[v61 + i * 32:v61 + i * 32 + len(name_b)] = name_b

    # Layer descriptors
    seg[v70:v70 + len(layers_desc)] = layers_desc

    # Fixup arg_offset fields: convert from weight-blob-relative to segment-relative
    # by adding weight_data_off. This is needed because the HW uses
    # model_phys + arg_offset directly to find weight data.
    off = v70
    for _ in range(total_layers):
        ltype = seg[off]
        if ltype == 0:  # Conv: arg_offset at desc+24 (4 bytes)
            old = struct.unpack_from('<I', seg, off + 24)[0]
            struct.pack_into('<I', seg, off + 24, old + weight_data_off)
            off += 80
        elif ltype == 2:  # FC: arg_offset at desc+20 (4 bytes)
            old = struct.unpack_from('<I', seg, off + 20)[0]
            struct.pack_into('<I', seg, off + 20, old + weight_data_off)
            off += 64
        elif ltype == 4:  # Unpack: no weight data, don't patch
            off += 64
        elif ltype == 5:  # Preproc
            off += 48
        elif ltype == 1:  # Flatten
            off += 48
        else:
            off += 80  # default

    # Weight data
    seg[weight_data_off:weight_data_off + len(weight_data)] = weight_data

    # Name table
    seg[name_table_off:name_table_off + len(name_entries)] = name_entries

    # CRC32: compute over bytes [4..segment_size), store ~crc at [0]
    crc = crc32_oms(bytes(seg[4:segment_size]))
    struct.pack_into('<I', seg, 0, (~crc) & 0xFFFFFFFF)

    return bytes(seg)


def build_outer_header(segment_data: bytes, version='v1003') -> bytes:
    """Build the OMS outer file header."""
    if version == 'v1003':
        # 0x50-byte outer header
        hdr = bytearray(0x50)
        total_size = 0x50 + len(segment_data)
        struct.pack_into('<I', hdr, 4, total_size)
        struct.pack_into('<I', hdr, 8, 0x20)  # header_half
        struct.pack_into('<I', hdr, 0x0C, 0x0100)  # flags
        struct.pack_into('<I', hdr, 0x10, 0x20)
        # +0x20: inner header (copies some segment fields)
        seg_crc = struct.unpack_from('<I', segment_data, 0)[0]
        seg_size = struct.unpack_from('<I', segment_data, 4)[0]
        struct.pack_into('<I', hdr, 0x20, seg_crc)
        struct.pack_into('<I', hdr, 0x24, seg_size + 0x30)
        struct.pack_into('<I', hdr, 0x28, 0x30)
        # Segment count flags
        struct.pack_into('<I', hdr, 0x2C, 0x0100)  # 1 segment
        # Timestamp from segment
        hdr[0x30:0x40] = segment_data[0x20:0x30]
        struct.pack_into('<I', hdr, 0x40, 0x30)  # seg1 offset within payload
        struct.pack_into('<I', hdr, 0x44, 1)
        # seg2 offset = 0 (no second segment)
        # Outer CRC
        outer_crc = crc32_oms(bytes(hdr[4:0x50]))
        struct.pack_into('<I', hdr, 0, (~outer_crc) & 0xFFFFFFFF)
        return bytes(hdr)
    else:
        raise ValueError(f'Unsupported version: {version}')


def build_fc_model(input_h, input_w, input_c, output_size,
                   weight_np, bias_np, output_path,
                   weight_scale=None):
    """Build an OMS file for a Flatten + FC model.

    Args:
        input_h, input_w, input_c: input spatial dimensions and channels
        output_size: FC output dimension
        weight_np: float32 weight array [output_size, input_c * input_h * input_w]
        bias_np: float32 bias array [output_size]
        output_path: where to write the .oms file
        weight_scale: quantization scale (auto-computed if None)
    """
    flat_size = input_c * input_h * input_w
    assert weight_np.shape == (output_size, flat_size), \
        f'Weight shape {weight_np.shape} != ({output_size}, {flat_size})'
    assert bias_np.shape == (output_size,), \
        f'Bias shape {bias_np.shape} != ({output_size},)'

    # Quantize
    w_q, b_q, scale = quantize_linear(weight_np, bias_np, weight_scale)
    print(f'Quantization scale: {scale:.6f}')
    print(f'Weight range: [{w_q.min()}, {w_q.max()}]')
    print(f'Bias range: [{b_q.min()}, {b_q.max()}]')

    # Pack weights: 1 byte padding (arg_offset must be >= 1) + int8 weights + int32 bias
    weight_blob = b'\x00' + w_q.tobytes() + b_q.tobytes()
    weight_size = len(weight_blob)
    print(f'Weight blob: {weight_size} bytes ({w_q.nbytes} weights + {b_q.nbytes} bias)')

    # Compute tmp buffer size needed
    # The VGS preproc uses significantly more tmp buffer than the raw
    # pixel output — it needs working memory for resize, CSC, and
    # intermediate normalization. The vendor's 32×32 model uses 147712.
    # Use a generous allocation based on reference: ~144 * input_h * input_w.
    preproc_out = align16(input_w) * input_h * input_c
    flatten_out = align16(flat_size)
    fc_out = align16(output_size * 4)
    vgs_overhead = input_h * input_w * 128  # VGS working memory
    tmp_buf_size = preproc_out + flatten_out + fc_out + vgs_overhead + 65536

    # Tmp buffer offsets for each layer's input/output
    preproc_out_off = 0
    flatten_in_off = preproc_out_off
    flatten_out_off = align16(preproc_out)
    fc_in_off = flatten_out_off
    fc_out_off = align16(flatten_out_off + flatten_out)
    unpack_in_off = fc_out_off
    unpack_out_off = align16(fc_out_off + fc_out)

    # Build layer descriptors
    layers = bytearray()

    # Layer 0: Preproc (VGS, 32x32x3 → internal format)
    layers += make_preproc(input_h, input_w, input_c, preproc_out_off)

    # Layer 1: Flatten (C×H×W → flat_size)
    layers += make_flatten(input_c, input_h, input_w,
                          flatten_in_off, flatten_out_off, layer_index=1)

    # Layer 2: FC (flat_size → output_size)
    layers += make_fc(flat_size, output_size,
                     arg_len_cumulative=weight_size,
                     arg_offset=1,  # offset within weight blob (real HW adds weight_data_off internally)
                     in_tmp_offset=fc_in_off,
                     out_tmp_offset=fc_out_off,
                     layer_index=2)

    # Layer 3: Unpack (output format conversion)
    layers += make_unpack(1, 1, output_size,
                         unpack_in_off, unpack_out_off, layer_index=3)

    layers = bytes(layers)
    total_layers = 4

    # Name table
    names = bytearray()
    names += make_name_entry('data', 0)
    names += make_name_entry('flatten', 1)
    names += make_name_entry('fc', 2)
    names += make_name_entry('output', 3)
    names = bytes(names)

    # Build segment
    timestamp = '2024010100000000'
    segment = build_segment(
        layers_desc=layers,
        weight_data=weight_blob,
        name_entries=names,
        src_num=1, dst_num=1,
        total_layers=total_layers,
        tmp_buf_size=tmp_buf_size,
        src_names=['data'],
        dst_names=['output'],
        timestamp=timestamp,
    )

    # Build outer header + segment
    outer = build_outer_header(segment)
    oms_data = outer + segment

    # Write file
    with open(output_path, 'wb') as f:
        f.write(oms_data)

    print(f'\nOMS file written: {output_path}')
    print(f'  Total size: {len(oms_data)} bytes')
    print(f'  Outer header: {len(outer)} bytes')
    print(f'  Segment: {len(segment)} bytes')
    print(f'  Layers: {total_layers}')
    print(f'  Weights: {weight_size} bytes')
    print(f'  Tmp buffer: {tmp_buf_size} bytes')

    return oms_data


def build_test_model(output_path):
    """Build a test OMS model: Flatten(32x32x3) + FC(3072 -> 10)."""
    np.random.seed(42)
    input_h, input_w, input_c = 32, 32, 3
    output_size = 10
    flat_size = input_c * input_h * input_w  # 3072

    # Random weights for testing
    weight = np.random.randn(output_size, flat_size).astype(np.float32) * 0.01
    bias = np.zeros(output_size, dtype=np.float32)

    print(f'Test model: Flatten({input_h}x{input_w}x{input_c}) + FC({flat_size} -> {output_size})')
    print(f'Weight shape: {weight.shape}, Bias shape: {bias.shape}')
    print()

    build_fc_model(input_h, input_w, input_c, output_size,
                   weight, bias, output_path)


def build_from_pytorch(model_path, output_path):
    """Convert a PyTorch model to OMS."""
    import torch

    state = torch.load(model_path, map_location='cpu', weights_only=True)

    # Expect keys like 'layer.weight' and 'layer.bias'
    weight_key = [k for k in state.keys() if 'weight' in k][0]
    bias_key = [k for k in state.keys() if 'bias' in k][0]

    weight = state[weight_key].numpy()
    bias = state[bias_key].numpy()

    output_size, flat_size = weight.shape

    # Infer input dimensions (assume square, 3 channels)
    # flat_size = C * H * W, try C=3 first
    for c in [3, 1]:
        hw = flat_size // c
        h = int(hw ** 0.5)
        if h * h * c == flat_size:
            input_h = input_w = h
            input_c = c
            break
    else:
        # Non-square: assume H=1
        input_h = 1
        input_w = flat_size // 3
        input_c = 3
        if input_c * input_h * input_w != flat_size:
            input_c = 1
            input_w = flat_size

    print(f'PyTorch model: {model_path}')
    print(f'  Weight: {weight_key} {weight.shape}')
    print(f'  Bias: {bias_key} {bias.shape}')
    print(f'  Inferred input: {input_h}x{input_w}x{input_c} = {flat_size}')
    print()

    build_fc_model(input_h, input_w, input_c, output_size,
                   weight, bias, output_path)


def build_conv_test(output_path):
    """Build minimal Conv test: Preproc + Conv(3×3, 1→1) + Unpack.

    Uses identity-like weights (center=127, rest=0) so the Conv output
    is approximately 127× the input — predictable for HW verification.
    """
    input_h, input_w = 32, 32  # match reference Preproc (32×32)
    input_c = 1  # grayscale
    kernel_size = 3
    output_c = 1
    output_h = input_h - kernel_size + 1  # 6
    output_w = input_w - kernel_size + 1  # 6

    # Identity-ish weights: only center pixel has weight=1, rest=0
    # This makes Conv output ≈ input_center_pixel × 1 + bias
    w = np.zeros((output_c, input_c, kernel_size, kernel_size), dtype=np.int8)
    w[0, 0, 1, 1] = 1  # center pixel only
    b = np.zeros(output_c, dtype=np.int32)

    # Weight blob: 1 byte pad + weights + bias
    weight_blob = b'\x00' + w.tobytes() + b.tobytes()
    weight_size = len(weight_blob)
    print(f'Conv test: {input_h}×{input_w}×{input_c} → 3×3 → {output_h}×{output_w}×{output_c}')
    print(f'Weight blob: {weight_size} bytes')

    # Tmp buffer layout
    preproc_out = align16(input_w) * input_h * 32  # 32ch preproc output
    conv_in_off = 0
    conv_out_off = align16(preproc_out)
    unpack_in_off = conv_out_off
    unpack_out_off = align16(conv_out_off + align16(output_w) * output_h * output_c)
    # VGS preproc needs large working memory (~144*H*W + overhead)
    vgs_overhead = input_h * input_w * 144 + 65536
    tmp_buf_size = max(unpack_out_off + 4096, vgs_overhead)

    # Build layer descriptors
    layers = bytearray()

    # Layer 0: Preproc
    layers += make_preproc(input_h, input_w, 3, 0)

    # Layer 1: Conv 3×3
    layers += make_conv(
        input_c=input_c, input_h=input_h, input_w=input_w,
        output_c=output_c, output_h=output_h, output_w=output_w,
        kernel_size=kernel_size, pool_mode=0, af_mode=0,
        arg_len_cumulative=weight_size,
        arg_offset=1,  # skip 1-byte pad
        in_tmp_offset=conv_in_off,
        out_tmp_offset=conv_out_off,
        layer_index=1)

    # Layer 2: Unpack
    layers += make_unpack(
        out_c=output_c, out_h=output_h, out_w=output_w,
        in_tmp_offset=unpack_in_off,
        out_tmp_offset=unpack_out_off,
        layer_index=2)

    total_layers = 3

    # Calculate weight_data_off (same as build_segment)
    v63 = ((-2 * 2) & 0xF) + 2 * 2 + 80
    v61 = 32 + v63
    v70 = 32 + v61
    layer_desc_end = v70 + len(layers)
    weight_data_off = max(align16(layer_desc_end + 16), 1)

    # Build name entries
    names = bytearray()
    for nm in ['data', 'conv', 'output']:
        entry = bytearray(64)
        entry[:len(nm)] = nm.encode('ascii')
        names += entry

    segment = build_segment(
        layers_desc=bytes(layers),
        weight_data=weight_blob,
        name_entries=bytes(names),
        src_num=1, dst_num=1,
        total_layers=total_layers,
        tmp_buf_size=tmp_buf_size,
        src_names=['data'],
        dst_names=['output'],
    )

    outer = build_outer_header(segment)
    oms_data = outer + segment

    with open(output_path, 'wb') as f:
        f.write(oms_data)

    print(f'Conv test OMS: {output_path} ({len(oms_data)} bytes)')
    print(f'  weight_data_off={weight_data_off} tmp_buf_size={tmp_buf_size}')
    return oms_data


def main():
    parser = argparse.ArgumentParser(
        description='Build HiSilicon IVE XNN .oms model files')
    parser.add_argument('input', nargs='?',
                        help='PyTorch model (.pt/.pth) to convert')
    parser.add_argument('output', help='Output .oms file path')
    parser.add_argument('--test', action='store_true',
                        help='Generate test model (Flatten+FC, random weights)')
    parser.add_argument('--conv-test', action='store_true',
                        help='Generate Conv test model (single 3×3 conv)')
    args = parser.parse_args()

    if args.conv_test:
        build_conv_test(args.output)
    elif args.test:
        build_test_model(args.output)
    elif args.input:
        build_from_pytorch(args.input, args.output)
    else:
        parser.error('Specify --test, --conv-test, or provide a PyTorch model')

    # Verify with parser
    print('\n--- Verification ---')
    from oms_parser import parse_oms, print_model
    model = parse_oms(args.output)
    print_model(model)


if __name__ == '__main__':
    main()
