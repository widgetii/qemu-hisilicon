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
    """Build a 48-byte Preproc layer descriptor."""
    d = bytearray(48)
    d[0] = 5  # type = Preproc
    d[4] = 3 if need_vgs else 0  # need_vgs (3 = VGS with specific mode)
    struct.pack_into('<H', d, 6, input_h)
    struct.pack_into('<H', d, 8, input_w)
    # Preproc uses a special encoding for blob type + stride info
    # Copy pattern from reference: bytes 0x0A..0x1F contain stride/format constants
    # For VGS mode with YUV420SP input:
    struct.pack_into('<H', d, 0x0A, 0x8404)  # stride constant from reference
    struct.pack_into('<H', d, 0x0C, 0x8404)
    struct.pack_into('<H', d, 0x0E, 0x8404)
    struct.pack_into('<H', d, 0x10, 0xFE00)
    struct.pack_into('<H', d, 0x12, 0xFE00)
    struct.pack_into('<H', d, 0x14, 0xFE00)
    struct.pack_into('<H', d, 0x16, input_h)
    struct.pack_into('<I', d, 0x18, align16(input_w) * input_c)
    d[0x2C] = 2  # blob_type = YVU420SP
    struct.pack_into('<I', d, 0x20, out_tmp_offset)
    return bytes(d)


def make_flatten(in_c, in_h, in_w, in_tmp_offset, out_tmp_offset,
                 layer_index):
    """Build a 48-byte Flatten layer descriptor."""
    d = bytearray(48)
    d[0] = 1  # type = Flatten
    d[1] = layer_index & 0xFF  # layer index in chain
    d[3] = 1  # unknown flag (always 1 in reference)
    d[4] = 1  # out_fmt = 1
    struct.pack_into('<H', d, 8, in_c)
    struct.pack_into('<H', d, 10, in_h)
    struct.pack_into('<H', d, 12, in_w)
    struct.pack_into('<I', d, 0x10, in_tmp_offset)
    struct.pack_into('<I', d, 0x14, out_tmp_offset)
    out_total = in_c * in_h * in_w
    in_stride_w = align16(in_w)
    out_stride_w = align16(out_total)
    struct.pack_into('<H', d, 0x18, in_stride_w)
    struct.pack_into('<H', d, 0x1A, out_stride_w)
    struct.pack_into('<I', d, 0x1C, in_h * in_stride_w)  # in_stride_c
    return bytes(d)


def make_fc(input_w, output_w, arg_len_cumulative, arg_offset,
            in_tmp_offset, out_tmp_offset, layer_index,
            af_mode=0, in_fmt=2, out_fmt=1):
    """Build a 64-byte FC layer descriptor."""
    d = bytearray(64)
    d[0] = 2  # type = FC
    d[1] = layer_index & 0xFF
    d[4] = in_fmt
    d[5] = out_fmt
    d[6] = 0  # batch_num
    d[7] = af_mode
    struct.pack_into('<H', d, 0x0A, input_w)
    struct.pack_into('<H', d, 0x0C, output_w)
    in_stride = align16(input_w)
    out_stride = align16(output_w * 4)  # output in int32 for accumulation
    struct.pack_into('<H', d, 0x0E, in_stride)
    struct.pack_into('<H', d, 0x10, out_stride)
    struct.pack_into('<I', d, 0x14, arg_len_cumulative)
    struct.pack_into('<I', d, 0x18, arg_offset)
    struct.pack_into('<I', d, 0x1C, in_tmp_offset)
    struct.pack_into('<I', d, 0x20, out_tmp_offset)
    return bytes(d)


def make_unpack(out_c, out_h, out_w, in_tmp_offset, out_tmp_offset,
                layer_index):
    """Build a 64-byte Unpack layer descriptor."""
    d = bytearray(64)
    d[0] = 4  # type = Unpack
    d[1] = layer_index & 0xFF
    d[4] = 1  # in_fmt
    d[5] = 0  # out_fmt
    struct.pack_into('<H', d, 8, out_c)
    struct.pack_into('<H', d, 10, out_h)
    struct.pack_into('<H', d, 12, out_w)
    in_stride = align16(out_w)
    out_stride = align16(out_w)
    struct.pack_into('<H', d, 0x0E, in_stride)
    struct.pack_into('<H', d, 0x10, out_stride)
    struct.pack_into('<I', d, 0x1C, in_tmp_offset)
    struct.pack_into('<I', d, 0x20, out_tmp_offset)
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
    weight_data_off = align16(layer_desc_end + 16)  # align with padding
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
    seg[22] = roi_batch
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
    # Dst node name IDs
    dst_name_off = 80 + align16(src_num * 2)
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

    # Pack weights: int8 weights (row-major) + int32 bias
    weight_blob = w_q.tobytes() + b_q.tobytes()
    weight_size = len(weight_blob)
    print(f'Weight blob: {weight_size} bytes ({w_q.nbytes} weights + {b_q.nbytes} bias)')

    # Compute tmp buffer size needed
    # Preproc output + flatten output + FC output
    preproc_out = align16(input_w) * input_h * input_c
    flatten_out = align16(flat_size)
    fc_out = align16(output_size * 4)  # int32 accumulation
    tmp_buf_size = preproc_out + flatten_out + fc_out + 4096  # extra margin

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
                     arg_offset=0,  # weights at start of weight blob
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


def main():
    parser = argparse.ArgumentParser(
        description='Build HiSilicon IVE XNN .oms model files')
    parser.add_argument('input', nargs='?',
                        help='PyTorch model (.pt/.pth) to convert')
    parser.add_argument('output', help='Output .oms file path')
    parser.add_argument('--test', action='store_true',
                        help='Generate test model (Flatten+FC, random weights)')
    args = parser.parse_args()

    if args.test:
        build_test_model(args.output)
    elif args.input:
        build_from_pytorch(args.input, args.output)
    else:
        parser.error('Specify --test or provide a PyTorch model path')

    # Verify with parser
    print('\n--- Verification ---')
    from oms_parser import parse_oms, print_model
    model = parse_oms(args.output)
    print_model(model)


if __name__ == '__main__':
    main()
