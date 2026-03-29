#!/usr/bin/env python3
"""
OMS Model Parser — HiSilicon IVE XNN neural network model format

Parses .oms files used by the IVE XNN CNN accelerator on Hi3516EV200/EV300.
Extracts network architecture, layer descriptors, weight statistics, and
layer name tables.

Format reverse-engineered from:
  - Ghidra decompilation of hi3516ev200_ive.ko (ive_xnn_loop_parse_every_layer)
  - Byte-level scanning of vendor and production models
  - Cross-validation against kernel validation constraints

Usage:
  python3 oms_parser.py model.oms
  python3 oms_parser.py model.oms --weights    # dump weight statistics
  python3 oms_parser.py model.oms --hex        # hex dump layer descriptors
  python3 oms_parser.py model.oms --json       # JSON output
"""

import struct
import sys
import json
import argparse
from dataclasses import dataclass, field, asdict
from typing import List, Optional


# Layer type enum
LAYER_TYPES = {0: 'Conv', 1: 'Flatten', 2: 'FC', 3: 'Eltwise',
               4: 'Unpack', 5: 'Preproc', 6: 'DMA'}
LAYER_SIZES = {0: 80, 1: 48, 2: 64, 3: 96, 4: 64, 5: 48, 6: 32}

POOL_MODES = {0: 'none', 1: 'max', 2: 'avg', 3: 'l2', 4: 'stochastic'}
AF_MODES = {0: 'none', 1: 'relu', 2: 'sigmoid', 3: 'tanh'}
BLOB_TYPES = {0: 'S32', 1: 'U8', 2: 'YVU420SP', 3: 'YVU422SP',
              4: 'VEC_S32', 5: 'SEQ_S32', 6: 'U16', 7: 'S8', 8: 'F32'}


@dataclass
class ConvLayer:
    type: str = 'Conv'
    in_fmt: int = 0
    out_fmt: int = 0
    pool_mode: int = 0
    af_mode: int = 0
    kernel_size: int = 0
    in_bond_num: int = 0
    input_c: int = 0
    input_h: int = 0
    input_w: int = 0
    output_c: int = 0
    output_h: int = 0
    output_w: int = 0
    arg_len: int = 0
    arg_offset: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0
    is_pad: int = 0
    in_stride_w: int = 0
    out_stride_w: int = 0
    in_stride_c: int = 0
    out_stride_c: int = 0


@dataclass
class FlattenLayer:
    type: str = 'Flatten'
    out_fmt: int = 0
    input_c: int = 0
    input_h: int = 0
    input_w: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0
    in_stride_w: int = 0
    out_stride_w: int = 0
    in_stride_c: int = 0


@dataclass
class FCLayer:
    type: str = 'FC'
    in_fmt: int = 0
    out_fmt: int = 0
    af_mode: int = 0
    input_w: int = 0
    output_w: int = 0
    arg_len: int = 0
    arg_offset: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0


@dataclass
class EltwiseLayer:
    type: str = 'Eltwise'
    in_fmt: int = 0
    out_fmt: int = 0
    in_num: int = 0
    eltwise_op: int = 0
    af_mode: int = 0
    output_c: int = 0
    output_h: int = 0
    output_w: int = 0
    arg_len: int = 0
    arg_offset: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0


@dataclass
class UnpackLayer:
    type: str = 'Unpack'
    in_fmt: int = 0
    out_fmt: int = 0
    output_c: int = 0
    output_h: int = 0
    output_w: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0
    arg_len: int = 0
    arg_offset: int = 0


@dataclass
class PreprocLayer:
    type: str = 'Preproc'
    need_vgs: int = 0
    input_c: int = 0
    input_h: int = 0
    input_w: int = 0
    blob_type: int = 0
    out_stride_w: int = 0
    out_stride_c: int = 0
    out_tmp_offset: int = 0


@dataclass
class DMALayer:
    type: str = 'DMA'
    in_width: int = 0
    in_height: int = 0
    in_stride: int = 0
    out_stride: int = 0
    in_tmp_offset: int = 0
    out_tmp_offset: int = 0


@dataclass
class LayerName:
    index: int = 0
    name: str = ''


@dataclass
class SegmentHeader:
    crc32: int = 0
    segment_size: int = 0
    model_buf_offset: int = 0
    tmp_buf_size: int = 0
    data_slice: int = 0
    roi_batch_num: int = 0
    total_scale_num: int = 0
    total_layer_num: int = 0
    src_num: int = 0
    dst_num: int = 0
    top_layer_num: int = 0
    weight_data_size: int = 0
    weight_data_offset: int = 0
    layer_name_offset: int = 0


@dataclass
class Segment:
    file_offset: int = 0
    header: SegmentHeader = field(default_factory=SegmentHeader)
    layers: List = field(default_factory=list)
    layer_names: List[LayerName] = field(default_factory=list)
    layer_desc_offset: int = 0


@dataclass
class OMSModel:
    filename: str = ''
    file_size: int = 0
    timestamp: str = ''
    segments: List[Segment] = field(default_factory=list)


def parse_segment_header(data: bytes) -> SegmentHeader:
    h = SegmentHeader()
    h.crc32 = struct.unpack_from('<I', data, 0)[0]
    h.segment_size = struct.unpack_from('<I', data, 4)[0]
    h.model_buf_offset = struct.unpack_from('<I', data, 8)[0]
    h.tmp_buf_size = struct.unpack_from('<I', data, 12)[0]
    h.data_slice = data[20]
    h.roi_batch_num = data[22]
    h.total_scale_num = struct.unpack_from('<H', data, 48)[0]
    h.total_layer_num = struct.unpack_from('<H', data, 50)[0]
    h.src_num = data[52]
    h.dst_num = data[53]
    h.top_layer_num = data[54]
    h.weight_data_size = struct.unpack_from('<I', data, 56)[0]
    h.weight_data_offset = struct.unpack_from('<I', data, 60)[0]
    h.layer_name_offset = struct.unpack_from('<I', data, 64)[0]
    return h


def calc_layer_desc_start(src_num: int, dst_num: int) -> int:
    v63 = ((-2 * (src_num + dst_num)) & 0xF) + 2 * (src_num + dst_num) + 80
    v61 = 32 * src_num + v63
    v70 = 32 * dst_num + v61
    return v70


def parse_conv(d: bytes) -> ConvLayer:
    c = ConvLayer()
    c.in_fmt = d[4]
    c.out_fmt = d[5]
    c.pool_mode = d[6]
    c.af_mode = d[7]
    c.input_c = struct.unpack_from('<H', d, 0x08)[0]
    c.input_h = struct.unpack_from('<H', d, 0x0A)[0]
    c.input_w = struct.unpack_from('<H', d, 0x0C)[0]
    c.output_c = struct.unpack_from('<H', d, 0x0E)[0]
    c.output_h = struct.unpack_from('<H', d, 0x10)[0]
    c.output_w = struct.unpack_from('<H', d, 0x12)[0]
    c.arg_len = struct.unpack_from('<I', d, 0x18)[0]
    c.arg_offset = struct.unpack_from('<I', d, 0x1C)[0]
    c.in_tmp_offset = struct.unpack_from('<I', d, 0x2C)[0]
    c.out_tmp_offset = struct.unpack_from('<I', d, 0x30)[0]
    c.in_stride_w = struct.unpack_from('<H', d, 0x34)[0]
    c.out_stride_w = struct.unpack_from('<H', d, 0x36)[0]
    c.in_stride_c = struct.unpack_from('<I', d, 0x38)[0]
    c.out_stride_c = struct.unpack_from('<I', d, 0x3C)[0]
    c.kernel_size = d[0x3D]
    c.in_bond_num = d[0x3E]
    return c


def parse_flatten(d: bytes) -> FlattenLayer:
    f = FlattenLayer()
    f.out_fmt = d[4]
    f.input_c = struct.unpack_from('<H', d, 8)[0]
    f.input_h = struct.unpack_from('<H', d, 10)[0]
    f.input_w = struct.unpack_from('<H', d, 12)[0]
    f.in_tmp_offset = struct.unpack_from('<I', d, 0x10)[0]
    f.out_tmp_offset = struct.unpack_from('<I', d, 0x14)[0]
    f.in_stride_w = struct.unpack_from('<H', d, 0x18)[0]
    f.out_stride_w = struct.unpack_from('<H', d, 0x1A)[0]
    f.in_stride_c = struct.unpack_from('<I', d, 0x1C)[0]
    return f


def parse_fc(d: bytes) -> FCLayer:
    f = FCLayer()
    f.in_fmt = d[4]
    f.out_fmt = d[5]
    f.af_mode = d[7]
    f.input_w = struct.unpack_from('<H', d, 0x0A)[0]
    f.output_w = struct.unpack_from('<H', d, 0x0C)[0]
    f.arg_len = struct.unpack_from('<I', d, 0x14)[0]
    f.arg_offset = struct.unpack_from('<I', d, 0x18)[0]
    f.in_tmp_offset = struct.unpack_from('<I', d, 0x1C)[0]
    f.out_tmp_offset = struct.unpack_from('<I', d, 0x20)[0]
    return f


def parse_eltwise(d: bytes) -> EltwiseLayer:
    e = EltwiseLayer()
    e.in_fmt = d[4]
    e.out_fmt = d[5]
    e.in_num = d[6]
    e.eltwise_op = d[7]
    e.output_c = struct.unpack_from('<H', d, 8)[0]
    e.output_h = struct.unpack_from('<H', d, 10)[0]
    e.output_w = struct.unpack_from('<H', d, 12)[0]
    e.arg_len = struct.unpack_from('<I', d, 0x3C)[0]
    e.arg_offset = struct.unpack_from('<I', d, 0x40)[0]
    e.in_tmp_offset = struct.unpack_from('<I', d, 0x1C)[0]
    e.out_tmp_offset = struct.unpack_from('<I', d, 0x20)[0]
    e.af_mode = d[0x39]
    return e


def parse_unpack(d: bytes) -> UnpackLayer:
    u = UnpackLayer()
    u.in_fmt = d[4]
    u.out_fmt = d[5]
    u.output_c = struct.unpack_from('<H', d, 8)[0]
    u.output_h = struct.unpack_from('<H', d, 10)[0]
    u.output_w = struct.unpack_from('<H', d, 12)[0]
    u.in_tmp_offset = struct.unpack_from('<I', d, 0x1C)[0]
    u.out_tmp_offset = struct.unpack_from('<I', d, 0x20)[0]
    u.arg_len = struct.unpack_from('<I', d, 0x34)[0]
    u.arg_offset = struct.unpack_from('<I', d, 0x38)[0]
    return u


def parse_preproc(d: bytes) -> PreprocLayer:
    p = PreprocLayer()
    p.need_vgs = d[4]
    p.input_c = struct.unpack_from('<H', d, 6)[0]
    p.input_h = struct.unpack_from('<H', d, 8)[0]
    p.input_w = struct.unpack_from('<H', d, 10)[0]
    p.out_stride_w = struct.unpack_from('<H', d, 0x18)[0]
    p.out_stride_c = struct.unpack_from('<I', d, 0x1C)[0]
    if len(d) > 0x2C:
        p.blob_type = d[0x2C]
    if len(d) > 0x30 + 4:
        p.out_tmp_offset = struct.unpack_from('<I', d, 0x30)[0]
    return p


def parse_dma(d: bytes) -> DMALayer:
    dm = DMALayer()
    dm.in_width = struct.unpack_from('<H', d, 6)[0]
    dm.in_height = struct.unpack_from('<H', d, 8)[0]
    dm.in_stride = struct.unpack_from('<H', d, 0x0A)[0]
    dm.out_stride = struct.unpack_from('<H', d, 0x0C)[0]
    dm.in_tmp_offset = struct.unpack_from('<I', d, 0x10)[0]
    dm.out_tmp_offset = struct.unpack_from('<I', d, 0x14)[0]
    return dm


PARSERS = {
    0: parse_conv, 1: parse_flatten, 2: parse_fc, 3: parse_eltwise,
    4: parse_unpack, 5: parse_preproc, 6: parse_dma,
}


def parse_layers(seg_data: bytes, header: SegmentHeader) -> list:
    start = calc_layer_desc_start(header.src_num, header.dst_num)
    ld = seg_data[start:]
    off = 0
    layers = []
    for _ in range(header.total_layer_num):
        if off >= len(ld):
            break
        lt = ld[off]
        sz = LAYER_SIZES.get(lt)
        if sz is None:
            break
        if off + sz > len(ld):
            break
        parser = PARSERS.get(lt)
        if parser:
            layer = parser(ld[off:off + sz])
        else:
            layer = {'type': f'unknown_{lt}'}
        layers.append(layer)
        off += sz
    return layers


def parse_layer_names(seg_data: bytes, header: SegmentHeader) -> List[LayerName]:
    off = header.layer_name_offset
    names = []
    for _ in range(header.total_layer_num + 10):
        if off + 0x40 > len(seg_data):
            break
        raw_name = seg_data[off:off + 32]
        name = raw_name.split(b'\x00')[0].decode('ascii', errors='replace')
        idx = struct.unpack_from('<I', seg_data, off + 0x30)[0]
        if name and all(c.isprintable() for c in name):
            names.append(LayerName(index=idx, name=name))
        off += 0x40
    return names


def parse_segment(data: bytes, file_offset: int) -> Segment:
    seg_data = data[file_offset:]
    seg = Segment()
    seg.file_offset = file_offset
    seg.header = parse_segment_header(seg_data)
    seg.layer_desc_offset = calc_layer_desc_start(
        seg.header.src_num, seg.header.dst_num)
    seg.layers = parse_layers(seg_data, seg.header)
    seg.layer_names = parse_layer_names(seg_data, seg.header)
    return seg


def parse_oms(filename: str) -> OMSModel:
    with open(filename, 'rb') as f:
        data = f.read()

    model = OMSModel()
    model.filename = filename
    model.file_size = len(data)

    # Scan for timestamp (16 ASCII digits YYMMDDHHMMSSxxxx)
    for ts_off in range(0x60, min(0x100, len(data) - 16)):
        ts = data[ts_off:ts_off + 16]
        if all(0x30 <= b <= 0x39 for b in ts):
            raw = ts.decode()
            model.timestamp = f'20{raw[0:2]}-{raw[2:4]}-{raw[4:6]} {raw[6:8]}:{raw[8:10]}:{raw[10:12]}'
            break

    # Scan for valid segment headers.
    # A valid segment has: total_scale_num [1,16] at +0x30,
    # total_layer_num [1,500] at +0x32, src_num [1,16] at +0x34, dst_num [1,16] at +0x35
    segments_found = []
    for off in range(0x40, min(len(data) - 80, 0x200), 0x10):
        scale = struct.unpack_from('<H', data, off + 48)[0]
        layers = struct.unpack_from('<H', data, off + 50)[0]
        src = data[off + 52]
        dst = data[off + 53]
        if 1 <= scale <= 16 and 1 <= layers <= 500 and 1 <= src <= 16 and 1 <= dst <= 16:
            seg_size = struct.unpack_from('<I', data, off + 4)[0]
            if seg_size < len(data) * 2:  # reasonable size
                segments_found.append(off)

    if not segments_found:
        return model

    seg1 = parse_segment(data, segments_found[0])
    model.segments.append(seg1)

    # Find segment 2 by scanning after segment 1 data
    seg1_end = segments_found[0] + seg1.header.segment_size
    for off in range(seg1_end - 0x100, min(len(data) - 80, seg1_end + 0x200), 0x10):
        if off <= segments_found[0]:
            continue
        scale = struct.unpack_from('<H', data, off + 48)[0]
        layers = struct.unpack_from('<H', data, off + 50)[0]
        src = data[off + 52]
        dst = data[off + 53]
        if 1 <= scale <= 16 and 1 <= layers <= 500 and 1 <= src <= 16 and 1 <= dst <= 16:
            seg2 = parse_segment(data, off)
            model.segments.append(seg2)
            break

    return model


def format_layer(i: int, layer, show_weights=False) -> str:
    if isinstance(layer, ConvLayer):
        k = layer.kernel_size
        pool = POOL_MODES.get(layer.pool_mode, f'{layer.pool_mode}')
        af = AF_MODES.get(layer.af_mode, f'{layer.af_mode}')
        s = (f'[{i:2d}] Conv {k}x{k}  '
             f'{layer.input_c:>4d}ch {layer.input_h:>3d}x{layer.input_w:>4d} -> '
             f'{layer.output_c:>4d}ch {layer.output_h:>3d}x{layer.output_w:>4d}  '
             f'pool={pool} af={af}')
        if show_weights:
            s += f'  weights={layer.arg_len}B @0x{layer.arg_offset:x}'
        return s
    elif isinstance(layer, FlattenLayer):
        return f'[{i:2d}] Flatten  {layer.input_c}ch {layer.input_h}x{layer.input_w}'
    elif isinstance(layer, FCLayer):
        af = AF_MODES.get(layer.af_mode, f'{layer.af_mode}')
        s = f'[{i:2d}] FC  {layer.input_w} -> {layer.output_w}  af={af}'
        if show_weights:
            s += f'  weights={layer.arg_len}B @0x{layer.arg_offset:x}'
        return s
    elif isinstance(layer, EltwiseLayer):
        return (f'[{i:2d}] Eltwise  in_num={layer.in_num} op={layer.eltwise_op} '
                f'{layer.output_c}ch {layer.output_h}x{layer.output_w}')
    elif isinstance(layer, UnpackLayer):
        return f'[{i:2d}] Unpack  {layer.output_c}ch {layer.output_h}x{layer.output_w}'
    elif isinstance(layer, PreprocLayer):
        mode = 'VGS' if layer.need_vgs else 'CPU'
        bt = BLOB_TYPES.get(layer.blob_type, f'{layer.blob_type}')
        return (f'[{i:2d}] Preproc({mode})  '
                f'{layer.input_c}ch {layer.input_h}x{layer.input_w}  blob={bt}')
    elif isinstance(layer, DMALayer):
        return f'[{i:2d}] DMA  {layer.in_width}x{layer.in_height}'
    else:
        return f'[{i:2d}] {layer}'


def print_model(model: OMSModel, show_weights=False, show_hex=False):
    print(f'File: {model.filename}')
    print(f'Size: {model.file_size} bytes ({model.file_size/1024:.1f} KB)')
    if model.timestamp:
        print(f'Built: {model.timestamp}')
    print(f'Segments: {len(model.segments)}')
    print()

    for si, seg in enumerate(model.segments):
        h = seg.header
        print(f'=== Segment {si+1} (file offset 0x{seg.file_offset:x}) ===')
        print(f'  CRC32:           0x{h.crc32:08x}')
        print(f'  Segment size:    {h.segment_size} bytes')
        print(f'  Tmp buffer:      {h.tmp_buf_size} bytes ({h.tmp_buf_size/1024:.0f} KB)')
        print(f'  Scales:          {h.total_scale_num}')
        print(f'  Layers:          {h.total_layer_num}')
        print(f'  Inputs:          {h.src_num}')
        print(f'  Outputs:         {h.dst_num}')
        print(f'  Data slice:      {h.data_slice}')
        print(f'  ROI batch:       {h.roi_batch_num}')
        print(f'  Weight data:     {h.weight_data_size} bytes at offset 0x{h.weight_data_offset:x}')
        print(f'  Layer names:     at offset 0x{h.layer_name_offset:x}')
        print(f'  Layer descs:     at offset 0x{seg.layer_desc_offset:x}')
        print()

        # Count layer types
        type_counts = {}
        for layer in seg.layers:
            t = layer.type if hasattr(layer, 'type') else 'unknown'
            type_counts[t] = type_counts.get(t, 0) + 1
        print(f'  Layer summary: {", ".join(f"{v}x {k}" for k, v in type_counts.items())}')
        print()

        print(f'  Layers:')
        for i, layer in enumerate(seg.layers):
            print(f'    {format_layer(i, layer, show_weights)}')

        if seg.layer_names:
            print()
            print(f'  Layer names:')
            for ln in seg.layer_names:
                print(f'    [{ln.index:2d}] {ln.name}')

        print()


def weight_stats(model: OMSModel):
    """Print weight data statistics for each conv/fc layer."""
    import numpy as np

    for si, seg in enumerate(model.segments):
        with open(model.filename, 'rb') as f:
            f.seek(seg.file_offset)
            seg_data = f.read(seg.header.segment_size + 100)

        print(f'=== Segment {si+1} Weight Statistics ===')
        prev_arg_len = 0
        for i, layer in enumerate(seg.layers):
            if isinstance(layer, (ConvLayer, FCLayer)):
                delta = layer.arg_len - prev_arg_len
                off = layer.arg_offset
                if off + delta <= len(seg_data) and delta > 0:
                    w = np.frombuffer(seg_data[off:off + delta], dtype=np.int8)
                    nz = (w == 0).sum()
                    print(f'  [{i:2d}] {layer.type:7s} '
                          f'off=0x{off:06x} size={delta:>7d}  '
                          f'min={w.min():>4d} max={w.max():>4d} '
                          f'mean={w.mean():>6.1f} std={w.std():>5.1f} '
                          f'zeros={100*nz/len(w):>4.0f}%')
                prev_arg_len = layer.arg_len
        print()


def main():
    parser = argparse.ArgumentParser(
        description='Parse HiSilicon IVE XNN .oms model files')
    parser.add_argument('model', help='Path to .oms model file')
    parser.add_argument('--weights', action='store_true',
                        help='Show weight statistics (requires numpy)')
    parser.add_argument('--hex', action='store_true',
                        help='Show raw hex of layer descriptors')
    parser.add_argument('--json', action='store_true',
                        help='Output as JSON')
    args = parser.parse_args()

    model = parse_oms(args.model)

    if args.json:
        out = {
            'filename': model.filename,
            'file_size': model.file_size,
            'timestamp': model.timestamp,
            'segments': []
        }
        for seg in model.segments:
            seg_out = {
                'file_offset': seg.file_offset,
                'header': asdict(seg.header),
                'layers': [asdict(l) for l in seg.layers],
                'layer_names': [asdict(n) for n in seg.layer_names],
            }
            out['segments'].append(seg_out)
        json.dump(out, sys.stdout, indent=2)
        print()
    else:
        print_model(model, show_weights=args.weights or args.hex, show_hex=args.hex)

    if args.weights:
        try:
            weight_stats(model)
        except ImportError:
            print('numpy required for --weights', file=sys.stderr)


if __name__ == '__main__':
    main()
