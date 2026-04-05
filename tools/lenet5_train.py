#!/usr/bin/env python3
"""
LeNet-5-3x3: Quantization-Aware Training for HiSilicon IVE XNN.

Trains directly in INT8 (simulated quantization) on MNIST using PyTorch's
QAT framework. The IVE XNN hardware only supports 3×3 and 1×1 kernels,
so we replace each 5×5 conv with two stacked 3×3 convs.

Architecture:
  Conv1: 3×3, 1→6, relu          → 30×30×6
  Conv2: 3×3, 6→6, relu, maxpool → 14×14×6
  Conv3: 3×3, 6→16, relu         → 12×12×16
  Conv4: 3×3, 16→16, relu, maxpool → 5×5×16
  FC1:   400→120, relu
  FC2:   120→84, relu
  FC3:   84→10

Usage:
  python3 lenet5_train.py                       # QAT train + export
  python3 lenet5_train.py --load weights.npz    # test only
"""

import argparse
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.ao.quantization import (
    get_default_qat_qconfig, prepare_qat, convert, QuantStub, DeQuantStub
)
from torchvision import datasets, transforms


class LeNet5_3x3(nn.Module):
    def __init__(self):
        super().__init__()
        self.quant = QuantStub()
        self.conv1 = nn.Conv2d(1, 6, 3)
        self.relu1 = nn.ReLU()
        self.conv2 = nn.Conv2d(6, 6, 3)
        self.relu2 = nn.ReLU()
        self.pool1 = nn.MaxPool2d(2, 2)
        self.conv3 = nn.Conv2d(6, 16, 3)
        self.relu3 = nn.ReLU()
        self.conv4 = nn.Conv2d(16, 16, 3)
        self.relu4 = nn.ReLU()
        self.pool2 = nn.MaxPool2d(2, 2)
        self.fc1 = nn.Linear(16 * 5 * 5, 120)
        self.relu5 = nn.ReLU()
        self.fc2 = nn.Linear(120, 84)
        self.relu6 = nn.ReLU()
        self.fc3 = nn.Linear(84, 10)
        self.dequant = DeQuantStub()

    def forward(self, x):
        x = self.quant(x)
        x = self.relu1(self.conv1(x))
        x = self.pool1(self.relu2(self.conv2(x)))
        x = self.relu3(self.conv3(x))
        x = self.pool2(self.relu4(self.conv4(x)))
        x = x.view(-1, 16 * 5 * 5)
        x = self.relu5(self.fc1(x))
        x = self.relu6(self.fc2(x))
        x = self.fc3(x)
        x = self.dequant(x)
        return x


def get_data():
    transform = transforms.Compose([
        transforms.Resize(32),
        transforms.ToTensor(),
        transforms.Normalize((0.1307,), (0.3081,))
    ])
    train_data = datasets.MNIST('data', train=True, download=True, transform=transform)
    test_data = datasets.MNIST('data', train=False, transform=transform)
    train_loader = torch.utils.data.DataLoader(train_data, batch_size=64, shuffle=True)
    test_loader = torch.utils.data.DataLoader(test_data, batch_size=1000)
    return train_loader, test_loader


def evaluate(model, test_loader, device='cpu'):
    model.eval()
    correct = total = 0
    with torch.no_grad():
        for data, target in test_loader:
            data, target = data.to(device), target.to(device)
            pred = model(data).argmax(dim=1)
            correct += pred.eq(target).sum().item()
            total += target.size(0)
    return 100. * correct / total


def train_qat(epochs=15):
    """Quantization-Aware Training: train with simulated INT8 quantization."""
    train_loader, test_loader = get_data()
    model = LeNet5_3x3()

    # Phase 1: Float training (5 epochs for warm-up)
    print("Phase 1: Float32 warm-up training...")
    optimizer = optim.Adam(model.parameters(), lr=0.001)
    criterion = nn.CrossEntropyLoss()

    for epoch in range(min(5, epochs)):
        model.train()
        for data, target in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(data), target)
            loss.backward()
            optimizer.step()
        acc = evaluate(model, test_loader)
        print(f'  Epoch {epoch+1}: {acc:.2f}%')

    # Phase 2: QAT fine-tuning
    print("\nPhase 2: Quantization-Aware Training...")
    model.train()
    model.qconfig = get_default_qat_qconfig('fbgemm')
    prepare_qat(model, inplace=True)

    optimizer = optim.Adam(model.parameters(), lr=0.0001)
    for epoch in range(max(epochs - 5, 5)):
        model.train()
        for data, target in train_loader:
            optimizer.zero_grad()
            loss = criterion(model(data), target)
            loss.backward()
            optimizer.step()
        acc = evaluate(model, test_loader)
        print(f'  QAT Epoch {epoch+1}: {acc:.2f}%')

    # Convert to quantized
    model.eval()
    quantized = convert(model)
    qacc = evaluate(quantized, test_loader)
    print(f'\nQuantized accuracy: {qacc:.2f}%')

    return model, quantized


def extract_int8_weights(model):
    """Extract INT8 weights and scales from QAT-trained model."""
    layers = {}

    for name in ['conv1', 'conv2', 'conv3', 'conv4', 'fc1', 'fc2', 'fc3']:
        m = getattr(model, name)
        w = m.weight().detach().cpu()
        # Get quantization params
        w_scale = w.q_per_channel_scales().numpy() if w.qscheme() == torch.per_channel_affine else np.array([w.q_scale()])
        w_zp = w.q_per_channel_zero_points().numpy() if w.qscheme() == torch.per_channel_affine else np.array([w.q_zero_point()])
        w_int = w.int_repr().numpy()

        b = m.bias().detach().cpu()
        b_int = b.int_repr().numpy() if hasattr(b, 'int_repr') else b.numpy()

        layers[f'{name}_weight'] = w_int.astype(np.int8)
        layers[f'{name}_bias'] = b_int.astype(np.int32)
        layers[f'{name}_w_scale'] = w_scale.astype(np.float32)

        print(f'{name}: w_shape={w_int.shape} w_range=[{w_int.min()},{w_int.max()}] '
              f'b_range=[{b_int.min()},{b_int.max()}] scale={w_scale.mean():.6f}')

    return layers


def extract_int8_weights_manual(float_model):
    """Manual symmetric INT8 quantization from float model."""
    layers = {}

    for name in ['conv1', 'conv2', 'conv3', 'conv4', 'fc1', 'fc2', 'fc3']:
        m = getattr(float_model, name)
        w = m.weight.detach().cpu().numpy()
        b = m.bias.detach().cpu().numpy()

        w_max = max(abs(w.min()), abs(w.max()), 1e-8)
        scale = w_max / 127.0
        w_q = np.clip(np.round(w / scale), -128, 127).astype(np.int8)
        b_q = np.round(b / scale).astype(np.int32)

        layers[f'{name}_weight'] = w_q
        layers[f'{name}_bias'] = b_q
        layers[f'{name}_w_scale'] = np.float32(scale)

        print(f'{name}: shape={w_q.shape} scale={scale:.6f} '
              f'w=[{w_q.min()},{w_q.max()}] b=[{b_q.min()},{b_q.max()}]')

    return layers


def int8_conv2d(x, w, b, pool=False, shift=0):
    """INT8 convolution: int8×int8 → int32 → relu → >>shift → clamp int8.

    The HW accumulates in int32, applies ReLU, right-shifts by a fixed
    amount (determined at compile time), then clamps to int8 for the
    next layer. The shift is stored per-layer in the model.
    """
    in_c, in_h, in_w = x.shape
    out_c, _, kh, kw = w.shape
    oh, ow = in_h - kh + 1, in_w - kw + 1

    # Use numpy einsum for speed instead of nested loops
    # Unfold input into patches
    patches = np.zeros((oh, ow, in_c, kh, kw), dtype=np.int16)
    for ky in range(kh):
        for kx in range(kw):
            patches[:, :, :, ky, kx] = x[:, ky:ky+oh, kx:kx+ow].transpose(1, 2, 0)

    patches_flat = patches.reshape(oh * ow, in_c * kh * kw)
    w_flat = w.reshape(out_c, in_c * kh * kw)

    # int16 × int8 matmul → int32 (avoid overflow with int16 patches)
    out = patches_flat.astype(np.int32) @ w_flat.astype(np.int32).T  # (oh*ow, out_c)
    out = out.T.reshape(out_c, oh, ow)  # (out_c, oh, ow)
    out += b.reshape(-1, 1, 1)  # add bias

    out = np.maximum(out, 0)  # ReLU

    # Right-shift and clamp to int8
    if shift > 0:
        out = out >> shift
    out = np.clip(out, -128, 127).astype(np.int8)

    if pool:
        _, ph, pw = out.shape
        out = out.reshape(out_c, ph//2, 2, pw//2, 2).max(axis=(2, 4))

    return out


def int8_fc(x, w, b):
    """INT8 FC: int8×int8 → int32 >> 19 → int16."""
    out = np.zeros(w.shape[0], dtype=np.int32)
    for j in range(w.shape[0]):
        acc = int(b[j])
        for i in range(w.shape[1]):
            acc += int(x[i]) * int(w[j, i])
        out[j] = acc
    return (out >> 19).astype(np.int16)


def calibrate_conv_shifts(layers, cal_images, n=50):
    """Determine per-layer shift by running calibration images."""
    shifts = {}
    for name in ['conv1', 'conv2', 'conv3', 'conv4']:
        shifts[name] = 0  # start with no shift

    for idx in range(min(n, len(cal_images))):
        img, _ = cal_images[idx]
        pixels = (img.numpy()[0] * 255).astype(np.uint8)
        x = np.clip(np.round((pixels / 255.0 - 0.1307) / 0.3081 * 45),
                    -128, 127).astype(np.int8).reshape(1, 32, 32)

        for name, pool in [('conv1', False), ('conv2', True),
                           ('conv3', False), ('conv4', True)]:
            w, b = layers[f'{name}_weight'], layers[f'{name}_bias']
            in_c, in_h, in_w = x.shape
            out_c, _, kh, kw = w.shape
            oh, ow = in_h - kh + 1, in_w - kw + 1

            patches = np.zeros((oh, ow, in_c, kh, kw), dtype=np.int16)
            for ky in range(kh):
                for kx in range(kw):
                    patches[:, :, :, ky, kx] = x[:, ky:ky+oh, kx:kx+ow].transpose(1,2,0)
            pf = patches.reshape(oh*ow, in_c*kh*kw)
            wf = w.reshape(out_c, in_c*kh*kw)
            out = pf.astype(np.int32) @ wf.astype(np.int32).T
            out = out.T.reshape(out_c, oh, ow) + b.reshape(-1,1,1)
            out = np.maximum(out, 0)

            mx = max(abs(out.max()), abs(out.min()), 1)
            need_shift = max(0, int(np.ceil(np.log2(mx / 127))))
            shifts[name] = max(shifts[name], need_shift)

            out_q = np.clip(out >> shifts[name], -128, 127).astype(np.int8)
            if pool:
                _, ph, pw = out_q.shape
                out_q = out_q.reshape(out_c, ph//2, 2, pw//2, 2).max(axis=(2,4))
            x = out_q

    return shifts


def int8_inference(layers, image_u8, shifts=None):
    """Full INT8 inference pipeline with calibrated shifts."""
    if shifts is None:
        shifts = {'conv1': 7, 'conv2': 7, 'conv3': 7, 'conv4': 7}

    # MNIST normalization: (pixel/255 - 0.1307) / 0.3081, scaled to int8
    # Scale factor 45 maps normalized range [-0.42, 2.82] to [-19, 127]
    x = np.clip(np.round((image_u8 / 255.0 - 0.1307) / 0.3081 * 45),
                -128, 127).astype(np.int8).reshape(1, 32, 32)

    x = int8_conv2d(x, layers['conv1_weight'], layers['conv1_bias'],
                    shift=shifts['conv1'])
    x = int8_conv2d(x, layers['conv2_weight'], layers['conv2_bias'],
                    pool=True, shift=shifts['conv2'])
    x = int8_conv2d(x, layers['conv3_weight'], layers['conv3_bias'],
                    shift=shifts['conv3'])
    x = int8_conv2d(x, layers['conv4_weight'], layers['conv4_bias'],
                    pool=True, shift=shifts['conv4'])

    x = x.reshape(-1)
    x = int8_fc(x, layers['fc1_weight'], layers['fc1_bias'])
    x = np.maximum(x, 0).astype(np.int8)
    x = int8_fc(x, layers['fc2_weight'], layers['fc2_bias'])
    x = np.maximum(x, 0).astype(np.int8)
    x = int8_fc(x, layers['fc3_weight'], layers['fc3_bias'])
    return x


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load', help='Load weights .npz')
    parser.add_argument('--epochs', type=int, default=15)
    parser.add_argument('--output', default='lenet5_weights.npz')
    args = parser.parse_args()

    if args.load:
        data = np.load(args.load)
        layers = {k: data[k] for k in data.files}
    else:
        print("Training LeNet5-3x3 with QAT on MNIST...")
        try:
            float_model, quant_model = train_qat(args.epochs)
            print("\nExtracting INT8 weights from QAT model...")
            layers = extract_int8_weights(quant_model)
        except Exception as e:
            print(f"QAT failed ({e}), falling back to manual quantization...")
            model = LeNet5_3x3()
            train_loader, test_loader = get_data()
            optimizer = optim.Adam(model.parameters(), lr=0.001)
            criterion = nn.CrossEntropyLoss()
            for epoch in range(args.epochs):
                model.train()
                for data, target in train_loader:
                    optimizer.zero_grad()
                    loss = criterion(model(data), target)
                    loss.backward()
                    optimizer.step()
                acc = evaluate(model, test_loader)
                print(f'Epoch {epoch+1}: {acc:.2f}%')
            print("\nManual INT8 quantization...")
            layers = extract_int8_weights_manual(model)

        np.savez(args.output, **layers)
        print(f'\nSaved to {args.output}')

    # Calibrate Conv shifts
    print('\nCalibrating Conv output shifts...')
    raw_transform = transforms.Compose([transforms.Resize(32), transforms.ToTensor()])
    cal_data = datasets.MNIST('data', train=False, download=True, transform=raw_transform)
    shifts = calibrate_conv_shifts(layers, cal_data, n=50)
    print(f'Shifts: {shifts}')

    # Save shifts with weights
    if not args.load:
        np.savez(args.output, **layers, **{f'shift_{k}': v for k, v in shifts.items()})
        print(f'Updated {args.output} with shifts')

    # INT8 inference test
    print('\nINT8 inference on MNIST test set:')
    correct = total = 0
    for idx in range(min(1000, len(cal_data))):
        img, label = cal_data[idx]
        pixels = (img.numpy()[0] * 255).astype(np.uint8)
        output = int8_inference(layers, pixels, shifts)
        pred = int(output.argmax())
        if pred == label:
            correct += 1
        total += 1
        if idx < 10:
            print(f'  [{idx}] label={label} pred={pred} output={output.tolist()}')

    print(f'\nINT8 accuracy: {correct}/{total} = {100*correct/total:.1f}%')


if __name__ == '__main__':
    main()
