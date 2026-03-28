#!/bin/bash
#
# End-to-end IVE motion detection demo:
#  1. Generate synthetic CCTV scene
#  2. Run host reference algorithm → bboxes
#  3. Build + run QEMU IVE test → bboxes
#  4. Compare host vs QEMU output
#  5. Create annotated demo video
#
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
QEMU="$REPO_ROOT/qemu-src/build/qemu-system-arm"
CC="${CROSS_COMPILE:-$HOME/git/firmware/output-hi3516ev300_lite/host/bin/arm-openipc-linux-musleabi-}gcc"

echo "=== Step 1: Generate synthetic CCTV scene ==="
python3 "$SCRIPT_DIR/generate_scene.py" "$SCRIPT_DIR/frames"

echo ""
echo "=== Step 2: Host reference motion detection ==="
python3 "$SCRIPT_DIR/ive_demo.py" \
    --frames-dir "$SCRIPT_DIR/frames" \
    --output-dir "$SCRIPT_DIR/output" \
    --visualize \
    > "$SCRIPT_DIR/output/host_bboxes.txt" 2>/dev/null

echo "  Host bboxes: $(wc -l < "$SCRIPT_DIR/output/host_bboxes.txt") frames with motion"

echo ""
echo "=== Step 3: Build + run QEMU IVE test ==="

# Build ARM test binary
if [ ! -x "$CC" ]; then
    echo "  ERROR: cross-compiler not found: $CC"
    echo "  Set CROSS_COMPILE=path/to/arm-...- or install the toolchain"
    exit 1
fi

TESTBIN="/tmp/test-ive-video"
"$CC" -static -O2 -o "$TESTBIN" "$REPO_ROOT/qemu-boot/test-ive-video.c" 2>/dev/null

# Create initramfs with test binary + frames
INITRD_DIR="/tmp/ive-demo-initrd"
rm -rf "$INITRD_DIR"
mkdir -p "$INITRD_DIR"
cp "$TESTBIN" "$INITRD_DIR/init"
chmod +x "$INITRD_DIR/init"

# Embed first 30 frames as raw data (keep initrd small)
head -c $((320*240*30)) "$SCRIPT_DIR/frames/frames.bin" > "$INITRD_DIR/frames.bin"

cd "$INITRD_DIR"
INITRD="/tmp/ive-demo.gz"
find . | cpio -o -H newc 2>/dev/null | gzip > "$INITRD"
cd "$REPO_ROOT"

# Run QEMU
if [ ! -x "$QEMU" ]; then
    echo "  ERROR: QEMU not built. Run: bash qemu/setup.sh"
    exit 1
fi

timeout 60 "$QEMU" -M hi3516ev300 -m 128M \
    -kernel "qemu-boot/uImage.hi3516ev300" \
    -initrd "$INITRD" \
    -nographic -serial file:"$SCRIPT_DIR/output/qemu_serial.log" \
    -append "console=ttyAMA0,115200 mem=128M root=/dev/ram0 rdinit=/init" \
    2>/dev/null || true

# Extract bbox lines from QEMU serial output
grep "^FRAME " "$SCRIPT_DIR/output/qemu_serial.log" > "$SCRIPT_DIR/output/qemu_bboxes.txt" 2>/dev/null || true
echo "  QEMU bboxes: $(wc -l < "$SCRIPT_DIR/output/qemu_bboxes.txt") frames with motion"

echo ""
echo "=== Step 4: Compare host vs QEMU ==="
# Compare only first 29 frames (QEMU processes 30 frames, first has no prev)
head -29 "$SCRIPT_DIR/output/host_bboxes.txt" > /tmp/host_first29.txt
if diff -q "$SCRIPT_DIR/output/qemu_bboxes.txt" /tmp/host_first29.txt >/dev/null 2>&1; then
    echo "  MATCH: host and QEMU produce identical bounding boxes"
else
    echo "  DIFFERENCE detected:"
    diff "$SCRIPT_DIR/output/qemu_bboxes.txt" /tmp/host_first29.txt || true
fi

echo ""
echo "=== Step 5: Demo video ==="
echo "  Output: $SCRIPT_DIR/output/demo_output.mp4"
echo "  Frames: $SCRIPT_DIR/output/frame_NNNN.png (150 annotated frames)"
echo ""
echo "Done!"
