#!/bin/bash
#
# Set up QEMU build tree with HiSilicon CV300/EV300 SoC emulation.
# Run from the repository root:
#
#   bash qemu/setup.sh
#
# This will:
#   1. Clone QEMU 10.2.0 into qemu-src/
#   2. Copy all HiSilicon source files
#   3. Patch Kconfig and meson.build
#   4. Configure and build (arm target only)
#
set -e

QEMU_VERSION="v10.2.0"
QEMU_DIR="qemu-src"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$REPO_ROOT"

# ── 1. Clone QEMU ──────────────────────────────────────────────────────
if [ -d "$QEMU_DIR" ]; then
    echo "qemu-src/ already exists, skipping clone"
else
    echo "Cloning QEMU $QEMU_VERSION..."
    git clone --depth 1 --branch "$QEMU_VERSION" \
        https://gitlab.com/qemu-project/qemu.git "$QEMU_DIR"
fi

# ── 2. Copy source files ───────────────────────────────────────────────
echo "Copying HiSilicon SoC sources..."

cp qemu/hw/arm/hisilicon.c          "$QEMU_DIR/hw/arm/"
cp qemu/include/hw/arm/hisilicon.h  "$QEMU_DIR/include/hw/arm/"
cp qemu/hw/misc/hisi-sysctl.c       "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-crg.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-fmc.c          "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-himci.c        "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-regbank.c      "$QEMU_DIR/hw/misc/"
cp qemu/hw/misc/hisi-vedu.c         "$QEMU_DIR/hw/misc/"
cp qemu/hw/net/hisi-femac.c         "$QEMU_DIR/hw/net/"
cp qemu/hw/i2c/hisi-i2c.c          "$QEMU_DIR/hw/i2c/"
cp qemu/hw/i2c/hisi-imx335.c       "$QEMU_DIR/hw/i2c/"

# ── 3. Patch build system ──────────────────────────────────────────────
echo "Patching build system..."

# hw/arm/Kconfig
if ! grep -q HISILICON "$QEMU_DIR/hw/arm/Kconfig"; then
    cat >> "$QEMU_DIR/hw/arm/Kconfig" <<'KCONFIG'

config HISILICON
    bool
    default y
    depends on TCG && ARM
    select ARM_GIC
    select PL190
    select PL011
    select ARM_TIMER
    select PL061
    select PL022
    select PL080
    select UNIMP
    select SDHCI
    select HISI_MISC
    select HISI_FEMAC
    select I2C
    select HISI_I2C
KCONFIG
    echo "  patched hw/arm/Kconfig"
else
    echo "  hw/arm/Kconfig already patched"
fi

# hw/arm/meson.build
if ! grep -q hisilicon "$QEMU_DIR/hw/arm/meson.build"; then
    echo "arm_ss.add(when: 'CONFIG_HISILICON', if_true: files('hisilicon.c'))" \
        >> "$QEMU_DIR/hw/arm/meson.build"
    echo "  patched hw/arm/meson.build"
else
    echo "  hw/arm/meson.build already patched"
fi

# hw/misc/Kconfig
if ! grep -q HISI_MISC "$QEMU_DIR/hw/misc/Kconfig"; then
    cat >> "$QEMU_DIR/hw/misc/Kconfig" <<'KCONFIG'

config HISI_MISC
    bool
KCONFIG
    echo "  patched hw/misc/Kconfig"
else
    echo "  hw/misc/Kconfig already patched"
fi

# hw/misc/meson.build
if ! grep -q hisi-sysctl "$QEMU_DIR/hw/misc/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_MISC', if_true: files('hisi-sysctl.c', 'hisi-crg.c', 'hisi-fmc.c', 'hisi-himci.c', 'hisi-regbank.c', 'hisi-vedu.c'))" \
        >> "$QEMU_DIR/hw/misc/meson.build"
    echo "  patched hw/misc/meson.build"
else
    echo "  hw/misc/meson.build already patched"
fi

# hw/net/Kconfig
if ! grep -q HISI_FEMAC "$QEMU_DIR/hw/net/Kconfig"; then
    cat >> "$QEMU_DIR/hw/net/Kconfig" <<'KCONFIG'

config HISI_FEMAC
    bool
KCONFIG
    echo "  patched hw/net/Kconfig"
else
    echo "  hw/net/Kconfig already patched"
fi

# hw/net/meson.build
if ! grep -q hisi-femac "$QEMU_DIR/hw/net/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_FEMAC', if_true: files('hisi-femac.c'))" \
        >> "$QEMU_DIR/hw/net/meson.build"
    echo "  patched hw/net/meson.build"
else
    echo "  hw/net/meson.build already patched"
fi

# hw/i2c/Kconfig
if ! grep -q HISI_I2C "$QEMU_DIR/hw/i2c/Kconfig"; then
    cat >> "$QEMU_DIR/hw/i2c/Kconfig" <<'KCONFIG'

config HISI_I2C
    bool
    select I2C
KCONFIG
    echo "  patched hw/i2c/Kconfig"
else
    echo "  hw/i2c/Kconfig already patched"
fi

# hw/i2c/meson.build
if ! grep -q hisi-i2c "$QEMU_DIR/hw/i2c/meson.build"; then
    echo "system_ss.add(when: 'CONFIG_HISI_I2C', if_true: files('hisi-i2c.c', 'hisi-imx335.c'))" \
        >> "$QEMU_DIR/hw/i2c/meson.build"
    echo "  patched hw/i2c/meson.build"
else
    echo "  hw/i2c/meson.build already patched"
fi

# ── 4. Patch QEMU SD card model to accept 1.8V ─────────────────────────
# The EV300 kernel DT forces UHS-I / HS200 1.8V signaling.  QEMU's SD
# card model rejects 1.8V by default.  Patch it to accept 1.8V and
# advertise the capability in OCR so card enumeration succeeds.
if ! grep -q '1701.*2000' "$QEMU_DIR/hw/sd/sd.c"; then
    sed -i '/case 2001 \.\.\. 3000:/i\    case 1701 ... 2000: /* SD_VOLTAGE_1_8V */' \
        "$QEMU_DIR/hw/sd/sd.c"
    sed -i 's/sd->ocr = R_OCR_VDD_VOLTAGE_WIN_HI_MASK;/sd->ocr = R_OCR_VDD_VOLTAGE_WIN_HI_MASK | R_OCR_ACCEPT_SWITCH_1V8_MASK;/' \
        "$QEMU_DIR/hw/sd/sd.c"
    echo "  patched hw/sd/sd.c (1.8V voltage support)"
else
    echo "  hw/sd/sd.c already patched"
fi

# ── 5. Build ────────────────────────────────────────────────────────────
echo "Building QEMU (arm target)..."
cd "$QEMU_DIR"
mkdir -p build && cd build
../configure --target-list=arm-softmmu --disable-docs
ninja -j$(nproc)

echo ""
echo "Done! Verify:"
echo "  ./qemu-src/build/qemu-system-arm -machine help | grep hi3516"
