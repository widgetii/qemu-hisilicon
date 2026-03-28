# QEMU HiSilicon IP Camera SoC Emulator

QEMU machine definitions for HiSilicon IP camera SoCs (V1 through V5 generations),
targeting QEMU v10.2.0. Boots unmodified [OpenIPC](https://openipc.org/) firmware
to a full Linux userspace on all supported platforms.

## Supported Machines (17 total)

| Machine | Generation | CPU | IRQ | Kernel | Boot tested |
|---------|-----------|-----|-----|--------|-------------|
| `hi3516cv100` | V1 | ARM926EJ-S | VIC | 3.0.8 | yes |
| `hi3516cv200` | V2 | ARM926EJ-S | VIC | 4.9.37 | yes |
| `hi3516av100` | V2A | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3516cv300` | V3 | ARM926EJ-S | VIC | 3.18.20 | yes |
| `hi3516cv500` | V3.5 | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3519v101` | V3A | Cortex-A7 | GIC | 3.18.20 | yes |
| `hi3516ev300` | V4 | Cortex-A7 | GIC | 4.9.37 | yes |
| `hi3516ev200` | V4 | Cortex-A7 | GIC | — | — |
| `hi3518ev300` | V4 | Cortex-A7 | GIC | — | — |
| `hi3516dv200` | V4 | Cortex-A7 | GIC | — | — |
| `gk7205v200` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7205v300` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7202v300` | V4/Goke | Cortex-A7 | GIC | — | — |
| `gk7605v100` | V4/Goke | Cortex-A7 | GIC | — | — |
| `hi3516cv608` | V5 | Cortex-A7 MP2 | GIC | 5.10 | — |
| `hi3516cv610` | **V5** | Cortex-A7 MP2 | GIC | **5.10** | yes |
| `hi3516cv613` | V5 | Cortex-A7 MP2 | GIC | 5.10 | — |

### V5 Model Suffix → Chip ID Mapping

From datasheet Section 1.2.14 and lab identification:

| Model | Chip | SoC ID | NPU | Max Res | DDR | Package |
|-------|------|--------|-----|---------|-----|---------|
| — | Hi3516CV608 | 0x3516C608 | 0.2 TOPS | 3M | DDR2 512Mb | QFN |
| 10B | Hi3516CV610 | 0x3516C610 | 0.5 TOPS | 5M | DDR2 512Mb | QFN |
| 20S | Hi3516CV613 | 0x3516C613 | 1 TOPS | 4K | DDR3 1Gb | QFN |
| 00S | unknown | — | 1 TOPS | 4K | DDR3 1Gb | QFN |
| 20G | unknown | — | 1 TOPS | 4K | DDR3 1Gb | QFN, GB35114 |
| 00G | unknown | — | 1 TOPS | 4K | ext DDR3 4Gb | BGA, GB35114 |

ipctool also lists Hi3516DV500 (`0x3516D500`) and Hi3519DV500 (`0x3519D500`)
as `HISI_OT` generation — likely same V5 address map, awaiting SDK/lab confirmation.

## Peripheral Support Matrix

| Peripheral | CV100 | CV200 | AV100 | CV300 | CV500 | 3519V101 | V4 (×8) | **V5 (×3)** |
|---|---|---|---|---|---|---|---|---|
| **Flash** | SFC350 | HiFMC | SFC350 | HiFMC | HiFMC | HiFMC | HiFMC | HiFMC |
| **Ethernet** | FEMAC | FEMAC | — | FEMAC | FEMAC | — | FEMAC | FEMAC |
| **SD/MMC** | himci×1 | himci×2 | himci×2 | himci×3 | himci×3 | himci×3 | SDHCI×2 | SDHCI×2 |
| **UARTs** | 3 | 3 | 4 | 3 | 3 | 5 | 3 | 3 |
| **SPI (PL022)** | — | 2 | 2 | 2 | 3 | 4 | 2 | 2 |
| **GPIO (PL061)** | 12 | 9 | 15 | 9 | 11 | 17 | 8-10 | 11 |
| **I2C** | — | 3 | 3 | 2 | 7 | 4 | 3 | 3 |
| **DMA (PL080)** | yes | yes | — | yes | — | — | — | — |
| **MIPI RX** | — | — | stub | — | yes | stub | yes | yes |
| **RTC** | — | — | yes | — | yes | yes | yes | yes |
| **VEDU/JPGE** | — | — | stub | — | yes | stub | yes | yes |
| **Watchdog** | yes | yes | yes | yes | yes | yes | yes | yes |
| **Sensor I2C** | — | — | — | — | — | — | yes | — |
| **IVE (18 ops)** | — | — | — | — | — | — | **yes** | — |
| **NPU** | — | — | — | — | — | — | — | stub |

Notes: `stub` = regbank stub only. V5 (×3) = CV608/CV610/CV613 (same die, different feature
tiers). AV100/3519V101 use GMAC (gigabit) — not emulated, boot without networking.
IVE = Intelligent Video Engine with 18 operations validated against real IVE silicon.

## Project Structure

```
qemu/
├── hw/arm/hisilicon.c           # Machine definitions (all 17 SoCs)
├── hw/misc/hisi-sysctl.c        # SysCtrl (SoC ID, reset, general registers)
├── hw/misc/hisi-crg.c           # CRG clock/reset stub
├── hw/misc/hisi-fmc.c           # HiFMC V100 flash controller (CV200+)
├── hw/misc/hisi-sfc350.c        # HISFC350 flash controller (CV100, AV100)
├── hw/misc/hisi-himci.c         # DW MMC (himciv200) SD/MMC controller
├── hw/misc/hisi-regbank.c       # Generic RAM-backed register bank
├── hw/misc/hisi-ive.c           # IVE: 18 ops (SAD, CCL, Sub, Thresh, Erode...)
├── hw/misc/hisi-vedu.c          # Video encoder stub (VEDU + JPGE)
├── hw/misc/hisi-mipi-rx.c       # MIPI RX controller stub
├── hw/misc/hisi-rtc.c           # SPI-bridge RTC device
├── hw/net/hisi-femac.c          # Fast Ethernet MAC + MDIO PHY stub
├── hw/i2c/hisi-i2c.c            # HiBVT I2C controller
├── hw/i2c/hisi-imx335.c         # IMX335 image sensor I2C device
├── include/hw/arm/hisilicon.h   # SoC config structure + constants
└── setup.sh                     # Clone QEMU v10.2.0, copy sources, build
qemu-boot/
├── run-cv100.sh                 # Run scripts for each boot-tested platform
├── run-cv200.sh
├── run-av100.sh
├── run-cv300.sh
├── run-cv500.sh
├── run-3519v101.sh
├── run-ev300.sh
├── run-cv610.sh
├── test-ive-init.c              # IVE basic test (hw_id, dma, sad, ccl)
├── test-ive-ops.c               # IVE operations test for QEMU (register-level)
├── test-ive-mpi.c               # IVE operations test for real board (MPI API)
├── test-ive-video-mpi.c         # IVE video processing for real board (MPI API, 352×288)
├── test-ive.c                   # IVE test (standalone)
├── test-ive-video.c             # IVE motion detection on video frames (QEMU)
└── test-ive-abandoned.c         # IVE abandoned object detection (QEMU)
demo/
├── generate_scene.py            # Synthetic CCTV scene generator
├── generate_abandoned.py        # Synthetic abandoned bag scene generator
├── ive_demo.py                  # Host reference SAD+CCL + visualization
├── abandoned_demo.py            # Host reference abandoned object + visualization
└── run_demo.sh                  # End-to-end demo orchestration
docs/
├── ive-registers.md             # IVE register map (from live EV300 capture)
└── nnie-vs-npu.md               # NNIE vs SVP_NPU architecture comparison
qemu-src/                        # (generated by setup.sh, not committed)
```

## Quick Start

```bash
# Build QEMU
bash qemu/setup.sh

# Download OpenIPC firmware (example: most boot-tested platforms)
cd qemu-boot
for soc in hi3516cv100 hi3516cv200 hi3516av100 hi3516cv300 hi3516cv500 hi3519v101 hi3516ev300; do
    curl -sL "https://github.com/OpenIPC/firmware/releases/download/latest/openipc.${soc}-nor-lite.tgz" \
        -o "openipc.${soc}-nor-lite.tgz"
    tar xzf "openipc.${soc}-nor-lite.tgz"
done
cd ..

# Boot any platform
bash qemu-boot/run-cv300.sh
bash qemu-boot/run-ev300.sh
bash qemu-boot/run-cv500.sh
# etc.
```

Default login: `root` / `12345`

### CV610 Boot (FIT image)

CV610 uses a FIT image (kernel + DTB in one file). Extract components first:

```bash
# Extract kernel, DTB, and rootfs from firmware
cd qemu-boot
dumpimage -T flat_dt -p 0 -o cv610.dtb firmware.bin.hi3516cv610
dumpimage -T flat_dt -p 1 -o kernel.hi3516cv610 firmware.bin.hi3516cv610
# rootfs is at offset 0x2A0000 in the firmware blob
dd if=firmware.bin.hi3516cv610 of=rootfs.squashfs.hi3516cv610 bs=1 skip=$((0x2A0000))

# Boot
bash run-cv610.sh
```

## Architecture

The emulator uses a table-driven design. Each SoC is a `HisiSoCConfig` struct
with addresses, IRQs, and peripheral counts. A single `hisilicon_common_init()`
function handles all platforms — adding a new SoC variant requires only a new
config struct and a one-line `DEFINE_HISI_MACHINE()` macro call.

### Memory Map Eras

| Era | SoCs | SysCtrl | UARTs | RAM | IRQ | GIC base |
|-----|------|---------|-------|-----|-----|----------|
| V1/V2 (0x20xx) | CV100, CV200 | 0x20050000 | 0x20080000 | 0x80M | VIC | — |
| V2A (0x20xx+GIC) | AV100 | 0x20050000 | 0x20080000 | 0x80M | GIC | 0x20301000 |
| V3 (0x12xx) | CV300 | 0x12020000 | 0x12100000 | 0x80M | VIC | — |
| V3.5 (unique) | CV500 | 0x12020000 | 0x120A0000 | 0x80M | GIC | 0x10301000 |
| V3A (0x12xx+GIC) | 3519V101 | 0x12020000 | 0x12100000 | 0x80M | GIC | 0x10301000 |
| V4 (0x12xx) | EV300, Goke | 0x12020000 | 0x12040000 | 0x40M | GIC | 0x10301000 |
| **V5 (0x11xx)** | **CV610** | **0x11020000** | **0x11040000** | **0x40M** | **GIC** | **0x12401000** |

### Appended DTB Auto-Patching

OpenIPC uImages for DT-based kernels have an appended device tree. The emulator
automatically patches it at boot:

- Adds `chosen/stdout-path` for console output
- Inserts padding for kernel's `atags_to_fdt()` initrd patching
- Unifies SP804 split clocks (e.g., AV100's 50 MHz/3 MHz → 3 MHz) to prevent timer storms
- Disables NAND controllers whose register-level polling hangs on stubs

This runs transparently — unmodified firmware artifacts work as-is.
CV610 uses a separate DTB file (`-dtb` flag) instead of appended DTB.

## SD Card Emulation

```bash
# Create a 64 MB image with FAT32
dd if=/dev/zero of=sdcard.img bs=1M count=64
echo -e "o\nn\np\n1\n\n\nt\nc\nw\n" | fdisk sdcard.img
LOOP=$(sudo losetup --find --show --partscan sdcard.img)
sudo mkfs.vfat -F 32 -n SDCARD "${LOOP}p1"
sudo losetup -d "$LOOP"

# Attach to any platform
bash qemu-boot/run-ev300.sh -drive file=sdcard.img,if=sd,format=raw
```

## Ethernet

The FEMAC device provides a single 8 KiB MMIO region with port, MDIO, and GLB
register blocks. An integrated PHY stub at MDIO address 1 reports 100 Mbps
full-duplex (PHY ID `0x00446161`). TX uses DMA read + `qemu_send_packet()`;
RX uses a 64-entry ring buffer with IRQ notification.

Run scripts pass `-nic user` for QEMU's SLIRP networking (NAT to host,
gateway `10.0.2.2`). PHY link-up takes a few seconds — DHCP may need a retry.

Note: AV100 and 3519V101 use GMAC (gigabit) which is not yet emulated;
those SoCs boot without networking.

## IVE (Intelligent Video Engine)

The IVE device (`hisi-ive`) provides hardware-accelerated image processing
for V4 SoCs. Register map reverse-engineered from live EV300 hardware capture
(554 register changes, see `docs/ive-registers.md`). 18 operations implemented,
validated against real IVE silicon on EV300 board:

| Category | Operations | Status |
|----------|-----------|--------|
| Memory | DMA (copy with stride) | ✓ HW verified |
| Motion detection | SAD (4×4 block diff + threshold), CCL (connected components) | ✓ HW verified |
| Pixel-wise | Sub, Add (weighted blend), And, Or, Xor | ✓ HW verified |
| Thresholding | Thresh (binary), Hist (256-bin histogram) | ✓ HW verified |
| Convolution | Filter (5×5 kernel), Sobel (3×3 edge detection) | ✓ HW verified |
| Morphology | Dilate (5×5 max), Erode (5×5 min) | ✓ HW verified |
| Analysis | Integ (integral image), Map (LUT), NCC (cross-correlation) | ✓ verified |
| Background | GMM2 (Gaussian Mixture Model, stateful per-pixel) | ✓ verified |

### Validation against real hardware

Two test binaries validate the same algorithms through different paths:

| Test | Platform | Path | Image size |
|------|----------|------|-----------|
| `test-ive-mpi` | Real EV300 board | MPI API → libmpi.so → kernel module → IVE silicon | 64×64 |
| `test-ive-ops` | QEMU | Register writes → hisi-ive.c device | 64×64 |

Per-pixel output values match between real IVE hardware and QEMU emulation:
```
            Real HW                    QEMU
sub:        [18,22,26,30]              [18,22,26,30]       ✓
and:        [13,0,17,0]                [13,0,17,0]         ✓
or:         [31,62,63,98]              [31,62,63,98]       ✓
xor:        [18,62,46,98]              [18,62,46,98]       ✓
thresh:     [0,0,0,0]                  [0,0,0,0]           ✓
sobel:      [28,56,56,56]              [28,56,56,56]       ✓
dilate:     [223,255,255,255]          [223,255,255,255]    ✓
erode:      [0,0,0,0]                  [0,0,0,0]           ✓
```

```bash
# QEMU test (register-level, runs as init in initramfs)
CC=path/to/arm-openipc-linux-musleabi-gcc
$CC -static -O2 -o /tmp/init qemu-boot/test-ive-ops.c
mkdir -p /tmp/ive && cp /tmp/init /tmp/ive/init
cd /tmp/ive && find . | cpio -oH newc | gzip > /tmp/ive.gz
qemu-system-arm -M hi3516ev300 -m 128M \
    -kernel qemu-boot/uImage.hi3516ev300 -initrd /tmp/ive.gz \
    -nographic -serial mon:stdio \
    -append "console=ttyAMA0,115200 mem=128M root=/dev/ram0 rdinit=/init"

# Real board test (MPI API, needs SDK libs)
$CC -o test-ive-mpi qemu-boot/test-ive-mpi.c -I$SDK/include -L$LIBS -lmpi -live ...
# Upload and run: load_hisilicon -i; killall majestic; ./test-ive-mpi
```

### Full Hardware Pipeline

The real board test (`test-ive-video-mpi`) uses a **full hardware pipeline** with
zero CPU pixel loops. Reads Y4M (YUV4MPEG2 Cmono) or legacy raw .bin files.
Resolution is parsed from the Y4M header — follows Majestic's approach:
`MD_SIZE(x) = (x/2) & ~0xF` (half sensor resolution, 16-byte aligned).

| Platform | Binary | IVE path | Notes |
|----------|--------|----------|-------|
| Real EV300 board | `test-ive-video-mpi` | MPI API → libmpi.so → kernel module → IVE silicon | Ground truth |
| QEMU | `test-ive-video` | Register writes → hisi-ive.c device | Emulation |
| Host | `ive_demo.py` / `abandoned_demo.py` | Python reference + visualization | Demo video |

#### Motion Detection: SAD → CCL (2 IVE ops)

```
Frame (960×528) → SAD 4×4 blocks → threshold output (240×132) → CCL → blob regions
```

```bash
# Convert 1080p source to Majestic-equivalent resolution
ffmpeg -y -i source.mp4 -vf "scale=960:528,format=gray" -r 10 \
    -pix_fmt gray -f yuv4mpegpipe output.y4m

# Run on real IVE hardware
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/meva-school-vehicles.y4m md'
# → FRAME 210: (840,316)-(848,328) area=5  ... vehicle motion detected
```

#### Abandoned Object Detection: 9-step IVE pipeline

```
Frame → Sub(cur,ref) → Thresh → Erode → Dilate → SAD(mask,zero) → blockify
     → SAD(cur,prev) → Thresh(invert) → AND(fg,stationary) → CCL → blob regions
```

CPU only reads the CCL blob struct (~3 KB) per frame — no pixel scanning.
The `SAD(mask, zero_image)` trick reduces the full-frame binary mask (960×528)
to block-level (240×132), staying within CCL's 64-720 pixel size constraint.

```bash
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/meva-abandon-package.y4m abandoned'
# → FRAME 105: ABANDONED (804,212)-(848,256) area=82 dur=30
# → ... sustained 70+ frames (package left at bus stop)
```

#### License Plate Region Detection: 6-step IVE pipeline

```
Frame → Sobel(vertical) → 16BitTo8Bit(abs) → Thresh → Dilate → Erode → CCL
                                                                         ↓
                                               CPU: filter by aspect ratio 2.0-6.0
```

Detects plate-shaped rectangular regions with dense edges. Not full OCR — finds
WHERE the plate is as a pre-filter for cloud-based recognition.

```bash
ssh root@ev300-board 'killall majestic; /utils/ive-test/test-ive-video-mpi \
    /utils/ive-test/parking.y4m lpr'
# → FRAME 1: PLATE (452,500)-(516,520) area=32 ratio=3.2
```

#### Benchmark: 960×528 on real EV300 IVE silicon

| Component | Motion detection | Abandoned detection | Plate region |
|-----------|-----------------|-------------------|--------------|
| **IVE hardware** | **2.4 ms/frame** | **6.7 ms/frame** | **9.1 ms/frame** |
| CPU (blob parse) | 0.0 ms/frame | 0.1 ms/frame | 0.2 ms/frame |
| I/O (NFS read) | 40.8 ms/frame | 9.9 ms/frame | 33.3 ms/frame |
| **IVE capacity** | **416 fps** | **149 fps** | **110 fps** |
| Total (with NFS) | 23 fps | 60 fps | 24 fps |

In a real camera, VPSS feeds frames via DMA (zero I/O overhead), so the IVE
pipeline easily exceeds the 10 fps target. NFS is the bottleneck in the test setup.

### Quantitative Evaluation (MEVA + VIRAT Ground Truth)

Evaluated on **3132 motion events** and **131 abandoned-object events** from the
[MEVA](https://mevadata.org/) (CC BY 4.0) and [VIRAT Ground 2.0](https://viratdata.org/)
datasets. All processing on **real EV300 IVE silicon** at Majestic-equivalent resolution
(960×528 for 1080p, 640×352 for 720p sources).

#### Event-level metrics

| Algorithm | Precision | Recall | F1 | TP | FP | FN | Events |
|-----------|-----------|--------|------|-----|-----|------|--------|
| **Motion detection** | **0.998** | **1.000** | **0.999** | 3126 | 6 | 0 | 3132 |
| **Abandoned object** | **0.361** | **0.673** | **0.470** | 35 | 62 | 17 | 131 |

**Motion detection** is near-perfect: all motion events detected (vehicles, people
walking, carrying objects) with only 6 false alarms across 3132 test events.

**Abandoned object detection** catches 67% of events (bags, packages, unloaded objects)
but has false positives from stationary scene elements (parked cars, furniture) that
differ from the learned reference background. Threshold sweep (diff_thr 30-60,
sad_thr 100-300, area_thr 4-16) confirmed this is an inherent limitation of
background subtraction — the algorithm cannot distinguish "newly abandoned bag" from
"car that was parked during reference learning." Production-grade precision requires
owner-object tracking or semantic classification (see `docs/ive-applications.md`).

#### Dataset composition

Sources: MEVA examples (121 pre-trimmed 1080p clips) + VIRAT Ground 2.0 (83 HD clips).
Activities used as ground truth:

| Eval task | Positives | Negatives | Activity types |
|-----------|-----------|-----------|---------------|
| Motion | 3126 events | 6 static clips | walking, vehicle_moving, carrying, riding, ... |
| Abandoned | 52 events | 79 clips | Abandon_Package, SetDown, Unloading, Drop |

```bash
# Reproduce evaluation
python3 scripts/eval_download_virat.py       # download VIRAT videos (~18 GB)
python3 scripts/eval_build_dataset.py        # convert to Y4M
bash scripts/eval_run_ive.sh both            # run on real EV300 board
python3 scripts/eval_metrics.py              # compute P/R/F1
```

See `docs/meva-dataset-spec.md` for dataset structure, annotation format, and
evaluation methodology.

See `docs/ive-applications.md` for a roadmap of 9 CV applications
(tamper detection, line crossing, zone intrusion, loitering, etc.)
that can be built with the existing IVE operations.

## References

- OpenIPC firmware: https://github.com/openipc/firmware
- ipctool: https://github.com/openipc/ipctool
- SDK device trees:
  - CV100: `Hi3518_SDK_V1.0.B.0/.../mach-hi3518/`
  - CV200: `Hi3518E_SDK_V1.0.4.0/.../hi3518ev20x.dtsi`
  - AV100: `Hi3516A_SDK_V1.0.8.0/.../hi3516a.dtsi`
  - CV300: `Hi3516CV300_SDK_V1.0.4.0/.../hi3516cv300.dtsi`
  - CV500: `Hi3516CV500_SDK_V2.0.2.1/.../hi3516cv500.dtsi`
  - 3519V101: `Hi3519V101_SDK_V1.0.5.0/...patch (hi3519v101.dtsi)`
  - EV300: `Hi3516EV200_SDK_V1.0.1.2/.../hi3516ev300.dtsi`
  - CV610: `Hi3516CV610R001C01SPC020/...tgz (hi3516cv610.dtsi)`
