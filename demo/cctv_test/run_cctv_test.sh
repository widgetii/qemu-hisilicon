#!/bin/bash
#
# Run motion detection on real CCTV footage from C-MOR and CAVIAR datasets.
# Prerequisites: videos in /mnt/data/video-sources/cctv-test/
#
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

VIDS="/mnt/data/video-sources/cctv-test"

echo "=== C-MOR Motion Detection (1080p/720p surveillance cameras) ==="
for src in cmor-computer-room-1080p cmor-warehouse-1080p cmor-entrance-720p cmor-outside-720p; do
    [ -f "$VIDS/${src}.mp4" ] || { echo "  SKIP $src (not found)"; continue; }
    dir="$SCRIPT_DIR/${src}"
    mkdir -p "$dir/raw"
    ffmpeg -y -i "$VIDS/${src}.mp4" -vf "scale=320:240,format=gray" -r 10 -t 15 "$dir/raw/frame_%04d.raw" 2>/dev/null
    ffmpeg -y -i "$VIDS/${src}.mp4" -vf "scale=320:240" -r 10 -t 15 "$dir/frame_%04d.png" 2>/dev/null
    # Fix 1-based → 0-based
    cd "$dir"; for f in frame_*.png; do [ -f "$f" ] || continue; n=$(echo "$f"|sed 's/frame_0*\([0-9]*\).png/\1/'); mv "$f" "$(printf 'frame_%04d.png' $((n-1)))"; done
    cd raw; for f in frame_*.raw; do [ -f "$f" ] || continue; n=$(echo "$f"|sed 's/frame_0*\([0-9]*\).raw/\1/'); mv "$f" "$(printf 'frame_%04d.raw' $((n-1)))"; done
    cd "$REPO_ROOT"
    nf=$(ls "$dir/raw/"*.raw|wc -l)
    nm=$(python3 demo/ive_demo.py --frames-dir "$dir" 2>/dev/null|wc -l)
    echo "  $src: $nf frames, $nm with motion"
    python3 demo/ive_demo.py --frames-dir "$dir" --output-dir "$SCRIPT_DIR/output_${src}" --visualize 2>/dev/null
done

echo ""
echo "=== CAVIAR Abandoned Bag Scenarios (384×288 research dataset) ==="
for src in LeftBag LeftBag_AtChair LeftBox Walk1; do
    [ -f "$VIDS/${src}.mpg" ] || { echo "  SKIP $src (not found)"; continue; }
    dir="$SCRIPT_DIR/caviar-${src}"
    mkdir -p "$dir/raw"
    ffmpeg -y -i "$VIDS/${src}.mpg" -vf "scale=320:240,format=gray" -r 10 "$dir/raw/frame_%04d.raw" 2>/dev/null
    ffmpeg -y -i "$VIDS/${src}.mpg" -vf "scale=320:240" -r 10 "$dir/frame_%04d.png" 2>/dev/null
    cd "$dir"; for f in frame_*.png; do [ -f "$f" ] || continue; n=$(echo "$f"|sed 's/frame_0*\([0-9]*\).png/\1/'); mv "$f" "$(printf 'frame_%04d.png' $((n-1)))"; done
    cd raw; for f in frame_*.raw; do [ -f "$f" ] || continue; n=$(echo "$f"|sed 's/frame_0*\([0-9]*\).raw/\1/'); mv "$f" "$(printf 'frame_%04d.raw' $((n-1)))"; done
    cd "$REPO_ROOT"
    nf=$(ls "$dir/raw/"*.raw|wc -l)
    nm=$(python3 demo/ive_demo.py --frames-dir "$dir" 2>/dev/null|wc -l)
    echo "  CAVIAR $src: $nf frames, $nm with motion"
    python3 demo/ive_demo.py --frames-dir "$dir" --output-dir "$SCRIPT_DIR/output_caviar-${src}" --visualize 2>/dev/null
done

echo ""
echo "=== Demo videos ==="
ls -lh "$SCRIPT_DIR"/output_*/demo_output.mp4 2>/dev/null
