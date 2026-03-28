#!/bin/bash
# Run IVE test on all Y4M evaluation clips on real EV300 board.
#
# Usage: bash scripts/eval_run_ive.sh [abandoned|md|both] [--diff-thr=N] [--sad-thr=N]

set -e

Y4M_DIR="/mnt/data/datasets/meva/y4m"
NFS_DIR="/mnt/noc/ive-test"
RESULTS_DIR="/mnt/data/datasets/meva/results"
BOARD="root@IVG85HG50PYA-S.dlab.doty.ru"
BOARD_DIR="/utils/ive-test"

MODE="${1:-both}"
shift || true
EXTRA_ARGS="$@"

mkdir -p "$RESULTS_DIR"

# Stop majestic once
echo "Stopping majestic..."
ssh "$BOARD" 'killall majestic 2>/dev/null; sleep 1' 2>/dev/null || true

run_clip() {
    local y4m="$1"
    local mode="$2"
    local base=$(basename "$y4m" .y4m)
    local clip_name="${base%.*}"  # Remove .abandoned or .md suffix
    local result_file="$RESULTS_DIR/${clip_name}.${mode}.txt"
    local bench_file="$RESULTS_DIR/${clip_name}.${mode}.bench"

    if [ -f "$result_file" ] && [ -s "$result_file" ]; then
        return 0  # Already processed
    fi

    # Copy Y4M to NFS
    cp "$y4m" "$NFS_DIR/eval_clip.y4m"

    # Run on board
    ssh "$BOARD" "timeout 300 $BOARD_DIR/test-ive-video-mpi $BOARD_DIR/eval_clip.y4m $mode $EXTRA_ARGS" \
        > "$result_file" 2> "$bench_file" || true

    # Count detections
    local n_det=$(grep -c "^FRAME" "$result_file" 2>/dev/null || echo 0)
    echo "  $clip_name ($mode): $n_det detections"
}

# Collect Y4M files
count=0
total=$(ls "$Y4M_DIR"/*.y4m 2>/dev/null | wc -l)

for y4m in "$Y4M_DIR"/*.y4m; do
    [ -f "$y4m" ] || continue
    base=$(basename "$y4m" .y4m)
    count=$((count + 1))

    # Determine which mode to run based on filename suffix
    if echo "$base" | grep -q '\.abandoned$'; then
        if [ "$MODE" = "both" ] || [ "$MODE" = "abandoned" ]; then
            echo "[$count/$total] $base"
            run_clip "$y4m" "abandoned"
        fi
    elif echo "$base" | grep -q '\.md$'; then
        if [ "$MODE" = "both" ] || [ "$MODE" = "md" ]; then
            echo "[$count/$total] $base"
            run_clip "$y4m" "md"
        fi
    else
        # No mode suffix — run requested mode
        if [ "$MODE" = "both" ]; then
            echo "[$count/$total] $base (both modes)"
            run_clip "$y4m" "md"
            run_clip "$y4m" "abandoned"
        else
            echo "[$count/$total] $base ($MODE)"
            run_clip "$y4m" "$MODE"
        fi
    fi
done

# Restart majestic
echo "Restarting majestic..."
ssh "$BOARD" 'majestic &' 2>/dev/null || true

# Summary
n_results=$(ls "$RESULTS_DIR"/*.txt 2>/dev/null | wc -l)
echo ""
echo "Done: $n_results result files in $RESULTS_DIR"
