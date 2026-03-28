#!/bin/bash
set -euo pipefail

# Bypass proxy for direct S3 access (faster downloads)
unset HTTP_PROXY HTTPS_PROXY http_proxy https_proxy NO_PROXY no_proxy 2>/dev/null || true

DEST=/mnt/data/datasets/meva
BUCKET=s3://mevadata-public-01
S3="aws s3 --no-sign-request"

mkdir -p "$DEST"

echo "=== Phase 1: Downloading annotated examples (~900 MB) ==="
$S3 sync "$BUCKET/examples/" "$DEST/examples/"

echo ""
echo "=== Phase 2: Downloading full annotation archive (~175 MB) ==="
ZIP="$DEST/viratannotations.snapshot.23jul2025.zip"
if [ ! -f "$ZIP" ]; then
    $S3 cp "$BUCKET/viratannotations.snapshot.23jul2025.zip" "$ZIP"
else
    echo "  Already downloaded: $ZIP"
fi

echo "  Extracting annotations..."
unzip -qo "$ZIP" -d "$DEST/virat-annotations/"

echo ""
echo "=== Download complete ==="
echo "Examples videos: $(find "$DEST/examples/videos" -type f 2>/dev/null | wc -l) files"
echo "Annotation dirs: $(find "$DEST/examples/annotations" -type d 2>/dev/null | wc -l) dirs"
echo "VIRAT annotations: $(find "$DEST/virat-annotations" -type f 2>/dev/null | wc -l) files"
echo "Total size: $(du -sh "$DEST" | cut -f1)"
