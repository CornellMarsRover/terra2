#!/usr/bin/env bash
# Generate sample_video.h264 for the mock camera replayer.
#
# We use ffmpeg's built-in ``testsrc`` source to produce a self-contained
# H.264 Annex-B bytestream (no container, just NAL units), which is exactly
# what MockCameraReplayer expects. Running this once per dev machine is the
# deal — the resulting file is gitignored.
#
# Requires ffmpeg on PATH (apt: ``ffmpeg``, brew: ``brew install ffmpeg``).

set -euo pipefail

ASSETS_DIR="$(cd "$(dirname "$0")/.." && pwd)/src/urc_mock_rover/urc_mock_rover/assets"
OUT="${ASSETS_DIR}/sample_video.h264"

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "error: ffmpeg not found on PATH." >&2
    echo "  apt: sudo apt install ffmpeg" >&2
    echo "  brew: brew install ffmpeg" >&2
    exit 1
fi

mkdir -p "${ASSETS_DIR}"

echo "generating ${OUT} (10 s test pattern, ~30 fps, H.264 Annex-B)..."
ffmpeg \
    -y \
    -hide_banner \
    -loglevel warning \
    -f lavfi \
    -i "testsrc=duration=10:size=640x480:rate=30" \
    -c:v libx264 \
    -preset ultrafast \
    -tune zerolatency \
    -pix_fmt yuv420p \
    -g 30 \
    -bsf:v h264_mp4toannexb \
    -f h264 \
    "${OUT}"

size_bytes="$(wc -c < "${OUT}")"
echo "wrote ${OUT} (${size_bytes} bytes)"
echo "done."
