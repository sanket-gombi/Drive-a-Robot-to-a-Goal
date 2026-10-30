#!/usr/bin/env bash
# Helper script to copy the latest screenshot and screencast into docs/media/
set -e

MEDIA_DIR="/home/sanket/orca_robotics_task/docs/media"
mkdir -p "$MEDIA_DIR"

LATEST_SCREENSHOT=$(ls -t ~/Pictures/Screenshots/*.png 2>/dev/null | head -n 1 || true)
if [ -n "$LATEST_SCREENSHOT" ]; then
    cp "$LATEST_SCREENSHOT" "$MEDIA_DIR/latest_screenshot.png"
    echo "✅ Copied latest screenshot: $(basename "$LATEST_SCREENSHOT") -> docs/media/latest_screenshot.png"
else
    echo "⚠️ No screenshots found in ~/Pictures/Screenshots/"
fi

LATEST_VIDEO=$(ls -t ~/Videos/Screencasts/*.webm 2>/dev/null | head -n 1 || true)
if [ -n "$LATEST_VIDEO" ]; then
    cp "$LATEST_VIDEO" "$MEDIA_DIR/simulation_demo.webm"
    echo "✅ Copied latest video: $(basename "$LATEST_VIDEO") -> docs/media/simulation_demo.webm"
else
    echo "⚠️ No videos found in ~/Videos/Screencasts/"
fi

ls -lh "$MEDIA_DIR"
