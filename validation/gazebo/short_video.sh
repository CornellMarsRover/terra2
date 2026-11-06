#!/usr/bin/env bash
set -euo pipefail
session="$1"
input="$session/demo.mp4"
output="$session/demo_short.mp4"
command -v ffmpeg >/dev/null || { echo 'ffmpeg unavailable; full video retained.' >&2; exit 0; }
command -v ffprobe >/dev/null || { echo 'ffprobe unavailable; full video retained.' >&2; exit 0; }
[[ -s "$input" ]] || exit 0
duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$input")"
factor="$(awk -v duration="$duration" 'BEGIN {factor=duration/60; if (factor<1) factor=1; printf "%.3f", factor}')"
ffmpeg -v error -nostdin -y -i "$input" -vf "setpts=PTS/$factor,fps=24" \
  -an -c:v libx264 -preset veryfast -crf 24 -pix_fmt yuv420p \
  -movflags +faststart "$output"
short="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$output")"
printf "Fast-forward playback: %sx; raw recording: %ss; short clip: %ss\n" "$factor" "$duration" "$short" > "$session/playback.txt"
printf 'Playback video: %s (%.1fs at %sx)\n' "$output" "$short" "$factor"
