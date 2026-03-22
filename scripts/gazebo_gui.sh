#!/bin/bash

set -euo pipefail

mode="${TERRA_GUI_MODE:-xpra}"
display=""
command=()

detect_xpra_display() {
  local detected
  detected="$(xpra list 2>/dev/null | sed -n 's/.*LIVE session at \(:[0-9][0-9]*\).*/\1/p' | head -n1 || true)"
  if [[ -n "$detected" ]]; then
    printf '%s\n' "$detected"
  fi
}

usage() {
  cat <<'EOF'
Usage:
  scripts/gazebo_gui.sh [--mode xpra|xquartz|custom] [--display DISPLAY] [-- command...]

Defaults:
  --mode xpra
  command: gzclient --verbose

Examples:
  scripts/gazebo_gui.sh
  scripts/gazebo_gui.sh --mode xquartz
  scripts/gazebo_gui.sh --mode xquartz --display host.docker.internal:0
  scripts/gazebo_gui.sh -- xclock
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mode)
      mode="${2:-}"
      shift 2
      ;;
    --display)
      display="${2:-}"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    --)
      shift
      command=("$@")
      break
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ ${#command[@]} -eq 0 ]]; then
  command=(gzclient --verbose)
fi

case "$mode" in
  xpra)
    if [[ -n "$display" ]]; then
      export DISPLAY="$display"
    else
      detected_xpra_display="$(detect_xpra_display)"
      export DISPLAY="${detected_xpra_display:-${TERRA_XPRA_DISPLAY:-:0}}"
    fi
    ;;
  xquartz)
    export DISPLAY="${display:-${TERRA_XQUARTZ_DISPLAY:-host.docker.internal:0}}"
    ;;
  custom)
    if [[ -n "$display" ]]; then
      export DISPLAY="$display"
    elif [[ -z "${DISPLAY:-}" ]]; then
      echo "custom mode requires DISPLAY to already be set or passed with --display" >&2
      exit 1
    fi
    ;;
  *)
    echo "Unsupported mode: $mode" >&2
    usage >&2
    exit 1
    ;;
esac

export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-cmr}"

mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

echo "Launching GUI command using mode=$mode display=$DISPLAY"
exec "${command[@]}"
