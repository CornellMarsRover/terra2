#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="terra2-astronaut-humble:local"
MODE="${1:-help}"

if ! command -v docker >/dev/null 2>&1; then
  echo "Docker is not installed or is not on PATH." >&2
  echo "Install and start Docker Desktop, then rerun this command." >&2
  exit 1
fi

case "$MODE" in
  build)
    exec docker build -t "$IMAGE" \
      -f "$ROOT/docker/astronaut-humble/Dockerfile" "$ROOT"
    ;;
  test|sim)
    mkdir -p "$ROOT/artifacts/astronaut_sim"
    extra=()
    if [[ "$MODE" == sim ]]; then
      extra=(--pipeline-only)
    fi
    exec docker run --rm \
      -v "$ROOT/artifacts/astronaut_sim:/tmp/astronaut_sim" \
      "$IMAGE" bash scripts/test_astronaut_ros_pipeline.sh "${extra[@]}"
    ;;
  shell)
    exec docker run --rm -it \
      -v "$ROOT/artifacts/astronaut_sim:/tmp/astronaut_sim" \
      "$IMAGE" bash
    ;;
  *)
    echo "Usage: $0 {build|test|sim|shell}"
    exit 2
    ;;
esac
