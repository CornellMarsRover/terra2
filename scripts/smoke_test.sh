#!/usr/bin/env bash
# Smoke test for the URC Astrotech mock rover.
#
# Builds cmr_msgs + urc_mock_rover, launches the mock, verifies every
# expected topic appears, checks two rates, and calls one service.
#
# Requires: ROS 2 Humble, colcon, ffmpeg (for the sample video), foxglove
# bridge installed (``ros-humble-foxglove-bridge``). No GUI; exits non-zero
# on first failure. Not CI-wired yet — invoke manually from the repo root:
#
#     bash scripts/smoke_test.sh
#
# Each check is numbered and prefixed with its step from the Phase 2a brief.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "${REPO_ROOT}"

log() { printf '\033[1;34m[smoke]\033[0m %s\n' "$*"; }
fail() { printf '\033[1;31m[smoke FAIL]\033[0m %s\n' "$*" >&2; exit 1; }

# --- Step 1: source ROS and build our packages ---
log "sourcing /opt/ros/humble/setup.bash"
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

log "colcon build --packages-select cmr_msgs urc_mock_rover"
colcon build \
    --packages-select cmr_msgs urc_mock_rover \
    --symlink-install \
    --event-handlers console_cohesion+

log "sourcing install/setup.bash"
# shellcheck source=/dev/null
source install/setup.bash

# --- Step 2: sample video must exist (camera replayer loads it from disk) ---
ASSET="src/urc_mock_rover/urc_mock_rover/assets/sample_video.h264"
if [[ ! -s "${ASSET}" ]]; then
    log "sample video missing; running scripts/fetch_sample_video.sh"
    bash scripts/fetch_sample_video.sh
fi

# --- Step 3: launch the mock rover in the background ---
LOG_DIR="$(mktemp -d)"
LAUNCH_LOG="${LOG_DIR}/mock_launch.log"
log "launching mock.launch.py (logs in ${LAUNCH_LOG})"
ros2 launch urc_mock_rover mock.launch.py >"${LAUNCH_LOG}" 2>&1 &
MOCK_PID=$!
cleanup() {
    if kill -0 "${MOCK_PID}" 2>/dev/null; then
        log "terminating mock rover (pid ${MOCK_PID})"
        kill "${MOCK_PID}" 2>/dev/null || true
        wait "${MOCK_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

log "waiting 5 s for nodes to come up"
sleep 5

# --- Step 4: every expected topic must be advertised ---
EXPECTED_TOPICS=(
    "/astrotech/auger/state"
    "/astrotech/mixing_servo/state"
    "/astrotech/raman/spectrum"
    "/astrotech/env/sample"
    "/camera_0/h264"
    "/camera_2/h264"
    "/camera_4/h264"
)
log "listing topics..."
ALL_TOPICS="$(ros2 topic list)"
for t in "${EXPECTED_TOPICS[@]}"; do
    if ! grep -qxF "${t}" <<<"${ALL_TOPICS}"; then
        echo "${ALL_TOPICS}"
        fail "expected topic ${t} not found"
    fi
    log "  ok: ${t}"
done

# --- Step 5: rate check — camera must be 10 < Hz < 30 ---
log "measuring /camera_0/h264 rate for 3 s"
CAM_HZ_OUT="$(timeout 4 ros2 topic hz /camera_0/h264 2>&1 | head -n 20 || true)"
echo "${CAM_HZ_OUT}"
CAM_RATE="$(grep -Eo 'average rate: [0-9.]+' <<<"${CAM_HZ_OUT}" | tail -n 1 | awk '{print $3}')"
if [[ -z "${CAM_RATE}" ]]; then
    fail "could not parse rate from ros2 topic hz output for /camera_0/h264"
fi
# Integer comparison via awk to avoid floating-point shell headaches.
if awk "BEGIN{exit !(${CAM_RATE} > 10 && ${CAM_RATE} < 30)}"; then
    log "  ok: /camera_0/h264 = ${CAM_RATE} Hz (10 < rate < 30)"
else
    fail "/camera_0/h264 rate ${CAM_RATE} Hz out of range (10, 30)"
fi

# --- Step 6: rate check — Raman must be > 0.5 Hz ---
log "measuring /astrotech/raman/spectrum rate for 3 s"
RAMAN_HZ_OUT="$(timeout 4 ros2 topic hz /astrotech/raman/spectrum 2>&1 | head -n 20 || true)"
echo "${RAMAN_HZ_OUT}"
RAMAN_RATE="$(grep -Eo 'average rate: [0-9.]+' <<<"${RAMAN_HZ_OUT}" | tail -n 1 | awk '{print $3}')"
if [[ -z "${RAMAN_RATE}" ]]; then
    fail "could not parse rate for /astrotech/raman/spectrum"
fi
if awk "BEGIN{exit !(${RAMAN_RATE} > 0.5)}"; then
    log "  ok: /astrotech/raman/spectrum = ${RAMAN_RATE} Hz (> 0.5)"
else
    fail "/astrotech/raman/spectrum rate ${RAMAN_RATE} Hz <= 0.5"
fi

# --- Step 7: service call returns success on a valid preset ---
log "calling /astrotech/mixing_servo/set_preset with preset_name=S1"
SVC_OUT="$(ros2 service call /astrotech/mixing_servo/set_preset \
    cmr_msgs/srv/SetMixingServoPreset \
    '{preset_name: S1}' 2>&1)"
echo "${SVC_OUT}"
if ! grep -q 'success=True' <<<"${SVC_OUT}"; then
    fail "service did not return success=True"
fi
log "  ok: service returned success"

# --- All checks passed ---
log "all checks passed; exiting 0"
exit 0
