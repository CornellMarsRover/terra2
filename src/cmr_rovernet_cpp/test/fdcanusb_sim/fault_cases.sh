#!/bin/bash
# Fault-injection checks for usama_control_testing_node (C++) on fake adapters.
# Run inside the ROS docker image with the workspace sourced:  fault_cases.sh <out dir>
#  A  configured adapter missing, only the *arm* adapter (IDs 9-14) present
#       -> must not open it: only read-only probe queries, no stop/command frames
#  B  configured adapter missing, an adapter with the drive IDs present -> auto-selected
#  C  adapter never answers (mute) -> errors every cycle, never "stop sent"/healthy
#  D  tty write blocks forever (ptyshim) -> SIGINT exits within ~2 s (rc 1), no hang
OUT=${1:?out dir}; HERE=$(cd "$(dirname "$0")" && pwd)
BIN=$(ros2 pkg prefix cmr_rovernet_cpp)/lib/cmr_rovernet_cpp/usama_control_testing_node
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=43
gcc -shared -fPIC -o /tmp/ptyshim.so "$HERE/ptyshim.c" -ldl || exit 1
mkdir -p $OUT /dev/serial/by-id
fail=0
check() { if eval "$2"; then echo "  PASS $1"; else echo "  FAIL $1"; fail=1; fi; }
run_case() {  # name, link, fake mode, fake ids, can_port, run seconds
  name=$1; link=$2; D=$OUT/$name; rm -rf $D; mkdir -p $D; rm -f /dev/serial/by-id/*
  python3 "$HERE/fake_fdcanusb.py" $link $D/frames.log $3 $4 & FAKE=$!
  sleep 0.5
  LD_PRELOAD=/tmp/ptyshim.so $BIN --ros-args -p can_port:=$5 > $D/node.log 2>&1 & NODE=$!
  sleep $6
  [ $name = D ] && { touch /tmp/ptyshim_stall; sleep 1; }
  t0=$(date +%s.%N); kill -INT $NODE; ( sleep 10; kill -9 $NODE 2>/dev/null ) & wait $NODE; RC=$?
  EXIT_S=$(python3 -c "import time; print('%.1f' % (time.time() - $t0))")
  kill $FAKE 2>/dev/null; touch $D/frames.log; rm -f /tmp/ptyshim_stall
}
nonquery() { grep -c "can send 00" $1; }   # frames without the reply-request bit: stop / commands

echo "A: only the arm adapter present"
run_case A /dev/serial/by-id/usb-mjbots_fdcanusb_FAKEARM-if00 ok 9,10,11,12,13,14 /dev/serial/by-id/usb-mjbots_fdcanusb_MISSING-if00 6
check "no stop/command frame reached the arm bus" "[ $(nonquery $OUT/A/frames.log) -eq 0 ]"
check "probe queries were sent" "[ $(grep -c 'can send 80' $OUT/A/frames.log) -gt 0 ]"
check "never opened a transport" "! grep -q 'Opened moteus' $OUT/A/node.log"
check "fallback logged via ROS logger" "grep -q 'WARN.*not found; falling back to probing autodetect' $OUT/A/node.log"
check "no-responder error logged" "grep -q 'No fdcanusb adapter answered' $OUT/A/node.log"
check "clean exit (rc=$RC)" "[ $RC -eq 0 ]"

echo "B: adapter with the drive IDs present under another name"
run_case B /dev/serial/by-id/usb-mjbots_fdcanusb_FAKEDRV-if00 ok 1,2,3,4,5,6,7,8 /dev/serial/by-id/usb-mjbots_fdcanusb_MISSING-if00 6
check "auto-selected after probing" "grep -q 'auto-selected CAN adapter /dev/serial/by-id/usb-mjbots_fdcanusb_FAKEDRV-if00' $OUT/B/node.log"
check "stops were sent" "[ $(nonquery $OUT/B/frames.log) -gt 0 ]"
check "clean exit (rc=$RC)" "[ $RC -eq 0 ]"

echo "C: mute adapter (explicit path)"
run_case C /tmp/fakecan_mute mute "" /tmp/fakecan_mute 5
check "errors logged" "grep -q 'ERROR.*did not acknowledge' $OUT/C/node.log"
check "final stop reported as failed" "grep -q 'Drive shutdown stop failed' $OUT/C/node.log"
check "clean exit (rc=$RC)" "[ $RC -eq 0 ]"

echo "D: stalled tty writes"
run_case D /tmp/fakecan_stall ok "" /tmp/fakecan_stall 4
check "worker timeout logged" "grep -q 'Drive worker did not stop within 2.0 s' $OUT/D/node.log"
check "exited without hanging (${EXIT_S}s, rc=$RC)" "[ $RC -eq 1 ] && python3 -c 'import sys; sys.exit(0 if $EXIT_S < 4.0 else 1)'"
[ $fail -eq 0 ] && echo "RESULT: ALL FAULT CASES PASS" || echo "RESULT: FAULT CASE FAILURES"
exit $fail
