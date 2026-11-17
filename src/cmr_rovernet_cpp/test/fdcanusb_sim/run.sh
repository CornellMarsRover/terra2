#!/bin/bash
# Hardware-free Python-vs-C++ check of the drive stack on a simulated fdcanusb.
#
# For each implementation (py = cmr_rovernet, cpp = cmr_rovernet_cpp) this
# starts a fake fdcanusb (pty that ACKs and logs every CAN frame), the
# drive_command_mux and one drive node, replays the scripted operator session
# in scenario.py and SIGINTs the node.  analyze.py then compares, step by
# step, the CAN frames both stacks put on the bus (byte for byte).
#
# Run inside the ROS docker image with the workspace built and sourced and
# cmr_rovernet importable (PYTHONPATH=<repo>/src/cmr_rovernet):
#   run.sh <out dir> [usama_control_testing|steer_only_control|drive_only_control|single_drive_wheel_control]
#   python3 analyze.py <out dir>
OUT=${1:?out dir}; NODE_MOD=${2:-usama_control_testing}
HERE=$(cd "$(dirname "$0")" && pwd)
CPP_BIN=$(ros2 pkg prefix cmr_rovernet_cpp)/lib/cmr_rovernet_cpp
case $NODE_MOD in usama_control_testing) CPP_NODE=usama_control_testing_node;; *) CPP_NODE=${NODE_MOD}_node;; esac
export ROS_LOCALHOST_ONLY=1
# the vendored C++ moteus transport sets low-latency mode (TIOCGSERIAL), which a pty lacks
gcc -shared -fPIC -o /tmp/ptyshim.so "$HERE/ptyshim.c" -ldl || exit 1
PYSIGINT="import signal, runpy, sys; signal.signal(signal.SIGINT, signal.default_int_handler)"

one() {
  IMPL=$1
  export ROS_DOMAIN_ID=$([ $IMPL = py ] && echo 41 || echo 42)
  D=$OUT/out_$IMPL; rm -rf $D; mkdir -p $D
  python3 "$HERE/fake_fdcanusb.py" /tmp/fakecan_$IMPL $D/frames.log & FAKE=$!
  sleep 0.5
  if [ $IMPL = py ]; then
    python3 -c "from cmr_rovernet.command_mux import main; main()" > $D/mux.log 2>&1 & MUX=$!
    python3 -c "$PYSIGINT; runpy.run_module('cmr_rovernet.$NODE_MOD', run_name='__main__')" \
      --ros-args -p can_port:=/tmp/fakecan_$IMPL > $D/node.log 2>&1 & NODE=$!
  else
    $CPP_BIN/drive_command_mux > $D/mux.log 2>&1 & MUX=$!
    LD_PRELOAD=/tmp/ptyshim.so $CPP_BIN/$CPP_NODE \
      --ros-args -p can_port:=/tmp/fakecan_$IMPL > $D/node.log 2>&1 & NODE=$!
  fi
  echo $MUX > $D/mux.pid
  python3 "$HERE/scenario.py" $D/markers.log $D/mux.pid
  echo "$(python3 -c 'import time;print("%.4f"%time.monotonic())') STEP SIGINT" >> $D/markers.log
  kill -INT $NODE; ( sleep 8; kill -9 $NODE 2>/dev/null ) & wait $NODE; echo "node exit=$?" >> $D/node.log
  sleep 0.3; kill $FAKE 2>/dev/null
  echo "$IMPL: $(wc -l < $D/frames.log) frames, $(tail -1 $D/node.log)"
}
one cpp & one py; wait
