"""Mock camera replayer.

Reads a pre-encoded H.264 Annex-B byte stream from
``urc_mock_rover/assets/sample_video.h264`` and republishes each access unit
as a ``foxglove_msgs/CompressedVideo`` message. No runtime encoder is
involved — this is a byte-level scan for NAL start codes and a timed loop.

Expected bitstream: Annex-B formatted (``00 00 00 01`` or ``00 00 01`` start
codes between NAL units). Generated locally from ``ffmpeg`` by
``scripts/fetch_sample_video.sh``; the file is **not** checked in.

TODO(astrotech-q-9): camera id → logical role ("auger_cam" etc.) is a
placeholder assignment defined in ``astrotech_interfaces.yaml``. The
replayer itself is role-agnostic.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

from foxglove_msgs.msg import CompressedVideo
from rclpy.node import Node


# H.264 NAL unit types we care about.
_NAL_TYPE_NON_IDR = 1   # coded slice of a non-IDR picture
_NAL_TYPE_IDR = 5       # coded slice of an IDR picture
_NAL_TYPE_SEI = 6
_NAL_TYPE_SPS = 7
_NAL_TYPE_PPS = 8
_NAL_TYPE_AUD = 9


@dataclass(frozen=True)
class _NalUnit:
    """A single NAL unit sliced out of the bytestream.

    ``data`` contains the full Annex-B unit: leading 00 00 00 01 start code
    followed by the NALU body. Keeping the start code in each unit means
    downstream decoders can concatenate ``data`` values without fixing up
    framing.
    """

    nal_type: int
    data: bytes


def _split_annex_b(stream: bytes) -> List[_NalUnit]:
    """Split an Annex-B H.264 bytestream into NAL units.

    Supports both 3-byte (``00 00 01``) and 4-byte (``00 00 00 01``) start
    codes. Each returned unit's ``data`` starts with ``00 00 00 01`` for
    uniformity.
    """
    units: List[_NalUnit] = []
    n = len(stream)
    # Build list of (start_code_begin, body_begin).
    positions: List[tuple[int, int]] = []
    i = 0
    while i < n - 2:
        if stream[i] == 0 and stream[i + 1] == 0:
            if i + 3 < n and stream[i + 2] == 0 and stream[i + 3] == 1:
                positions.append((i, i + 4))
                i += 4
                continue
            if stream[i + 2] == 1:
                positions.append((i, i + 3))
                i += 3
                continue
        i += 1

    if not positions:
        return units

    # For each start code, body is from body_begin up to next start_code_begin.
    for idx, (_sc_begin, body_begin) in enumerate(positions):
        body_end = positions[idx + 1][0] if idx + 1 < len(positions) else n
        if body_begin >= body_end:
            continue
        nal_type = stream[body_begin] & 0x1F
        # Re-prefix with canonical 4-byte start code so consumers can glue
        # units together without guessing.
        data = b"\x00\x00\x00\x01" + stream[body_begin:body_end]
        units.append(_NalUnit(nal_type=nal_type, data=data))

    return units


def _pack_access_units(units: List[_NalUnit]) -> List[bytes]:
    """Group parameter sets with their following IDR frame.

    Output list is one entry per *frame* (one non-IDR or one
    SPS+PPS+[SEI]+IDR bundle). Non-slice NAL types that appear mid-stream
    without a following slice are attached to the *next* slice.
    """
    packed: List[bytes] = []
    pending: List[bytes] = []
    for unit in units:
        if unit.nal_type in (_NAL_TYPE_NON_IDR, _NAL_TYPE_IDR):
            if pending:
                frame = b"".join(pending) + unit.data
                pending = []
            else:
                frame = unit.data
            packed.append(frame)
        elif unit.nal_type in (_NAL_TYPE_SPS, _NAL_TYPE_PPS, _NAL_TYPE_SEI, _NAL_TYPE_AUD):
            pending.append(unit.data)
        # Other NAL types (filler, subset SPS, etc.) are dropped; they are
        # not required for playback of the synthetic testsrc asset.
    return packed


def _find_asset_path() -> Path:
    """Locate ``sample_video.h264`` relative to the installed package."""
    # When running from a colcon install tree this file lives at
    # install/urc_mock_rover/lib/python3.*/site-packages/urc_mock_rover/
    #   drivers/camera_replayer.py
    # The assets dir is a sibling of ``drivers``.
    return Path(__file__).resolve().parent.parent / "assets" / "sample_video.h264"


class MockCameraReplayer:
    """Publishes one camera feed by replaying an H.264 asset on a timer.

    Multiple instances share the asset; each keeps its own playback cursor.
    """

    def __init__(
        self,
        node: Node,
        feed_cfg: dict,
        fps: float,
    ) -> None:
        self._node = node
        self._feed_cfg = feed_cfg
        self._topic = feed_cfg["topic"]
        self._role = feed_cfg.get("role", "camera")
        self._frame_id = self._role

        asset = _find_asset_path()
        if not asset.is_file():
            raise FileNotFoundError(
                f"H.264 asset missing at {asset}. "
                "Run scripts/fetch_sample_video.sh and rebuild "
                "(ament_python installs only copy data_files listed in "
                "setup.py; the asset is loaded directly from the package "
                "source tree via __file__)."
            )
        bytestream = asset.read_bytes()
        units = _split_annex_b(bytestream)
        self._frames: List[bytes] = _pack_access_units(units)
        if not self._frames:
            raise RuntimeError(
                f"No H.264 frames parsed from {asset} "
                f"({len(bytestream)} bytes, {len(units)} NALs). "
                "Is the file Annex-B formatted?"
            )

        self._cursor = 0
        self._pub = node.create_publisher(CompressedVideo, self._topic, 10)
        self._timer = node.create_timer(1.0 / float(fps), self._tick)

        node.get_logger().info(
            f"MockCameraReplayer up: topic={self._topic} role={self._role} "
            f"frames={len(self._frames)} fps={fps}"
        )

    def _tick(self) -> None:
        data = self._frames[self._cursor]
        self._cursor = (self._cursor + 1) % len(self._frames)

        msg = CompressedVideo()
        stamp = self._node.get_clock().now().to_msg()
        msg.timestamp.sec = stamp.sec
        msg.timestamp.nanosec = stamp.nanosec
        msg.frame_id = self._frame_id
        msg.format = "h264"
        # ``uint8[]`` accepts bytes directly in rclpy (matches
        # usb_camera_publisher/publisher.py's ``vid.data = mi.data`` idiom).
        msg.data = data
        self._pub.publish(msg)
