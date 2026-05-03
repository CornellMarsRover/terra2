import math
import re
import unittest
from pathlib import Path

from cmr_imu.hwt905_parser import HWT905Parser, is_supported_frame, normalize_degrees


ACCEL_FRAME = bytes.fromhex("55 51 0b fe c4 fe a8 07 8e 09 b7")
GYRO_FRAME = bytes.fromhex("55 52 3f 00 f9 04 6f ff a3 09 fd")
ANGLE_FRAME = bytes.fromhex("55 53 8a f8 93 05 6d f3 f5 46 5d")
MAG_FRAME = bytes.fromhex("55 54 16 05 5d 0b 23 ef 00 00 3e")


class TestHWT905Parser(unittest.TestCase):
    def test_valid_frames_decode_units(self):
        parser = HWT905Parser()
        updates = parser.parse(ACCEL_FRAME + GYRO_FRAME + ANGLE_FRAME + MAG_FRAME)

        self.assertEqual(len(updates), 4)
        sample = updates[-1]
        self.assertTrue(sample.has_accel)
        self.assertTrue(sample.has_gyro)
        self.assertTrue(sample.has_angle)
        self.assertTrue(sample.has_mag)

        self.assertAlmostEqual(sample.accx, raw_to_accel(-501))
        self.assertAlmostEqual(sample.accy, raw_to_accel(-316))
        self.assertAlmostEqual(sample.accz, raw_to_accel(1960))
        self.assertAlmostEqual(sample.gyrox, raw_to_gyro(63))
        self.assertAlmostEqual(sample.gyroy, raw_to_gyro(1273))
        self.assertAlmostEqual(sample.gyroz, raw_to_gyro(-145))
        self.assertAlmostEqual(sample.anglex, raw_to_angle(-1910))
        self.assertAlmostEqual(sample.angley, raw_to_angle(1427))
        self.assertAlmostEqual(sample.anglez, raw_to_angle(-3219))
        self.assertEqual(sample.magx, 1302)
        self.assertEqual(sample.magy, 2909)
        self.assertEqual(sample.magz, -4317)

    def test_rejects_bad_checksum(self):
        frame = bytearray(ANGLE_FRAME)
        frame[-1] ^= 0xFF

        parser = HWT905Parser()
        self.assertEqual(parser.parse(bytes(frame)), [])
        self.assertFalse(is_supported_frame(bytes(frame)))

    def test_skips_noise_before_header(self):
        parser = HWT905Parser()
        updates = parser.parse(b"\x00\x01\x02junk" + ANGLE_FRAME)

        self.assertEqual(len(updates), 1)
        self.assertTrue(updates[0].has_angle)

    def test_buffers_partial_frames(self):
        parser = HWT905Parser()
        self.assertEqual(parser.parse(ANGLE_FRAME[:5]), [])

        updates = parser.parse(ANGLE_FRAME[5:])
        self.assertEqual(len(updates), 1)
        self.assertTrue(updates[0].has_angle)

    def test_multiple_frames_in_one_read(self):
        parser = HWT905Parser()
        updates = parser.parse(ANGLE_FRAME + MAG_FRAME)

        self.assertEqual(len(updates), 2)
        self.assertTrue(updates[-1].has_angle)
        self.assertTrue(updates[-1].has_mag)

    def test_saved_yaw_log_has_clean_changing_yaw(self):
        log_path = find_fixture("hwt905_yaw_move_hex.log")
        self.assertIsNotNone(log_path, "hwt905_yaw_move_hex.log fixture not found")

        parser = HWT905Parser()
        yaws = [sample.anglez for sample in parser.parse(hex_log_bytes(log_path)) if sample.has_angle]

        self.assertGreaterEqual(len(yaws), 90)
        self.assertGreater(max(yaws) - min(yaws), 300.0)
        unwrapped = unwrap_degrees(yaws)
        self.assertGreater(abs(unwrapped[-1] - unwrapped[0]), 1000.0)

    def test_saved_static_log_has_clean_full_frame_sets(self):
        log_path = find_fixture("hwt905_raw_hex.log")
        self.assertIsNotNone(log_path, "hwt905_raw_hex.log fixture not found")

        parser = HWT905Parser()
        updates = parser.parse(hex_log_bytes(log_path))
        complete_samples = [
            sample for sample in updates
            if sample.has_accel and sample.has_gyro and sample.has_angle and sample.has_mag
        ]

        self.assertGreaterEqual(len(complete_samples), 90)
        self.assertTrue(all(-180.0 <= sample.anglez < 180.0 for sample in complete_samples))

    def test_normalize_degrees(self):
        self.assertAlmostEqual(normalize_degrees(181.0), -179.0)
        self.assertAlmostEqual(normalize_degrees(-181.0), 179.0)


def raw_to_accel(raw):
    return raw / 32768.0 * 16.0 * 9.81


def raw_to_gyro(raw):
    return math.radians(raw / 32768.0 * 2000.0)


def raw_to_angle(raw):
    return raw / 32768.0 * 180.0


def hex_log_bytes(path):
    values = re.findall(r"\b[0-9a-fA-F]{2}\b", path.read_text(encoding="utf-8"))
    return bytes(int(value, 16) for value in values)


def find_fixture(name):
    candidates = [
        Path(__file__).resolve().parent / "fixtures" / name,
        Path.cwd() / name,
    ]
    candidates.extend(parent / name for parent in Path(__file__).resolve().parents)

    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def unwrap_degrees(angles):
    unwrapped = []
    offset = 0.0
    previous = None

    for angle in angles:
        if previous is not None:
            delta = angle - previous
            if delta > 180.0:
                offset -= 360.0
            elif delta < -180.0:
                offset += 360.0
        unwrapped.append(angle + offset)
        previous = angle

    return unwrapped


if __name__ == "__main__":
    unittest.main()
