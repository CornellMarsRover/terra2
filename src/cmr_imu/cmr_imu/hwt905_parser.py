import math
from dataclasses import dataclass
from typing import List, Optional


FRAME_HEADER = 0x55
FRAME_LENGTH = 11
ACCEL_FRAME = 0x51
GYRO_FRAME = 0x52
ANGLE_FRAME = 0x53
MAG_FRAME = 0x54
GRAVITY_MPS2 = 9.81


@dataclass
class HWT905Sample:
    temp: float = 0.0
    accx: float = 0.0
    accy: float = 0.0
    accz: float = 0.0
    gyrox: float = 0.0
    gyroy: float = 0.0
    gyroz: float = 0.0
    anglex: float = 0.0
    angley: float = 0.0
    anglez: float = 0.0
    magx: int = 0
    magy: int = 0
    magz: int = 0
    has_accel: bool = False
    has_gyro: bool = False
    has_angle: bool = False
    has_mag: bool = False

    def copy(self) -> "HWT905Sample":
        return HWT905Sample(
            temp=self.temp,
            accx=self.accx,
            accy=self.accy,
            accz=self.accz,
            gyrox=self.gyrox,
            gyroy=self.gyroy,
            gyroz=self.gyroz,
            anglex=self.anglex,
            angley=self.angley,
            anglez=self.anglez,
            magx=self.magx,
            magy=self.magy,
            magz=self.magz,
            has_accel=self.has_accel,
            has_gyro=self.has_gyro,
            has_angle=self.has_angle,
            has_mag=self.has_mag,
        )


class HWT905Parser:
    def __init__(self):
        self._buffer = bytearray()
        self._sample = HWT905Sample()

    def parse(self, data: bytes) -> List[HWT905Sample]:
        updates = []
        self._buffer.extend(data)

        while len(self._buffer) >= FRAME_LENGTH:
            if self._buffer[0] != FRAME_HEADER:
                del self._buffer[0]
                continue

            frame = bytes(self._buffer[:FRAME_LENGTH])
            if not is_supported_frame(frame):
                del self._buffer[0]
                continue

            del self._buffer[:FRAME_LENGTH]
            sample = self._apply_frame(frame)
            if sample is not None:
                updates.append(sample.copy())

        return updates

    @property
    def sample(self) -> HWT905Sample:
        return self._sample.copy()

    def _apply_frame(self, frame: bytes) -> Optional[HWT905Sample]:
        frame_type = frame[1]
        x = int16_le(frame[2], frame[3])
        y = int16_le(frame[4], frame[5])
        z = int16_le(frame[6], frame[7])
        temp = int16_le(frame[8], frame[9]) / 100.0

        if frame_type == ACCEL_FRAME:
            self._sample.accx = x / 32768.0 * 16.0 * GRAVITY_MPS2
            self._sample.accy = y / 32768.0 * 16.0 * GRAVITY_MPS2
            self._sample.accz = z / 32768.0 * 16.0 * GRAVITY_MPS2
            self._sample.temp = temp
            self._sample.has_accel = True
            return self._sample

        if frame_type == GYRO_FRAME:
            self._sample.gyrox = math.radians(x / 32768.0 * 2000.0)
            self._sample.gyroy = math.radians(y / 32768.0 * 2000.0)
            self._sample.gyroz = math.radians(z / 32768.0 * 2000.0)
            self._sample.temp = temp
            self._sample.has_gyro = True
            return self._sample

        if frame_type == ANGLE_FRAME:
            self._sample.anglex = x / 32768.0 * 180.0
            self._sample.angley = y / 32768.0 * 180.0
            self._sample.anglez = z / 32768.0 * 180.0
            self._sample.temp = temp
            self._sample.has_angle = True
            return self._sample

        if frame_type == MAG_FRAME:
            self._sample.magx = x
            self._sample.magy = y
            self._sample.magz = z
            self._sample.temp = temp
            self._sample.has_mag = True
            return self._sample

        return None


def int16_le(low_byte: int, high_byte: int) -> int:
    return int.from_bytes(bytes((low_byte, high_byte)), "little", signed=True)


def is_supported_frame(frame: bytes) -> bool:
    if len(frame) != FRAME_LENGTH:
        return False
    if frame[0] != FRAME_HEADER:
        return False
    if frame[1] not in (ACCEL_FRAME, GYRO_FRAME, ANGLE_FRAME, MAG_FRAME):
        return False
    return (sum(frame[:10]) & 0xFF) == frame[10]


def normalize_degrees(angle: float) -> float:
    return float((angle + 180.0) % 360.0 - 180.0)
