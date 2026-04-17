from .FdCanInterface import FdCanInterface

class ServoController:
    def __init__(self, can: FdCanInterface, servo_id: int = 0, home: int = 0, can_id: int = 1):
        self.can = can
        self.servo_id = servo_id
        self.home = home
        self.can_id = can_id

    def _make_control_frame(self, control_mode: int, control_data: int,
                            home: int = 0, reset_home: int = 0,
                            clear_faults: int = 0, query_data: int = 0) -> bytes:
        frame = [0] * 32
        frame[0] = (1 << 6) | ((self.servo_id & 0x0F) << 2) | ((home & 0x01) << 1) | (reset_home & 0x01)
        frame[1] = ((control_mode & 0x03) << 6) | ((control_data >> 6) & 0x3F)
        frame[2] = (control_data & 0x3F) << 2
        frame[3] = ((clear_faults & 0x01) << 7) | ((query_data & 0x01) << 6)
        return bytes(frame)

    async def go_to_position(self, degrees: int):
        frame = self._make_control_frame(control_mode=0, control_data=degrees)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags='FB')

    async def stop(self):
        frame = self._make_control_frame(control_mode=2, control_data=0)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags='FB')

    async def set_home(self, degrees: int):
        frame = self._make_control_frame(control_mode=3, control_data=degrees, home=0, reset_home=1)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags='FB')

    async def go_home(self):
        frame = self._make_control_frame(control_mode=0, control_data=self.home)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags='FB')
