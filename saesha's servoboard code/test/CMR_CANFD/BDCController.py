from .FdCanInterface import FdCanInterface

class BDCController:
    def __init__(self, can: FdCanInterface, motor_id: int = 0, can_id: int = 2):
        self.can = can
        self.motor_id = motor_id
        self.can_id = can_id

    def _make_control_frame(self, Msg_Type: int,
                        Mode_Select: int = 0, Duration_Units: int = 0,
                        Duration: int = 0, Clear_Faults: int = 0, 
                        Query_data: int=0) -> bytes:
        
        frame = [0] * 32
        frame[0] = ((Msg_Type & 0x03) << 0) | ((self.motor_id & 0x07) << 2) | ((Mode_Select & 0x07) << 5) 
        frame[1] = ((Duration_Units & 0x03) << 0) 
        frame[2] = ((Duration & 0x7F) << 0) 
        frame[3] = ((Clear_Faults & 0x01) << 0) | ((Query_data & 0x01) << 1)

        return bytes(frame)

    # Duration sent in seconds
    async def clear_faults(self):
        frame = self._make_control_frame(Msg_Type=0, Duration_Units=1, Duration=0, Clear_Faults=1, Query_data=1)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags="FB")
        
    async def move_motor_forward(self, Duration: int,):
        frame = self._make_control_frame(Msg_Type=1, Duration_Units=1, Duration= Duration, Clear_Faults=0, Query_data=1, Mode_Select=2)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags="FB")

    async def move_motor_reverse(self, Duration: int):
        frame = self._make_control_frame(Msg_Type=3, Duration_Units=1, Duration= Duration, Clear_Faults=0, Query_data=1, Mode_Select=3)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags="FB")

    async def stop_motor(self):
        frame = self._make_control_frame(Msg_Type=0, Duration_Units=1, Duration= 0, Clear_Faults=0, Query_data=1)
        await self.can.write_frame(std_id=self.can_id, data_hex=frame.hex(), flags="FB")



