import asyncio
import serial_asyncio

class FdCanInterface:
    """
    Async interface for CAN-FD via mjbots usbcanfd device.
    Supports bus configuration, one-shot transmit, continuous monitoring,
    and a request–response pattern using an 8-bit sequence in byte 0.
    """

    def __init__(self, port: str = '/dev/tty.usbmodemD8957AFF1', baud: int = 115200, timeout: float = 1.0):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.reader = None
        self.writer = None
        self._seq = 0

    async def open(self):
        self.reader, self.writer = await serial_asyncio.open_serial_connection(
            url=self.port,
            baudrate=self.baud,
        )
        await asyncio.sleep(1.0)
        await self._clear_buffer()

    async def close(self):
        if self.writer:
            self.writer.close()
            await self.writer.wait_closed()

    async def _clear_buffer(self):
        while True:
            try:
                line = await asyncio.wait_for(self.reader.readline(), timeout=0.1)
                if not line:
                    break
            except asyncio.TimeoutError:
                break

    async def _send_command(self, cmd: str) -> str:
        self.writer.write((cmd + '\n').encode('ascii'))
        await self.writer.drain()
        while True:
            line = await self.reader.readline()
            text = line.decode('ascii', errors='ignore').strip()
            if text == 'OK' or text.startswith('ERR'):
                return text

    async def configure_bus(self):
        for cmd in (
            'can off',
            'conf set can.bitrate 1000000',
            'conf set can.fdcan_frame on',
            'conf set can.bitrate_switch on',
            'conf set can.termination on',
            'can on',
        ):
            resp = await self._send_command(cmd)
            print(f'> {cmd} → {resp}')

    async def write_frame(self, std_id: int, data_hex: str, flags='FB'):
        cmd = f'can std {std_id:X} {data_hex} {flags}'
        self.writer.write((cmd + '\n').encode('ascii'))
        try:
            await asyncio.wait_for(self.writer.drain(), timeout=1.0)
        except asyncio.TimeoutError:
            print('Timeout while draining writer in write_frame()')

    async def read_loop(self):
        while True:
            raw = await self.reader.readline()
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if line.startswith('rcv '):
                parts = line.split()
                can_id = int(parts[1], 16)
                data_hex = parts[2]
                flags = parts[3:]
                print(f'RX ID=0x{can_id:X}, data={data_hex}, flags={flags}')

    async def request_response(self,
                               std_id: int,
                               payload: bytes,
                               flags: str = 'FB',
                               timeout: float = 1.0,
                               ) -> tuple[bytes, list[str]]:
        if len(payload) > 7:
            raise ValueError('Payload too long: max 7 bytes')
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        full = bytes([seq]) + payload.ljust(7, b'\x00')
        hex_str = full.hex().upper()
        await self.write_frame(std_id, hex_str, flags)
        deadline = asyncio.get_event_loop().time() + timeout
        while True:
            remaining = deadline - asyncio.get_event_loop().time()
            if remaining <= 0:
                raise asyncio.TimeoutError()
            line = await asyncio.wait_for(self.reader.readline(), timeout=remaining)
            text = line.decode('ascii', errors='ignore').strip()
            if not text.startswith('rcv '):
                continue
            parts = text.split()
            rcv_id = int(parts[1], 16)
            rcv_data = bytes.fromhex(parts[2])
            rcv_flags = parts[3:]
            if rcv_id == std_id and rcv_data and rcv_data[0] == seq:
                return rcv_data, rcv_flags
