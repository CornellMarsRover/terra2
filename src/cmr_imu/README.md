# cmr_imu

Package to obtain data from the rover IMU.

## HWT905-RS232 runtime node

The `imu_node` executable reads the WITMOTION HWT905-RS232 WIT standard serial
stream and publishes `cmr_msgs/IMUSensorData`.

Default settings:

- Serial port: `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0`
- Baud: `9600`
- Publish topic: `/hwt905/imu`

Run manually:

```bash
ros2 run cmr_imu imu_node
```

Override the serial port if needed:

```bash
ros2 run cmr_imu imu_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p frame_topic:=/hwt905/imu
```

Check output:

```bash
ros2 topic echo /hwt905/imu
```

## HWT905-RS232 bench test

Use this to verify whether the physical IMU is streaming WIT standard RS232
frames or responding like a WIT 485/Modbus device.

Connect the sensor through a USB-RS232 adapter, not USB-TTL.

Build/source the workspace, then list serial devices:

```bash
ros2 run cmr_imu hwt905_bench_test --list-ports
```

Prefer a stable `/dev/serial/by-id/...` path if one appears. Otherwise use the
likely `/dev/ttyUSB0`.

Read the passive RS232 stream at 9600 baud and save a fixture:

```bash
ros2 run cmr_imu hwt905_bench_test \
  --port /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 \
  --baud 9600 \
  --duration 10 \
  --output hwt905_raw_hex.log
```

HWT905-RS232 should continuously show checksum-valid WIT frames:

```text
55 51 ...  acceleration
55 52 ...  gyro
55 53 ...  angle
55 54 ...  magnetic field
```

If no `55 51/52/53/54` frames appear, try common baud rates:

```bash
ros2 run cmr_imu hwt905_bench_test --port /dev/ttyUSB0 --scan-baud
```

Only if the passive stream test fails, probe the current 485/Modbus path:

```bash
ros2 run cmr_imu hwt905_bench_test --port /dev/ttyUSB0 --baud 9600 --modbus-probe
```

A likely 485 response starts with:

```text
50 03 ...
```

Interpretation:

- Repeating checksum-valid `55 51`, `55 52`, `55 53`, `55 54`: implement the
  native HWT905-RS232 WIT parser/runtime path.
- `50 03 ...` only after the Modbus probe: the current 485-style driver is
  likely the right direction.
- No data either way: check wiring, adapter type, baud, power, TX/RX crossover,
  and whether the sensor appears under another serial device.
