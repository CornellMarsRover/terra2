import struct

VALID_SUBTEAMS = ["arm", "drives", "astrotech", "business", "homeless san fran ben dodson"]
VALID_MOTORS = ["front_right", "front_left", "back_right", "back_left"]

#subteam = which subteam is being controller
#motor_id = which motor is being commanded
#position = the position of the arm motor from 0 to 100 (0 = 0 degress or n o turn, 100 = 360 degrees or full turn )

def byte_command_converter(subteam, motor, position, drives_velocity, max_torque, max_vel, max_accel, logger):
    if subteam not in VALID_SUBTEAMS:
        raise ValueError(f"Invalid Subteam in command, needs to be one of {VALID_SUBTEAMS}")
    if motor not in VALID_MOTORS:
        raise ValueError("Invalid motor_id")
    if position is not None and position not in range(100):
        raise ValueError("Invalid position")
    if not isinstance(drives_velocity, float) and not isinstance(drives_velocity, int) :
        raise TypeError("drives_velocity must be an float or int")
    
    #Init hex values to be output


    subteam_ids = {"drives": 0x01, "arm": 0x02, "astrotech": 0x03}
    motor_ids = {"front_left": 0x01, "front_right": 0x03, "back_right": 0x02, "back_left": 0x04}
    
    subteam_hex = subteam_ids.get(subteam, 0x00)
    motor_hex = motor_ids.get(motor, 0x00)
    position_hex = struct.pack('f', position) if position is not None else b'\xFF\xFF\xFF\xFF'
    drives_vel_hex = struct.pack('B', abs(int(drives_velocity)))
    direction_hex = struct.pack('B', int(drives_velocity < 0))
    max_torque_hex = struct.pack('f', max_torque) if max_torque is not None else b'\xFF\xFF\xFF\xFF'
    max_vel_hex = struct.pack('f', max_vel) if max_vel is not None else b'\xFF\xFF\xFF\xFF'
    max_accel_hex = struct.pack('f', max_accel) if max_accel is not None else b'\xFF\xFF\xFF\xFF'

    # drives_vel_hex_string = drives_vel_hex.hex()
    # logger.info(f'Vel: {drives_vel_hex_string}')

    # max_torque_hex_string = max_torque_hex.hex()
    # logger.info(f'Vel: {max_torque_hex_string}')

    # max_accel_hex_string = max_accel_hex.hex()
    # logger.info(f'Vel: {max_accel_hex_string}')

    # Concatenate the parts and pad to ensure the total length is 40 bytes
    output = bytes([subteam_hex, motor_hex]) + position_hex + drives_vel_hex + direction_hex + max_torque_hex + max_vel_hex + max_accel_hex

    output_string = output.hex()
    logger.info(f'Output: {output_string}')
    output = output.ljust(40, b'\x00')

    return output


def scale_value(value, old_min, old_max, new_min, new_max):
    # Scale the old range to the new range
    return (new_max - new_min) * (value - old_min) / (old_max - old_min) + new_min


def send_number(self, serial_port, byte):
    """
    Send a number over the specified serial port.
    """
    # Convert the number to a string and encode it to bytes
    
    # Write the data to the serial port
    serial_port.write(byte)


