import struct
from toml import load
from os import path

#VALID SUBTEAMS
DRIVES = 0x01
ARM = 0x02
ASTROTECH = 0x03
RECEIVE_DATA = 0x04

#VALID MOTORS
FRONT_LEFT = 0x01
BACK_RIGHT = 0x04
FRONT_RIGHT = 0x03
BACK_LEFT = 0x02

#VALID ARM MOTORS
ARM_BASE = 0x09
ARM_SHOULDER = 0x0B
ARM_ELBOW = 0x0A
WRIST_ROTATE_1 = 0x0C
WRIST_ROTATE_2 = 0x0E
WRIST_TILT = 0x0D

#TRIGGERS - INDEX 0 OF BUTTON_ARRAY
L1 = 1 #0x01
R1 = 256 #0x0101
L2 = 16711680 
R2 = -16777216 #0x01010101

#BUTTONS - INDEX 1 OF BUTTON_ARRAY
SQUARE = 1 #0x01
X = 256 #0x0101
CIRCLE = 65536 #0x010101
TRIANGLE = 16777216 #0x01010101


def byte_command_converter(subteam, motor, position, drives_velocity, max_torque, max_vel, max_accel, ff_torque, logger):
    """
    Helper function to convert a given motor command into a 40-byte encoding. Current format, with 
    number of bytes written in parantheses:
    
    - SUBTEAM (1): either 'drives', 'arm', 'astrotech', 'business'
    - MOTOR (1): either 'front_right', 'front_left', 'back_right', 'back_left'
    - POSITION (4)
    - DRIVES_VELOCITY (1)
    - DIRECTION (1): derived from DRIVES_VELOCITY
    - MAX_TORQUE (4)
    - MAX_VELOCITY (4)
    - MAX_ACCEL (4) 
    - FEED FORWARD TORQUE (4)
    - EXTRA (20) 
    """
    
    #stop command - subteam byte, motor byte, then all FF bytes
    #subteam = which subteam is being controller
    #motor_id = which motor is being commanded
    #position = the position of the arm motor from 0 to 100 (0 = 0 degress or n o turn, 100 = 360 degrees or full turn )
    
    if subteam < DRIVES or subteam > RECEIVE_DATA:
        raise ValueError(f"Invalid Subteam in command")
    # if motor < FRONT_LEFT or motor > BACK_RIGHT:
    #     raise ValueError("Invalid motor_id")
    # if position is not None and position not in range(100):
    #     raise ValueError("Invalid position")
    # if drives_velocity is not None and not isinstance(drives_velocity, float) and not isinstance(drives_velocity, int):
    #     raise TypeError("drives_velocity must be an float or int")
    
    #Init hex values to be output
    
    subteam_hex = subteam #1 byte
    motor_hex = motor #1 byte
    position_hex = struct.pack('f', position) if position is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    drives_vel_hex = struct.pack('B', abs(int(drives_velocity))) if drives_velocity is not None else b'\xFF' #1 byte
    direction_hex = struct.pack('B', int(drives_velocity < 0)) if drives_velocity is not None else b'\xFF' #1 byte
    max_torque_hex = struct.pack('f', max_torque) if max_torque is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    max_vel_hex = struct.pack('f', max_vel) if max_vel is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    max_accel_hex = struct.pack('f', max_accel) if max_accel is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    ff_torque_hex = struct.pack('f', ff_torque) if ff_torque is not None else b'\xFF\xFF\xFF\xFF' #4 bytes

    # drives_vel_hex_string = drives_vel_hex.hex()
    # logger.info(f'Vel: {drives_vel_hex_string}')

    ff_torque_hex_string = ff_torque_hex.hex()
    logger.info(f'Vel: {ff_torque_hex_string}')

    # max_torque_hex_string = max_torque_hex.hex()
    # logger.info(f'Vel: {max_torque_hex_string}')

    # max_accel_hex_string = max_accel_hex.hex()
    # logger.info(f'Vel: {max_accel_hex_string}')

    # Concatenate the parts and pad to ensure the total length is 40 bytes
    output = bytes([subteam_hex, motor_hex]) + position_hex + drives_vel_hex + direction_hex + max_torque_hex + max_vel_hex + max_accel_hex + ff_torque_hex

    # output_string = output.hex()
    # logger.info(f'Output: {output_string}')
    output = output.ljust(40, b'\x00')
    return output


def scale_value(value, old_min, old_max, new_min, new_max):
    """
    Scale a value to new boundaries from old boundaries
    """
    
    return (new_max - new_min) * (value - old_min) / (old_max - old_min) + new_min


def send_number(serial_port, byte):
    """
    Send a number over the specified serial port.
    """
    
    # Convert the number to a string and encode it to bytes
    # Write the data to the serial port
    serial_port.write(byte)
    
def parse_toml(toml_name):
    """
    Helper function to parse a toml file from the "config" directory given a [toml_name]
    """
    
    folder_path = "/cmr/terra/src/cmr_rovernet/config"
    toml_file_path = path.join(folder_path, f"{toml_name}.toml")
    
    try:
        with open(toml_file_path, 'r') as file:
            toml_data = load(file)
        # Your code to work with the parsed data goes here
        return toml_data
        
    except Exception as e:
        print(f"An unexpected error occurred: {e}")


