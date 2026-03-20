import struct
from toml import load
from os import path
import time
import math
import logging
import moteus
import asyncio
import threading

#VALID SUBTEAMS
DRIVES = 0x01
ARM = 0x02
ASTROTECH = 0x03
RECEIVE_DATA = 0x04
BRAKE = 0x08

#VALID MOTORS
FRONT_LEFT = 0x01
BACK_RIGHT = 0x04
FRONT_RIGHT = 0x03
BACK_LEFT = 0x02

#VALID ARM MOTORS
ARM_BASE = 0x09
ARM_SHOULDER = 0x0A
ARM_ELBOW = 0x0B
WRIST_ROTATE_1 = 0x0C
WRIST_ROTATE_2 = 0x0E
WRIST_TILT = 0x0D

#TRIGGERS - INDEX 0 OF BUTTON_ARRAY
L1 = 1 #0x01
R1 = 256 #0x0101
L2_MIN = 65536
L2 = 16711680 
R2_MIN = 26711680
R2 = -16777216 #0x01010101

#BUTTONS - INDEX 1 OF BUTTON_ARRAY
SQUARE = 1 #0x01
X = 256 #0x0101
CIRCLE = 65536 #0x010101
TRIANGLE = 16777216 #0x01010101

#SWERVE MOTORS
FRONT_LEFT_SWERVE = 0x05
BACK_RIGHT_SWERVE = 0x08
FRONT_RIGHT_SWERVE = 0x07
BACK_LEFT_SWERVE = 0x06

#WHEEL BASE IN METERS
ROVER_LENGTH = 39
ROVER_WIDTH = 39
R = math.sqrt(ROVER_LENGTH**2 + ROVER_WIDTH**2)
maxWheelVelocity = 12


# Create a global event loop that will run in another thread
_moteus_loop = None
_moteus_loop_thread = None

def init_moteus_loop():
    """
    Initializes a dedicated asyncio event loop in a separate thread.
    Call this once at the start of your node.
    """
    global _moteus_loop, _moteus_loop_thread
    if _moteus_loop is None:
        _moteus_loop = asyncio.new_event_loop()
        _moteus_loop_thread = threading.Thread(
            target=_moteus_loop.run_forever, 
            daemon=True
        )
        _moteus_loop_thread.start()

async def _send_moteus_stop_async(
    controller: moteus.Controller,
    motor: int,
    logger: logging.Logger | None = None
):
    """
    The actual async version of stopping the servo.
    """
    if logger:
        logger.info(f"[motor={motor}] calling set_stop()")
    # This tells the moteus servo to stop actively controlling 
    # (this is the 'stop' mode on the servo).
    await controller.set_stop()  # By default, query=False
    

def send_moteus_stop_sync(
    controller: moteus.Controller,
    motor: int,
    logger: logging.Logger | None = None
):
    """
    Synchronous wrapper that schedules an async stop call on our single
    global event loop thread, then waits for completion.
    """
    global _moteus_loop
    if _moteus_loop is None:
        raise RuntimeError("Moteus loop not initialized. Call init_moteus_loop() first!")

    future = asyncio.run_coroutine_threadsafe(
        _send_moteus_stop_async(
            controller=controller,
            motor=motor,
            logger=logger
        ),
        _moteus_loop
    )
    return future.result()  # block until done (or raise exception on error)

async def __async_initialize_moteus(servos: list):
    #transport = moteus.Fdcanusb()
    self.transport = moteus.Fdcanusb("/dev/ttyACM0")
    # 1, 2, 3, 4  = drives: front left, back left, front right, back right
    # 5, 6, 7, 8 = swerves: front left, back left, front right, back right
    s = {
        servo_id : moteus.Controller(id=servo_id, transport=transport) for servo_id in servos
    }

    # reset servo positions
    await transport.cycle([x.make_stop() for x in s.values()])
    await transport.cycle([x.make_rezero() for x in s.values()])
    

def initialize_moteus_sync(servos: list, logger):
    global _moteus_loop
    if _moteus_loop is None:
        raise RuntimeError("Moteus loop not initialized. Call init_moteus_loop() first!")

    future = asyncio.run_coroutine_threadsafe(
        __async_initialize_moteus(servos),
        _moteus_loop
    )
    logger.info("SENT RESET COMMAND")
    return future.result()  # block until done (or raise exception on error)

async def _send_moteus_command_async(
    controller: moteus.Controller,
    motor: int,
    position: float | None,
    drives_velocity: float | None,
    maximum_torque: float | None,
    velocity_limit: float | None,
    accel_limit: float | None,
    ff_torque: float | None,
    logger: logging.Logger | None = None
):
    """
    The actual async version of sending commands to moteus.
    (Adapt the parameter names to match the current moteus library.)
    """
    # fallback defaults
    if position is None:
        position = math.nan
    if drives_velocity is None:
        drives_velocity = math.nan
    if maximum_torque is None:
        maximum_torque = 1.0
    if velocity_limit is None:
        velocity_limit = 10.0
    if accel_limit is None:
        accel_limit = 10.0
    if ff_torque is None:
        ff_torque = 0.0

    # Optional logging
    if logger:
        logger.info(f"[motor={motor}] position={position}, velocity={drives_velocity}, "
                    f"maximum_torque={maximum_torque}, velocity_limit={velocity_limit}, "
                    f"accel_limit={accel_limit}, feedforward_torque={ff_torque}")

    # Use moteus "official" parameter names
    result = await controller.set_position(
        position=position,
        velocity=drives_velocity,
        maximum_torque=maximum_torque,
        velocity_limit=velocity_limit,
        accel_limit=accel_limit,
        feedforward_torque=ff_torque
    )

    logger.info(f"{result}")

def send_moteus_command_sync(
    controller: moteus.Controller,
    motor: int,
    position: float | None,
    drives_velocity: float | None,
    maximum_torque: float | None,
    velocity_limit: float | None,
    accel_limit: float | None,
    ff_torque: float | None,
    logger: logging.Logger | None = None
):
    """
    Synchronous wrapper that schedules an async call on the single
    global event loop thread, then waits for completion.
    """
    # Make sure our loop is initialized
    global _moteus_loop
    if _moteus_loop is None:
        raise RuntimeError("Moteus loop not initialized. Call init_moteus_loop() first!")

    future = asyncio.run_coroutine_threadsafe(
        _send_moteus_command_async(
            controller=controller,
            motor=motor,
            position=position,
            drives_velocity=drives_velocity,
            maximum_torque=maximum_torque,
            velocity_limit=velocity_limit,
            accel_limit=accel_limit,
            ff_torque=ff_torque,
            logger=logger
        ),
        _moteus_loop
    )
    # Block until done (or raise exception if error)
    return future.result()

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
    drives_vel_hex = struct.pack('f', drives_velocity) if drives_velocity is not None else b'\xFF\xFF\xFF\xFF' #4 byte
    direction_hex = struct.pack('B', drives_velocity < 0) if drives_velocity is not None else b'\xFF' #1 byte
    max_torque_hex = struct.pack('f', max_torque) if max_torque is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    max_vel_hex = struct.pack('f', max_vel) if max_vel is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    max_accel_hex = struct.pack('f', max_accel) if max_accel is not None else b'\xFF\xFF\xFF\xFF' #4 bytes
    ff_torque_hex = struct.pack('f', ff_torque) if ff_torque is not None else b'\xFF\xFF\xFF\xFF' #4 bytes

    # drives_vel_hex_string = drives_vel_hex.hex()
    # logger.info(f'Vel: {drives_vel_hex_string}')

    # direction_hex_string = direction_hex.hex()
    # logger.info(f'Direction: {direction_hex_string}')
    
    # position_hex_string = position_hex.hex()
    # logger.info(f'Direction: {position_hex_string}')

    # ff_torque_hex_string = ff_torque_hex.hex()
    # logger.info(f'Vel: {ff_torque_hex_string}')

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
    
    folder_path = "/home/cmr/cmr/terra2/src/cmr_rovernet/config"
    toml_file_path = path.join(folder_path, f"{toml_name}.toml")
    
    try:
        with open(toml_file_path, 'r') as file:
            toml_data = load(file)
        # Your code to work with the parsed data goes here
        return toml_data
        
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

drives_net_table = parse_toml("drivesnet")
drives_net_node = drives_net_table['node']
CONTROLLER_MAX_SPEED = 2.5
MOTOR_MAX_SPEED = drives_net_node['motor_max_speed']
GRADUAL_INCREASE_RATE = drives_net_node['gradual_increase_rate']
BASE_DYNAMIC_RATIO = drives_net_node['base_dynamic_ratio']
MAX_ACCELERATION = drives_net_node['acceleration_limit']
MAX_TORQUE = drives_net_node['torque_limit']
MAX_FEED_FORWARD_TORQUE = drives_net_node['feed_forward_torque_limit']

def set_speed(speed, serial_port): 
    """
    Helper function to drive forward or backword at a certain speed

    speed - Units in RPM, negative speed for reverse  
    serial_port - serial port to send information

    Example for port:
    self.port = "/dev/ttyTHS0"
    self.baud_rate = 115200
    self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
    """
    back_right =  byte_command_converter(DRIVES, BACK_RIGHT, None, -speed, MAX_TORQUE, None,  MAX_ACCELERATION, 0, None)
    front_right = byte_command_converter(DRIVES, FRONT_RIGHT, None, -speed, MAX_TORQUE, None, MAX_ACCELERATION, 0, None)
    front_left =  byte_command_converter(DRIVES, FRONT_LEFT, None, speed, MAX_TORQUE, None,   MAX_ACCELERATION, 0, None)
    back_left =   byte_command_converter(DRIVES, BACK_LEFT, None, speed, MAX_TORQUE, None,    MAX_ACCELERATION, 0, None)
    send_number(serial_port, back_right)
    send_number(serial_port, front_right)
    send_number(serial_port, front_left)
    send_number(serial_port, back_left)
    

def set_speed_forward_timed(speed, drive_time, serial_port):
    """
    Helper function to drive forward or backword at a certain speed 
    for a certain amount of time

    speed - Units in RPM, negative speed for reverse  
    drive_time - duration of drive
    serial_port - serial port to send information

    Example for port:
    self.port = "/dev/ttyTHS0"
    self.baud_rate = 115200
    self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
    """
    set_speed(speed, serial_port)
    time.sleep(drive_time)
    set_speed(0, serial_port)
 
def drive_distance(speed, distance, wheel_diameter, serial_port):
    """
    Helper function to drive forward or backword for a set distance

    speed - Units in RPM, negative speed for reverse  
    distance - distance in inches
    wheel_diameter - Right now, the wheel diameter is 11 inches
    serial_port - serial port to send information

    Example for port:
    self.port = "/dev/ttyTHS0"
    self.baud_rate = 115200
    self.serial_port = serial.Serial(self.port, self.baud_rate, timeout=1)
    """    
    pi = 3.14159
    time_in_minutes = distance / ((pi * wheel_diameter) * speed)
    set_speed_forward_timed(speed, time_in_minutes*60, serial_port)
    

