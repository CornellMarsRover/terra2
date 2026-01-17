from pydualsense import pydualsense, TriggerModes
import time
import socket
import struct

# Network settings
# UDP_IP   = "192.168.1.102"
# UDP_IP = "10.49.43.10" # JETSON IP
# UDP_IP = "10.49.15.204"

#UDP_IP = "192.168.1.69" # JETSON IP ON RADIO
UDP_IP = "192.168.1.69" # JETSON IP ON REDROVER WIFI 
# UDP_IP = "128.253.53.211"
UDP_PORT = 5040 # 5010 - Drives, 5020 - Controller Arm, 5030 - Mini Arm

def format_data_for_udp(ds_state):
    # Assuming ds_state is an instance of DSState or similar with attributes for each button/trigger
    # Convert button states to binary representation
    # Note: Adjust these according to the actual attributes/methods provided by pydualsense for button states
    lx = int(ds_state.LX) + 128
    ly = int(ds_state.LY) + 128
    rx = int(ds_state.RX) + 128
    ry = int(ds_state.RY) + 128
    square_button   = int(ds_state.square)
    cross_button    = int(ds_state.cross)
    circle_button   = int(ds_state.circle)
    triangle_button = int(ds_state.triangle)
    l1_button       = int(ds_state.L1)
    r1_button       = int(ds_state.R1)
    l2_button       = int(ds_state.L2)  
    r2_button       = int(ds_state.R2)
    
    # Convert hat direction to binary representation, if available
    # Adjust this part according to your needs and the data provided by pydualsense
    hat_switch = 0  # Example placeholder, adjust based on available data

    if l2_button < 30: 
        l2_button = 0
    if r2_button < 30: 
        r2_button = 0

    # Combine into a single list
    data_list = [lx, ly, rx, ry, l1_button, r1_button, l2_button, r2_button, square_button, cross_button, circle_button, triangle_button, hat_switch]
    return data_list

def main():
    # Initialize controller and network settings
    ds = pydualsense()
    ds.init()
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    try:
        while True:
            # Update controller state
            #ds.update()
            #data = controller.read(64)
            # Format the controller state data for UDP transmission
            data_list = format_data_for_udp(ds.state)
            # ds.triggerR.setMode(TriggerModes.Rigid)
            # ds.triggerR.setForce(1, 255)
            for x in range(len(data_list)):
                if data_list[x] != 0 and data_list[x] != 1:
                    data_list[x] -= 1
            print(data_list)


            #labelled_data = data
            # Convert data_list to bytes
            byte_data = bytearray(data_list)
            
            # Send data over UDP
            client_socket.sendto(byte_data, (UDP_IP, UDP_PORT))
            client_socket2.sendto(byte_data, (UDP_IP, 5040))
            
            # Set touchpad color
            ds.light.setColorI(15, 0, 90)
            
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopping controller read...")
    finally:
        ds.close()  # Close the controller
        client_socket.close()

if __name__ == "__main__":
    main()