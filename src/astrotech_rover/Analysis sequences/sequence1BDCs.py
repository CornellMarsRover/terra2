import asyncio
from CMR_CANFD import FdCanInterface, ServoController, BDCController

async def main():
    # to change port open Device Manager -> ports check
    fd = FdCanInterface(port="/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00", baud=115200)
    await fd.open()
    await fd.configure_bus()

    #SERVO BOARD PLACEHOLDER
    ee = ServoController(can=fd, servo_id=0, home=40, can_id=26)

    #-------------------------------------------------------------#

    # BDC Board 1 - BDC board with CAN ID 2 (default) - 6 motors per board
    #DMSO SHIT double green
    dmsoS1 = BDCController(can=fd, motor_id=0, can_id=18)
    dmsoS2 = BDCController(can=fd, motor_id=1, can_id=18)
    ninview = BDCController(can=fd, motor_id=2, can_id=18)
    dmsoWaste = BDCController(can=fd, motor_id=3, can_id=18)
    heattry1 = BDCController(can=fd, motor_id=4, can_id=18)
    heattry2 = BDCController(can=fd, motor_id=5, can_id=18)

    # BDC Board 2 - Additional BDC board with CAN ID 4 (example for second board)
    # H2O shit,  one blue
    h2oS1 = BDCController(can=fd, motor_id=0, can_id=20)
    h2oS2 = BDCController(can=fd, motor_id=1, can_id=20)
    h2obayers = BDCController(can=fd, motor_id=2, can_id=20)
    bayview = BDCController(can=fd, motor_id=3, can_id=20)
    mbview = BDCController(can=fd, motor_id=4, can_id=20)
    h2owaste = BDCController(can=fd, motor_id=5, can_id=20)

    #BDC Board 3 double blue, raman
    acetic_acid_1 = BDCController(can=fd, motor_id=0, can_id=21)
    led = BDCController(can=fd, motor_id=1, can_id=21)
    acetic_acid_2 = BDCController(can=fd, motor_id=2, can_id=21)

    # READ ME: 
    # Control commands are as follows:
    # 
    # # GENERAL COMMANDS
    # # asynchio.sleep(duration in seconds) -- wait command, for the inputted duration
    #
    # # BDC MOTOR MOVEMENT COMMANDS
    # # MOTOR.move_motor_forward(duration in seconds) -- moves a bdc motor forward for a certain duration in seconds
    # # MOTOR.move_motor_reverse(duration in seconds) -- moves a bdc motor in reverse for a certain duration in seconds
    # # MOTOR.stop_motor() -- stop the specified bdc motor
    # 
    # # SERVO MOTOR MOVEMENT COMMANDS
    # # SERVO.go_to_position(value in degrees) -- moves the specified servo motor to a certain position in degrees
    # # SERVO.go_home() -- returns the specified servo motor to its "home" position
    # # SERVO.set_home(value in degrees) -- sets the "home" position of a specified servo motor to a certian position in degrees


    try:
        
        await h2oS1.clear_faults()
        await h2owaste.clear_faults()
        await led.clear_faults()
        await dmsoS1.clear_faults()
        await dmsoWaste.clear_faults()
        await h2obayers.clear_faults()
        await ninview.clear_faults()
        #await heattry1.clear_faults()
        #await heattry2.clear_faults()
        await bayview.clear_faults()
        await mbview.clear_faults()
        await h2oS2.clear_faults()
        await dmsoS2.clear_faults()

        await h2oS1.stop_motor()
        await h2owaste.stop_motor()
        await led.stop_motor()
        await dmsoS1.stop_motor()
        await dmsoWaste.stop_motor()
        await h2obayers.stop_motor()
        await ninview.stop_motor()
        #await heattry1.stop_motor()
        #await heattry2.stop_motor()
        await bayview.stop_motor()
        await mbview.stop_motor()
        await h2oS2.stop_motor()
        await dmsoS2.stop_motor()


        #await h2oS1.stop_motor()
        #await h2oS1.move_motor_forward(10)
        #await acetic_acid_1.clear_faults()
        #await acetic_acid_1.stop_motor()
        #await acetic_acid_1.move_motor_forward(10)
        #await bdc1.stop_motor()
        #await bdc6.move_motor_forward(5)
    #--------------------------------------------------------------#
        #Site 1 Sequence


        #await led.move_motor_reverse(2000)

        #Simulate mixing chamber FIX THIS BITCH
        await h2oS1.move_motor_reverse(20)
        await dmsoS1.move_motor_reverse(15)
        await acetic_acid_1.move_motor_forward(20)

        await asyncio.sleep(10)
        #outta mixing
        await h2oS1.move_motor_forward(40)
        await dmsoS1.move_motor_forward(40)


        await asyncio.sleep(5.5)

        #Get dmso into cuvette
        await dmsoS1.move_motor_forward(4)
        await dmsoWaste.move_motor_reverse(4)
        await h2oS1.move_motor_forward(5)
        await h2obayers.move_motor_reverse(5)
        await asyncio.sleep(4.5)
        await led.move_motor_reverse(300)

        #get nin and dmso into cuvette
        await dmsoS1.move_motor_forward(4)
        await dmsoWaste.move_motor_reverse(4)
        await ninview.move_motor_reverse(4)
        await h2oS1.move_motor_forward(8)
        await h2owaste.move_motor_reverse(8)
        await h2obayers.move_motor_reverse(8)
        #await heattry1.move_motor_forward(45)
        #await heattry2.move_motor_forward(45)
        await asyncio.sleep(5.5) #see color change
        await bayview.move_motor_reverse(4)
        await mbview.move_motor_reverse(4)
        await h2owaste.move_motor_reverse(5)

        await asyncio.sleep(70) #wait for reaction to occur, see color change

        #Flush cuvette
        await dmsoS1.move_motor_forward(8)
        await dmsoWaste.move_motor_reverse(8)
        
        await h2owaste.move_motor_reverse(15)
        await h2obayers.move_motor_reverse(15)
        await h2oS2.move_motor_forward(15)
        await asyncio.sleep(9)

        ##NIN SITE 2
        await dmsoS2.move_motor_forward(2)
        await dmsoWaste.move_motor_reverse(2)
        await asyncio.sleep(2.5)

        await dmsoWaste.move_motor_reverse(3)
        await ninview.move_motor_reverse(3)
        await asyncio.sleep(4)
       
        
        while(1):
            await ee.go_to_position(0)
            
    finally:



        # await ee.stop()
        await fd.close()
        print("Stopped and closed the interface.")

if __name__ == "__main__":
    asyncio.run(main())