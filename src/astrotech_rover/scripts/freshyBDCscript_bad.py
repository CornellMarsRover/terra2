import asyncio

from CMR_CANFD import FdCanInterface, ServoController, BDCController



async def main():

    # to change port open Device Manager -> ports check

    #fd = FdCanInterface(port="COM11", baud=115200)

    fd = FdCanInterface(port="/dev/serial/by-id/usb-mjbots_fdcanusb_8249C85D-if00", baud=115200)

    await fd.open()

    await fd.configure_bus()



    # Board 1 - Servo board with CAN ID 1 (default)



    # Board 2 - Additional servo board with CAN ID 3 (example for second board)

    # Uncomment to use a second servo board

    # ee16 = ServoController(can=fd, servo_id=0, home=40, can_id=3)

    # ee17 = ServoController(can=fd, servo_id=1, home=60, can_id=3)



    # Board 1 - BDC board with CAN ID 2 (default) - 6 motors per board

    #DMSO SHIT double green

    dmsoS1 = BDCController(can=fd, motor_id=0, can_id=18)

    dmsoS2 = BDCController(can=fd, motor_id=1, can_id=18)

    ninview = BDCController(can=fd, motor_id=2, can_id=18)

    dmsoWaste = BDCController(can=fd, motor_id=3, can_id=18)

    heattry1 = BDCController(can=fd, motor_id=4, can_id=18)

    heattry2 = BDCController(can=fd, motor_id=5, can_id=18)



    # Board 2 - Additional BDC board with CAN ID 4 (example for second board)

    # H2O shit,  one blue

    h2oS1 = BDCController(can=fd, motor_id=0, can_id=20)

    h2oS2 = BDCController(can=fd, motor_id=1, can_id=20)

    h2obayers = BDCController(can=fd, motor_id=2, can_id=20)

    bayview = BDCController(can=fd, motor_id=3, can_id=20)

    mbview = BDCController(can=fd, motor_id=4, can_id=20)

    h2owaste = BDCController(can=fd, motor_id=5, can_id=20)



    #double blue, raman

    acetic_acid_1 = BDCController(can=fd, motor_id=0, can_id=21)

    led = BDCController(can=fd, motor_id=1, can_id=21)

    acetic_acid_2 = BDCController(can=fd, motor_id=2, can_id=21)



    try:

        """

        await led.move_motor_reverse(200)



        #NIN

        #Simulate DMSO mixing chamber

        await dmsoS1.move_motor_forward(1)



        await asyncio.sleep(1.5)



        #Get dmso into cuvette

        await dmsoS1.move_motor_forward(4)

        await dmsoWaste.move_motor_reverse(4)

        await asyncio.sleep(4.5)



        #get nin and dmso into cuvette

        await dmsoS1.move_motor_forward(4)

        await dmsoWaste.move_motor_reverse(4)

        await ninview.move_motor_reverse(4)

        await asyncio.sleep(20)



        #Flush cuvette

        await dmsoS1.move_motor_forward(8)

        await dmsoWaste.move_motor_reverse(8)

        await asyncio.sleep(9)



        ##NIN SITE 2

        await dmsoS2.move_motor_forward(2)

        await dmsoWaste.move_motor_reverse(2)

        await asyncio.sleep(2.5)



        await dmsoWaste.move_motor_reverse(3)

        await ninview.move_motor_reverse(3)

        await asyncio.sleep(20)





        #flush out

        #await dmsoS1.move_motor_reverse(3)

        #await dmsoS2.move_motor_reverse(3)

        #await dmsoWaste.move_motor_reverse(6)



        #await ninview.move_motor_forward(10)

        #await dmsoWaste.move_motor_reverse(10)

        #"""

        #"""

        #await dmsoS1.move_motor_reverse(10)

        #await dmsoS2.move_motor_reverse(10)

        #await ninview.move_motor_forward(10)

        #await dmsoWaste.move_motor_reverse(10)

        #"""



        """

        #WATER SYSTEM

        #Simulate / skip mixing chamber

        await asyncio.sleep(20)

        await h2oS1.move_motor_forward(1)

        await led.move_motor_reverse(15)

        #await bayview.move_motor_reverse(1)

        #await mbview.move_motor_reverse(1)



        await asyncio.sleep(1.5)



        await h2oS1.move_motor_forward(5)

        await h2obayers.move_motor_reverse(5)



        await asyncio.sleep(5.5) #make this wait for a minute



        await h2oS1.move_motor_forward(8)

        await h2owaste.move_motor_reverse(8)

        await h2obayers.move_motor_reverse(8)



        await asyncio.sleep(8.5)



        await bayview.move_motor_reverse(4)

        await mbview.move_motor_reverse(4)

        await h2owaste.move_motor_reverse(5)

        #await asyncio.sleep(4.5)



        await asyncio.sleep(10) #see color change



        #FLUSH AND PREPARE SITE 2

        await h2owaste.move_motor_reverse(15)

        await h2obayers.move_motor_reverse(15)

        await h2oS2.move_motor_forward(15)



        await asyncio.sleep(20) #see color change



        await bayview.move_motor_reverse(4)

        await mbview.move_motor_reverse(4)

        await h2owaste.move_motor_reverse(5)



        """

        #REVERSING

        #await h2oS2.move_motor_reverse(10)

        await h2oS1.move_motor_reverse(10)

        #await raman.move_motor_reverse(5)

        #await bayview.move_motor_forward(10)

        #await mbview.move_motor_forward(10)

        #await h2owaste.move_motor_reverse(10)

        await h2obayers.move_motor_forward(10)

        #"""



        while(1):

            # await ee.go_to_position(0)

            # await asyncio.sleep(0.02)

            # await ee2.go_to_position(0)

            # await asyncio.sleep(0.40)

            # await bdc.move_motor_forward(2)

            # await bdc1.move_motor_reverse(3)

            # await asyncio.sleep(1.5)

            # await ee.go_to_position(180)

            # await asyncio.sleep(0.02)

            # await ee2.go_to_position(180)

            #await bdc4.move_motor_forward(5)

            #await asyncio.sleep(1.5)





            # print("sending CAN command")

            # await ee.go_to_position(30)

            # await ee.go_to_position(10)

            # await asyncio.sleep(0.5)  # 2 Hz send rate

            # await ee.go_to_position(180)

            # await asyncio.sleep(0.02)   # 20 ms spacing

            # await bdc.move_motor_forward(5)

            # await bdc1.move_motor_forward(5)

            # # await ee2.go_to_position(0)

            # await asyncio.sleep(0.02)

            # await ee4.go_to_position(0)

            # await asyncio.sleep(0.02)

            # await ee7.go_to_position(0)

            # await asyncio.sleep(0.40)   # remaining time

            # await asyncio.sleep(0.5)

            # await ee.go_to_position(50)

            # await asyncio.sleep(0.5)

            # await ee.go_to_position(70)

            # await asyncio.sleep(0.5)

            # await ee.go_to_position(90)

            # await asyncio.sleep(0.5)

            # await ee.go_to_position(110)

            # await asyncio.sleep(0.5)

            # await ee.go_to_position(130)

            #await ee.go_to_position(90)

            #await asyncio.sleep(1)

            #await ee.go_to_position(30)

            await asyncio.sleep(1.0)

        # await ee.go_to_position(20)

        # await ee.set_home(40)

        # await ee1.set_home(0)

        # await ee2.set_home(0)

        # await ee3.set_home(0)

        # await ee4.set_home(0)

        # await ee5.set_home(0)

        # await ee6.set_home(0)

        # await ee7.set_home(0)

        # await ee8.set_home(0)

        # await ee9.set_home(0)

        # await ee10.set_home(0)

        # await ee11.set_home(0)

        # await ee12.set_home(0)

        # await ee13.set_home(0)

        # await ee14.set_home(0)

        # await ee15.set_home(0)

        # print("Set home.")

        # await asyncio.sleep(2.0)

        # for deg in range(0, 211, 1):

        #     await ee.go_to_position(deg)

        #     await ee1.go_to_position(deg)

        #     await ee2.go_to_position(deg)

        #     await ee3.go_to_position(deg)

        #     await ee4.go_to_position(deg)

        #     await ee5.go_to_position(deg)

        #     await ee6.go_to_position(deg)

        #     await ee7.go_to_position(deg)

        #     await ee8.go_to_position(deg)

        #     await ee9.go_to_position(deg)

        #     await ee10.go_to_position(deg)

        #     await ee11.go_to_position(deg)

        #     await ee12.go_to_position(deg)

        #     await ee13.go_to_position(deg)

        #     await ee14.go_to_position(deg)

        #     await ee15.go_to_position(deg)

        #     print(f"Moved to {deg}°")

        #     await asyncio.sleep(0.01)

        # for deg in range(180, -1, -1):

        #     await ee.go_to_position(deg)

        #     await ee1.go_to_position(deg)

        #     await ee2.go_to_position(deg)

        #     await ee3.go_to_position(deg)

        #     await ee4.go_to_position(deg)

        #     await ee5.go_to_position(deg)

        #     await ee6.go_to_position(deg)

        #     await ee7.go_to_position(deg)

        #     await ee8.go_to_position(deg)

        #     await ee9.go_to_position(deg)

        #     await ee10.go_to_position(deg)

        #     await ee11.go_to_position(deg)

        #     await ee12.go_to_position(deg)

        #     await ee13.go_to_position(deg)

        #     await ee14.go_to_position(deg)

        #     await ee15.go_to_position(deg)

        #     print(f"Moved to {deg}°")

        #     await asyncio.sleep(0.01)

        #await ee.go_home()

        # await ee1.go_home()

        # await ee2.go_home()

        # await ee3.go_home()

        # await ee4.go_home()

        # await ee5.go_home()

        # await ee6.go_home()

        # await ee7.go_home()

        # await ee8.go_home()

        # await ee9.go_home()

        # await ee10.go_home()

        # await ee11.go_home()

        # await ee12.go_home()

        # await ee13.go_home()

        # await ee14.go_home()

        # await ee15.go_home()

        # print("Returned to home position.")

        # await asyncio.sleep(2.0)

        # while(1):

        #     await ee.go_to_position(100)

        #     print("Moved to 100°")

        #     await asyncio.sleep(10.0)

        #     await ee.go_to_position(180)

        #     print("Moved to 180°")

        #     await asyncio.sleep(2.0)
    finally:
        # await ee.stop()
        await fd.close()
        print("Stopped and closed the interface.")



if __name__ == "__main__":

    asyncio.run(main())
