/*
 * This file is copied from micro/CCB4/Core/Inc/boards.h
 * It's imperative that all the IDs here match the ones present on the CCB,
 * so keep this file in sync with the CCB firmware's copy.
 */
#ifndef INC_BOARDS_H_
#define INC_BOARDS_H_

// address SUBTEAM|BOARD TYPE|NUMBER -> xx|xx|xxxx
// used to determine can priority

/*--------------SUBTEAMS--------------------*/
#define ECE 0x00
#define DRIVES 0x40
#define ARM 0x80
#define ASTRO 0xC0

/*--------------BOARD TYPES-----------------*/

#define BLDC 0x00
#define BDC 0x10
#define SERVO 0x20
#define SENSOR 0x30

/*--------------ELECTRICAL BOARD ID---------*/

// If need to add more PDB just increase CCB
// and VOLTM right most value by 1 then follow same PDB rule as shown below

#define NUM_PDB 2

#define PDB1 ECE | 0x0
#define PDB2 ECE | 0x1

#define CCB ECE | 0x2

#define VOLTM ECE | 0x3

#define SENSOR1 ECE | SENSOR | 0x0
#define SENSOR2 ECE | SENSOR | 0x1

/*--------------DRIVES BOARD ID-------------*/
// LEFT SIDE
#define BLDC_D1 DRIVES | BLDC | 0x0
#define BLDC_D2 DRIVES | BLDC | 0x1
#define BLDC_D3 DRIVES | BLDC | 0x2
// RIGHT SIDE
#define BLDC_D4 DRIVES | BLDC | 0x3
#define BLDC_D5 DRIVES | BLDC | 0x4
#define BLDC_D6 DRIVES | BLDC | 0x5

#define BDC_D1 DRIVES | BDC | 0x0

#define SERVO_PAN1 DRIVES | SERVO | 0x0
#define SERVO_TILT1 DRIVES | SERVO | 0x1
#define SERVO_PAN2 DRIVES | SERVO | 0x2
#define SERVO_TILT2 DRIVES | SERVO | 0x3

/*--------------ARM BOARD ID----------------*/

#define BLDC_AR1 ARM | BLDC | 0x6
#define BLDC_AR2 ARM | BLDC | 0x7
#define BLDC_AR3 ARM | BLDC | 0x8
#define BLDC_AR4 ARM | BLDC | 0x9
#define BLDC_AR5 ARM | BLDC | 0xA
#define BLDC_AR6 ARM | BLDC | 0xB

#define BDC_EXT ARM | BDC | 0x1
#define BDC_ENDO ARM | BDC | 0x2

#define SERVO_ENDO ARM | SERVO | 0x4

/*--------------ASTROTECH BOARD ID----------*/

#define BDC_LEAD ASTRO | BDC | 0x3
#define BDC_SCOOP ASTRO | BDC | 0x4
#define BDC_MIX1 ASTRO | BDC | 0x5
#define BDC_MIX2 ASTRO | BDC | 0x6
#define BDC_MIX3 ASTRO | BDC | 0x7
#define BDC_MIX4 ASTRO | BDC | 0x8
#define BDC_PUMP1 ASTRO | BDC | 0x9
#define BDC_PUMP2 ASTRO | BDC | 0xA
#define BDC_PUMP3 ASTRO | BDC | 0xB
#define BDC_PUMP4 ASTRO | BDC | 0xC
#define BDC_ROTATE ASTRO | BDC | 0xD
#define BDC_COLL ASTRO | BDC | 0xE

#define SENSOR_NPK ASTRO | SENSOR | 0x2

/*--------------FUNCTIONS------------------*/

/*--------------BLDC functions-------------*/
// writes
#define BLDC_VEL_W 0x00
#define BLDC_POSITION_W 0x01
#define BLDC_EFFORT_W 0x02  // do not use

// reads
#define BLDC_VEL_R 0x00
#define BLDC_POSITION_R 0x01

/*--------------BDC functions--------------*/
// writes
#define BLDC_VEL_W 0x00       // no support
#define BLDC_POSITION_W 0x01  // no support
#define BLDC_EFFORT_W 0x02

/*--------------SERVO functions------------*/
// writes
#define SERVO_VEL_W 0x00
#define BLDC_POSITION_W 0x01
#define BLDC_EFFORT_W 0x02  // no support

/*--------------SENSOR functions-----------*/

// write
#define SENSOR_TRAVEL 0x00
#define SENSOR_SCIENCE 0x01

// reads
// board will automatically send data every second without us asking
#define LIDAR_R 0x00
#define IMU_DATA_accel 0x01  // split into accel, gyro, and mag
#define IMU_DATA_gyro 0x02
#define IMU_DATA_mag 0x03
#define CO2_R1 0x04

#define CO2_R2 0x05
#define CO2_R3 0x06
#define CO2_R4 0x07

/*--------------NPK functions--------------*/

/*--------------PDB functions--------------*/
// if address is all 0 then off
// cast address |PORT_ON|HPPx or PORT_OFF|LPPx or ALL_OFF

#define ALL_OFF 0x00

#define PORT_OFF 0x20
#define PORT_ON 0x30  // 0x40

#define HPP1 0x01
#define HPP2 0x02
#define HPP3 0x03
#define HPP4 0x04
#define HPP5 0x05
#define HPP6 0x06
#define HPP7 0x07

#define LPP1 0x08
#define LPP2 0x09
#define LPP3 0x0A
#define LPP4 0x0B
#define LPP5 0x0C
#define LPP6 0x0D
#define LPP7 0x0F
#define LPP8 0x10

/*--------------VOLTAGE MON functions------*/

#define VOLTAGE_R 0x08

#endif /* INC_BOARDS_H_ */
