This program is intended to control the small gimbal designed by the University
of Tokyo's Suzuki-Tsuchiya Laboratory.

At this stage, the software is intended to run on an Atmel ATMega 168 or 328p
and communicates with an InvenSense MPU-6050 (3-axis gyro and 3-axis
accelerometer combo) and possibly Freescale MAG3110 (3-axis digital
magnetometer).

Requires avr-gcc

Currently, this software reads the MPU6050, the results of which are used to
drive a brushless motor.