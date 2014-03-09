This program is intended to control the small gimbal designed by the University
of Tokyo's Suzuki-Tsuchiya Laboratory.

At this stage, the software is intended to run on an Atmel ATMega 168 and
communicates with an InvenSense MPU-6050 (3-axis gyro and 3-axis accelerometer
combo) and Freescale MAG3110 (3-axis digital magnetometer).

Requires avr-gcc

Currently, this software explores reading the sensors in order to develop fusion
algorithms. It is expected that the software will later be transitioned to
different microcontrollers and brushless motor control.