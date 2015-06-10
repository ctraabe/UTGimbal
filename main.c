#include "main.h"

#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"
#include "eeprom.h"
#include "endian.h"
#include "i2c.h"
#include "motors.h"
#include "mpu6050_dmp.h"
#include "print.h"
#include "timer0.h"
#include "uart.h"
#include "utilities.h"


// =============================================================================
// Private data:

static volatile uint8_t _count_idle = 0, _flag_5hz = 0, _flag_125hz = 0;
static volatile enum MPU6050Mode _status_MPU6050 = MPU6050_DATA_WAITING;
// static volatile enum MAG3110Mode _status_MAG3110 = MAG3110_DATA_WAITING;


// =============================================================================
// Private functions:

static void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin B5 to output (attached to green LED).
  DDRD |= _BV(DDD2);  // Set pin D2 to output (attached to buzzer).

  I2CInit(I2C_SPEED);
  UARTInit(UART_BAUD);
  MotorPWMTimersInit();
  Timer0Init();  // Depends on Timer0 settings from MotorPWMTimersInit().

  sei();  // Enable interrupts

  Timer0Delay(100);
  BatteryMeasurementInit();

  // MPU6050RawInit();  // Read the raw gyro and accelerometer values.
  MPU6050DMPInit();  // Use the DMP quaternion estimate.
  // MAG3110Init();  // Digital compass (not connected).
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  uint8_t soft_start_shifter = 4;

  // Main loop
  for (;;) {
    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      DMPReadFIFO();
      _status_MPU6050 = MPU6050_IDLE;

/*
      static enum DMPCalibrationMode dmp_calibrate_mode = DMP_CALIBRATE_START;
      if (dmp_calibrate_mode) {
        DMPCalibrate(&dmp_calibrate_mode);
      }
*/

      float p = (float)dmp_gyro(0) * DMP_GYRO_TO_RADPS;
      float q = (float)dmp_gyro(1) * DMP_GYRO_TO_RADPS;

      float phi = dmp_roll_angle();
      float tht = dmp_pitch_angle();

      static float phi_int = 0., tht_int = 0.;
      phi_int += phi * DMP_SAMPLE_TIME;
      tht_int += tht * DMP_SAMPLE_TIME;
/*
      float motor_a_command =
        -0.037588 * p
        + 0.030793 * q
        + -0.56192 * phi
        + 0.48222 * tht
        + -9.9607 * phi_int
        + 4.559 * tht_int;
      float motor_b_command =
        0.0048041 * p
        + 0.040965 * q
        + -0.049835 * phi
        + 0.56766 * tht
        + 4.559 * phi_int
        + 9.9607 * tht_int;
*/
      float motor_a_command =
        -0.0586245 * p
        + 0.0404012 * q
        + -0.947997 * phi
        + 0.761743 * tht
        + -20.4648 * phi_int
        + 9.01067 * tht_int;
      float motor_b_command =
        0.0128858 * p
        + 0.0634179 * q
        + -0.0308123 * phi
        + 0.983245 * tht
        + 9.01067 * phi_int
        + 20.4648 * tht_int;
/*
      if (abs(motor_a_command) > MOTOR_COMMAND_LIMIT)
        break;
      if (abs(motor_b_command) > MOTOR_COMMAND_LIMIT)
        break;
*/
      MotorSetAngle(MOTOR_A, motor_a_command, soft_start_shifter);
      MotorSetAngle(MOTOR_B, motor_b_command, soft_start_shifter);

      float r = (float)dmp_gyro(2) * DMP_GYRO_TO_RADPS;

      float psi = dmp_yaw_angle();

      static float psi_int = 0.;
      psi_int += psi * DMP_SAMPLE_TIME;

      float yaw_motor_command =
        0.0648126 * r
        + 0.541236 * psi
        + 10 * psi_int;

      MotorSetAngle(MOTOR_YAW, yaw_motor_command, 0);

      // Check for frame overrun (turn on green LED)
      if (_status_MPU6050 == MPU6050_DATA_WAITING) {
        PORTB |= _BV(PORTB5);  // Green LED
      }
    }

    if (Timer0Tick()) {  // 125 Hz
      // if (I2CInactivtyCounter() > 1)
      //   I2CReset();
      static uint8_t seconds_counter = 1;
      if (!--seconds_counter) {
        seconds_counter = 125;

        // UARTPrintf("%f", (float)dmp_gyro(2) * DMP_GYRO_TO_RADPS);

        if (soft_start_shifter) --soft_start_shifter;

        if (BatteryIsLow())
          PORTD ^= _BV(PORTD2);  // Toggle buzzer
        else
          PORTD &= ~_BV(PORTD2);  // Turn off buzzer
      }
      BatteryMeasureVoltage();
    }
  }

  MotorsKill();

  for (;;) continue;
}

// -----------------------------------------------------------------------------
// Pin change interrupt on 
ISR(PCINT1_vect)
{
  // Check for an interrupt signal (high state).
  if (PINC & (_BV(PINC3))) {
      // PCINT11 (C3) is connected to MPU6050 interrupt line. This interrupt
      // should come at 200Hz in DMP mode.
    _status_MPU6050 = MPU6050_DATA_WAITING;
  }
/*
  if (PINC & (_BV(PINC3))) {  // PCINT10 connected to MAG3110 interrupt line.
    _status_MPU6050 = MPU6050_DATA_WAITING;
  }
*/
}
