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

static volatile enum MPU6050Mode status_MPU6050_ = MPU6050_DATA_WAITING;
static enum GimbalStatus status_ = GIMBAL_STATUS_MOTOR_STARTING;
// static volatile enum MAG3110Mode _status_MAG3110 = MAG3110_DATA_WAITING;


// =============================================================================
// Private functions:

static void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin B5 to output (attached to green LED).
  DDRD |= _BV(DDD2);  // Set pin D2 to output (attached to buzzer).
  PORTB |= _BV(PORTB4);  // Pull-up pin B4 (attached to the button).

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
static void RollPitchController(uint8_t starting)
{
  float p = (float)dmp_gyro(0) * DMP_GYRO_TO_RADPS;
  float q = (float)dmp_gyro(1) * DMP_GYRO_TO_RADPS;

  float phi = dmp_roll_angle();
  float tht = dmp_pitch_angle();

  static float phi_int = 0., tht_int = 0., neutral_a = 0., neutral_b = 0.;
  phi_int += phi * DMP_SAMPLE_TIME;
  tht_int += tht * DMP_SAMPLE_TIME;

  float motor_a_command, motor_b_command;
  if (!starting)
  {
    motor_a_command =
      -0.0586245 * p
      + 0.0404012 * q
      + -0.947997 * phi
      + 0.761743 * tht
      + -20.4648 * phi_int
      + 9.01067 * tht_int;
    motor_b_command =
      0.0128858 * p
      + 0.0634179 * q
      + -0.0308123 * phi
      + 0.983245 * tht
      + 9.01067 * phi_int
      + 20.4648 * tht_int;
  }
  else
  {
    motor_a_command = -2. * phi_int + 1. * tht_int;
    motor_b_command = 1. * phi_int + 2. * tht_int;

    neutral_a = motor_a_command;
    neutral_b = motor_b_command;
  }

  if ( abs(motor_a_command - neutral_a) > MOTOR_ANGLE_LIMIT
    || abs(motor_b_command - neutral_b) > MOTOR_ANGLE_LIMIT
    || abs(MotorSetAngle(MOTOR_A, motor_a_command)) > MOTOR_SEGMENT_LIMIT
    || abs(MotorSetAngle(MOTOR_B, motor_b_command)) > MOTOR_SEGMENT_LIMIT
    )
  {
    status_ = GIMBAL_STATUS_ERROR;
  }
}

// -----------------------------------------------------------------------------
static void YawController(uint8_t starting)
{
  float r = (float)dmp_gyro(2) * DMP_GYRO_TO_RADPS;

  float psi = dmp_yaw_angle();

  static float psi_int = 0., neutral = 0.;
  psi_int += psi * DMP_SAMPLE_TIME;

  float motor_command;
  if (!starting)
  {
    motor_command =
      0.0648126 * r
      + 0.541236 * psi
      + 10. * psi_int;
  }
  else
  {
    motor_command = 1.0 * psi_int;
    neutral = motor_command;
  }

  if ( abs(motor_command - neutral) > YAW_MOTOR_ANGLE_LIMIT
    || abs(MotorSetAngle(MOTOR_YAW, motor_command)) > YAW_MOTOR_SEGMENT_LIMIT
    )
  {
    status_ = GIMBAL_STATUS_ERROR;
  }
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  enum DMPCalibrationMode dmp_calibrate_mode = DMP_CALIBRATE_DONE;
  uint8_t control_starting_timer = 250;

  // Main loop
  for (;;) {

    if (status_MPU6050_ == MPU6050_DATA_WAITING)
    {
      DMPReadFIFO();
      status_MPU6050_ = MPU6050_IDLE;

      if (status_ == GIMBAL_STATUS_RUNNING)
      {
        RollPitchController(control_starting_timer);
        YawController(control_starting_timer);

        if (status_ != GIMBAL_STATUS_RUNNING) break;
      }
      else if (dmp_calibrate_mode)
      {
        DMPCalibrate(&dmp_calibrate_mode);
      }
      else if (status_ == GIMBAL_STATUS_MOTOR_STARTING)
      {
        static uint8_t button_pv = _BV(PINB4);

        if (button_pv & ~(PINB & _BV(PINB4)))
        {
          MotorsKill();
          dmp_calibrate_mode = DMP_CALIBRATE_START;
          status_ = GIMBAL_STATUS_CALIBRATION;
        }
        else if (MotorsStartup())
        {
          status_ = GIMBAL_STATUS_RUNNING;
        }

        button_pv = PINB & _BV(PINB4);
      }

      // Check for frame overrun (turn on green LED)
      if (status_MPU6050_ == MPU6050_DATA_WAITING)
        PORTB |= _BV(PORTB5);  // Green LED
    }

    if (Timer0Tick())  // 125 Hz
    {
      static uint8_t seconds_counter = 125;
      if (!--seconds_counter) {
        seconds_counter = 125;

        // Check for low voltage.
        if (BatteryIsLow())
          PORTD ^= _BV(PORTD2);  // Toggle buzzer
        else
          PORTD &= ~_BV(PORTD2);  // Turn off buzzer
      }

      BatteryMeasureVoltage();
    }
  }

  MotorsKill();

  for (;;)
  {
    if (Timer0Tick())  // 125 Hz
    {
      static uint8_t counter_25hz = 1;
      if (!--counter_25hz) {
        counter_25hz = 125 / 25;

        if (control_starting_timer && status_ == GIMBAL_STATUS_RUNNING)
          control_starting_timer--;

        PORTB ^= _BV(PORTB5);  // Fast blink indicates an error occurred
      }
    }
  }
}

// -----------------------------------------------------------------------------
// Pin change interrupt on 
ISR(PCINT1_vect)
{
  // Check for an interrupt signal (high state).
  if (PINC & _BV(PINC3)) {
      // PCINT11 (C3) is connected to MPU6050 interrupt line. This interrupt
      // should come at 200Hz in DMP mode.
    status_MPU6050_ = MPU6050_DATA_WAITING;
  }
/*
  if (PINC & _BV(PINC3)) {  // PCINT10 connected to MAG3110 interrupt line.
    status_MPU6050_ = MPU6050_DATA_WAITING;
  }
*/
}
