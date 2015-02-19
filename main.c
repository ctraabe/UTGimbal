#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"
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
void UARTPrintS16(int16_t value)
{
  uint8_t ascii[8] = { '0', '0', '0', '0', '0', '0', '\r', '\n' };
  PrintS16(value, ascii);
  UARTTxBytes(ascii, 8);
}

// -----------------------------------------------------------------------------
void Calibrate(void)
{
  // int16_t temp = dmp_accelerometer(0);
  // int16_t temp = dmp_accelerometer(1);
  // int16_t temp = dmp_accelerometer(2);
  // int16_t temp = dmp_gyro(0);
  int16_t temp = dmp_gyro(1);
  // int16_t temp = dmp_gyro(2);

  static int16_t offset = 0;
  if (temp > 0) --offset;
  else ++offset;

  // MPU6050SetAccelerometerBias(MPU6050_X_AXIS, offset);
  // MPU6050SetAccelerometerBias(MPU6050_Y_AXIS, offset);
  // MPU6050SetAccelerometerBias(MPU6050_Z_AXIS, offset);
  // MPU6050SetGyroBias(MPU6050_X_AXIS, offset);
  MPU6050SetGyroBias(MPU6050_Y_AXIS, offset);
  // MPU6050SetGyroBias(MPU6050_Z_AXIS, offset);

  UARTPrintS16(offset);
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  int16_t dmp_gyro_pv[3] = {0};
  uint8_t soft_start_shifter = 4;

  union
  {
    int8_t command;
    uint8_t byte;
  } yaw_message;

  // Main loop
  for (;;) {  // Preferred over while(1)
    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      DMPReadFIFO();
      _status_MPU6050 = MPU6050_IDLE;

      // Calibrate();

      // Position control
      float roll_p_command = dmp_roll_angle() * 0.025
        * RADIANS_TO_MOTOR_SEGMENTS;
      float pitch_p_command = dmp_pitch_angle() * -0.025
        * RADIANS_TO_MOTOR_SEGMENTS;

      // Velocity control
      float roll_v_command = (float)dmp_gyro(0) * 0.8 * DMP_GYRO_TO_RADPS
        * RADIANS_TO_MOTOR_SEGMENTS * DMP_SAMPLE_TIME;
      float pitch_v_command = (float)dmp_gyro(1) * -0.8 * DMP_GYRO_TO_RADPS
        * RADIANS_TO_MOTOR_SEGMENTS * DMP_SAMPLE_TIME;

      // Acceleration control
      float roll_a_command = (float)(dmp_gyro(0) - dmp_gyro_pv[0]) * 0.03
        * DMP_GYRO_TO_RADPS * RADIANS_TO_MOTOR_SEGMENTS;
      float pitch_a_command = (float)(dmp_gyro(1) - dmp_gyro_pv[1]) * -0.03
        * DMP_GYRO_TO_RADPS * RADIANS_TO_MOTOR_SEGMENTS;

      // MotorMove(MOTOR_ROLL, (int8_t)(roll_p_command + pitch_p_command),
      //   soft_start_shifter);
      // MotorMove(MOTOR_PITCH, (int8_t)pitch_p_command, soft_start_shifter);

      MotorMove(MOTOR_ROLL, (int8_t)(roll_p_command + roll_v_command
        + roll_a_command + pitch_p_command + pitch_v_command + pitch_a_command),
        soft_start_shifter);
      MotorMove(MOTOR_PITCH, (int8_t)(pitch_p_command + pitch_v_command
        + pitch_a_command), soft_start_shifter);

      // Yaw control law (this could be moved to the yaw control unit)
      float yaw_p_command = dmp_yaw_angle() * P_GAIN
        * RADIANS_TO_MOTOR_SEGMENTS;

      yaw_message.command = -(int8_t)yaw_p_command;
      I2CTxBytes(YAW_CONTROLLER_ADDRESS, &yaw_message.byte, 1);

      // Save past values
      dmp_gyro_pv[0] = dmp_gyro(0);
      dmp_gyro_pv[1] = dmp_gyro(1);
      dmp_gyro_pv[2] = dmp_gyro(2);

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
        // PORTB ^= _BV(PORTB5);  // Green LED Heartbeat

        if (soft_start_shifter) --soft_start_shifter;

        if (BatteryIsLow())
          PORTD ^= _BV(PORTD2);  // Toggle buzzer
        else
          PORTD &= ~_BV(PORTD2);  // Turn off buzzer
      }
      BatteryMeasureVoltage();
    }
  }
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
