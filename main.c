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
void UARTPrintS16(int16_t value)
{
  uint8_t ascii[8] = { '0', '0', '0', '0', '0', '0', '\r', '\n' };
  PrintS16(value, ascii);
  UARTTxBytes(ascii, 8);
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  uint8_t soft_start_shifter = 4;
/*
  union
  {
    int8_t command;
    uint8_t byte;
  } yaw_message;
*/
  // Main loop
  for (;;) {  // Preferred over while(1)
    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      DMPReadFIFO();
      _status_MPU6050 = MPU6050_IDLE;

/*
      static enum DMPCalibrationMode dmp_calibrate_mode = DMP_CALIBRATE_START;
      if (dmp_calibrate_mode) {
        DMPCalibrate(&dmp_calibrate_mode);
      }
*/
/*
      // Position control
      // const float kP = 0.025;
      const float kP = 0.015;
      float roll_p_command = dmp_roll_angle() * -kP
        * RADIANS_TO_MOTOR_SEGMENTS;
      float pitch_p_command = dmp_pitch_angle() * kP
        * RADIANS_TO_MOTOR_SEGMENTS;

      // Velocity control
      const float kV = 0.3;
      float roll_v_command = (float)dmp_gyro(0) * -kV * DMP_GYRO_TO_RADPS
        * RADIANS_TO_MOTOR_SEGMENTS * DMP_SAMPLE_TIME;
      float pitch_v_command = (float)dmp_gyro(1) * kV * DMP_GYRO_TO_RADPS
        * RADIANS_TO_MOTOR_SEGMENTS * DMP_SAMPLE_TIME;

      // Acceleration control
      // const float kA = 0.025;
      const float kA = 0.00;
      static int16_t dmp_gyro_pv[3] = {0};
      float roll_a_command = (float)(dmp_gyro(0) - dmp_gyro_pv[0]) * -kA
        * DMP_GYRO_TO_RADPS * RADIANS_TO_MOTOR_SEGMENTS;
      float pitch_a_command = (float)(dmp_gyro(1) - dmp_gyro_pv[1]) * kA
        * DMP_GYRO_TO_RADPS * RADIANS_TO_MOTOR_SEGMENTS;

      // Save past values
      dmp_gyro_pv[0] = dmp_gyro(0);
      dmp_gyro_pv[1] = dmp_gyro(1);

      MotorMove(MOTOR_A, roll_p_command + roll_v_command + roll_a_command,
        soft_start_shifter);
      MotorMove(MOTOR_B, pitch_p_command + pitch_v_command + pitch_a_command
        - roll_p_command - roll_v_command - roll_a_command, soft_start_shifter);

      // Yaw control law
      float yaw_p_command = dmp_yaw_angle() * 0.025
        * YAW_RADIANS_TO_MOTOR_SEGMENTS;
      float yaw_v_command = (float)dmp_gyro(2) * -0.6 * DMP_GYRO_TO_RADPS
        * YAW_RADIANS_TO_MOTOR_SEGMENTS * DMP_SAMPLE_TIME;
      float yaw_a_command = (float)(dmp_gyro(2) - dmp_gyro_pv[2]) * -0.02
        * DMP_GYRO_TO_RADPS * YAW_RADIANS_TO_MOTOR_SEGMENTS;

      // Save past values
      dmp_gyro_pv[2] = dmp_gyro(2);

      yaw_message.command = -yaw_p_command + yaw_v_command + yaw_a_command;
      I2CTxBytes(YAW_CONTROLLER_ADDRESS, &yaw_message.byte, 1);
*/

      float p = (float)dmp_gyro(0) * DMP_GYRO_TO_RADPS;
      float q = (float)dmp_gyro(1) * DMP_GYRO_TO_RADPS;

      float phi = dmp_roll_angle();
      float tht = dmp_pitch_angle();

      static float phi_int = 0.0, tht_int = 0.0;
      phi_int += phi * DMP_SAMPLE_TIME;
      tht_int += tht * DMP_SAMPLE_TIME;

      float motor_a = -0.037588 * p
        + 0.030793 * q
        + -0.56192 * phi
        + 0.48222 * tht
        + -9.9607 * phi_int
        + 4.559 * tht_int;
      float motor_b = 0.0048041 * p
        + 0.040965 * q
        + -0.049835 * phi
        + 0.56766 * tht
        + 4.559 * phi_int
        + 9.9607 * tht_int;

/*
      static float motor_a = 0., motor_b = 0.;
      motor_a += 0 / RADIANS_TO_MOTOR_SEGMENTS;
      motor_b += 1 / RADIANS_TO_MOTOR_SEGMENTS;
*/

      MotorAngle(MOTOR_A, motor_a, soft_start_shifter);
      MotorAngle(MOTOR_B, motor_b, soft_start_shifter);

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

        // UARTPrintf("%f", (float)dmp_gyro(1) * DMP_GYRO_TO_RADPS);

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
