#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

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
  DDRB |= _BV(DDB4);  // Set pin B4 to output (attached to red LED).
  DDRC |= _BV(DDC1);  // Set pin C1 to output (attached to green LED).

  I2CInit(I2C_SPEED);
  UARTInit(UART_BAUD);
  MotorPWMTimersInit();
  Timer0Init();  // Depends on Timer0 settings from MotorPWMTimersInit().

  sei();  // Enable interrupts

  MPU6050DMPInit();
  // MAG3110Init();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  // Main loop
  for (;;) {  // Preferred over while(1)
    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      DMPReadFIFO();
      PORTC ^= _BV(PORTC1);  // Green LED Heartbeat

      // Pitch control law
      float pitch_p_command = dmp_pitch_angle() * P_GAIN
        * RADIANS_TO_MOTOR_SEGMENTS;

      // Roll control law
      float roll_p_command = dmp_roll_angle() * P_GAIN
        * RADIANS_TO_MOTOR_SEGMENTS;

      MotorMove(MOTOR_ROLL, (int8_t)roll_p_command + (int8_t)pitch_p_command);
      MotorMove(MOTOR_PITCH, -(int8_t)pitch_p_command);

      _status_MPU6050 = MPU6050_IDLE;
    }
  }
}

// -----------------------------------------------------------------------------
// External interrupt at pin int0, activates on rising edge. Connected to the
// interrupt pin on the MPU6050, indicating that data is ready.
ISR(INT0_vect)
{
  PORTB ^= _BV(PORTB4);  // Red LED Heartbeat
  _status_MPU6050 = MPU6050_DATA_WAITING;
}

// -----------------------------------------------------------------------------
// External interrupt at pin int1, activates on rising edge. Connected to the
// interrupt pin on the MAG3110, indicating that data is ready.
ISR(INT1_vect)
{
/*
  if (I2CIsIdle()) {
    // Prevent other interrupts from attempting to start i2c during this call
    cli();
    MAG3110Read(&_mag3110_data.bytes[0]);
    sei();
    _status_MAG3110 = MAG3110_READING_DATA;
    _count_idle = 0;
  } else {
    _status_MAG3110 = MAG3110_DATA_WAITING;
  }
*/
}
