#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "battery.h"
#include "endian.h"
#include "i2c.h"
#include "motors.h"
#include "mpu6050_raw.h"
#include "print.h"
#include "timer0.h"
#include "uart.h"
#include "utilities.h"


// =============================================================================
// Private data:

static volatile uint8_t _count_idle = 0, _flag_5hz = 0, _flag_125hz = 0;
static volatile enum MPU6050Mode _status_MPU6050 = MPU6050_IDLE;
// static volatile enum MAG3110Mode _status_MAG3110 = MAG3110_DATA_WAITING;


// =============================================================================
// Private functions:

static void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin B5 to output (attached to green LED).
  DDRD |= _BV(DDD2);  // Set pin D2 to output (attached to buzzer).

  // Enable pin change interrupts for pins PCINT14..8 and mask all but PCINT11.
  PCICR = _BV(PCIE1);
  PCMSK1 = _BV(PCINT11);

  I2CInit(I2C_SPEED);
  UARTInit(UART_BAUD);
  MotorPWMTimersInit();
  Timer0Init();  // Depends on Timer0 settings from MotorPWMTimersInit().

  sei();  // Enable interrupts
/*
  Timer0Delay(100);
  BatteryMeasureVoltage();
  Timer0Delay(100);
  BatteryMeasurementInit();
*/
  MPU6050RawInit();
  // MPU6050DMPInit();  // Note, sets int1 to input (conflict with buzzer)
  // MAG3110Init();  // Note, sets int1 to input (conflict with buzzer)
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  uint8_t message[5] = { '0', '0', '0', '\r', '\n' };
  volatile union {
    struct str_MPU6050RawData data;
    uint8_t bytes[sizeof(struct str_MPU6050RawData)];
  } mpu6050_raw;

  // Main loop
  for (;;) {  // Preferred over while(1)

    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      MPU6050RawRead(mpu6050_raw.bytes);
/*
      // Pitch control law
      float pitch_p_command = dmp_pitch_angle() * P_GAIN
        * RADIANS_TO_MOTOR_SEGMENTS;

      // Roll control law
      float roll_p_command = dmp_roll_angle() * P_GAIN
        * RADIANS_TO_MOTOR_SEGMENTS;

      MotorMove(MOTOR_ROLL, (int8_t)roll_p_command + (int8_t)pitch_p_command);
      MotorMove(MOTOR_PITCH, -(int8_t)pitch_p_command);
*/
      _status_MPU6050 = MPU6050_IDLE;
    }

    if (Timer0Tick()) {  // 125 Hz
      message[0] = '0';
      message[1] = '0';
      message[2] = '0';
      while (mpu6050_raw.bytes[0] > 99) {
        mpu6050_raw.bytes[0] -= 100;
        ++message[0];
      }
      while (mpu6050_raw.bytes[0] > 9) {
        mpu6050_raw.bytes[0] -= 10;
        ++message[1];
      }
      message[2] += mpu6050_raw.bytes[0];
      UARTTxBytes(message, 5);
      // if (I2CInactivtyCounter() > 1)
      //   I2CReset();
      static uint8_t seconds_counter = 1;
      if (!--seconds_counter) {
        seconds_counter = 125;
        PORTB ^= _BV(PORTB5);  // Green LED Heartbeat
/*
        if (BatteryIsLow())
          PORTB ^= _BV(PORTB4);  // Toggle red LED
        else
          PORTB &= ~_BV(PORTB4);  // Turn off red LED
*/
      }
      // BatteryMeasureVoltage();
    }
  }
}

ISR(PCINT1_vect)
{
  // Only do something on the high state.
  if (PINC & (_BV(PINC3))) {
    _status_MPU6050 = MPU6050_DATA_WAITING;
  }
}

// -----------------------------------------------------------------------------
// External interrupt at pin int0, activates on rising edge. Connected to the
// interrupt pin on the MPU6050, indicating that data is ready.
ISR(INT0_vect)
{
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
