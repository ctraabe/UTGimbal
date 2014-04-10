#include "main.h"

#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#include "dmp.h"
#include "i2c.h"
#include "mag3110.h"
#include "mpu6050.h"
#include "print.h"
#include "uart.h"
#include "timer0.h"


// =============================================================================
// Private data:

static volatile uint8_t _count_idle = 0, _flag_5hz = 0, _flag_125hz = 0;
static volatile enum MPU6050Mode _status_MPU6050 = MPU6050_DATA_WAITING;
static volatile enum MAG3110Mode _status_MAG3110 = MAG3110_DATA_WAITING;


// =============================================================================
// Private functions:

static void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin PB4 to output (attached to red LED)
  DDRD &= ~_BV(DDD3) & ~_BV(DDD2);  // Set pins D3 (int1) and D2 (int0) to input
  // Enable the interrupt on pins D3 (int1) and D2 (int0)
  EIMSK |= _BV(INT1) | _BV(INT0);
  // Set the interrupts int1 and int0 to trigger on the rising edge
  EICRA |= _BV(ISC11) | _BV(ISC10) | _BV(ISC01) | _BV(ISC00);

  I2CInit(I2C_SPEED);
  UARTInit(UART_BAUD);
  Timer0Init();

  sei();  // Enable interrupts

  // MPU6050Init();
  // MAG3110Init();
  DMPInit();
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  // Main loop
  for (;;) {  // Preferred over while(1)
    if (_status_MPU6050 == MPU6050_DATA_WAITING) {
      // enum MPU6050Error error = DMPReadFIFO();
      DMPReadFIFO();

      static uint8_t uart_tx_buffer[80];
      uint8_t i = 0;
/*
      i += PrintS32(dmp_quaternion(0), uart_tx_buffer + i);
      i += PrintSpace(uart_tx_buffer + i);
      i += PrintS32(dmp_quaternion(1), uart_tx_buffer + i);
      i += PrintSpace(uart_tx_buffer + i);
      i += PrintS32(dmp_quaternion(2), uart_tx_buffer + i);
      i += PrintSpace(uart_tx_buffer + i);
      i += PrintS32(dmp_quaternion(3), uart_tx_buffer + i);
      i += PrintEOL(uart_tx_buffer + i);

      i += PrintU8(error, uart_tx_buffer + i);
      i += PrintEOL(uart_tx_buffer + i);
*/
      float temp = sqrt(
        dmp_quaternion(0) * dmp_quaternion(0) +
        dmp_quaternion(1) * dmp_quaternion(1) +
        dmp_quaternion(2) * dmp_quaternion(2) +
        dmp_quaternion(3) * dmp_quaternion(3));
      i += PrintS32((uint32_t)temp, uart_tx_buffer + i);
      i += PrintEOL(uart_tx_buffer + i);
      UARTTxBytes(uart_tx_buffer, i);

      _status_MPU6050 = MPU6050_IDLE;
    }
  }
}

// -----------------------------------------------------------------------------
// External interrupt at pin int0, activates on rising edge. Connected to the
// interrupt pin on the MPU6050, indicating that data is ready.
ISR(INT0_vect)
{
  PORTB ^= _BV(PORTB5);  // Red LED Heartbeat
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
