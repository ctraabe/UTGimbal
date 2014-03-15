#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"
#include "timer0.h"


// ============================================================================+
// Private data:

static volatile uint8_t _flag_5hz = 0, _flag_125hz = 0;
static volatile uint8_t _status_MP6050 = MPU6050_NEW_DATA, _error_cnt = 0;
static uint8_t _rx_buffer[10] = {0};


// ============================================================================+
// Private functions:

void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin PB4 to output (attached to red LED)
  DDRD &= ~_BV(DDD2);  // Set pin D2 (int0) to input
  EIMSK |= _BV(INT0);  // Enable the interrupt on pin D2 (int0)
  EICRA |= _BV(ISC01) | _BV(ISC00);  // Set the interrupt to rising edge

  InitI2C(I2C_SPEED);
  InitUART(UART_BAUD);
  InitTimer0(TIMER0_FREQUENCY);

  sei();  // Enable interrupts

  InitMPU6050(MPU6050_DEFAULT_ADDRESS);
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  // Main loop
  for (;;) {  // Preferred over do ... while
    if (I2CIsIdle()) {
      if (_status_MP6050 == MPU6050_NEW_DATA) {
        ReadMPU6050(MPU6050_DEFAULT_ADDRESS, &_rx_buffer[0]);
        _status_MP6050 = MPU6050_READING_NEW_DATA;
      } else if (_status_MP6050 == MPU6050_READING_NEW_DATA) {
        if (UCSR0A & _BV(UDRE0))  // Transfer buffer is clear
          UDR0 = _rx_buffer[0] + 128;
        _status_MP6050 = MPU6050_IDLE;
      }
    }
  }
}

// -----------------------------------------------------------------------------
ISR(INT0_vect)
{
  PORTB ^= _BV(PORTB5);  // Red LED Heartbeat

  if (_status_MP6050 == MPU6050_IDLE)
    _status_MP6050 = MPU6050_NEW_DATA;
}
