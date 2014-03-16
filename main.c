#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2c.h"
#include "mpu6050.h"
#include "uart.h"
#include "timer0.h"


// ============================================================================+
// Private data:

static volatile uint8_t _count_idle = 0, _flag_5hz = 0, _flag_125hz = 0;
static volatile uint8_t _status_MPU6050 = MPU6050_DATA_WAITING;
volatile union {
  struct str_MPU6050Data s;
  uint8_t bytes[sizeof(struct str_MPU6050Data)];
} _mpu6050_data;



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
  // InitTimer0(TIMER0_FREQUENCY);

  sei();  // Enable interrupts

  InitMPU6050(MPU6050_DEFAULT_ADDRESS);
}

// -----------------------------------------------------------------------------
void swap_volatile_bytes(volatile uint8_t *byte_array)
{
  uint8_t temp = byte_array[0];
  byte_array[0] = byte_array[1];
  byte_array[1] = temp;
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  // Main loop
  for (;;) {  // Preferred over while(1)
    if (_status_MPU6050 != MPU6050_IDLE && I2CIsIdle()) {
      if (_status_MPU6050 == MPU6050_READING_DATA) {
        _status_MPU6050 = MPU6050_IDLE;
        swap_volatile_bytes(&_mpu6050_data.bytes[0]);
        swap_volatile_bytes(&_mpu6050_data.bytes[2]);
        swap_volatile_bytes(&_mpu6050_data.bytes[4]);
        swap_volatile_bytes(&_mpu6050_data.bytes[8]);
        swap_volatile_bytes(&_mpu6050_data.bytes[10]);
        swap_volatile_bytes(&_mpu6050_data.bytes[12]);

        static union {
          int32_t int32;
          uint8_t bytes[4];
        } HPIntegral;
        HPIntegral.int32 += _mpu6050_data.s.x_gyro;
        if (UCSR0A & _BV(UDRE0))  // Transfer buffer is clear
          UDR0 = HPIntegral.bytes[2] + 128;
      } else if (_status_MPU6050 == MPU6050_DATA_WAITING ||
          _count_idle > IDLE_LIMIT) {
        ReadMPU6050(MPU6050_DEFAULT_ADDRESS, &_mpu6050_data.bytes[0]);
        _status_MPU6050 = MPU6050_READING_DATA;
        _count_idle = 0;
      }
    }
    if (TickTimer0())
      ++_count_idle;
  }
}

// -----------------------------------------------------------------------------
ISR(INT0_vect)
{
  PORTB ^= _BV(PORTB5);  // Red LED Heartbeat

  if (I2CIsIdle()) {
    ReadMPU6050(MPU6050_DEFAULT_ADDRESS, &_mpu6050_data.bytes[0]);
    _status_MPU6050 = MPU6050_READING_DATA;
    _count_idle = 0;
  } else {
    _status_MPU6050 = MPU6050_DATA_WAITING;
  }
}
