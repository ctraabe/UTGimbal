#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2c.h"
#include "mag3110.h"
#include "mpu6050.h"
#include "uart.h"
#include "timer0.h"


// ============================================================================+
// Private data:

static volatile uint8_t _count_idle = 0, _flag_5hz = 0, _flag_125hz = 0;
static volatile uint8_t _status_MPU6050 = MPU6050_DATA_WAITING;
static volatile uint8_t _status_MAG3110 = MAG3110_DATA_WAITING;
volatile union {
  struct str_MPU6050Data s;
  uint8_t bytes[sizeof(struct str_MPU6050Data)];
} _mpu6050_data;
volatile union {
  struct str_MAG3110Data s;
  uint8_t bytes[sizeof(struct str_MAG3110Data)];
} _mag3110_data;


// ============================================================================+
// Private functions:

void Initialization(void)
{
  DDRB |= _BV(DDB5);  // Set pin PB4 to output (attached to red LED)
  DDRD &= ~_BV(DDD3) & ~_BV(DDD2);  // Set pins D3 (int1) and D2 (int0) to input
  // Enable the interrupt on pins D3 (int1) and D2 (int0)
  EIMSK |= _BV(INT1) | _BV(INT0);
  // Set the interrupts int1 and int0 to trigger on the rising edge
  EICRA |= _BV(ISC11) | _BV(ISC10) | _BV(ISC01) | _BV(ISC00);

  InitI2C(I2C_SPEED);
  InitUART(UART_BAUD);
  InitTimer0(TIMER0_FREQUENCY);

  sei();  // Enable interrupts

  // InitMPU6050(MPU6050_DEFAULT_ADDRESS);
  InitMAG3110();
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
    if (_status_MAG3110 != MAG3110_IDLE && I2CIsIdle()) {
      if (_status_MAG3110 == MAG3110_READING_DATA) {
        _status_MAG3110 = MAG3110_IDLE;

        // Atmel is apparently uses big endian
        swap_volatile_bytes(&_mag3110_data.bytes[0]);
        swap_volatile_bytes(&_mag3110_data.bytes[2]);
        swap_volatile_bytes(&_mag3110_data.bytes[4]);

        if (UCSR0A & _BV(UDRE0))  // Transfer buffer is clear
          UDR0 = (uint8_t)((_mag3110_data.s.x_magnetometer >> 2) + 128);
      } else if (_status_MAG3110 == MAG3110_DATA_WAITING) {
        // Prevent interrupts from attempting to start i2c during this call
        cli();
        ReadMAG3110(&_mag3110_data.bytes[0]);
        sei();
        _status_MAG3110 = MAG3110_READING_DATA;
      }
    }

    if (_status_MAG3110 != MAG3110_IDLE && I2CIsIdle()) {
      if (_status_MPU6050 == MPU6050_READING_DATA) {
        _status_MPU6050 = MPU6050_IDLE;

        // Atmel is apparently uses big endian
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
        // if (UCSR0A & _BV(UDRE0))  // Transfer buffer is clear
        //   UDR0 = HPIntegral.bytes[2] + 128;
      } else if (_status_MPU6050 == MPU6050_DATA_WAITING ||
          _count_idle > IDLE_LIMIT) {
        // Prevent interrupts from attempting to start i2c during this call
        cli();
        ReadMPU6050(MPU6050_DEFAULT_ADDRESS, &_mpu6050_data.bytes[0]);
        sei();
        _status_MPU6050 = MPU6050_READING_DATA;
        _count_idle = 0;
      }
    }

    if (TickTimer0()) {
      ++_count_idle;
    }
  }
}

// -----------------------------------------------------------------------------
// External interrupt at pin int0, activates on rising edge. Connected to the
// interrupt pin on the MPU6050, indicating that data is ready.
ISR(INT0_vect)
{
  PORTB ^= _BV(PORTB5);  // Red LED Heartbeat

  if (I2CIsIdle()) {
    // Note: this interrupt (int0) has highest priority, so it is not necessary
    // to protect this call from other interrupts.
    ReadMPU6050(MPU6050_DEFAULT_ADDRESS, &_mpu6050_data.bytes[0]);
    _status_MPU6050 = MPU6050_READING_DATA;
    _count_idle = 0;
  } else {
    _status_MPU6050 = MPU6050_DATA_WAITING;
  }
}

// -----------------------------------------------------------------------------
// External interrupt at pin int1, activates on rising edge. Connected to the
// interrupt pin on the MAG3110, indicating that data is ready.
ISR(INT1_vect)
{
  PORTB ^= _BV(PORTB5);  // Red LED Heartbeat

  if (I2CIsIdle()) {
    // Prevent other interrupts from attempting to start i2c during this call
    cli();
    ReadMAG3110(&_mag3110_data.bytes[0]);
    sei();
    _status_MAG3110 = MAG3110_READING_DATA;
    _count_idle = 0;
  } else {
    _status_MAG3110 = MAG3110_DATA_WAITING;
  }
}
