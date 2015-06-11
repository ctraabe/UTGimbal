#include "timer0.h"

#include <avr/io.h>
#include <util/atomic.h>


// ============================================================================+
// Private data:

volatile uint16_t ms_timestamp_ = 0;

enum Timer0ClockSource
{
  CS0_NONE   = 0<<CS02 | 0<<CS01 | 0<<CS00,  // No clock source.
  CS0_PS0    = 0<<CS02 | 0<<CS01 | 1<<CS00,  // clk I/O / 1.
  CS0_PS8    = 0<<CS02 | 1<<CS01 | 0<<CS00,  // clk I/O / 8.
  CS0_PS64   = 0<<CS02 | 1<<CS01 | 1<<CS00,  // clk I/O / 64.
  CS0_PS256  = 1<<CS02 | 0<<CS01 | 0<<CS00,  // clk I/O / 256.
  CS0_PS1024 = 1<<CS02 | 0<<CS01 | 1<<CS00,  // clk I/O / 1024.
  CS0_EXTF   = 1<<CS02 | 1<<CS01 | 0<<CS00,  // External clock on T0 falling.
  CS0_EXTR   = 1<<CS02 | 1<<CS01 | 1<<CS00,  // External clock on T0 rising.
};


// ============================================================================+
// Public functions:

// This function initializes TIMER0. This timer triggers the interrupt "TIMER0
// COMPA" at 1kHz.
void Timer0Init(void)
{
  // Clear TIMER0 registers.
  TCCR0A = 0;
  TCCR0B = 0;
  TIMSK &= ~_BV(OCIE0A) & ~_BV(OCIE0B) & ~_BV(TOIE0) & ~_BV(TICIE0);
  // Waveform generation mode bits:
  TCCR0A |= (0 << ICEN0);
  TCCR0A |= (0 << TCW0);
  TCCR0A |= (1 << WGM00);
  // Clock select bits:
  TCCR0B |= CS0_PS64;
  // Overflow interrupt enable bit:
  TIMSK |= (0 << TOIE0);
  // Output compare match A
  TIMSK |= (1 << OCIE0A);  // Output compare match interrupt enable.
  OCR0A = 250;  // Output compare register.
  // Output compare match B
  TIMSK |= (0 << OCIE0B);  // Output compare match interrupt enable.
  OCR0B = 0;  // Output compare register.
  // Clear the timer.
  TCNT0L = 0;
  TCNT0H = 0;
}

// -----------------------------------------------------------------------------
// This function returns the current timestamp.
int16_t GetTimestamp(void)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  return ms_timestamp;
}

// -----------------------------------------------------------------------------
// This function returns a timestamp corresponding to "t" ms in the future. This
// timestamp can be checked against the current timestamp to see if a certain
// amount of time has passed. This function works for durations up to 32767 ms.
int16_t GetTimestampMillisFromNow(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  return ms_timestamp + t + 1;
}

// -----------------------------------------------------------------------------
// This function compares a timestamp to the current timestamp and returns TRUE
// if the timestamp is in the past. This function works for durations up to
// 32767 ms.
uint8_t TimestampInPast(int16_t t)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  return (t - ms_timestamp) < 0;
}

// -----------------------------------------------------------------------------
// This function returns the amount of time that has elapsed since the timestamp
// "last_time" has occurred. This function works for time periods up to 65535
// ms. The function also automatically updates last_time so that it can be
// easily be called periodically.
uint16_t MillisSinceTimestamp(int16_t *last_time)
{
  int16_t ms_timestamp;
  ATOMIC_BLOCK(ATOMIC_FORCEON) { ms_timestamp = ms_timestamp_; }
  uint16_t ret = (uint16_t)(ms_timestamp - *last_time);
  *last_time = ms_timestamp;
  return ret;
}

// -----------------------------------------------------------------------------
// This function delays execution of the program for "t" ms. Functions triggered
// by interrupts will still execute during this period. This function works for
// time periods up to 32767 ms.
void Wait(uint16_t w)
{
  int16_t timestamp = GetTimestampMillisFromNow(w);
  while (!TimestampInPast(timestamp));
}
