#include "timer0.h"

#include <avr/interrupt.h>
#include <avr/io.h>


// =============================================================================
// Private data:

static volatile bool _tick = FALSE;
static volatile uint8_t _timestamp = 0;


// =============================================================================
// Public functions:

// Initialize Timer0 to produce an interrupt at 125Hz
void Timer0Init(void)
{
  TCCR0A = _BV(WGM01);  // Set "Clear Timer on Compare" mode
  TCCR0B = _BV(CS02) | _BV(CS00);  // Set prescaler to 1/1024
  TCNT0 = 0;  // Clear the timer
  // Set compare to occur at 125 Hz
  OCR0A = (uint8_t)(F_CPU / 1024 / 125) - 1;
  TIMSK0 = _BV(OCIE0A);  // Enable "Output Compare Match A" interrupt
}

// -----------------------------------------------------------------------------
// Returns TRUE if an interrupt from Timer0 has occurred since the last time
// this function returned TRUE.
bool Timer0Tick(void)
{
  cli();
  bool ret = _tick;
  // Note that if interrupts weren't disabled, the timer interrupt might occur
  // here causing a missed tick. Disabling and re-enabling interrupts is more
  // efficient than branching.
  _tick = FALSE;
  sei();
  return ret;
}

// -----------------------------------------------------------------------------
// This function can delay execution from 0 to 2040 ms with a precision of 8 ms
// (rounded up). It is designed to work with a clock frequency of 125 Hz.
void Timer0Delay(uint16_t ms)
{
  uint8_t start_timestamp = _timestamp;
  uint8_t count = (uint8_t)((ms + 7) >> 3);
  while ((uint8_t)(_timestamp - start_timestamp) < count) continue;
}


// =============================================================================
// Private functions:

// This interrupt occurs when the counter matched the value in OCR0A, which
// should have been set in the initialization function.
ISR(TIMER0_COMPA_vect)
{
  _tick = TRUE;
  ++_timestamp;
}
