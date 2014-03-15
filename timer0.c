#include "timer0.h"

#include <avr/interrupt.h>
#include <avr/io.h>


// ============================================================================+
// Private data:

static volatile bool _tick = FALSE;


// ============================================================================+
// Public functions:

void InitTimer0(uint8_t frequency)
{
  TCCR0A = _BV(WGM01);  // Set "Clear Timer on Compare" mode
  TCCR0B = _BV(CS02) | _BV(CS00);  // Set prescaler to 1/1024
  TCNT0 = 0;  // Clear the timer
  // Set compare to occur at "frequency" Hz
  OCR0A = (uint8_t)(F_CPU / 1024 / frequency) - 1;
  TIMSK0 = _BV(OCIE0A);  // Enable "Output Compare Match A" interrupt
}

// -----------------------------------------------------------------------------
bool TickTimer0(void)
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


// ============================================================================+
// Private functions:

// This interrupt occurs when the counter matched the value in OCR0A, which
// should have been set in the initialization function.
ISR(TIMER0_COMPA_vect)
{
  _tick = TRUE;
}
