#include "timer0.h"

#include <avr/interrupt.h>
#include <avr/io.h>


// =============================================================================
// Private data:

static volatile bool _tick = FALSE;
static volatile uint8_t _timestamp = 0;


// =============================================================================
// Public functions:

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

// This interrupt occurs when Timer0 reaches BOTOM, which occurs at 31.25 kHz. A
// sub-counter is used to reduce the ticks to 31250 Hz / 250 = 125Hz (0.008 s).
ISR(TIMER0_OVF_vect)
{
  static uint8_t subcounter = 250;
  if (!--subcounter) {
    subcounter = 250;
    _tick = TRUE;
    ++_timestamp;
  }
}
