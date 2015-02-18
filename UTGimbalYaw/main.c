#include "main.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "i2c.h"
#include "motors.h"
#include "timer0.h"


// =============================================================================
// Private data:


// =============================================================================
// Private functions:

static void Initialization(void)
{
  DDRA |= _BV(DDA4);  // Set pin A4 to output (green LED).

  I2CInit();
  MotorPWMTimerInit();
  Timer0Init();

  sei();  // Enable interrupts
}

// -----------------------------------------------------------------------------
int16_t main(void)
{
  Initialization();

  // Main loop
  uint16_t stopwatch = GetDelay(0);
  PORTA |= _BV(PORTA4);
  for (;;)
  {
    while (!I2CDataIncoming()) continue;
    while (!I2CDataInBuffer()) continue;
    MotorMove(I2CPeek());

    if (CheckDelay(stopwatch))
    {
      stopwatch += 1000;
      PORTA ^= _BV(PORTA4);
    }
  }
}
