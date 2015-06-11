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

  // Startup ramp
  while (MotorsStartup()) Wait(5);

  // Main loop
  uint16_t timer = GetTimestamp();
  for (;;)
  {
    while (!I2CDataInBuffer()) continue;

    union {
      int8_t command;
      uint8_t byte;
    } yaw_message;
    yaw_message.byte = I2CPeek();

    MotorMoveDeltaSegments(yaw_message.command);

    if (TimestampInPast(timer))
    {
      timer += 500;
      PORTA ^= _BV(PORTA4);  // Green LED heartbeat
    }
  }
}
