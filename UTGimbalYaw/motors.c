#include "motors.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

// =============================================================================
// Private data:

const uint8_t sin_table[SINE_TABLE_LENGTH/4] PROGMEM = {
128, 129, 130, 131, 132, 132, 133, 134, 135, 136, 137, 138, 139, 139, 140, 141,
142, 143, 144, 145, 146, 147, 147, 148, 149, 150, 151, 152, 153, 154, 154, 155,
156, 157, 158, 159, 160, 160, 161, 162, 163, 164, 165, 166, 166, 167, 168, 169,
170, 171, 172, 172, 173, 174, 175, 176, 176, 177, 178, 179, 180, 181, 181, 182,
183, 184, 185, 185, 186, 187, 188, 189, 189, 190, 191, 192, 192, 193, 194, 195,
195, 196, 197, 198, 198, 199, 200, 201, 201, 202, 203, 204, 204, 205, 206, 206,
207, 208, 208, 209, 210, 210, 211, 212, 212, 213, 214, 214, 215, 216, 216, 217,
218, 218, 219, 220, 220, 221, 221, 222, 223, 223, 224, 224, 225, 225, 226, 227,
227, 228, 228, 229, 229, 230, 230, 231, 231, 232, 232, 233, 233, 234, 234, 235,
235, 236, 236, 237, 237, 238, 238, 239, 239, 239, 240, 240, 241, 241, 242, 242,
242, 243, 243, 243, 244, 244, 245, 245, 245, 246, 246, 246, 247, 247, 247, 247,
248, 248, 248, 249, 249, 249, 249, 250, 250, 250, 250, 251, 251, 251, 251, 252,
252, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 254, 254, 254, 254, 254,
254, 254, 254, 254, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
255
};


// =============================================================================
// Public functions:

void MotorPWMTimerInit(void) {
  // Clear the output compare registers
  OCR1A = 0x00;  // OC1A is pin B0
  OCR1B = 0x00;  // OC1B is pin B2
  OCR1D = 0x00;  // OC1D is pin B4

  OCR1C = 0xFF;  // Set TOP to 255

  // Set the PWM pins to output.
  DDRB |= _BV(DDB4) | _BV(DDB2) | _BV(DDB0);

  // Set timer1 to 16MHz / 256 / 2 = 31.25 kHz (ultrasound) phase-correct
  // PWM with output pins cleared on up-count match and set on down-count.
  TCCR1A = _BV(COM1A0) | _BV(COM1B0) | _BV(PWM1A) | _BV(PWM1B);
  TCCR1B = _BV(PWM1X) | _BV(CS10);
  TCCR1C = _BV(COM1A0S) | _BV(COM1B0S) | _BV(COM1D0) | _BV(PWM1D);
  TCCR1D = _BV(WGM10);
}

// -----------------------------------------------------------------------------
void MotorMoveDeltaSegments(int8_t delta_segments, uint8_t shift)
{
  static int16_t magnetic_field_direction = 0;
  magnetic_field_direction += delta_segments;
  if (magnetic_field_direction > SINE_TABLE_LENGTH)
    magnetic_field_direction -= SINE_TABLE_LENGTH;

  int16_t segment = magnetic_field_direction;
  uint8_t stator_pwm[3] = {0}, i = 2;
  for (;;)
  {
    if (segment >= SINE_TABLE_LENGTH * 3 / 4)
      stator_pwm[i]
        = 255 - pgm_read_byte(&(sin_table[SINE_TABLE_LENGTH - segment - 1]));
    else if (segment >= SINE_TABLE_LENGTH * 2 / 4)
      stator_pwm[i]
        = 255 - pgm_read_byte(&(sin_table[segment - SINE_TABLE_LENGTH / 2]));
    else if (segment >= SINE_TABLE_LENGTH * 1 / 4)
      stator_pwm[i]
        = pgm_read_byte(&(sin_table[SINE_TABLE_LENGTH / 2 - segment - 1]));
    else // (segment >= SINE_TABLE_LENGTH * 0 / 4)
      stator_pwm[i] = pgm_read_byte(&(sin_table[segment]));

    if (!i--)
      break;  // Quit this loop after i == 0

    segment += SINE_TABLE_LENGTH * 1 / 3;
    if (segment >= SINE_TABLE_LENGTH) segment -= SINE_TABLE_LENGTH;
  }

  OCR1A = stator_pwm[0] >> shift;  // OC1A is pin B0
  OCR1B = stator_pwm[1] >> shift;  // OC1B is pin B2
  OCR1D = stator_pwm[2] >> shift;  // OC1D is pin B4
}
