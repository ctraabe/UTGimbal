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
static int16_t magnetic_field_direction[NUMBER_OF_MOTORS] = {0};
static int8_t magnetic_field_rotations[NUMBER_OF_MOTORS] = {0};


// =============================================================================
// Private function declarations:

static void MoveToPosition(enum Motors motor);


// =============================================================================
// Public functions:

void MotorPWMTimersInit(void) {
  // Clear the output compare registers
  OCR0A = 0;  // OC0A is pin D6
  OCR0B = 0;  // OC0B is pin D5
  OCR1A = 0;  // OC1A is pin B1
  OCR1B = 0;  // OC1B is pin B2
  OCR2A = 0;  // OC2A is pin B3
  OCR2B = 0;  // OC2B is pin D3

  // Set the PWM pins for the roll driver to output
  DDRB |= _BV(DDB3) | _BV(DDB2) | _BV(DDB1);

  // Set the PWM pins for the pitch driver to output
  DDRD |= _BV(DDD6) | _BV(DDD5) | _BV(DDD3);

  // Set each timer to 16Mhz / 256 / 2 = 31.25 kHz (ultrasound) phase-correct
  // PWM with output pins cleared on up-count match and set on down-count.
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
  TCCR0B = _BV(CS00);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM10);
  TCCR1B = _BV(CS10);
  TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
}

void MotorMove(enum Motors motor, int8_t segments)
{
  magnetic_field_direction[motor] += segments;
  if (magnetic_field_direction[motor] < 0)
  {
    magnetic_field_direction[motor] += SINE_TABLE_LENGTH;
    --magnetic_field_rotations[motor];
  }
  else if (magnetic_field_direction[motor] >= SINE_TABLE_LENGTH)
  {
    magnetic_field_direction[motor] -= SINE_TABLE_LENGTH;
    ++magnetic_field_rotations[motor];
  }

  MoveToPosition(motor);
}


// =============================================================================
// Private functions:

static void MoveToPosition(enum Motors motor) {
  int16_t index = magnetic_field_direction[motor];
  uint8_t stator_pwm[3] = {0};

  uint8_t i = 2;
  for (;;)
  {
    if (index >= SINE_TABLE_LENGTH * 3 / 4)
      stator_pwm[i]
        = 255 - pgm_read_byte(&(sin_table[SINE_TABLE_LENGTH - index - 1]));
    else if (index >= SINE_TABLE_LENGTH * 2 / 4)
      stator_pwm[i]
        = 255 - pgm_read_byte(&(sin_table[index - SINE_TABLE_LENGTH / 2]));
    else if (index >= SINE_TABLE_LENGTH * 1 / 4)
      stator_pwm[i]
        = pgm_read_byte(&(sin_table[SINE_TABLE_LENGTH / 2 - index - 1]));
    else // (index >= SINE_TABLE_LENGTH * 0 / 4)
      stator_pwm[i] = pgm_read_byte(&(sin_table[index]));

    if (!i--)
      break;  // Quit this loop after i == 0

    index += SINE_TABLE_LENGTH * 1 / 3;
    if (index >= SINE_TABLE_LENGTH) index -= SINE_TABLE_LENGTH;
  }

  if (motor == MOTOR_ROLL)
  {
    OCR1A = stator_pwm[0];
    OCR1B = stator_pwm[1];
    OCR2A = stator_pwm[2];
  }
  else
  {
    OCR2B = stator_pwm[0];
    OCR0B = stator_pwm[1];
    OCR0A = stator_pwm[2];
  }
}
