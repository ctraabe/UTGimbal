#include "motors.h"

#include <avr/io.h>
#include <avr/pgmspace.h>

const uint8_t sin_table[180] PROGMEM = {
127,131,136,140,145,149,153,158,162,166,170,175,179,183,187,191,194,198,202,205,
209,212,215,218,221,224,227,230,232,235,237,239,241,243,245,246,248,249,250,251,
252,253,253,254,254,254,254,254,253,253,252,251,250,249,248,246,245,243,241,239,
237,235,232,230,227,224,221,218,215,212,209,205,202,198,194,191,187,183,179,175,
170,166,162,158,153,149,145,140,136,131,127,123,118,114,109,105,101,96,92,88,84,
79,75,71,67,63,60,56,52,49,45,42,39,36,33,30,27,24,22,19,17,15,13,11,9,8,6,5,4,
3,2,1,1,0,0,0,0,0,1,1,2,3,4,5,6,8,9,11,13,15,17,19,22,24,27,30,33,36,39,42,45,
49,52,56,60,63,67,71,75,79,84,88,92,96,101,105,109,114,118,123
};

void InitMotors(void) {
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

  // Enable the interrupt (TIMER0_OVF) when Timer0 reaches BOTTOM (31.25 Hz).
  TIMSK0 = _BV(TOIE0);
}

void MoveMotorTo(uint8_t position) {
  OCR1A = pgm_read_byte(&(sin_table[position]));
  if (position < 120)
    position += 60;
  else
    position -= 120;
  OCR1B = pgm_read_byte(&(sin_table[position]));
  if (position < 120)
    position += 60;
  else
    position -= 120;
  OCR2A = pgm_read_byte(&(sin_table[position]));
}
