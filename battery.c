#include "battery.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "boolean.h"


// =============================================================================
// Private data:

static volatile uint8_t _battery_voltage_adc = 0, _battery_measurement_done = 0;
static uint8_t _battery_low_voltage = 0;


// =============================================================================
// Private function declarations:

void BatteryGuessCells(void);


// =============================================================================
// Public functions:

void BatteryMeasurementInit(void)
{
  DDRC &= ~_BV(DDC2);  // Make sure that pin C2 is set to input
  // Enable the ADC and ADC interrupt, and set the prescaler to 1/128
  ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  // Set the voltage reference to internal 1.1V, use only 8-bits, and set input
  // to ADC2
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(ADLAR) | _BV(MUX1);

  BatteryGuessCells();
}

// -----------------------------------------------------------------------------
void BatteryMeasureVoltage(void)
{
  _battery_measurement_done = 0;
  ADCSRA |= _BV(ADSC);
}

// -----------------------------------------------------------------------------
uint8_t BatteryIsLow(void)
{
  return _battery_voltage_adc < _battery_low_voltage;
}


// =============================================================================
// Private functions:

// This function guesses the number of cells of the attached battery given its
// voltage.
void BatteryGuessCells(void)
{
  BatteryMeasureVoltage();
  while (!_battery_measurement_done) continue;

  if (_battery_voltage_adc < 96)  // 2 cell battery
    _battery_low_voltage = 67;
  else if (_battery_voltage_adc < 128)  // 3 cell battery
    _battery_low_voltage = 100;
  else  // More than 3 cells
    _battery_low_voltage = 132;
}

// -----------------------------------------------------------------------------
ISR(ADC_vect) {
  _battery_voltage_adc = ADCH;
  _battery_measurement_done = 1;
}
