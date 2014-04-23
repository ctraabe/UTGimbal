#ifndef _BATTERY_H
#define _BATTERY_H

#include <inttypes.h>

// This file assumes that the battery is connected to ADC pin C2 via a voltage
// divider utilizing 22 KOhm and 1KOhm resistors. Therefore, the voltage at pin
// C2 is the battery voltage divided by 23. Furthermore, the ADC reference
// voltage is set to 1.1 V using 8-bit resolution. Therefore, the ADC resolution
// in terms of volts is 1.1 V * 23 / 2^8 steps ~ 0.1 V/step.

void BatteryMeasurementInit(void);
void BatteryMeasureVoltage(void);
uint8_t BatteryIsLow(void);

#endif //_BATTERY_H
