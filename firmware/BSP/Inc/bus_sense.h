#ifndef BUS_SENSE_H
#define BUS_SENSE_H

#include <stdbool.h>
#include <stdint.h>

/* Enable after fitting the PA3 divider. KEY shorts the lower resistor. */
#ifndef BUS_SENSE_ADC_ENABLED
#define BUS_SENSE_ADC_ENABLED 0
#endif
#define BUS_SENSE_TOP_OHM 100000u
#define BUS_SENSE_BOTTOM_OHM 10000u
#define BUS_SENSE_ADC_REFERENCE_MV 3300u
#define BUS_SENSE_DEFAULT_MV 12000u
#define BUS_SENSE_KEY_PRESS_PIN_MV 100u
#define BUS_SENSE_KEY_RELEASE_PIN_MV 200u

void BusSense_Update(uint16_t raw, bool connected);
uint16_t BusSense_Voltage(void);
bool BusSense_KeyPressed(void);
bool BusSense_TakeKeyPress(void);

#endif
