#include "bus_sense.h"

enum {
    ADC_MAX = 4095,
    KEY_PRESS_COUNTS = BUS_SENSE_KEY_PRESS_PIN_MV * ADC_MAX / BUS_SENSE_ADC_REFERENCE_MV,
    KEY_RELEASE_COUNTS = BUS_SENSE_KEY_RELEASE_PIN_MV * ADC_MAX / BUS_SENSE_ADC_REFERENCE_MV,
};

static int32_t voltage_q3 = BUS_SENSE_DEFAULT_MV * 8;
static uint8_t press_samples;
static bool key_pressed, key_pending;

void BusSense_Update(uint16_t raw, bool connected) {
    if (!connected) {
        voltage_q3 = BUS_SENSE_DEFAULT_MV * 8;
        key_pressed = key_pending = false;
        press_samples = 0;
        return;
    }
    if (raw <= KEY_PRESS_COUNTS) {
        if (press_samples < 2) ++press_samples;
        if (press_samples == 2 && !key_pressed) {
            key_pressed = key_pending = true;
        }
        /* A pressed KEY is not a supply-voltage measurement. */
        return;
    }
    press_samples = 0;
    if (raw < KEY_RELEASE_COUNTS) return;
    key_pressed = false;
    /* Constant-folded divider gain. No 64-bit arithmetic in the sample path. */
    const uint32_t gain_q16 = (uint32_t)(
        (uint64_t)BUS_SENSE_ADC_REFERENCE_MV * (BUS_SENSE_TOP_OHM + BUS_SENSE_BOTTOM_OHM) *
        65536u / (ADC_MAX * BUS_SENSE_BOTTOM_OHM));
    int32_t measured_mv = (int32_t)(((uint32_t)raw * gain_q16 + 32768u) >> 16);
    voltage_q3 += (measured_mv * 8 - voltage_q3) / 4;
}

uint16_t BusSense_Voltage(void) { return (uint16_t)(voltage_q3 / 8); }
bool BusSense_KeyPressed(void) { return key_pressed; }
bool BusSense_TakeKeyPress(void) {
    bool requested = key_pending;
    key_pending = false;
    return requested;
}
