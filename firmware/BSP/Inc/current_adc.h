#ifndef CURRENT_ADC_H
#define CURRENT_ADC_H

#include <stdint.h>
#include <stdbool.h>

enum {
    CURRENT_PULSE_VECTORS = 4,
    CURRENT_PULSE_POINTS = 5,
    CURRENT_HFI_CYCLES = 40,
    CURRENT_HFI_CONTROL_CYCLES = CURRENT_HFI_CYCLES,
    CURRENT_HFI_SAMPLE_US = 1600,
    CURRENT_SMO_SAMPLE_US = 200,
};

typedef struct {
    uint32_t repeats;
    int32_t sum[CURRENT_PULSE_POINTS][2];
    uint32_t square[CURRENT_PULSE_POINTS][2];
} CurrentPulseResponse;

typedef struct {
    uint32_t sequence;
    int16_t delta[CURRENT_PULSE_VECTORS][2];
    uint16_t raw_b, raw_c;
    int32_t voltage_alpha_mv, voltage_beta_mv;
    uint16_t sample_us;
} CurrentHfiFrame;

void CurrentAdc_Init(void);
void CurrentAdc_StartAudio(uint16_t zero_b, uint16_t zero_c, uint32_t frames, bool mulaw);
void CurrentAdc_StopAudio(void);
uint16_t CurrentAdc_SupplyRaw(void);
/* Main-context supply update for duty-to-voltage reconstruction. */
void CurrentAdc_SetBusVoltage(uint16_t bus_mv);
/* Consume the oldest retained sample. Sequence gaps expose queue overflow. */
uint32_t CurrentAdc_Next(uint16_t *raw_b, uint16_t *raw_c, uint32_t *time_us);
void CurrentAdc_StartPulse(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], uint8_t periods,
                           uint8_t repeats, uint16_t zero_b, uint16_t zero_c);
void CurrentAdc_CancelPulse(void);
bool CurrentAdc_PulseActive(void);
uint16_t CurrentAdc_PulseFault(void);
const CurrentPulseResponse *CurrentAdc_PulseResponses(void);
/* Continuous HFI duties use Q8 timer counts at 100 kHz, in [0, 61440]. */
void CurrentAdc_StartHfi(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], uint16_t zero_b, uint16_t zero_c);
void CurrentAdc_SetHfiCurrentLimit(uint16_t current_ma);
void CurrentAdc_SetHfiDuty(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3]);
bool CurrentAdc_ReadHfi(CurrentHfiFrame *frame);
/* Change the running sensorless sample rate without disabling the bridge.
 * The transition target retains the HFI Q8 timer-count format. */
void CurrentAdc_SetSmoMode(bool enabled, uint16_t duty[CURRENT_PULSE_VECTORS + 1][3]);
void CurrentAdc_SetSmoDuty(uint16_t a, uint16_t b, uint16_t c);

#endif
