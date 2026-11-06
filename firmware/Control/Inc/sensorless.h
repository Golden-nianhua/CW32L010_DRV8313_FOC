#ifndef SENSORLESS_H
#define SENSORLESS_H

#include "smo.h"

typedef enum {
    CONTROL_NONE,
    CONTROL_ENCODER,
    CONTROL_HFI,
    CONTROL_TO_SMO,
    CONTROL_SMO,
    CONTROL_TO_HFI,
    CONTROL_LOST,
} ControlSource;

typedef struct {
    int32_t phase_offset;
    uint32_t qualified_us;
    uint32_t loss_us;
    uint32_t retry_us;
    uint16_t phase;
    uint16_t blend_q8;
    ControlSource source;
    int32_t speed_q8;
    uint32_t settling_us;
} Sensorless;

void Sensorless_Start(Sensorless *control, uint16_t phase);
/* field_speed_q8 uses electrical counts per observer update, as does SMO. */
uint16_t Sensorless_Step(Sensorless *control, SmoObserver *observer,
                         const SmoConfig *config, uint16_t field_phase,
                         int32_t field_speed_q8, bool enabled);

#endif
