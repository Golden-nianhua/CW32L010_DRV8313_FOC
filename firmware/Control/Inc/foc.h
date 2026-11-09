#ifndef FOC_H
#define FOC_H

#include <stdint.h>

typedef struct {
    int32_t d;
    int32_t q;
} Dq;

typedef struct {
    int32_t sin_q14;
    int32_t cos_q14;
} FocAngle;

typedef struct {
    uint16_t a;
    uint16_t b;
    uint16_t c;
} PhaseDuty;

/* Main-context supply update. Gains are prepared outside the control hot path. */
void FOC_SetBusVoltage(uint16_t bus_mv);
uint16_t FOC_GetBusVoltage(void);
uint16_t FOC_GetVoltageLimit(void);

/* One electrical turn is 16384 counts, matching MT6701. */
FocAngle FOC_Angle(uint16_t phase);
/* atan2(y,x) in electrical counts. (0,0) maps to zero. */
uint16_t FOC_Atan2(int32_t y, int32_t x);
/* Exact floor(sqrt(value)) across the uint32_t range. */
uint32_t FOC_Sqrt(uint32_t value);
/* Conservative root for current budgets, within about 0.2% plus one count. */
uint32_t FOC_SqrtLowerBound(uint32_t value);
Dq FOC_Park(int32_t current_a_ma, int32_t current_b_ma, FocAngle angle);
PhaseDuty FOC_Modulate(Dq voltage_mv, FocAngle angle, uint16_t bus_mv, uint16_t pwm_max);
PhaseDuty FOC_ModulateStationary(int32_t alpha_mv, int32_t beta_mv,
                                 uint16_t bus_mv, uint16_t pwm_max);

#endif
