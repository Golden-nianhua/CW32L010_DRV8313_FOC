#ifndef SMO_H
#define SMO_H

#include <stdbool.h>
#include <stdint.h>

/*
 * A stationary-frame sliding-mode observer for sensorless FOC.
 *
 * Inputs and outputs use the same engineering units as the current and
 * voltage control code. Sensorless_Step qualifies and blends its control
 * angle into the low-speed startup path.
 */
typedef struct {
    int32_t resistance_mohm;
    int32_t inductance_uh;
    int32_t sample_us;
    int32_t sliding_gain_mv;
    int32_t boundary_ma;
    int32_t emf_filter_shift;
    int32_t pll_kp_q8;
    int32_t pll_ki_q8;
    int32_t min_emf_mv;
    int32_t decay_q16;
    int32_t voltage_gain_q16;
    int32_t sliding_gain_q17;
    int32_t takeover_emf_mv;
    int32_t fallback_emf_mv;
    int32_t qualify_us;
    int32_t sliding_boundary_q8;
} SmoConfig;

typedef struct {
    int32_t voltage_alpha_mv;
    int32_t voltage_beta_mv;
    int32_t current_alpha_ma;
    int32_t current_beta_ma;
} SmoInput;

typedef struct {
    int32_t current_alpha_q8;
    int32_t current_beta_q8;
    int32_t emf_alpha_mv;
    int32_t emf_beta_mv;
    int32_t angle_q8;
    int32_t speed_q8;
    int32_t direction;
    int32_t phase_error; /* Smoothed absolute PLL error in electrical counts. */
    uint16_t emf_magnitude_mv;
    uint16_t valid_frames;
    int32_t ready_us;
    bool ready;
} SmoObserver;

void Smo_ConfigDefault(SmoConfig *config);
/* Recompute coefficients after changing motor, sampling or sliding parameters. */
void Smo_Prepare(SmoConfig *config);
/* Change update period while retaining the selected electrical control angle. */
void Smo_SetSamplePeriod(SmoObserver *observer, SmoConfig *config,
                         uint16_t sample_us, uint16_t control_phase);
void Smo_Init(SmoObserver *observer);
void Smo_Reset(SmoObserver *observer);
/* Seed direction from the starting observer. Speed is electrical counts/tick Q8. */
void Smo_Seed(SmoObserver *observer, uint16_t phase, int32_t speed_q8);
bool Smo_Update(SmoObserver *observer, const SmoConfig *config,
                const SmoInput *input);
uint16_t Smo_Angle(const SmoObserver *observer);
int32_t Smo_ElectricalSpeedQ8(const SmoObserver *observer);
uint16_t Smo_EmfMagnitudeMv(const SmoObserver *observer);
bool Smo_IsValid(const SmoObserver *observer, const SmoConfig *config);
bool Smo_CanTakeOver(const SmoObserver *observer);

#endif
