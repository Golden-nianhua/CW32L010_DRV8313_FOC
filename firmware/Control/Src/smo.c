#include "smo.h"
#include "foc.h"

#include <string.h>

enum {
    TURN = 16384,
    HALF_TURN = TURN / 2,
    Q8 = 256,
    MIN_FILTER_SHIFT = 1,
    MAX_FILTER_SHIFT = 8,
    VALID_FRAMES = 32,
    MAX_PHASE_ERROR = TURN / 24,
};

static int32_t clamp32(int32_t value, int32_t minimum, int32_t maximum) {
    return value < minimum ? minimum : (value > maximum ? maximum : value);
}

static int32_t wrap_phase(int32_t value) {
    return (int32_t)((uint32_t)(value + HALF_TURN) & (TURN - 1u)) - HALF_TURN;
}

static int32_t current_step(int32_t *estimate_q8, int32_t *previous_drive_mv, int32_t measured_ma,
                            int32_t voltage_mv, const SmoConfig *config) {
    /* Solve the sliding boundary layer implicitly. The 200 us path uses
     * trapezoidal R/L integration with the preceding voltage-minus-sliding
     * drive. Longer periods use backward Euler. */
    /* Keep the hot path in 32-bit arithmetic on Cortex-M0+. Split the
     * current's integer/fractional parts before applying its Q16 decay.
     * Q8 voltage gain loses less than 0.2 mA at the supported 12 V input. */
    int32_t integer_current = *estimate_q8 >> 8;
    int32_t fractional_current = *estimate_q8 & 255;
    int32_t decayed = integer_current * config->decay_q16 +
                      ((fractional_current * config->decay_q16) >> 8);
    int32_t voltage_gain_q8 = config->voltage_gain_q16 >> 8;
    int32_t drive = voltage_mv;
    if (config->sample_us == 200) drive += *previous_drive_mv;
    int32_t prediction = (decayed + voltage_gain_q8 * drive) >> 8;
    int32_t error = prediction - measured_ma * Q8;
    error = clamp32(error, -config->sliding_boundary_q8, config->sliding_boundary_q8);
    /* Saturation before multiplication bounds this product to gain_mv*2^17,
     * within int32_t for the configured maximum gain of 12000 mV. */
    int32_t correction = (config->sliding_gain_q17 * error) >> 17;
    *estimate_q8 = prediction - ((voltage_gain_q8 * correction) >> 8);
    *previous_drive_mv = voltage_mv - correction;
    return correction;
}

void Smo_ConfigDefault(SmoConfig *config) {
    *config = (SmoConfig){
        .resistance_mohm = 3700,
        .inductance_uh = 1000,
        .sample_us = 50,
        .sliding_gain_mv = 9000,
        .boundary_ma = 120,
        .emf_filter_shift = 3,
        .pll_kp_q8 = 96,
        .pll_ki_q8 = 8,
        .min_emf_mv = 120,
        .takeover_emf_mv = 400,
        .fallback_emf_mv = 280,
        .qualify_us = 100000,
    };
    Smo_Prepare(config);
}

void Smo_Prepare(SmoConfig *config) {
    int32_t l = config->inductance_uh * 1000;
    int32_t decay_numerator = l;
    if (config->sample_us == 200) {
        /* Trapezoidal integration cancels the backward-Euler phase error
         * amplified by large rotating negative Id in the fast observer. */
        l *= 2;
        decay_numerator = l - config->sample_us * config->resistance_mohm;
    }
    int32_t denominator = l + config->sample_us * config->resistance_mohm;
    config->decay_q16 = (int32_t)((int64_t)decay_numerator * 65536 / denominator);
    config->voltage_gain_q16 =
        (int32_t)((int64_t)config->sample_us * 1000 * Q8 * 65536 / denominator);
    config->sliding_boundary_q8 = config->boundary_ma * Q8 +
        (int32_t)(((int64_t)config->voltage_gain_q16 * config->sliding_gain_mv) >> 16);
    config->sliding_gain_q17 =
        (int32_t)((int64_t)config->sliding_gain_mv * (1 << 17) / config->sliding_boundary_q8);
}

void Smo_Init(SmoObserver *observer) {
    memset(observer, 0, sizeof(*observer));
    observer->direction = 1;
}

void Smo_Reset(SmoObserver *observer) {
    Smo_Init(observer);
}

void Smo_Seed(SmoObserver *observer, uint16_t phase, int32_t speed_q8) {
    observer->angle_q8 = (phase & (TURN - 1u)) * Q8;
    observer->speed_q8 = speed_q8;
    observer->direction = speed_q8 < 0 ? -1 : 1;
    observer->valid_frames = 0;
    observer->ready_us = 0;
    observer->ready = false;
}

void Smo_SetSamplePeriod(SmoObserver *observer, SmoConfig *config,
                         uint16_t sample_us, uint16_t control_phase) {
    observer->speed_q8 = (int32_t)((int64_t)observer->speed_q8 * sample_us /
                                  config->sample_us);
    /* Preserve the one-update prediction used by Sensorless_Step. */
    observer->angle_q8 = (int32_t)((uint32_t)(
        (int32_t)control_phase * Q8 - observer->speed_q8) & (TURN * Q8 - 1u));
    config->sample_us = sample_us;
    Smo_Prepare(config);
}

bool Smo_Update(SmoObserver *observer, const SmoConfig *config,
                const SmoInput *input) {
    int32_t correction_alpha = current_step(
        &observer->current_alpha_q8, &observer->previous_drive_alpha_mv, input->current_alpha_ma,
        input->voltage_alpha_mv, config);
    int32_t correction_beta = current_step(
        &observer->current_beta_q8, &observer->previous_drive_beta_mv, input->current_beta_ma,
        input->voltage_beta_mv, config);

    int32_t shift = clamp32(config->emf_filter_shift,
                            MIN_FILTER_SHIFT, MAX_FILTER_SHIFT);
    observer->emf_alpha_mv +=
        (correction_alpha - observer->emf_alpha_mv) >> shift;
    observer->emf_beta_mv +=
        (correction_beta - observer->emf_beta_mv) >> shift;

    uint32_t magnitude = FOC_Sqrt(
        (uint32_t)(observer->emf_alpha_mv * observer->emf_alpha_mv) +
        (uint32_t)(observer->emf_beta_mv * observer->emf_beta_mv));
    int32_t filtered_magnitude = observer->emf_magnitude_mv;
    filtered_magnitude += ((int32_t)magnitude - filtered_magnitude) / 8;
    observer->emf_magnitude_mv = (uint16_t)filtered_magnitude;

    bool visible = observer->emf_magnitude_mv >=
                   (uint16_t)clamp32(config->min_emf_mv, 0, 65535);
    observer->angle_q8 = (int32_t)((uint32_t)(observer->angle_q8 + observer->speed_q8) &
                                   (TURN * Q8 - 1u));
    if (visible) {
        /* e_alpha=-sin(theta), e_beta=cos(theta), so atan2(-e_alpha,e_beta)
         * gives the electrical rotor angle. */
        int32_t measured = FOC_Atan2(-observer->emf_alpha_mv,
                                     observer->emf_beta_mv);
        if (observer->direction < 0) measured += HALF_TURN;
        /* Exact sampled first-order filter lag:
         * atan2((2^shift-1) sin(w), 2^shift-(2^shift-1) cos(w)). */
        FocAngle step = FOC_Angle((uint16_t)(observer->speed_q8 / Q8));
        int32_t lag_y = (int32_t)((uint32_t)step.sin_q14 << shift) - step.sin_q14;
        int32_t lag_x = (16384 << shift) -
            ((int32_t)((uint32_t)step.cos_q14 << shift) - step.cos_q14);
        measured += wrap_phase(FOC_Atan2(lag_y, lag_x));
        int32_t estimated = observer->angle_q8 / Q8;
        int32_t error = wrap_phase((int32_t)measured - estimated);
        int32_t absolute_error = error < 0 ? -error : error;
        observer->phase_error += (absolute_error - observer->phase_error) / 8;
        observer->angle_q8 += config->pll_kp_q8 * error;
        observer->speed_q8 += config->pll_ki_q8 * error;
        observer->speed_q8 = clamp32(observer->speed_q8,
                                     -(HALF_TURN * Q8 - 1), HALF_TURN * Q8 - 1);
    }
    if (visible && observer->phase_error >= -MAX_PHASE_ERROR &&
        observer->phase_error <= MAX_PHASE_ERROR) {
        if (observer->valid_frames < VALID_FRAMES) ++observer->valid_frames;
    } else {
        observer->valid_frames = 0;
    }
    observer->angle_q8 = (int32_t)((uint32_t)observer->angle_q8 &
                                   (TURN * Q8 - 1u));
    bool valid = Smo_IsValid(observer, config);
    if (!valid || observer->emf_magnitude_mv < config->fallback_emf_mv) {
        observer->ready_us = 0;
        observer->ready = false;
    } else if (!observer->ready) {
        if (observer->emf_magnitude_mv >= config->takeover_emf_mv) {
            observer->ready_us += config->sample_us;
            observer->ready = observer->ready_us >= config->qualify_us;
        } else {
            observer->ready_us = 0;
        }
    }
    return valid;
}

uint16_t Smo_Angle(const SmoObserver *observer) {
    return (uint16_t)((uint32_t)(observer->angle_q8 / Q8) &
                      (TURN - 1u));
}

int32_t Smo_ElectricalSpeedQ8(const SmoObserver *observer) {
    return observer->speed_q8;
}

uint16_t Smo_EmfMagnitudeMv(const SmoObserver *observer) {
    return observer->emf_magnitude_mv;
}

bool Smo_IsValid(const SmoObserver *observer, const SmoConfig *config) {
    return observer->valid_frames >= VALID_FRAMES &&
           observer->emf_magnitude_mv >=
               (uint16_t)clamp32(config->min_emf_mv, 0, 65535);
}

bool Smo_CanTakeOver(const SmoObserver *observer) {
    return observer->ready;
}
