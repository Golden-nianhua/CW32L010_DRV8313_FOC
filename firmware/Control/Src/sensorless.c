#include "sensorless.h"

enum {
    TURN = 16384,
    MAX_HANDOVER_ERROR = TURN / 16,
    PHASE_SLEW = 16,
    BLEND_STEP = 4,
    RETRY_US = 500000,
    FALLBACK_US = 40000,
    /* 20 mechanical rad/s, seven pole pairs, Q8 counts per microsecond. */
    HFI_SPEED_Q8_PER_US = 94,
};

static int32_t magnitude(int32_t value) {
    return value < 0 ? -value : value;
}

static int32_t wrap(int32_t value) {
    return (int32_t)((uint32_t)(value + TURN / 2) & (TURN - 1u)) - TURN / 2;
}

static void begin_fallback(Sensorless *control, SmoObserver *observer) {
    control->source = CONTROL_TO_HFI;
    control->retry_us = RETRY_US;
    control->qualified_us = 0;
    control->loss_us = 0;
    observer->ready = false;
    observer->ready_us = 0;
}

void Sensorless_Start(Sensorless *control, uint16_t phase) {
    *control = (Sensorless){.phase = phase, .source = CONTROL_HFI};
}

uint16_t Sensorless_Step(Sensorless *control, SmoObserver *observer,
                         const SmoConfig *config, uint16_t field_phase,
                         int32_t field_speed_q8, bool enabled) {
    if (control->source == CONTROL_LOST) return control->phase;
    bool was_smo = control->source == CONTROL_TO_SMO || control->source == CONTROL_SMO;
    /* The measured frame precedes the PWM being prepared. Predict one update. */
    uint16_t predicted = (uint16_t)((Smo_Angle(observer) +
                                    observer->speed_q8 / 256) & (TURN - 1u));
    bool emf_available = enabled &&
        observer->emf_magnitude_mv >= config->fallback_emf_mv;
    bool available = emf_available && Smo_IsValid(observer, config) &&
                     observer->phase_error <= TURN / 24;
    bool resampling = control->settling_us != 0;
    control->settling_us = control->settling_us > (uint32_t)config->sample_us
        ? control->settling_us - config->sample_us : 0;
    int32_t maximum_phase_step = magnitude(control->speed_q8) / 128 + 512;
    bool phase_step_valid = true;
    if (control->source == CONTROL_TO_SMO || control->source == CONTROL_SMO) {
        phase_step_valid =
            magnitude(wrap((int32_t)predicted + control->phase_offset - control->phase))
                <= maximum_phase_step;
        /* A rate change shifts the current-average/filter delay. If the PLL
         * remains qualified, absorb a bounded phase correction in the existing
         * slew offset instead of permanently rejecting the shifted trajectory. */
        int32_t continuation = control->phase + control->speed_q8 / 256;
        int32_t correction = wrap((int32_t)predicted + control->phase_offset - continuation);
        bool phase_qualified = available || (resampling && emf_available &&
                                             observer->phase_error <= TURN / 24);
        int32_t correction_limit = resampling ? TURN / 3 : TURN / 8;
        if (!phase_step_valid && phase_qualified && magnitude(correction) < correction_limit) {
            control->phase_offset = wrap(control->phase_offset - correction);
            phase_step_valid = true;
        }
    }
    bool low_speed = magnitude(control->speed_q8) <=
                     config->sample_us * HFI_SPEED_Q8_PER_US;
    bool tracking = emf_available && observer->phase_error < TURN / 4 &&
                    phase_step_valid;
    if (was_smo && !phase_step_valid && low_speed) {
        begin_fallback(control, observer);
    }
    if (control->source == CONTROL_TO_SMO || control->source == CONTROL_SMO) {
        control->loss_us = tracking ? 0 : control->loss_us + config->sample_us;
    }
    if ((control->source == CONTROL_TO_SMO || control->source == CONTROL_SMO) &&
        control->loss_us >= FALLBACK_US) {
        if (low_speed) begin_fallback(control, observer);
        else {
            control->source = CONTROL_LOST;
            return control->phase;
        }
    }
    if (control->source == CONTROL_HFI) {
        if (control->retry_us != 0) {
            control->retry_us = control->retry_us > (uint32_t)config->sample_us
                ? control->retry_us - config->sample_us : 0;
        }
        int32_t field_offset = wrap(field_phase - predicted);
        int32_t phase_error = wrap(field_offset - control->phase_offset);
        control->phase_offset = wrap(control->phase_offset + phase_error / 8);
        int32_t speed_error = magnitude(observer->speed_q8 - field_speed_q8);
        int32_t speed_window = magnitude(field_speed_q8) / 4 + 256;
        bool aligned = available && Smo_CanTakeOver(observer) &&
            magnitude(control->phase_offset) <= MAX_HANDOVER_ERROR &&
            magnitude(field_offset) < TURN / 8 &&
            speed_error <= speed_window && control->retry_us == 0;
        control->qualified_us = aligned ? control->qualified_us + config->sample_us : 0;
        if (control->qualified_us >= (uint32_t)config->qualify_us) {
            control->source = CONTROL_TO_SMO;
            control->phase_offset = field_offset;
            control->speed_q8 = observer->speed_q8;
        }
    }
    if (control->source == CONTROL_TO_SMO || control->source == CONTROL_SMO) {
        if (phase_step_valid) control->speed_q8 = observer->speed_q8;
        else {
            /* Reject a bad observer jump while retaining the rotating field. */
            predicted = (uint16_t)((control->phase + control->speed_q8 / 256 -
                                    control->phase_offset) & (TURN - 1u));
        }
        int32_t offset = control->phase_offset;
        if (offset > PHASE_SLEW) offset -= PHASE_SLEW;
        else if (offset < -PHASE_SLEW) offset += PHASE_SLEW;
        else offset = 0;
        control->phase_offset = offset;
        if (control->blend_q8 < 256) control->blend_q8 += BLEND_STEP;
        control->phase = (uint16_t)((predicted + offset) & (TURN - 1u));
        if (offset == 0 && control->blend_q8 == 256) control->source = CONTROL_SMO;
    } else {
        if (was_smo && control->source == CONTROL_TO_HFI) {
            field_phase = (uint16_t)((control->phase + control->speed_q8 / 256) &
                                     (TURN - 1u));
        }
        control->phase = field_phase;
        if (control->source == CONTROL_TO_HFI) {
            control->blend_q8 -= BLEND_STEP;
            if (control->blend_q8 == 0) control->source = CONTROL_HFI;
        }
    }
    return control->phase;
}
