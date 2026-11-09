#include "motion.h"

#define OUTER_MS 5
#define PI_TICKS_PER_MS 5

enum {
    Q_AXIS_RESISTANCE_MOHM = 3700,
    Q_AXIS_BACK_EMF_MV_PER_RAD_S = 19,
    SPEED_DAMPING_MA_PER_RAD_S = 12,
    SPEED_DELTA_LIMIT_MRAD_S = 5000,
    SPEED_DAMPING_FULL_MRAD_S = 10000,
    SPEED_DAMPING_END_MRAD_S = 20000,
    SPEED_DAMPING_SPAN_MRAD_S = SPEED_DAMPING_END_MRAD_S - SPEED_DAMPING_FULL_MRAD_S,
    TURN_MRAD = 6283,
    RATCHET_HALF_TOOTH = TURN_MRAD / 2,
    RATCHET_SWITCH = TURN_MRAD * 55 / 100,
    RATCHET_RETURN_TAPER_MRAD_S = 1000,
    RATCHET_FINE_TEETH = 48,
    RATCHET_DAMPING_SCALE = 4000,
    RATCHET_MOVING_CURRENT_MA = 250,
    RATCHET_BOOST_SPEED_MRAD_S = 600,
    RATCHET_BOOST_SAMPLES = 20,
    HFI_FRAME_TICKS = 8,
    HFI_TURN = 16384,
    HFI_ANGLE_SCALE = 256,
};

static int32_t clamp(int32_t value, int32_t min, int32_t max) {
    return value < min ? min : (value > max ? max : value);
}

static int32_t magnitude(int32_t value) {
    return value < 0 ? -value : value;
}

static uint16_t hfi_current_budget(const Motion *motion) {
    int32_t id = motion->hfi_field_target_ma;
    if (id > motion->current_limit_ma) id = motion->current_limit_ma;
    return (uint16_t)FOC_Sqrt(
        (uint32_t)(motion->current_limit_ma * motion->current_limit_ma - id * id));
}

/* Exact division in the usual PI range with 32-bit reciprocal multiplication.
 * Keep software division for large products allowed by runtime gain settings. */
__attribute__((noinline)) static int32_t per_thousand(int32_t value) {
    uint32_t absolute = value < 0 ? 0u - (uint32_t)value : (uint32_t)value;
    uint32_t quotient;
    if (absolute <= 32000000u) {
        quotient = ((absolute >> 8) * 16777u) >> 16;
        if ((quotient + 1u) * 1000u <= absolute) ++quotient;
    } else {
        quotient = absolute / 1000u;
    }
    return value < 0 ? -(int32_t)quotient : (int32_t)quotient;
}

static int32_t pi_step(MotorPI *pi, int32_t error, int32_t limit, int32_t ticks, int32_t bias) {
    error = clamp(error, -30000, 30000);
    int32_t base = bias + per_thousand(pi->kp * error);
    int32_t integral_limit = limit * 1000;
    int32_t previous = clamp(pi->integral, -integral_limit, integral_limit);
    int32_t next = clamp(previous + per_thousand(pi->ki * error) * ticks / PI_TICKS_PER_MS,
                         -integral_limit, integral_limit);
    int32_t output = base + per_thousand(next);
    if ((output > limit && error > 0) || (output < -limit && error < 0)) {
        next = previous;
        output = base + per_thousand(next);
    }
    pi->integral = next;
    return clamp(output, -limit, limit);
}

static uint32_t integer_sqrt(uint32_t value) {
    uint32_t root = 0;
    for (uint32_t bit = 1u << 30; bit != 0; bit >>= 2) {
        if (value >= root + bit) {
            value -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
    }
    return root;
}

static void trajectory_step(Motion *motion) {
    int32_t error = motion->request.target - motion->profile_position;
    int32_t velocity = motion->profile_velocity;
    int32_t accel = motion->request.option_b;
    int32_t vmax = motion->request.option_a;
    int32_t dv = accel * OUTER_MS / 1000;
    if (magnitude(error) <= 1 && magnitude(velocity) <= dv) {
        motion->profile_position = motion->request.target;
        motion->profile_velocity = 0;
        motion->profile_fraction = 0;
        return;
    }
    int32_t braking = vmax;
    if (magnitude(error) <= vmax * vmax / (2 * accel)) {
        braking = (int32_t)integer_sqrt((uint32_t)(2 * accel * magnitude(error)));
    }
    int32_t target_speed = error < 0 ? -braking : braking;
    motion->profile_velocity = clamp(target_speed, velocity - dv, velocity + dv);
    int32_t step = motion->profile_velocity * OUTER_MS + motion->profile_fraction;
    motion->profile_position += step / 1000;
    motion->profile_fraction = step % 1000;
}

static int32_t ratchet_current(Motion *motion, int32_t position, int32_t speed) {
    int32_t teeth = motion->request.target;
    int32_t limit = clamp(motion->request.option_a, 0, motion->current_limit_ma);
    if (limit > RATCHET_MOVING_CURRENT_MA) {
        /* Reserve extra holding torque for slow motion and stationary teeth. */
        if (magnitude(speed) > RATCHET_BOOST_SPEED_MRAD_S) {
            motion->ratchet_boost = 0;
        } else if (motion->ratchet_boost < RATCHET_BOOST_SAMPLES) {
            ++motion->ratchet_boost;
        }
        limit = RATCHET_MOVING_CURRENT_MA +
            (limit - RATCHET_MOVING_CURRENT_MA) * motion->ratchet_boost / RATCHET_BOOST_SAMPLES;
    }
    /* Tooth coordinates keep fractional milliradians until the final division. */
    int32_t offset = (position - motion->ratchet_origin) * teeth - motion->ratchet_tooth * TURN_MRAD;
    if (magnitude(offset) >= RATCHET_SWITCH) {
        int32_t step = (magnitude(offset) + RATCHET_HALF_TOOTH) / TURN_MRAD;
        if (offset < 0) step = -step;
        motion->ratchet_tooth += step;
        offset -= step * TURN_MRAD;
        motion->profile_position = motion->ratchet_origin + motion->ratchet_tooth * TURN_MRAD / teeth;
    }
    int32_t deflection = clamp(offset, -RATCHET_HALF_TOOTH, RATCHET_HALF_TOOTH);
    /* Ease in near the center while keeping full resistance at the tooth edge. */
    int32_t spring = -deflection * limit / RATCHET_HALF_TOOTH;
    spring = spring * (RATCHET_HALF_TOOTH + 2 * magnitude(deflection)) / (3 * RATCHET_HALF_TOOTH);
    if (teeth > RATCHET_FINE_TEETH) spring = spring * magnitude(deflection) / RATCHET_HALF_TOOTH;
    /* A fast approach needs less pull so the rotor settles in the new tooth. */
    int32_t approach = clamp(offset < 0 ? speed : -speed, 0, RATCHET_RETURN_TAPER_MRAD_S);
    spring -= spring * 7 * approach / (8 * RATCHET_RETURN_TAPER_MRAD_S);
    int32_t damping_gain = motion->request.option_b + teeth * limit / RATCHET_DAMPING_SCALE;
    int32_t damping = damping_gain * clamp(speed, -MOTOR_SPEED_LIMIT_MRAD_S, MOTOR_SPEED_LIMIT_MRAD_S) / 1000;
    return clamp(spring - damping, -limit, limit);
}

void Motion_Init(Motion *motion) {
    *motion = (Motion){
        .loops = {
            [LOOP_D] = {.kp = 500, .ki = 30000},
            [LOOP_Q] = {.kp = 500, .ki = 30000},
            [LOOP_SPEED] = {.kp = 20, .ki = 500},
            [LOOP_POSITION] = {.kp = 4000},
        },
        .current_limit_ma = MOTOR_CURRENT_LIMIT_MA,
        .voltage_limit_mv = MOTOR_VOLTAGE_LIMIT_MV,
        .hfi_field_target_ma = MOTOR_HFI_FIELD_DEFAULT_MA,
        .sensorless_speed = {.kp = 5, .ki = 20},
        .sample_ticks = HFI_FRAME_TICKS,
        .hfi_acceleration_mrad_s2 = MOTOR_HFI_ACCEL_MRAD_S2,
        .weakening_max_ma = 1000,
        .weakening_utilization_permille = 950,
        .weakening_slew_ma_s = 1000,
    };
    Motion_Reset(motion, 0, 0);
}

void Motion_SetLimits(Motion *motion, int32_t current_ma, int32_t voltage_mv) {
    motion->current_limit_ma = current_ma;
    motion->voltage_limit_mv = voltage_mv;
    motion->current_q_limit_ma = (uint16_t)current_ma;
    motion->weakening_q_limit_ma = (uint16_t)current_ma;
    motion->hfi_q_limit_ma = hfi_current_budget(motion);
    motion->id_slew_fraction = 0;
}

void Motion_SetHfiFieldCurrent(Motion *motion, int32_t current_ma) {
    motion->hfi_field_target_ma = current_ma;
    motion->hfi_q_limit_ma = hfi_current_budget(motion);
}

void Motion_SetWeakening(Motion *motion, uint16_t maximum_ma,
                         uint16_t utilization_permille, uint16_t slew_ma_s) {
    motion->weakening_max_ma = maximum_ma;
    motion->weakening_utilization_permille = utilization_permille;
    motion->weakening_slew_ma_s = slew_ma_s;
}

void Motion_Reset(Motion *motion, int32_t position, int32_t speed) {
    for (unsigned i = 0; i < LOOP_COUNT; ++i) {
        motion->loops[i].integral = 0;
    }
    motion->iq_ref = 0;
    motion->hfi_reference_q8 = 0;
    motion->hfi_field_ma = 0;
    motion->hfi_iq_target = 0;
    motion->hfi_speed_divider = 0;
    motion->previous_speed = speed;
    motion->profile_position = position;
    motion->profile_velocity = 0;
    motion->profile_fraction = 0;
    motion->ratchet_origin = position;
    motion->ratchet_tooth = 0;
    motion->ratchet_boost = 0;
    motion->outer_count = OUTER_MS - 1u;
    motion->sensorless_regulate = false;
    motion->sensorless_blend_q8 = 0;
    motion->sensorless_speed.integral = 0;
    motion->hfi_velocity_q8 = 0;
    motion->voltage_saturation = 0;
    motion->sample_ticks = HFI_FRAME_TICKS;
    motion->iq_slew_fraction = 0;
    motion->injection_headroom_mv = 0;
    motion->weakening_integral_q6 = 0;
    motion->current_q_limit_ma = (uint16_t)motion->current_limit_ma;
    motion->weakening_q_limit_ma = (uint16_t)motion->current_limit_ma;
    motion->previous_vd_mv = motion->previous_vq_mv = 0;
    motion->weakening_voltage_mv = 0;
    motion->hfi_q_limit_ma = hfi_current_budget(motion);
    motion->id_slew_fraction = 0;
}

static void update_outer(Motion *motion, int32_t position, int32_t speed) {
    int32_t target = motion->request.target;
    int32_t target_speed;
    switch (motion->request.mode) {
        case MOTOR_IMPEDANCE:
            motion->iq_ref = clamp(
                (motion->request.option_a * clamp(target - position, -30000, 30000)) / 1000
                - (motion->request.option_b * clamp(speed, -30000, 30000)) / 1000,
                -motion->current_limit_ma, motion->current_limit_ma);
            return;
        case MOTOR_TRAJECTORY:
            trajectory_step(motion);
            target_speed = pi_step(&motion->loops[LOOP_POSITION], motion->profile_position - position,
                                   motion->request.option_a, OUTER_MS * PI_TICKS_PER_MS, 0);
            target_speed = clamp(target_speed + motion->profile_velocity,
                                 -motion->request.option_a, motion->request.option_a);
            break;
        case MOTOR_POSITION:
            target_speed = pi_step(&motion->loops[LOOP_POSITION], target - position,
                                   MOTOR_SPEED_LIMIT_MRAD_S, OUTER_MS * PI_TICKS_PER_MS, 0);
            break;
        default:
            target_speed = target;
            break;
    }
    /* Use measured acceleration so a setpoint step does not kick the damping term. */
    int32_t speed_delta = clamp(speed - motion->previous_speed,
                                -SPEED_DELTA_LIMIT_MRAD_S, SPEED_DELTA_LIMIT_MRAD_S);
    motion->previous_speed = speed;
    /* Fade the damping at higher speeds, where it amplified the measured ripple. */
    int32_t span = clamp(SPEED_DAMPING_END_MRAD_S - magnitude(target_speed), 0, SPEED_DAMPING_SPAN_MRAD_S);
    int32_t damping = -SPEED_DAMPING_MA_PER_RAD_S * speed_delta * span / (1000 * SPEED_DAMPING_SPAN_MRAD_S);
    motion->iq_ref = pi_step(&motion->loops[LOOP_SPEED], target_speed - speed,
                             motion->current_limit_ma, OUTER_MS * PI_TICKS_PER_MS, damping);
}

void Motion_HfiStart(Motion *motion, uint16_t rotor_phase) {
    Motion_Reset(motion, 0, 0);
    motion->hfi_reference_q8 = rotor_phase * HFI_ANGLE_SCALE;
}

uint16_t Motion_HfiAdvance(Motion *motion) {
    int32_t target = motion->request.target;
    /* Keep the 625 Hz startup field below its doubled-angle aliasing range
     * until the observer and current loop have switched to 5 kHz. */
    if (motion->sample_ticks != 1) {
        target = clamp(target, -MOTOR_HFI_START_SPEED_MRAD_S, MOTOR_HFI_START_SPEED_MRAD_S);
    }
    int32_t delta = target - motion->profile_velocity;
    unsigned period_shift = motion->sample_ticks == 1 ? 0u : 3u;
    uint32_t accumulated = ((uint32_t)motion->hfi_acceleration_mrad_s2 << period_shift) +
                           (uint32_t)motion->profile_fraction;
    /* The numerator is at most 8004999. This reciprocal is below 1/5000
     * and at most one quotient count short across that range. */
    uint32_t acceleration = ((accumulated >> 8) * 3355u) >> 16;
    if ((acceleration + 1u) * 5000u <= accumulated) ++acceleration;
    motion->profile_fraction = (int32_t)(accumulated - acceleration * 5000u);
    motion->profile_velocity += clamp(delta, -(int32_t)acceleration, (int32_t)acceleration);
    /* Advance the electrical field at the requested mechanical speed. */
    uint32_t speed = (uint32_t)magnitude(motion->profile_velocity);
    /* Q16 coefficient 61250 approximates 0.9346 within 0.9 ppm. Split
     * the product to keep 1000 rad/s within 32 bits without division. */
    uint32_t scaled_speed = (speed >> 8) * 61250u +
                           (((speed & 255u) * 61250u) >> 8);
    uint32_t advance_magnitude = (scaled_speed >> 8) << period_shift;
    int32_t advance = motion->profile_velocity < 0 ? -(int32_t)advance_magnitude : (int32_t)advance_magnitude;
    motion->hfi_velocity_q8 = advance;
    int32_t half = HFI_TURN * HFI_ANGLE_SCALE / 2;
    motion->hfi_reference_q8 =
        (int32_t)((uint32_t)(motion->hfi_reference_q8 + advance) & (2 * half - 1u));
    return (uint16_t)(motion->hfi_reference_q8 / HFI_ANGLE_SCALE);
}

void Motion_SetSamplePeriod(Motion *motion, uint16_t sample_us) {
    motion->sample_ticks = (uint8_t)(sample_us / 200u);
    motion->hfi_speed_divider = 0;
    motion->iq_slew_fraction = 0;
    motion->id_slew_fraction = 0;
}

void Motion_SetSensorless(Motion *motion, bool regulate, uint16_t blend_q8) {
    if (regulate && !motion->sensorless_regulate) {
        motion->sensorless_speed.integral = motion->iq_ref * 1000;
        motion->hfi_iq_target = motion->iq_ref;
        motion->hfi_speed_divider = 0;
    } else if (!regulate && motion->sensorless_regulate) {
        motion->hfi_iq_target = 0;
    }
    motion->sensorless_regulate = regulate;
    motion->sensorless_blend_q8 = blend_q8 > 256 ? 256 : blend_q8;
}

static void weakening_step(Motion *motion, int32_t speed) {
    bool active = motion->sensorless_regulate && motion->sensorless_blend_q8 == 256 &&
        motion->sample_ticks == 1 && magnitude(speed) >= 200000 &&
        motion->weakening_max_ma != 0;
    if (!active && motion->weakening_integral_q6 == 0) {
        motion->weakening_voltage_mv = 0;
        return;
    }
    int32_t vd = motion->previous_vd_mv, vq = motion->previous_vq_mv;
    int32_t voltage = (int32_t)FOC_Sqrt((uint32_t)(vd * vd + vq * vq));
    motion->weakening_voltage_mv = (uint16_t)voltage;
    int32_t desired = 0;
    if (active) {
        int32_t target_mv = per_thousand(
            motion->voltage_limit_mv * motion->weakening_utilization_permille);
        int32_t maximum_ma = motion->weakening_max_ma;
        if (maximum_ma > motion->current_limit_ma) maximum_ma = motion->current_limit_ma;
        /* The voltage loop runs every 8 ms. Its integral is mA in Q6,
         * so a 64 mV excess changes the negative Id target by 1 mA. */
        desired = clamp(motion->weakening_integral_q6 + target_mv - voltage,
                         -maximum_ma * 64, 0);
    }
    int32_t step = per_thousand(motion->weakening_slew_ma_s * (8 * 64));
    motion->weakening_integral_q6 += clamp(
        desired - motion->weakening_integral_q6, -step, 2 * step);
}

Dq Motion_Step(Motion *motion, int32_t position, int32_t speed, Dq current) {
    int32_t voltage_limit = motion->voltage_limit_mv - motion->injection_headroom_mv;
    if (motion->request.mode == MOTOR_VOLTAGE) {
        return (Dq){.q = clamp(motion->request.target,
                               -voltage_limit, voltage_limit)};
    }
    if (motion->request.mode == MOTOR_CURRENT) {
        motion->iq_ref = motion->request.target;
    } else if (motion->request.mode == MOTOR_HFI) {
        int32_t handover = motion->sensorless_blend_q8;
        /* Establish observer angle control before withdrawing the startup field. */
        int32_t field_target = handover == 256 ? motion->weakening_integral_q6 / 64
                                              : motion->hfi_field_target_ma;
        field_target = clamp(field_target, -motion->current_limit_ma, motion->current_limit_ma);
        int32_t previous_d = motion->hfi_field_ma;
        unsigned speed_frames = motion->sample_ticks == 1 ? 40u : 5u;
        if (motion->hfi_speed_divider == 0 ||
            motion->hfi_speed_divider == speed_frames / 2u) {
            /* Reserve Id movement after each outer-loop phase. Keep the
             * voltage, current-budget and speed calculations on separate
             * frames. On release reserve the larger actual negative Id. */
            int32_t reserved_d = field_target < previous_d ? field_target : previous_d;
            motion->weakening_q_limit_ma = reserved_d < 0 ? (uint16_t)FOC_SqrtLowerBound(
                (uint32_t)(motion->current_limit_ma * motion->current_limit_ma -
                           reserved_d * reserved_d)) : (uint16_t)motion->current_limit_ma;
        }
        int32_t down_step = 1;
        if (previous_d > 0 && field_target <= 0) {
            unsigned ticks = motion->id_slew_fraction + motion->sample_ticks;
            down_step = (int32_t)(ticks / HFI_FRAME_TICKS);
            motion->id_slew_fraction = (uint8_t)(ticks & (HFI_FRAME_TICKS - 1u));
        }
        motion->hfi_field_ma += clamp(field_target - motion->hfi_field_ma, -down_step, 4);
        if (motion->hfi_field_ma >= 0) {
            motion->current_q_limit_ma = motion->hfi_field_ma == 0 && field_target == 0
                ? (uint16_t)motion->current_limit_ma : motion->hfi_q_limit_ma;
        } else {
            motion->current_q_limit_ma = motion->weakening_q_limit_ma;
        }
        if (motion->hfi_speed_divider == speed_frames / 2u - 1u) {
            weakening_step(motion, speed);
        }
        if (++motion->hfi_speed_divider >= speed_frames) {
            motion->hfi_speed_divider = 0;
            if (motion->sensorless_regulate) {
                int32_t speed_error = motion->profile_velocity - speed;
                bool voltage_limited =
                    (motion->voltage_saturation > 0 && speed_error > 0) ||
                    (motion->voltage_saturation < 0 && speed_error < 0);
                motion->hfi_iq_target = pi_step(
                    &motion->sensorless_speed, speed_error,
                    motion->current_q_limit_ma, voltage_limited ? 0 : 40, 0);
            }
        }
        unsigned slew_ticks = motion->iq_slew_fraction + motion->sample_ticks;
        int32_t slew = (int32_t)(slew_ticks / 2u);
        motion->iq_slew_fraction = (uint8_t)(slew_ticks & 1u);
        motion->iq_ref += clamp(motion->hfi_iq_target - motion->iq_ref, -slew, slew);
        motion->iq_ref = clamp(motion->iq_ref, -motion->current_q_limit_ma, motion->current_q_limit_ma);
    } else if (motion->request.mode == MOTOR_RATCHET) {
        motion->iq_ref = ratchet_current(motion, position, speed);
    } else if (++motion->outer_count == OUTER_MS) {
        motion->outer_count = 0;
        update_outer(motion, position, speed);
    }
    int32_t period = motion->request.mode == MOTOR_HFI ? motion->sample_ticks : MOTOR_CONTROL_US * PI_TICKS_PER_MS / 1000;
    int32_t id_ref = motion->request.mode == MOTOR_HFI ? motion->hfi_field_ma : 0;
    int32_t vd = pi_step(&motion->loops[LOOP_D], id_ref - current.d, voltage_limit,
                         period, per_thousand(Q_AXIS_RESISTANCE_MOHM * id_ref));
    int32_t feedforward_mv = per_thousand(Q_AXIS_RESISTANCE_MOHM * motion->iq_ref);
    /* Cancel the estimated back EMF in current mode. Ratchet and speed modes
       retain the motor's electrical damping. */
    if (motion->request.mode == MOTOR_CURRENT) {
        feedforward_mv += Q_AXIS_BACK_EMF_MV_PER_RAD_S * speed / 1000;
    }
    uint32_t voltage_remaining = (uint32_t)(voltage_limit * voltage_limit - vd * vd);
    int32_t q_limit = vd == 0 ? voltage_limit : (int32_t)(motion->request.mode == MOTOR_HFI
        ? FOC_SqrtLowerBound(voltage_remaining) : FOC_Sqrt(voltage_remaining));
    int32_t vq = pi_step(&motion->loops[LOOP_Q], motion->iq_ref - current.q,
                         q_limit, period, feedforward_mv);
    motion->voltage_saturation = magnitude(vq) >= q_limit
        ? (vq < 0 ? -1 : 1) : 0;
    motion->previous_vd_mv = (int16_t)vd;
    motion->previous_vq_mv = (int16_t)vq;
    return (Dq){vd, vq};
}
