#include "board_time.h"
#include "bsp.h"
#include "bus_sense.h"
#include "console.h"
#include "current_adc.h"
#include "drv8313.h"
#include "hfi.h"
#include "motion.h"
#include "mt6701.h"
#include "smo.h"
#include "sensorless.h"


enum {
    ELECTRICAL_TURN = 16384,
    POLE_PAIRS = 7,
    ENCODER_DIRECTION = -1,
    CURRENT_ZERO_SAMPLES = 100,
    DEFAULT_ALIGN_MV = 600,
    ALIGN_US = 500000,
    TELEMETRY_US = 40000,
    PULSE_TIMEOUT_US = 300000,
    HFI_PRE_ALIGN_US = 250000,
    HFI_LOCK_TIMEOUT_US = 1000000,
    HFI_FRAME_TIMEOUT_US = 12000,
    HFI_VOLTAGE_DEFAULT_MV = 200,
    HFI_VOLTAGE_MIN_MV = 50,
    HFI_VOLTAGE_MAX_MV = 500,
    HFI_CURRENT_TRIP_DEFAULT_MA = 2000,
    HFI_CURRENT_TRIP_MAX_MA = 2000,
    HFI_ENCODER_DIVIDER = 20,
    SMO_FAST_ENTER_MRAD_S = 40000,
    SMO_FAST_EXIT_MRAD_S = 20000,
    ENCODER_MASK = MT6701_RESOLUTION - 1u,
    TURN_MRAD = 6283,
};

typedef enum { UNALIGNED, CALIBRATING, ALIGNING, READY, RUNNING, PROBING, HFI_LOCKING } MotorState;
typedef enum { OK, BAD_ARGUMENT, BAD_STATE, UNKNOWN_COMMAND } CommandResult;
typedef enum { NO_EVENT, ENCODER_ERROR, SAMPLE_GAP, SAMPLE_TIMEOUT, CURRENT_HIGH, KEY_STOP, HFI_LOST, SMO_LOST } MotorEvent;

typedef struct {
    Motion motion;
    HfiObserver hfi;
    SmoObserver smo;
    SmoConfig smo_config;
    Sensorless sensorless;
    int32_t smo_voltage_alpha;
    int32_t smo_voltage_beta;
    CurrentHfiFrame hfi_frame;
    ConsoleStatus status;
    MotorState state;
    uint32_t adc_sequence;
    uint32_t encoder_sequence;
    uint32_t encoder_time;
    uint32_t sample_time;
    uint32_t alignment_start;
    uint32_t pulse_start;
    uint32_t hfi_frame_time;
    uint32_t hfi_missed_frames;
    int32_t pulse_position;
    uint32_t sum_b;
    uint32_t sum_c;
    uint16_t zero_b;
    uint16_t zero_c;
    uint16_t hfi_voltage_mv;
    uint16_t calibration_samples;
    uint16_t raw_angle;
    uint16_t electrical_zero;
    uint16_t align_mv;
    uint16_t voltage_cap_mv;
    uint16_t pulse_angle;
    uint16_t pulse_motion;
    uint16_t pending_b, pending_c;
    uint16_t trace_index;
    uint16_t trace_count;
    uint8_t trace_divider;
    uint8_t trace_skip;
    uint8_t pulse_periods;
    int32_t position_fraction;
    int32_t ratchet_speed;
    bool have_encoder;
    bool have_current_zero;
    bool hfi_starting;
    bool hfi_pre_align;
    bool smo_enabled;
    bool encoder_pending;
    bool bus_manual_override;
} MotorApp;

static void apply_voltage(Dq voltage, FocAngle angle) {
    PhaseDuty duty = FOC_Modulate(voltage, angle, FOC_GetBusVoltage(), DRV8313_PWM_MAX);
    DRV8313_SetDuty(duty.a, duty.b, duty.c);
}

static void stop(MotorApp *app) {
    if (app->state == PROBING || app->state == HFI_LOCKING ||
        (app->state == RUNNING && app->motion.request.mode == MOTOR_HFI)) CurrentAdc_CancelPulse();
    DRV8313_SetEnabled(false);
    app->hfi_starting = false;
    app->smo.ready = false;
    app->smo.ready_us = 0;
    app->smo.valid_frames = 0;
    app->sensorless = (Sensorless){0};
    apply_voltage((Dq){0}, FOC_Angle(0));
    if (app->state == RUNNING || app->state == PROBING || app->state == HFI_LOCKING) {
        app->state = READY;
    } else if (app->state != READY) {
        app->state = UNALIGNED;
    }
    Motion_Reset(&app->motion, app->status.position, app->status.speed);
    app->status.vd = 0;
    app->status.vq = 0;
}

/* Called in the main context. Future ADC supply measurements use this entry. */
static void set_bus_voltage(MotorApp *app, uint16_t bus_mv) {
    FOC_SetBusVoltage(bus_mv);
    CurrentAdc_SetBusVoltage(bus_mv);
    int32_t limit = FOC_GetVoltageLimit();
    if (app->voltage_cap_mv != 0 && app->voltage_cap_mv < limit) limit = app->voltage_cap_mv;
    /* Supply updates leave current budgets and current-slew state intact. */
    app->motion.voltage_limit_mv = limit;
}

static bool in_range(int32_t value, int32_t min, int32_t max) {
    return value >= min && value <= max;
}

static uint16_t electrical_angle(const MotorApp *app) {
    return (app->raw_angle * POLE_PAIRS - app->electrical_zero) & ENCODER_MASK;
}

static void reset_smo(MotorApp *app, uint16_t phase, int32_t speed_mrad_s,
                       int32_t sample_us) {
    Smo_Reset(&app->smo);
    app->smo_config.sample_us = sample_us;
    Smo_Prepare(&app->smo_config);
    app->smo_voltage_alpha = app->smo_voltage_beta = 0;
    /* Mechanical mrad/s to electrical phase counts/tick Q8, seven pole pairs. */
    int32_t velocity_q8 = (int32_t)((int64_t)speed_mrad_s * sample_us * 4673 / 1000000);
    Smo_Seed(&app->smo, phase, velocity_q8);
}

static void observe_smo(MotorApp *app, int32_t ia, int32_t ib) {
    if (!app->smo_enabled) return;
    SmoInput input = {
        .voltage_alpha_mv = app->smo_voltage_alpha,
        .voltage_beta_mv = app->smo_voltage_beta,
        .current_alpha_ma = ia,
        .current_beta_ma = (ia + 2 * ib) * 9459 / 16384,
    };
    Smo_Update(&app->smo, &app->smo_config, &input);
}

static void smo_voltage(MotorApp *app, Dq voltage, FocAngle angle) {
    app->smo_voltage_alpha =
        (voltage.d * angle.cos_q14 - voltage.q * angle.sin_q14) / 16384;
    app->smo_voltage_beta =
        (voltage.d * angle.sin_q14 + voltage.q * angle.cos_q14) / 16384;
}

static void pulse_duties(uint16_t duty[CURRENT_PULSE_VECTORS + 1][3], Dq voltage,
                         FocAngle angle, int32_t amplitude, bool continuous) {
    int32_t alpha = (voltage.d * angle.cos_q14 - voltage.q * angle.sin_q14) / 16384;
    int32_t beta = (voltage.d * angle.sin_q14 + voltage.q * angle.cos_q14) / 16384;
    /* +alpha, -alpha, +beta, -beta, base. */
    uint16_t bus_mv = FOC_GetBusVoltage();
    for (unsigned i = 0; i <= CURRENT_PULSE_VECTORS; ++i) {
        int32_t injection = i == CURRENT_PULSE_VECTORS ? 0 : amplitude;
        if (i & 1u) injection = -injection;
        Dq stationary = {.d = alpha + (i < 2 ? injection : 0), .q = beta + (i >= 2 ? injection : 0)};
        PhaseDuty pwm = FOC_ModulateStationary(stationary.d, stationary.q, bus_mv,
                                               continuous ? 61440 : DRV8313_PWM_MAX);
        duty[i][0] = pwm.a;
        duty[i][1] = pwm.b;
        duty[i][2] = pwm.c;
    }
}

static CommandResult start_pulse(MotorApp *app, const ConsoleCommand *command) {
    const int32_t *args = command->args;
    if (command->count != 3 || !in_range(args[1], 0, 4) || !in_range(args[0], 0, args[1] == 0 ? 2000 : 500)
                            || !in_range(args[2], 1, 64)) return BAD_ARGUMENT;
    if (app->state != READY || BSP_StopPressed() || app->trace_index != app->trace_count ||
        app->encoder_sequence != app->adc_sequence || !in_range(app->status.speed, -300, 300)) return BAD_STATE;
    uint16_t duty[CURRENT_PULSE_VECTORS + 1][3];
    pulse_duties(duty, (Dq){0}, (FocAngle){.cos_q14 = 16384}, args[0], false);
    app->pulse_start = BoardTime_NowUs();
    app->pulse_position = app->status.position;
    app->pulse_angle = electrical_angle(app);
    app->pulse_motion = 0;
    app->pulse_periods = (uint8_t)args[1];
    app->state = PROBING;
    CurrentAdc_StartPulse(duty, app->pulse_periods, (uint8_t)args[2], app->zero_b, app->zero_c);
    return OK;
}

static void poll_pulse(MotorApp *app, uint32_t now) {
    int32_t motion = app->status.position - app->pulse_position;
    if (motion < 0) motion = -motion;
    if (motion > app->pulse_motion) app->pulse_motion = motion > UINT16_MAX ? UINT16_MAX : (uint16_t)motion;
    if (now - app->pulse_start >= PULSE_TIMEOUT_US) CurrentAdc_CancelPulse();
    if (CurrentAdc_PulseActive()) return;

    uint16_t fault = CurrentAdc_PulseFault();
    if (fault == 1) app->status.event = CURRENT_HIGH;
    if (fault == 2) app->status.event = SAMPLE_TIMEOUT;
    const CurrentPulseResponse *results = CurrentAdc_PulseResponses();
    for (uint8_t i = 0; i < CURRENT_PULSE_VECTORS; ++i) {
        Console_Pulse(i, app->pulse_periods, fault, app->pulse_angle, electrical_angle(app), app->pulse_motion, &results[i]);
    }
    app->state = READY;
}

static CommandResult set_mode(MotorApp *app, const ConsoleCommand *command, MotorMode mode) {
    const int32_t limits[] = {
        [MOTOR_VOLTAGE] = app->motion.voltage_limit_mv,
        [MOTOR_CURRENT] = app->motion.current_limit_ma,
        [MOTOR_SPEED] = MOTOR_SPEED_LIMIT_MRAD_S,
        [MOTOR_POSITION] = MOTOR_POSITION_LIMIT_MRAD,
        [MOTOR_IMPEDANCE] = MOTOR_POSITION_LIMIT_MRAD,
        [MOTOR_TRAJECTORY] = MOTOR_POSITION_LIMIT_MRAD,
        [MOTOR_RATCHET] = MOTOR_RATCHET_MAX_TEETH,
        [MOTOR_HFI] = MOTOR_HFI_SPEED_LIMIT_MRAD_S,
    };
    const int32_t *args = command->args;
    int32_t limit = limits[mode];
    if (!in_range(args[0], -limit, limit)) return BAD_ARGUMENT;
    if (mode == MOTOR_IMPEDANCE) {
        if (command->count != 3 || !in_range(args[1], 0, MOTOR_GAIN_LIMIT)
                                || !in_range(args[2], 0, MOTOR_GAIN_LIMIT)) return BAD_ARGUMENT;
    } else if (mode == MOTOR_TRAJECTORY) {
        if (command->count != 3 || !in_range(args[1], 1, MOTOR_SPEED_LIMIT_MRAD_S)
                                || !in_range(args[2], 200, MOTOR_ACCEL_LIMIT_MRAD_S2)) return BAD_ARGUMENT;
    } else if (mode == MOTOR_RATCHET) {
        if (command->count != 3 || args[0] < 1 || !in_range(args[1], 1, MOTOR_RATCHET_MAX_CURRENT_MA)
                                || !in_range(args[2], 0, MOTOR_RATCHET_MAX_DAMPING)) return BAD_ARGUMENT;
    } else if (command->count != 1) {
        return BAD_ARGUMENT;
    }

    if (mode == MOTOR_HFI && app->state != READY && app->state != UNALIGNED &&
        app->motion.request.mode != MOTOR_HFI) return BAD_STATE;

    if (mode != app->motion.request.mode || (mode == MOTOR_RATCHET && args[0] != app->motion.request.target)) {
        Motion_Reset(&app->motion, app->status.position, app->status.speed);
    }
    app->motion.request = (MotorRequest){mode, args[0], args[1], args[2]};
    return OK;
}

static CommandResult command_result(MotorApp *app, const ConsoleCommand *command) {
    if (!command->valid) return BAD_ARGUMENT;
    if (app->state == PROBING && command->code != 'X' && command->code != 'Q') return BAD_STATE;
    if (app->motion.request.mode == MOTOR_HFI && app->state != READY && app->state != UNALIGNED &&
        command->code != 'X' && command->code != 'Q' && command->code != 'Y' &&
        command->code != 'A' && command->code != 'D' && command->code != 'W') return BAD_STATE;
    const int32_t *args = command->args;
    switch (command->code) {
        case 'V': return set_mode(app, command, MOTOR_VOLTAGE);
        case 'I': return set_mode(app, command, MOTOR_CURRENT);
        case 'S': return set_mode(app, command, MOTOR_SPEED);
        case 'P': return set_mode(app, command, MOTOR_POSITION);
        case 'Z': return set_mode(app, command, MOTOR_IMPEDANCE);
        case 'T': return set_mode(app, command, MOTOR_TRAJECTORY);
        case 'H': return set_mode(app, command, MOTOR_RATCHET);
        case 'J': return start_pulse(app, command);
        case 'Y': return set_mode(app, command, MOTOR_HFI);
        case 'A':
            if (command->count != 1 ||
                !in_range(args[0], 5000, MOTOR_HFI_ACCEL_MAX_MRAD_S2)) return BAD_ARGUMENT;
            app->motion.hfi_acceleration_mrad_s2 = args[0];
            return OK;
        case 'B':
            if (command->count != 1 || (args[0] != 0 && !in_range(args[0], MOTOR_BUS_MIN_MV, MOTOR_BUS_MAX_MV))) return BAD_ARGUMENT;
            if (app->state != READY && app->state != UNALIGNED) return BAD_STATE;
            app->bus_manual_override = args[0] != 0;
            set_bus_voltage(app, args[0] == 0 ? BSP_BusVoltage() : (uint16_t)args[0]);
            return OK;
        case 'W':
            if (command->count != 3 || !in_range(args[0], 0, MOTOR_CURRENT_MAX_MA) ||
                !in_range(args[1], 700, 990) || !in_range(args[2], 100, 10000)) return BAD_ARGUMENT;
            Motion_SetWeakening(&app->motion, (uint16_t)args[0], (uint16_t)args[1], (uint16_t)args[2]);
            return OK;
        case 'G':
            if (command->count != 3 || !in_range(args[0], 0, LOOP_COUNT)
                                    || !in_range(args[1], 0, MOTOR_GAIN_LIMIT)
                                    || !in_range(args[2], 0, MOTOR_GAIN_LIMIT)) return BAD_ARGUMENT;
            if (app->state != READY && app->state != UNALIGNED) return BAD_STATE;
            if (args[0] == LOOP_COUNT) {
                app->motion.sensorless_speed = (MotorPI){.kp = args[1], .ki = args[2]};
            } else {
                app->motion.loops[args[0]] = (MotorPI){.kp = args[1], .ki = args[2]};
            }
            return OK;
        case 'F':
            if (command->count != 3 || !in_range(args[0], HFI_VOLTAGE_MIN_MV, HFI_VOLTAGE_MAX_MV)
                                    || !in_range(args[1], 100, MOTOR_HFI_FIELD_MAX_MA)
                                    || !in_range(args[2], 400, HFI_CURRENT_TRIP_MAX_MA)
                                    || args[2] <= args[1]) return BAD_ARGUMENT;
            if ((app->state != READY && app->state != UNALIGNED) || BSP_StopPressed()) return BAD_STATE;
            app->hfi_voltage_mv = (uint16_t)args[0];
            Motion_SetHfiFieldCurrent(&app->motion, args[1]);
            CurrentAdc_SetHfiCurrentLimit((uint16_t)args[2]);
            return OK;
        case 'L': {
            int32_t voltage_limit = args[1] == 0 ? FOC_GetVoltageLimit() : args[1];
            if (command->count != 2 || !in_range(args[0], 100, MOTOR_CURRENT_MAX_MA) ||
                !in_range(voltage_limit, 500, FOC_GetVoltageLimit())) return BAD_ARGUMENT;
            if ((app->state != READY && app->state != UNALIGNED) || BSP_StopPressed()) return BAD_STATE;
            if (app->motion.request.mode == MOTOR_CURRENT &&
                !in_range(app->motion.request.target, -args[0], args[0])) return BAD_ARGUMENT;
            if (app->motion.request.mode == MOTOR_VOLTAGE &&
                !in_range(app->motion.request.target, -voltage_limit, voltage_limit)) return BAD_ARGUMENT;
            app->voltage_cap_mv = (uint16_t)args[1];
            Motion_SetLimits(&app->motion, args[0], voltage_limit);
            return OK;
        }
        case 'O':
            if (command->count != 2 || !in_range(args[0], 0, 11)) return BAD_ARGUMENT;
            if (app->state != READY && app->state != UNALIGNED) return BAD_STATE;
            switch (args[0]) {
                case 0:
                    if (!in_range(args[1], 0, 1)) return BAD_ARGUMENT;
                    app->smo_enabled = args[1] != 0;
                    break;
                case 1:
                    if (!in_range(args[1], 1000, 20000)) return BAD_ARGUMENT;
                    app->smo_config.resistance_mohm = args[1];
                    break;
                case 2:
                    if (!in_range(args[1], 100, 20000)) return BAD_ARGUMENT;
                    app->smo_config.inductance_uh = args[1];
                    break;
                case 3:
                    if (!in_range(args[1], 100, 12000)) return BAD_ARGUMENT;
                    app->smo_config.sliding_gain_mv = args[1];
                    break;
                case 4:
                    if (!in_range(args[1], 1, 2000)) return BAD_ARGUMENT;
                    app->smo_config.boundary_ma = args[1];
                    break;
                case 5:
                    if (!in_range(args[1], 1, 8)) return BAD_ARGUMENT;
                    app->smo_config.emf_filter_shift = args[1];
                    break;
                case 6:
                    if (!in_range(args[1], 1, 192)) return BAD_ARGUMENT;
                    app->smo_config.pll_kp_q8 = args[1];
                    break;
                case 7:
                    if (!in_range(args[1], 0, 32)) return BAD_ARGUMENT;
                    app->smo_config.pll_ki_q8 = args[1];
                    break;
                case 8:
                    if (!in_range(args[1], 50, app->smo_config.fallback_emf_mv)) return BAD_ARGUMENT;
                    app->smo_config.min_emf_mv = args[1];
                    break;
                case 9:
                    if (!in_range(args[1], app->smo_config.fallback_emf_mv + 1, 3000)) return BAD_ARGUMENT;
                    app->smo_config.takeover_emf_mv = args[1];
                    break;
                case 10:
                    if (!in_range(args[1], app->smo_config.min_emf_mv,
                                  app->smo_config.takeover_emf_mv - 1)) return BAD_ARGUMENT;
                    app->smo_config.fallback_emf_mv = args[1];
                    break;
                case 11:
                    if (!in_range(args[1], 10, 1000)) return BAD_ARGUMENT;
                    app->smo_config.qualify_us = args[1] * 1000;
                    break;
            }
            Smo_Prepare(&app->smo_config);
            return OK;
        case 'C':
            if (command->count > 1 ||
                (command->count == 1 && !in_range(args[0], 1, app->motion.voltage_limit_mv))) {
                return BAD_ARGUMENT;
            }
            if ((app->state != READY && app->state != UNALIGNED) || BSP_StopPressed()) return BAD_STATE;
            app->align_mv = command->count == 1 ? (uint16_t)args[0] : DEFAULT_ALIGN_MV;
            app->sum_b = app->sum_c = app->calibration_samples = 0;
            app->have_current_zero = false;
            app->state = CALIBRATING;
            return OK;
        case 'E':
            if (command->count != 0) return BAD_ARGUMENT;
            if (app->state == RUNNING) return OK;
            if (app->motion.request.mode == MOTOR_HFI) {
                if ((app->state != READY && app->state != UNALIGNED) || BSP_StopPressed()) return BAD_STATE;
                app->align_mv = 800;
                app->sum_b = app->sum_c = app->calibration_samples = 0;
                app->have_current_zero = false;
                app->hfi_starting = true;
                app->hfi_pre_align = true;
                app->state = CALIBRATING;
                return OK;
            }
            if (app->state != READY || BSP_StopPressed()) return BAD_STATE;
            Motion_Reset(&app->motion, app->status.position, app->status.speed);
            reset_smo(app, electrical_angle(app),
                       app->motion.request.mode == MOTOR_SPEED ? app->motion.request.target : app->status.speed,
                       MOTOR_CONTROL_US);
            DRV8313_SetEnabled(true);
            app->state = RUNNING;
            return OK;
        case 'X':
            if (command->count != 0) return BAD_ARGUMENT;
            stop(app);
            return OK;
        case 'Q':
            if (command->count != 0) return BAD_ARGUMENT;
            return OK;
        case 'R':
            if (command->count != 0) return BAD_ARGUMENT;
            app->status.event = NO_EVENT;
            app->status.missed_samples = 0;
            app->status.encoder_errors = 0;
            return OK;
        case 'D':
            if (command->count == 1 && args[0] == 0) {
                app->trace_count = app->trace_index;
                return OK;
            }
            if (command->count != 2 || !in_range(args[0], 1, 20000)
                                    || !in_range(args[1], 1, 10)) return BAD_ARGUMENT;
            app->trace_index = 0;
            app->trace_skip = 0;
            app->trace_count = (uint16_t)args[0];
            app->trace_divider = (uint8_t)args[1];
            return OK;
        default:
            return UNKNOWN_COMMAND;
    }
}

static void accept_encoder(MotorApp *app, uint16_t raw) {
    raw = (uint16_t)((uint32_t)(ENCODER_DIRECTION * (int32_t)raw) & ENCODER_MASK);
    uint32_t now = BoardTime_NowUs();

    if (!app->have_encoder) {
        int32_t initial = raw;
        if (initial > (int32_t)(MT6701_RESOLUTION * 4u / 5u)) initial -= (int32_t)MT6701_RESOLUTION;
        int32_t scaled = initial * TURN_MRAD;
        app->status.position = scaled / (int32_t)MT6701_RESOLUTION;
        app->position_fraction = scaled % (int32_t)MT6701_RESOLUTION;
        app->have_encoder = true;
    } else {
        int32_t delta = (int32_t)((raw - app->raw_angle + MT6701_RESOLUTION / 2u) & ENCODER_MASK)
            - (int32_t)(MT6701_RESOLUTION / 2u);
        int32_t scaled = delta * TURN_MRAD + app->position_fraction;
        int32_t position = (int32_t)((uint32_t)app->status.position +
            (uint32_t)(scaled / (int32_t)MT6701_RESOLUTION));
        int32_t position_limit = 2 * MOTOR_POSITION_LIMIT_MRAD;
        if (app->motion.request.mode != MOTOR_HFI &&
            !in_range(position, -position_limit, position_limit)) {
            position = position < 0 ? -position_limit : position_limit;
        }
        app->status.position = position;
        app->position_fraction = scaled % (int32_t)MT6701_RESOLUTION;

        uint32_t elapsed = now - app->encoder_time;
        /* At most half a mechanical turn. The unsigned numerator fits 32 bits. */
        uint32_t distance = (uint32_t)(delta < 0 ? -delta : delta) * 98172u >> 8;
        int32_t velocity = elapsed == 0 ? 0 : (int32_t)(distance * 1000u / elapsed);
        if (delta < 0) velocity = -velocity;
        app->status.speed += (velocity - app->status.speed) / 8;
        /* Haptic damping needs less delay than the speed controller and telemetry. */
        if (app->motion.request.mode == MOTOR_RATCHET) {
            app->ratchet_speed += (velocity - app->ratchet_speed) / 2;
        } else {
            app->ratchet_speed = app->status.speed;
        }
    }
    app->raw_angle = raw;
    app->encoder_sequence = app->adc_sequence;
    app->encoder_time = now;
}

static bool update_encoder(MotorApp *app) {
    uint16_t raw;
    if (!MT6701_ReadAngle(&raw)) {
        ++app->status.encoder_errors;
        if (app->status.event < ENCODER_ERROR) app->status.event = ENCODER_ERROR;
        return false;
    }
    accept_encoder(app, raw);
    return true;
}

static void start_hfi(MotorApp *app, uint32_t now) {
    uint16_t duty[CURRENT_PULSE_VECTORS + 1][3];
    pulse_duties(duty, (Dq){0}, (FocAngle){.cos_q14 = 16384}, app->hfi_voltage_mv, true);
    app->hfi = (HfiObserver){0};
    Sensorless_Start(&app->sensorless, 0);
    reset_smo(app, 0, app->motion.profile_velocity, 1600);
    app->hfi_frame.sequence = 0;
    app->hfi_missed_frames = 0;
    app->hfi_frame_time = app->pulse_start = now;
    app->status.vd = app->status.vq = 0;
    app->hfi_starting = false;
    app->state = HFI_LOCKING;
    CurrentAdc_StartHfi(duty, app->zero_b, app->zero_c);
}

static int32_t smo_speed(const MotorApp *app) {
    int32_t scaled = Smo_ElectricalSpeedQ8(&app->smo) * 107;
    return app->smo_config.sample_us == CURRENT_SMO_SAMPLE_US
        ? scaled / 100 : scaled / 800;
}

static void set_sensorless_period(MotorApp *app, bool fast, Dq voltage, FocAngle angle) {
    uint16_t duty[CURRENT_PULSE_VECTORS + 1][3];
    pulse_duties(duty, voltage, angle, 0, true);
    uint16_t period = fast ? CURRENT_SMO_SAMPLE_US : CURRENT_HFI_SAMPLE_US;
    /* Prepare the rare coefficient changes while the ADC still has the
     * longer frame, so the 64-bit setup divisions do not block a fast frame. */
    if (fast) {
        Smo_SetSamplePeriod(&app->smo, &app->smo_config, period, app->sensorless.phase);
        CurrentAdc_SetSmoMode(true, duty);
    } else {
        CurrentAdc_SetSmoMode(false, duty);
        Smo_SetSamplePeriod(&app->smo, &app->smo_config, period, app->sensorless.phase);
    }
    Motion_SetSamplePeriod(&app->motion, period);
    app->sensorless.speed_q8 = app->smo.speed_q8;
    app->sensorless.settling_us = 40000;
}

static void process_hfi(MotorApp *app, uint32_t now) {
    bool locking = app->state == HFI_LOCKING;
    if (!CurrentAdc_PulseActive() || now - app->hfi_frame_time > HFI_FRAME_TIMEOUT_US ||
        (locking && now - app->pulse_start > HFI_LOCK_TIMEOUT_US)) {
        app->status.event = CurrentAdc_PulseFault() == 1 ? CURRENT_HIGH : HFI_LOST;
        stop(app);
        return;
    }
    uint32_t previous_sequence = app->hfi_frame.sequence;
    if (!CurrentAdc_ReadHfi(&app->hfi_frame)) return;
    app->hfi_missed_frames += app->hfi_frame.sequence - previous_sequence - 1u;
    app->hfi_frame_time = now;
    bool locked = true;
    if (locking || (app->sensorless.source != CONTROL_TO_SMO &&
                    app->sensorless.source != CONTROL_SMO)) {
        locked = Hfi_Update(&app->hfi, app->hfi_frame.delta, locking);
    }
    if (locking) {
        if (locked) {
            Motion_HfiStart(&app->motion, Hfi_Angle(&app->hfi));
            Sensorless_Start(&app->sensorless, Hfi_Angle(&app->hfi));
            reset_smo(app, Hfi_Angle(&app->hfi), app->motion.profile_velocity, 1600);
            app->state = RUNNING;
        }
        return;
    }
    if (!locked) {
        app->status.event = HFI_LOST;
    }
    int32_t ib = ((int32_t)app->zero_b - app->hfi_frame.raw_b) * 6602 / 4096;
    int32_t ic = ((int32_t)app->zero_c - app->hfi_frame.raw_c) * 6602 / 4096;
    app->smo_voltage_alpha = app->hfi_frame.voltage_alpha_mv;
    app->smo_voltage_beta = app->hfi_frame.voltage_beta_mv;
    observe_smo(app, -ib - ic, ib);
    /* The current average belongs to the duty used during the completed frame. */
    uint16_t sample_phase = app->sensorless.phase;
    uint16_t field_phase = Motion_HfiAdvance(&app->motion);
    int32_t field_speed_q8 = app->motion.hfi_velocity_q8;
    ControlSource previous_source = app->sensorless.source;
    uint16_t phase = Sensorless_Step(&app->sensorless, &app->smo, &app->smo_config,
                                     field_phase, field_speed_q8, app->smo_enabled);
    if (app->sensorless.source == CONTROL_LOST) {
        app->status.event = SMO_LOST;
        stop(app);
        return;
    }
    bool closed = app->sensorless.source == CONTROL_TO_SMO ||
                  app->sensorless.source == CONTROL_SMO;
    if (closed) app->motion.hfi_reference_q8 = (int32_t)phase * 256;
    if (previous_source != app->sensorless.source &&
        app->sensorless.source == CONTROL_TO_HFI) {
        int32_t velocity_q8 = app->sensorless.speed_q8;
        if (app->smo_config.sample_us == CURRENT_SMO_SAMPLE_US) velocity_q8 *= 8;
        /* Resume the low-speed field from the actual trajectory, including
         * its direction, even when the requested target has already reversed. */
        app->motion.profile_velocity = velocity_q8 * 107 / 800;
        app->motion.hfi_reference_q8 = (int32_t)phase * 256;
        Hfi_ReleaseAngle(&app->hfi, phase, velocity_q8);
    } else if (previous_source != app->sensorless.source &&
               app->sensorless.source == CONTROL_TO_SMO) {
        Hfi_SetAngle(&app->hfi, phase);
    }
    int32_t speed = closed ? smo_speed(app)
                           : Hfi_Speed(&app->hfi);
    if (!closed && app->motion.profile_velocity != 0 &&
        (app->motion.profile_velocity < 0 ? -1 : 1) != app->smo.direction) {
        reset_smo(app, phase, app->motion.profile_velocity, 1600);
    }
    Motion_SetSensorless(&app->motion, closed,
                         app->sensorless.blend_q8);
    FocAngle angle = FOC_Angle(phase);
    Dq current = FOC_Park(-ib - ic, ib, FOC_Angle(sample_phase));
    app->motion.injection_headroom_mv =
        (uint16_t)(app->hfi_voltage_mv * (256 - app->sensorless.blend_q8) / 256);
    Dq voltage = Motion_Step(&app->motion, 0, speed, current);
    app->status.id = current.d;
    app->status.iq = current.q;
    app->status.vd = voltage.d;
    app->status.vq = voltage.q;
    bool fast = app->smo_config.sample_us == CURRENT_SMO_SAMPLE_US;
    int32_t absolute_speed = smo_speed(app);
    if (absolute_speed < 0) absolute_speed = -absolute_speed;
    int32_t reference_speed = app->motion.profile_velocity;
    if (reference_speed < 0) reference_speed = -reference_speed;
    bool want_fast = app->sensorless.source == CONTROL_SMO &&
        (fast ? absolute_speed > SMO_FAST_EXIT_MRAD_S
              : reference_speed >= SMO_FAST_ENTER_MRAD_S &&
                absolute_speed >= SMO_FAST_ENTER_MRAD_S * 3 / 4 &&
                (app->motion.hfi_field_ma <= 0 ||
                 reference_speed >= MOTOR_HFI_START_SPEED_MRAD_S));
    if (fast != want_fast) {
        set_sensorless_period(app, want_fast, voltage, angle);
    } else if (fast) {
        PhaseDuty duty = FOC_Modulate(voltage, angle, FOC_GetBusVoltage(), DRV8313_PWM_MAX);
        CurrentAdc_SetSmoDuty(duty.a, duty.b, duty.c);
    } else {
        uint16_t duty[CURRENT_PULSE_VECTORS + 1][3];
        int32_t injection = app->hfi_voltage_mv * (256 - app->sensorless.blend_q8) / 256;
        pulse_duties(duty, voltage, angle, injection, true);
        CurrentAdc_SetHfiDuty(duty);
    }
}

static void process_sample(MotorApp *app, uint16_t raw_b, uint16_t raw_c, uint32_t now) {
    bool hfi_active = app->motion.request.mode == MOTOR_HFI &&
        (app->state == HFI_LOCKING || app->state == RUNNING);
    uint32_t encoder_divider = app->smo_config.sample_us == CURRENT_SMO_SAMPLE_US
        ? 1u : HFI_ENCODER_DIVIDER;
    bool read_encoder = !hfi_active ||
        (app->trace_index != app->trace_count && app->trace_skip == 0) ||
        app->adc_sequence - app->encoder_sequence >= encoder_divider;
    bool encoder_ok = !read_encoder || update_encoder(app);
    if (app->state == CALIBRATING) {
        app->sum_b += raw_b;
        app->sum_c += raw_c;
        if (++app->calibration_samples == CURRENT_ZERO_SAMPLES) {
            app->zero_b = (uint16_t)((app->sum_b + CURRENT_ZERO_SAMPLES / 2) / CURRENT_ZERO_SAMPLES);
            app->zero_c = (uint16_t)((app->sum_c + CURRENT_ZERO_SAMPLES / 2) / CURRENT_ZERO_SAMPLES);
            app->have_current_zero = true;
            app->alignment_start = now;
            app->state = ALIGNING;
            app->status.vd = app->align_mv;
            apply_voltage((Dq){.d = app->align_mv}, FOC_Angle(app->hfi_starting ? 4096u : 0));
            DRV8313_SetEnabled(true);
        }
        return;
    }
    if (app->motion.request.mode == MOTOR_HFI && (app->state == HFI_LOCKING || app->state == RUNNING)) {
        return;
    }
    if (!app->have_current_zero || !encoder_ok) return;

    /* 3.3 V ADC, 50 V/V gain, 10 mOhm shunt. Board phase current sign
       follows the existing firmware's negated Iq convention. */
    int32_t ib = ((int32_t)app->zero_b - raw_b) * 6602 / 4096;
    int32_t ic = ((int32_t)app->zero_c - raw_c) * 6602 / 4096;
    int32_t ia = -ib - ic;
    int32_t high = 2 * app->motion.current_limit_ma;
    if (high > MOTOR_CURRENT_MAX_MA) high = MOTOR_CURRENT_MAX_MA;
    if (!in_range(ia, -high, high) || !in_range(ib, -high, high) || !in_range(ic, -high, high)) {
        app->status.event = CURRENT_HIGH;
    }

    FocAngle angle = FOC_Angle(electrical_angle(app));
    Dq current = FOC_Park(ia, ib, angle);
    app->status.id = current.d;
    app->status.iq = current.q;
    if (app->state == RUNNING) {
        observe_smo(app, ia, ib);
        int32_t speed = app->motion.request.mode == MOTOR_RATCHET ? app->ratchet_speed : app->status.speed;
        Dq voltage = Motion_Step(&app->motion, app->status.position, speed, current);
        app->status.vd = voltage.d;
        app->status.vq = voltage.q;
        apply_voltage(voltage, angle);
        smo_voltage(app, voltage, angle);
    }
}

static void report(MotorApp *app) {
    app->status.mode = app->motion.request.mode;
    app->status.control_source = app->state == RUNNING || app->state == HFI_LOCKING
        ? (app->motion.request.mode == MOTOR_HFI ? app->sensorless.source : CONTROL_ENCODER)
        : CONTROL_NONE;
    app->status.bus_source = app->bus_manual_override ? 1 : (BUS_SENSE_ADC_ENABLED ? 2 : 0);
    app->status.supply_raw = CurrentAdc_SupplyRaw();
    app->status.key_pressed = BSP_StopPressed();
    app->status.bus_mv = FOC_GetBusVoltage();
    app->status.voltage_limit_mv = (uint16_t)app->motion.voltage_limit_mv;
    app->status.state = app->state;
    app->status.target = app->motion.request.target;
    app->status.iq_ref = app->motion.iq_ref;
    app->status.profile = app->motion.request.mode == MOTOR_HFI
        ? (int32_t)app->sensorless.phase * TURN_MRAD / (int32_t)MT6701_RESOLUTION : app->motion.profile_position;
    if (app->motion.request.mode == MOTOR_HFI) {
        ConsoleStatus status = app->status;
        bool closed = app->sensorless.source == CONTROL_TO_SMO || app->sensorless.source == CONTROL_SMO;
        status.speed = app->state != RUNNING ? 0 : closed
            ? smo_speed(app) : Hfi_Speed(&app->hfi);
        Console_Report(&status);
    } else {
        Console_Report(&app->status);
    }
}

static void trace_sample(MotorApp *app, uint16_t raw_b, uint16_t raw_c) {
    if (app->trace_index == app->trace_count) return;
    if (app->trace_skip != 0) {
        --app->trace_skip;
        return;
    }
    app->trace_skip = app->trace_divider - 1u;
    uint32_t encoder_age = app->adc_sequence - app->encoder_sequence;
    bool valid = app->have_encoder && encoder_age <= HFI_ENCODER_DIVIDER;
    uint16_t raw = (uint16_t)((uint32_t)(ENCODER_DIRECTION * (int32_t)app->raw_angle) & ENCODER_MASK);
    bool hfi_active = app->motion.request.mode == MOTOR_HFI && app->state == RUNNING;
    int32_t ib = ((int32_t)app->zero_b - app->hfi_frame.raw_b) * 6602 / 4096;
    int32_t ic = ((int32_t)app->zero_c - app->hfi_frame.raw_c) * 6602 / 4096;
    Console_Trace(&(ConsoleTrace){
        .sequence = app->adc_sequence,
        .time_us = app->sample_time,
        .speed = app->status.speed,
        .hfi_speed = app->motion.request.mode == MOTOR_HFI && app->state == RUNNING
            ? Hfi_Speed(&app->hfi) : 0,
        .raw_angle = valid ? raw : UINT16_MAX,
        .electrical_angle = valid && (app->state == READY || app->state == RUNNING || app->state == HFI_LOCKING)
            ? electrical_angle(app) : UINT16_MAX,
        .hfi_angle = app->motion.request.mode == MOTOR_HFI && app->state == RUNNING
            ? Hfi_Angle(&app->hfi) : UINT16_MAX,
        .adc_b = raw_b,
        .adc_c = raw_c,
        .id = (int16_t)app->status.id,
        .iq = (int16_t)app->status.iq,
        .iq_ref = (int16_t)app->motion.iq_ref,
        .vd = (int16_t)app->status.vd,
        .vq = (int16_t)app->status.vq,
        .observer_flags = Smo_IsValid(&app->smo, &app->smo_config) |
                         (Smo_CanTakeOver(&app->smo) << 1),
        .control_source = app->motion.request.mode == MOTOR_HFI
            ? app->sensorless.source : CONTROL_ENCODER,
        .index = app->trace_index++,
        .smo_speed_q8 = Smo_ElectricalSpeedQ8(&app->smo),
        .smo_angle = app->state == RUNNING ? Smo_Angle(&app->smo) : UINT16_MAX,
        .smo_emf = Smo_EmfMagnitudeMv(&app->smo),
        .smo_error = (uint16_t)app->smo.phase_error,
        .hfi_sequence = app->hfi_frame.sequence,
        .hfi_missed_frames = app->hfi_missed_frames,
        .smo_sample_us = app->smo_config.sample_us,
        .control_angle = app->motion.request.mode == MOTOR_HFI
            ? app->sensorless.phase : electrical_angle(app),
        .injection_mv = app->motion.request.mode == MOTOR_HFI
            ? app->hfi_voltage_mv * (256 - app->sensorless.blend_q8) / 256 : 0,
        .observer_voltage_alpha = (int16_t)app->smo_voltage_alpha,
        .observer_voltage_beta = (int16_t)app->smo_voltage_beta,
        .observer_current_alpha = hfi_active ? (int16_t)(-ib - ic) : 0,
        .observer_current_beta = hfi_active ? (int16_t)((ib - ic) * 9459 / 16384) : 0,
        .id_ref = hfi_active ? (int16_t)app->motion.hfi_field_ma : 0,
        .iq_limit = app->motion.current_q_limit_ma,
        .weakening_target = (int16_t)(app->motion.weakening_integral_q6 / 64),
        .voltage_magnitude = app->motion.weakening_voltage_mv,
    });
}

int main(void) {
    BSP_Init();

    MotorApp app = {0};
    Motion_Init(&app.motion);
    set_bus_voltage(&app, MOTOR_BUS_MV);
    Smo_ConfigDefault(&app.smo_config);
    app.smo_config.sample_us = MOTOR_CONTROL_US;
    app.smo_config.emf_filter_shift = 1;
    Smo_Prepare(&app.smo_config);
    app.smo_enabled = true;
    app.hfi_voltage_mv = HFI_VOLTAGE_DEFAULT_MV;
    CurrentAdc_SetHfiCurrentLimit(HFI_CURRENT_TRIP_DEFAULT_MA);
    app.sample_time = BoardTime_NowUs();
    uint32_t last_report = app.sample_time;
    uint32_t previous_sample_period = MOTOR_CONTROL_US;
    unsigned sample_transition = 2;
    for (;;) {
        if (BSP_StopRequested()) {
            stop(&app);
            app.status.event = KEY_STOP;
        }

        uint16_t raw_b, raw_c;
        uint32_t conversion_time;
        /* Finish the in-flight I2C transaction before another diagnostic can
         * start a blocking read during a fast-to-slow rate transition. */
        uint32_t sequence = app.encoder_pending ? app.adc_sequence
            : CurrentAdc_Next(&raw_b, &raw_c, &conversion_time);
        uint32_t now = BoardTime_NowUs();
        if (app.motion.request.mode == MOTOR_HFI &&
            (app.state == HFI_LOCKING || app.state == RUNNING)) process_hfi(&app, now);
        if (app.state == ALIGNING && app.hfi_starting && app.hfi_pre_align &&
            now - app.alignment_start >= HFI_PRE_ALIGN_US) {
            app.hfi_pre_align = false;
            app.alignment_start = now;
            apply_voltage((Dq){.d = app.align_mv}, FOC_Angle(0));
        }
        if (app.state == ALIGNING && now - app.alignment_start >= ALIGN_US) {
            bool encoder_ok = update_encoder(&app);
            if (encoder_ok) {
                app.electrical_zero = (app.raw_angle * POLE_PAIRS) & ENCODER_MASK;
            }
            if (app.hfi_starting) {
                start_hfi(&app, now);
            } else {
                stop(&app);
                if (encoder_ok) app.state = READY;
            }
        }
        uint32_t gap = sequence - app.adc_sequence;
        uint32_t elapsed = now - app.sample_time;
        if (gap != 0u) {
            elapsed = conversion_time - app.sample_time;
            app.adc_sequence = sequence;
            app.sample_time = conversion_time;
            BSP_UpdateSupply();
            uint16_t sensed_bus = BSP_BusVoltage();
            if (!app.bus_manual_override && sensed_bus != FOC_GetBusVoltage()) {
                set_bus_voltage(&app, sensed_bus);
            }
            app.status.sample_us = elapsed / gap;
            uint32_t nominal_sample_us =
                app.motion.request.mode == MOTOR_HFI &&
                (app.state == HFI_LOCKING || app.state == RUNNING) ? 2000u : MOTOR_CONTROL_US;
            if (nominal_sample_us != previous_sample_period) {
                previous_sample_period = nominal_sample_us;
                sample_transition = 2;
            }
            if (gap > 1u) {
                app.status.missed_samples += gap - 1u;
                if (app.status.event < SAMPLE_GAP) app.status.event = SAMPLE_GAP;
            } else if (sample_transition == 0 &&
                       (elapsed < 3u * nominal_sample_us / 4u ||
                        elapsed > 5u * nominal_sample_us / 4u)) {
                if (app.status.event < SAMPLE_GAP) app.status.event = SAMPLE_GAP;
            }
            if (sample_transition != 0) --sample_transition;
            if (app.motion.request.mode == MOTOR_HFI && app.state == RUNNING &&
                app.smo_config.sample_us == CURRENT_SMO_SAMPLE_US) {
                app.pending_b = raw_b;
                app.pending_c = raw_c;
                MT6701_BeginRead();
                app.encoder_pending = true;
            } else {
                process_sample(&app, raw_b, raw_c, now);
                trace_sample(&app, raw_b, raw_c);
            }
        } else if (elapsed > 4u * previous_sample_period) {
            if (app.status.event < SAMPLE_TIMEOUT) app.status.event = SAMPLE_TIMEOUT;
            app.status.sample_us = 0;
            /* Diagnostic publication and I2C may lag while control frames
             * remain valid. process_hfi owns the control-frame watchdog. */
        }

        if (app.encoder_pending) {
            uint16_t raw;
            MT6701_ReadResult result = MT6701_PollAngle(&raw);
            if (result != MT6701_BUSY) {
                app.encoder_pending = false;
                if (result == MT6701_READY) accept_encoder(&app, raw);
                else {
                    ++app.status.encoder_errors;
                    if (app.status.event < ENCODER_ERROR) app.status.event = ENCODER_ERROR;
                }
                trace_sample(&app, app.pending_b, app.pending_c);
            }
        }

        if (app.state == PROBING) poll_pulse(&app, now);

        ConsoleCommand command;
        if (Console_Poll(&command)) {
            app.status.result = command_result(&app, &command);
            app.status.command = (uint8_t)command.code;
            ++app.status.ack;
            report(&app);
            last_report = now;
        }
        if (now - last_report >= TELEMETRY_US) {
            report(&app);
            last_report = now;
        }
    }
}
