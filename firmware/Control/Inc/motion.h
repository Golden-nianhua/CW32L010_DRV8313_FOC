#ifndef MOTION_H
#define MOTION_H

#include "foc.h"
#include <stdbool.h>

enum {
    MOTOR_CONTROL_US = 1000,
    MOTOR_CURRENT_LIMIT_MA = 500,
    MOTOR_CURRENT_MAX_MA = 2000,
    MOTOR_BUS_MV = 12000,
    /* Linear SVPWM phase-vector amplitude, Vbus / sqrt(3). */
    MOTOR_VOLTAGE_MAX_MV = 6928,
    MOTOR_VOLTAGE_LIMIT_MV = MOTOR_VOLTAGE_MAX_MV,
    MOTOR_HFI_FIELD_DEFAULT_MA = 400,
    MOTOR_HFI_FIELD_MAX_MA = 800,
    MOTOR_SPEED_LIMIT_MRAD_S = 30000,
    MOTOR_POSITION_LIMIT_MRAD = 1000000,
    MOTOR_ACCEL_LIMIT_MRAD_S2 = 50000,
    MOTOR_GAIN_LIMIT = 64000,
    MOTOR_RATCHET_MAX_TEETH = 120,
    MOTOR_RATCHET_MAX_CURRENT_MA = 500,
    MOTOR_RATCHET_MAX_DAMPING = 100,
    MOTOR_HFI_SPEED_LIMIT_MRAD_S = 1000000,
    MOTOR_HFI_START_SPEED_MRAD_S = 100000,
    MOTOR_HFI_ACCEL_MRAD_S2 = 100000,
    MOTOR_HFI_ACCEL_MAX_MRAD_S2 = 1000000,
};

typedef enum {
    MOTOR_VOLTAGE,
    MOTOR_CURRENT,
    MOTOR_SPEED,
    MOTOR_POSITION,
    MOTOR_IMPEDANCE,
    MOTOR_TRAJECTORY,
    MOTOR_RATCHET,
    MOTOR_HFI,
} MotorMode;

typedef enum {
    LOOP_D,
    LOOP_Q,
    LOOP_SPEED,
    LOOP_POSITION,
    LOOP_COUNT,
} MotorLoop;

typedef struct {
    int32_t kp;
    int32_t ki;
    int32_t integral;
} MotorPI;

typedef struct {
    MotorMode mode;
    int32_t target;
    int32_t option_a;
    int32_t option_b;
} MotorRequest;

typedef struct {
    MotorRequest request;
    MotorPI loops[LOOP_COUNT];
    int32_t hfi_reference_q8;
    int32_t hfi_field_target_ma;
    int32_t hfi_field_ma;
    int32_t hfi_iq_target;
    int32_t current_limit_ma;
    int32_t voltage_limit_mv;
    int32_t iq_ref;
    int32_t previous_speed;
    int32_t profile_position;
    int32_t profile_velocity;
    int32_t profile_fraction;
    int32_t ratchet_origin;
    int32_t ratchet_tooth;
    uint8_t outer_count;
    uint8_t hfi_speed_divider;
    uint8_t ratchet_boost;
    bool sensorless_regulate;
    uint16_t sensorless_blend_q8;
    MotorPI sensorless_speed;
    int32_t hfi_velocity_q8;
    int32_t voltage_saturation;
    uint8_t sample_ticks;
    uint8_t iq_slew_fraction;
    uint16_t injection_headroom_mv;
    int32_t hfi_acceleration_mrad_s2;
} Motion;

void Motion_Init(Motion *motion);
void Motion_Reset(Motion *motion, int32_t position, int32_t speed);
void Motion_SetLimits(Motion *motion, int32_t current_ma, int32_t voltage_mv);
void Motion_SetHfiFieldCurrent(Motion *motion, int32_t current_ma);
void Motion_HfiStart(Motion *motion, uint16_t rotor_phase);
/* Sensorless operation uses 200 us or 1600 us control periods. */
void Motion_SetSamplePeriod(Motion *motion, uint16_t sample_us);
uint16_t Motion_HfiAdvance(Motion *motion);
void Motion_SetSensorless(Motion *motion, bool regulate, uint16_t blend_q8);
Dq Motion_Step(Motion *motion, int32_t position, int32_t speed, Dq current);

#endif
