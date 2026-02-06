//
// Created by Golde on 2026/2/4.
//

#ifndef CW32L010_DRV8313_FOC_FOC_H
#define CW32L010_DRV8313_FOC_FOC_H

#include "pid.h"
#include "filter.h"
#include "position_sensor.h"
#include "current.h"

#define PI 3.14159265359f
#define PI_2 1.57079632679f
#define PI_3 1.0471975512f
#define _3PI_2 4.71238898038f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

#define SQRT3 1.73205080757f
#define SQRT3_2 0.86602540378f

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

// 硬件抽象层回调函数类型
typedef void (*FOC_PWM_Callback)(float dc_a, float dc_b, float dc_c);
typedef void (*FOC_Enable_Callback)(void);
typedef void (*FOC_Disable_Callback)(void);

typedef struct {
    float voltage_power_supply;     // 电源电压
    float zero_electric_angle;      // 零电角度
    int PP;                         // 极对数
    int DIR;                        // 方向

    PositionSensor *sensor;         // 位置传感器
    CurrentSensor *current_sensor;// 电流传感器

    // PID控制器（内嵌，用户可直接修改参数）
    PIDController pid_velocity;     // 速度环PID
    PIDController pid_position;     // 位置环PID
    PIDController pid_current_q;    // 电流环PID (q轴)
    PIDController pid_current_d;    // 电流环PID (d轴)

    // 低通滤波器
    LowPassFilter lpf_velocity;     // 速度环低通滤波器
    LowPassFilter lpf_current_q;    // 电流环低通滤波器 (q轴)
    LowPassFilter lpf_current_d;    // 电流环低通滤波器 (d轴)

    // 硬件抽象层回调函数
    FOC_PWM_Callback setPWM;        // 设置PWM的回调函数
    FOC_Enable_Callback enable;     // 使能驱动器的回调函数
    FOC_Disable_Callback disable;   // 禁用驱动器的回调函数
} FOCMotor;

// 初始化FOC电机（PID控制器在内部初始化，用户可通过motor->pid_velocity和motor->pid_position修改参数）
void FOC_Init(FOCMotor *motor, PositionSensor *sensor, CurrentSensor *current_sensor,
              float power_supply, int pp, int dir,
              FOC_PWM_Callback pwm_cb, FOC_Enable_Callback enable_cb, FOC_Disable_Callback disable_cb);

// 归一化角度到[0, 2π]
float FOC_NormalizeAngle(float angle);

// 获取电角度
float FOC_GetElectricalAngle(const FOCMotor *motor);

// 获取角度
float FOC_GetAngle(const FOCMotor *motor);

// 设置PWM占空比
void FOC_SetPWM(FOCMotor *motor, float Ua, float Ub, float Uc);

// 设置力矩（Uq, Ud, 电角度）
void FOC_SetTorque(FOCMotor *motor, float Uq, float angle_el);

float FOC_CalCulateIQ(FOCMotor *motor, float angle_el);

// 传感器对齐
void FOC_AlignSensor(FOCMotor *motor);


// PID参数设置宏（直接调用PID_Init）
#define FOC_SET_VELOCITY_PID(motor, p, i, d, ramp, limit) \
    PID_Init(&(motor)->pid_velocity, p, i, d, ramp, limit)

#define FOC_SET_POSITION_PID(motor, p, i, d, ramp, limit) \
    PID_Init(&(motor)->pid_position, p, i, d, ramp, limit)

#define FOC_SET_CURRENT_Q_PID(motor, p, i, d, ramp, limit) \
    PID_Init(&(motor)->pid_current_q, p, i, d, ramp, limit)

#define FOC_SET_CURRENT_D_PID(motor, p, i, d, ramp, limit) \
    PID_Init(&(motor)->pid_current_d, p, i, d, ramp, limit)

// 低通滤波器参数设置宏
#define FOC_SET_VELOCITY_LPF(motor, tf) \
    LPF_Init(&(motor)->lpf_velocity, tf)

#define FOC_SET_CURRENT_Q_LPF(motor, tf) \
    LPF_Init(&(motor)->lpf_current_q, tf)

#define FOC_SET_CURRENT_D_LPF(motor, tf) \
    LPF_Init(&(motor)->lpf_current_d, tf)

// 获取滤波后的速度宏（使用内嵌滤波器）
#define FOC_GET_VELOCITY(motor) \
    LPF_Calculate(&(motor)->lpf_velocity, (motor)->DIR * PositionSensor_GetVelocity((motor)->sensor))

#define FOC_GET_IQ(motor) \
    LPF_Calculate(&(motor)->lpf_current_q, FOC_CalCulateIQ(motor, FOC_GetElectricalAngle(motor)))

// 力矩设置宏（自动获取电角度）
#define FOC_SET_TORQUE(motor, torque) \
    FOC_SetTorque(motor, torque, FOC_GetElectricalAngle(motor))

// 速度环控制宏（输入目标速度，直接控制电机）
#define FOC_VELOCITY_CONTROL(motor, target_vel) \
    FOC_SET_TORQUE(motor, \
        PID_Calculate(&(motor)->pid_velocity, \
            ((target_vel) - FOC_GET_VELOCITY(motor)) * 180 / PI))

// 力位控制宏（输入目标位置，直接控制电机 - 仅位置环PID）
#define FOC_FORCEANGLE_CONTROL(motor, target_angle) \
    FOC_SET_TORQUE(motor, \
        PID_Calculate(&(motor)->pid_position, \
            ((target_angle) - (motor)->DIR * PositionSensor_GetAngle((motor)->sensor)) * 180 / PI))

// 角度环控制宏（输入目标角度，直接控制电机 - 位置环+速度环级联）
#define FOC_ANGLE_CONTROL(motor, target_angle) \
    FOC_SET_TORQUE(motor, \
        PID_Calculate(&(motor)->pid_velocity, \
            PID_Calculate(&(motor)->pid_position, \
                ((target_angle) - (motor)->DIR * PositionSensor_GetAngle((motor)->sensor)) * 180 / PI) - \
            FOC_GET_VELOCITY(motor)))

#endif //CW32L010_DRV8313_FOC_FOC_H