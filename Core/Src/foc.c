#include "foc.h"
#include "math.h"
#include "cw32l010_systick.h"

// 归一化角度到[0, 2π]
float FOC_NormalizeAngle(float angle) {
    float a = fmodf(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}

// 获取电角度
float FOC_GetElectricalAngle(const FOCMotor *motor) {
    return FOC_NormalizeAngle((float)(motor->DIR * motor->PP) * PositionSensor_GetMechanicalAngle(motor->sensor) - motor->zero_electric_angle);
}

// 获取角度
float FOC_GetAngle(const FOCMotor *motor) {
    return (float)motor->DIR * PositionSensor_GetAngle(motor->sensor);
}

// 设置PWM占空比
void FOC_SetPWM(FOCMotor *motor, float Ua, float Ub, float Uc) {
    Ua = _constrain(Ua, 0.0f, motor->voltage_power_supply);
    Ub = _constrain(Ub, 0.0f, motor->voltage_power_supply);
    Uc = _constrain(Uc, 0.0f, motor->voltage_power_supply);

    float dc_a = _constrain(Ua / motor->voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / motor->voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / motor->voltage_power_supply, 0.0f, 1.0f);

    // 使用回调函数设置PWM
    if (motor->setPWM != NULL) {
        motor->setPWM(dc_a, dc_b, dc_c);
    }
}

/*// 设置力矩 SPWM
void FOC_SetTorque(FOCMotor *motor, float Uq, float angle_el) {
    Uq = _constrain(Uq, -(motor->voltage_power_supply) / 2, (motor->voltage_power_supply) / 2);
    angle_el = FOC_NormalizeAngle(angle_el);

    // Park逆变换（Uq -> Ualpha, Ubeta）
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);

    // Clark逆变换（Ualpha, Ubeta -> Ua, Ub, Uc）
    float Ua = Ualpha + motor->voltage_power_supply / 2;
    float Ub = (-Ualpha + sqrtf(3) * Ubeta) / 2 + motor->voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + motor->voltage_power_supply / 2;

    FOC_SetPWM(motor, Ua, Ub, Uc);
}*/

// 设置力矩 SVPWM
void FOC_SetTorque(FOCMotor *motor, float Uq, float angle_el) {
    if (Uq < 0) angle_el += PI;
    Uq = fabsf(Uq);

    angle_el = FOC_NormalizeAngle(angle_el + PI_2);
    int sector = (int)floorf(angle_el / PI_3) + 1;

    float T1 = SQRT3 * sinf((float)sector * PI_3 - angle_el) * Uq / motor->voltage_power_supply;
    float T2 = SQRT3 * sinf(angle_el - (float)(sector - 1) * PI_3) * Uq / motor->voltage_power_supply;
    float T0 = 1.0f - T1 - T2;

    float Ta, Tb, Tc;
    switch (sector) {
        case 1:
            Ta = T1 + T2 + T0 / 2;
            Tb = T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 2:
            Ta = T1 + T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T0 / 2;
            break;
        case 3:
            Ta = T0 / 2;
            Tb = T1 + T2 + T0 / 2;
            Tc = T2 + T0 / 2;
            break;
        case 4:
            Ta = T0 / 2;
            Tb = T1 + T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 5:
            Ta = T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T2 + T0 / 2;
            break;
        case 6:
            Ta = T1 + T2 + T0 / 2;
            Tb = T0 / 2;
            Tc = T1 + T0 / 2;
            break;
        default:
            Ta = 0;
            Tb = 0;
            Tc = 0;
    }

    float Ua = Ta * motor->voltage_power_supply;
    float Ub = Tb * motor->voltage_power_supply;
    float Uc = Tc * motor->voltage_power_supply;

    FOC_SetPWM(motor, Ua, Ub, Uc);
}

float FOC_CalCulateIQ(FOCMotor *motor, float angle_el) {
    float Ialpha = motor->current_sensor->current_a;
    float Ibeta = _1_SQRT3 * motor->current_sensor->current_a + _2_SQRT3 * motor->current_sensor->current_b;

    float ct = cosf(angle_el);
    float st = sinf(angle_el);

    // float Id = Ialpha * ct + Ibeta * st;
    float Iq = Ibeta * ct - Ialpha * st;
    return Iq;
}

// 传感器对齐
void FOC_AlignSensor(FOCMotor *motor) {
    // 使能驱动器
    if (motor->enable != NULL) {
        motor->enable();
    }

    // 施加固定力矩，让电机转到确定位置
    FOC_SetTorque(motor, 3, _3PI_2);
    SysTickDelay(1000);  // 等待1秒让电机稳定

    // 更新传感器并记录零点电角度
    PositionSensor_Update(motor->sensor);
    motor->zero_electric_angle = FOC_GetElectricalAngle(motor);

    // 释放力矩
    FOC_SetTorque(motor, 0, _3PI_2);

    // 可选：禁用驱动器
    // if (motor->disable != NULL) {
    //     motor->disable();
    // }
}

// 初始化FOC电机
void FOC_Init(FOCMotor *motor, PositionSensor *sensor, CurrentSensor *current_sensor,
              float power_supply, int pp, int dir,
              FOC_PWM_Callback pwm_cb, FOC_Enable_Callback enable_cb, FOC_Disable_Callback disable_cb) {
    motor->voltage_power_supply = power_supply;
    motor->zero_electric_angle = 0.0f;
    motor->PP = pp;
    motor->DIR = dir;
    motor->sensor = sensor;
    motor->current_sensor = current_sensor;

    // 初始化速度环PID (P=2, I=0, D=0, ramp=100000, limit=Vbus/2)
    PID_Init(&motor->pid_velocity, 0.005f, 0.0001f, 0.00001f, 0.0f, power_supply / 2.0f);

    // 初始化位置环PID (P=2, I=0, D=0, ramp=100000, limit=100 rad/s)
    PID_Init(&motor->pid_position, 0.133f, 0.0f, 0.0f, 100000.0f, 100.0f);

    // 初始化电流环PID - q轴 (P=1, I=0.5, D=0, ramp=1000, limit=Vbus/2)
    PID_Init(&motor->pid_current_q, 5.0f, 200.f, 0.0f, 100000.0f, power_supply);

    // 初始化电流环PID - d轴 (P=1, I=0.5, D=0, ramp=1000, limit=Vbus/2)
    PID_Init(&motor->pid_current_d, 5.0f, 200.f, 0.0f, 100000.0f, power_supply / 2.0f);

    // 初始化速度环低通滤波器 (Tf=0.01)
    LPF_Init(&motor->lpf_velocity, 0.01f);

    // 初始化电流环低通滤波器 (Tf=0.005，电流环响应需要更快)
    LPF_Init(&motor->lpf_current_q, 0.005f);
    LPF_Init(&motor->lpf_current_d, 0.005f);

    // 设置回调函数
    motor->setPWM = pwm_cb;
    motor->enable = enable_cb;
    motor->disable = disable_cb;

    // 自动传感器对齐（暂时禁用，避免栈溢出）
    // FOC_AlignSensor(motor);
    // 用户可以在main函数中手动调用: FOC_AlignSensor(&motor);
}

