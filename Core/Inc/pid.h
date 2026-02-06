//
// Created by Golde on 2026/2/5.
//

#ifndef CW32L010_DRV8313_FOC_PID_H
#define CW32L010_DRV8313_FOC_PID_H

#include <stdint.h>

typedef struct {
    float P;              // 比例增益
    float I;              // 积分增益
    float D;              // 微分增益
    float output_ramp;    // PID控制器加速度限幅
    float limit;          // PID控制器输出限幅
    float error_prev;     // 上一次的误差
    float output_prev;    // 上一次的输出
    float integral_prev;  // 积分累积值
    uint32_t timestamp_prev; // 上一次的时间戳
} PIDController;

// 初始化PID控制器
void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit);

// PID控制器计算函数
float PID_Calculate(PIDController *pid, float error);

#endif //CW32L010_DRV8313_FOC_PID_H