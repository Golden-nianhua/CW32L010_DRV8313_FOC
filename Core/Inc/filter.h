//
// Created by Golde on 2026/2/5.
//

#ifndef CW32L010_DRV8313_FOC_FILTER_H
#define CW32L010_DRV8313_FOC_FILTER_H

#include <stdint.h>

typedef struct {
    float Tf;                // 时间常数
    float y_prev;            // 上一次的输出值
    uint32_t timestamp_prev; // 上一次的时间戳
} LowPassFilter;

// 初始化低通滤波器
void LPF_Init(LowPassFilter *lpf, float time_constant);

// 低通滤波器计算函数
float LPF_Calculate(LowPassFilter *lpf, float x);

#endif //CW32L010_DRV8313_FOC_FILTER_H