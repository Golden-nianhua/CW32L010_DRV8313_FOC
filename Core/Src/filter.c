#include "filter.h"

extern volatile uint32_t microseconds;

// 初始化低通滤波器
void LPF_Init(LowPassFilter *lpf, float time_constant)
{
    lpf->Tf = time_constant;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = microseconds;
}

// 低通滤波器计算函数
float LPF_Calculate(LowPassFilter *lpf, float x)
{
    uint32_t timestamp = microseconds;
    float dt = (float)(timestamp - lpf->timestamp_prev) * 1e-6f;

    if (dt < 0.0f) {
        dt = 1e-3f;
    } else if (dt > 0.3f) {
        lpf->y_prev = x;
        lpf->timestamp_prev = timestamp;
        return x;
    }

    float alpha = lpf->Tf / (lpf->Tf + dt);
    float y = alpha * lpf->y_prev + (1.0f - alpha) * x;
    lpf->y_prev = y;
    lpf->timestamp_prev = timestamp;
    return y;
}