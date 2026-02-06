#include "pid.h"

#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern volatile uint32_t microseconds;

// 初始化PID控制器
void PID_Init(PIDController *pid, float P, float I, float D, float ramp, float limit)
{
    pid->P = P;
    pid->I = I;
    pid->D = D;
    pid->output_ramp = ramp;  // PID控制器加速度限幅
    pid->limit = limit;       // PID控制器输出限幅
    pid->error_prev = 0.0f;
    pid->output_prev = 0.0f;
    pid->integral_prev = 0.0f;
    pid->timestamp_prev = microseconds;
}

// PID 控制器计算函数
float PID_Calculate(PIDController *pid, float error)
{
    // 计算两次循环中间的间隔时间
    uint32_t timestamp_now = microseconds;
    float Ts = (float)(timestamp_now - pid->timestamp_prev) * 1e-6f;
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // P环
    float proportional = pid->P * error;
    // Tustin 散点积分（I环）
    float integral = pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
    integral = _constrain(integral, -pid->limit, pid->limit);
    // D环（微分环节）
    float derivative = pid->D * (error - pid->error_prev) / Ts;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -pid->limit, pid->limit);

    if(pid->output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - pid->output_prev) / Ts;
        if (output_rate > pid->output_ramp)
            output = pid->output_prev + pid->output_ramp * Ts;
        else if (output_rate < -pid->output_ramp)
            output = pid->output_prev - pid->output_ramp * Ts;
    }
    // 保存值（为了下一次循环）
    pid->integral_prev = integral;
    pid->output_prev = output;
    pid->error_prev = error;
    pid->timestamp_prev = timestamp_now;
    return output;
}