#include "position_sensor.h"

#include <stddef.h>

extern volatile uint32_t microseconds;

// 初始化位置传感器
void PositionSensor_Init(PositionSensor *sensor, uint16_t resolution,
                         PositionSensor_GetRawAngle_Callback getRawAngle_cb) {
    sensor->angle_prev = 0.0f;
    sensor->angle_prev_ts = 0;
    sensor->vel_angle_prev = 0.0f;
    sensor->vel_angle_prev_ts = 0;
    sensor->full_rotations = 0;
    sensor->vel_full_rotations = 0;
    sensor->resolution = resolution;
    sensor->getRawAngle = getRawAngle_cb;
}

// 更新传感器数据
void PositionSensor_Update(PositionSensor *sensor) {
    if (sensor->getRawAngle == NULL) return;

    // 读取原始角度并转换为弧度
    float val = (float)sensor->getRawAngle() * _2PI / (float)sensor->resolution;
    sensor->angle_prev_ts = microseconds;

    // 计算角度变化
    float d_angle = val - sensor->angle_prev;

    // 检测圈数变化（阈值为0.8*2π）
    if (d_angle < (-0.8f * _2PI)) {
        sensor->full_rotations += 1;
    } else if (d_angle > (0.8f * _2PI)) {
        sensor->full_rotations -= 1;
    }

    sensor->angle_prev = val;
}

// 获取机械角度（单圈，0-2π）
float PositionSensor_GetMechanicalAngle(PositionSensor *sensor) {
    return sensor->angle_prev;
}

// 获取累计角度（多圈）
float PositionSensor_GetAngle(PositionSensor *sensor) {
    return (float)sensor->full_rotations * _2PI + sensor->angle_prev;
}

// 获取角速度（rad/s）
float PositionSensor_GetVelocity(PositionSensor *sensor) {
    // 计算时间差（秒）
    float Ts = (float)(sensor->angle_prev_ts - sensor->vel_angle_prev_ts) * 1e-6f;
    if (Ts <= 0.0f) Ts = 1e-3f;

    // 计算角度差
    float angle_diff = (float)(sensor->full_rotations - sensor->vel_full_rotations) * _2PI
                       + (sensor->angle_prev - sensor->vel_angle_prev);

    // 计算角速度
    float vel = angle_diff / Ts;

    // 更新速度计算历史
    sensor->vel_angle_prev = sensor->angle_prev;
    sensor->vel_angle_prev_ts = sensor->angle_prev_ts;
    sensor->vel_full_rotations = sensor->full_rotations;

    return vel;
}
