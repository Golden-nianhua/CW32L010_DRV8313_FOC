//
// Created by Golde on 2026/2/5.
//

#ifndef CW32L010_DRV8313_FOC_POSITION_SENSOR_H
#define CW32L010_DRV8313_FOC_POSITION_SENSOR_H

#include <stdint.h>

#define PI                      3.14159265359f
#define _2PI                    6.28318530718f

// 位置传感器硬件抽象层回调函数类型
typedef uint16_t (*PositionSensor_GetRawAngle_Callback)(void);

// 位置传感器结构体
typedef struct {
    float angle_prev;           // 最后一次角度值
    uint32_t angle_prev_ts;     // 上次角度更新时间戳
    float vel_angle_prev;       // 最后一次速度计算时的角度
    uint32_t vel_angle_prev_ts; // 上次速度计算时间戳
    int32_t full_rotations;     // 总圈数计数
    int32_t vel_full_rotations; // 用于速度计算的圈数

    uint16_t resolution;        // 传感器分辨率（如MT6701为16384）

    // 硬件抽象层回调函数
    PositionSensor_GetRawAngle_Callback getRawAngle;
} PositionSensor;

// 初始化位置传感器
void PositionSensor_Init(PositionSensor *sensor, uint16_t resolution,
                         PositionSensor_GetRawAngle_Callback getRawAngle_cb);

// 更新传感器数据
void PositionSensor_Update(PositionSensor *sensor);

// 获取机械角度（单圈，0-2π）
float PositionSensor_GetMechanicalAngle(PositionSensor *sensor);

// 获取累计角度（多圈）
float PositionSensor_GetAngle(PositionSensor *sensor);

// 获取角速度（rad/s）
float PositionSensor_GetVelocity(PositionSensor *sensor);

#endif //CW32L010_DRV8313_FOC_POSITION_SENSOR_H
