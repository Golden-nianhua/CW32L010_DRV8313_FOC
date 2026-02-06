//
// Created by Copilot on 2026/2/6.
//

#ifndef CW32L010_DRV8313_FOC_CURRENT_H
#define CW32L010_DRV8313_FOC_CURRENT_H

#include "base_types.h"

// ADC 读取回调函数类型（返回两个通道的ADC原始值：B相和C相）
typedef void (*ADC_ReadCallback)(uint16_t *adc_value_b, uint16_t *adc_value_c);

// 电流采样结构体
typedef struct {
    float current_a;        // A相电流 (A，通过Ia+Ib+Ic=0计算)
    float current_b;        // B相电流 (A，ADC测量)
    float current_c;        // C相电流 (A，ADC测量)

    float voltage_b;        // B相电压 (V，ADC测量)
    float voltage_c;        // C相电压 (V，ADC测量)

    uint16_t adc_raw[2];    // ADC原始值 [0]=B相, [1]=C相

    float shunt_resistor;   // 采样电阻值 (Ω)
    float amplifier_gain;   // 放大器增益
    float vref;             // ADC参考电压 (V)
    uint16_t adc_resolution;// ADC分辨率 (bits)

    // 零点校准偏移量（电压值）
    float voltage_offset_b; // B相电压零点偏移
    float voltage_offset_c; // C相电压零点偏移

    // ADC 读取回调函数
    ADC_ReadCallback adc_read;
} CurrentSensor;

// 初始化电流传感器
void CurrentSensor_Init(CurrentSensor *sensor,
                       float shunt_resistor,
                       float amplifier_gain,
                       float vref,
                       uint16_t adc_resolution,
                       ADC_ReadCallback adc_read_cb);

// 更新电流采样（从ADC读取并计算）
void CurrentSensor_Update(CurrentSensor *sensor);

// 零点校准（均值校准，采样 num_samples 次取平均值）
void CurrentSensor_Calibrate(CurrentSensor *sensor, uint16_t num_samples);

// 获取A相电流（通过Ia+Ib+Ic=0计算）
float CurrentSensor_GetCurrentA(const CurrentSensor *sensor);

// 获取B相电流（ADC测量）
float CurrentSensor_GetCurrentB(const CurrentSensor *sensor);

// 获取C相电流（ADC测量）
float CurrentSensor_GetCurrentC(const CurrentSensor *sensor);

#endif //CW32L010_DRV8313_FOC_CURRENT_H
