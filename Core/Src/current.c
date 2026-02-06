#include "current.h"
#include "cw32l010_adc.h"


// 初始化电流传感器
void CurrentSensor_Init(CurrentSensor *sensor,
                       float shunt_resistor,
                       float amplifier_gain,
                       float vref,
                       uint16_t adc_resolution,
                       ADC_ReadCallback adc_read_cb) {
    sensor->current_a = 0.0f;
    sensor->current_b = 0.0f;
    sensor->current_c = 0.0f;
    sensor->voltage_b = 0.0f;
    sensor->voltage_c = 0.0f;
    sensor->adc_raw[0] = 0;
    sensor->adc_raw[1] = 0;

    sensor->shunt_resistor = shunt_resistor;
    sensor->amplifier_gain = amplifier_gain;
    sensor->vref = vref;
    sensor->adc_resolution = adc_resolution;

    // 初始化零点偏移量为 0
    sensor->voltage_offset_b = 0.0f;
    sensor->voltage_offset_c = 0.0f;

    sensor->adc_read = adc_read_cb;
}

// 更新电流采样（从ADC读取并计算）
void CurrentSensor_Update(CurrentSensor *sensor) {
    // 通过回调函数读取ADC原始值（解耦硬件层）
    // adc_raw[0] = B相, adc_raw[1] = C相
    if (sensor->adc_read != NULL) {
        sensor->adc_read(&sensor->adc_raw[0], &sensor->adc_raw[1]);
    }

    // 计算电压值 (V)
    sensor->voltage_b = (float)sensor->adc_raw[0] * sensor->vref / (float)((1 << sensor->adc_resolution) - 1);
    sensor->voltage_c = (float)sensor->adc_raw[1] * sensor->vref / (float)((1 << sensor->adc_resolution) - 1);

    // 计算电流值 (A)
    // 公式: I = (Vadc - Voffset - Vref/2) / (Gain * Rshunt)
    // INA240的输出为单端，中点为Vref/2
    // 减去零点偏移量进行校准
    float vref_half = sensor->vref / 2.0f;
    sensor->current_b = (sensor->voltage_b - sensor->voltage_offset_b - vref_half) / (sensor->amplifier_gain * sensor->shunt_resistor);
    sensor->current_c = (sensor->voltage_c - sensor->voltage_offset_c - vref_half) / (sensor->amplifier_gain * sensor->shunt_resistor);

    // 根据基尔霍夫电流定律: Ia + Ib + Ic = 0 => Ia = -(Ib + Ic)
    sensor->current_a = -(sensor->current_b + sensor->current_c);
}

// 获取A相电流
float CurrentSensor_GetCurrentA(const CurrentSensor *sensor) {
    return sensor->current_a;
}

// 获取B相电流
float CurrentSensor_GetCurrentB(const CurrentSensor *sensor) {
    return sensor->current_b;
}

// 获取C相电流
float CurrentSensor_GetCurrentC(const CurrentSensor *sensor) {
    return sensor->current_c;
}

// 零点校准（均值校准，采样 num_samples 次取平均值）
void CurrentSensor_Calibrate(CurrentSensor *sensor, uint16_t num_samples) {
    if (num_samples == 0) {
        num_samples = 100; // 默认采样100次
    }

    float sum_voltage_b = 0.0f;
    float sum_voltage_c = 0.0f;

    // 采样 num_samples 次
    for (uint16_t i = 0; i < num_samples; i++) {
        // 读取ADC原始值
        if (sensor->adc_read != NULL) {
            sensor->adc_read(&sensor->adc_raw[0], &sensor->adc_raw[1]);
        }

        // 计算电压值并累加
        float voltage_b = (float)sensor->adc_raw[0] * sensor->vref / (float)((1 << sensor->adc_resolution) - 1);
        float voltage_c = (float)sensor->adc_raw[1] * sensor->vref / (float)((1 << sensor->adc_resolution) - 1);

        sum_voltage_b += voltage_b;
        sum_voltage_c += voltage_c;

        // 添加小延时，避免采样过快（可选）
        // 这里可以根据需要添加延时函数
    }

    // 计算平均值
    float avg_voltage_b = sum_voltage_b / (float)num_samples;
    float avg_voltage_c = sum_voltage_c / (float)num_samples;

    // 零点偏移量 = 实际测量值 - 理论中点值
    // 理论中点值应该是 Vref/2
    float vref_half = sensor->vref / 2.0f;
    sensor->voltage_offset_b = avg_voltage_b - vref_half;
    sensor->voltage_offset_c = avg_voltage_c - vref_half;
}

