#include "mt6701.h"

// 获取原始角度值（14位，0-16383）
uint16_t MT6701_GetRawAngle(void) {
    uint8_t data[2] = {0};
    MT6701_ReadReg(MT6701_ADDR, MT6701_REG_ANGLE, data, 2);
    return ((data[0] << 6) | (data[1] >> 2));
}

