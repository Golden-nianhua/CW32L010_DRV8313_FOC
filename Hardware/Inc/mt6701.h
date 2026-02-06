//
// Created by Golde on 2025/10/24.
//

#ifndef CW32L010_DRV8313_FOC_MT6701_H
#define CW32L010_DRV8313_FOC_MT6701_H

#include <stdint.h>
#include "i2c_eeprom.h"

#define MT6701_ADDR             0x06
#define MT6701_REG_ANGLE        0x03
#define MT6701_RESOLUTION       16384

#define MT6701_ReadReg(addr, reg, buf, len)  I2C_eepromReadBytes(CW_I2C1, addr << 1, reg,  buf, len);

// 获取原始角度值（14位，0-16383）
uint16_t MT6701_GetRawAngle(void);


#endif //CW32L010_DRV8313_FOC_MT6701_H