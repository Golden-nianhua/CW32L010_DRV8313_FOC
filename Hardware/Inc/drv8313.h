//
// Created by Golde on 2025/12/22.
//

#ifndef CW32L010_DRV8313_FOC_DRV8313_H
#define CW32L010_DRV8313_FOC_DRV8313_H

#include "cw32l010_gpio.h"
#include "cw32l010_atim.h"

#define DRV8313_ENABLE()     PB01_SETHIGH()
#define DRV8313_DISABLE()    PB01_SETLOW()

#define DRV8313_UA_PWM(dc)   ATIM_SetCompare1((uint16_t)(dc * 1199));
#define DRV8313_UB_PWM(dc)   ATIM_SetCompare2((uint16_t)(dc * 1199));
#define DRV8313_UC_PWM(dc)   ATIM_SetCompare4((uint16_t)(dc * 1199));

#endif //CW32L010_DRV8313_FOC_DRV8313_H