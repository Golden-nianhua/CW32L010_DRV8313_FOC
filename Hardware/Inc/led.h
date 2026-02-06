//
// Created by Golde on 2025/12/22.
//

#ifndef CW32L010_DRV8313_FOC_LED_H
#define CW32L010_DRV8313_FOC_LED_H

#include "cw32l010_gpio.h"
#include "cw32l010_gtim.h"

#define LED_ON()         GTIM_OC3Cmd(CW_GTIM1, ENABLE);
#define LED_OFF()        GTIM_OC3Cmd(CW_GTIM1, DISABLE);
#define LED_SET(x)       GTIM_SetCompare3(CW_GTIM1, (x * 10U));


#endif //CW32L010_DRV8313_FOC_LED_H