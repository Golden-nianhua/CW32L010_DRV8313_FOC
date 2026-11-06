#include "board_time.h"

#include "cw32l010_btim.h"
#include "cw32l010_flash.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_systick.h"

#define TIME_STEP_US 50u

static volatile uint32_t timestamp_us;

void BoardTime_InitClock(void) {
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    SYSCTRL_LSI_Enable();

    __SYSCTRL_FLASH_CLK_ENABLE();
    FLASH_SetLatency(FLASH_Latency_2);

    SYSCTRL_SysClk_Switch(SYSCTRL_SYSCLKSRC_HSI);
    SYSCTRL_HCLKPRS_Config(SYSCTRL_HCLK_DIV1);
    SYSCTRL_PCLKPRS_Config(SYSCTRL_PCLK_DIV1);
    SYSCTRL_SystemCoreClockUpdate(BOARD_CLOCK_HZ);
    InitTick(BOARD_CLOCK_HZ);
}

void BoardTime_Start(void) {
    __SYSCTRL_BTIM123_CLK_ENABLE();

    BTIM_TimeBaseInitTypeDef BTIM_TimeBaseInitStruct = {0};

    BTIM_TimeBaseInitStruct.BTIM_Mode = BTIM_MODE_TIMER;
    BTIM_TimeBaseInitStruct.BTIM_Period = BOARD_CLOCK_HZ / 1000000u - 1u;
    BTIM_TimeBaseInitStruct.BTIM_Prescaler = TIME_STEP_US - 1u;

    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_TimeBaseInitStruct);
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
    BTIM_Cmd(CW_BTIM1, ENABLE);

    __disable_irq();
    NVIC_EnableIRQ(BTIM1_IRQn);
    __enable_irq();
}

uint32_t BoardTime_NowUs(void) {
    return timestamp_us;
}

void BoardTime_DelayMs(uint32_t milliseconds) {
    SysTickDelay(milliseconds);
}

void BTIM1_IRQHandler(void) {
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE)) {
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
        timestamp_us += TIME_STEP_US;
    }
}
