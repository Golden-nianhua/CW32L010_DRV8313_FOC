#include "drv8313.h"

#include "cw32l010_atim.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"

#define PWM_RELOAD (DRV8313_PWM_MAX + 1u)

void DRV8313_Init(void) {
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_ATIM_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ATIM_InitTypeDef ATIM_InitStructure = {0};
    ATIM_OCInitTypeDef ATIM_OCInitStructure = {0};

    PB01_SETLOW();

    // A: PB0/CH1, B: PB2/CH2, C: PA0/CH4, enable: PB1.

    GPIO_InitStructure.Pins = GPIO_PIN_0;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pins = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOB, &GPIO_InitStructure);

    PB02_AFx_ATIMCH2();
    PA00_AFx_ATIMCH4();
    PB00_AFx_ATIMCH1();

    ATIM_InitStructure.BufferState = DISABLE;
    ATIM_InitStructure.CounterAlignedMode = ATIM_COUNT_ALIGN_MODE_CENTER_BOTH;
    ATIM_InitStructure.CounterDirection = ATIM_COUNTING_UP;
    ATIM_InitStructure.CounterOPMode = ATIM_OP_MODE_REPETITIVE;
    ATIM_InitStructure.Prescaler = 0;
    ATIM_InitStructure.ReloadValue = PWM_RELOAD;
    ATIM_InitStructure.RepetitionCounter = 0;
    ATIM_Init(&ATIM_InitStructure);

    ATIM_OCInitStructure.BufferState = ENABLE;
    ATIM_OCInitStructure.OCComplement = DISABLE;
    ATIM_OCInitStructure.OCFastMode = DISABLE;
    ATIM_OCInitStructure.OCMode = ATIM_OCMODE_PWM2;
    ATIM_OCInitStructure.OCPolarity = ATIM_OCPOLARITY_NONINVERT;

    ATIM_OC1Init(&ATIM_OCInitStructure);
    ATIM_OC2Init(&ATIM_OCInitStructure);
    ATIM_OC4Init(&ATIM_OCInitStructure);

    ATIM_SetCompare1(0);
    ATIM_SetCompare2(0);
    ATIM_SetCompare4(0);

    /* Feed the CH5 reference directly to the ADC for one trigger per PWM cycle. */
    ATIM_OCInitStructure.BufferState = DISABLE;
    ATIM_OC5Init(&ATIM_OCInitStructure);
    ATIM_SetCompare5(96u);
    ATIM_MasterModeTrgo2Config(ATIM_MASTER_OUTPUT_OC5REFC);

    ATIM_CH1Config(ENABLE);
    ATIM_CH2Config(ENABLE);
    ATIM_CH4Config(ENABLE);

    ATIM_CtrlPWMOutputs(ENABLE);
    ATIM_Cmd(ENABLE);
}

void DRV8313_SetEnabled(bool enabled) {
    if (enabled) {
        PB01_SETHIGH();
    } else {
        PB01_SETLOW();
    }
}

void DRV8313_SetHfiTiming(bool enabled) {
    uint32_t reload = enabled ? DRV8313_HFI_PWM_MAX + 1u : PWM_RELOAD;
    if (CW_ATIM->ARR == reload) return;
    uint32_t a = CW_ATIM->CCR1, b = CW_ATIM->CCR2, c = CW_ATIM->CCR4;
    /* Preserve normalized phase voltages across the 5:1 PWM frequency change. */
    if (enabled) {
        a = (a + 2u) / 5u;
        b = (b + 2u) / 5u;
        c = (c + 2u) / 5u;
    } else {
        a *= 5u;
        b *= 5u;
        c *= 5u;
    }
    ATIM_Cmd(DISABLE);
    ATIM_SetCounterValue(0);
    ATIM_SetReloadValue(reload);
    ATIM_SetCompare5(enabled ? 24u : 96u);
    DRV8313_SetDuty((uint16_t)a, (uint16_t)b, (uint16_t)c);
    /* Commit buffered compare values against the new period before restarting. */
    CW_ATIM->EGR = ATIM_EGR_UG_Msk;
    ATIM_Cmd(ENABLE);
}

__attribute__((section(".RamFunc"))) void DRV8313_SetDuty(uint16_t a, uint16_t b, uint16_t c) {
    CW_ATIM->CR1_f.UDIS = 1;
    CW_ATIM->CCR1 = a;
    CW_ATIM->CCR2 = b;
    CW_ATIM->CCR4 = c;
    CW_ATIM->CR1_f.UDIS = 0;
}
