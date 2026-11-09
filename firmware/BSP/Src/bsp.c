#include "bsp.h"

#include "board_time.h"
#include "current_adc.h"
#include "bus_sense.h"
#include "drv8313.h"
#include "mt6701.h"
#include "cw32l010_gpio.h"
#include "cw32l010_gtim.h"
#include "cw32l010_sysctrl.h"

static volatile bool stop_requested;

static void init_key(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();

#if BUS_SENSE_ADC_ENABLED
    PA03_PUR_DISABLE();
    PA03_DIR_INPUT();
    PA03_ANALOG_ENABLE();
    return;
#endif
    GPIO_InitStruct.IT = GPIO_IT_RISING | GPIO_IT_FALLING;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_InitStruct.Pins = GPIO_PIN_3;

    GPIO_Init(CW_GPIOA, &GPIO_InitStruct);

    GPIOA_INTFLAG_CLR(bv3);
    NVIC_EnableIRQ(GPIOA_IRQn);
}

static void init_led(void) {
    GTIM_InitTypeDef GTIM_InitStructure = {0};
    GTIM_OCModeCfgTypeDef GTIM_OCModeCfgStructure = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GTIM1_CLK_ENABLE();

    PA04_DIGTAL_ENABLE();
    PA04_DIR_OUTPUT();
    PA04_PUSHPULL_ENABLE();
    PA04_AFx_GTIM1CH3();

    GTIM_InitStructure.AlignMode = GTIM_ALIGN_MODE_EDGE;
    GTIM_InitStructure.ARRBuffState = GTIM_ARR_BUFF_EN;
    GTIM_InitStructure.Direction = GTIM_DIRECTION_UP;
    GTIM_InitStructure.EventOption = GTIM_EVENT_NORMAL;
    GTIM_InitStructure.Prescaler = BOARD_CLOCK_HZ / 1000000u - 1u;
    GTIM_InitStructure.PulseMode = GTIM_PULSE_MODE_DIS;
    GTIM_InitStructure.ReloadValue = 1000 - 1;
    GTIM_InitStructure.UpdateOption = GTIM_UPDATE_DIS;
    GTIM_TimeBaseInit(CW_GTIM1, &GTIM_InitStructure);

    GTIM_OCModeCfgStructure.FastMode = DISABLE;
    GTIM_OCModeCfgStructure.OCMode = GTIM_OC_MODE_PWM2;
    GTIM_OCModeCfgStructure.OCPolarity = GTIM_OC_POLAR_NONINVERT;
    GTIM_OCModeCfgStructure.PreloadState = DISABLE;
    GTIM_OC3ModeCfg(CW_GTIM1, &GTIM_OCModeCfgStructure);

    GTIM_SetCompare3(CW_GTIM1, 0);
    GTIM_OC3Cmd(CW_GTIM1, DISABLE);
    GTIM_Cmd(CW_GTIM1, ENABLE);
}

static void set_led_level(unsigned percent) {
    GTIM_SetCompare3(CW_GTIM1, percent * 10u);
}

void BSP_Init(void) {
    BoardTime_InitClock();
    init_led();
    GTIM_OC3Cmd(CW_GTIM1, ENABLE);
    set_led_level(100u);
    BoardTime_DelayMs(250u);
    set_led_level(25u);
    BoardTime_DelayMs(250u);
    set_led_level(0u);

    init_key();
    MT6701_Init();
    CurrentAdc_Init();
    DRV8313_Init();
    BoardTime_Start();
}

void GPIOA_IRQHandler(void) {
    if (CW_GPIOA->ISR_f.PIN3) {
        GPIOA_INTFLAG_CLR(bv3);
        if (BSP_StopPressed()) {
            stop_requested = true;
        }
    }
}

void BSP_UpdateSupply(void) {
    BusSense_Update(CurrentAdc_SupplyRaw(), BUS_SENSE_ADC_ENABLED);
    if (BusSense_TakeKeyPress()) stop_requested = true;
}

uint16_t BSP_BusVoltage(void) { return BusSense_Voltage(); }

bool BSP_StopPressed(void) {
    bool pressed = BusSense_KeyPressed();
#if !BUS_SENSE_ADC_ENABLED
    pressed = pressed || (CW_GPIOA->IDR & GPIO_PIN_3) == 0u;
#endif
    return pressed;
}

bool BSP_StopRequested(void) {
    if (!stop_requested) return false;
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bool requested = stop_requested;
    stop_requested = false;
    __set_PRIMASK(mask);
    return requested;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    (void)file;
    (void)line;
}
#endif
