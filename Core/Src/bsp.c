#include "bsp.h"
#include "base_types.h"
#include "cw32l010.h"
#include "cw32l010_gpio.h"
#include "cw32l010_uart.h"
#include "cw32l010_flash.h"
#include "cw32l010_i2c.h"
#include "cw32l010_iwdt.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_systick.h"
#include "cw32l010_adc.h"
#include "cw32l010_atim.h"
#include "cw32l010_btim.h"
#include "cw32l010_gtim.h"
#include "cw32l010_rtc.h"
#include "i2c_eeprom.h"
#include "system_cw32l010.h"

void rtc_Init(void) {
    RTC_InitTypeDef RTC_InitStruct = {0};
    RTC_AlarmTypeDef RTC_AlarmStruct = {0};

    RTC_InitStruct.DateStruct.Day = 0x21;             //设置日期，DAY、MONTH、YEAR必须为BCD方式，星期为0~6，代表星期日，星期一至星期六
    RTC_InitStruct.DateStruct.Month = RTC_Month_June;
    RTC_InitStruct.DateStruct.Week = RTC_Weekday_Monday;
    RTC_InitStruct.DateStruct.Year = 0x21;

    RTC_InitStruct.TimeStruct.Hour = 0x12;        //设置时间，HOUR、MINIUTE、SECOND必须为BCD方式，用户须保证HOUR、AMPM、H24之间的关联正确性
    RTC_InitStruct.TimeStruct.Minute = 0x59;
    RTC_InitStruct.TimeStruct.Second = 0x51;
    RTC_InitStruct.TimeStruct.AMPM = 1;
    RTC_InitStruct.TimeStruct.H24 = 0;

    RTC_InitStruct.RTC_ClockSource = RTC_RTCCLK_FROM_LSI;
    RTC_Init(&RTC_InitStruct);    //  用户需选定需要使用的时钟源

    RTC_SetInterval(RTC_INTERVAL_EVERY_0_5S);

    RTC_ITConfig(RTC_IT_INTERVAL, ENABLE);

    __disable_irq();
    NVIC_EnableIRQ(RTC_IRQn);
    __enable_irq();
}

void iwdt_Init(void) {

    __SYSCTRL_IWDT_CLK_ENABLE(); //使能IWDT模块

    IWDT_InitTypeDef IWDT_InitStruct = {0};

    IWDT_InitStruct.IWDT_ITState = ENABLE;
    IWDT_InitStruct.IWDT_OverFlowAction = IWDT_OVERFLOW_ACTION_INT;
    IWDT_InitStruct.IWDT_Pause = IWDT_SLEEP_PAUSE;
    IWDT_InitStruct.IWDT_Prescaler = IWDT_Prescaler_DIV512;    // IWDT使用LSI为计时时钟，分频后计时间隔为15.6ms
    IWDT_InitStruct.IWDT_ReloadValue = IWDT_2_SECS;  // 2s
    IWDT_InitStruct.IWDT_WindowValue = 0xFFF;
    IWDT_Init(&IWDT_InitStruct);

    IWDT_Cmd();
    while(!CW_IWDT->SR_f.RUN) {}
    IWDT_Refresh();

    __disable_irq();
    NVIC_EnableIRQ(WDT_IRQn);
    __enable_irq();
}

void rcc_Init(void) {
    // 配置HSI时钟为48MHz
    SYSCTRL_HSI_Enable(SYSCTRL_HSIOSC_DIV1);
    SYSCTRL_LSI_Enable( );

    //打开FLASH时钟 配置等待周期
    __SYSCTRL_FLASH_CLK_ENABLE();
    FLASH_SetLatency(FLASH_Latency_2);

    //切换系统时钟为HSI
    SYSCTRL_SysClk_Switch(SYSCTRL_SYSCLKSRC_HSI);
    SYSCTRL_HCLKPRS_Config(SYSCTRL_HCLK_DIV1);
    SYSCTRL_PCLKPRS_Config(SYSCTRL_PCLK_DIV1);

    SYSCTRL_SystemCoreClockUpdate(48000000);
    InitTick(48000000);
}

void uart_Init(void) {
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_UART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_WritePin(CW_GPIOA, GPIO_PIN_8, 1);

    GPIO_InitStructure.Pins = GPIO_PIN_8;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pins = GPIO_PIN_7;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);

    PA07_AFx_UART1RXD();
    PA08_AFx_UART1TXD();

    GPIO_SWD2GPIO();

    UART_InitTypeDef UART_InitStructure = {0};

    UART_InitStructure.UART_BaudRate = 115200;
    UART_InitStructure.UART_Over = UART_Over_16;
    UART_InitStructure.UART_Source = UART_Source_PCLK;
    UART_InitStructure.UART_UclkFreq = 48000000;
    UART_InitStructure.UART_StartBit = UART_StartBit_FE;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_HardwareFlowControl = UART_HardwareFlowControl_None;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
    UART_Init(CW_UART1, &UART_InitStructure);

    NVIC_SetPriority(UART1_IRQn, 0);
    NVIC_EnableIRQ(UART1_IRQn);

    UART_ITConfig(CW_UART1, UART_IT_RC, ENABLE);
    UART_ClearITPendingBit(CW_UART1, UART_IT_RC);
}

void i2c_Init(void) {
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_I2C_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    PA05_AFx_I2C1SDA();
    PA06_AFx_I2C1SCL();

    GPIO_InitStructure.Pins = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStruct = {0};

    I2C_InitStruct.PCLK_Freq = 48000000;
    I2C_InitStruct.I2C_SCL_Freq = 400000;
    I2C_InitStruct.I2C_SCL_Source = I2C_SCL_SRC_GPIO;
    I2C_InitStruct.I2C_SDA_Source = I2C_SDA_SRC_GPIO;

    I2C1_DeInit();
    I2C_Master_Init(CW_I2C1, &I2C_InitStruct);
    I2C_AcknowledgeConfig(CW_I2C1, ENABLE);
    I2C_Cmd(CW_I2C1, ENABLE);

    __disable_irq();
    NVIC_EnableIRQ(I2C1_IRQn);
    __enable_irq();
}

void key_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __SYSCTRL_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.IT = GPIO_IT_RISING | GPIO_IT_FALLING;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT_PULLUP;
    GPIO_InitStruct.Pins = GPIO_PIN_3;

    GPIO_Init(CW_GPIOA, &GPIO_InitStruct);

    GPIOA_INTFLAG_CLR(bv3);
    NVIC_EnableIRQ(GPIOA_IRQn);
}

void led_Init(void) {
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
    GTIM_InitStructure.Prescaler = 48 - 1;
    GTIM_InitStructure.PulseMode = GTIM_PULSE_MODE_DIS;
    GTIM_InitStructure.ReloadValue = 1000 -1;
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

    __disable_irq();
    NVIC_EnableIRQ(GTIM1_IRQn);
    __enable_irq();
}

void adc_Init(void) {
    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_ADC_CLK_ENABLE();

    PB04_ANALOG_ENABLE();
    PB05_ANALOG_ENABLE();

    ADC_InitTypeDef ADC_InitStructure = {0};

    ADC_InitStructure.ADC_ClkDiv = ADC_Clk_Div1;
    ADC_InitStructure.ADC_ConvertMode = ADC_ConvertMode_Once;
    ADC_InitStructure.ADC_IN0.ADC_InputChannel = ADC_InputCH11;
    ADC_InitStructure.ADC_IN0.ADC_SampTime = ADC_SampTime18Clk;
    ADC_InitStructure.ADC_IN1.ADC_InputChannel = ADC_InputCH12;
    ADC_InitStructure.ADC_IN1.ADC_SampTime = ADC_SampTime18Clk;
    ADC_InitStructure.ADC_SQREns = ADC_SqrEns0to1;

    ADC_Init(&ADC_InitStructure);

    ADC_ClearITPendingAll();
    ADC_ITConfig(ADC_IT_EOS, ENABLE);
    NVIC_EnableIRQ(ADC_IRQn);

    ADC_SoftwareStartConvCmd(DISABLE);

    ADC_Enable();
}

void atim_Init(void) {
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_GPIOB_CLK_ENABLE();
    __SYSCTRL_ATIM_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};
    ATIM_InitTypeDef ATIM_InitStructure = {0};
    ATIM_OCInitTypeDef ATIM_OCInitStructure = {0};

    PB01_SETLOW();

    // PB1 DRV_EN
    // PB2 DRV_IN1 ATIM_CH2
    // PA0 DRV_IN2 ATIM_CH4
    // PB2 DRV_IN3 ATIM_CH1

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
    ATIM_InitStructure.ReloadValue = 1200;
    ATIM_InitStructure.RepetitionCounter = 0;
    ATIM_Init(&ATIM_InitStructure);

    ATIM_OCInitStructure.BufferState = DISABLE;
    ATIM_OCInitStructure.OCComplement = DISABLE;
    ATIM_OCInitStructure.OCFastMode = DISABLE;
    ATIM_OCInitStructure.OCInterruptState = ENABLE;
    ATIM_OCInitStructure.OCMode = ATIM_OCMODE_PWM2;
    ATIM_OCInitStructure.OCPolarity = ATIM_OCPOLARITY_NONINVERT;

    ATIM_OC1Init(&ATIM_OCInitStructure);
    ATIM_OC2Init(&ATIM_OCInitStructure);
    ATIM_OC4Init(&ATIM_OCInitStructure);

    ATIM_SetCompare1(0);
    ATIM_SetCompare2(0);
    ATIM_SetCompare4(0);

    ATIM_CH1Config(ENABLE);
    ATIM_CH2Config(ENABLE);
    ATIM_CH4Config(ENABLE);

    ATIM_CtrlPWMOutputs(ENABLE);
    ATIM_Cmd(ENABLE);

    __disable_irq();
    NVIC_EnableIRQ(ATIM_IRQn);
    __enable_irq();
}

void btim1_Init(void) {
    __SYSCTRL_BTIM123_CLK_ENABLE();

    BTIM_TimeBaseInitTypeDef BTIM_TimeBaseInitStruct = {0};

    BTIM_TimeBaseInitStruct.BTIM_Mode = BTIM_MODE_TIMER;
    BTIM_TimeBaseInitStruct.BTIM_Period = 48-1;
    BTIM_TimeBaseInitStruct.BTIM_Prescaler = 9;    // 7+1=8分频

    BTIM_TimeBaseInit(CW_BTIM1, &BTIM_TimeBaseInitStruct);
    BTIM_ITConfig(CW_BTIM1, BTIM_IT_UPDATE, ENABLE);
    BTIM_Cmd(CW_BTIM1, ENABLE);

    __disable_irq();
    NVIC_EnableIRQ(BTIM1_IRQn);
    __enable_irq();
}


