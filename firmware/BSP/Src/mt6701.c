#include "mt6701.h"

#include "board_time.h"
#include "cw32l010_gpio.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_i2c.h"

#define MT6701_ADDR 0x06u
#define MT6701_REG_ANGLE 0x03u
static uint32_t read_started_us;
static uint8_t read_step, read_high;

void MT6701_Init(void) {
    __SYSCTRL_GPIOA_CLK_ENABLE();
    __SYSCTRL_I2C_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure = {0};

    PA05_AFx_I2C1SDA();
    PA06_AFx_I2C1SCL();

    GPIO_InitStructure.Pins = GPIO_PIN_5 | GPIO_PIN_6;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_Init(CW_GPIOA, &GPIO_InitStructure);

    I2C_InitTypeDef I2C_InitStruct = {0};

    I2C_InitStruct.PCLK_Freq = BOARD_CLOCK_HZ;
    I2C_InitStruct.I2C_SCL_Freq = 1000000;
    I2C_InitStruct.I2C_SCL_Source = I2C_SCL_SRC_GPIO;
    I2C_InitStruct.I2C_SDA_Source = I2C_SDA_SRC_GPIO;

    I2C1_DeInit();
    I2C_Master_Init(CW_I2C1, &I2C_InitStruct);
    I2C_AcknowledgeConfig(CW_I2C1, ENABLE);
    I2C_Cmd(CW_I2C1, ENABLE);
}

void MT6701_BeginRead(void) {
    read_started_us = BoardTime_NowUs();
    read_step = 0;
    /* At idle, clear SI before START so the new START event cannot be lost. */
    I2C_ClearIrq(CW_I2C1);
    I2C_GenerateSTART(CW_I2C1, ENABLE);
}

MT6701_ReadResult MT6701_PollAngle(uint16_t *angle) {
    static const uint8_t expected[] = {0x08, 0x18, 0x28, 0x10, 0x40, 0x50, 0x58};
    if (!I2C_GetIrq(CW_I2C1)) {
        if (BoardTime_NowUs() - read_started_us < 600u) return MT6701_BUSY;
        goto error;
    }
    if (I2C_GetState(CW_I2C1) != expected[read_step]) goto error;
    switch (read_step++) {
        case 0:
            I2C_GenerateSTART(CW_I2C1, DISABLE);
            I2C_Send7bitAddress(CW_I2C1, MT6701_ADDR << 1, I2C_Direction_Transmitter);
            break;
        case 1:
            I2C_SendData(CW_I2C1, MT6701_REG_ANGLE);
            break;
        case 2:
            I2C_GenerateSTART(CW_I2C1, ENABLE);
            break;
        case 3:
            I2C_GenerateSTART(CW_I2C1, DISABLE);
            I2C_Send7bitAddress(CW_I2C1, MT6701_ADDR << 1, I2C_Direction_Receiver);
            break;
        case 4:
            I2C_AcknowledgeConfig(CW_I2C1, ENABLE);
            break;
        case 5:
            read_high = I2C_ReceiveData(CW_I2C1);
            I2C_AcknowledgeConfig(CW_I2C1, DISABLE);
            break;
        case 6:
            *angle = (uint16_t)((read_high << 6) | (I2C_ReceiveData(CW_I2C1) >> 2));
            I2C_GenerateSTOP(CW_I2C1);
            I2C_ClearIrq(CW_I2C1);
            return MT6701_READY;
    }
    I2C_ClearIrq(CW_I2C1);
    return MT6701_BUSY;
error:
    I2C_GenerateSTOP(CW_I2C1);
    I2C_ClearIrq(CW_I2C1);
    return MT6701_ERROR;
}

bool MT6701_ReadAngle(uint16_t *angle) {
    MT6701_BeginRead();
    MT6701_ReadResult result;
    do {
        result = MT6701_PollAngle(angle);
    } while (result == MT6701_BUSY);
    return result == MT6701_READY;
}
