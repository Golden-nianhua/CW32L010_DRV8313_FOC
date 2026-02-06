/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    interrupts.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 CW.
  * All rights reserved.</center></h2>
  *
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "../inc/interrupts_cw32l010.h"

#include "cw32l010_gpio.h"
#include "cw32l010_btim.h"
#include "cw32l010_uart.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
/* USER CODE END TD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
extern void GPIOA_IRQHandlerCallback(void);
/* USER CODE END PFP */


/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/* USER CODE END EV */


/******************************************************************************/
/*           Cortex-M0P Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
    /* USER CODE BEGIN NonMaskableInt_IRQn */

    /* USER CODE END NonMaskableInt_IRQn */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
    /* USER CODE BEGIN HardFault_IRQn */

    /* USER CODE END HardFault_IRQn */
    while (1)
    {
        /* USER CODE BEGIN W1_HardFault_IRQn */

        /* USER CODE END W1_HardFault_IRQn */
    }
}


/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
    /* USER CODE BEGIN SVCall_IRQn */

    /* USER CODE END SVCall_IRQn */
}


/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
    /* USER CODE BEGIN PendSV_IRQn */

    /* USER CODE END PendSV_IRQn */
}


/******************************************************************************/
/* CW030 Peripheral Interrupt Handlers                                        */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_ps030.s).                        */
/******************************************************************************/

/**
 * @brief This funcation handles WDT
 */
void WDT_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles LVD
 */
void LVD_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles RTC
 */
void RTC_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles FLASHRAM
 */
void FLASHRAM_IRQHandler(void)
{
    /* USER CODE BEGIN */
    /* USER CODE END */
}

/**
 * @brief This funcation handles SYSCTRL
 */
void SYSCTRL_IRQHandler(void)
{
    /* USER CODE BEGIN */
    /* USER CODE END */
}

/**
 * @brief This funcation handles GPIOA
 */
void GPIOA_IRQHandler(void)
{
    /* USER CODE BEGIN */
    GPIOA_IRQHandlerCallback();
    // /* USER CODE END */
}

/**
 * @brief This funcation handles GPIOB
 */
void GPIOB_IRQHandler(void)
{
    /* USER CODE BEGIN */
    /* USER CODE END */
}


extern volatile uint8_t gFlagIrq;
/**
 * @brief This funcation handles ADC
 */
void ADC_IRQHandler(void)
{
    /* USER CODE BEGIN */
    if (CW_ADC->ISR & ADC_ISR_EOS_Msk)
    {
        gFlagIrq = CW_ADC->ISR;
    }
    CW_ADC->ICR = 0x00;
    /* USER CODE END */
}

/**
 * @brief This funcation handles ATIM
 */
void ATIM_IRQHandler(void)
{
    /* USER CODE BEGIN */
    /* USER CODE END */
}

/**
 * @brief This funcation handles VC1
 */
void VC1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles VC2
 */
void VC2_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles GTIM1
 */
void GTIM1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles LPTIM
 */
void LPTIM_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

extern volatile uint32_t microseconds;
/**
 * @brief This funcation handles BTIM1
 */
void BTIM1_IRQHandler(void)
{
    /* USER CODE BEGIN */
    if (BTIM_GetITStatus(CW_BTIM1, BTIM_IT_UPDATE))
    {
        BTIM_ClearITPendingBit(CW_BTIM1, BTIM_IT_UPDATE);
        microseconds += 10;
    }
    /* USER CODE END */
}

/**
 * @brief This funcation handles BTIM2
 */
void BTIM2_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles BTIM3
 */
void BTIM3_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles I2C1
 */
void I2C1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/**
 * @brief This funcation handles SPI1
 */
void SPI1_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

static char uart_rx_buffer[16];  // 接收缓冲区
static uint8_t uart_rx_index = 0;  // 缓冲区索引
static uint8_t uart_rx_started = 0;  // 是否开始接收
extern float motor_target;
/**
 * @brief This funcation handles UART1
 */
void UART1_IRQHandler(void)
{
    /* USER CODE BEGIN */
    uint8_t rx;
    if (UART_GetFlagStatus(CW_UART1, UART_IT_RC) != RESET) {
        UART_ClearITPendingBit(CW_UART1, UART_IT_RC);
        rx = UART_ReceiveData_8bit(CW_UART1);
        // 处理接收到的字符
        if (rx == '@') {
            // 起始符，开始接收
            uart_rx_started = 1;
            uart_rx_index = 0;
            uart_rx_buffer[0] = '\0';
        }
        else if (rx == ';' && uart_rx_started) {
            // 结束符，解析数据
            uart_rx_buffer[uart_rx_index] = '\0';  // 添加字符串结束符

            // 将字符串转换为整数
            int angle_value = 0;
            int is_negative = 0;
            uint8_t i = 0;

            // 检查是否为负数
            if (uart_rx_buffer[0] == '-') {
                is_negative = 1;
                i = 1;
            }

            // 转换数字
            while (i < uart_rx_index && uart_rx_buffer[i] >= '0' && uart_rx_buffer[i] <= '9') {
                angle_value = angle_value * 10 + (uart_rx_buffer[i] - '0');
                i++;
            }

            // 应用符号
            if (is_negative) {
                angle_value = -angle_value;
            }

            // 更新motor_target
            motor_target = (float)angle_value/1000.0f;
            // 重置状态
            uart_rx_started = 0;
            uart_rx_index = 0;
        }
        else if (uart_rx_started) {
            // 接收数据字符
            if (uart_rx_index < sizeof(uart_rx_buffer) - 1) {
                uart_rx_buffer[uart_rx_index++] = (char)rx;
            }
            else {
                // 缓冲区溢出，重置
                uart_rx_started = 0;
                uart_rx_index = 0;
            }
        }
    }
    /* USER CODE END */
}

/**
 * @brief This funcation handles UART2
 */
void UART2_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}


/**
 * @brief This funcation handles FAULT
 */
void CLKFAULT_IRQHandler(void)
{
    /* USER CODE BEGIN */

    /* USER CODE END */
}

/************************ (C) COPYRIGHT CW *****END OF FILE****/
