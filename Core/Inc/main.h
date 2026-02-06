/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "base_types.h"
#include "cw32l010.h"
#include "cw32l010_gpio.h"
#include "cw32l010_uart.h"
#include "cw32l010_flash.h"
#include "cw32l010_i2c.h"
#include "i2c_eeprom.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_digitalsign.h"
#include "cw32l010_sysctrl.h"
#include "cw32l010_systick.h"
#include "cw32l010_adc.h"
#include "interrupts_cw32l010.h"
#include "system_cw32l010.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mt6701.h"
#include "bsp.h"
#include "led.h"
#include "foc.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define ENABLE_LOG

#ifdef ENABLE_LOG
  #define LOG(...) printf(__VA_ARGS__)
#else
  #define LOG(...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT CW *****END OF FILE****/
