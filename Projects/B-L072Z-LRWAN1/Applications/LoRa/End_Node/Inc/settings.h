/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : settings.h
  * @brief          :
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SETTINGS_H
#define __SETTINGS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/


const uint8_t BATTERY_ID = 0x04;
const uint8_t PMSENSOR1_ID = 0x01;
const uint8_t PMSENSOR25_ID = 0x02;
const uint8_t PMSENSOR10_ID = 0x03;
const uint8_t DCSENSOR_ID = 0x05;
const uint8_t SWITCH_ID = 0x06;
const uint8_t TEMPERATURE_ID = 0x07;


uint8_t DevEui[8] = { 0x31, 0x31, 0x35, 0x38, 0x6E, 0x37, 0x77, 0x18 };
uint8_t JoinEui[8] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x01, 0xA8, 0x63 };
uint8_t AppKey[16] = { 0x95, 0x53, 0xE9, 0xEF, 0x9B, 0x5D, 0xDF, 0x7C, 0xE4, 0x97, 0x86, 0xBB, 0x6C, 0xF5, 0x38, 0x9C };
uint8_t NwkKey[16] = { 0x95, 0x53, 0xE9, 0xEF, 0x9B, 0x5D, 0xDF, 0x7C, 0xE4, 0x97, 0x86, 0xBB, 0x6C, 0xF5, 0x38, 0x9C };


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __SETTINGS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
