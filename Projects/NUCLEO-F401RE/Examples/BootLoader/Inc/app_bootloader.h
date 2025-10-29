/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_bootloader.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 3.0.0
  * @date    31-January-2023
  * @brief   This file provides code for the configuration FP-SNS-MOTENV1
  *          application instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_FP_SNS_MOTENV1_H
#define __APP_FP_SNS_MOTENV1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_exti.h"

/* Exported Defines ----------------------------------------------------------*/
#define STM32F4xx

/* Exported Variables --------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/
void MX_BootLoader_Init(void);
void MX_BootLoader_Process(void);

/* Update Node Name on to Meta Data */
extern void UpdateNodeNameMetaData(void);
/* User function for Erasing the Flash data */
extern uint32_t EraseMetaData(void);
/* User function for Saving the Meta Data on the Flash */
extern void SaveMetaData(void);

extern unsigned char ReCallNodeNameFromMemory(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FP_SNS_MOTENV1_H */

