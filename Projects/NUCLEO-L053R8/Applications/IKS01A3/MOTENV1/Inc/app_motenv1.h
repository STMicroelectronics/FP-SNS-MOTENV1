/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_motenv1.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
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
#include "stm32l0xx_hal.h"
#include "stm32l0xx_hal_exti.h"

#include "BLE_Manager.h"

/* Exported Defines ----------------------------------------------------------*/
#define STM32L0xx

/* Exported Variables --------------------------------------------------------*/

extern uint32_t Default_uhCCR1_Val;
extern uint32_t Default_uhCCR3_Val;
extern uint32_t Default_uhCCR4_Val;

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR3_Val;
extern uint32_t uhCCR4_Val;

extern volatile int MEMSInterrupt;

extern uint8_t LedTimerEnabled;
extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t InertialTimerEnabled;
extern uint8_t AccEventEnabled;
extern uint8_t LedEnabled;

/* Exported Functions --------------------------------------------------------*/
void MX_MOTENV1_Init(void);
void MX_MOTENV1_Process(void);

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FP_SNS_MOTENV1_H */

