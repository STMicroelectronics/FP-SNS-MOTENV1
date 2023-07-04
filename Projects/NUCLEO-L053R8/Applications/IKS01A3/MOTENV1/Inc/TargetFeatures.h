/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    TargetFeatures.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   Specification of the HW Features for each target platform
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
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"
#include "stm32l0xx_nucleo.h"
#include "stm32l0xx_hal_conf.h"

#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"

#include "MOTENV1_config.h"

/* Exported defines ------------------------------------------------------- */

/* Exported defines for TIMER MOTION ALGO ---------------------------------------------*/
#define TimCCHandle htim2
#define TIM_INSTANCE TIM2
#ifndef ALGO_FREQ_ENV
  #define ALGO_FREQ_ENV 2U
#endif /* ALGO_FREQ_ENV */

#ifndef FREQ_ACC_GYRO_MAG
  #define FREQ_ACC_GYRO_MAG 20U
#endif /* FREQ_ACC_GYRO_MAG */

#ifndef ALGO_FREQ_LED
  #define ALGO_FREQ_LED 1U
#endif /* ALGO_FREQ_LED */

/* Exported Variables --------------------------------------------------------*/
extern EXTI_HandleTypeDef hexti5;
#define H_EXTI_5 hexti5

extern TIM_HandleTypeDef htim2;

/* Mems Board Type */
#define _IKS01A2 0
#define _IKS01A3 1

/* Exported macros ------------------------------------------------------- */

/* Exported types ------------------------------------------------------- */

/**
 * @brief  Target's Features data structure definition
 */
typedef struct
{
  int32_t NumTempSensors;

  uint8_t LedStatus;
  uint8_t mems_expansion_board;

  uint8_t IKS01Ax_support;

} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void TIM_Init(void);

extern void SetAccIntPin_exti(void);

extern void InitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

