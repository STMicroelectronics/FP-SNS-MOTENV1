/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    TargetFeatures.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    31-January-2023
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
#include "stm32l4xx_hal.h"
#include "stm32l4xx_nucleo.h"
#include "stm32l4xx_hal_conf.h"

#include "iks01a2_motion_sensors.h"
#include "iks01a2_motion_sensors_ex.h"
#include "iks01a2_env_sensors.h"
#include "iks01a2_env_sensors_ex.h"

#include "MOTENV1_config.h"
#include "MetaDataManager.h"

#include "MotionFX_Manager.h"
#include "motion_fx.h"

#include "MotionCP_Manager.h"
#include "motion_cp.h"

#include "MotionAR_Manager.h"
#include "motion_ar.h"

#include "MotionGR_Manager.h"
#include "motion_gr.h"

#include "MotionPM_Manager.h"
#include "motion_pm.h"

#include "MotionID_Manager.h"
#include "motion_id.h"

/* Exported defines ------------------------------------------------------- */
/* Exported defines for TIMER Led Blinking ------------------------------------*/
#define TimLedHandle htim2
#define Led_TIM_INSTANCE TIM2
/* Exported defines for TIMER ENV ---------------------------------------------*/
#define TimEnvHandle htim4
#define ENV_TIM_INSTANCE TIM4
/* Exported defines for TIMER MOTION ALGO ---------------------------------------------*/
#define TimCCHandle htim1
#define MOTION_ALGO_TIM_INSTANCE TIM1

#ifndef ALGO_FREQ_CP_GR_PM
  #define ALGO_FREQ_CP_GR_PM 50U
#endif /* ALGO_FREQ_CP_GR_PM */

#ifndef ALGO_FREQ_AR_ID
  #define ALGO_FREQ_AR_ID 16U
#endif /* ALGO_FREQ_AR_ID */

#ifndef ALGO_FREQ_FX
  #define ALGO_FREQ_FX 100U
#endif /* ALGO_FREQ_FX */

#ifndef FREQ_ACC_GYRO_MAG
  #define FREQ_ACC_GYRO_MAG 20U
#endif /* FREQ_ACC_GYRO_MAG */

#ifndef ALGO_PERIOD_LED
  #define ALGO_PERIOD_LED 1000U
#endif /* ALGO_PERIOD_LED */

#ifndef ALGO_PERIOD_ENV
  #define ALGO_PERIOD_ENV 500U
#endif /* ALGO_PERIOD_ENV */

/* Exported Variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;

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

  /* Sensor Fusion Library */
  uint32_t MotionFXIsInitalized;

  /* Carry Position Library */
  uint32_t MotionCPIsInitalized;

  /* Activity Recognition Library */
  uint32_t MotionARIsInitalized;

  /* Gesture Recognition Library */
  uint32_t MotionGRIsInitalized;

  /* Pedometer Library */
  uint32_t MotionPMIsInitalized;

  /* Motion Intensity Detection Library */
  uint32_t MotionIDIsInitalized;
} TargetFeatures_t;

/* Exported variables ------------------------------------------------------- */
extern TargetFeatures_t TargetBoardFeatures;

/* Exported functions ------------------------------------------------------- */
extern void Led_TIM_Init(void);
extern void Env_TIM_Init(void);
extern void MotionAlgo_TIM_Init(void);

extern void InitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

