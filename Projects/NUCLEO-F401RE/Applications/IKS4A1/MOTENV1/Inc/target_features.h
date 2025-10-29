/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    target_features.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   Specification of the HW Features for each target platform
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_conf.h"

#include "iks4a1_motion_sensors.h"
#include "iks4a1_motion_sensors_ex.h"
#include "iks4a1_env_sensors.h"
#include "iks4a1_env_sensors_ex.h"

#include "motenv1_config.h"

#include "motion_fx_manager.h"
#include "motion_fx.h"

#include "motion_cp_manager.h"
#include "motion_cp.h"

#include "motion_ar_manager.h"
#include "motion_ar.h"

#include "motion_gr_manager.h"
#include "motion_gr.h"

#include "motion_pm_manager.h"
#include "motion_pm.h"

#include "motion_id_manager.h"
#include "motion_id.h"

/* Exported defines ------------------------------------------------------- */
/* Exported defines for TIMER Led Blinking ------------------------------------*/
#define TIM_LED_HANDLE htim2
#define LED_TIM_INSTANCE TIM2
/* Exported defines for TIMER ENV ---------------------------------------------*/
#define TIM_ENV_HANDLE htim4
#define ENV_TIM_INSTANCE TIM4
/* Exported defines for TIMER MOTION ALGO ---------------------------------------------*/
#define TIM_CC_HANDLE htim1
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
extern EXTI_HandleTypeDef hexti5;
#define H_EXTI_5 hexti5

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim1;

/* Exported define ------------------------------------------------------------*/
/* Mems Board Type */
#define _IKS4A1 0

#define USED_ACC_INT_PIN_5

#ifdef USED_ACC_INT_PIN_4
#define MOTION_SENSOR_INT_PIN MOTION_SENSOR_INT1_PIN
#endif /* USED_ACC_INT_PIN_4 */
#ifdef USED_ACC_INT_PIN_5
#define MOTION_SENSOR_INT_PIN MOTION_SENSOR_INT2_PIN
#endif /* USED_ACC_INT_PIN_5 */

#ifndef MOTION_SENSOR_INT_PIN
#define MOTION_SENSOR_INT_PIN MOTION_SENSOR_INT1_PIN
#endif /* MOTION_SENSOR_INT_PIN */

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

extern void SetAccIntPin_exti(void);

extern void InitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */

