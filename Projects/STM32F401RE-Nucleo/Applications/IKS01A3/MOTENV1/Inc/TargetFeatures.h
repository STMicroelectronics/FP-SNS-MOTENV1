/**
  ******************************************************************************
  * @file    TargetFeatures.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Specification of the HW Features for each target platform
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _TARGET_FEATURES_H_
#define _TARGET_FEATURES_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
   
#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"
#include "stm32f4xx_hal_conf.h"

#include "iks01a3_motion_sensors.h"
#include "iks01a3_motion_sensors_ex.h"
#include "iks01a3_env_sensors.h"
#include "iks01a3_env_sensors_ex.h"

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

  int32_t HWAdvanceFeatures;
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
extern void InitTargetPlatform(void);
extern void LedOnTargetPlatform(void);
extern void LedOffTargetPlatform(void);

#ifdef __cplusplus
}
#endif

#endif /* _TARGET_FEATURES_H_ */


