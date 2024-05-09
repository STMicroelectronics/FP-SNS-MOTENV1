/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_id_manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   Header for motion_id_manager.c
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#ifndef _MOTIONID_MANAGER_H_
#define _MOTIONID_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_id.h"

/** @defgroup Drv_Sensor      Drv_Sensor
  * @{
  */

/** @defgroup Drv_MotionAR            Drv_MotionID
  * @brief    This file includes Motion Intensity Detection interface functions
  * @{
  */

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionID_manager_init(void);
extern void MotionID_manager_run(MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw);

/* Exported Variables -------------------------------------------------------------*/
extern MID_output_t MIDCode;

/**
  * @}
  */  /* end of group Drv_MotionID */

/**
  * @}
  */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif /* _MOTIONID_MANAGER_H_ */

