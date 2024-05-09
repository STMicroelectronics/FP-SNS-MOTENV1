/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_pm_manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   Header for motion_pm_manager.c
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
#ifndef _MOTIONPM_MANAGER_H_
#define _MOTIONPM_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_pm.h"

/** @defgroup Drv_Sensor      Drv_Sensor
  * @{
  */

/** @defgroup Drv_MotionGR            Drv_MotionPM
  * @brief    This file includes Pedometer interface functions
  * @{
  */

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionPM_manager_init(void);
extern void MotionPM_manager_run(MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw);

/* Exported Variables -------------------------------------------------------------*/
extern MPM_output_t PM_DataOUT;

/**
  * @}
  */  /* end of group Drv_MotionPM */

/**
  * @}
  */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif /* _MOTIONPM_MANAGER_H_ */

