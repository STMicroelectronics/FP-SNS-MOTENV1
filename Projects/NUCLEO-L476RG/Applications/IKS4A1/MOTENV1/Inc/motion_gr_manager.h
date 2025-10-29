/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_gr_manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   Header for motion_gr_manager.c
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
#ifndef _MOTIONGR_MANAGER_H_
#define _MOTIONGR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_gr.h"

/** @defgroup Drv_Sensor      Drv_Sensor
  * @{
  */

/** @defgroup Drv_MotionGR            Drv_MotionGR
  * @brief    This file includes Gesture Recognition interface functions
  * @{
  */

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionGR_manager_init(void);
extern void MotionGR_manager_run(MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw);

/* Exported Variables -------------------------------------------------------------*/
extern MGR_output_t GestureRecognitionCode;

/**
  * @}
  */  /* end of group Drv_MotionGR */

/**
  * @}
  */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif /* _MOTIONGR_MANAGER_H_ */

