/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_ar_manager.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   Header for motion_ar_manager.c
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
#ifndef _MOTIONAR_MANAGER_H_
#define _MOTIONAR_MANAGER_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "motion_ar.h"

/** @defgroup Drv_Sensor      Drv_Sensor
  * @{
  */

/** @defgroup Drv_MotionAR            Drv_MotionAR
  * @brief    This file includes Activity Recognition interface functions
  * @{
  */

/* Exported Functions Prototypes ---------------------------------------------*/
extern void MotionAR_manager_init(void);
extern void MotionAR_manager_run(MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw, uint32_t TimeStamp);

/* Exported Variables -------------------------------------------------------------*/
extern MAR_output_t ActivityCode;

/**
  * @}
  */  /* end of group Drv_MotionAR */

/**
  * @}
  */  /* end of group Drv_Sensor */

#ifdef __cplusplus
}
#endif

#endif /* _MOTIONAR_MANAGER_H_ */
