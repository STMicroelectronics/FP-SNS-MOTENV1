/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motion_ar_manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   This file includes activity recognition interface functions
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include <limits.h>
#include "target_features.h"

/* Code for MotionAR integration - Start Section */

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MAR_output_t ActivityCode = MAR_NOACTIVITY;

/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionAR    Drv_MotionAR
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
/**
  * @brief  Run activity recognition algorithm. This function collects and scale data
  * from accelerometer and calls the Activity Recognition Algo
  * @param  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw Acceleration value (x/y/z)
  * @retval None
  */
void MotionAR_manager_run(MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw, uint32_t TimeStamp)
{
  MAR_input_t iDataIN;

  iDataIN.acc_x = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.acc_y = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.acc_z = ACC_Value_Raw.z * sensitivity_Mul;

  MotionAR_Update(&iDataIN, &ActivityCode, (long int)(TimeStamp & LONG_MAX));
}

/**
  * @brief  Initialises MotionAR algorithm
  * @param  None
  * @retval None
  */
void MotionAR_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionAR_Initialize();
  MotionAR_GetLibVersion(LibVersion);

  acc_orientation[0] = 's';
  acc_orientation[1] = 'e';
  acc_orientation[2] = 'u';

  MotionAR_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionARIsInitalized = 1;
  MOTENV1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
  * @}
  */ /* end of group  Drv_MotionAR        Drv_MotionAR*/

/**
  * @}
  */ /* end of group Drv_Sensor          Drv_Sensor*/

/* Code for MotionAR integration - End Section */

