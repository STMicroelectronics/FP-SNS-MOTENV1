/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MotionPM_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   This file includes Pedometer interface functions
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include "TargetFeatures.h"

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MPM_output_t PM_DataOUT;

/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionGR    Drv_MotionPM
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run gesture recognition algorithm. This function collects and scale data
* from accelerometer and calls the Pedometer Algo
* @param  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw Acceleration values (x/y/z)
* @retval None
*/
void MotionPM_manager_run(MOTION_SENSOR_AxesRaw_t ACC_Value_Raw)
{
  MPM_input_t iDataIN;

  iDataIN.AccX = ((float) ACC_Value_Raw.x) * sensitivity_Mul;
  iDataIN.AccY = ((float) ACC_Value_Raw.y) * sensitivity_Mul;
  iDataIN.AccZ = ((float) ACC_Value_Raw.z) * sensitivity_Mul;

  MotionPM_Update(&iDataIN, &PM_DataOUT);
}

/**
* @brief  Initialises MotionPM algorithm
* @param  None
* @retval None
*/
void MotionPM_manager_init(void)
{
  char LibVersion[36];

  MotionPM_Initialize();
  MotionPM_GetLibVersion(LibVersion);

  TargetBoardFeatures.MotionPMIsInitalized=1;
  MOTENV1_PRINTF("Initialized %s\n\r", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionGR        Drv_MotionPM*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

