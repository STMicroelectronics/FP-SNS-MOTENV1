/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MotionCP_Manager.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   This file includes carry position recognition interface functions
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

/* Code for MotionCP integration - Start Section */

/* Imported Variable -------------------------------------------------------------*/
extern float sensitivity_Mul;

/* exported Variable -------------------------------------------------------------*/
MCP_output_t CarryPositionCode = MCP_UNKNOWN;

/* Private defines -----------------------------------------------------------*/

/** @addtogroup  Drv_Sensor      Drv_Sensor
  * @{
  */

/** @addtogroup Drv_MotionCP    Drv_MotionCP
  * @{
  */

/* Exported Functions --------------------------------------------------------*/
/**
* @brief  Run carry position algorithm. This function collects and scale data
* from accelerometer and calls the Carry Position Algo
* @param  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw Acceleration values (x,y,z)
* @retval None
*/
void MotionCP_manager_run(MOTION_SENSOR_AxesRaw_t ACC_Value_Raw)
{
  MCP_input_t iDataIN;

  iDataIN.AccX = ACC_Value_Raw.x * sensitivity_Mul;
  iDataIN.AccY = ACC_Value_Raw.y * sensitivity_Mul;
  iDataIN.AccZ = ACC_Value_Raw.z * sensitivity_Mul;

  MotionCP_Update(&iDataIN, &CarryPositionCode);
}

/**
* @brief  Initialises MotionCP algorithm
* @param  None
* @retval None
*/
void MotionCP_manager_init(void)
{
  char LibVersion[36];
  char acc_orientation[3];

  MotionCP_Initialize();
  MotionCP_GetLibVersion(LibVersion);

  acc_orientation[0] ='s';
  acc_orientation[1] ='e';
  acc_orientation[2] ='u';

  MotionCP_SetOrientation_Acc(acc_orientation);

  TargetBoardFeatures.MotionCPIsInitalized=1;
  MOTENV1_PRINTF("Initialized %s\r\n", LibVersion);
}

/**
 * @}
 */ /* end of group  Drv_MotionCP        Drv_MotionCP*/

/**
 * @}
 */ /* end of group Drv_Sensor          Drv_Sensor*/

/* Code for MotionCP integration - End Section */

