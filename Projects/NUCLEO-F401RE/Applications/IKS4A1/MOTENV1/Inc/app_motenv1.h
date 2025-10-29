/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_motenv1.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   This file provides code for the configuration FP-SNS-MOTENV1
  *          application instances.
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
#ifndef __APP_FP_SNS_MOTENV1_H
#define __APP_FP_SNS_MOTENV1_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_exti.h"

#include "ble_manager.h"
#include "hw_advance_features.h"

#include "ota.h"

/* Exported Defines ----------------------------------------------------------*/
#define STM32F4xx

/* Define How Many quaternions you want to transmit (from 1 to 3) */
#define SEND_N_QUATERNIONS NUMBER_OF_QUATERNION

/* IMPORTANT
The Sensors fusion runs at 100Hz so like MAXIMUM it possible to send:
1 quaternion every 10ms
2 quaternions every 20ms
3 quaternions every 30ms
*/

/* Define The transmission interval in Multiple of 10ms for quaternions */
#define QUAT_UPDATE_MUL_10MS NUMBER_OF_QUATERNION

/* Control Section */

#if ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3))
#error "SEND_N_QUATERNIONS could be only 1,2 or 3"
#endif /* ((SEND_N_QUATERNIONS<1) || (SEND_N_QUATERNIONS>3)) */

#if ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1))
#error "If QUAT_UPDATE_MUL_10MS!=3 then SEND_N_QUATERNIONS must be = 1"
#endif /* ((QUAT_UPDATE_MUL_10MS!=3) && (SEND_N_QUATERNIONS!=1)) */

/* Exported Variables --------------------------------------------------------*/
extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR2_Val;
extern uint32_t uhCCR3_Val;
extern uint32_t uhCCR4_Val;

extern volatile uint8_t MEMSInterrupt;

extern uint8_t LedTimerEnabled;
extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t InertialTimerEnabled;
extern uint8_t AccEventEnabled;
extern uint8_t LedEnabled;

extern uint8_t ActivityRecognitionEnabled;
extern uint8_t CarryPositionEnabled;
extern uint8_t GestureRecognitionEnabled;
extern uint8_t MotionIntensityEnabled;
extern uint8_t PedometerAlgorithmEnabled;
extern uint8_t SensorFusionEnabled;
extern uint8_t ECompassEnabled;

extern MPM_output_t PM_DataOUTStored;

extern unsigned char isCal;

extern uint32_t ForceReCalibration;

/* Exported Functions --------------------------------------------------------*/
void MX_MOTENV1_Init(void);
void MX_MOTENV1_Process(void);

extern void Set2GAccelerometerFullScale(void);
extern void Set4GAccelerometerFullScale(void);
/* Update Node Name on to Meta Data */
extern void UpdateNodeNameMetaData(void);
/* User function for Erasing the Flash data */
extern uint32_t EraseMetaData(void);
/* User function for Saving the Meta Data on the Flash */
extern void SaveMetaData(void);

extern unsigned char ReCallNodeNameFromMemory(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_FP_SNS_MOTENV1_H */

