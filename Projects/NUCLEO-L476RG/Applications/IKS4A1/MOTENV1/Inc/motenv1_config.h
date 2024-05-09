/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motenv1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   FP-SNS-MOTENV1 configuration
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
#ifndef __MOTENV1_CONFIG_H
#define __MOTENV1_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define ------------------------------------------------------------*/

/*************************************/
/*  Remapping instance sensor defines */
/*************************************/
/* Motion Sensor Instance */
#define ACCELERO_INSTANCE        IKS4A1_LSM6DSV16X_0
#define GYRO_INSTANCE            IKS4A1_LSM6DSV16X_0
#define MAGNETO_INSTANCE         IKS4A1_LIS2MDL_0

/* Environmental Sensor Instance */
#define TEMPERATURE_INSTANCE_1  IKS4A1_STTS22H_0
#define HUMIDITY_INSTANCE       IKS4A1_SHT40AD1B_0
#define TEMPERATURE_INSTANCE_2  IKS4A1_LPS22DF_0
#define PRESSURE_INSTANCE       IKS4A1_LPS22DF_0
/*************************************/

/********************************/
/*  Remapping APIsensor defines */
/********************************/
/* Environmental Sensor API */
#define ENV_SENSOR_INIT                 IKS4A1_ENV_SENSOR_Init
#define ENV_SENSOR_ENABLE               IKS4A1_ENV_SENSOR_Enable
#define ENV_SENSOR_GET_VALUE            IKS4A1_ENV_SENSOR_GetValue
#define ENV_SENSOR_GET_OUTPUT_DATA_RATE IKS4A1_ENV_SENSOR_GetOutputDataRate
#define ENV_SENSOR_SET_OUTPUT_DATA_RATE IKS4A1_ENV_SENSOR_SetOutputDataRate

/* Motion Sensor API */
#define MOTION_SENSOR_INIT                    IKS4A1_MOTION_SENSOR_Init
#define MOTION_SENSOR_ENABLE                  IKS4A1_MOTION_SENSOR_Enable

#define MOTION_SENSOR_INT1_PIN                IKS4A1_MOTION_SENSOR_INT1_PIN
#define MOTION_SENSOR_INT2_PIN                IKS4A1_MOTION_SENSOR_INT2_PIN

#define MOTION_SENSOR_AXES_RAW_T              IKS4A1_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_AXES_T                  IKS4A1_MOTION_SENSOR_Axes_t
#define MOTION_SENSOR_EVENT_STATUS_T          IKS4A1_MOTION_SENSOR_Event_Status_t

#define MOTION_SENSOR_GET_AXES_RAW            IKS4A1_MOTION_SENSOR_GetAxesRaw
#define MOTION_SENSOR_GET_AXES                IKS4A1_MOTION_SENSOR_GetAxes
#define MOTION_SENSOR_GET_EVENT_STATUS        IKS4A1_MOTION_SENSOR_Get_Event_Status

#define MOTION_SENSOR_GET_SENSITIVITY         IKS4A1_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SET_FULL_SCALE          IKS4A1_MOTION_SENSOR_SetFullScale

#define MOTION_SENSOR_GET_DRDY_STATUS         IKS4A1_MOTION_SENSOR_Get_DRDY_Status

#define MOTION_SENSOR_SET_OUTPUT_DATA_RATE              IKS4A1_MOTION_SENSOR_SetOutputDataRate
#define MOTION_SENSOR_GET_OUTPUT_DATA_RATE              IKS4A1_MOTION_SENSOR_GetOutputDataRate
#define MOTION_SENSOR_ENABLE_TILT_DETECTION             IKS4A1_MOTION_SENSOR_Enable_Tilt_Detection
#define MOTION_SENSOR_DISABLE_TILT_DETECTION            IKS4A1_MOTION_SENSOR_Disable_Tilt_Detection
#define MOTION_SENSOR_ENABLE_WAKE_UP_DETECTION          IKS4A1_MOTION_SENSOR_Enable_Wake_Up_Detection
#define MOTION_SENSOR_DISABLE_WAKE_UP_DETECTION         IKS4A1_MOTION_SENSOR_Disable_Wake_Up_Detection
#define MOTION_SENSOR_ENABLE_DOUBLE_TAP_DETECTION       IKS4A1_MOTION_SENSOR_Enable_Double_Tap_Detection
#define MOTION_SENSOR_DISABLE_DOUBLE_TAP_DETECTION      IKS4A1_MOTION_SENSOR_Disable_Double_Tap_Detection
#define MOTION_SENSOR_ENABLE_SINGLE_TAP_DETECTION       IKS4A1_MOTION_SENSOR_Enable_Single_Tap_Detection
#define MOTION_SENSOR_DISABLE_SINGLE_TAP_DETECTION      IKS4A1_MOTION_SENSOR_Disable_Single_Tap_Detection
#define MOTION_SENSOR_ENABLE_PEDOMETER                  IKS4A1_MOTION_SENSOR_Enable_Pedometer
#define MOTION_SENSOR_DISABLE_PEDOMETER                 IKS4A1_MOTION_SENSOR_Disable_Pedometer
#define MOTION_SENSOR_RESET_STEP_COUNTER                IKS4A1_MOTION_SENSOR_Reset_Step_Counter
#define MOTION_SENSOR_GET_STEP_COUNT                    IKS4A1_MOTION_SENSOR_Get_Step_Count
#define MOTION_SENSOR_ENABLE_FREE_FALL_DETECTION        IKS4A1_MOTION_SENSOR_Enable_Free_Fall_Detection
#define MOTION_SENSOR_DISABLE_FREE_FALL_DETECTION       IKS4A1_MOTION_SENSOR_Disable_Free_Fall_Detection
#define MOTION_SENSOR_SET_FREE_FALL_THRESHOLD           IKS4A1_MOTION_SENSOR_Set_Free_Fall_Threshold
#define MOTION_SENSOR_ENABLE_6D_ORIENTATION             IKS4A1_MOTION_SENSOR_Enable_6D_Orientation
#define MOTION_SENSOR_DISABLE_6D_ORIENTATION            IKS4A1_MOTION_SENSOR_Disable_6D_Orientation
#define MOTION_SENSOR_GET_6D_ORIENTATION_XL             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_XL
#define MOTION_SENSOR_GET_6D_ORIENTATION_XH             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_XH
#define MOTION_SENSOR_GET_6D_ORIENTATION_YL             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_YL
#define MOTION_SENSOR_GET_6D_ORIENTATION_YH             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_YH
#define MOTION_SENSOR_GET_6D_ORIENTATION_ZL             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_ZL
#define MOTION_SENSOR_GET_6D_ORIENTATION_ZH             IKS4A1_MOTION_SENSOR_Get_6D_Orientation_ZH

/********************************/

/* USER CODE BEGIN 1 */
/*************** Debug Defines ******************/
#define MOTENV1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define MOTENV1_DEBUG_CONNECTION

/* For enabling transmission for notified services (except for quaternions) */
#define MOTENV1_DEBUG_NOTIFY_TRAMISSION

/* Firmware Id */
#define CUSTOM_FIRMWARE_ID 0x09

/*************** Don't Change the following defines *************/
/* USER CODE END 1 */

/* Package Version only numbers 0->9 */
#define MOTENV1_VERSION_MAJOR '5'
#define MOTENV1_VERSION_MINOR '0'
#define MOTENV1_VERSION_PATCH '0'

/* Package Name */
#define MOTENV1_PACKAGENAME "FP-SNS-MOTENV1"

#ifdef MOTENV1_ENABLE_PRINTF
#define MOTENV1_PRINTF(...) printf(__VA_ARGS__)
#else /* MOTENV1_ENABLE_PRINTF */
#define MOTENV1_PRINTF(...)
#endif /* MOTENV1_ENABLE_PRINTF */

#ifdef __cplusplus
}
#endif

#endif /* __MOTENV1_CONFIG_H */

