/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MOTENV1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   FP-SNS-MOTENV1 configuration
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTENV1_CONFIG_H
#define __MOTENV1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/*************************************/
/*  Remapping istance sensor defines */
/*************************************/
/* Motion Sensor Istance */
#define ACCELERO_INSTANCE        IKS01A3_LSM6DSO_0
#define GYRO_INSTANCE            IKS01A3_LSM6DSO_0
#define MAGNETO_INSTANCE         IKS01A3_LIS2MDL_0

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE_1  IKS01A3_HTS221_0
#define HUMIDITY_INSTANCE       IKS01A3_HTS221_0
#define TEMPERATURE_INSTANCE_2  IKS01A3_LPS22HH_0
#define PRESSURE_INSTANCE       IKS01A3_LPS22HH_0
/*************************************/

/********************************/
/*  Remapping APIsensor defines */
/********************************/
/* Environmental Sensor API */
#define ENV_SENSOR_Init         IKS01A3_ENV_SENSOR_Init
#define ENV_SENSOR_Enable       IKS01A3_ENV_SENSOR_Enable
#define ENV_SENSOR_GetValue     IKS01A3_ENV_SENSOR_GetValue

/* Motion Sensor API */
#define MOTION_SENSOR_Init                    IKS01A3_MOTION_SENSOR_Init
#define MOTION_SENSOR_Enable                  IKS01A3_MOTION_SENSOR_Enable

#define MOTION_SENSOR_INT1_PIN                IKS01A3_MOTION_SENSOR_INT1_PIN

#define MOTION_SENSOR_AxesRaw_t               IKS01A3_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_Axes_t                  IKS01A3_MOTION_SENSOR_Axes_t
#define MOTION_SENSOR_Event_Status_t          IKS01A3_MOTION_SENSOR_Event_Status_t

#define MOTION_SENSOR_GetAxesRaw              IKS01A3_MOTION_SENSOR_GetAxesRaw
#define MOTION_SENSOR_GetAxes                 IKS01A3_MOTION_SENSOR_GetAxes
#define MOTION_SENSOR_Get_Event_Status        IKS01A3_MOTION_SENSOR_Get_Event_Status

#define MOTION_SENSOR_GetSensitivity          IKS01A3_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SetFullScale            IKS01A3_MOTION_SENSOR_SetFullScale

#define MOTION_SENSOR_SetOutputDataRate       IKS01A3_MOTION_SENSOR_SetOutputDataRate

#define MOTION_SENSOR_GetOutputDataRate                 IKS01A3_MOTION_SENSOR_GetOutputDataRate
#define MOTION_SENSOR_Enable_Tilt_Detection             IKS01A3_MOTION_SENSOR_Enable_Tilt_Detection
#define MOTION_SENSOR_Disable_Tilt_Detection            IKS01A3_MOTION_SENSOR_Disable_Tilt_Detection
#define MOTION_SENSOR_Enable_Wake_Up_Detection          IKS01A3_MOTION_SENSOR_Enable_Wake_Up_Detection
#define MOTION_SENSOR_Disable_Wake_Up_Detection         IKS01A3_MOTION_SENSOR_Disable_Wake_Up_Detection
#define MOTION_SENSOR_Enable_Double_Tap_Detection       IKS01A3_MOTION_SENSOR_Enable_Double_Tap_Detection
#define MOTION_SENSOR_Disable_Double_Tap_Detection      IKS01A3_MOTION_SENSOR_Disable_Double_Tap_Detection
#define MOTION_SENSOR_Enable_Single_Tap_Detection       IKS01A3_MOTION_SENSOR_Enable_Single_Tap_Detection
#define MOTION_SENSOR_Disable_Single_Tap_Detection      IKS01A3_MOTION_SENSOR_Disable_Single_Tap_Detection
#define MOTION_SENSOR_Enable_Pedometer                  IKS01A3_MOTION_SENSOR_Enable_Pedometer
#define MOTION_SENSOR_Disable_Pedometer                 IKS01A3_MOTION_SENSOR_Disable_Pedometer
#define MOTION_SENSOR_Reset_Step_Counter                IKS01A3_MOTION_SENSOR_Reset_Step_Counter
#define MOTION_SENSOR_Get_Step_Count                    IKS01A3_MOTION_SENSOR_Get_Step_Count
#define MOTION_SENSOR_Enable_Free_Fall_Detection        IKS01A3_MOTION_SENSOR_Enable_Free_Fall_Detection
#define MOTION_SENSOR_Set_Tap_Threshold                 IKS01A3_MOTION_SENSOR_Set_Tap_Threshold
#define MOTION_SENSOR_Disable_Free_Fall_Detection       IKS01A3_MOTION_SENSOR_Disable_Free_Fall_Detection
#define MOTION_SENSOR_Set_Free_Fall_Threshold           IKS01A3_MOTION_SENSOR_Set_Free_Fall_Threshold
#define MOTION_SENSOR_Enable_6D_Orientation             IKS01A3_MOTION_SENSOR_Enable_6D_Orientation
#define MOTION_SENSOR_Disable_6D_Orientation            IKS01A3_MOTION_SENSOR_Disable_6D_Orientation
#define MOTION_SENSOR_Get_6D_Orientation_XL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XL
#define MOTION_SENSOR_Get_6D_Orientation_XH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_XH
#define MOTION_SENSOR_Get_6D_Orientation_YL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YL
#define MOTION_SENSOR_Get_6D_Orientation_YH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_YH
#define MOTION_SENSOR_Get_6D_Orientation_ZL             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZL
#define MOTION_SENSOR_Get_6D_Orientation_ZH             IKS01A3_MOTION_SENSOR_Get_6D_Orientation_ZH
/********************************/

/* USER CODE BEGIN 1 */
/*************** Debug Defines ******************/
/* Due to flash constrain the VCOM printf is disabled */
//#define MOTENV1_ENABLE_PRINTF

/* For enabling connection and notification subscriptions debug */
#define MOTENV1_DEBUG_CONNECTION

/* For enabling transmission for notified services (except for quaternions) */
#define MOTENV1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/
/* USER CODE END 1 */

/* Package Version only numbers 0->9 */
#define MOTENV1_VERSION_MAJOR '4'
#define MOTENV1_VERSION_MINOR '3'
#define MOTENV1_VERSION_PATCH '2'

/* Package Name */
#define MOTENV1_PACKAGENAME "FP-SNS-MOTENV1"

#ifdef MOTENV1_ENABLE_PRINTF
  #define MOTENV1_PRINTF(...) printf(__VA_ARGS__)
#else /* MOTENV1_ENABLE_PRINTF */
  #define MOTENV1_PRINTF(...)
#endif /* MOTENV1_ENABLE_PRINTF */

#endif /* __MOTENV1_CONFIG_H */

