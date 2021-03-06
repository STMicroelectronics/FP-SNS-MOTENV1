/**
  ******************************************************************************
  * @file    MOTENV1_config.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   FP-SNS-MOTENV1 configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTENV1_CONFIG_H
#define __MOTENV1_CONFIG_H

/* Exported define ------------------------------------------------------------*/

/*************************************/
/*  Remapping istance sensor defines */
/*************************************/
/* Motion Sensor Istance */
#define ACCELERO_INSTANCE        IKS01A2_LSM6DSL_0
#define GYRO_INSTANCE            IKS01A2_LSM6DSL_0
#define MAGNETO_INSTANCE         IKS01A2_LSM303AGR_MAG_0

/* Environmental Sensor Istance */
#define TEMPERATURE_INSTANCE_1  IKS01A2_HTS221_0
#define HUMIDITY_INSTANCE       IKS01A2_HTS221_0
#define TEMPERATURE_INSTANCE_2  IKS01A2_LPS22HB_0
#define PRESSURE_INSTANCE       IKS01A2_LPS22HB_0
/*************************************/

/********************************/
/*  Remapping APIsensor defines */
/********************************/
/* Environmental Sensor API */
#define ENV_SENSOR_Init         IKS01A2_ENV_SENSOR_Init
#define ENV_SENSOR_Enable       IKS01A2_ENV_SENSOR_Enable
#define ENV_SENSOR_GetValue     IKS01A2_ENV_SENSOR_GetValue

/* Motion Sensor API */
#define MOTION_SENSOR_Init                    IKS01A2_MOTION_SENSOR_Init
#define MOTION_SENSOR_Enable                  IKS01A2_MOTION_SENSOR_Enable

#define MOTION_SENSOR_INT1_PIN                IKS01A2_MOTION_SENSOR_INT1_PIN

#define MOTION_SENSOR_AxesRaw_t               IKS01A2_MOTION_SENSOR_AxesRaw_t
#define MOTION_SENSOR_Axes_t                  IKS01A2_MOTION_SENSOR_Axes_t
#define MOTION_SENSOR_Event_Status_t          IKS01A2_MOTION_SENSOR_Event_Status_t

#define MOTION_SENSOR_GetAxesRaw              IKS01A2_MOTION_SENSOR_GetAxesRaw
#define MOTION_SENSOR_GetAxes                 IKS01A2_MOTION_SENSOR_GetAxes
#define MOTION_SENSOR_Get_Event_Status        IKS01A2_MOTION_SENSOR_Get_Event_Status

#define MOTION_SENSOR_GetSensitivity          IKS01A2_MOTION_SENSOR_GetSensitivity
#define MOTION_SENSOR_SetFullScale            IKS01A2_MOTION_SENSOR_SetFullScale

#define MOTION_SENSOR_SetOutputDataRate       IKS01A2_MOTION_SENSOR_SetOutputDataRate

#define MOTION_SENSOR_GetOutputDataRate                 IKS01A2_MOTION_SENSOR_GetOutputDataRate
#define MOTION_SENSOR_Enable_Tilt_Detection             IKS01A2_MOTION_SENSOR_Enable_Tilt_Detection
#define MOTION_SENSOR_Disable_Tilt_Detection            IKS01A2_MOTION_SENSOR_Disable_Tilt_Detection
#define MOTION_SENSOR_Enable_Wake_Up_Detection          IKS01A2_MOTION_SENSOR_Enable_Wake_Up_Detection
#define MOTION_SENSOR_Disable_Wake_Up_Detection         IKS01A2_MOTION_SENSOR_Disable_Wake_Up_Detection
#define MOTION_SENSOR_Enable_Double_Tap_Detection       IKS01A2_MOTION_SENSOR_Enable_Double_Tap_Detection
#define MOTION_SENSOR_Disable_Double_Tap_Detection      IKS01A2_MOTION_SENSOR_Disable_Double_Tap_Detection
#define MOTION_SENSOR_Enable_Single_Tap_Detection       IKS01A2_MOTION_SENSOR_Enable_Single_Tap_Detection
#define MOTION_SENSOR_Disable_Single_Tap_Detection      IKS01A2_MOTION_SENSOR_Disable_Single_Tap_Detection
#define MOTION_SENSOR_Enable_Pedometer                  IKS01A2_MOTION_SENSOR_Enable_Pedometer
#define MOTION_SENSOR_Disable_Pedometer                 IKS01A2_MOTION_SENSOR_Disable_Pedometer
#define MOTION_SENSOR_Reset_Step_Counter                IKS01A2_MOTION_SENSOR_Reset_Step_Counter
#define MOTION_SENSOR_Get_Step_Count                    IKS01A2_MOTION_SENSOR_Get_Step_Count
#define MOTION_SENSOR_Enable_Free_Fall_Detection        IKS01A2_MOTION_SENSOR_Enable_Free_Fall_Detection
#define MOTION_SENSOR_Disable_Free_Fall_Detection       IKS01A2_MOTION_SENSOR_Disable_Free_Fall_Detection
#define MOTION_SENSOR_Set_Free_Fall_Threshold           IKS01A2_MOTION_SENSOR_Set_Free_Fall_Threshold
#define MOTION_SENSOR_Enable_6D_Orientation             IKS01A2_MOTION_SENSOR_Enable_6D_Orientation
#define MOTION_SENSOR_Disable_6D_Orientation            IKS01A2_MOTION_SENSOR_Disable_6D_Orientation
#define MOTION_SENSOR_Get_6D_Orientation_XL             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XL
#define MOTION_SENSOR_Get_6D_Orientation_XH             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_XH
#define MOTION_SENSOR_Get_6D_Orientation_YL             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YL
#define MOTION_SENSOR_Get_6D_Orientation_YH             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_YH
#define MOTION_SENSOR_Get_6D_Orientation_ZL             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZL
#define MOTION_SENSOR_Get_6D_Orientation_ZH             IKS01A2_MOTION_SENSOR_Get_6D_Orientation_ZH
/********************************/




/*************** Debug Defines ******************/
#ifndef __GNUC__
  #define MOTENV1_ENABLE_PRINTF
#endif /* __GNUC__ */

/* For enabling connection and notification subscriptions debug */
#define MOTENV1_DEBUG_CONNECTION

/* For enabling transmission for notified services (except for quaternions) */
#define MOTENV1_DEBUG_NOTIFY_TRAMISSION

/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define MOTENV1_VERSION_MAJOR '4'
#define MOTENV1_VERSION_MINOR '2'
#define MOTENV1_VERSION_PATCH '0'

/* Define the MOTENV1 Name MUST be 7 char long */
#define NAME_BLUEMS 'M','E','1','V',MOTENV1_VERSION_MAJOR,MOTENV1_VERSION_MINOR,MOTENV1_VERSION_PATCH

/* Package Name */
#define MOTENV1_PACKAGENAME "FP-SNS-MOTENV1"

#ifdef MOTENV1_ENABLE_PRINTF
  #define MOTENV1_PRINTF(...) printf(__VA_ARGS__)
#else /* MOTENV1_ENABLE_PRINTF */
  #define MOTENV1_PRINTF(...)
#endif /* MOTENV1_ENABLE_PRINTF */

/* STM32 Unique ID */
#ifdef USE_STM32F4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7A10)
#endif /* USE_STM32F4XX_NUCLEO */

#ifdef USE_STM32L4XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FFF7590)
#endif /* USE_STM32L4XX_NUCLEO */

#ifdef USE_STM32L0XX_NUCLEO
#define STM32_UUID ((uint32_t *)0x1FF80050)
#endif /* USE_STM32L0XX_NUCLEO */

/* STM32 MCU_ID */
#define STM32_MCU_ID ((uint32_t *)0xE0042000)

#endif /* __MOTENV1_CONFIG_H */

