/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#include "hci_tl_interface.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MOTENV1_config.h"
#include "TargetFeatures.h"
#include "BLE_Manager.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
#define MCR_BLUEMS_F2I_3D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*1000);};

/* @brief  Scale factor. It is used to scale acceleration from mg to g */ 
#define FROM_MG_TO_G    0.001f
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

extern unsigned char ReCallCalibrationFromMemory(void);
extern unsigned char SaveCalibrationToMemory(void);

/* Exported variables  ------------------------------------------------------- */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

#define TimEnvHandle htim4
#define TimCCHandle htim1

extern unsigned char isCal;

extern uint32_t ForceReCalibration;

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR2_Val;
extern uint32_t uhCCR3_Val;
extern uint32_t uhCCR4_Val;

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

extern uint8_t TIM1_CHANNEL_1_Enabled;
extern uint8_t TIM1_CHANNEL_2_Enabled;
extern uint8_t TIM1_CHANNEL_3_Enabled;
extern uint8_t TIM1_CHANNEL_4_Enabled;

extern uint8_t NodeName[];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM_CLOCK_ENV 2000U
#define ALGO_FREQ_ENV 2U
#define ALGO_FREQ_FX 100U
#define DEFAULT_uhCCR1_Val (10000U / ALGO_FREQ_FX)
#define ALGO_FREQ_CP_GR_PM 50U
#define DEFAULT_uhCCR2_Val (10000U / ALGO_FREQ_CP_GR_PM)
#define ALGO_FREQ_AR_ID 16U
#define DEFAULT_uhCCR3_Val (10000U / ALGO_FREQ_AR_ID)
#define FREQ_ACC_GYRO_MAG 20U
#define DEFAULT_uhCCR4_Val (10000U / FREQ_ACC_GYRO_MAG)
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define SPI1_EXTI_Pin_Pin GPIO_PIN_0
#define SPI1_EXTI_Pin_GPIO_Port GPIOA
#define SPI1_EXTI_Pin_EXTI_IRQn EXTI0_IRQn
#define SPI1_CS_Pin_Pin GPIO_PIN_1
#define SPI1_CS_Pin_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SPI1_RST_Pin GPIO_PIN_8
#define SPI1_RST_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define MEMS_ACC_INT_Pin GPIO_PIN_5
#define MEMS_ACC_INT_GPIO_Port GPIOB
#define MEMS_ACC_INT_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* 10kHz/100 For MotionFX@100Hz as defaul value */
/* Algorithm period for MotionFX [ms] */
#define ALGO_PERIOD_FX          (1000U / ALGO_FREQ_FX) 

/* 10kHz/50 as defaul value for:
  MotionCP@50Hz or 
  MotionGR@50Hz or
  MotionPM@50Hz */
/* Algorithm period for MotionCP, MotionGR and MotionPM [ms] */
#define ALGO_PERIOD_CP_GR_PM    (1000U / ALGO_FREQ_CP_GR_PM)

/* 10kHz/16 as defaul value for:
  MotionAR@16Hz
  MotionID@16Hz */
/* Algorithm period for MotionAR, MotionID libraries [ms] */
#define ALGO_PERIOD_AR_ID    (1000U / ALGO_FREQ_AR_ID) 

/* 10kHz/20  For Acc/Gyro/Mag@20Hz */
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_ACC_GYRO_MAG        (1000U / FREQ_ACC_GYRO_MAG) 

/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
