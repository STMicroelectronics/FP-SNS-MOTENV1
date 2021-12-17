/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#include "stm32l0xx_hal.h"

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
#define FROM_MG_TO_G    0.001
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern void Init_SPI1(void);
extern void Init_I2C1(void);

extern void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend);

/* Exported variables  ------------------------------------------------------- */
extern UART_HandleTypeDef huart2;
extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c1;

extern TIM_HandleTypeDef htim2;

#define TimCCHandle htim2

extern uint32_t uhCCR1_Val;
extern uint32_t uhCCR4_Val;

extern uint8_t EnvironmentalTimerEnabled;
extern uint8_t InertialTimerEnabled;
extern uint8_t AccEventEnabled;
extern uint8_t LedEnabled;

extern uint8_t TIM2_CHANNEL_1_Enabled;
extern uint8_t TIM2_CHANNEL_4_Enabled;

extern uint8_t NodeName[];
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ALGO_FREQ_ENV 2U
#define DEFAULT_uhCCR1_Val (10000/ALGO_FREQ_ENV)
#define ALGO_FREQ_INERTIAL 20U
#define DEFAULT_uhCCR4_Val (10000/ALGO_FREQ_INERTIAL)
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI4_15_IRQn
#define SPI1_EXTI_Pin_Pin GPIO_PIN_0
#define SPI1_EXTI_Pin_GPIO_Port GPIOA
#define SPI1_EXTI_Pin_EXTI_IRQn EXTI0_1_IRQn
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
#define MEMS_ACC_INT_EXTI_IRQn EXTI4_15_IRQn
/* USER CODE BEGIN Private defines */
/* Update period for environmental sensor [ms] */
#define ALGO_PERIOD_ENV (1000U / ALGO_FREQ_ENV)
/* Update period for Acc/Gyro/Mag [ms] */
#define ALGO_PERIOD_ACC_GYRO_MAG        (1000U / ALGO_FREQ_INERTIAL) 
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

