/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   Header for main.c file.
  *          This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

#include "TargetFeatures.h"
#include "hci_tl_interface.h"
#include "stm32l0xx_nucleo.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BSP_BUTTON_EXTI_IRQn EXTI4_15_IRQn
#define SPI1_EXTI_Pin_Pin GPIO_PIN_0
#define SPI1_EXTI_Pin_GPIO_Port GPIOA
#define SPI1_EXTI_Pin_EXTI_IRQn EXTI0_1_IRQn
#define SPI1_CS_Pin_Pin GPIO_PIN_1
#define SPI1_CS_Pin_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
