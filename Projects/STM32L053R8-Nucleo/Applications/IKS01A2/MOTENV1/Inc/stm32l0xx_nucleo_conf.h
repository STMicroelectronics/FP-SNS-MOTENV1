/**
  ******************************************************************************
  * @file    stm32l0xx_nucleo_conf.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Configuration file
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
#ifndef STM32L0XX_NUCLEO_CONF_H
#define STM32L0XX_NUCLEO_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "stm32l0xx_hal.h"

/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32L0XX_NUCLEO
  * @{
  */

/** @defgroup STM32L0XX_NUCLEO_CONFIG Config
  * @{
  */

/** @defgroup STM32L0XX_NUCLEO_CONFIG_Exported_Constants
  * @{
  */
/* COM Feature define */
#define USE_BSP_COM_FEATURE                 0U

/* COM define */
#define USE_COM_LOG                         1U

/* IRQ priorities */
#define BSP_BUTTON_USER_IT_PRIORITY         15U

/* I2C1 Frequeny in Hz  */
#define BUS_I2C1_FREQUENCY                  100000U /* Frequency of I2C1 = 100 KHz*/

/* SPI1 Baud rate in bps  */
#define BUS_SPI1_BAUDRATE                   16000000U /* baud rate of SPIn = 16 Mbps */

/* UART1 Baud rate in bps  */
#define BUS_UART1_BAUDRATE                  9600U /* baud rate of UARTn = 9600 baud */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif
#endif  /* STM32L0XX_NUCLEO_CONF_H */
