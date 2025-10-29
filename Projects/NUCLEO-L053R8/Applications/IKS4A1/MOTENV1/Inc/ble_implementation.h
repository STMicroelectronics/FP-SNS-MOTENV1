/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_implementation.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   BLE Implementation header template file.
  *          This file should be copied to the application folder and renamed
  *          to ble_implementation.h.
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
#ifndef _BLE_IMPLEMENTATION_H_
#define _BLE_IMPLEMENTATION_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/**
  * User can added here the header file for the selected BLE features.
  * For example:
  * #include "ble_environmental.h"
  * #include "ble_inertial.h"
  */

#include "ble_acc_event.h"
#include "ble_environmental.h"
#include "ble_inertial.h"
#include "ble_led.h"

/* Exported Defines --------------------------------------------------------*/
#define STM32L0xx

/* Enable/Disable BlueNRG config extend services */
#define ENABLE_EXT_CONFIG      0
/* Enable/Disable BlueNRG config services */
#define ENABLE_CONFIG      1
/* Enable/Disable Secure Connection */
#define ENABLE_SECURE_CONNECTION      0
/* Number of audio channels (Max audio channels 4) */
#define AUDIO_CHANNELS_NUMBER      1
/* Number of the general purpose features to use */
#define NUM_GENERAL_PURPOSE      3
/* Enable/Disable magnetometer data (Disable= 0- Enable=1) */
#define ENABLE_MAG_DATA      1
/* Secure PIN */
#define SECURE_PIN      123456
/* Audio Scene Classificatio algorithm code */
#define ALGORITHM_CODE      0
/* variable/Fixed length flag */
#define CHAR_SIZE_VARIABLE      0
/* Enable/Disable pressure data (Disable= 0- Enable=1) */
#define ENABLE_ENV_PRESSURE_DATA      1
/* Max Dimension of the BLE Char */
#define MAX_CHAR_SIZE      3
/* Enable/Disable Random Secure PIN */
#define ENABLE_RANDOM_SECURE_PIN      0
/* Enable/Disable BlueNRG console services */
#define ENABLE_CONSOLE      1
/* Enable/Disable giroscope data (Disable= 0- Enable=1) */
#define ENABLE_GYRO_DATA      1
/* Size of the general purpose feature */
#define GENERAL_PURPOSE_SIZE_3      1
/* Size of the general purpose feature */
#define GENERAL_PURPOSE_SIZE_2      1
/* Enable/Disable humidity data (Disable= 0- Enable=1) */
#define ENABLE_ENV_HUMIDITY_DATA      1
/* Size of the general purpose feature */
#define GENERAL_PURPOSE_SIZE_1      1
/* Number of quaternion to send (max value 3) */
#define NUMBER_OF_QUATERNION      1
/* Enable/Disable number of temperature (0, 1, 2) */
#define ENABLE_ENV_TEMPERATURE_DATA      2
/* Supported hardware platform */
#define USED_PLATFORM      0x7DU
/* Enable/Disable accelerometer data (Disable= 0- Enable=1) */
#define ENABLE_ACC_DATA      1
/* Select the used hardware platform
 *
 * STEVAL-WESU1                         --> BLE_MANAGER_STEVAL_WESU1_PLATFORM
 * STEVAL-STLKT01V1 (SensorTile)        --> BLE_MANAGER_SENSOR_TILE_PLATFORM
 * STEVAL-BCNKT01V1 (BlueCoin)          --> BLE_MANAGER_BLUE_COIN_PLATFORM
 * STEVAL-IDB008Vx                      --> BLE_MANAGER_STEVAL_IDB008VX_PLATFORM
 * STEVAL-BCN002V1B (BlueTile)          --> BLE_MANAGER_STEVAL_BCN002V1_PLATFORM
 * STEVAL-MKSBOX1V1 (SensorTile.box)    --> BLE_MANAGER_SENSOR_TILE_BOX_PLATFORM
 * DISCOVERY-IOT01A                     --> BLE_MANAGER_DISCOVERY_IOT01A_PLATFORM
 * STEVAL-STWINKT1                      --> BLE_MANAGER_STEVAL_STWINKT1_PLATFORM
 * STEVAL-STWINKT1B                     --> BLE_MANAGER_STEVAL_STWINKT1B_PLATFORM
 * STEVAL_STWINBX1                      --> BLE_MANAGER_STEVAL_STWINBX1_PLATFORM
 * SENSOR_TILE_BOX_PRO                  --> BLE_MANAGER_SENSOR_TILE_BOX_PRO_PLATFORM
 * STEVAL_ASTRA1                        --> BLE_MANAGER_STEVAL_ASTRA1_PLATFORM
 * STM32NUCLEO Board                    --> BLE_MANAGER_NUCLEO_PLATFORM
 * STM32U5A5ZJ_NUCLEO Board             --> BLE_MANAGER_STM32U5A5ZJ_NUCLEO_PLATFORM
 * STM32U575ZI_NUCLEO Board             --> BLE_MANAGER_STM32U575ZI_NUCLEO_PLATFORM
 * STM32F446RE_NUCLEO Board             --> BLE_MANAGER_STM32F446RE_NUCLEO_PLATFORM
 * STM32L053R8_NUCLEO Board             --> BLE_MANAGER_STM32L053R8_NUCLEO_PLATFORM
 * STM32L476RG_NUCLEO Board             --> BLE_MANAGER_STM32L476RG_NUCLEO_PLATFORM
 * STM32F401RE_NUCLEO Board             --> BLE_MANAGER_STM32F401RE_NUCLEO_PLATFORM
 * Not defined platform                 --> BLE_MANAGER_UNDEF_PLATFORM
 *
 * For example:
 * #define BLE_MANAGER_USED_PLATFORM  BLE_MANAGER_NUCLEO_PLATFORM
 *
*/

/* Used platform */
#define BLE_MANAGER_USED_PLATFORM       USED_PLATFORM

/* STM32 Unique ID */
#define BLE_STM32_UUID          UID_BASE

/* STM32 MCU_ID */
#ifdef DBGMCU_BASE
#define BLE_STM32_MCU_ID        ((uint32_t *)DBGMCU_BASE)
#else /* DBGMCU_BASE */
#define BLE_STM32_MCU_ID        ((uint32_t *)0x00000000UL)
#endif /* DBGMCU_BASE */

/* STM32  Microcontrolles type */
#define BLE_STM32_MICRO "STM32L0xx"

/* USER CODE BEGIN 1 */

/* Package Version firmware */
#define BLE_VERSION_FW_MAJOR  '4'
#define BLE_VERSION_FW_MINOR  '1'
#define BLE_VERSION_FW_PATCH  '0'

/* Firmware Package Name */
#define BLE_FW_PACKAGENAME    "X-CUBE-BLEMGR"

/* USER CODE END 1 */

/* Exported Variables ------------------------------------------------------- */

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* Exported functions ------------------------------------------------------- */
extern void ble_init_custom_service(void);
extern void ble_set_custom_advertise_data(uint8_t *manuf_data);
extern void bluetooth_init(void);
extern void set_board_name(void);

#if (BLUE_CORE != BLUE_WB)
extern void reset_ble_manager(void);
#endif /* (BLUE_CORE != BLUE_WB) */

/* USER CODE BEGIN 3 */

/* USER CODE END 3 */

#ifdef __cplusplus
}
#endif

#endif /* _BLE_IMPLEMENTATION_H_ */

