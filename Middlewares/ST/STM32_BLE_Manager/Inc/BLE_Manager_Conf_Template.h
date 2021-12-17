/**
  ******************************************************************************
  * @file    BLE_Manager_Conf_Template.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   BLE Manager configuration template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Manager_Conf.h.
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
#ifndef __BLE_MANAGER_CONF_H__
#define __BLE_MANAGER_CONF_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Exported define -----------------------------------------------------------*/

/* Enable for BlueNRG-MS - As default BlueNRG-2 is enabled */
//#define BLUE_MS

/* Select the used hardware platform
 *
 * STEVAL-WESU1                         --> BLE_MANAGER_STEVAL_WESU1_PLATFORM
 * STEVAL-STLKT01V1 (SensorTile)        --> BLE_MANAGER_SENSOR_TILE_PLATFORM
 * STEVAL-BCNKT01V1 (BlueCoin)          --> BLE_MANAGER_BLUE_COIN_PLATFORM
 * STEVAL-IDB008Vx                      --> BLE_MANAGER_STEVAL_IDB008VX_PLATFORM
 * STEVAL-BCN002V1B (BlueTile)          --> BLE_MANAGER_STEVAL_BCN002V1_PLATFORM
 * STEVAL-MKSBOX1V1 (SensorTile.box)    --> BLE_MANAGER_SENSOR_TILE_BOX_PLATFORM
 * DISCOVERY-IOT01A                     --> BLE_MANAGER_DISCOVERY_IOT01A_PLATFORM
 * STEVAL-STWINKT1                      --> BLE_MANAGER_STEVAL_STWINKIT1_PLATFORM
 * STM32NUCLEO Board                    --> BLE_MANAGER_NUCLEO_PLATFORM
 *
 * Undefined platform                   --> BLE_MANAGER_UNDEF_PLATFORM
*/
  
/* Identify the used hardware platform  */
#define BLE_MANAGER_USED_PLATFORM	BLE_MANAGER_UNDEF_PLATFORM

/* Define the Max dimesion of the Bluetooth characteristics for each packet */
/* For BlueNRG-2 */
#define DEFAULT_MAX_CHAR_LEN 155
/* For BlueNRG-MS */
//#define DEFAULT_MAX_CHAR_LEN 20
   
#define BLE_MANAGER_MAX_ALLOCABLE_CHARS 16U

/* For enabling the capability to handle BlueNRG Congestion */
#define ACC_BLUENRG_CONGESTION
  
/* Define the Delay function to use inside the BLE Manager */
#define BLE_MANAGER_DELAY HAL_Delay
  
/****************** Malloc/Free **************************/
#define BLE_MallocFunction malloc
#define BLE_FreeFunction free
  
/*---------- Print messages from BLE Manager files at middleware level -------*/
/* Uncomment the following define for  enabling print debug messages */
#define BLE_MANAGER_DEBUG

#ifdef BLE_MANAGER_DEBUG
  /**
  * User can change here printf with a custom implementation.
  */

 #include <stdio.h>
 #define BLE_MANAGER_PRINTF(...)	printf(__VA_ARGS__)
 
 /* Define the Debug Level: 1/2/3(default value) */
 #define BLE_DEBUG_LEVEL 1
#else
  #define BLE_MANAGER_PRINTF(...)
#endif

/*---------------- Don't change the following defines ------------------------*/
//#define BLE_MANAGER_SDKV2

#ifdef __cplusplus
}
#endif

#endif /* __BLE_MANAGER_CONF_H__*/

