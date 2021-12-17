/**
  ******************************************************************************
  * @file    BLE_GestureRecognition.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   Gesture Recognition info service APIs.
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
#ifndef _BLE_GESTURE_RECOGNITION_H_
#define _BLE_GESTURE_RECOGNITION_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef void (*CustomReadRequestGestureRecognition_t)(void);

typedef enum
{
  BLE_GR_NOGESTURE       = 0x00,
  BLE_GR_PICKUP          = 0x01,
  BLE_GR_GLANCE          = 0x02,
  BLE_GR_WAKEUP          = 0x03
} BLE_GR_output_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_GestureRecognition_NotifyEvent;
extern CustomReadRequestGestureRecognition_t CustomReadRequestGestureRecognition;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Gesture Recognition info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Gesture Recognition info service
 */
extern BleCharTypeDef* BLE_InitGestureRecognitionService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Gesture Recognition Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
extern void BLE_SetGestureRecognitionAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Gesture Recognition characteristic
 * @param  BLE_GR_output_t GestureRecognitionCode Gesture Recognition Recognized
 * @retval tBleStatus   Status
 */
extern tBleStatus BLE_GestureRecognitionUpdate(BLE_GR_output_t GestureRecognitionCode);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_GESTURE_RECOGNITION_H_ */

