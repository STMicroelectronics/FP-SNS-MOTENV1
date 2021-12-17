/**
  ******************************************************************************
  * @file    BLE_CarryPosition.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   Carry Position info service APIs.
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
#ifndef _BLE_CARRY_POSITION_H_
#define _BLE_CARRY_POSITION_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/

/* Exported typedef --------------------------------------------------------- */
typedef void (*CustomReadRequestCarryPosition_t)(void);

typedef enum
{
  BLE_CP_UNKNOWN             = 0x00,
  BLE_CP_ONDESK              = 0x01,
  BLE_CP_INHAND              = 0x02,
  BLE_CP_NEARHEAD            = 0x03,
  BLE_CP_SHIRTPOCKET         = 0x04,
  BLE_CP_TROUSERPOCKET       = 0x05,
  BLE_CP_ARMSWING            = 0x06,
  BLE_CP_JACKETPOCKET        = 0x07
} BLE_CP_output_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_CarryPosition_NotifyEvent;
extern CustomReadRequestCarryPosition_t CustomReadRequestCarryPosition;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init Carry Position info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Carry Position info service
 */
extern BleCharTypeDef* BLE_InitCarryPositionService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Carry Position Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
extern void BLE_SetCarryPositionAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Carry Position characteristic
 * @param  BLE_CP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus   Status
 */
extern tBleStatus BLE_CarryPositionUpdate(BLE_CP_output_t CarryPositionCode);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_CARRY_POSITION_H_ */

