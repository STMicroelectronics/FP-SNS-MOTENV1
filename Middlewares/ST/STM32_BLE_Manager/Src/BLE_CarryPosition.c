/**
  ******************************************************************************
  * @file    BLE_CarryPosition.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   Add Carry Position service using vendor specific profiles.
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "BLE_Manager.h"
#include "BLE_ManagerCommon.h"

/* Private define ------------------------------------------------------------*/
#define COPY_CARRY_POSITION_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x08,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define CARRY_POSITION_ADVERTIZE_DATA_POSITION  18

/* Exported variables --------------------------------------------------------*/
BLE_NotifyEnv_t BLE_CarryPosition_NotifyEvent = BLE_NOTIFY_NOTHING;
CustomReadRequestCarryPosition_t CustomReadRequestCarryPosition=NULL;

/* Private variables ---------------------------------------------------------*/
/* Data structure pointer for Carry Position service */
static BleCharTypeDef BleCarryPosition;

/* Private functions ---------------------------------------------------------*/
static void AttrMod_Request_CarryPosition(void *BleCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data);
static void Read_Request_CarryPosition(void *BleCharPointer,uint16_t handle);

/**
 * @brief  Init Carry Position service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Carry Position service
 */
BleCharTypeDef* BLE_InitCarryPositionService(void)
{
  /* Data structure pointer for BLE service */
  BleCharTypeDef *BleCharPointer;

  /* Init data structure pointer for Carry Position info service */
  BleCharPointer = &BleCarryPosition;
  memset(BleCharPointer,0,sizeof(BleCharTypeDef));  
  BleCharPointer->AttrMod_Request_CB = AttrMod_Request_CarryPosition;
  BleCharPointer->Read_Request_CB= Read_Request_CarryPosition;
  COPY_CARRY_POSITION_CHAR_UUID((BleCharPointer->uuid));
  BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
  BleCharPointer->Char_Value_Length=2+1; /* 2 byte timestamp, 1 byte action */
  BleCharPointer->Char_Properties = ((uint8_t)CHAR_PROP_NOTIFY) | ((uint8_t)CHAR_PROP_READ);
  BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
  BleCharPointer->GATT_Evt_Mask=GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
  BleCharPointer->Enc_Key_Size=16;
  BleCharPointer->Is_Variable=0;
  
  if(CustomReadRequestCarryPosition == NULL) {
    BLE_MANAGER_PRINTF("Warning: Read request Carry Position function not defined\r\n");
  }
  
  BLE_MANAGER_PRINTF("BLE Carry Position features ok\r\n");
  
  return BleCharPointer;
}

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Carry Position Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
void BLE_SetCarryPositionAdvertizeData(uint8_t *manuf_data)
{
  manuf_data[CARRY_POSITION_ADVERTIZE_DATA_POSITION] |= 0x08U;
}
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Carry Position characteristic
 * @param  BLE_CP_output_t CarryPositionCode Carry Position Recognized
 * @retval tBleStatus   Status
 */
tBleStatus BLE_CarryPositionUpdate(BLE_CP_output_t CarryPositionCode)
{  
  tBleStatus ret;
  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = (uint8_t)CarryPositionCode;
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(&BleCarryPosition, 0, 2+1, buff);

  if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Carry Position Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Error Updating Carry Position Char\r\n");
    }
  }
  return ret;
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 *         With this function it's possible to understand if Carry Position is subscribed or not to the one service
 * @param  void *VoidCharPointer
 * @param  uint16_t attr_handle Handle of the attribute
 * @param  uint16_t Offset: (SoC mode) the offset is never used and it is always 0. Network coprocessor mode: 
 *                          - Bits 0-14: offset of the reported value inside the attribute.
 *                          - Bit 15: if the entire value of the attribute does not fit inside a single ACI_GATT_ATTRIBUTE_MODIFIED_EVENT event,
 *                            this bit is set to 1 to notify that other ACI_GATT_ATTRIBUTE_MODIFIED_EVENT events will follow to report the remaining value.                  
 * @param  uint8_t data_length length of the data
 * @param  uint8_t *att_data attribute data
 * @retval None
 */
static void AttrMod_Request_CarryPosition(void *VoidCharPointer, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_CarryPosition_NotifyEvent= BLE_NOTIFY_SUB;
  } else if (att_data[0] == 0U){
    BLE_CarryPosition_NotifyEvent= BLE_NOTIFY_UNSUB;
  }
 
#if (BLE_DEBUG_LEVEL>1)
 if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
   BytesToWrite =(uint8_t)sprintf((char *)BufferToWrite,"--->Carry Position=%s\n", (BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB) ? " ON" : " OFF");
   Term_Update(BufferToWrite,BytesToWrite);
 } else {
   BLE_MANAGER_PRINTF("--->Carry Position=%s", (BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB) ? " ON\r\n" : " OFF\r\n");
 }
#endif
}

/**
 * @brief  This event is given when a read request is received by the server from the client.
 * @param  void *VoidCharPointer
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
static void Read_Request_CarryPosition(void *VoidCharPointer,uint16_t handle)
{
  if(CustomReadRequestCarryPosition != NULL) {
    CustomReadRequestCarryPosition();
  }
}

