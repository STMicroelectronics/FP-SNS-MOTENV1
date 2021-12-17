/**
  ******************************************************************************
  * @file    BLE_GestureRecognition.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   Add Gesture Recognition service using vendor specific profiles.
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
#define COPY_GESTURE_REC_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define GESTURE_RECOGNITION_ADVERTIZE_DATA_POSITION  18

/* Exported variables --------------------------------------------------------*/
BLE_NotifyEnv_t BLE_GestureRecognition_NotifyEvent = BLE_NOTIFY_NOTHING;
CustomReadRequestGestureRecognition_t CustomReadRequestGestureRecognition=NULL;

/* Private variables ---------------------------------------------------------*/
/* Data structure pointer for Gesture Recognition service */
static BleCharTypeDef BleGestureRecognition;

/* Private functions ---------------------------------------------------------*/
static void AttrMod_Request_GestureRecognition(void *BleCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data);
static void Read_Request_GestureRecognition(void *BleCharPointer,uint16_t handle);

/**
 * @brief  Init Gesture Recognition service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for Gesture Recognition service
 */
BleCharTypeDef* BLE_InitGestureRecognitionService(void)
{
  /* Data structure pointer for BLE service */
  BleCharTypeDef *BleCharPointer;

  /* Init data structure pointer for Gesture Recognition info service */
  BleCharPointer = &BleGestureRecognition;
  memset(BleCharPointer,0,sizeof(BleCharTypeDef));  
  BleCharPointer->AttrMod_Request_CB = AttrMod_Request_GestureRecognition;
  BleCharPointer->Read_Request_CB= Read_Request_GestureRecognition;
  COPY_GESTURE_REC_CHAR_UUID((BleCharPointer->uuid));
  BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
  BleCharPointer->Char_Value_Length=2+1; /* 2 byte timestamp, 1 byte action */
  BleCharPointer->Char_Properties = ((uint8_t)CHAR_PROP_NOTIFY) | ((uint8_t)CHAR_PROP_READ);
  BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
  BleCharPointer->GATT_Evt_Mask=GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
  BleCharPointer->Enc_Key_Size=16;
  BleCharPointer->Is_Variable=0;
  
  if(CustomReadRequestGestureRecognition == NULL) {
    BLE_MANAGER_PRINTF("Warning: Read request Gesture Recognition function not defined\r\n");
  }
  
  BLE_MANAGER_PRINTF("BLE Gesture Recognition features ok\r\n");
  
  return BleCharPointer;
}

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting Gesture Recognition Advertise Data
 * @param  uint8_t *manuf_data: Advertise Data
 * @retval None
 */
void BLE_SetGestureRecognitionAdvertizeData(uint8_t *manuf_data)
{
  manuf_data[GESTURE_RECOGNITION_ADVERTIZE_DATA_POSITION] |= 0x02U;
}
#endif /* BLE_MANAGER_SDKV2 */

/**
 * @brief  Update Gesture Recognition characteristic
 * @param  BLE_GR_output_t GestureRecognitionCode Gesture Recognition Recognized
 * @retval tBleStatus   Status
 */
tBleStatus BLE_GestureRecognitionUpdate(BLE_GR_output_t GestureRecognitionCode)
{  
  tBleStatus ret;
  uint8_t buff[2+1];

  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  buff[2] = (uint8_t)GestureRecognitionCode;
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(&BleGestureRecognition, 0, 2+1, buff);

  if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Gesture Recognition Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Error Updating Gesture Recognition Char\r\n");
    }
  }
  return ret;
}

/**
 * @brief  This function is called when there is a change on the gatt attribute
 *         With this function it's possible to understand if Gesture Recognition is subscribed or not to the one service
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
static void AttrMod_Request_GestureRecognition(void *VoidCharPointer, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_GestureRecognition_NotifyEvent= BLE_NOTIFY_SUB;
  } else if (att_data[0] == 0U){
    BLE_GestureRecognition_NotifyEvent= BLE_NOTIFY_UNSUB;
  }
 
#if (BLE_DEBUG_LEVEL>1)
 if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
   BytesToWrite =(uint8_t)sprintf((char *)BufferToWrite,"--->Gesture Recognition=%s\n", (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB) ? " ON" : " OFF");
   Term_Update(BufferToWrite,BytesToWrite);
 } else {
   BLE_MANAGER_PRINTF("--->Gesture Recognition=%s", (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB) ? " ON\r\n" : " OFF\r\n");
 }
#endif
}

/**
 * @brief  This event is given when a read request is received by the server from the client.
 * @param  void *VoidCharPointer
 * @param  uint16_t handle Handle of the attribute
 * @retval None
 */
static void Read_Request_GestureRecognition(void *VoidCharPointer,uint16_t handle)
{
  if(CustomReadRequestGestureRecognition != NULL) {
    CustomReadRequestGestureRecognition();
  }
}

