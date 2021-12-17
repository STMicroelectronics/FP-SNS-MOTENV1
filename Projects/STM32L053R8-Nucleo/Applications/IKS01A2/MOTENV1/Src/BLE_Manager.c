/**
  ******************************************************************************
  * @file    BLE_Manager.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Add bluetooth services using vendor specific profiles.
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

/* Includes ---------------------------------------------------------------------*/
#include <stdio.h>
#include "BLE_Manager.h"
#include "BLE_ManagerCommon.h"

/* Private define ---------------------------------------------------------------*/
#ifndef MIN
#define MIN(a,b)            ((a) < (b) )? (a) : (b)
#endif

/* Max Number of Bonded Devices */
#define BLE_MANAGER_MAX_BONDED_DEVICES 3

/* Length of AdvData in octets */
#define ADVERTIZE_DATA_LENGHT 25

/* Hardware & Software Characteristics Service */
#define COPY_FEATURES_SERVICE_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_EXT_CONFIG_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x14,0x00,0x02,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Console Service */
#define COPY_CONSOLE_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0E,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_TERM_CHAR_UUID(uuid_struct)        COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x01,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_STDERR_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0E,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Configuration Service */
#define COPY_CONFIG_SERVICE_UUID(uuid_struct)   COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x0F,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CONFIG_CHAR_UUID(uuid_struct)      COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x02,0x00,0x0F,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Exported variables -----------------------------------------------------------*/
/* Identifies if the configuration service are enabled or not */
BLE_ServEnab_t BLE_Conf_Service;

/* Identifies if the standard terminal service are enabled or not */
BLE_ServEnab_t BLE_StdTerm_Service;

/* Identifies if the standard error message service are enabled or not */
BLE_ServEnab_t BLE_StdErr_Service;

/* Identifies if the Extended Configuration characteristic value service are enabled or not */
BLE_ServEnab_t BLE_ExtConf_Service;

/* Contains the message of the standard terminal or of the standard error */
uint8_t BufferToWrite[256];

/* Total number of characters written for the message in the standard terminal or for the standard error */
uint8_t BytesToWrite;

uint8_t set_connectable;

/* BlueNRG Stack */
BlueNRG_StackTypeDef BlueNRG_StackValue;

/**************** Bluetooth Comunication *************************/
CustomPairingCompleted_t                CustomPairingCompleted;
CustomSetConnectable_t                  CustomSetConnectable;
CustomConnectionCompleted_t             CustomConnectionCompleted;
CustomDisconnectionCompleted_t          CustomDisconnectionCompleted;
CustomAciGattTxPoolAvailableEvent_t     CustomAciGattTxPoolAvailableEvent;

/**************** Debug Console *************************/
CustomDebugConsoleParsing_t CustomDebugConsoleParsingCallback;

/******************* Config Char *************************/
CustomAttrModConfig_t CustomAttrModConfigCallback;
CustomWriteRequestConfig_t CustomWriteRequestConfigCallback;

static uint8_t LastStderrBuffer[DEFAULT_MAX_CHAR_LEN];
static uint8_t LastStderrLen;
static uint8_t LastTermBuffer[DEFAULT_MAX_CHAR_LEN];
static uint8_t LastTermLen;
static uint16_t connection_handle;

static uint8_t MaxBLECharLen;

static BleCharTypeDef BleCharConfig;
static BleCharTypeDef BleCharStdOut;
static BleCharTypeDef BleCharStdErr;

static BleCharTypeDef *BleCharsArray[BLE_MANAGER_MAX_ALLOCABLE_CHARS];
static uint8_t UsedBleChars;
static uint8_t UsedStandardBleChars;

/* Private functions prototype --------------------------------------------------*/
static void APP_UserEvtRx(void *pData);
static void UpdateWhiteList(void);

static tBleStatus InitBleManager_BlueNRG_Stack(void);

static tBleStatus InitBleManagerServices(void);

static tBleStatus BLE_Manager_AddFeaturesService(void);
static tBleStatus BLE_Manager_AddConsoleService(void);
static tBleStatus BLE_Manager_AddConfigService(void);

static tBleStatus UpdateTermStdOut(uint8_t *data,uint8_t length);
static tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length);

static void Read_Request_StdErr(void *VoidCharPointer,uint16_t handle);
static void Read_Request_Term(void *VoidCharPointer,uint16_t handle);

/* Private functions ------------------------------------------------------------*/
/**
* @brief  Add the Config service using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
static tBleStatus BLE_Manager_AddConfigService(void)
{
  tBleStatus ret;
  
  Service_UUID_t service_uuid;
  Char_UUID_t char_uuid;
  
  uint8_t uuid[16];
  
  COPY_CONFIG_SERVICE_UUID(uuid);
  
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+3,&(BleCharConfig.Service_Handle));
  
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
    goto EndLabel;
  }
  
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, BleCharConfig.uuid, 16);

  ret =  aci_gatt_add_char(BleCharConfig.Service_Handle,
                           BleCharConfig.Char_UUID_Type,					   
                           &char_uuid,
                           BleCharConfig.Char_Value_Length,
                           BleCharConfig.Char_Properties,
                           BleCharConfig.Security_Permissions,
                           BleCharConfig.GATT_Evt_Mask,
                           BleCharConfig.Enc_Key_Size,
                           BleCharConfig.Is_Variable,
                           &(BleCharConfig.attr_handle));
  
EndLabel:
  return ret;
}


/**
* @brief  Add the Console service using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
static tBleStatus BLE_Manager_AddConsoleService(void)
{
  tBleStatus ret;
  
  Service_UUID_t service_uuid;
  Char_UUID_t char_uuid;
  
  uint8_t uuid[16];
  
  COPY_CONSOLE_SERVICE_UUID(uuid);
 
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, 1+(3*2),&(BleCharStdOut.Service_Handle));
  
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
    goto EndLabel;
  }  
  
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, BleCharStdOut.uuid, 16);

  ret =  aci_gatt_add_char(BleCharStdOut.Service_Handle,
                           BleCharStdOut.Char_UUID_Type, 						   
                           &char_uuid,
                           BleCharStdOut.Char_Value_Length,
                           BleCharStdOut.Char_Properties,
                           BleCharStdOut.Security_Permissions,
                           BleCharStdOut.GATT_Evt_Mask,
                           BleCharStdOut.Enc_Key_Size,
                           BleCharStdOut.Is_Variable,
                           &(BleCharStdOut.attr_handle));
  
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
    goto EndLabel;
  }
  
  BleCharStdErr.Service_Handle = BleCharStdOut.Service_Handle;
  
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, BleCharStdErr.uuid, 16);

  ret =  aci_gatt_add_char(BleCharStdErr.Service_Handle,
                           BleCharStdErr.Char_UUID_Type, 						   
                           &char_uuid,
                           BleCharStdErr.Char_Value_Length,
                           BleCharStdErr.Char_Properties,
                           BleCharStdErr.Security_Permissions,
                           BleCharStdErr.GATT_Evt_Mask,
                           BleCharStdErr.Enc_Key_Size,
                           BleCharStdErr.Is_Variable,
                           &(BleCharStdErr.attr_handle));
  
EndLabel:
  return ret;
}

/**
* @brief  Update Stdout characteristic value (when lenght is <=MaxBLECharLen)
* @param  uint8_t *data string to write
* @param  uint8_t lenght lengt of string to write
* @retval tBleStatus      Status
*/
static tBleStatus UpdateTermStdOut(uint8_t *data,uint8_t length)
{
  if (ACI_GATT_UPDATE_CHAR_VALUE(&BleCharStdOut, 0, length , data) != (tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: Updating Stdout Char\r\n");
    return BLE_STATUS_ERROR;
  }
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Update Stderr characteristic value (when lenght is <=MaxBLECharLen)
* @param  uint8_t *data string to write
* @param  uint8_t lenght lengt of string to write
* @retval tBleStatus      Status
*/
static tBleStatus UpdateTermStdErr(uint8_t *data,uint8_t length)
{
  if (ACI_GATT_UPDATE_CHAR_VALUE(&BleCharStdErr, 0, length , data) != (tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: Updating Stdout Char\r\n");
    return BLE_STATUS_ERROR;
  } 
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Update Stderr characteristic value after a read request
* @param None
* @retval tBleStatus Status
*/
static tBleStatus Stderr_Update_AfterRead(BleCharTypeDef *BleCharPointer)
{
  tBleStatus ret;
  ret = ACI_GATT_UPDATE_CHAR_VALUE(BleCharPointer, 0, LastStderrLen , LastStderrBuffer);
  return ret;
}

/**
* @brief  Update Terminal characteristic value after a read request
* @param None
* @retval tBleStatus Status
*/
static tBleStatus Term_Update_AfterRead(void *BleCharPointer)
{
  tBleStatus ret;
  ret = ACI_GATT_UPDATE_CHAR_VALUE(BleCharPointer, 0, LastTermLen , LastTermBuffer);
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Stdout Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Error: Updating Stdout Char\r\n");
    }
  }
  return ret;
}

/**
* @brief  This event is given when a read request is received by the server from the client.
* @param  void *VoidCharPointer
* @param  uint16_t handle Handle of the attribute
* @retval None
*/
static void Read_Request_StdErr(void *VoidCharPointer,uint16_t handle)
{
  BleCharTypeDef *BleCharPointer = (BleCharTypeDef *) VoidCharPointer;
  
  /* Send again the last packet for StdError */
  Stderr_Update_AfterRead(BleCharPointer);
#if (BLE_DEBUG_LEVEL>1)
  BLE_MANAGER_PRINTF("Read for StdErr\r\n");
#endif
  
}

/**
* @brief  This event is given when a read request is received by the server from the client.
* @param  void *VoidCharPointer
* @param  uint16_t handle Handle of the attribute
* @retval None
*/
static void Read_Request_Term(void *VoidCharPointer,uint16_t handle)
{
  BleCharTypeDef *BleCharPointer = (BleCharTypeDef *) VoidCharPointer;
  
  /* Send again the last packet for StdError */
  Term_Update_AfterRead(BleCharPointer);
#if (BLE_DEBUG_LEVEL>1)
  BLE_MANAGER_PRINTF("Read for Term\r\n");
#endif
}

/**
* @brief  This function is called when there is a change on the gatt attribute
*         With this function it's possible to understand if config is subscribed or not to the one service
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
static void AttrMod_Request_Config(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == (uint8_t)01) {
    BLE_Conf_Service= BLE_SERV_ENABLE;
  } else if (att_data[0] == 0U){
    BLE_Conf_Service= BLE_SERV_NOT_ENABLE;
  }
  
  if(CustomAttrModConfigCallback!=NULL) {
    CustomAttrModConfigCallback(att_data,data_length);
  }
  
#if (BLE_DEBUG_LEVEL>1)
  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
    BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite,"--->Conf=%s\n", (BLE_Conf_Service == BLE_SERV_ENABLE) ? "ON" : "OFF");
    Term_Update(BufferToWrite,BytesToWrite);
  } else {
    BLE_MANAGER_PRINTF("--->Conf=%s\r\n", (BLE_Conf_Service == BLE_SERV_ENABLE) ? "ON" : "OFF");
  }
#endif
}

/**
* @brief  This function is called when there is a change on the gatt attribute as consequence of write request for the Config service
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
static void Write_Request_Config(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{ 
  /* Received one write command from Client on Configuration characteristc */
  if(CustomWriteRequestConfigCallback!=NULL) {
    CustomWriteRequestConfigCallback(att_data,data_length);
  }
}

/**
* @brief  This function is called when there is a change on the gatt attribute
*         With this function it's possible to understand if Std Err is subscribed or not to the one service
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
static void AttrMod_Request_StdErr(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_StdErr_Service= BLE_SERV_ENABLE;
  } else if (att_data[0] == 0U){
    BLE_StdErr_Service= BLE_SERV_NOT_ENABLE;
  }
}

/**
* @brief  This function is called when there is a change on the gatt attribute
*         With this function it's possible to understand if Term is subscribed or not to the one service
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
static void AttrMod_Request_Term(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_StdTerm_Service= BLE_SERV_ENABLE;
  } else if (att_data[0] == 0U){
    BLE_StdTerm_Service= BLE_SERV_NOT_ENABLE;
  }
}

/**
* @brief  This function is called when there is a change on the gatt attribute as consequence of write request for the Term service
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
static void Write_Request_Term(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  /* By default Answer with the same message received */
  uint32_t SendBackData =1; 
  
  /* Received one write from Client on Terminal characteristc */
  if(CustomDebugConsoleParsingCallback!=NULL) {
    SendBackData = CustomDebugConsoleParsingCallback(att_data,data_length);
  }
  
  /* Send it back if it's necessary */
  if(SendBackData) {
    Term_Update(att_data,data_length);
  }
}

/**
* @brief  Add the HW Features service using a vendor specific profile
* @param  None
* @retval tBleStatus Status
*/
static tBleStatus BLE_Manager_AddFeaturesService(void)
{
  tBleStatus ret;
   
  Service_UUID_t service_uuid;
  Char_UUID_t char_uuid;
  
  uint8_t BleChar;
  uint8_t uuid[16];
  uint16_t Service_Handle;
  uint8_t NumberCustomBLEChars = UsedBleChars-UsedStandardBleChars;
  
  COPY_FEATURES_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128,  &service_uuid, PRIMARY_SERVICE, (1U+(3U*NumberCustomBLEChars)),&Service_Handle);
  
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
    goto EndLabel;
  }
  
  for(BleChar=UsedStandardBleChars;BleChar<UsedBleChars;BleChar++) {
    BleCharsArray[BleChar]->Service_Handle = Service_Handle;

    BLUENRG_memcpy(&char_uuid.Char_UUID_128, BleCharsArray[BleChar]->uuid, 16);

    ret =  aci_gatt_add_char(BleCharsArray[BleChar]->Service_Handle,
                             BleCharsArray[BleChar]->Char_UUID_Type,							 
                             &char_uuid,
                             BleCharsArray[BleChar]->Char_Value_Length,
                             BleCharsArray[BleChar]->Char_Properties,
                             BleCharsArray[BleChar]->Security_Permissions,
                             BleCharsArray[BleChar]->GATT_Evt_Mask,
                             BleCharsArray[BleChar]->Enc_Key_Size,
                             BleCharsArray[BleChar]->Is_Variable,
                             &(BleCharsArray[BleChar]->attr_handle));
    
    if (ret != (tBleStatus)BLE_STATUS_SUCCESS) {
      goto EndLabel;
    }
  }
  
EndLabel:
  return ret;
}

#ifdef ACC_BLUENRG_CONGESTION
static int32_t breath=0;

/* @brief  Update the value of a characteristic avoiding (for a short time) to
*         send the next updates if an error in the previous sending has
*         occurred.
* @param  BleCharPointer pointer to the BleCharTypeDef for the current ble char
* @param  charValOffset The offset of the characteristic
* @param  charValueLen The length of the characteristic
* @param  charValue The pointer to the characteristic
* @retval tBleStatus Status
*/
tBleStatus safe_aci_gatt_update_char_value(BleCharTypeDef *BleCharPointer,
                                           uint8_t charValOffset,
                                           uint8_t charValueLen,
                                           uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  if (breath==0){
    ret = aci_gatt_update_char_value(BleCharPointer->Service_Handle,BleCharPointer->attr_handle,charValOffset,charValueLen,charValue);
    
    if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
#if (BLE_DEBUG_LEVEL>2)
      BLE_MANAGER_PRINTF("Error: Updating Char handle=%x ret=%x\r\n",BleCharPointer->attr_handle,ret);
#endif
      if(ret==(tBleStatus)BLE_STATUS_INSUFFICIENT_RESOURCES){
#if (BLE_DEBUG_LEVEL>2)
        BLE_MANAGER_PRINTF("Char handle=%x insufficient resources\r\n",BleCharPointer->attr_handle);
#endif
        breath = 1;
      }
    }
  }
  return ret;
}
#endif /* ACC_BLUENRG_CONGESTION */

/**
* @brief Each time BLE FW stack raises the error code @ref ble_status_insufficient_resources (0x64),
the @ref aci_gatt_tx_pool_available_event event is generated as soon as the available buffer size 
is greater than maximum ATT MTU (on stack versions below v2.1 this event is generated when at least 2 packets
with MTU of 23 bytes are available).
* @param Connection_Handle Connection handle related to the request
* @param Available_Buffers Not used.
* @retval None
*/
void aci_gatt_tx_pool_available_event(uint16_t Connection_Handle,
                                      uint16_t Available_Buffers)
{
#if (BLE_DEBUG_LEVEL>1)
  BLE_MANAGER_PRINTF("aci_gatt_tx_pool_available_event\r\n");
#endif
  
#ifdef ACC_BLUENRG_CONGESTION
  breath=0;
#endif /* ACC_BLUENRG_CONGESTION */
  
  if(CustomAciGattTxPoolAvailableEvent != NULL) {
    CustomAciGattTxPoolAvailableEvent();
  }
}

/* @brief  Update the value of a characteristic
* @param  BleCharPointer pointer to the BleCharTypeDef for the current ble char
* @param  charValOffset The offset of the characteristic
* @param  charValueLen The length of the characteristic
* @param  charValue The pointer to the characteristic
* @retval tBleStatus Status
*/
tBleStatus aci_gatt_update_char_value_wrapper(BleCharTypeDef *BleCharPointer,
                                              uint8_t charValOffset,
                                              uint8_t charValueLen,
                                              uint8_t *charValue)
{
  tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
  ret = aci_gatt_update_char_value(BleCharPointer->Service_Handle,BleCharPointer->attr_handle,charValOffset,charValueLen,charValue);
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
#if (BLE_DEBUG_LEVEL>2)
    BLE_MANAGER_PRINTF("Error: Updating Char handle=%x ret=%x\r\n",BleCharPointer->attr_handle,ret);
#endif
  }
  return ret;
}

/**
* @brief  Update Stderr characteristic value
* @param  uint8_t *data string to write
* @param  uint8_t lenght lengt of string to write
* @retval tBleStatus      Status
*/
tBleStatus Stderr_Update(uint8_t *data,uint8_t length)
{
  uint8_t Offset;
  uint8_t DataToSend;
  /* Split the code in Chunks */
  /* First Chunk */
  DataToSend = (length>MaxBLECharLen) ?  MaxBLECharLen : length;
  
  /* keep a copy */
  memcpy(LastStderrBuffer,data,DataToSend);
  LastStderrLen = DataToSend;
  
  if(UpdateTermStdErr(data,DataToSend)!=(tBleStatus)BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }
  
  /* Following Chunks if necessary */
  Offset = MaxBLECharLen;
  for(; Offset<length; Offset +=MaxBLECharLen){
    /* Add a Delay respect previous chunk */
    BLE_MANAGER_DELAY(20);
    
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>MaxBLECharLen) ?  MaxBLECharLen : DataToSend;
    
    /* keep a copy */
    memcpy(LastStderrBuffer,data+Offset,DataToSend);
    LastStderrLen = DataToSend;
    
    if(UpdateTermStdErr(data+Offset,DataToSend)!=(tBleStatus)BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  
* @param  uint8_t* buffer
* @param  uint8_t len
* @retval tBleStatus   Status
*/
tBleStatus BLE_StdOutSendBuffer(uint8_t* buffer, uint8_t len)
{
  tBleStatus ret;
  
  ret = aci_gatt_update_char_value(BleCharStdOut.Service_Handle, BleCharStdOut.attr_handle, 0, len, buffer);
  
  return ret;
}

/**
* @brief  Update Terminal characteristic value
* @param  uint8_t *data string to write
* @param  uint8_t lenght lengt of string to write
* @retval tBleStatus      Status
*/
tBleStatus Term_Update(uint8_t *data,uint8_t length)
{
  uint8_t   Offset;
  uint8_t   DataToSend;
  
  /* Split the code in Chunks */
  /* First Chunk */
  DataToSend = (length>MaxBLECharLen) ?  MaxBLECharLen : length;
  
  /* keep a copy */
  memcpy(LastTermBuffer,data,DataToSend);
  LastTermLen = DataToSend;
  
  if(UpdateTermStdOut(data,DataToSend)!=(tBleStatus)BLE_STATUS_SUCCESS) {
    return BLE_STATUS_ERROR;
  }
  
  /* Following Chunks if necessary */
  Offset = MaxBLECharLen;
  for(; Offset<length; Offset +=MaxBLECharLen){
    
    /* Add a Delay respect previous chunk */
    BLE_MANAGER_DELAY(20);
    
    DataToSend = (length-Offset);
    DataToSend = (DataToSend>MaxBLECharLen) ?  MaxBLECharLen : DataToSend;
    
    /* keep a copy */
    memcpy(LastTermBuffer,data+Offset,DataToSend);
    LastTermLen = DataToSend;
    
    if(UpdateTermStdOut(data+Offset,DataToSend)!=(tBleStatus)BLE_STATUS_SUCCESS) {
      return BLE_STATUS_ERROR;
    }
  }
  return BLE_STATUS_SUCCESS;
}

/* @brief  Send a BLE notification for answering to a configuration command for Accelerometer events
* @param  uint32_t Feature Feature type
* @param  uint8_t Command Replay to this Command
* @param  uint8_t data result to send back
* @retval tBleStatus Status
*/
tBleStatus Config_Update(uint32_t Feature,uint8_t Command,uint8_t data)
{
  uint8_t buff[2+4+1+1];
  
  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7] = data;
  
  if (ACI_GATT_UPDATE_CHAR_VALUE(&BleCharConfig, 0, 8 , buff) != (tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: Updating Configuration Char\r\n");
    return BLE_STATUS_ERROR;
  } 
  return BLE_STATUS_SUCCESS;
  
}

/* @brief  Send a BLE notification for answering to a configuration command for Accelerometer events
* @param  uint32_t Feature Feature type
* @param  uint8_t Command Replay to this Command
* @param  uint32_t data result to send back
* @retval tBleStatus Status
*/
tBleStatus Config_Update_32(uint32_t Feature,uint8_t Command,uint32_t data)
{
  uint8_t buff[2+4+1+4];
  
  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  STORE_BE_32(buff+2,Feature);
  buff[6] = Command;
  buff[7]  = (uint8_t)((data       ) & 0xFFU);
  buff[8]  = (uint8_t)((data >>  8U) & 0xFFU);
  buff[9]  = (uint8_t)((data >> 16U) & 0xFFU);
  buff[10] = (uint8_t)((data >> 24U) & 0xFFU);
  
  if (ACI_GATT_UPDATE_CHAR_VALUE(&BleCharConfig, 0, 11 , buff) != (tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: Updating Configuration Char\r\n");
    return BLE_STATUS_ERROR;
  } 
  return BLE_STATUS_SUCCESS;
}

/**
* @brief  Puts the device in connectable mode.
* @param  None
* @retval None
*/
void setConnectable(void)
{
  uint8_t local_name[8] = {AD_TYPE_COMPLETE_LOCAL_NAME,
  BlueNRG_StackValue.BoardName[0],
  BlueNRG_StackValue.BoardName[1],
  BlueNRG_StackValue.BoardName[2],
  BlueNRG_StackValue.BoardName[3],
  BlueNRG_StackValue.BoardName[4],
  BlueNRG_StackValue.BoardName[5],
  BlueNRG_StackValue.BoardName[6]};
  tBleStatus RetStatus= BLE_STATUS_SUCCESS; 
  
  uint8_t manuf_data[ADVERTIZE_DATA_LENGHT] = {
    8U,0x09U,
    BlueNRG_StackValue.BoardName[0],           /* Complete Name */
    BlueNRG_StackValue.BoardName[1],
    BlueNRG_StackValue.BoardName[2],
    BlueNRG_StackValue.BoardName[3],
    BlueNRG_StackValue.BoardName[4],
    BlueNRG_StackValue.BoardName[5],
    BlueNRG_StackValue.BoardName[6],            
    15U,0xFFU,
    0x30U,0x00U,                                  /* STM Manufacter AD */
#ifdef BLE_MANAGER_SDKV2
    0x02U,                                       /* SDK version V2 */
#else /* BLE_MANAGER_SDKV2 */
    0x01U,                                       /* SDK version V1 */
#endif /* BLE_MANAGER_SDKV2 */
    BLE_MANAGER_USED_PLATFORM,                  /* BoardType */
    0x00U,
    0x00U,
    0x00U,
    0x00U,
    BlueNRG_StackValue.BleMacAddress[5],        /* BLE MAC start */
    BlueNRG_StackValue.BleMacAddress[4],
    BlueNRG_StackValue.BleMacAddress[3],
    BlueNRG_StackValue.BleMacAddress[2],
    BlueNRG_StackValue.BleMacAddress[1],
    BlueNRG_StackValue.BleMacAddress[0],        /* BLE MAC stop */
  };
  
  /* Set the Custom BLE Advertise Data */
  BLE_SetCustomAdvertizeData(manuf_data);
  
  /* disable scan response */
  RetStatus = hci_le_set_scan_response_data(0U,NULL);
  if(RetStatus !=(tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: hci_le_set_scan_response_data [%x]\r\n",RetStatus);
    goto EndLabel;
  }
  
  /* Set the board discoverable */
  if(BlueNRG_StackValue.AdvertisingFilter == ((uint8_t)NO_WHITE_LIST_USE)) {
    RetStatus = aci_gap_set_discoverable(ADV_IND, 0,0,
                                         RANDOM_ADDR,
                                         BlueNRG_StackValue.AdvertisingFilter,
                                         (uint8_t)(sizeof(local_name)), local_name, 0, NULL, 0, 0);
    if(RetStatus != (tBleStatus)BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("Error: aci_gap_set_discoverable [%x] Filter=%x\r\n",RetStatus,BlueNRG_StackValue.AdvertisingFilter);
      goto EndLabel;
    } else {
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("aci_gap_set_discoverable OK Filter=%x\r\n",BlueNRG_StackValue.AdvertisingFilter);
#endif
    }
  } else {
    /* Advertising filter is enabled: enter in undirected connectable mode in order to use the advertising filter on bonded device */
    RetStatus = aci_gap_set_undirected_connectable(0,0,RANDOM_ADDR, BlueNRG_StackValue.AdvertisingFilter);
    if(RetStatus != (tBleStatus)BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("Error: aci_gap_set_undirected_connectable [%x] Filter=%x\r\n",RetStatus,BlueNRG_StackValue.AdvertisingFilter);
      goto EndLabel;
    } else {
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("aci_gap_set_undirected_connectable OK Filter=%x\r\n",BlueNRG_StackValue.AdvertisingFilter);
#endif
    }
  }
  
  if(CustomSetConnectable!=NULL){
    CustomSetConnectable(manuf_data);
  }
  
  /* Send Advertising data */
  RetStatus = aci_gap_update_adv_data(ADVERTIZE_DATA_LENGHT, manuf_data);
  if(RetStatus != (tBleStatus)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: aci_gap_update_adv_data [%x]\r\n",RetStatus);
  } else {
    BLE_MANAGER_PRINTF("aci_gap_update_adv_data OK\r\n");
  } 
  
EndLabel:
  return;
}

/**
* @brief  Exits the device from connectable mode.
* @param  None
* @retval None
*/
void setNotConnectable(void)
{
  aci_gap_set_non_discoverable();
}

/**
* @brief  Added BLE service
* @param  BleCharTypeDef *BleChar: Data structure pointer for BLE service
* @retval 1 in case of success
*/
int32_t BleManagerAddChar(BleCharTypeDef *BleChar)
{
  int32_t retValue=0;
  
  if(BleChar != NULL) {
    if(UsedBleChars<BLE_MANAGER_MAX_ALLOCABLE_CHARS) {
      BleCharsArray[UsedBleChars] = BleChar;
      UsedBleChars++;
      retValue=1;
    }
  }
  
  return retValue;
}

/**
* @brief  ResetBleManager
* @param  None
* @retval None
*/
void ResetBleManager(void)
{
  BLE_MANAGER_PRINTF("\r\nReset BleManager\r\n\r\n");
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_RESET);
  BLE_MANAGER_DELAY(300);
  HAL_GPIO_WritePin(HCI_TL_RST_PORT, HCI_TL_RST_PIN, GPIO_PIN_SET);
  BLE_MANAGER_DELAY(300);
}

/**
* @brief  ResetBleManagerCallbackFunctionPointer
* @param  None
* @retval None
*/
static void ResetBleManagerCallbackFunctionPointer(void)
{
  /**************** Bluetooth Comunication *************************/
  CustomPairingCompleted=NULL;
  CustomSetConnectable=NULL;
  CustomConnectionCompleted=NULL;
  CustomDisconnectionCompleted=NULL;
  CustomAciGattTxPoolAvailableEvent=NULL;

  /**************** Debug Console *************************/
  CustomDebugConsoleParsingCallback=NULL;

  /******************* Config Char *************************/
  CustomAttrModConfigCallback=NULL;
  CustomWriteRequestConfigCallback=NULL;
}

/**
* @brief  Init Ble Manager 
* @param  None
* @retval tBleStatus Status
*/
tBleStatus InitBleManager(void)
{
  tBleStatus ret;
  
  BLE_Conf_Service = BLE_SERV_NOT_ENABLE;
  BLE_StdTerm_Service = BLE_SERV_NOT_ENABLE;
  BLE_StdErr_Service = BLE_SERV_NOT_ENABLE;
  BLE_ExtConf_Service = BLE_SERV_NOT_ENABLE;

  UsedBleChars =0;
  UsedStandardBleChars = 0;
  connection_handle = 0;
  set_connectable = FALSE;
  MaxBLECharLen = DEFAULT_MAX_CHAR_LEN;

  /* BlueNRG stack initialization */
  ret = InitBleManager_BlueNRG_Stack();
  
  ResetBleManagerCallbackFunctionPointer();
  //ClearCustomCommandsList();
  
  if(ret==(tBleStatus)BLE_STATUS_SUCCESS) {
    /* Ble Manager services initialization */
    ret = InitBleManagerServices();
  }
  
  set_connectable=TRUE;
  
  return ret;
}

/** @brief HCI Transport layer user function
* @param void *pData pointer to HCI event data
* @retval None
*/
static void APP_UserEvtRx(void *pData)
{
  uint32_t i;
  
  hci_spi_pckt *hci_pckt = (hci_spi_pckt *)pData;
  
  if(hci_pckt->type == (uint8_t)HCI_EVENT_PKT) {
    hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;
    
    if(event_pckt->evt == (uint8_t)EVT_LE_META_EVENT) {
      evt_le_meta_event *evt = (void *)event_pckt->data;
      
      for (i = 0; i < (sizeof(hci_le_meta_events_table)/sizeof(hci_le_meta_events_table_type)); i++) {
        if (evt->subevent == hci_le_meta_events_table[i].evt_code) {
          hci_le_meta_events_table[i].process((void *)evt->data);
        }
      }
    } else if(event_pckt->evt == (uint8_t)EVT_VENDOR) {
      evt_blue_aci *blue_evt = (void*)event_pckt->data;        
      
      for (i = 0; i < (sizeof(hci_vendor_specific_events_table)/sizeof(hci_vendor_specific_events_table_type)); i++) {
        if (blue_evt->ecode == hci_vendor_specific_events_table[i].evt_code) {
          hci_vendor_specific_events_table[i].process((void *)blue_evt->data);
        }
      }
    } else {
      for (i = 0; i < (sizeof(hci_events_table)/sizeof(hci_events_table_type)); i++) {
        if (event_pckt->evt == hci_events_table[i].evt_code) {
          hci_events_table[i].process((void *)event_pckt->data);
        }
      }
    }
  }
}

/** @brief Initialize the BlueNRG Stack
* @param None
* @retval tBleStatus
*/
static tBleStatus InitBleManager_BlueNRG_Stack(void)
{
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  tBleStatus ret;
  uint8_t data_len_out;
  uint8_t random_number[8];
  
  /* Initialize the BlueNRG HCI */
  hci_init(APP_UserEvtRx, NULL);
  
  /* we will let the BLE chip to use its Random MAC address */
#define CONFIG_DATA_RANDOM_ADDRESS          (0x80) /**< Stored static random address. Read-only. */
  ret = aci_hal_read_config_data(CONFIG_DATA_RANDOM_ADDRESS, &data_len_out, BlueNRG_StackValue.BleMacAddress);
  
  if(ret != BLE_STATUS_SUCCESS){
    BLE_MANAGER_PRINTF("\r\nReading  Random BD_ADDR failed\r\n");
    goto fail;
  }
  
  /* Generate Random Key at every boot */
  if(BlueNRG_StackValue.EnableRandomSecurePIN) {
    BlueNRG_StackValue.SecurePIN = 99999;
    
    /* get a random number from BlueNRG-1 */
    if(hci_le_rand(random_number) != BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("hci_le_rand() call failed\r\n");
    }
    
    /* setup random_key with random number */
    for (uint8_t i=0; i<8U; i++) {
      BlueNRG_StackValue.SecurePIN += (435U*((uint32_t)random_number[i]));
    }
    
    /* Control section because we need 6 digits */
    if (BlueNRG_StackValue.SecurePIN <99999U) {
      BlueNRG_StackValue.SecurePIN += 100000U;
    }
  }
  
  ret = aci_hal_write_config_data(BlueNRG_StackValue.ConfigValueOffsets,
                                  BlueNRG_StackValue.ConfigValuelength,
                                  BlueNRG_StackValue.BleMacAddress);
  if(ret != BLE_STATUS_SUCCESS){
    BLE_MANAGER_PRINTF("\r\nSetting Public BD_ADDR failed\r\n");
    goto fail;
  }
  
  ret = aci_gatt_init();
  if(ret != BLE_STATUS_SUCCESS){
    BLE_MANAGER_PRINTF("\r\nGATT_Init failed\r\n");
    goto fail;
  }
  
  ret = aci_gap_init(BlueNRG_StackValue.GAP_Roles, 0, (uint8_t) strlen(BlueNRG_StackValue.BoardName), &service_handle, &dev_name_char_handle, &appearance_char_handle);
  
  
  if(ret != BLE_STATUS_SUCCESS){
    BLE_MANAGER_PRINTF("\r\nGAP_Init failed\r\n");
    goto fail;
  }
  
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   (uint8_t) strlen(BlueNRG_StackValue.BoardName), (uint8_t *)BlueNRG_StackValue.BoardName);
  
  if(ret != BLE_STATUS_SUCCESS){
    BLE_MANAGER_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    goto fail;
  }
  
  if(BlueNRG_StackValue.EnableSecureConnection) {
    /* Set the I/O capability  Otherwise the Smartphone will propose a Pin
    * that will be acepted without any control */
    if(aci_gap_set_io_capability(BlueNRG_StackValue.IO_capabilities)==BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("I/O Capability Configurated\r\n");
    } else {
      BLE_MANAGER_PRINTF("Error Setting I/O Capability\r\n");
    }
    
    if(BlueNRG_StackValue.EnableRandomSecurePIN) {
      ret = aci_gap_set_authentication_requirement(BlueNRG_StackValue.AuthenticationRequirements,
                                                   BlueNRG_StackValue.MITM_ProtectionRequirements,
                                                   BlueNRG_StackValue.SecureConnectionSupportOptionCode,
                                                   BlueNRG_StackValue.SecureConnectionKeypressNotification,
                                                   7, 
                                                   16,
                                                   DONOT_USE_FIXED_PIN_FOR_PAIRING,
                                                   BlueNRG_StackValue.SecurePIN,
                                                   0x01);
    } else {
      ret = aci_gap_set_authentication_requirement(BlueNRG_StackValue.AuthenticationRequirements,
                                                   BlueNRG_StackValue.MITM_ProtectionRequirements,
                                                   BlueNRG_StackValue.SecureConnectionSupportOptionCode,
                                                   BlueNRG_StackValue.SecureConnectionKeypressNotification,
                                                   7, 
                                                   16,
                                                   USE_FIXED_PIN_FOR_PAIRING,
                                                   BlueNRG_StackValue.SecurePIN,
                                                   0x01);
    }
    
  } else {
    ret = aci_gap_set_authentication_requirement(BlueNRG_StackValue.AuthenticationRequirements,
                                                 BlueNRG_StackValue.MITM_ProtectionRequirements,
                                                 BlueNRG_StackValue.SecureConnectionSupportOptionCode,
                                                 BlueNRG_StackValue.SecureConnectionKeypressNotification,
                                                 7, 
                                                 16,
                                                 USE_FIXED_PIN_FOR_PAIRING,
                                                 BlueNRG_StackValue.SecurePIN,
                                                 0x00);
  }
  
  if (ret != BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("\r\nGAP setting Authentication failed\r\n");
    goto fail;
  }
  
  BLE_MANAGER_PRINTF("\r\nSERVER: BLE Stack Initialized \r\n"
                     "\t\tBoardName= %s\r\n"
                       "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n",
                       BlueNRG_StackValue.BoardName,
                       BlueNRG_StackValue.BleMacAddress[5],
                       BlueNRG_StackValue.BleMacAddress[4],
                       BlueNRG_StackValue.BleMacAddress[3],
                       BlueNRG_StackValue.BleMacAddress[2],
                       BlueNRG_StackValue.BleMacAddress[1],
                       BlueNRG_StackValue.BleMacAddress[0]);
  
  if(BlueNRG_StackValue.EnableSecureConnection) {
    BLE_MANAGER_PRINTF("\t-->ONLY SECURE CONNECTION<--\r\n");
    
    if(BlueNRG_StackValue.EnableRandomSecurePIN) {
      BLE_MANAGER_PRINTF("\t\tRandom Key = %ld\r\n",BlueNRG_StackValue.SecurePIN);
    } else {
      BLE_MANAGER_PRINTF("\t\tFixed  Key = %ld\r\n",BlueNRG_StackValue.SecurePIN);
    }
  }
  
  /* Set output power level */
  aci_hal_set_tx_power_level(BlueNRG_StackValue.EnableHighPowerMode,
                             BlueNRG_StackValue.PowerAmplifierOutputLevel);
  
fail:
  return ret;
}

/**
* @brief  Init Ble Manager Services
* @param  None
* @retval tBleStatus Status
*/
static tBleStatus InitBleManagerServices(void)
{
  tBleStatus Status = BLE_ERROR_UNSPECIFIED;
  BleCharTypeDef *BleCharPointer;
  
  //Set the Malloc/Free Functions  used inside the Json Parser
  //json_set_allocation_functions(BLE_MallocFunction, BLE_FreeFunction);
  
#ifdef BLE_MANAGER_SDKV2
   BLE_MANAGER_PRINTF("BlueST-SDK V2\r\n");
#else /* BLE_MANAGER_SDKV2 */
   BLE_MANAGER_PRINTF("BlueST-SDK V1\r\n");
#endif /* BLE_MANAGER_SDKV2 */
  
  if(BlueNRG_StackValue.EnableConfig) {
    BleCharPointer = &BleCharConfig;
    memset(BleCharPointer,0,sizeof(BleCharTypeDef));
    BleCharPointer->AttrMod_Request_CB = AttrMod_Request_Config;
    BleCharPointer->Write_Request_CB = Write_Request_Config;
    COPY_CONFIG_CHAR_UUID((BleCharPointer->uuid));
    BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
    BleCharPointer->Char_Value_Length=DEFAULT_MAX_CONFIG_CHAR_LEN;
    BleCharPointer->Char_Properties=((uint8_t)CHAR_PROP_NOTIFY) | ((uint8_t)CHAR_PROP_WRITE_WITHOUT_RESP);
    BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
    BleCharPointer->GATT_Evt_Mask= ((uint8_t)GATT_NOTIFY_ATTRIBUTE_WRITE) | ((uint8_t)GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP);
    BleCharPointer->Enc_Key_Size=16;
    BleCharPointer->Is_Variable=1;
    BleManagerAddChar(BleCharPointer);
    
    Status = BLE_Manager_AddConfigService();
    if(Status == (tBleStatus)BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("Config  Service added successfully\r\n");
    } else {
      BLE_MANAGER_PRINTF("Error: while adding Config Service\r\n");
    }
  }
  
  if(BlueNRG_StackValue.EnableConsole) {
    BleCharPointer = &BleCharStdOut;
    memset(BleCharPointer,0,sizeof(BleCharTypeDef));    
    BleCharPointer->AttrMod_Request_CB = AttrMod_Request_Term;
    BleCharPointer->Write_Request_CB = Write_Request_Term;
    BleCharPointer->Read_Request_CB = Read_Request_Term;
    COPY_TERM_CHAR_UUID((BleCharPointer->uuid));
    BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
    BleCharPointer->Char_Value_Length=DEFAULT_MAX_STDOUT_CHAR_LEN;
    BleCharPointer->Char_Properties= ((uint8_t)CHAR_PROP_NOTIFY)| ((uint8_t)CHAR_PROP_WRITE_WITHOUT_RESP) | ((uint8_t)CHAR_PROP_WRITE) | ((uint8_t)CHAR_PROP_READ);
    BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
    BleCharPointer->GATT_Evt_Mask= ((uint8_t)GATT_NOTIFY_ATTRIBUTE_WRITE) | ((uint8_t)GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP);
    BleCharPointer->Enc_Key_Size=16;
    BleCharPointer->Is_Variable=1;
    BleManagerAddChar(BleCharPointer);
    
    BleCharPointer = &BleCharStdErr;
    memset(BleCharPointer,0,sizeof(BleCharTypeDef));
    BleCharPointer->AttrMod_Request_CB = AttrMod_Request_StdErr;
    BleCharPointer->Read_Request_CB = Read_Request_StdErr;
    COPY_STDERR_CHAR_UUID((BleCharPointer->uuid));
    BleCharPointer->Char_UUID_Type =UUID_TYPE_128;
    BleCharPointer->Char_Value_Length=DEFAULT_MAX_STDERR_CHAR_LEN;
    BleCharPointer->Char_Properties= ((uint8_t)CHAR_PROP_NOTIFY) | ((uint8_t)CHAR_PROP_READ);
    BleCharPointer->Security_Permissions=ATTR_PERMISSION_NONE;
    BleCharPointer->GATT_Evt_Mask=GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
    BleCharPointer->Enc_Key_Size=16;
    BleCharPointer->Is_Variable=1;
    BleManagerAddChar(BleCharPointer);
    
    Status = BLE_Manager_AddConsoleService();
    if(Status == (tBleStatus)BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("Console Service added successfully\r\n");
    } else {
      BLE_MANAGER_PRINTF("Error: while adding Console Service\r\n");
    }
  }
  
  UsedStandardBleChars = UsedBleChars;
  
  /* Set Custom Configuration and Services */
  BLE_InitCustomService();
  
  if((UsedBleChars-UsedStandardBleChars) > 0U)
  {
    Status = BLE_Manager_AddFeaturesService();
    if(Status == (tBleStatus)BLE_STATUS_SUCCESS) {
      BLE_MANAGER_PRINTF("Features Service added successfully (Status= 0x%x)\r\n", Status);
    } else {
      BLE_MANAGER_PRINTF("Error: while adding Features Service (Status= 0x%x)\r\n", Status);
    }
  }
  
  return Status;
}

/* ***************** BlueNRG-1 Stack Callbacks ********************************/

/*******************************************************************************
* Function Name  : hci_le_connection_complete_event.
* Description    : This event indicates that a new connection has been created.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_le_connection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Role,
                                      uint8_t Peer_Address_Type,
                                      uint8_t Peer_Address[6],
                                      uint16_t Conn_Interval,
                                      uint16_t Conn_Latency,
                                      uint16_t Supervision_Timeout,
                                      uint8_t Master_Clock_Accuracy)
{
  connection_handle = Connection_Handle;
  
  BLE_MANAGER_PRINTF(">>>>>>CONNECTED %x:%x:%x:%x:%x:%x\r\n",Peer_Address[5],Peer_Address[4],Peer_Address[3],Peer_Address[2],Peer_Address[1],Peer_Address[0]);

  if(BlueNRG_StackValue.EnableSecureConnection) {
    tBleStatus RetStatus;
    /* Check if the device is already bonded */
    RetStatus = aci_gap_is_device_bonded(Peer_Address_Type,Peer_Address);
    if( RetStatus != (tBleStatus)BLE_STATUS_SUCCESS) {
      /* Send a slave security request to the master */
      RetStatus = aci_gap_slave_security_req(Connection_Handle);
      if (RetStatus != (tBleStatus)BLE_STATUS_SUCCESS) {
        BLE_MANAGER_PRINTF("Error: GAP Slave secury request failed %d\r\n",RetStatus);
      } else {
#if (BLE_DEBUG_LEVEL>1)
        BLE_MANAGER_PRINTF("GAP Slave secury request Done\r\n");
#endif
      }
    } else {
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("Device already bounded\r\n");
#endif
    }
  }
  
  /* Start one Exchange configuration for understaning the maxium ATT_MTU */
  aci_gatt_exchange_config(connection_handle);  
  
  if(CustomConnectionCompleted!=NULL){
    CustomConnectionCompleted(connection_handle, Peer_Address);
  }
  
}/* end hci_le_connection_complete_event() */

/*******************************************************************************
* Function Name  : hci_disconnection_complete_event.
* Description    : This event occurs when a connection is terminated.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_disconnection_complete_event(uint8_t Status,
                                      uint16_t Connection_Handle,
                                      uint8_t Reason)
{  
  /* No Device Connected */
  connection_handle =0;
  
  BLE_MANAGER_PRINTF("<<<<<<DISCONNECTED\r\n");
  
  /* Make the device connectable again. */
  set_connectable = TRUE;
  
  if(CustomDisconnectionCompleted!=NULL){
    CustomDisconnectionCompleted();
  }
  
  //  if(BlueNRG_StackValue.EnableSecureConnection) {
  //    BlueNRG_StackValue.AdvertisingFilter = WHITE_LIST_FOR_ALL;
  //  }
  
}/* end hci_disconnection_complete_event() */

/*******************************************************************************
* Function Name  : aci_gatt_read_permit_req_event.
* Description    : This event is given when a read request is received
*                  by the server from the client.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gatt_read_permit_req_event(uint16_t Connection_Handle,
                                    uint16_t Attribute_Handle,
                                    uint16_t Offset)
{
  uint32_t FoundHandle=0;
  uint8_t RegisteredHandle;
  
  //Search inside all the registed handles
  for(RegisteredHandle=0;((RegisteredHandle<UsedBleChars) && (FoundHandle==0U));RegisteredHandle++) {
    if(BleCharsArray[RegisteredHandle]->Read_Request_CB!=NULL) {
      if(Attribute_Handle==(BleCharsArray[RegisteredHandle]->attr_handle+1U)) {
        BleCharsArray[RegisteredHandle]->Read_Request_CB(BleCharsArray[RegisteredHandle],Attribute_Handle);
      }
    }
  }
  
  if(connection_handle != 0U)
    aci_gatt_allow_read(connection_handle);
}

/*******************************************************************************
* Function Name  : aci_gatt_attribute_modified_event.
* Description    : This event is given when an attribute change his value.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gatt_attribute_modified_event(uint16_t Connection_Handle,
                                       uint16_t Attr_Handle,
                                       uint16_t Offset,
                                       uint16_t Attr_Data_Length,
                                       uint8_t Attr_Data[])
{
  uint32_t FoundHandle=0;
  uint8_t RegisteredHandle;
  
  if (Attr_Handle==((uint16_t)(0x0002+2))) {
    BLE_MANAGER_PRINTF("Notification on Service Change Characteristic\r\n");
    FoundHandle=1;
    if(BlueNRG_StackValue.ForceRescan) {
      /* Force one UUID rescan */
      tBleStatus ret = BLE_STATUS_INSUFFICIENT_RESOURCES;
      uint8_t buff[4];
      
      /* Delete all the Handles from 0x0001 to 0xFFFF */
      STORE_LE_16(buff  ,0x0001U);
      STORE_LE_16(buff+2,0xFFFFU);
      
      ret = aci_gatt_update_char_value(0x0001,0x0002,0,4,buff);
      
      if (ret == (tBleStatus)BLE_STATUS_SUCCESS){
        BLE_MANAGER_PRINTF("UUID Rescan Forced\r\n");
      } else {
        BLE_MANAGER_PRINTF("Error: Problem forcing UUID Rescan\r\n");
      }
    }
  }
  
  //Search inside all the registed handles  
  for(RegisteredHandle=0;((RegisteredHandle<UsedBleChars) && (FoundHandle==0U));RegisteredHandle++) {
    /* Notification */
    if(BleCharsArray[RegisteredHandle]->AttrMod_Request_CB!=NULL) {
      if(Attr_Handle==(BleCharsArray[RegisteredHandle]->attr_handle+2U)) {
        FoundHandle = 1U;
        BleCharsArray[RegisteredHandle]->AttrMod_Request_CB(BleCharsArray[RegisteredHandle],Attr_Handle, Offset, Attr_Data_Length, Attr_Data);
      }
    }
    
    /* Write */
    if(FoundHandle==0U) {
      if(BleCharsArray[RegisteredHandle]->Write_Request_CB!=NULL) {
        if(Attr_Handle==(BleCharsArray[RegisteredHandle]->attr_handle+1U)) {
          FoundHandle = 1U;
          BleCharsArray[RegisteredHandle]->Write_Request_CB(BleCharsArray[RegisteredHandle],Attr_Handle, Offset, Attr_Data_Length, Attr_Data);
        }
      }
    }
  }
  
  if(FoundHandle==0U) {
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite =(uint8_t)sprintf((char *)BufferToWrite, "Notification UNKNOWN handle\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Notification UNKNOWN handle =%d\r\n",Attr_Handle);
    }
  }
}

/*******************************************************************************
* Function Name  : aci_att_exchange_mtu_resp_event
* Description    : This event is generated in response to an Exchange MTU request
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_att_exchange_mtu_resp_event(uint16_t Connection_Handle,
                                     uint16_t Server_RX_MTU)
{
  if((Server_RX_MTU-3U)<MaxBLECharLen) {
    MaxBLECharLen = (uint8_t)(Server_RX_MTU-3U);
  }
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("aci_att_exchange_mtu_resp_event Server_RX_MTU=%d\r\n",Server_RX_MTU);
#endif
}

/*******************************************************************************
* Function Name  : aci_gap_pairing_complete_event
* Description    : This event is generated when the pairing process has completed 
*                  successfully or a pairing procedure timeout has occurred or 
*                  the pairing has failed
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gap_pairing_complete_event(uint16_t Connection_Handle,
                                    uint8_t Status,
                                    uint8_t Reason)
{
  
  if(Status==0x00U) {
    BLE_MANAGER_PRINTF("Pairing Completed\r\n");
  } else {
    BLE_MANAGER_PRINTF("Pairing Not Completed for [%s] with reason=%x\r\n",
                       (Status==0x01U) ? "Timeout" : "Failed",Reason);
  }
  
  UpdateWhiteList();
  
  if(CustomPairingCompleted!=NULL){
    CustomPairingCompleted(Status);
  }
    
  BLE_MANAGER_DELAY(100);
}

void aci_gap_pass_key_req_event(uint16_t Connection_Handle)
{
  tBleStatus status;
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("aci_gap_pass_key_req_event [Requested PassWd=%ld]\r\n", BlueNRG_StackValue.SecurePIN);
#endif
  status = aci_gap_pass_key_resp(connection_handle, BlueNRG_StackValue.SecurePIN);
  if (status != (uint8_t)BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: aci_gap_pass_key_resp failed:0x%02x\r\n", status);
#if (BLE_DEBUG_LEVEL>1)
  } else {
    BLE_MANAGER_PRINTF("aci_gap_pass_key_resp OK\r\n");
#endif
  }
  
}

/*******************************************************************************
* Function Name  : aci_l2cap_connection_update_resp_event
* Description    : This event is generated when the master responds to the connection
*                  update request packet with a connection update response packet
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_l2cap_connection_update_resp_event(uint16_t Connection_Handle,
                                            uint16_t Result)
{
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("aci_l2cap_connection_update_resp_event Result=%d\r\n",Result);
#endif
}

/*******************************************************************************
* Function Name  : hci_le_connection_update_complete_event
* Description    : It indicates that the Controller process to update the connection has completed
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_le_connection_update_complete_event(uint8_t Status,
                                             uint16_t Connection_Handle,
                                             uint16_t Conn_Interval,
                                             uint16_t Conn_Latency,
                                             uint16_t Supervision_Timeout)
{
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("hci_le_connection_update_complete_event:\r\n");
  BLE_MANAGER_PRINTF("\tStatus=%d\r\n",Status);
  BLE_MANAGER_PRINTF("\tConn_Interval=%d\r\n",Conn_Interval);
  BLE_MANAGER_PRINTF("\tConn_Latency=%d\r\n",Conn_Latency);
  BLE_MANAGER_PRINTF("\tSupervision_Timeout=%d\r\n",Supervision_Timeout);
#endif
}

/*******************************************************************************
* Function Name  : aci_gatt_proc_complete_event
* Description    : This event is generated when a GATT client procedure completes
*                  either with error or successfully
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gatt_proc_complete_event(uint16_t Connection_Handle,
                                  uint8_t Error_Code)
{
  if(Error_Code!=0U) {
    BLE_MANAGER_PRINTF("Error: aci_gatt_proc_complete_event Error Code=%d\r\n",Error_Code);
#if (BLE_DEBUG_LEVEL>2)
  } else {
    BLE_MANAGER_PRINTF("aci_gatt_proc_complete_event Success\r\n");
#endif
  }
}

/*******************************************************************************
* Function Name  : hci_le_data_length_change_event
* Description    : The LE Data Length Change event notifies the Host of a change
*                  to either the maximum Payload length or the maximum transmission
*                  time of Data Channel PDUs in either direction
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_le_data_length_change_event(uint16_t Connection_Handle,
                                     uint16_t MaxTxOctets,
                                     uint16_t MaxTxTime,
                                     uint16_t MaxRxOctets,
                                     uint16_t MaxRxTime)
{
  tBleStatus RetStatus;
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("hci_le_data_length_change_event\r\n");
#endif
  RetStatus = aci_gatt_exchange_config(Connection_Handle);
  if( RetStatus !=BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: ACI GATT Exchange Config Failed (0x%x)\r\n", RetStatus);
#if (BLE_DEBUG_LEVEL>2)
  } else {
    BLE_MANAGER_PRINTF("ACI GATT Exchange Config Done\r\n");
#endif
  }
}

/**
* @brief This event is generated when an indication is received from the server.
* @param Connection_Handle Connection handle related to the response
* @param Attribute_Handle The handle of the attribute
* @param Attribute_Value_Length Length of Attribute_Value in octets
* @param Attribute_Value The current value of the attribute
* @retval None
*/
void aci_gatt_indication_event(uint16_t Connection_Handle,
                               uint16_t Attribute_Handle,
                               uint8_t Attribute_Value_Length,
                               uint8_t Attribute_Value[])
{
  tBleStatus RetStatus;
  
  /* This callback should be called when we connect the .box also to something
  * that could work also like server mode.
  * In our case we don't need to do nothing when we receive this indication,
  * except it's confirmation
  */
#ifdef BLE_MANAGER_DEBUG
  #if (BLE_DEBUG_LEVEL>2)
    BLE_MANAGER_PRINTF("aci_gatt_indication_event:\r\n");
    BLE_MANAGER_PRINTF("\tConnection_Handle=0x%x\r\n",Connection_Handle);
    BLE_MANAGER_PRINTF("\tAttribute_Handle=0x%x\r\n",Attribute_Handle);
    if(Attribute_Value_Length==4U) {
      uint16_t StartHandle = 0;
      uint16_t StopHandle  = 0;
      /* Should be the range of Handles */
      StartHandle = (((uint16_t) Attribute_Value[1])<<8);
      StartHandle = StartHandle  | ((uint16_t)Attribute_Value[0]);
      StopHandle  = (((uint16_t) Attribute_Value[3])<<8);
      StopHandle = StopHandle | ((uint16_t)Attribute_Value[2]);
      BLE_MANAGER_PRINTF("\tFrom Handles =0x%x to 0x%x\r\n", StartHandle,StopHandle);
    }
    
    BLE_MANAGER_PRINTF("Nothing to do except send confirmation\r\n");
  #endif
#endif /* BLE_MANAGER_DEBUG */
  RetStatus = aci_gatt_confirm_indication(Connection_Handle);
  if (RetStatus != BLE_STATUS_SUCCESS) {
    BLE_MANAGER_PRINTF("Error: aci_gatt_confirm_indicationt failed %d\r\n",RetStatus);
#if (BLE_DEBUG_LEVEL>2)
  } else {
    BLE_MANAGER_PRINTF("aci_gatt_confirm_indication Done\r\n");
#endif
  }
}

/**
* @brief The Hardware Error event is used to indicate some implementation specific type of hardware failure for the controller. This event is used to notify the Host that a hardware failure has occurred in the Controller.
* @param Hardware_Code Hardware Error Event code.
Error code 0x01 and 0x02 are errors generally caused by hardware issue on the PCB; another possible cause is a slow crystal startup.
In the latter case, the HS_STARTUP_TIME in the device configuration needs to be tuned.
Error code 0x03 indicates an internal error of the protocol stack.
After this event is recommended to force device reset.
* Values:
- 0x01: Radio state error
- 0x02: Timer overrun error
- 0x03: Internal queue overflow error
* @retval None
*/
void hci_hardware_error_event(uint8_t Hardware_Code)
{
  BLE_MANAGER_PRINTF("Error: hci_hardware_error_event Hardware_Code=%x\r\n",Hardware_Code);
  //BLE_MANAGER_DELAY(1000);
  HAL_NVIC_SystemReset();
}
/*******************************************************************************
* Function Name  : aci_gap_bond_lost_event
* Description    : This event is generated on the slave when a 
*                  ACI_GAP_SLAVE_SECURITY_REQUEST is called to reestablish the bond.
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gap_bond_lost_event(void) {
  aci_gap_allow_rebond(connection_handle);
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("aci_gap_allow_rebond()\r\n");
#endif
}

/**
* @brief  This function Updates the White list for BLE Connection
* @param None
* @retval None
*/
static void UpdateWhiteList(void)
{
  tBleStatus RetStatus;
  uint8_t NumOfAddresses; 
  Bonded_Device_Entry_t BondedDeviceEntry[BLE_MANAGER_MAX_BONDED_DEVICES];
  
  RetStatus =  aci_gap_get_bonded_devices(&NumOfAddresses, BondedDeviceEntry);
  
  if (RetStatus == BLE_STATUS_SUCCESS) {
    if (NumOfAddresses > 0U) {
#if (BLE_DEBUG_LEVEL>2)
      BLE_MANAGER_PRINTF("Bonded with %d Device(s): \r\n", NumOfAddresses);
#endif
      RetStatus = aci_gap_configure_whitelist();
      if (RetStatus != BLE_STATUS_SUCCESS) {
        BLE_MANAGER_PRINTF("Error: aci_gap_configure_whitelist() failed:0x%02x\r\n", RetStatus);
#if (BLE_DEBUG_LEVEL>2)
      } else {
        BLE_MANAGER_PRINTF("aci_gap_configure_whitelist --> SUCCESS\r\n");
#endif
      }
    }
  }
}

/*******************************************************************************
* Function Name  : aci_gap_numeric_comparison_value_event
* Description    : This event is sent only during SC v.4.2 Pairing
*                  when Numeric Comparison Association model is selected
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void aci_gap_numeric_comparison_value_event(uint16_t Connection_Handle, uint32_t Numeric_Value)
{
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("aci_gap_numeric_comparison_value_event Numeric_Value=%ld\r\n",Numeric_Value);
#endif
  
  /* Confirm Yes... without control of Numeric Value received from Master */
  aci_gap_numeric_comparison_value_confirm_yesno(Connection_Handle,0x01);
}

/*******************************************************************************
* Function Name  : hci_encryption_change_event
* Description    : It is used to indicate that the change of the encryption
*                  mode has been completed
* Input          : See file bluenrg1_events.h
* Output         : See file bluenrg1_events.h
* Return         : See file bluenrg1_events.h
*******************************************************************************/
void hci_encryption_change_event(uint8_t Status,uint16_t Connection_Handle,uint8_t Encryption_Enabled)
{
#if (BLE_DEBUG_LEVEL>2)
  BLE_MANAGER_PRINTF("hci_encryption_change_event\r\n");  
#endif
}

