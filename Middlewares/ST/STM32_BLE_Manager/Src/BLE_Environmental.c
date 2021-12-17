/**
  ******************************************************************************
  * @file    BLE_Environmental.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   Add environmental info services using vendor specific profiles.
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
#define COPY_ENVIRONMENTAL_CHAR_UUID(uuid_struct) COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

#define ENVIRONMENTAL_ADVERTIZE_DATA_POSITION  16

/* Exported variables --------------------------------------------------------*/
/* Identifies the notification Events */
BLE_NotifyEnv_t BLE_Env_NotifyEvent = BLE_NOTIFY_NOTHING;

CustomReadRequestEnv_t CustomReadRequestEnv=NULL;

/* Private Types ----------------------------------------------------------- */
typedef struct
{
  uint8_t PressureIsEnable;
  uint8_t HumidityIsEnable;
  uint8_t NumberTemperaturesEnabled;
} BLE_Manager_EnvFeaturesEnabled;

/* Private variables ---------------------------------------------------------*/
/* Data structure for identify environmental info services enabled */
BLE_Manager_EnvFeaturesEnabled EnvFeaturesEnabled;
/* Data structure pointer for environmental info service */
static BleCharTypeDef BleCharEnv;
/* Size for Environmental BLE characteristic */
static uint8_t  EnvironmentalCharSize;

/* Private functions ---------------------------------------------------------*/
static void AttrMod_Request_Env(void *BleCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data);
static void Read_Request_Env(void *BleCharPointer,uint16_t handle);

/**
* @brief  Init environmental info service
* @param  uint8_t PressEnable:    1 for enabling the BLE pressure feature, 0 otherwise.
* @param  uint8_t HumEnable:      1 for enabling the BLE humidity feature, 0 otherwise.
* @param  uint8_t NumTempEnabled: 0 for disabling BLE temperature feature
*                                 1 for enabling only one BLE temperature feature
*                                 2 for enabling two BLE temperatures feature
* @retval BleCharTypeDef* BleCharPointer: Data structure pointer for environmental info service
*/
BleCharTypeDef* BLE_InitEnvService(uint8_t PressEnable, uint8_t HumEnable, uint8_t NumTempEnabled)
{
  /* Data structure pointer for BLE service */
  BleCharTypeDef *BleCharPointer= NULL;
  EnvironmentalCharSize=2;
  
  /* Init data structure for identify environmental info services enabled */
  EnvFeaturesEnabled.PressureIsEnable= PressEnable;
  EnvFeaturesEnabled.HumidityIsEnable= HumEnable;
  EnvFeaturesEnabled.NumberTemperaturesEnabled= NumTempEnabled;
  
  if( (PressEnable == 1U) ||
     (HumEnable == 1U)   ||
       (NumTempEnabled > 0U) )
  {
    /* Init data structure pointer for environmental info service */
    BleCharPointer = &BleCharEnv;
    memset(BleCharPointer,0,sizeof(BleCharTypeDef));  
    BleCharPointer->AttrMod_Request_CB= AttrMod_Request_Env;
    BleCharPointer->Read_Request_CB= Read_Request_Env;
    COPY_ENVIRONMENTAL_CHAR_UUID((BleCharPointer->uuid));
    
    /* Enables BLE Pressure feature */
    if(PressEnable == 1U){
      BleCharPointer->uuid[14] |= 0x10U;
      EnvironmentalCharSize+= 4U;
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("\t--> Pressure feature enabled\r\n");
#endif
    }
    
    /* Enables BLE Humidity feature */
    if(HumEnable == 1U){
      BleCharPointer->uuid[14] |= 0x08U;
      EnvironmentalCharSize+= 2U;
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("\t--> Humidity feature enabled\r\n");
#endif
    }
    
    if(NumTempEnabled == 1U){
      /* Enables only one BLE temperature feature */
      BleCharPointer->uuid[14] |= 0x04U;
      EnvironmentalCharSize+= 2U;
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("\t--> Only one temperature feature enabled\r\n");
#endif
    } else if(NumTempEnabled == 2U){
      /* Enables two BLE temperatures feature */
      BleCharPointer->uuid[14] |= 0x05U;
      EnvironmentalCharSize+= 4U;
#if (BLE_DEBUG_LEVEL>1)
      BLE_MANAGER_PRINTF("\t--> Two temperatures features enabled\r\n");
#endif
    }
    
    BleCharPointer->Char_UUID_Type= UUID_TYPE_128;
    BleCharPointer->Char_Value_Length= EnvironmentalCharSize;
    BleCharPointer->Char_Properties= ((uint8_t)(CHAR_PROP_NOTIFY))|((uint8_t)(CHAR_PROP_READ));
    BleCharPointer->Security_Permissions= ATTR_PERMISSION_NONE;
    BleCharPointer->GATT_Evt_Mask= GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP;
    BleCharPointer->Enc_Key_Size= 16;
    BleCharPointer->Is_Variable= 0;
    
    if(CustomReadRequestEnv == NULL) {
      BLE_MANAGER_PRINTF("Warning: Read request environmental function not defined\r\n");
    }
    
    BLE_MANAGER_PRINTF("BLE Environmental features ok\r\n");
  } else {
    BLE_MANAGER_PRINTF(" ERROR: None environmental features is enabled\r\n");
  }
  
  return BleCharPointer;
}

#ifndef BLE_MANAGER_SDKV2
/**
* @brief  Setting Environmental Advertise Data
* @param  uint8_t *manuf_data: Advertise Data
* @retval None
*/
void BLE_SetEnvAdvertizeData(uint8_t *manuf_data)
{
  /* Setting Pressure Advertise Data */
  if(EnvFeaturesEnabled.PressureIsEnable == 1U) {
    manuf_data[ENVIRONMENTAL_ADVERTIZE_DATA_POSITION] |= 0x10U;
  }
  
  /* Setting Humidity Advertise Data */
  if(EnvFeaturesEnabled.HumidityIsEnable == 1U) {
    manuf_data[ENVIRONMENTAL_ADVERTIZE_DATA_POSITION] |= 0x08U;
  }
  
  /* Setting One Temperature Advertise Data */
  if(EnvFeaturesEnabled.NumberTemperaturesEnabled == 1U) {
    manuf_data[ENVIRONMENTAL_ADVERTIZE_DATA_POSITION] |= 0x04U;
  }
  
  /* Setting Two Temperature Advertise Data */
  if(EnvFeaturesEnabled.NumberTemperaturesEnabled == 2U) {
    manuf_data[ENVIRONMENTAL_ADVERTIZE_DATA_POSITION] |= 0x05U;
  }
}
#endif /* BLE_MANAGER_SDKV2 */

/**
* @brief  Update Environmental characteristic value
* @param  int32_t Press:       Pressure in mbar (Set 0 if not used)
* @param  uint16_t Hum:        humidity RH (Relative Humidity) in thenths of % (Set 0 if not used)
* @param  int16_t Temp1:       Temperature in tenths of degree (Set 0 if not used)
* @param  int16_t Temp2:       Temperature in tenths of degree (Set 0 if not used)
* @retval tBleStatus:          Status
*/
tBleStatus BLE_EnvironmentalUpdate(int32_t Press, uint16_t Hum, int16_t Temp1, int16_t Temp2)
{
  tBleStatus ret;
  uint8_t BuffPos;
  
  uint8_t buff[2 + 4/*Press*/ + 2/*Hum*/ + 2/*Temp1*/ + 2/*Temp2*/];
  
  /* Time Stamp */ 
  STORE_LE_16(buff  ,(HAL_GetTick()>>3));
  BuffPos= 2;
  
  if(EnvFeaturesEnabled.PressureIsEnable == 1U) {
    STORE_LE_32((buff+BuffPos),((uint32_t)Press));
    BuffPos+= 4U;
  }
  
  if(EnvFeaturesEnabled.HumidityIsEnable == 1U) {
    STORE_LE_16((buff+BuffPos),Hum);
    BuffPos+= 2U;
  }
  
  if(EnvFeaturesEnabled.NumberTemperaturesEnabled >= 1U) {
    STORE_LE_16((buff+BuffPos),((uint16_t)Temp1));
    BuffPos+= 2U;
  }
  
  if(EnvFeaturesEnabled.NumberTemperaturesEnabled == 2U) {
    STORE_LE_16((buff+BuffPos),((uint16_t)Temp2));
    BuffPos+= 2U;
  }
  
  ret = ACI_GATT_UPDATE_CHAR_VALUE(&BleCharEnv, 0, EnvironmentalCharSize,buff);
  
  if (ret != (tBleStatus)BLE_STATUS_SUCCESS){
    if(BLE_StdErr_Service==BLE_SERV_ENABLE){
      BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite, "Error Updating Environmental Char\n");
      Stderr_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Error: Updating Environmental Char\r\n");
    }
  }
  return ret;
}

/**
* @brief  This function is called when there is a change on the gatt attribute
*         With this function it's possible to understand if enviromental is subscribed or not to the one service
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
static void AttrMod_Request_Env(void *VoidCharPointer,uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if (att_data[0] == 01U) {
    BLE_Env_NotifyEvent= BLE_NOTIFY_SUB;
  } else if (att_data[0] == 0U){
    BLE_Env_NotifyEvent= BLE_NOTIFY_UNSUB;
  }
  
#if (BLE_DEBUG_LEVEL>1)
  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
    BytesToWrite = (uint8_t)sprintf((char *)BufferToWrite,"--->Env=%s\n", (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) ? " ON" : " OFF");
    Term_Update(BufferToWrite,BytesToWrite);
  } else {
    BLE_MANAGER_PRINTF("--->Env=%s", (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) ? " ON\r\n" : " OFF\r\n");
  }
#endif
}

/**
* @brief  This event is given when a read request is received by the server from the client.
* @param  void *VoidCharPointer
* @param  uint16_t handle Handle of the attribute
* @retval None
*/
static void Read_Request_Env(void *VoidCharPointer,uint16_t handle)
{
  if(CustomReadRequestEnv != NULL) {
    CustomReadRequestEnv();
  } else {
    BLE_MANAGER_PRINTF("\r\n\nRead request environmental function not defined\r\n\n");
  } 
}

