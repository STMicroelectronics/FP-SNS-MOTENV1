/**
  ******************************************************************************
  * @file    BLE_Implementation.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   BLE Implementation template file.
  *          This file should be copied to the application folder and renamed
  *          to BLE_Implementation.c.
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
#include "HWAdvanceFeatures.h"
#include "BLE_Manager.h"
#include "MOTENV1_config.h"

/* Exported Variables --------------------------------------------------------*/
uint8_t connected= FALSE;
int32_t  NeedToClearSecureDB=0;
uint32_t ConnectionBleStatus  =0;

/* Private variables ------------------------------------------------------------*/
volatile uint32_t FeatureMask;
      
/* Private functions ---------------------------------------------------------*/
static uint8_t getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion);
static uint32_t DebugConsoleParsing(uint8_t * att_data, uint8_t data_length);
static void ReadRequestEnvFunction(void);
static void DisconnectionCompletedFunction(void);
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Addr[6]);
static void WriteRequestConfigFunction(uint8_t * att_data, uint8_t data_length);

static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);

/**********************************************************************************************
 * Callback functions prototypes to manage the extended configuration characteristic commands *
 **********************************************************************************************/
//static void ExtExtConfigUidCommandCallback(uint8_t **UID);
//static void ExtConfigInfoCommandCallback(uint8_t *Answer);
//static void ExtConfigHelpCommandCallback(uint8_t *Answer);
//static void ExtConfigVersionFwCommandCallback(uint8_t *Answer);

/** @brief Initialize the BlueNRG stack and services
  * @param  None
  * @retval None
  */
void BluetoothInit(void)
{
  /* BlueNRG stack setting */
  BlueNRG_StackValue.ConfigValueOffsets                   = CONFIG_DATA_PUBADDR_OFFSET;
  BlueNRG_StackValue.ConfigValuelength                    = CONFIG_DATA_PUBADDR_LEN;
  BlueNRG_StackValue.GAP_Roles                            = GAP_PERIPHERAL_ROLE;
  BlueNRG_StackValue.IO_capabilities                      = IO_CAP_DISPLAY_ONLY;
  BlueNRG_StackValue.AuthenticationRequirements           = BONDING;
  BlueNRG_StackValue.MITM_ProtectionRequirements          = MITM_PROTECTION_REQUIRED;
  BlueNRG_StackValue.SecureConnectionSupportOptionCode    = SC_IS_SUPPORTED;
  BlueNRG_StackValue.SecureConnectionKeypressNotification = KEYPRESS_IS_NOT_SUPPORTED;

  /* To set the TX power level of the bluetooth device ( -2,1 dBm )*/
  BlueNRG_StackValue.EnableHighPowerMode= 1; /*  High Power */
  
  /* Values: 0x00 ... 0x31 - The value depends on the device */
  BlueNRG_StackValue.PowerAmplifierOutputLevel =4;
  
  /* BlueNRG services setting */
  BlueNRG_StackValue.EnableConfig    = 1;
  BlueNRG_StackValue.EnableConsole   = 1;
  
  /* For Enabling the Secure Connection */
  BlueNRG_StackValue.EnableSecureConnection=0;
  /* Default Secure PIN */
  BlueNRG_StackValue.SecurePIN=123456;
  /* For creating a Random Secure PIN */
  BlueNRG_StackValue.EnableRandomSecurePIN = 0;
  
  BlueNRG_StackValue.AdvertisingFilter    = NO_WHITE_LIST_USE;
  
  if(BlueNRG_StackValue.EnableSecureConnection) {
    /* Using the Secure Connection, the Rescan should be done by BLE chip */    
    BlueNRG_StackValue.ForceRescan =0;
  } else {
    BlueNRG_StackValue.ForceRescan =1;
  }
  
  InitBleManager();
}

/**
 * @brief  Custom Service Initialization.
 * @param  None
 * @retval None
 */
void BLE_InitCustomService(void) {
  /* Define Custom Function for Debug Console Command parsing */
  CustomDebugConsoleParsingCallback = &DebugConsoleParsing;
  
  /* Define Custom Function for Connection Completed */
  CustomConnectionCompleted = &ConnectionCompletedFunction;
  
  /* Define Custom Function for Disconnection Completed */
  CustomDisconnectionCompleted = &DisconnectionCompletedFunction;
  
  /* Define Custom Command for Parsing Write on Config Char */
  CustomWriteRequestConfigCallback = &WriteRequestConfigFunction;
  
  /***********************************************************************************
   * Callback functions to manage the extended configuration characteristic commands *
   ***********************************************************************************/
//  CustomExtConfigUidCommandCallback  = &ExtExtConfigUidCommandCallback;
//  CustomExtConfigInfoCommandCallback = &ExtConfigInfoCommandCallback;
//  CustomExtConfigHelpCommandCallback = &ExtConfigHelpCommandCallback;
//  CustomExtConfigVersionFwCommandCallback = &ExtConfigVersionFwCommandCallback;
  
  /**
  * For each features, user can assign here the pointer at the function for the read request data.
  * For example for the environmental features:
  * 
  * CustomReadRequestEnv = &ReadRequestEnvFunction;
  * 
  * User can define and insert in the BLE_Implementation.c source code the functions for the read request data
  * ReadRequestEnvFunction function is already defined.
  *
  */
  
  /* Define Custom Function for Read Request Environmental Data */
  CustomReadRequestEnv = &ReadRequestEnvFunction;
  
  /*******************
   * User code begin *
   *******************/
  
  /**
  * User can added here the custom service initialization for the selected BLE features.
  * For example for the environmental features:
  * 
  * //BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled)
  * BleManagerAddChar(BleCharPointer= BLE_InitEnvService(1, 1, 1));
  */
  
  /* Service initialization and adding for the environmental features */
  /* BLE_InitEnvService(PressEnable,HumEnable,NumTempEnabled) */
  BleManagerAddChar(BLE_InitEnvService(1, 1, 2));
  
  /* Service initialization and adding  for the inertial features */
  /* BLE_InitInertialService(AccEnable,GyroEnable,MagEnabled) */
  BleManagerAddChar(BLE_InitInertialService(1,1,1));
  
  /* Service initialization and adding for the accelerometer events features */
  BleManagerAddChar(BLE_InitAccEnvService());
  
  /* Service initialization and adding for the Led features */
  BleManagerAddChar(BLE_InitLedService());
  
  /*****************
   * User code end *
   *****************/
}

/**
 * @brief  Set Custom Advertize Data.
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetCustomAdvertizeData(uint8_t *manuf_data)
{
  /**
  * User can add here the custom advertize data setting  for the selected BLE features.
  * For example for the environmental features:
  * 
  * BLE_SetCustomEnvAdvertizeData(manuf_data);
  */
  
#ifndef BLE_MANAGER_SDKV2
  /* Custom advertize data setting for the environmental features */
  BLE_SetEnvAdvertizeData(manuf_data);
  
  /* Custom advertize data setting for the inertial features */
  BLE_SetInertialAdvertizeData(manuf_data);
  
  /* Custom advertize data setting for the accelerometer events features */
  BLE_SetAccEnvAdvertizeData(manuf_data);
  
  /* Custom advertize data setting for the led features */
  BLE_SetLedAdvertizeData(manuf_data);
#else /* BLE_MANAGER_SDKV2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1]=0x01; /* TMP Code BLE_PROPOSAL */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD2]=0x03; /* Dummy */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3]=0x33; /* Dummy */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4]=0x44; /* Dummy */
#endif /* BLE_MANAGER_SDKV2 */
}

/**
 * @brief  Get hardware and firmware version
 *
 * @param  Hardware version
 * @param  Firmware version
 * @retval Status
 */
static uint8_t getBlueNRG2_Version(uint8_t *hwVersion, uint16_t *fwVersion)
{
  uint8_t status;
  uint8_t hci_version, lmp_pal_version;
  uint16_t hci_revision, manufacturer_name, lmp_pal_subversion;
  uint8_t DTM_version_major, DTM_version_minor, DTM_version_patch, DTM_variant, BTLE_Stack_version_major, BTLE_Stack_version_minor, BTLE_Stack_version_patch, BTLE_Stack_development;
  uint16_t DTM_Build_Number, BTLE_Stack_variant, BTLE_Stack_Build_Number;


  status = hci_read_local_version_information(&hci_version, &hci_revision, &lmp_pal_version, 
				                              &manufacturer_name, &lmp_pal_subversion);

  if (status == BLE_STATUS_SUCCESS) {
    *hwVersion = hci_revision >> 8;
  }
  else {
    BLE_MANAGER_PRINTF("Error= %x \r\n", status);
  }
  
  
  status = aci_hal_get_firmware_details(&DTM_version_major,
                                        &DTM_version_minor,
                                        &DTM_version_patch,
                                        &DTM_variant,
                                        &DTM_Build_Number,
                                        &BTLE_Stack_version_major,
                                        &BTLE_Stack_version_minor,
                                        &BTLE_Stack_version_patch,
                                        &BTLE_Stack_development,
                                        &BTLE_Stack_variant,
                                        &BTLE_Stack_Build_Number);
  
  if (status == BLE_STATUS_SUCCESS) {
    *fwVersion = BTLE_Stack_version_major  << 8;  // Major Version Number
    *fwVersion |= BTLE_Stack_version_minor << 4;  // Minor Version Number
    *fwVersion |= BTLE_Stack_version_patch;       // Patch Version Number
  }
  else {
    BLE_MANAGER_PRINTF("Error= %x \r\n", status);
  }
  
    
  return status;
}

/**
* @brief  This function makes the parsing of the Debug Console
* @param  uint8_t *att_data attribute data
* @param  uint8_t data_length length of the data
* @retval uint32_t SendBackData true/false
*/
static uint32_t DebugConsoleParsing(uint8_t * att_data, uint8_t data_length)
{
  /* By default Answer with the same message received */
  uint32_t SendBackData =1; 
 
  /* Received one write from Client on Terminal characteristc */
  SendBackData = DebugConsoleCommandParsing(att_data,data_length);
  
  return SendBackData;
}

/**
 * @brief  This function makes the parsing of the Debug Console Commands
 * @param  uint8_t *att_data attribute data
 * @param  uint8_t data_length length of the data
 * @retval uint32_t SendBackData true/false
 */
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;
  
  /* Help Command */
  if(!strncmp("help",(char *)(att_data),4))
  {
    /* Print Legend */
    SendBackData=0;

    BytesToWrite =sprintf((char *)BufferToWrite,"Command:\r\n"
      "pr->HW pedometer reset\r\n"
      "info-> System Info\r\n"
      "versionFw-> FW Version\r\n"
      "versionBle-> Ble Version\r\n");   
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if((att_data[0]=='p') & (att_data[1]=='r'))
  {
    /* Reset the pedometer DS3 HW counter */
    ResetHWPedometer();
    SendBackData=0;
    BytesToWrite =sprintf((char *)BufferToWrite,"Pedometer HW counter resetted\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if(!strncmp("versionFw",(char *)(att_data),9))
  {
    BytesToWrite =sprintf((char *)BufferToWrite,"%s_%s_%c.%c.%c\r\n",
                          BLE_STM32_MICRO,
                          MOTENV1_PACKAGENAME,
                          MOTENV1_VERSION_MAJOR,
                          MOTENV1_VERSION_MINOR,
                          MOTENV1_VERSION_PATCH);
    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;
  }
  else if(!strncmp("info",(char *)(att_data),4))
  {
    SendBackData=0;
      
    BytesToWrite =sprintf((char *)BufferToWrite,"\r\nSTMicroelectronics %s:\r\n"
        "\tVersion %c.%c.%c\r\n"
        "\tSTM32L053R8-Nucleo board"
         "\r\n",
         MOTENV1_PACKAGENAME,
         MOTENV1_VERSION_MAJOR,MOTENV1_VERSION_MINOR,MOTENV1_VERSION_PATCH);
    
    Term_Update(BufferToWrite,BytesToWrite);

    BytesToWrite =sprintf((char *)BufferToWrite,"\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n",
#elif defined (__CC_ARM)
        " (KEIL)\r\n",
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n",
#endif
        HAL_GetHalVersion() >>24,
        (HAL_GetHalVersion() >>16)&0xFF,
        (HAL_GetHalVersion() >> 8)&0xFF,
        HAL_GetHalVersion()      &0xFF,
        __DATE__,__TIME__);

    Term_Update(BufferToWrite,BytesToWrite);

    if(TargetBoardFeatures.IKS01Ax_support)
    {
      BytesToWrite =sprintf((char *)BufferToWrite,"Code compiled for X-NUCLEO-IKS01A3\r\n");
    }
    else
    {
      BytesToWrite =sprintf((char *)BufferToWrite,"\tX-NUCLEO-IKS01Ax Board not present\r\n");
    }
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if(!strncmp("versionBle",(char *)(att_data),10))
  {
    uint8_t  hwVersion;
    uint16_t fwVersion= 0;
    /* get the BlueNRG HW and FW versions */
    //getBlueNRGVersion(&hwVersion, &fwVersion);
    getBlueNRG2_Version(&hwVersion, &fwVersion);
      BytesToWrite =sprintf((char *)BufferToWrite,"%s_%d.%d.%c\r\n",
                            "BlueNRG2",
                            (fwVersion>>8)&0xF,
                            (fwVersion>>4)&0xF,
                            ('a' + (fwVersion&0xF)));
    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;
  } 
  else if((att_data[0]=='u') & (att_data[1]=='i') & (att_data[2]=='d'))
  {
    /* Write back the STM32 UID */
    uint8_t *uid = (uint8_t *)STM32_UUID;
    uint32_t MCU_ID = STM32_MCU_ID[0]&0xFFF;
    BytesToWrite =sprintf((char *)BufferToWrite,"%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                          uid[ 3],uid[ 2],uid[ 1],uid[ 0],
                          uid[ 7],uid[ 6],uid[ 5],uid[ 4],
                          uid[11],uid[ 10],uid[9],uid[8],
                          MCU_ID);
    Term_Update(BufferToWrite,BytesToWrite);
    SendBackData=0;
  }

#if 1
  /* If it's something not yet recognized... only for testing.. This must be removed*/
  if(SendBackData) {
    if(att_data[0]=='@') {
      if(att_data[1]=='T') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_TEMP1>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_TEMP1>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_TEMP1>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_TEMP1    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        WriteRequestConfigFunction(loc_att_data,loc_data_length);
        SendBackData = 0;
      } else if(att_data[1]=='A') {
        uint8_t loc_att_data[6];
        uint8_t loc_data_length=6;

        loc_att_data[0] = (FEATURE_MASK_ACC>>24)&0xFF;
        loc_att_data[1] = (FEATURE_MASK_ACC>>16)&0xFF;
        loc_att_data[2] = (FEATURE_MASK_ACC>>8 )&0xFF;
        loc_att_data[3] = (FEATURE_MASK_ACC    )&0xFF;
        loc_att_data[4] = 255;

        switch(att_data[2]) {
          case 'L':
            loc_att_data[5] = 50; /* @5S */
          break;
          case 'M':
            loc_att_data[5] = 10; /* @1S */
          break;
          case 'H':
            loc_att_data[5] = 1; /* @100mS */
          break;
          case 'D':
            loc_att_data[5] = 0; /* Default */
          break;
        }
        WriteRequestConfigFunction(loc_att_data,loc_data_length);
        SendBackData = 0;
      }
    }
  }
#endif
  return SendBackData;
}

/**
 * @brief  This function is called when there is a Bluetooth Read request.
 * @param  None 
 * @retval None
 */
static void ReadRequestEnvFunction(void)
{
  /* Read Request for Pressure,Humidity, and Temperatures*/
  int32_t PressToSend;
  uint16_t HumToSend;
  int16_t Temp1ToSend;
  int16_t Temp2ToSend;

  /* Read all the Environmental Sensors */
  ReadEnvironmentalData(&PressToSend,&HumToSend, &Temp1ToSend,&Temp2ToSend);
  
  /* Send the Data with BLE */
  BLE_EnvironmentalUpdate(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  
  BLE_MANAGER_PRINTF("Read for Env\r\n");
}

/**
 * @brief  This function is called when the peer device get disconnected.
 * @param  None 
 * @retval None
 */
static void DisconnectionCompletedFunction(void)
{
  connected = FALSE;
  
  AccEventEnabled= 0;
  LedEnabled= 0;
  
  /* Disable all timer */
  if(TIM2_CHANNEL_1_Enabled) {
    /* Stop the TIM Base generation in interrupt mode (for mic audio level) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }  
    
    TIM2_CHANNEL_1_Enabled= 0;
    EnvironmentalTimerEnabled= 0;
  }
  
  if(TIM2_CHANNEL_4_Enabled){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }
    
    TIM2_CHANNEL_4_Enabled= 0;
    InertialTimerEnabled= 0;
  }

  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
 * @brief  This function is called when there is a LE Connection Complete event.
 * @param  None 
 * @retval None
 */
static void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Addr[6])
{
  connected = TRUE;
  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
* @brief  This function makes the parsing of the Configuration Commands
* @param uint8_t *att_data attribute data
* @param uint8_t data_length length of the data
* @retval None
*/
static void WriteRequestConfigFunction(uint8_t * att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2]<<8) | (att_data[1]<<16) | (att_data[0]<<24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];
  
  switch (FeatureMask) {  
  case FEATURE_MASK_ACC_EVENTS:
    /* Acc events */
#ifdef MOTENV1_DEBUG_CONNECTION
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
      BytesToWrite =sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%c D=%x\r\n",FeatureMask,Command,Data);
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%c D=%x\r\n",FeatureMask,Command,Data);
    }
#endif /* MOTENV1_DEBUG_CONNECTION */      
    switch(Command) {
      case 'm':
        /* Multiple Events */
        switch(Data) {
          case 1:
            EnableHWMultipleEvents();
            ResetHWPedometer();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWMultipleEvents();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
        break;
      case 'f':
        /* FreeFall */
        switch(Data) {
          case 1:
            EnableHWFreeFall();
             Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWFreeFall();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
         }
      break;
      case 'd':
        /* Double Tap */
        switch(Data) {
          case 1:
            EnableHWDoubleTap();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWDoubleTap();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 's':
        /* Single Tap */
        switch(Data) {
          case 1:
            EnableHWSingleTap();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWSingleTap();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'p':
        /* Pedometer */
        switch(Data) {
          case 1:
            EnableHWPedometer();
            ResetHWPedometer();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWPedometer();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
      case 'w':
        /* Wake UP */
        switch(Data) {
          case 1:
            EnableHWWakeUp();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWWakeUp();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
       break;
       case 't':
         /* Tilt */
        switch(Data) {
          case 1:
            EnableHWTilt();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
          case 0:
            DisableHWTilt();
            Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
            break;
        }
      break;
      case 'o' :
        /* Tilt */
        switch(Data) {
        case 1:
          EnableHWOrientation6D();
          Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        case 0:
          DisableHWOrientation6D();
          Config_Update(FEATURE_MASK_ACC_EVENTS,Command,Data);
          break;
        }
      break;
    }
    break;
    case FEATURE_MASK_LED:
      /* Led events */
#ifdef MOTENV1_DEBUG_CONNECTION
      if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
        BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x\n\r",FeatureMask,Command);
        Term_Update(BufferToWrite,BytesToWrite);
      } else {
        BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%2x\r\n",FeatureMask,Command);
      }
#endif /* MOTENV1_DEBUG_CONNECTION */
     switch(Command) {
      case 1:
        LedOnTargetPlatform();
        Config_Update(FEATURE_MASK_LED,Command,Data);
        break;
      case 0:
        LedOffTargetPlatform();
        Config_Update(FEATURE_MASK_LED,Command,Data);
        break;
     }
     /* Update the LED feature */
     if(LedEnabled) {
       BLE_LedStatusUpdate(TargetBoardFeatures.LedStatus);
     }
    break;
    /* Environmental features */
    case FEATURE_MASK_TEMP1:
    case FEATURE_MASK_TEMP2:
    case FEATURE_MASK_PRESS:
    case FEATURE_MASK_HUM:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            uhCCR1_Val  = 1000*Data;
          } else {
            /* Default Values */
            uhCCR1_Val  = DEFAULT_uhCCR1_Val;
          }
#ifdef MOTENV1_DEBUG_CONNECTION
          if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
        break;
      }
    break;
    /* Inertial features */
    case FEATURE_MASK_ACC:
    case FEATURE_MASK_GRYO:
    case FEATURE_MASK_MAG:
      switch(Command) {
        case 255:
          /* Change the Sending interval */
          if(Data!=0) {
            /* Multiple of 100mS */
            uhCCR4_Val  = 1000*Data;
          } else {
            /* Default Value */
            uhCCR4_Val  = DEFAULT_uhCCR4_Val;
          }
#ifdef MOTENV1_DEBUG_CONNECTION
          if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
            BytesToWrite = sprintf((char *)BufferToWrite,"Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
            Term_Update(BufferToWrite,BytesToWrite);
          } else {
            BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%2x Data=%2x\n\r",FeatureMask,Command,Data);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
        break;
      }
    break;
  }
}

