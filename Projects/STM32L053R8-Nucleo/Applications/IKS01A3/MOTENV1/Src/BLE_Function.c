/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    BLE_Function.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    31-January-2023
  * @brief   BLE function API definition
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "HWAdvanceFeatures.h"
#include "BLE_Manager.h"
#include "app_motenv1.h"
#include "main.h"
#include "MOTENV1_config.h"

/* Exported Variables --------------------------------------------------------*/
uint8_t connected= FALSE;

/* Private variables ------------------------------------------------------------*/
volatile uint32_t FeatureMask;

static uint8_t TIM2_CHANNEL_1_Enabled= 0;
static uint8_t TIM2_CHANNEL_3_Enabled= 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t * att_data, uint8_t data_length);

static void AccEnv_StartStop(BLE_NotifyEvent_t Event);

static void TIM2_CHANNEL_1_StartStop(BLE_NotifyEvent_t Event);
static void TIM2_CHANNEL_3_StartStop(BLE_NotifyEvent_t Event);

/**
 * @brief  Set Board Name.
 * @param  None
 * @retval None
 */
void SetBoardName(void)
{
  sprintf(BLE_StackValue.BoardName,"%s%c%c%c","ME1V",
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH);
}

/**
 * @brief  Set Custom Advertize Data.
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetCustomAdvertiseData(uint8_t *manuf_data)
{
#ifndef BLE_MANAGER_SDKV2
  /**
  * For only SDKV1, user can add here the custom advertize data setting for the selected BLE features.
  * For example for the environmental features:
  *
  * BLE_SetCustomEnvAdvertizeData(manuf_data);
  */

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

#else /* BLE_MANAGER_SDKV2 */
  /* USER CODE BEGIN 2 */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1]=0x02; /* Custom Firmware */
  manuf_data[BLE_MANAGER_CUSTOM_FIELD2]=0x01;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3]=0x00;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4]=0x00;
  /* USER CODE END 2 */
#endif /* BLE_MANAGER_SDKV2 */
}

/**
* @brief  This function makes the parsing of the Debug Console
* @param  uint8_t *att_data attribute data
* @param  uint8_t data_length length of the data
* @retval uint32_t SendBackData true/false
*/
uint32_t DebugConsoleParsing(uint8_t * att_data, uint8_t data_length)
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
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
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

      BytesToWrite =sprintf((char *)BufferToWrite,"\tX-NUCLEO-IKS01A3 Board not present\r\n");
    }
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else if(!strncmp("versionBle",(char *)(att_data),10))
  {
    uint8_t  hwVersion;
    uint16_t fwVersion= 0;
    /* get the BlueNRG HW and FW versions */
    getBlueNRGVersion(&hwVersion, &fwVersion);
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
    uint8_t *uid = (uint8_t *)BLE_STM32_UUID;
    uint32_t MCU_ID = BLE_STM32_MCU_ID[0]&0xFFF;
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
 * @brief  This function is called when the peer device get disconnected.
 * @param  None
 * @retval None
 */
void DisconnectionCompletedFunction(void)
{
  connected = FALSE;

  AccEventEnabled= 0;
  LedEnabled= 0;

  /* Enable timer for led blinking */
  if(!LedTimerEnabled) {
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }

    LedTimerEnabled= 1;
  }

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

  if(TIM2_CHANNEL_3_Enabled){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_3_Enabled= 0;
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
void ConnectionCompletedFunction(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6])
{
  connected = TRUE;

  /* Disable timer for led blinking */
  if(LedTimerEnabled) {
    /* Stop the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }

    LedTimerEnabled= 0;
  }

  BLE_MANAGER_PRINTF("Call to ConnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
* @brief  This function makes the parsing of the Configuration Commands
* @param uint8_t *att_data attribute data
* @param uint8_t data_length length of the data
* @retval None
*/
void WriteRequestConfigFunction(uint8_t * att_data, uint8_t data_length)
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
            uhCCR1_Val  = ((SystemCoreClock / 10000) / (10 / Data));
          } else {
            /* Default Values */
            uhCCR1_Val  = Default_uhCCR1_Val;
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
            uhCCR3_Val  = ((SystemCoreClock / 10000) / (10 / Data));
          } else {
            /* Default Value */
            uhCCR3_Val  = Default_uhCCR3_Val;
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

/**************************************************
 * Callback functions to manage the notify events *
 **************************************************/
/**
  * @brief  Enable/Disable Led BLE Features
  * @param  BLE_NotifyEvent_t Event
  * @retval None
  */
void NotifyEventLed(BLE_NotifyEvent_t Event)
{
  /* Led Features */
  if(Event == BLE_NOTIFY_SUB)
  {
    LedEnabled= 1;
    BLE_LedStatusUpdate(TargetBoardFeatures.LedStatus);
  }

  if(Event == BLE_NOTIFY_UNSUB)
    LedEnabled= 0;
}

/**
  * @brief  Enable/Disable Accelerometer events BLE Features
  * @param  BLE_NotifyEvent_t Event
  * @retval None
  */
void NotifyEventAccEvent(BLE_NotifyEvent_t Event)
{
  /* Accelerometer events Features */
  if(Event != BLE_NOTIFY_NOTHING)
  {
    AccEnv_StartStop(Event);
  }
}

/**
  * @brief  Enable/Disable environmental BLE Features
  * @param  BLE_NotifyEvent_t Event
  * @retval None
  */
void NotifyEventEnv(BLE_NotifyEvent_t Event)
{
  /* Enviromental Features */
  if(Event != BLE_NOTIFY_NOTHING)
  {
    TIM2_CHANNEL_1_StartStop(Event);
  }
}

/**
  * @brief  Enable/Disable inertial BLE Features
  * @param  BLE_NotifyEvent_t Event
  * @retval None
  */
void NotifyEventInertial(BLE_NotifyEvent_t Event)
{
  /* Inertial Features */
  if(Event != BLE_NOTIFY_NOTHING)
  {
    TIM2_CHANNEL_3_StartStop(Event);
  }
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/

/**
  * @brief  Enable/Disable Accelerometer events
  * @param  None
  * @retval None
  */
static void AccEnv_StartStop(BLE_NotifyEvent_t Event)
{
  if( (Event == BLE_NOTIFY_SUB) &&
      (!AccEventEnabled) ){
        EnableHWMultipleEvents();
        ResetHWPedometer();
        Config_Update(FEATURE_MASK_ACC_EVENTS,'m',1);
        AccEventEnabled= 1;
  }

  if( (Event == BLE_NOTIFY_UNSUB) &&
      (AccEventEnabled) ){
        DisableHWMultipleEvents();
        AccEventEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 1
  * @param  None
  * @retval None
  */
static void TIM2_CHANNEL_1_StartStop(BLE_NotifyEvent_t Event)
{
  if( (Event == BLE_NOTIFY_SUB) &&
      (!TIM2_CHANNEL_1_Enabled) ){

    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    }

    TIM2_CHANNEL_1_Enabled= 1;
    EnvironmentalTimerEnabled= 1;
  }

  if( (Event == BLE_NOTIFY_UNSUB) &&
      (TIM2_CHANNEL_1_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_1_Enabled= 0;
    EnvironmentalTimerEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 3
  * @param  None
  * @retval None
  */
static void TIM2_CHANNEL_3_StartStop(BLE_NotifyEvent_t Event)
{
  if( (Event == BLE_NOTIFY_SUB) &&
      (!TIM2_CHANNEL_3_Enabled) ){
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    }

    TIM2_CHANNEL_3_Enabled= 1;
    InertialTimerEnabled= 1;
  }

  if( (Event == BLE_NOTIFY_UNSUB) &&
      (TIM2_CHANNEL_3_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_3_Enabled= 0;
    InertialTimerEnabled= 0;
  }
}
