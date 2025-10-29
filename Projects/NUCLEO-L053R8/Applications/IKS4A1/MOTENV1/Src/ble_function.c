/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_function.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @brief   BLE function API definition
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "hw_advance_features.h"
#include "ble_manager.h"
#include "app_motenv1.h"
#include "main.h"
#include "motenv1_config.h"

/* Exported Variables --------------------------------------------------------*/
/* Identifies if the IoT node is connect to BLE device */
uint8_t connected = FALSE;

/* Private variables ------------------------------------------------------------*/
volatile uint32_t FeatureMask;

/* Identifies if channel 1 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM2_CHANNEL_1_Enabled = 0;
/* Identifies if channel 3 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM2_CHANNEL_3_Enabled = 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length);

/* Start/Stop BLE notify for accelerometer events */
static void AccEnv_StartStop(ble_notify_event_t event);

/* Start/Stop channels 1 of the timer for BLE notify */
static void TIM2_CHANNEL_1_StartStop(ble_notify_event_t event);
/* Start/Stop channels 3 of the timer for BLE notify */
static void TIM2_CHANNEL_3_StartStop(ble_notify_event_t event);

/**
  * @brief  Set Board Name.
  * @param  None
  * @retval None
  */
void set_board_name(void)
{
  sprintf(ble_stack_value.board_name, "%s%c%c%c", "ME1V",
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH);
}

/**
  * @brief  Set Custom Advertize Data.
  * @param  uint8_t *manuf_data: Advertize Data
  * @retval None
  */
void ble_set_custom_advertise_data(uint8_t *manuf_data)
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
  manuf_data[BLE_MANAGER_CUSTOM_FIELD1] = CUSTOM_FIRMWARE_ID; /* Custom Firmware */
  if (TargetBoardFeatures.IKS01Ax_support)
  {
    manuf_data[BLE_MANAGER_CUSTOM_FIELD2] = 0x00;
  } /*  TargetBoardFeatures.IKS01Ax_support  */
  else
  {
    manuf_data[BLE_MANAGER_CUSTOM_FIELD2] = 0xFF;
  }
  manuf_data[BLE_MANAGER_CUSTOM_FIELD3] = 0x00;
  manuf_data[BLE_MANAGER_CUSTOM_FIELD4] = 0x00;
  /* USER CODE END 2 */
#endif /* BLE_MANAGER_SDKV2 */
}

/**
  * @brief  This function makes the parsing of the Debug Console
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval uint32_t SendBackData true/false
  */
uint32_t debug_console_parsing(uint8_t *att_data, uint8_t data_length)
{
  /* By default Answer with the same message received */
  uint32_t SendBackData = 1;

  /* Received one write from Client on Terminal characteristc */
  SendBackData = DebugConsoleCommandParsing(att_data, data_length);

  return SendBackData;
}

/**
  * @brief  This function makes the parsing of the Debug Console Commands
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval uint32_t SendBackData true/false
  */
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length)
{
  uint32_t SendBackData = 1;

  /* Help Command */
  if (!strncmp("help", (char *)(att_data), 4))
  {
    /* Print Legend */
    SendBackData = 0;

    bytes_to_write = sprintf((char *)buffer_to_write, "Command:\r\n"
                             "pr->HW pedometer reset\r\n"
                             "info-> System Info\r\n"
                             "versionBle-> Ble Version\r\n");
    term_update(buffer_to_write, bytes_to_write);
  }
  /* HW pedometer reset command */
  else if ((att_data[0] == 'p') & (att_data[1] == 'r'))
  {
    /* Reset the pedometer DS3 HW counter */
    ResetHWPedometer();
    SendBackData = 0;
    bytes_to_write = sprintf((char *)buffer_to_write, "Pedometer HW counter reset\r\n");
    term_update(buffer_to_write, bytes_to_write);
  }
  /* info command */
  else if (!strncmp("info", (char *)(att_data), 4))
  {
    SendBackData = 0;

    bytes_to_write = sprintf((char *)buffer_to_write, "\r\nSTMicroelectronics %s:\r\n"
                             "\tVersion %c.%c.%c\r\n"
                             "\tSTM32L053R8-Nucleo board"
                             "\r\n",
                             MOTENV1_PACKAGENAME,
                             MOTENV1_VERSION_MAJOR, MOTENV1_VERSION_MINOR, MOTENV1_VERSION_PATCH);

    term_update(buffer_to_write, bytes_to_write);

    bytes_to_write = sprintf((char *)buffer_to_write, "\t(HAL %ld.%ld.%ld_%ld)\r\n"
                             "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                             " (IAR)\r\n",
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
                             " (KEIL)\r\n",
#elif defined (__GNUC__)
                             " (STM32CubeIDE)\r\n",
#endif /* IDE */
                             HAL_GetHalVersion() >> 24,
                             (HAL_GetHalVersion() >> 16) & 0xFF,
                             (HAL_GetHalVersion() >> 8) & 0xFF,
                             HAL_GetHalVersion()      & 0xFF,
                             __DATE__, __TIME__);

    term_update(buffer_to_write, bytes_to_write);

    if (TargetBoardFeatures.IKS01Ax_support)
    {
      bytes_to_write = sprintf((char *)buffer_to_write, "Code compiled for X-NUCLEO-IKS4A1\r\n");
    }
    else
    {
      bytes_to_write = sprintf((char *)buffer_to_write, "\tX-NUCLEO-IKS4A1 Board not present\r\n");
    }
    term_update(buffer_to_write, bytes_to_write);
  }
  /* versionBle command */
  else if (!strncmp("versionBle", (char *)(att_data), 10))
  {
    uint8_t  hwVersion;
    uint16_t fwVersion = 0;
    /* get the BlueNRG HW and FW versions */
    get_blue_nrg_version(&hwVersion, &fwVersion);
    bytes_to_write = sprintf((char *)buffer_to_write, "%s_%d.%d.%c\r\n",
                             "BlueNRG2",
                             (fwVersion >> 8) & 0xF,
                             (fwVersion >> 4) & 0xF,
                             ('a' + (fwVersion & 0xF)));
    term_update(buffer_to_write, bytes_to_write);
    SendBackData = 0;
  }
  /* UID command */
  else if ((att_data[0] == 'u') & (att_data[1] == 'i') & (att_data[2] == 'd'))
  {
    /* Write back the STM32 UID */
    uint8_t *uid = (uint8_t *)BLE_STM32_UUID;
    uint32_t MCU_ID = BLE_STM32_MCU_ID[0] & 0xFFF;
    bytes_to_write = sprintf((char *)buffer_to_write, "%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X%.2X_%.3lX\r\n",
                             uid[ 3], uid[ 2], uid[ 1], uid[ 0],
                             uid[ 7], uid[ 6], uid[ 5], uid[ 4],
                             uid[11], uid[ 10], uid[9], uid[8],
                             MCU_ID);
    term_update(buffer_to_write, bytes_to_write);
    SendBackData = 0;
  }

  return SendBackData;
}

/**
  * @brief  This function is called when the peer device get disconnected.
  * @param  None
  * @retval None
  */
void disconnection_completed_function(void)
{
  connected = FALSE;

  AccEventEnabled = 0;
  LedEnabled = 0;

  /* Enable timer for led blinking */
  if (!LedTimerEnabled)
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TIM_CC_HANDLE, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TIM_CC_HANDLE);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    }

    LedTimerEnabled = 1;
  }

  /* Disable all timer */
  if (TIM2_CHANNEL_1_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for mic audio level) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_1_Enabled = 0;
    EnvironmentalTimerEnabled = 0;
  }

  if (TIM2_CHANNEL_3_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_3_Enabled = 0;
    InertialTimerEnabled = 0;
  }

  BLE_MANAGER_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  BLE_MANAGER_DELAY(100);
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  None
  * @retval None
  */
void connection_completed_function(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6])
{
  connected = TRUE;

  /* Disable timer for led blinking */
  if (LedTimerEnabled)
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    LedTimerEnabled = 0;
  }

  /* Force switch off the led */
  LedOffTargetPlatform();

  MOTENV1_PRINTF("Call to ConnectionCompletedFunction\r\n");
  HAL_Delay(100);
}

/**
  * @brief  This function makes the parsing of the Configuration Commands
  * @param uint8_t *att_data attribute data
  * @param uint8_t data_length length of the data
  * @retval None
  */
void write_request_config_function(uint8_t *att_data, uint8_t data_length)
{
  uint32_t FeatureMask = (att_data[3]) | (att_data[2] << 8) | (att_data[1] << 16) | (att_data[0] << 24);
  uint8_t Command = att_data[4];
  uint8_t Data    = att_data[5];

  switch (FeatureMask)
  {
    case FEATURE_MASK_ACC_EVENTS:
      /* Acc events */
#ifdef MOTENV1_DEBUG_CONNECTION
      if (ble_std_term_service == BLE_SERV_ENABLE)
      {
        bytes_to_write = sprintf((char *)buffer_to_write, "Conf Sig F=%lx C=%c D=%x\r\n", FeatureMask, Command, Data);
        term_update(buffer_to_write, bytes_to_write);
      }
      else
      {
        BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%c D=%x\r\n", FeatureMask, Command, Data);
      }
#endif /* MOTENV1_DEBUG_CONNECTION */
      switch (Command)
      {
        case 'm':
          /* Multiple Events */
          switch (Data)
          {
            case 1:
              EnableHWMultipleEvents();
              ResetHWPedometer();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWMultipleEvents();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 'f':
          /* FreeFall */
          switch (Data)
          {
            case 1:
              EnableHWFreeFall();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWFreeFall();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 'd':
          /* Double Tap */
          switch (Data)
          {
            case 1:
              EnableHWDoubleTap();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWDoubleTap();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 's':
          /* Single Tap */
          switch (Data)
          {
            case 1:
              EnableHWSingleTap();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWSingleTap();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 'p':
          /* Pedometer */
          switch (Data)
          {
            case 1:
              EnableHWPedometer();
              ResetHWPedometer();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWPedometer();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 'w':
          /* Wake UP */
          switch (Data)
          {
            case 1:
              EnableHWWakeUp();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWWakeUp();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 't':
          /* Tilt */
          switch (Data)
          {
            case 1:
              EnableHWTilt();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWTilt();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
        case 'o' :
          /* Tilt */
          switch (Data)
          {
            case 1:
              EnableHWOrientation6D();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
            case 0:
              DisableHWOrientation6D();
              config_update(FEATURE_MASK_ACC_EVENTS, Command, Data);
              break;
          }
          break;
      }
      break;
    case FEATURE_MASK_LED:
      /* Led events */
#ifdef MOTENV1_DEBUG_CONNECTION
      if (ble_std_term_service == BLE_SERV_ENABLE)
      {
        bytes_to_write = sprintf((char *)buffer_to_write, "Conf Sig F=%lx C=%2x\n\r", FeatureMask, Command);
        term_update(buffer_to_write, bytes_to_write);
      }
      else
      {
        BLE_MANAGER_PRINTF("Conf Sig F=%lx C=%2x\r\n", FeatureMask, Command);
      }
#endif /* MOTENV1_DEBUG_CONNECTION */
      switch (Command)
      {
        case 1:
          LedOnTargetPlatform();
          config_update(FEATURE_MASK_LED, Command, Data);
          break;
        case 0:
          LedOffTargetPlatform();
          config_update(FEATURE_MASK_LED, Command, Data);
          break;
      }
      /* Update the LED feature */
      if (LedEnabled)
      {
        ble_led_status_update(TargetBoardFeatures.LedStatus);
      }
      break;
  }
}

/***************************************************
  * Callback functions to manage the notify events *
  **************************************************/
/**
  * @brief  Enable/Disable Led BLE Features
  * @param  ble_notify_event_t event
  * @retval None
  */
void notify_event_led(ble_notify_event_t event)
{
  /* Led Features */
  if (event == BLE_NOTIFY_SUB)
  {
    LedEnabled = 1;
    ble_led_status_update(TargetBoardFeatures.LedStatus);
  }

  if (event == BLE_NOTIFY_UNSUB)
  {
    LedEnabled = 0;
  }
}

/**
  * @brief  Callback Function for Led read request.
  * @param  uint8_t *led_status Status of the led
  * @retval None
  */
void read_request_led_function(uint8_t *led_status)
{
  *led_status = TargetBoardFeatures.LedStatus;
}

/**
  * @brief  Enable/Disable Accelerometer events BLE Features
  * @param  ble_notify_event_t event
  * @retval None
  */
void notify_event_acc_event(ble_notify_event_t event)
{
  /* Accelerometer events Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    AccEnv_StartStop(event);
  }
}

/**
  * @brief  Enable/Disable environmental BLE Features
  * @param  ble_notify_event_t event
  * @retval None
  */
void notify_event_env(ble_notify_event_t event)
{
  /* Environmental Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM2 Channel 1 */
    TIM2_CHANNEL_1_StartStop(event);
  }
}

/**
  * @brief  Enable/Disable inertial BLE Features
  * @param  ble_notify_event_t event
  * @retval None
  */
void notify_event_inertial(ble_notify_event_t event)
{
  /* Inertial Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM2 Channel 3 */
    TIM2_CHANNEL_3_StartStop(event);
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
static void AccEnv_StartStop(ble_notify_event_t event)
{
  if ((event == BLE_NOTIFY_SUB) &&
      (!AccEventEnabled))
  {
    ResetHWPedometer();
    AccEventEnabled = 1;
  }

  if ((event == BLE_NOTIFY_UNSUB) &&
      (AccEventEnabled))
  {
    DisableHWMultipleEvents();
    AccEventEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 1
  * @param  None
  * @retval None
  */
static void TIM2_CHANNEL_1_StartStop(ble_notify_event_t event)
{
  /* Enable TIM2 Channel 1 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM2_CHANNEL_1_Enabled))
  {

    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TIM_CC_HANDLE);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    }

    TIM2_CHANNEL_1_Enabled = 1;
    EnvironmentalTimerEnabled = 1;
  }

  /* Disable TIM2 Channel 1 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM2_CHANNEL_1_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_1_Enabled = 0;
    EnvironmentalTimerEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 3
  * @param  None
  * @retval None
  */
static void TIM2_CHANNEL_3_StartStop(ble_notify_event_t event)
{
  /* Enable TIM2 Channel 3 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM2_CHANNEL_3_Enabled))
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TIM_CC_HANDLE, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TIM_CC_HANDLE);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));
    }

    TIM2_CHANNEL_3_Enabled = 1;
    InertialTimerEnabled = 1;
  }

  /* Disable TIM2 Channel 3 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM2_CHANNEL_3_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM2_CHANNEL_3_Enabled = 0;
    InertialTimerEnabled = 0;
  }
}
