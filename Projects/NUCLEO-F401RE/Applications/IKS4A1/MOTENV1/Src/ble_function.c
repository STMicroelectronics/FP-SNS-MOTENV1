/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ble_function.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
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
#include "ble_function.h"
#include "app_motenv1.h"
#include "main.h"
#include "motenv1_config.h"

/* Exported Variables --------------------------------------------------------*/
/* Identifies if the IoT node is connect to BLE device */
uint8_t connected = FALSE;
/* Identifies if the IoT node is connected to BLE device for a first time or not */
uint32_t FirstConnectionConfig = 0;

/* Private variables ------------------------------------------------------------*/
volatile uint32_t FeatureMask;

static uint16_t BLE_ConnectionHandle = 0;
/* Counter for number bytes remaining to send via BLE for the OTA */
static uint32_t OTA_RemainingSize = 0;
/* Used to restore the default accelerometer data rate value */
static float UsedAccelerometerDataRate;

/* Identifies if channel 1 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM1_CHANNEL_1_Enabled = 0;
/* Identifies if channel 2 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM1_CHANNEL_2_Enabled = 0;
/* Identifies if channel 3 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM1_CHANNEL_3_Enabled = 0;
/* Identifies if channel 4 of the timer is enabled (1) or disabled (0) */
static uint8_t TIM1_CHANNEL_4_Enabled = 0;

/* Private functions ---------------------------------------------------------*/
static uint32_t DebugConsoleCommandParsing(uint8_t *att_data, uint8_t data_length);

/* Start/Stop BLE notify for accelerometer events */
static void AccEnv_StartStop(ble_notify_event_t event);
/* Start/Stop environmental timer for BLE notify */
static void Environmental_StartStopTimer(ble_notify_event_t event);

/* Start/Stop channels 1 of the timer for BLE notify */
static void TIM1_CHANNEL_1_StartStop(ble_notify_event_t event, uint8_t Algorithm);
/* Start/Stop channels 2 of the timer for BLE notify */
static void TIM1_CHANNEL_2_StartStop(ble_notify_event_t event, uint8_t Algorithm);
/* Start/Stop channels 3 of the timer for BLE notify */
static void TIM1_CHANNEL_3_StartStop(ble_notify_event_t event, uint8_t Algorithm);
/* Start/Stop channels 4 of the timer for BLE notify */
static void TIM1_CHANNEL_4_StartStop(ble_notify_event_t event);

/**
  * @brief  Set Board Name.
  * @param  None
  * @retval None
  */
void set_board_name(void)
{
  /* Set Default Node Name */
  sprintf(ble_stack_value.board_name, "%s%c%c%c", "ME1V",
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH);

  /* Set Node Name */
  ReCallNodeNameFromMemory();
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

  if (OTA_RemainingSize != 0)
  {
    /* Firmware update */
    int8_t RetValue = UpdateFW(&OTA_RemainingSize, att_data, data_length, 1);
    if (RetValue != 0)
    {
      term_update((uint8_t *)&RetValue, 1);
      if (RetValue == 1)
      {
        /* if OTA checked */
        /* bytes_to_write =sprintf((char *)buffer_to_write,"The Board will restart in 5 seconds\r\n"); */
        /* term_update(buffer_to_write,bytes_to_write); */
        MOTENV1_PRINTF("%s will restart in 5 seconds\r\n", MOTENV1_PACKAGENAME);
        HAL_Delay(5000);
        /* Board restarts */
        HAL_NVIC_SystemReset();
      }
    }
    SendBackData = 0;
  }
  else
  {
    /* Received one write from Client on Terminal characteristc */
    SendBackData = DebugConsoleCommandParsing(att_data, data_length);
  }

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
    /* List of the available commands */
    bytes_to_write = sprintf((char *)buffer_to_write, "Command:\r\n"
                             "pr->HW pedometer reset\r\n"
                             "info-> System Info\r\n"
                             "versionFw-> FW Version\r\n"
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
                             "\tSTM32F401RE-Nucleo board"
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
  /* versionFw command */
  else if (!strncmp("versionFw", (char *)(att_data), 9))
  {
    bytes_to_write = sprintf((char *)buffer_to_write, "%s_%s_%c.%c.%c\r\n",
                             BLE_STM32_MICRO,
                             MOTENV1_PACKAGENAME,
                             MOTENV1_VERSION_MAJOR,
                             MOTENV1_VERSION_MINOR,
                             MOTENV1_VERSION_PATCH);
    term_update(buffer_to_write, bytes_to_write);
    SendBackData = 0;
  }
  /* upgradeFw command */
  else if (!strncmp("upgradeFw", (char *)(att_data), 9))
  {
    uint32_t OTA_crc;
    uint8_t *PointerByte = (uint8_t *) &OTA_RemainingSize;

    OTA_RemainingSize = atoi((char *)(att_data + 9));
    PointerByte[0] = att_data[ 9];
    PointerByte[1] = att_data[10];
    PointerByte[2] = att_data[11];
    PointerByte[3] = att_data[12];

    /* Check the Maximum Possible OTA size */
    if (OTA_RemainingSize > OTA_MAX_PROG_SIZE)
    {
      MOTENV1_PRINTF("OTA %s SIZE=%ld > %ld Max Allowed\r\n",
                     MOTENV1_PACKAGENAME,
                     OTA_RemainingSize,
                     OTA_MAX_PROG_SIZE);
      /* Answer with a wrong CRC value for signaling the problem to BlueMS application */
      buffer_to_write[0] = att_data[13];
      buffer_to_write[1] = (att_data[14] != 0) ? 0 : 1; /* In order to be sure to have a wrong CRC */
      buffer_to_write[2] = att_data[15];
      buffer_to_write[3] = att_data[16];
      bytes_to_write = 4;
      term_update(buffer_to_write, bytes_to_write);
    }
    else
    {
      PointerByte = (uint8_t *) &OTA_crc;
      PointerByte[0] = att_data[13];
      PointerByte[1] = att_data[14];
      PointerByte[2] = att_data[15];
      PointerByte[3] = att_data[16];

      MOTENV1_PRINTF("OTA %s SIZE=%ld OTA_crc=%lx\r\n", MOTENV1_PACKAGENAME, OTA_RemainingSize, OTA_crc);

      /* Reset the Flash */
      StartUpdateFW(OTA_RemainingSize, OTA_crc);

      /* Save the Meta Data.
       * We had always a Meta Data */
      SaveMetaData();

      /* Reduce the connection interval */
      {
        uint8_t ret = aci_l2cap_connection_parameter_update_req(BLE_ConnectionHandle,
                                                                10 /* interval_min*/,
                                                                10 /* interval_max */,
                                                                0   /* slave_latency */,
                                                                400 /*timeout_multiplier*/);

        /* Go to infinite loop if there is one error */
        if (ret != BLE_STATUS_SUCCESS)
        {
          while (1)
          {
            MOTENV1_PRINTF("Problem Changing the connection interval\r\n");
          }
        }
      }

      /* Signal that we are ready sending back the CRC value*/
      buffer_to_write[0] = PointerByte[0];
      buffer_to_write[1] = PointerByte[1];
      buffer_to_write[2] = PointerByte[2];
      buffer_to_write[3] = PointerByte[3];
      bytes_to_write = 4;
      term_update(buffer_to_write, bytes_to_write);
    }

    SendBackData = 0;
  }
  /* versionBle command */
  else if (!strncmp("versionBle", (char *)(att_data), 10))
  {
    uint8_t  hwVersion;
    uint16_t fwVersion;
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
  /* setName command */
  else if (!strncmp("setName ", (char *)(att_data), 8))
  {
    uint16_t NameLength = data_length - 1;

    if (NameLength > 8)
    {
      for (uint16_t i = 0; i < 7; i++)
      {
        ble_stack_value.board_name[i] = atoi(" ");
      }

      if ((NameLength - 8) > 7)
      {
        NameLength = 7;
      }
      else NameLength = NameLength - 8;

      for (uint16_t i = 0; i < NameLength; i++)
      {
        ble_stack_value.board_name[i] = att_data[i + 8];
      }

      UpdateNodeNameMetaData();

      bytes_to_write = sprintf((char *)buffer_to_write, "\nThe node name has been updated\r\n");
      term_update(buffer_to_write, bytes_to_write);
      bytes_to_write = sprintf((char *)buffer_to_write, "Disconnecting and riconnecting to see the new node name\r\n");
      term_update(buffer_to_write, bytes_to_write);
    }
    else
    {
      bytes_to_write = sprintf((char *)buffer_to_write, "\nInsert the node name\r\n");
      term_update(buffer_to_write, bytes_to_write);
      bytes_to_write = sprintf((char *)buffer_to_write, "Use command: setName 'xxxxxxx'\r\n");
      term_update(buffer_to_write, bytes_to_write);
    }

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

  ForceReCalibration = 0;
  FirstConnectionConfig = 0;

  AccEventEnabled = 0;
  LedEnabled = 0;

  /* Enable timer for led blinking */
  if (!LedTimerEnabled)
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_Base_Start_IT(&TIM_LED_HANDLE) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    LedTimerEnabled = 1;
  }

  /* Disable all timer */
  if (EnvironmentalTimerEnabled)
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_Base_Stop_IT(&TIM_ENV_HANDLE) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    EnvironmentalTimerEnabled = 0;
  }

  if (TIM1_CHANNEL_1_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for mic audio level) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM1_CHANNEL_1_Enabled = 0;
    SensorFusionEnabled = 0;
    ECompassEnabled = 0;
  }

  if (TIM1_CHANNEL_2_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM1_CHANNEL_2_Enabled = 0;
    CarryPositionEnabled = 0;
    GestureRecognitionEnabled = 0;
    PedometerAlgorithmEnabled = 0;
  }

  if (TIM1_CHANNEL_3_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM1_CHANNEL_3_Enabled = 0;
    ActivityRecognitionEnabled = 0;
    MotionIntensityEnabled = 0;
  }

  if (TIM1_CHANNEL_4_Enabled)
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM1_CHANNEL_4_Enabled = 0;
    InertialTimerEnabled = 0;
  }

  /* Reset for any problem during FOTA update */
  OTA_RemainingSize = 0;

  MOTENV1_PRINTF("Call to DisconnectionCompletedFunction\r\n");
  HAL_Delay(100);
}

/**
  * @brief  This function is called when there is a LE Connection Complete event.
  * @param  None
  * @retval None
  */
void connection_completed_function(uint16_t ConnectionHandle, uint8_t Address_Type, uint8_t addr[6])
{
  BLE_ConnectionHandle = ConnectionHandle;

  connected = TRUE;

  ForceReCalibration = 0;
  FirstConnectionConfig = 0;

  /* Disable timer for led blinking */
  if (LedTimerEnabled)
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_Base_Stop_IT(&TIM_LED_HANDLE) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    LedTimerEnabled = 0;
  }

  /* Force switch off the led if the board is not calibrated */
  if (!isCal)
  {
    LedOffTargetPlatform();
  }

  MOTENV1_PRINTF("Call to ConnectionCompletedFunction\r\n");
  HAL_Delay(100);
}

/**
  * @brief  This function is called when there is a change on the gatt attribute
  * @param  uint8_t *att_data attribute data
  * @param  uint8_t data_length length of the data
  * @retval None
  */
void attr_mod_config_function(uint8_t *att_data, uint8_t data_length)
{
  if (att_data[0] == 01)
  {
    config_update(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);
    config_update(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);
    FirstConnectionConfig = 1;
  }
  else if (att_data[0] == 0)
  {
    FirstConnectionConfig = 0;
  }
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
    case FEATURE_MASK_SENSORFUSION_SHORT:
      /* Sensor Fusion */
      switch (Command)
      {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration STATUS Signal For Features=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration STATUS Signal For Features=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            config_update(FeatureMask, Command, isCal ? 100 : 0);
          }
          break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration RESET Signal For Feature=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration RESET Signal For Feature=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReCalibration = 1;
          break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration STOP Signal For Feature=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration STOP Signal For Feature=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Do nothing in this case */
          break;
        default:
          if (ble_std_err_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration UNKNOWN Signal For Feature=%lx\n\r",
                                     FeatureMask);
            std_err_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration UNKNOWN Signal For Feature=%lx\n\r", FeatureMask);
          }
      }
      break;
    case FEATURE_MASK_ECOMPASS:
      /* e-compass */
      switch (Command)
      {
        case W2ST_COMMAND_CAL_STATUS:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration STATUS Signal For Features=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration STATUS Signal For Features=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Replay with the calibration status for the feature */
          /* Control the calibration status */
          {
            config_update(FeatureMask, Command, isCal ? 100 : 0);
          }
          break;
        case W2ST_COMMAND_CAL_RESET:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration RESET Signal For Feature=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration RESET Signal For Feature=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Reset the calibration */
          ForceReCalibration = 1;
          break;
        case W2ST_COMMAND_CAL_STOP:
#ifdef MOTENV1_DEBUG_CONNECTION
          if (ble_std_term_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration STOP Signal For Feature=%lx\n\r",
                                     FeatureMask);
            term_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration STOP Signal For Feature=%lx\n\r", FeatureMask);
          }
#endif /* MOTENV1_DEBUG_CONNECTION */
          /* Do nothing in this case */
          break;
        default:
          if (ble_std_err_service == BLE_SERV_ENABLE)
          {
            bytes_to_write = sprintf((char *)buffer_to_write,
                                     "Calibration UNKNOWN Signal For Feature=%lx\n\r",
                                     FeatureMask);
            std_err_update(buffer_to_write, bytes_to_write);
          }
          else
          {
            MOTENV1_PRINTF("Calibration UNKNOWN Signal For Feature=%lx\n\r", FeatureMask);
          }
      }
      break;
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
        MOTENV1_PRINTF("Conf Sig F=%lx C=%c D=%x\r\n", FeatureMask, Command, Data);
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
        MOTENV1_PRINTF("Conf Sig F=%lx C=%2x\r\n", FeatureMask, Command);
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
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
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
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
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
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_activity_recognition(ble_notify_event_t event)
{
  /* Activity Recognition Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 3 */
    TIM1_CHANNEL_3_StartStop(event, 0);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_carry_position(ble_notify_event_t event)
{
  /* Carry Position Algorithm Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 2 */
    TIM1_CHANNEL_2_StartStop(event, 0);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_e_compass(ble_notify_event_t event)
{
  /* E-Compass Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 1 */
    TIM1_CHANNEL_1_StartStop(event, 1);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_env(ble_notify_event_t event)
{
  /* Environmental Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable timer for Environmental Features */
    Environmental_StartStopTimer(event);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_gesture_recognition(ble_notify_event_t event)
{
  /* Gesture Recognition Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 2 */
    TIM1_CHANNEL_2_StartStop(event, 1);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_inertial(ble_notify_event_t event)
{
  /* Inertial Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 4 */
    TIM1_CHANNEL_4_StartStop(event);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_motion_intensity(ble_notify_event_t event)
{
  /* Motion Intensity Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 3 */
    TIM1_CHANNEL_3_StartStop(event, 1);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_pedometer_algorithm(ble_notify_event_t event)
{
  /* Pedometer Algorithm Features */
  {
    ble_pm_output_t PM_DataOUT_Sent;

    PM_DataOUT_Sent.cadence = 0;
    PM_DataOUT_Sent.n_steps = 0;

    PM_DataOUTStored.Cadence = 0;
    PM_DataOUTStored.Nsteps = 0;

    ble_pedometer_algorithm_update(&PM_DataOUT_Sent);

    /* Enable/Disable TIM1 Channel 2 */
    TIM1_CHANNEL_2_StartStop(event, 2);
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  ble_notify_event_t event: Subscription/Unsubscription BLE event
  * @retval None
  */
void notify_event_sensor_fusion(ble_notify_event_t event)
{
  /* Sensor Fusion Features */
  if (event != BLE_NOTIFY_NOTHING)
  {
    /* Enable/Disable TIM1 Channel 1 */
    TIM1_CHANNEL_1_StartStop(event, 0);
  }
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/

/**
  * @brief  This function is called when there is a change on the gatt attribute for Environmental
  *         for Start/Stop Timer
  * @param  None
  * @retval None
  */
static void Environmental_StartStopTimer(ble_notify_event_t event)
{
  if ((event == BLE_NOTIFY_SUB) &&
      (!EnvironmentalTimerEnabled))
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_Base_Start_IT(&TIM_ENV_HANDLE) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    EnvironmentalTimerEnabled = 1;
  }

  if ((event == BLE_NOTIFY_UNSUB) &&
      (EnvironmentalTimerEnabled))
  {
    /* Stop the TIM Base generation in interrupt mode */
    if (HAL_TIM_Base_Stop_IT(&TIM_ENV_HANDLE) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    EnvironmentalTimerEnabled = 0;
  }
}

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
static void TIM1_CHANNEL_1_StartStop(ble_notify_event_t event, uint8_t Algorithm)
{
  /* Enable TIM1 Channel 1 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM1_CHANNEL_1_Enabled))
  {

    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GET_OUTPUT_DATA_RATE(ACCELERO_INSTANCE, MOTION_ACCELERO, &UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 100 Hz*/
    MOTION_SENSOR_SET_OUTPUT_DATA_RATE(ACCELERO_INSTANCE, MOTION_ACCELERO, (float)ALGO_FREQ_FX);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();

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

    TIM1_CHANNEL_1_Enabled = 1;

    if (Algorithm == 0)
    {
      SensorFusionEnabled = 1;
    }

    if (Algorithm == 1)
    {
      ECompassEnabled = 1;
    }
  }

  /* Disable TIM1 Channel 1 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM1_CHANNEL_1_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_1) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    /* Set default ODR accelerometer value */
    MOTION_SENSOR_SET_OUTPUT_DATA_RATE(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();

    TIM1_CHANNEL_1_Enabled = 0;
    SensorFusionEnabled = 0;
    ECompassEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 2
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_2_StartStop(ble_notify_event_t event, uint8_t Algorithm)
{
  /* Enable TIM1 Channel 2 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM1_CHANNEL_2_Enabled))
  {
    /* Start the TIM Base generation in interrupt mode */
    if (HAL_TIM_OC_Start_IT(&TIM_CC_HANDLE, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TIM_CC_HANDLE);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    }

    Set4GAccelerometerFullScale();

    TIM1_CHANNEL_2_Enabled = 1;

    if (Algorithm == 0)
    {
      CarryPositionEnabled = 1;
    }
    if (Algorithm == 1)
    {
      GestureRecognitionEnabled = 1;
    }
    if (Algorithm == 2)
    {
      PedometerAlgorithmEnabled = 1;
    }
  }

  /* Disable TIM1 Channel 2 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM1_CHANNEL_2_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_2) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    Set2GAccelerometerFullScale();

    TIM1_CHANNEL_2_Enabled = 0;
    CarryPositionEnabled = 0;
    GestureRecognitionEnabled = 0;
    PedometerAlgorithmEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 3
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_3_StartStop(ble_notify_event_t event, uint8_t Algorithm)
{
  /* Enable TIM1 Channel 3 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM1_CHANNEL_3_Enabled))
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

    Set4GAccelerometerFullScale();

    TIM1_CHANNEL_3_Enabled = 1;

    if (Algorithm == 0)
    {
      ActivityRecognitionEnabled = 1;
      MotionAR_Reset();
    }
    if (Algorithm == 1)
    {
      MotionIntensityEnabled = 1;
    }
  }

  /* Disable TIM1 Channel 3 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM1_CHANNEL_3_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_3) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    Set2GAccelerometerFullScale();
    TIM1_CHANNEL_3_Enabled = 0;
    ActivityRecognitionEnabled = 0;
    MotionIntensityEnabled = 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 4
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_4_StartStop(ble_notify_event_t event)
{
  /* Enable TIM1 Channel 4 */
  if ((event == BLE_NOTIFY_SUB) &&
      (!TIM1_CHANNEL_4_Enabled))
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

    TIM1_CHANNEL_4_Enabled = 1;
    InertialTimerEnabled = 1;
  }

  /* Disable TIM1 Channel 4 */
  if ((event == BLE_NOTIFY_UNSUB) &&
      (TIM1_CHANNEL_4_Enabled))
  {
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if (HAL_TIM_OC_Stop_IT(&TIM_CC_HANDLE, TIM_CHANNEL_4) != HAL_OK)
    {
      /* Stopping Error */
      Error_Handler();
    }

    TIM1_CHANNEL_4_Enabled = 0;
    InertialTimerEnabled = 0;
  }
}

/************************************************************************************
  * Callback functions to manage the extended configuration characteristic commands *
  ***********************************************************************************/
/**
  * @brief  Callback Function for answering to VersionFw command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
void ext_config_version_fw_command_callback(uint8_t *Answer)
{
  /* Write package name and version firmware info */
  sprintf((char *)Answer, "%s_%s_%c.%c.%c",
          BLE_STM32_MICRO,
          MOTENV1_PACKAGENAME,
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH);
}

/**
  * @brief  Callback Function for answering to Info command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
void ext_config_info_command_callback(uint8_t *Answer)
{
  /* BlueNRG HW version info */
  uint8_t  hwVersion;
  /* BlueNRG FW version info */
  uint16_t fwVersion;

  /* get the BlueNRG HW and FW versions */
  get_blue_nrg_version(&hwVersion, &fwVersion);
  /* Write firmware info */
  sprintf((char *)Answer, "STMicroelectronics %s:\n"
          "Version %c.%c.%c\n"
          "%s board\n"
          "BlueNRG-2 HW ver%d.%d\n"
          "BlueNRG-2 FW ver%d.%d.%c\n"
          "(HAL %ld.%ld.%ld_%ld)\n"
          "Compiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
          " (IAR)"
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
          " (KEIL)"
#elif defined (__GNUC__)
          " (STM32CubeIDE)"
#endif /* IDE */
          "\nCode compiled for X-NUCLEO-IKS4A1 board\n",
          MOTENV1_PACKAGENAME,
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH,
          BLE_STM32_BOARD,
          ((hwVersion >> 4) & 0x0F),
          (hwVersion & 0x0F),
          (fwVersion >> 8) & 0xF,
          (fwVersion >> 4) & 0xF,
          ('a' + (fwVersion & 0xF)),
          HAL_GetHalVersion() >> 24,
          (HAL_GetHalVersion() >> 16) & 0xFF,
          (HAL_GetHalVersion() >> 8) & 0xFF,
          HAL_GetHalVersion()      & 0xFF,
          __DATE__, __TIME__);
}

/**
  * @brief  Callback Function for answering to Help command
  * @param  uint8_t *Answer Return String
  * @retval None
  */
void ext_config_help_command_callback(uint8_t *Answer)
{
  /* List of available command */
  sprintf((char *)Answer, "List of available command:\n"
          "1) Board Report\n"
          "- STM32 UID\n"
          "- Version Firmware\n"
          "- Info\n"
          "- Help\n\n"
          "2) Board Settings\n"
          "- Set Name\n");
}

/**
  * @brief  Callback Function for managing the SetName command
  * @param  uint8_t *NewName
  * @retval None
  */
void ext_config_set_name_command_callback(uint8_t *NewName)
{
  MOTENV1_PRINTF("New Board Name = <%s>\r\n", NewName);
  /* Change the Board Name */
  sprintf(ble_stack_value.board_name, "%s", NewName);

  UpdateNodeNameMetaData();

  MOTENV1_PRINTF("\nThe node name has been updated\r\n");
  MOTENV1_PRINTF("Disconnecting and riconnecting to see the new node name\r\n");
}

