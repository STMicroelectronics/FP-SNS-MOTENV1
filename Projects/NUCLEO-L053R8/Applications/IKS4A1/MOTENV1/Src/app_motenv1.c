/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_motenv1.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   This file provides code for FP-SNS-MOTENV1 application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "app_motenv1.h"

#include "main.h"
#include "BLE_Manager.h"
#include "ble_function.h"
#include "hw_advance_features.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private macro ------------------------------------------------------------*/
#define MCR_BLUEMS_F2I_1D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*10);};
#define MCR_BLUEMS_F2I_2D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*100);};
#define MCR_BLUEMS_F2I_3D(in, out_int, out_dec) {out_int = (int32_t)in; out_dec= (int32_t)((in-out_int)*1000);};

/* @brief  Scale factor. It is used to scale acceleration from mg to g */
#define FROM_MG_TO_G    0.001f

/* Private defines -----------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

/* Timer pulse value for channel 1 */
uint32_t uhCCR1_Val;
/* Timer pulse value for channel 3 */
uint32_t uhCCR3_Val;
/* Timer pulse value for channel 4 */
uint32_t uhCCR4_Val;
/* Identifies if the accelerometer interrupt events occurs (1) or not (0) */
volatile uint8_t MEMSInterrupt = 0U;
/* Identifies if the timer for the blinked led  is enabled (1) or disabled (0) */
uint8_t LedTimerEnabled = 0;
/* Identifies if the timer for the environmental features is enabled (1) or disabled (0) */
uint8_t EnvironmentalTimerEnabled = 0;
/* Identifies if the timer for the inertial features is enabled (1) or disabled (0) */
uint8_t InertialTimerEnabled = 0;
/* Identifies if the accelerometer events features are enabled (1) or disabled (0) */
uint8_t AccEventEnabled = 0;
/* Identifies if the led events features are enabled (1) or disabled (0) */
uint8_t LedEnabled = 0;

uint16_t PedometerStepCount = 0;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Imported Variables --------------------------------------------------------*/

/* USER CODE BEGIN IV */

/* USER CODE END IV */

/* Private variables ---------------------------------------------------------*/
/* Identifies if the user button was pressed (1) or not (0) */
static volatile uint32_t ButtonPressed  = 0;
/* Identifies if the BLE connectivity is enable (1) or not (0) for environmental data sensors */
static volatile uint32_t SendEnv        = 0;
/* Identifies if the BLE connectivity is enable (1) or not (0) for inertial data sensors */
static volatile uint32_t SendAccGyroMag = 0;
/* Identifies if the status of the led must be updated (1) or not (0) */
static uint8_t BlinkLed                 = 0;
static uint8_t FirstConnectionConfig    = 1;

static float sensitivity;

/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);

/* Set Default value for Pulse of the used Timer */
static void SetDefault_TIM_Pulse(void);
/* Send Environmetal Data (Temperature/Pressure/Humidity) to BLE */
static void SendEnvironmentalData(void);
/* Send Notification where there is a interrupt from MEMS */
static void MEMSCallback(void);
/* Send Motion Data Acc/Mag/Gyro to BLE */
static void SendMotionData(void);
/* Callback for user button */
static void ButtonCallback(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

void MX_MOTENV1_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MOTENV1_Init_PreTreatment */

  /* USER CODE END MOTENV1_Init_PreTreatment */

  /* Initialize MOTENV1 application */

  User_Init();

  /* USER CODE BEGIN MOTENV1_Init_PostTreatment */

  /* USER CODE END MOTENV1_Init_PostTreatment */
}

/*
 * FP-SNS-MOTENV1 background task
 */
void MX_MOTENV1_Process(void)
{
  /* USER CODE BEGIN MOTENV1_Process_PreTreatment */

  /* USER CODE END MOTENV1_Process_PreTreatment */

  /* Process of the MOTENV1 application */

  User_Process();

  /* USER CODE BEGIN MOTENV1_Process_PostTreatment */

  /* USER CODE END MOTENV1_Process_PostTreatment */
}

/**
  * @brief  Initialize User process.
  *
  * @param  None
  * @retval None
  */
static void User_Init(void)
{
  SetDefault_TIM_Pulse();

  TIM_Init();

  /* Set EXTI settings for Accelerometer Interrupt */
  SetAccIntPin_exti();

  InitTargetPlatform();

  MOTENV1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
                 "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                 " (IAR)\r\n"
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
                 " (KEIL)\r\n"
#elif defined (__GNUC__)
                 " (STM32CubeIDE)\r\n"
#endif /* IDE */
                 "\tSend Every %4d mS Temperature/Humidity/Pressure\r\n"
                 "\tSend Every %4d mS Acc/Gyro/Magneto\r\n\n"
                 ,
                 HAL_GetHalVersion() >> 24,
                 (HAL_GetHalVersion() >> 16) & 0xFF,
                 (HAL_GetHalVersion() >> 8) & 0xFF,
                 HAL_GetHalVersion()      & 0xFF,
                 __DATE__, __TIME__,
                 (1000 / (ALGO_FREQ_ENV)),
                 (1000 / (FREQ_ACC_GYRO_MAG)));

#ifdef MOTENV1_DEBUG_CONNECTION
  MOTENV1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* MOTENV1_DEBUG_CONNECTION */

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
  MOTENV1_PRINTF("Debug Notify Transmission Enabled\r\n\n");
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG stack and services */
  BluetoothInit();

  InitHWFeatures();

  /* Set Accelerometer Full Scale to 2G */
  if (TargetBoardFeatures.IKS01Ax_support)
  {
    Set2GAccelerometerFullScale();
  }

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
}

/**
  * @brief  Configure the device as Client or Server and manage the communication
  *         between a client and a server.
  *
  * @param  None
  * @retval None
  */
static void User_Process(void)
{
  /* Blinking the Led */
  if (!connected)
  {
    if (BlinkLed)
    {
      BSP_LED_Toggle(LED2);
      TargetBoardFeatures.LedStatus = !TargetBoardFeatures.LedStatus;
      BlinkLed = 0;
    }
  }

  if (set_connectable)
  {
    /* Now update the node name */
    if (FirstConnectionConfig)
    {
      sprintf(BLE_StackValue.BoardName, "%s%c%c%c", "ME1V",
              MOTENV1_VERSION_MAJOR,
              MOTENV1_VERSION_MINOR,
              MOTENV1_VERSION_PATCH);

      MOTENV1_PRINTF("\r\nFixed node name = %s\r\n\r\n", BLE_StackValue.BoardName);
      FirstConnectionConfig = 0;
    }

    setConnectable();
    set_connectable = FALSE;
  }

  /* handle BLE event */
  hci_user_evt_proc();

  /* Handle user button */
  if (ButtonPressed)
  {
    ButtonCallback();
    ButtonPressed = 0;
  }

  /* Handle Interrupt from MEMS */
  if (MEMSInterrupt)
  {
    MEMSCallback();
    MEMSInterrupt = 0;
  }

  /* Environmental Data */
  if (SendEnv)
  {
    SendEnv = 0;
    SendEnvironmentalData();
  }

  /* Motion Data */
  if (SendAccGyroMag)
  {
    SendAccGyroMag = 0;
    SendMotionData();
  }

  /* Wait for Event */
  __WFI();
}

/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  MOTION_SENSOR_SET_FULL_SCALE(ACCELERO_INSTANCE, MOTION_ACCELERO, 2.0f);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GET_SENSITIVITY(ACCELERO_INSTANCE, MOTION_ACCELERO, &sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  MOTENV1_PRINTF("UserButton Pressed\r\n");
}

/**
  * @brief  Send Notification where there is a interrupt from MEMS
  * @param  None
  * @retval None
  */
static void MEMSCallback(void)
{
  MOTION_SENSOR_EVENT_STATUS_T status;

  MOTION_SENSOR_GET_EVENT_STATUS(ACCELERO_INSTANCE, &status);

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to Pedometer */
    if (status.StepStatus != 0)
    {
      PedometerStepCount = GetStepHWPedometer();
      if (W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER))
      {
        BLE_AccEnvUpdate(PedometerStepCount, 2);
      }
    }
  }

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to Free Fall */
    if (status.FreeFallStatus != 0)
    {
      BLE_AccEnvUpdate(ACC_FREE_FALL, 2);
    }
  }

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to Single Tap */
    if (status.TapStatus != 0)
    {
      BLE_AccEnvUpdate(ACC_SINGLE_TAP, 2);
    }
  }

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to Double Tap */
    if (status.DoubleTapStatus != 0)
    {
      BLE_AccEnvUpdate(ACC_DOUBLE_TAP, 2);
    }
  }

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to Tilt */
    if (status.TiltStatus != 0)
    {
      BLE_AccEnvUpdate(ACC_TILT, 2);
    }
  }

  if ((W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)))
  {
    /* Check if the interrupt is due to 6D Orientation */
    if (status.D6DOrientationStatus != 0)
    {
      AccEventType Orientation = GetHWOrientation6D();
      BLE_AccEnvUpdate(Orientation, 2);
    }
  }

  if (W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP))
  {
    /* Check if the interrupt is due to Wake Up */
    if (status.WakeUpStatus != 0)
    {
      BLE_AccEnvUpdate(ACC_WAKE_UP, 2);
    }
  }

  if (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS))
  {
    BLE_AccEnvUpdate(PedometerStepCount, 3);
  }
}

/**
  * @brief  Send Motion Data Acc/Mag/Gyro to BLE
  * @param  None
  * @retval None
  */
static void SendMotionData(void)
{
  MOTION_SENSOR_AXES_T ACC_Value;
  MOTION_SENSOR_AXES_T GYR_Value;
  MOTION_SENSOR_AXES_T MAG_Value;

  BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;

  /* Read the Acc values */
  MOTION_SENSOR_GET_AXES(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value);

  /* Read the Magneto values */
  MOTION_SENSOR_GET_AXES(MAGNETO_INSTANCE, MOTION_MAGNETO, &MAG_Value);

  /* Read the Gyro values */
  MOTION_SENSOR_GET_AXES(GYRO_INSTANCE, MOTION_GYRO, &GYR_Value);

  ACC_SensorValue.Axis_x = ACC_Value.x;
  ACC_SensorValue.Axis_y = ACC_Value.y;
  ACC_SensorValue.Axis_z = ACC_Value.z;

  GYR_SensorValue.Axis_x = GYR_Value.x;
  GYR_SensorValue.Axis_y = GYR_Value.y;
  GYR_SensorValue.Axis_z = GYR_Value.z;

  MAG_SensorValue.Axis_x = MAG_Value.x;
  MAG_SensorValue.Axis_y = MAG_Value.y;
  MAG_SensorValue.Axis_z = MAG_Value.z;

  BLE_AccGyroMagUpdate(&ACC_SensorValue, &GYR_SensorValue, &MAG_SensorValue);
}

/**
  * @brief  Read Environmental Data (Temperature/Pressure/Humidity) from sensor
  * @param  int32_t *PressToSend
  * @param  uint16_t *HumToSend
  * @param  int16_t *Temp1ToSend
  * @param  int16_t *Temp2ToSend
  * @retval None
  */
void ReadEnvironmentalData(int32_t *PressToSend, uint16_t *HumToSend, int16_t *Temp1ToSend, int16_t *Temp2ToSend)
{
  float SensorValue;
  int32_t decPart, intPart;

  *PressToSend = 0;
  *HumToSend = 0;
  *Temp2ToSend = 0, *Temp1ToSend = 0;

  /* Read Humidity */
  ENV_SENSOR_GET_VALUE(HUMIDITY_INSTANCE, ENV_HUMIDITY, &SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *HumToSend = intPart * 10 + decPart;

  /* Read Temperature for sensor 1 */
  ENV_SENSOR_GET_VALUE(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE, &SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp1ToSend = intPart * 10 + decPart;

  /* Read Pressure */
  ENV_SENSOR_GET_VALUE(PRESSURE_INSTANCE, ENV_PRESSURE, &SensorValue);
  MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
  *PressToSend = intPart * 100 + decPart;

  /* Read Temperature for sensor 2 */
  ENV_SENSOR_GET_VALUE(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE, &SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp2ToSend = intPart * 10 + decPart;
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  /* Pressure,Humidity, and Temperatures*/
  if (EnvironmentalTimerEnabled)
  {
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t Temp1ToSend;
    int16_t Temp2ToSend;

    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend, &HumToSend, &Temp1ToSend, &Temp2ToSend);

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite, "Sending: ");
      Term_Update(BufferToWrite, BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite, "Press=%ld ", PressToSend);
      Term_Update(BufferToWrite, BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite, "Hum=%d ", HumToSend);
      Term_Update(BufferToWrite, BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite, "Temp=%d ", Temp1ToSend);
      Term_Update(BufferToWrite, BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite, "Temp2=%d ", Temp2ToSend);
      Term_Update(BufferToWrite, BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite, "\r\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }

    MOTENV1_PRINTF("Sending: ");
    MOTENV1_PRINTF("Press=%ld ", PressToSend);
    MOTENV1_PRINTF("Hum=%d ", HumToSend);
    MOTENV1_PRINTF("Temp1=%d ", Temp1ToSend);
    MOTENV1_PRINTF("Temp2=%d ", Temp2ToSend);
    MOTENV1_PRINTF("\r\n");
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */

    BLE_EnvironmentalUpdate(PressToSend, HumToSend, Temp2ToSend, Temp1ToSend);
  }
}

/**
  * @brief  Output Compare callback in non blocking mode
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture = 0;
  /* TIM1_CH1 toggling with frequency = 2 Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));

    if (EnvironmentalTimerEnabled)
    {
      SendEnv = 1;
    }
  }

  /* TIM1_CH3 toggling with frequency = 20 Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));

    if (InertialTimerEnabled)
    {
      SendAccGyroMag = 1;
    }
  }

  /* TIM1_CH4 toggling with frequency = 1 Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));

    if (LedTimerEnabled)
    {
      BlinkLed = 1;
    }
  }
}

/**
  * @brief  BSP Push Button callback
  * @param  Button Specifies the pin connected EXTI line
  * @retval None.
  */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);

  ButtonPressed = 1;
}

/**
  * @brief  Set Default value for Pulse of the used Timer
  * @param  None.
  * @retval None.
  */
static void SetDefault_TIM_Pulse(void)
{
  uhCCR1_Val = (10000 / ALGO_FREQ_ENV);
  uhCCR3_Val = (10000 / FREQ_ACC_GYRO_MAG);
  uhCCR4_Val = (10000 / ALGO_FREQ_LED);
}

/**
  * @brief This function provides accurate delay (in milliseconds) based
  *        on variable incremented.
  * @note This is a user implementation using WFI state
  * @param Delay: specifies the delay time length, in milliseconds.
  * @retval None
  */
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while ((HAL_GetTick() - tickstart) < Delay)
  {
    __WFI();
  }
}

