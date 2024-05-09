/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_motenv1.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   This file provides code for FP-SNS-MOTENV1 application.
  *
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
#define N_BUTTON_PRESS 3
#define CHECK_CALIBRATION ((uint32_t)0x12345678)

/* Meta data dimension (Multiply of 16 bytes)*/
#define META_DATA_DIMENSION 64
/* Meta data memory address for bank 1 */
#define ADDRESS_META_DATA_BANK1 0x081FE000
/* Meta data memory address for bank 2 */
#define ADDRESS_META_DATA_BANK2 0x080FE000
/* Magic number for a valid Fw Id saved on flash */
#define FW_ID_MAGIC_NUM 0xDEADBEEF

/* Exported Variables --------------------------------------------------------*/

/* Timer pulse value for channel 1 */
uint32_t uhCCR1_Val;
/* Timer pulse value for channel 2 */
uint32_t uhCCR2_Val;
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
/* Identifies if Activity Recognition is enabled (1) or disabled (0) */
uint8_t ActivityRecognitionEnabled = 0;
/* Identifies if Carry Position is enabled (1) or disabled (0) */
uint8_t CarryPositionEnabled = 0;
/* Identifies if Gesture Recognition is enabled (1) or disabled (0) */
uint8_t GestureRecognitionEnabled = 0;
/* Identifies if Motion Intensity is enabled (1) or disabled (0) */
uint8_t MotionIntensityEnabled = 0;
/* Identifies if Pedometer Algorithm is enabled (1) or disabled (0) */
uint8_t PedometerAlgorithmEnabled = 0;
/* Identifies if Sensor Fusion is enabled (1) or disabled (0) */
uint8_t SensorFusionEnabled = 0;
/* Identifies if E-Compass is enabled (1) or disabled (0) */
uint8_t ECompassEnabled = 0;
/* Identifies if inertial sensor is calibrated (1) or not (0) */
unsigned char isCal = 0;
/* Identifies if the calibration process will be restarts (1) or not (0) */
uint32_t ForceReCalibration = 0;
/* Count the numbers of step of the pedometer algorithm */
uint16_t PedometerStepCount = 0;
uint32_t CurrentActiveBank = 0;
uint32_t OthersBank = 0;
uint16_t FwId_Bank1;
uint16_t FwId_Bank2;

volatile uint32_t SwapBanks   = 0;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Imported Variables --------------------------------------------------------*/

/* USER CODE BEGIN IV */

/* USER CODE END IV */

/* Local types ---------------------------------------------------------------*/
/* Typedef Meta Data Structure */

typedef struct
{
  uint32_t Calibration[6];
  uint8_t  NodeName[9];
  uint8_t  NodeName_2[9];
  uint32_t FwIdMagicNum_Bank_1;
  uint16_t FwIdBank_1;
  uint32_t FwIdMagicNum_Bank_2;
  uint16_t FwIdBank_2;
  uint8_t  Padding[10]; /* we could write Multiply of 16 bytes at a time... */
} MetaData_t;

/* Private variables ---------------------------------------------------------*/
/* Identifies if the user button was pressed (1) or not (0) */
static volatile uint32_t ButtonPressed  = 0;
/* Identifies if the BLE connectivity is enable (1) or not (0) for environmental data sensors */
static volatile uint32_t SendEnv        = 0;
/* Identifies if the BLE connectivity is enable (1) or not (0) for inertial data sensors */
static volatile uint32_t SendAccGyroMag = 0;
static volatile uint32_t TimeStamp      = 0;

/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for sensors fusion data */
static volatile uint32_t Quaternion      = 0;
/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for activity recognition data */
static volatile uint32_t UpdateMotionAR  = 0;
/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for carry position data */
static volatile uint32_t UpdateMotionCP  = 0;
/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for gesture recognition data */
static volatile uint32_t UpdateMotionGR  = 0;
/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for pedomete algorithm data */
static volatile uint32_t UpdateMotionPM  = 0;
/* Identifies if the BLE connectivity for update the data is enable (1) or not (0) for motion intensity data */
static volatile uint32_t UpdateMotionID  = 0;
/* Identifies if the status of the led must be updated (1) or not (0) */
static uint8_t BlinkLed         = 0;
static uint32_t mag_time_stamp  = 0;

static float sensitivity;

/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

MFX_MagCal_output_t magOffset;
MOTION_SENSOR_AXES_T MAG_Offset;

static MetaData_t FirmwareMetaData;
static uint32_t NecessityToSaveMetaData = 0;

/* Private function prototypes -----------------------------------------------*/
static void User_Init(void);
static void User_Process(void);

/* Set Default value for Pulse of the used Timer */
static void SetDefault_TIM_Pulse(void);
/* Motion Libraries initialization */
static void InitMotionLibraries(void);
/* Test if calibration data are available */
static void MagCalibTest(void);
/* Reset the magneto calibration */
static void ReCalibration(void);
/* Reset the Magnetometer Calibration Values in Memory */
static unsigned char ResetCalibrationInMemory(void);
/* Save the Magnetometer Calibration Values to Memory */
static unsigned char SaveCalibrationToMemory(void);

/* Send Environmetal Data (Temperature/Pressure/Humidity) to BLE */
static void SendEnvironmentalData(void);
/* Send Notification where there is a interrupt from MEMS */
static void MEMSCallback(void);
/* Send Motion Data Acc/Mag/Gyro to BLE */
static void SendMotionData(void);
/* Callback for user button */
static void ButtonCallback(void);
/* MotionFX Working function */
static void ComputeQuaternions(void);
/* MotionAR Working function */
static void ComputeMotionAR(void);
/* MotionCP Working function */
static void ComputeMotionCP(void);
/* MotionGR Working function */
static void ComputeMotionGR(void);
/* MotionPM Working function */
static void ComputeMotionPM(void);
/* MotionID Working function */
static void ComputeMotionID(void);
/* Enable Disable the jump to second flash bank and reboot board */
static void EnableDisableDualBoot(void);
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

  Led_TIM_Init();
  Env_TIM_Init();
  MotionAlgo_TIM_Init();

  /* Set EXTI settings for Accelerometer Interrupt */
  SetAccIntPin_exti();

  InitTargetPlatform();

  CurrentActiveBank = 0;

  /* Check if we are running from Bank1 or Bank2 */
  {
    FLASH_OBProgramInitTypeDef    OBInit;
    /* Allow Access to Flash control registers and user Flash */
    HAL_FLASH_Unlock();
    /* Allow Access to option bytes sector */
    HAL_FLASH_OB_Unlock();
    /* Get the Dual boot configuration status */
    HAL_FLASHEx_OBGetConfig(&OBInit);
    if (((OBInit.USERConfig) & (OB_SWAP_BANK_ENABLE)) == OB_SWAP_BANK_ENABLE)
    {
      CurrentActiveBank = 2;
      OthersBank = 1;
    }
    else
    {
      CurrentActiveBank = 1;
      OthersBank = 2;
    }
    HAL_FLASH_OB_Lock();
    HAL_FLASH_Lock();
  }

  /* Check the Meta Data */
  if (CurrentActiveBank == 1)
  {
    memcpy((void *)&FirmwareMetaData, (void *)ADDRESS_META_DATA_BANK1, sizeof(MetaData_t));
  }

  if (CurrentActiveBank == 2)
  {
    memcpy((void *)&FirmwareMetaData, (void *)ADDRESS_META_DATA_BANK2, sizeof(MetaData_t));
  }

  MOTENV1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
                 "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
                 " (IAR)\r\n"
#elif defined (__CC_ARM) || (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)) /* For ARM Compiler 5 and 6 */
                 " (KEIL)\r\n"
#elif defined (__GNUC__)
                 " (STM32CubeIDE)\r\n"
#endif /* IDE */
                 "\tSend Every %4d mS %d Short precision Quaternions\r\n"
                 "\tSend Every %4d mS Temperature/Humidity/Pressure\r\n"
                 "\tSend Every %4d mS Acc/Gyro/Magneto\r\n\n"
                 ,
                 HAL_GetHalVersion() >> 24,
                 (HAL_GetHalVersion() >> 16) & 0xFF,
                 (HAL_GetHalVersion() >> 8) & 0xFF,
                 HAL_GetHalVersion()      & 0xFF,
                 __DATE__, __TIME__,
                 QUAT_UPDATE_MUL_10MS * 10, SEND_N_QUATERNIONS,
                 ALGO_PERIOD_ENV,
                 (1000 / FREQ_ACC_GYRO_MAG));

#ifdef MOTENV1_DEBUG_CONNECTION
  MOTENV1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* MOTENV1_DEBUG_CONNECTION */

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
  MOTENV1_PRINTF("Debug Notify Transmission Enabled\r\n\n");
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG stack and services */
  BluetoothInit();

  InitHWFeatures();

  /* Check the BootLoader Compliance */

  /* Set Accelerometer Full Scale to 2G */
  if (TargetBoardFeatures.IKS01Ax_support)
  {
    Set2GAccelerometerFullScale();
  }

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
    if (TargetBoardFeatures.IKS01Ax_support)
    {
      InitMotionLibraries();
    } /*  TargetBoardFeatures.IKS01Ax_support  */

    if (NecessityToSaveMetaData)
    {
      uint32_t Success = EraseMetaData();
      MOTENV1_PRINTF("Erase Meta Data\r\n");
      if (Success)
      {
        SaveMetaData();
        MOTENV1_PRINTF("Save Meta Data\r\n");
      }

      NecessityToSaveMetaData = 0;
    }

    /* Now update the BLE advertize data and make the Board connectable */
    setConnectable();
    set_connectable = FALSE;
  }

  /* handle BLE event */
  hci_user_evt_proc();

  /* Swap the Flash Banks */
  if (SwapBanks)
  {
    EnableDisableDualBoot();
    SwapBanks = 0;
  }

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

  /* Handle Re-Calibration */
  if (ForceReCalibration)
  {
    ForceReCalibration = 0;
    ReCalibration();
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

  /* MotionFX */
  if (Quaternion)
  {
    Quaternion = 0;
    ComputeQuaternions();
  }

  /* MotionAR */
  if (UpdateMotionAR)
  {
    UpdateMotionAR = 0;
    ComputeMotionAR();
  }

  /* MotionCP */
  if (UpdateMotionCP)
  {
    UpdateMotionCP = 0;
    ComputeMotionCP();
  }

  /* MotionGR */
  if (UpdateMotionGR)
  {
    UpdateMotionGR = 0;
    ComputeMotionGR();
  }

  /* MotionPM */
  if (UpdateMotionPM)
  {
    UpdateMotionPM = 0;
    ComputeMotionPM();
  }

  /* MotionID */
  if (UpdateMotionID)
  {
    UpdateMotionID = 0;
    ComputeMotionID();
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
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
void Set4GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-4g */
  MOTION_SENSOR_SET_FULL_SCALE(ACCELERO_INSTANCE, MOTION_ACCELERO, 4.0f);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GET_SENSITIVITY(ACCELERO_INSTANCE, MOTION_ACCELERO, &sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  Motion Libraries initialization
  * @param  None
  * @retval None
  */
static void InitMotionLibraries(void)
{
  /* Code for MotionFX integration - Start Section */
  /* Initialize MotionFX Library */
  if (TargetBoardFeatures.MotionFXIsInitalized == 0)
  {
    MotionFX_manager_init();
    MotionFX_manager_start_9X();
    /* Enable magnetometer calibration */
    MagCalibTest();
  }
  /* Code for MotionFX integration - End Section */

  /* Code for MotionAR integration - Start Section */
  /* Initialize MotionAR Library */
  if (TargetBoardFeatures.MotionARIsInitalized == 0)
  {
    MotionAR_manager_init();
  }
  /* Code for MotionAR integration - End Section */

  /* Code for MotionCP integration - Start Section */
  /* Initialize MotionCP Library */
  if (TargetBoardFeatures.MotionCPIsInitalized == 0)
  {
    MotionCP_manager_init();
  }
  /* Code for MotionCP integration - End Section */

  /* Code for MotionGR integration - Start Section */
  /* Initialize MotionGR Library */
  if (TargetBoardFeatures.MotionGRIsInitalized == 0)
  {
    MotionGR_manager_init();
  }
  /* Code for MotionGR integration - End Section */

  /* Code for MotionPM integration - Start Section */
  /* Initialize MotionPM Library */
  if (TargetBoardFeatures.MotionPMIsInitalized == 0)
  {
    MotionPM_manager_init();
  }
  /* Code for MotionPM integration - End Section */

  /* Initialize MotionID Library */
  if (TargetBoardFeatures.MotionIDIsInitalized == 0)
  {
    MotionID_manager_init();
  }
}

/**
  * @brief  Callback for user button
  * @param  None
  * @retval None
  */
static void ButtonCallback(void)
{
  /* Only if connected */
  if (connected)
  {
    static uint32_t HowManyButtonPress = 0;
    static uint32_t tickstart = 0;
    uint32_t tickstop;

    if (!tickstart)
    {
      tickstart = HAL_GetTick();
    }

    tickstop = HAL_GetTick();

    if ((tickstop - tickstart) > 2000)
    {
      HowManyButtonPress = 0;
      tickstart = tickstop;
    }

    if (TargetBoardFeatures.MotionFXIsInitalized)
    {
      if ((HowManyButtonPress + 1) == N_BUTTON_PRESS)
      {
        ForceReCalibration = 1;
        HowManyButtonPress = 0;
      }
      else
      {
        HowManyButtonPress++;
        if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
        {
          BytesToWrite = sprintf((char *)BufferToWrite,
                                 "%ld in %ldmS Reset Calib\r\n",
                                 3 - HowManyButtonPress,
                                 2000 - (tickstop - tickstart));
          Term_Update(BufferToWrite, BytesToWrite);
        }
        else
        {
          MOTENV1_PRINTF("%ld in %ldmS Reset Calib\r\n", 3 - HowManyButtonPress, 2000 - (tickstop - tickstart));
        }
      }
    }
    else
    {
      MOTENV1_PRINTF("UserButton Pressed\r\n");
    }
  }

}

/**
  * @brief  Reset the magneto calibration
  * @param  None
  * @retval None
  */
static void ReCalibration(void)
{
  /* Only if connected */
  if (connected)
  {
    /* Reset the Compass Calibration */
    isCal = 0;
    MFX_MagCal_output_t mag_cal_test;

    /* Notifications of Compass Calibration */
    Config_Update(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);
    Config_Update(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);

    /* Reset the Calibration */
    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite, "\r\nForce ReCalibration\r\n");
      Term_Update(BufferToWrite, BytesToWrite);
    }

    MOTENV1_PRINTF("\r\nForce ReCalibration\r\n");

    /* Enable magnetometer calibration */
    MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
    MotionFX_MagCal_getParams(&mag_cal_test);

    /* Switch off the LED */
    LedOffTargetPlatform();
  }
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
  * @brief  MotionCP Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionCP(void)
{
  static MCP_output_t CarryPositionCodeStored = MCP_UNKNOWN;
  BLE_CP_output_t CarryPositionCodeSent;
  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES_RAW(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value_Raw);
  MotionCP_manager_run(ACC_Value_Raw);

  if (CarryPositionCodeStored != CarryPositionCode)
  {
    CarryPositionCodeStored = CarryPositionCode;

    CarryPositionCodeSent = (BLE_CP_output_t)CarryPositionCode;
    BLE_CarryPositionUpdate(CarryPositionCodeSent);

    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite, "Sending: Carry Position Code= %d\r\n", CarryPositionCode);
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      MOTENV1_PRINTF("Sending: Carry Position Code= %d\r\n", CarryPositionCode);
    }
  }
}

/**
  * @brief  MotionGR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionGR(void)
{
  static MGR_output_t GestureRecognitionCodeStored = MGR_NOGESTURE;
  BLE_GR_output_t GestureRecognitionCodeSent;
  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES_RAW(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value_Raw);
  MotionGR_manager_run(ACC_Value_Raw);

  if (GestureRecognitionCodeStored != GestureRecognitionCode)
  {
    GestureRecognitionCodeStored = GestureRecognitionCode;
    GestureRecognitionCodeSent = (BLE_GR_output_t)GestureRecognitionCode;
    BLE_GestureRecognitionUpdate(GestureRecognitionCodeSent);

    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite,
                             "Sending: Gesture Recognition Code= %d\r\n",
                             GestureRecognitionCode);
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      MOTENV1_PRINTF("Sending: Gesture Recognition Code= %d\r\n", GestureRecognitionCode);
    }
  }
}

/**
  * @brief  MotionPM Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionPM(void)
{
  static MPM_output_t PM_DataOUTStored;
  BLE_PM_output_t PM_DataOUT_Sent;
  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES_RAW(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value_Raw);
  MotionPM_manager_run(ACC_Value_Raw);

  if ((PM_DataOUTStored.Nsteps != PM_DataOUT.Nsteps) | (PM_DataOUTStored.Cadence != PM_DataOUT.Cadence))
  {
    PM_DataOUTStored = PM_DataOUT;
    PM_DataOUT_Sent.Cadence = PM_DataOUT.Cadence;
    PM_DataOUT_Sent.Nsteps = PM_DataOUT.Nsteps;
    BLE_PedometerAlgorithmUpdate(&PM_DataOUT_Sent);
  }
}

/**
  * @brief  MotionAR Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionAR(void)
{
  static MAR_output_t ActivityCodeStored = MAR_NOACTIVITY;
  BLE_AR_output_t ActivityCodeSent;
  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES_RAW(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value_Raw);

  MotionAR_manager_run(ACC_Value_Raw, TimeStamp);

  if (ActivityCodeStored != ActivityCode)
  {
    ActivityCodeStored = ActivityCode;

    ActivityCodeSent = (BLE_AR_output_t)ActivityCode;

    BLE_ActRecUpdate(ActivityCodeSent, HAR_ALGO_IDX_NONE);

    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite, "Sending: Activity Recognition Code= %d\r\n", ActivityCode);
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      MOTENV1_PRINTF("Sending: Activity Recognition Code= %d\r\n", ActivityCode);
    }
  }
}

/**
  * @brief  MotionID Working function
  * @param  None
  * @retval None
  */
static void ComputeMotionID(void)
{
  static MID_output_t MIDStored = MID_ON_DESK; /* on desk */
  BLE_ID_output_t MIDCodeSent;
  MOTION_SENSOR_AXES_RAW_T ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES_RAW(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value_Raw);

  MotionID_manager_run(ACC_Value_Raw);

  if (MIDStored != MIDCode)
  {
    MIDStored = MIDCode;

    MIDCodeSent = (BLE_ID_output_t)MIDCode;

    BLE_MotionIntensityUpdate(MIDCodeSent);

    if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
    {
      BytesToWrite = sprintf((char *)BufferToWrite, "Sending: Motion Intensity Code= %d\r\n", MIDCode);
      Term_Update(BufferToWrite, BytesToWrite);
    }
    else
    {
      MOTENV1_PRINTF("Sending: Motion Intensity Code= %d\r\n", MIDCode);
    }
  }
}

/** @brief  MotionFX Working function
  * @param  None
  * @retval None
  */
static void ComputeQuaternions(void)
{
  static MOTION_SENSOR_AXES_T quat_axes[SEND_N_QUATERNIONS];
  static BLE_MOTION_SENSOR_Axes_t quat_axes_send[SEND_N_QUATERNIONS];
  static int32_t calibIndex = 0;
  static int32_t CounterFX  = 0;
  static int32_t CounterEC  = 0;

  MOTION_SENSOR_AXES_T ACC_Value;
  MOTION_SENSOR_AXES_T GYR_Value;
  MOTION_SENSOR_AXES_T MAG_Value;

  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;

  MFX_MagCal_input_t mag_data_in;

  /* Increment the Counter */
  if (ECompassEnabled)
  {
    CounterEC++;
  }
  else
  {
    CounterFX++;
  }

  /* Read the Acc RAW values */
  MOTION_SENSOR_GET_AXES(ACCELERO_INSTANCE, MOTION_ACCELERO, &ACC_Value);
  /* Convert acceleration from [mg] to [g] */
  data_in.acc[0] = (float)ACC_Value.x * FROM_MG_TO_G;
  data_in.acc[1] = (float)ACC_Value.y * FROM_MG_TO_G;
  data_in.acc[2] = (float)ACC_Value.z * FROM_MG_TO_G;

  /* Read the Magneto values */
  MOTION_SENSOR_GET_AXES(MAGNETO_INSTANCE, MOTION_MAGNETO, &MAG_Value);
  /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
  data_in.mag[0] = (float)(MAG_Value.x - MAG_Offset.x) * FROM_MGAUSS_TO_UT50;
  data_in.mag[1] = (float)(MAG_Value.y - MAG_Offset.y) * FROM_MGAUSS_TO_UT50;
  data_in.mag[2] = (float)(MAG_Value.z - MAG_Offset.z) * FROM_MGAUSS_TO_UT50;

  /* Read the Gyro values */
  MOTION_SENSOR_GET_AXES(GYRO_INSTANCE, MOTION_GYRO, &GYR_Value);
  /* Convert angular velocity from [mdps] to [dps] */
  data_in.gyro[0] = (float)GYR_Value.x * FROM_MDPS_TO_DPS;
  data_in.gyro[1] = (float)GYR_Value.y * FROM_MDPS_TO_DPS;
  data_in.gyro[2] = (float)GYR_Value.z * FROM_MDPS_TO_DPS;

  /* Check if is calibrated */
  if (isCal != 0x01)
  {
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4)
    {
      calibIndex = 0;
      mag_data_in.mag[0] = (float)MAG_Value.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1] = (float)MAG_Value.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2] = (float)MAG_Value.z * FROM_MGAUSS_TO_UT50;

      mag_data_in.time_stamp = (int)mag_time_stamp;
      mag_time_stamp += SAMPLE_PERIOD;

      MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);

      /* Control the calibration status */
      if ((magOffset.cal_quality == MFX_MAGCALOK) ||
          (magOffset.cal_quality == MFX_MAGCALGOOD))
      {
        isCal = 1;

        MAG_Offset.x = (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.y = (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.z = (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);

        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);

        SaveCalibrationToMemory();
      }

      if (isCal == 0x01)
      {
        if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
        {
          BytesToWrite = sprintf((char *)BufferToWrite, "\r\nCompass Calibrated\r\n");
          Term_Update(BufferToWrite, BytesToWrite);
        }

        MOTENV1_PRINTF("\r\nCompass Calibrated\r\n");

        /* Switch on the Led */
        LedOnTargetPlatform();
        if (LedEnabled)
        {
          BLE_LedStatusUpdate(TargetBoardFeatures.LedStatus);
        }

        /* Notifications of Compass Calibration */
        Config_Update(FEATURE_MASK_SENSORFUSION_SHORT, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);
        Config_Update(FEATURE_MASK_ECOMPASS, W2ST_COMMAND_CAL_STATUS, isCal ? 100 : 0);
      }
    }
  }
  else
  {
    calibIndex = 0;
  }

  MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);

  if (ECompassEnabled)
  {
    /* E-Compass Updated every 0.1 Seconds*/
    if (CounterEC == 10)
    {
      uint16_t Angle = (uint16_t)trunc(100 * pdata_out->heading);
      CounterEC = 0;
      BLE_ECompassUpdate(Angle);
    }
  }
  else
  {
    int32_t QuaternionNumber = (CounterFX > SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS - 1) : (CounterFX - 1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if (pdata_out->quaternion[3] < 0)
    {
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * (-10000));
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * (-10000));
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * (-10000));
    }
    else
    {
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * 10000);
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * 10000);
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * 10000);
    }

    quat_axes_send[QuaternionNumber].Axis_x = quat_axes[QuaternionNumber].x;
    quat_axes_send[QuaternionNumber].Axis_y = quat_axes[QuaternionNumber].y;
    quat_axes_send[QuaternionNumber].Axis_z = quat_axes[QuaternionNumber].z;

    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions information via bluetooth */
    if (CounterFX == QUAT_UPDATE_MUL_10MS)
    {
      BLE_SensorFusionUpdate(quat_axes_send, SEND_N_QUATERNIONS);
      CounterFX = 0;
    }
  }
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
  int32_t decPart;
  int32_t intPart;

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
  /* Notifications of Compass Calibration status*/
  if (FirstConnectionConfig)
  {
    FirstConnectionConfig = 0;

    /* Switch on/off the LED according to calibration */
    if (isCal)
    {
      LedOnTargetPlatform();
    }
    else
    {
      LedOffTargetPlatform();
    }
  }

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
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));

    if ((SensorFusionEnabled) || (ECompassEnabled))
    {
      Quaternion = 1;
    }
  }

  /* TIM1_CH2 toggling with frequency = 50Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));

    if (CarryPositionEnabled)
    {
      UpdateMotionCP = 1;
    }
    else if (PedometerAlgorithmEnabled)
    {
      UpdateMotionPM = 1;
    }
    else if (GestureRecognitionEnabled)
    {
      UpdateMotionGR = 1;
    }
  }

  /* TIM1_CH3 toggling with frequency = 16Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_3, (uhCapture + uhCCR3_Val));

    if (MotionIntensityEnabled)
    {
      UpdateMotionID = 1;
    }
    else if (ActivityRecognitionEnabled)
    {
      UpdateMotionAR = 1;
      TimeStamp += (1000 / ALGO_FREQ_AR_ID);
    }
  }

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TIM_CC_HANDLE, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));

    if (InertialTimerEnabled)
    {
      SendAccGyroMag = 1;
    }
  }
}

/**
  * @brief  Period elapsed callback in non blocking mode for Environmental timer
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&TIM_ENV_HANDLE))
  {
    /* Environmental */
    if (EnvironmentalTimerEnabled)
    {
      SendEnv = 1;
    }
  }
  else if (htim == (&TIM_LED_HANDLE))
  {
    /* Led */
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
  uhCCR1_Val = (10000 / ALGO_FREQ_FX);
  uhCCR2_Val = (10000 / ALGO_FREQ_CP_GR_PM);
  uhCCR3_Val = (10000 / ALGO_FREQ_AR_ID);
  uhCCR4_Val = (10000 / FREQ_ACC_GYRO_MAG);
}

/**
  * @brief  Test if calibration data are available
  * @param  None
  * @retval None
  */
static void MagCalibTest(void)
{
  if (FirmwareMetaData.Calibration[0] == CHECK_CALIBRATION)
  {
    if (FirmwareMetaData.Calibration[1] == TargetBoardFeatures.mems_expansion_board)
    {
      if ((FirmwareMetaData.Calibration[2] == MFX_MAGCALOK) ||
          (FirmwareMetaData.Calibration[2] == MFX_MAGCALGOOD))
      {
        MAG_Offset.x = FirmwareMetaData.Calibration[3];
        MAG_Offset.y = FirmwareMetaData.Calibration[4];
        MAG_Offset.z = FirmwareMetaData.Calibration[5];

        isCal = 1;

        MOTENV1_PRINTF("Magneto Calibration Read\r\n");
      }
      else
      {
        isCal = 0;
        MOTENV1_PRINTF("Magneto Calibration quality is not good\r\n");
      }
    }
    else
    {
      MOTENV1_PRINTF("Magneto Calibration Not correct for Current %s board\r\n",
                     TargetBoardFeatures.mems_expansion_board ? "IKS01A3" : "IKS4A1");
      ResetCalibrationInMemory();
      isCal = 0;
    }
  }
  else
  {
    MOTENV1_PRINTF("Magneto Calibration Not present\r\n");
    isCal = 0;
  }

  if (!isCal)
  {
    MAG_Offset.x = 0;
    MAG_Offset.y = 0;
    MAG_Offset.z = 0;

    /* Enable magnetometer calibration */
    MFX_MagCal_output_t mag_cal_test;
    MotionFX_manager_MagCal_start(SAMPLE_PERIOD);
    MotionFX_MagCal_getParams(&mag_cal_test);
    MOTENV1_PRINTF("After connection magneto calibration starts when sensor fusion feature is used\n\r");
  }
}

/**
  * @brief  Save the Magnetometer Calibration Values to Memory
  * @param uint32_t *MagnetoCalibration the Magneto Calibration
  * @retval unsigned char Success/Not Success
  */
static unsigned char SaveCalibrationToMemory(void)
{
  unsigned char Success = 1;

  /* Store in RAM */
  FirmwareMetaData.Calibration[0] = CHECK_CALIBRATION;
  FirmwareMetaData.Calibration[1] = TargetBoardFeatures.mems_expansion_board;
  FirmwareMetaData.Calibration[2] = magOffset.cal_quality;
  FirmwareMetaData.Calibration[3] = MAG_Offset.x;
  FirmwareMetaData.Calibration[4] = MAG_Offset.y;
  FirmwareMetaData.Calibration[5] = MAG_Offset.z;

  if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
  {
    BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be saved in FLASH\r\n");
    Term_Update(BufferToWrite, BytesToWrite);
  }
  else
  {
    MOTENV1_PRINTF("Magneto Calibration will be saved in FLASH\r\n");
  }

  NecessityToSaveMetaData = 1;

  return Success;
}

/**
  * @brief  Reset the Magnetometer Calibration Values in Memory
  * @param uint32_t *MagnetoCalibration the Magneto Calibration
  * @retval unsigned char Success/Not Success
  */
static unsigned char ResetCalibrationInMemory(void)
{
  /* Reset Calibration Values in RAM */
  unsigned char Success = 1;
  int32_t Counter;

  for (Counter = 0; Counter < 6; Counter++)
  {
    FirmwareMetaData.Calibration[Counter] = 0x0;
  }

  if (BLE_StdTerm_Service == BLE_SERV_ENABLE)
  {
    BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be eresed in FLASH\r\n");
    Term_Update(BufferToWrite, BytesToWrite);
  }
  else
  {
    MOTENV1_PRINTF("Magneto Calibration will be eresed in FLASH\r\n");
  }

  NecessityToSaveMetaData = 1;
  return Success;
}

/**
  * @brief  Enable Disable the jump to second flash bank and reboot board
  * @param  None
  * @retval None
  */
static void EnableDisableDualBoot(void)
{
  FLASH_OBProgramInitTypeDef    OBInit;
  /* Set BFB2 bit to enable boot from Flash Bank2 */
  /* Allow Access to Flash control registers and user Flash */
  HAL_FLASH_Unlock();

  /* Allow Access to option bytes sector */
  HAL_FLASH_OB_Unlock();

  /* Get the Dual boot configuration status */
  HAL_FLASHEx_OBGetConfig(&OBInit);

  /* Enable/Disable dual boot feature */
  OBInit.OptionType = OPTIONBYTE_USER;
  OBInit.USERType   = OB_USER_SWAP_BANK;

  if (((OBInit.USERConfig) & (FLASH_OPTR_SWAP_BANK)) == FLASH_OPTR_SWAP_BANK)
  {
    OBInit.USERConfig &= ~FLASH_OPTR_SWAP_BANK;
    MOTENV1_PRINTF("->Disable DualBoot\r\n");
  }
  else
  {
    OBInit.USERConfig = FLASH_OPTR_SWAP_BANK;
    MOTENV1_PRINTF("->Enable DualBoot\r\n");
  }

  if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
  {
    /*
    Error occurred while setting option bytes configuration.
    User can add here some code to deal with this error.
    To know the code error, user can call function 'HAL_FLASH_GetError()'
    */
    Error_Handler();
  }

  /* Start the Option Bytes programming process */
  if (HAL_FLASH_OB_Launch() != HAL_OK)
  {
    /*
    Error occurred while reloading option bytes configuration.
    User can add here some code to deal with this error.
    To know the code error, user can call function 'HAL_FLASH_GetError()'
    */
    Error_Handler();
  }
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
}

/**
  * @brief  Update Node Name on to Meta Data
  * @param  None
  * @retval None
  */
void UpdateNodeNameMetaData(void)
{

  for (uint32_t i = 0; i < 7; i++)
  {
    if (CurrentActiveBank == 1)
    {
      FirmwareMetaData.NodeName[i + 1] = BLE_StackValue.BoardName[i];
    }

    if (CurrentActiveBank == 2)
    {
      FirmwareMetaData.NodeName_2[i + 1] = BLE_StackValue.BoardName[i];
    }
  }

  /* Save the node name in the meta data */
  NecessityToSaveMetaData = 1;
}

/**
  * @brief  Check if there is a valid Node Name Values in Memory and read them
  * @param  None
  * @retval unsigned char Success/Not Success
  */
unsigned char ReCallNodeNameFromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success = 0;

  if (FirmwareMetaData.NodeName[0] != 0x12)
  {
    FirmwareMetaData.NodeName[0] = 0x12;

    for (uint32_t i = 0; i < 7; i++)
    {
      FirmwareMetaData.NodeName[i + 1] = BLE_StackValue.BoardName[i];
    }

    FirmwareMetaData.NodeName[8] = '\0';

    NecessityToSaveMetaData = 1;

    MOTENV1_PRINTF("\r\nNode name not present in FLASH\r\n");
    MOTENV1_PRINTF("\tNode name written to FLASH= %s\r\n", BLE_StackValue.BoardName);
  }
  else
  {
    for (uint32_t i = 0; i < 7; i++)
    {
      BLE_StackValue.BoardName[i] = FirmwareMetaData.NodeName[i + 1];
    }

    MOTENV1_PRINTF("\r\nNode name read from FLASH (%s)\r\n", BLE_StackValue.BoardName);
  }

  return Success;
}

/**
  * @brief  Check if there is a valid Node Name Values in Memory for bank 2 and read them
  * @param  None
  * @retval unsigned char Success/Not Success
  */
unsigned char ReCallNodeName2FromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success = 0;

  if (FirmwareMetaData.NodeName_2[0] != 0x12)
  {
    FirmwareMetaData.NodeName_2[0] = 0x12;

    for (uint32_t i = 0; i < 7; i++)
    {
      FirmwareMetaData.NodeName_2[i + 1] = BLE_StackValue.BoardName[i];
    }

    FirmwareMetaData.NodeName_2[8] = '\0';

    NecessityToSaveMetaData = 1;

    MOTENV1_PRINTF("\r\nNode name not present in FLASH\r\n");
    MOTENV1_PRINTF("\tNode name written to FLASH= %s\r\n", BLE_StackValue.BoardName);
  }
  else
  {
    for (uint32_t i = 0; i < 7; i++)
    {
      BLE_StackValue.BoardName[i] = FirmwareMetaData.NodeName_2[i + 1];
    }

    MOTENV1_PRINTF("\r\nNode name read from FLASH (%s)\r\n", BLE_StackValue.BoardName);
  }

  return Success;
}

/**
  * @brief  Check if there is a valid firmware Id Values in Memory for bank 1 and read them
  * @param  None
  * @retval unsigned char Success/Not Success
  */
unsigned char ReCallBank1FwIdFromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success = 0;

  if (FirmwareMetaData.FwIdMagicNum_Bank_1 != FW_ID_MAGIC_NUM)
  {
    FirmwareMetaData.FwIdMagicNum_Bank_1 = FW_ID_MAGIC_NUM;

    MOTENV1_PRINTF("\r\nBank 1 FW ID not present in FLASH");

    if (CurrentActiveBank == 1)
    {
      FirmwareMetaData.FwIdBank_1 = CUSTOM_FIRMWARE_ID;
      MOTENV1_PRINTF("\r\nBank 1 FW ID written to FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_1);
    }

    if (CurrentActiveBank == 2)
    {
      FirmwareMetaData.FwIdBank_1 = FW_ID_NOT_VALID;
      MOTENV1_PRINTF("\r\nBank 1 not valid FW ID written to FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_1);
    }

    NecessityToSaveMetaData = 1;
  }
  else
  {
    if (CurrentActiveBank == 1)
    {
      if (FirmwareMetaData.FwIdBank_1 != CUSTOM_FIRMWARE_ID)
      {
        FirmwareMetaData.FwIdBank_1 = CUSTOM_FIRMWARE_ID;
        NecessityToSaveMetaData = 1;
        MOTENV1_PRINTF("\r\nBank 1 FW ID updated and written to FLASH= 0x%x\r\n\r\n", FirmwareMetaData.FwIdBank_1);
      }
      else
      {
        MOTENV1_PRINTF("\r\nBank 1 FW ID read from FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_1);
      }
    }

    if (CurrentActiveBank == 2)
    {
      MOTENV1_PRINTF("\r\nBank 1 FW ID read from FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_1);
    }
  }

  FwId_Bank1 = FirmwareMetaData.FwIdBank_1;

  return Success;
}

/**
  * @brief  Check if there is a valid firmware Id Values in Memory for bank 2 and read them
  * @param  None
  * @retval unsigned char Success/Not Success
  */
unsigned char ReCallBank2FwIdFromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success = 0;

  if (FirmwareMetaData.FwIdMagicNum_Bank_2 != FW_ID_MAGIC_NUM)
  {
    FirmwareMetaData.FwIdMagicNum_Bank_2 = FW_ID_MAGIC_NUM;

    MOTENV1_PRINTF("\r\nBank 2 FW ID not present in FLASH");

    if (CurrentActiveBank == 1)
    {
      FirmwareMetaData.FwIdBank_2 = FW_ID_NOT_VALID;
      MOTENV1_PRINTF("\r\nBank 2 not valid FW ID written to FLASH= 0x%x\r\n\r\n", FirmwareMetaData.FwIdBank_2);
    }

    if (CurrentActiveBank == 2)
    {
      FirmwareMetaData.FwIdBank_2 = 0xFF;
      MOTENV1_PRINTF("\r\nBank 2 FW ID written to FLASH= 0x%x\r\n\r\n", FirmwareMetaData.FwIdBank_2);
    }

    NecessityToSaveMetaData = 1;
  }
  else
  {
    if (CurrentActiveBank == 2)
    {
      if (FirmwareMetaData.FwIdBank_2 != 0xFF)
      {
        FirmwareMetaData.FwIdBank_2 = 0xFF;
        NecessityToSaveMetaData = 1;
        MOTENV1_PRINTF("\r\nBank 2 FW ID updated and written to FLASH= 0x%x\r\n\r\n", FirmwareMetaData.FwIdBank_2);
      }
      else
      {
        MOTENV1_PRINTF("\r\nBank 2 FW ID read from FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_2);
      }
    }

    if (CurrentActiveBank == 1)
    {
      MOTENV1_PRINTF("\r\nBank 2 FW ID read from FLASH= 0x%x\r\n", FirmwareMetaData.FwIdBank_2);
    }
  }

  FwId_Bank2 = FirmwareMetaData.FwIdBank_2;

  return Success;
}

/**
  * @brief User function for Erasing the Flash data
  * @param None
  * @retval uint32_t Success/NotSuccess [1/0]
  */
uint32_t EraseMetaData(void)
{
  /* We need to change both the Banks' info */
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  uint32_t Success = 1;

  /* Disable instruction cache prior to internal cacheable memory update */
  if (HAL_ICACHE_Disable() != HAL_OK)
  {
    Error_Handler();
  }

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  EraseInitStruct.Page        = FLASH_PAGE_NB - 1;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.NbPages     = 1;

  MOTENV1_PRINTF("Start FLASH Erase Bank %ld\r\n", CurrentActiveBank);

  EraseInitStruct.Banks  = FLASH_BANK_2;

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /* Error occurred while sector erase.
    User can add here some code to deal with this error.
    SectorError will contain the faulty sector and then to know the code error on this sector,
    user can call function 'HAL_FLASH_GetError()'
    FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    Success = 0;
    MOTENV1_PRINTF("Error occurred while page erase\r\n");
  }
  else
  {
    MOTENV1_PRINTF("End FLASH Erase %lu Pages of %dBytes\r\n", EraseInitStruct.NbPages, FLASH_PAGE_SIZE);
  }

  return Success;
}

/**
  * @brief User function for Saving the MDM  on the Flash
  * @param None
  * @retval None
  */
void SaveMetaData()
{
  /* Store in Flash Memory */
  uint32_t Address;
  uint32_t BankInfoAddress;
  uint32_t Count;

  if (CurrentActiveBank == 1)
  {
    Address = ADDRESS_META_DATA_BANK1;
  }

  if (CurrentActiveBank == 2)
  {
    Address = ADDRESS_META_DATA_BANK2;
  }

  BankInfoAddress = (uint32_t)&FirmwareMetaData;

  /* Disable instruction cache prior to internal cacheable memory update */
  if (HAL_ICACHE_Disable() != HAL_OK)
  {
    Error_Handler();
  }

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  for (Count = 0; Count < META_DATA_DIMENSION; Count += 16)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, Address, BankInfoAddress) != HAL_OK)
    {
      MOTENV1_PRINTF("Error HAL_FLASH_Program\r\n");
    }

    Address += 16;
    BankInfoAddress += 16;
  }

  /* Lock the Flash to disable the flash control register access (recommended
   to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Re-enable instruction cache */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
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

