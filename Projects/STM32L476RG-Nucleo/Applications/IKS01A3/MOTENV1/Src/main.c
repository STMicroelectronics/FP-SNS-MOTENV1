/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "MetaDataManager.h"
#include "OTA.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BLUEMSYS_N_BUTTON_PRESS 3
#define BLUEMSYS_CHECK_CALIBRATION ((uint32_t)0x12345678)

/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

MFX_MagCal_output_t magOffset;
MOTION_SENSOR_Axes_t MAG_Offset;

uint32_t ForceReCalibration    =0;

volatile uint32_t HCI_ProcessEvent=0;

uint8_t bdaddr[6];

uint8_t EnvironmentalTimerEnabled= 0;
uint8_t InertialTimerEnabled= 0;
uint8_t AccEventEnabled= 0;
uint8_t LedEnabled= 0;

uint8_t ActivityRecognitionEnabled= 0;
uint8_t CarryPositionEnabled= 0;
uint8_t GestureRecognitionEnabled= 0;
uint8_t MotionIntensityEnabled= 0;
uint8_t PedometerAlgorithmEnabled= 0;
uint8_t SensorFusionEnabled= 0;
uint8_t ECompassEnabled= 0;

uint8_t TIM1_CHANNEL_1_Enabled= 0;
uint8_t TIM1_CHANNEL_2_Enabled= 0;
uint8_t TIM1_CHANNEL_3_Enabled= 0;
uint8_t TIM1_CHANNEL_4_Enabled= 0;

uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;
uint32_t uhCCR2_Val = DEFAULT_uhCCR2_Val;
uint32_t uhCCR3_Val = DEFAULT_uhCCR3_Val;
uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;

uint32_t CalibrationData[6];
uint8_t  NodeName[8];

/* Table with All the known Meta Data */
MDM_knownGMD_t known_MetaData[]={
  {GMD_MAG_CALIBRATION,(sizeof(CalibrationData))},
  {GMD_NODE_NAME,      (sizeof(NodeName))},
  {GMD_END    ,0}/* THIS MUST BE THE LAST ONE */
};

uint16_t PedometerStepCount= 0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile int ButtonPressed		= 0;
static volatile int MEMSInterrupt		= 0;
static volatile uint32_t SendEnv		= 0;
static volatile uint32_t SendAccGyroMag	= 0;
static volatile uint32_t TimeStamp		= 0;

static float UsedAccelerometerDataRate;

static volatile uint32_t Quaternion      = 0;
static volatile uint32_t UpdateMotionAR  = 0;
static volatile uint32_t UpdateMotionCP  = 0;
static volatile uint32_t UpdateMotionGR  = 0;
static volatile uint32_t UpdateMotionPM  = 0;
static volatile uint32_t UpdateMotionID  = 0;

unsigned char isCal = 0;

static uint32_t mag_time_stamp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
static void Set2GAccelerometerFullScale(void);
static void Set4GAccelerometerFullScale(void);
static void InitMotionLibraries(void);
static void EnableDisableFeatures(void);
  
static unsigned char ResetCalibrationInMemory(void);
static unsigned char ReCallNodeNameFromMemory(void);

static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void MagCalibTest(void);
static void ReCalibration(void);

static void SendMotionData(void);

static void ButtonCallback(void);

static void Environmental_StartStopTimer(void);

static void TIM1_CHANNEL_1_StartStop(void);
static void TIM1_CHANNEL_2_StartStop(void);
static void TIM1_CHANNEL_3_StartStop(void);
static void TIM1_CHANNEL_4_StartStop(void);

static void AccEnv_StartStop(void);

static void ComputeQuaternions(void);
static void ComputeMotionAR(void);
static void ComputeMotionCP(void);
static void ComputeMotionGR(void);
static void ComputeMotionPM(void);
static void ComputeMotionID(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  MX_CRC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  InitTargetPlatform();

  /* Check the MetaDataManager */
  InitMetaDataManager((void *)&known_MetaData,MDM_DATA_TYPE_GMD,NULL); 
  
  MOTENV1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n"
#endif
         "\tSend Every %4dmS %d Short precision Quaternions\r\n"
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n\n"
           ,
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         QUAT_UPDATE_MUL_10MS*10,SEND_N_QUATERNIONS,
         ALGO_PERIOD_ENV,
         ALGO_PERIOD_ACC_GYRO_MAG );

#ifdef MOTENV1_DEBUG_CONNECTION
  MOTENV1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* MOTENV1_DEBUG_CONNECTION */

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
  MOTENV1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */
  
    /* Set Node Name */
  ReCallNodeNameFromMemory();
  
  /* Initialize the BlueNRG stack and services */
  BluetoothInit();

  InitHWFeatures();
  
  /* Check the BootLoader Compliance */
  MOTENV1_PRINTF("\r\n");
  if(CheckBootLoaderCompliance()) {
    MOTENV1_PRINTF("BootLoader Compliant with FOTA procedure\r\n\n");
  } else {
    MOTENV1_PRINTF("ERROR: BootLoader NOT Compliant with FOTA procedure\r\n\n");
  }

  /* Set Accelerometer Full Scale to 2G */
  if(TargetBoardFeatures.IKS01Ax_support) {
    Set2GAccelerometerFullScale();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(!connected) {
      if(!TargetBoardFeatures.LedStatus) {
        if(!(HAL_GetTick()&0x3FF)) {
          LedOnTargetPlatform();
        }
      } else {
        if(!(HAL_GetTick()&0x3F)) {
          LedOffTargetPlatform();
        }
      }
    }

    if(set_connectable){
      if (TargetBoardFeatures.IKS01Ax_support) {
        InitMotionLibraries();
      } /*  TargetBoardFeatures.IKS01Ax_support  */
      
      if(NecessityToSaveMetaDataManager) {
        uint32_t Success = EraseMetaDataManager();
        if(Success) {
          SaveMetaDataManager();
        }
      }

      /* Now update the BLE advertize data and make the Board connectable */
      setConnectable();
      set_connectable = FALSE;
    }
    
    /* Enable/Disable BLE features and related timer */
    EnableDisableFeatures();
	
    /* handle BLE event */
    if(HCI_ProcessEvent) {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }

    /* Handle user button */
    if(ButtonPressed) {
      ButtonCallback();
      ButtonPressed=0;
    }

    /* Handle Interrupt from MEMS */
    if(MEMSInterrupt) {
      MEMSCallback();
      MEMSInterrupt=0;
    }

    /* Handle Re-Calibration */
    if(ForceReCalibration) {
      ForceReCalibration=0;
      ReCalibration();
    }
    
    /* Environmental Data */
    if(SendEnv) {
      SendEnv=0;
      SendEnvironmentalData();
    }
	
    /* Motion Data */
    if(SendAccGyroMag) {
      SendAccGyroMag=0;
      SendMotionData();
    }

    /* MotionFX */
    if(Quaternion) {
      Quaternion=0;
      ComputeQuaternions();
    }

    /* MotionAR */
    if(UpdateMotionAR) {
      UpdateMotionAR=0;
      ComputeMotionAR();
    }

    /* MotionCP */
    if(UpdateMotionCP) {
      UpdateMotionCP=0;
      ComputeMotionCP();
    }

    /* MotionGR */
    if(UpdateMotionGR) {
      UpdateMotionGR=0;
      ComputeMotionGR();
    }

    /* MotionPM */
    if(UpdateMotionPM) {
      UpdateMotionPM=0;
      ComputeMotionPM();
    }
    
    /* MotionID */
    if(UpdateMotionID) {
      UpdateMotionID=0;
      ComputeMotionID();
    }

    /* Wait for Event */
    __WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = DEFAULT_uhCCR1_Val;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR2_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR3_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR4_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = ((SystemCoreClock / ALGO_FREQ_ENV) - 1);
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = ((TIM_CLOCK_ENV / ALGO_FREQ_ENV) -1);
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI1_CS_Pin_Pin|LD2_Pin|SPI1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_EXTI_Pin_Pin */
  GPIO_InitStruct.Pin = SPI1_EXTI_Pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SPI1_EXTI_Pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin_Pin LD2_Pin SPI1_RST_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin_Pin|LD2_Pin|SPI1_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_ACC_INT_Pin */
  GPIO_InitStruct.Pin = MEMS_ACC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_ACC_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
  * @brief  This function sets the ACC FS to 2g
  * @param  None
  * @retval None
  */
static void Set2GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-2g */
  MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE,MOTION_ACCELERO,2.0f);
  
  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE,MOTION_ACCELERO,&sensitivity);
  sensitivity_Mul = sensitivity * ((float) FROM_MG_TO_G);
}

/**
  * @brief  This function dsets the ACC FS to 4g
  * @param  None
  * @retval None
  */
static void Set4GAccelerometerFullScale(void)
{
  /* Set Full Scale to +/-4g */
  MOTION_SENSOR_SetFullScale(ACCELERO_INSTANCE,MOTION_ACCELERO,4.0f);

  /* Read the Acc Sensitivity */
  MOTION_SENSOR_GetSensitivity(ACCELERO_INSTANCE,MOTION_ACCELERO,&sensitivity);
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
  if(TargetBoardFeatures.MotionFXIsInitalized==0) {
    MotionFX_manager_init();
    MotionFX_manager_start_9X();
    /* Enable magnetometer calibration */
    MagCalibTest();
  }
  /* Code for MotionFX integration - End Section */
  
  /* Code for MotionAR integration - Start Section */
  /* Initialize MotionAR Library */
  if(TargetBoardFeatures.MotionARIsInitalized==0) {
    MotionAR_manager_init();
  }
  /* Code for MotionAR integration - End Section */
  
  /* Code for MotionCP integration - Start Section */
  /* Initialize MotionCP Library */
  if(TargetBoardFeatures.MotionCPIsInitalized==0) {
    MotionCP_manager_init();
  }
  /* Code for MotionCP integration - End Section */
  
  /* Code for MotionGR integration - Start Section */
  /* Initialize MotionGR Library */
  if(TargetBoardFeatures.MotionGRIsInitalized==0) {
    MotionGR_manager_init();
  }
  /* Code for MotionGR integration - End Section */
  
  /* Code for MotionPM integration - Start Section */
  /* Initialize MotionPM Library */
  if(TargetBoardFeatures.MotionPMIsInitalized==0) {
    MotionPM_manager_init();
  }
  /* Code for MotionPM integration - End Section */
  
  /* Initialize MotionID Library */
  if(TargetBoardFeatures.MotionIDIsInitalized==0) {
    MotionID_manager_init();
  }
}

/**
  * @brief  Enable/Disable BLE Features
  * @param  None
  * @retval None
  */
static void EnableDisableFeatures(void)
{
  /* Enviromental Features */
  if(BLE_Env_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    Environmental_StartStopTimer();
    BLE_Env_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Accelerometer events Features */
  if(BLE_AccEnv_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    AccEnv_StartStop();   
    BLE_AccEnv_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Led Features */
  if(BLE_Led_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    if(BLE_Led_NotifyEvent == BLE_NOTIFY_SUB)
    {
      LedEnabled= 1;
      BLE_LedStatusUpdate(TargetBoardFeatures.LedStatus);
    }
    
    if(BLE_Led_NotifyEvent == BLE_NOTIFY_UNSUB)
      LedEnabled= 0;
    
    BLE_Led_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Sensor Fusion Features */
  if(BLE_SensorFusion_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_1_StartStop();   
    BLE_SensorFusion_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* E-Compass Features */
  if(BLE_ECompass_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_1_StartStop();   
    BLE_ECompass_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Activity Recognition Features */
  if(BLE_CarryPosition_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_CarryPosition_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Gesture Recognition Features */
  if(BLE_GestureRecognition_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_GestureRecognition_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Pedometer Algorithm Features */
  if(BLE_PedometerAlgorithm_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_2_StartStop();   
    BLE_PedometerAlgorithm_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Activity Recognition Features */
  if(BLE_ActRec_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop();   
    BLE_ActRec_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Gesture Recognition Features */
  if(BLE_MotionIntensity_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_3_StartStop();   
    BLE_MotionIntensity_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Inertial Features */
  if(BLE_Inertial_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM1_CHANNEL_4_StartStop();   
    BLE_Inertial_NotifyEvent = BLE_NOTIFY_NOTHING;
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
  if(connected) {
    static uint32_t HowManyButtonPress=0;
    static uint32_t tickstart=0;
    uint32_t tickstop;

    if(!tickstart) {
      tickstart = HAL_GetTick();
    }

    tickstop = HAL_GetTick();

    if((tickstop-tickstart)>2000) {
      HowManyButtonPress=0;
      tickstart=tickstop;
    }


    if(TargetBoardFeatures.MotionFXIsInitalized) {
      if((HowManyButtonPress+1)==BLUEMSYS_N_BUTTON_PRESS) {
        ForceReCalibration=1;
        HowManyButtonPress=0;
      } else {
        HowManyButtonPress++;
        if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
           BytesToWrite = sprintf((char *)BufferToWrite, "%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
           Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV1_PRINTF("%ld in %ldmS Reset Calib\r\n",3-HowManyButtonPress,2000-(tickstop-tickstart));
        }
      }
    } else {
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
  if(connected) {
    /* Reset the Compass Calibration */
    isCal=0;
    MFX_MagCal_output_t mag_cal_test;

    /* Notifications of Compass Calibration */
    Config_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
    Config_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);

    /* Reset the Calibration */
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite, "Force ReCalibration\n\r");
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Force ReCalibration\n\r");
    }

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
  MOTION_SENSOR_Event_Status_t status;
  
  MOTION_SENSOR_Get_Event_Status(ACCELERO_INSTANCE,&status);

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) ||
	  (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Pedometer */
    if(status.StepStatus != 0) {
      PedometerStepCount = GetStepHWPedometer();
      if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
         BLE_AccEnvUpdate(PedometerStepCount, 2);
      }
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Free Fall */
    if(status.FreeFallStatus != 0) {
      BLE_AccEnvUpdate(ACC_FREE_FALL, 2);
    }
  }
  
  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Single Tap */
    if(status.TapStatus != 0) {
      BLE_AccEnvUpdate(ACC_SINGLE_TAP, 2);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Double Tap */
    if(status.DoubleTapStatus != 0) {
      BLE_AccEnvUpdate(ACC_DOUBLE_TAP, 2);
    }
  }

  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to Tilt */
    if(status.TiltStatus != 0) {
      BLE_AccEnvUpdate(ACC_TILT, 2);
    }
  }
  
  if( (W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) ||
      (W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) ) {
    /* Check if the interrupt is due to 6D Orientation */
    if(status.D6DOrientationStatus != 0) {
      AccEventType Orientation = GetHWOrientation6D();
      BLE_AccEnvUpdate(Orientation, 2);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    /* Check if the interrupt is due to Wake Up */
    if(status.WakeUpStatus != 0) {
      BLE_AccEnvUpdate(ACC_WAKE_UP, 2);
    }
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS)) {
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
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  
  BLE_MANAGER_INERTIAL_Axes_t ACC_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t GYR_SensorValue;
  BLE_MANAGER_INERTIAL_Axes_t MAG_SensorValue;

  /* Read the Acc values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value);

  /* Read the Magneto values */
  MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE,MOTION_MAGNETO, &MAG_Value);

  /* Read the Gyro values */
  MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO, &GYR_Value);
  
  ACC_SensorValue.x= ACC_Value.x;
  ACC_SensorValue.y= ACC_Value.y;
  ACC_SensorValue.z= ACC_Value.z;
  
  GYR_SensorValue.x= GYR_Value.x;
  GYR_SensorValue.y= GYR_Value.y;
  GYR_SensorValue.z= GYR_Value.z;

  MAG_SensorValue.x= MAG_Value.x;
  MAG_SensorValue.y= MAG_Value.y;
  MAG_SensorValue.z= MAG_Value.z;

  BLE_AccGyroMagUpdate(&ACC_SensorValue,&GYR_SensorValue,&MAG_SensorValue);
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
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);
  MotionCP_manager_run(ACC_Value_Raw);

  if(CarryPositionCodeStored!=CarryPositionCode){
    CarryPositionCodeStored = CarryPositionCode;
    
    CarryPositionCodeSent= (BLE_CP_output_t)CarryPositionCode;
    BLE_CarryPositionUpdate(CarryPositionCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Carry Position Code= %d\r\n",CarryPositionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Sending: Carry Position Code= %d\r\n",CarryPositionCode);
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
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);
  MotionGR_manager_run(ACC_Value_Raw);

  if(GestureRecognitionCodeStored!=GestureRecognitionCode){
    GestureRecognitionCodeStored = GestureRecognitionCode;
    GestureRecognitionCodeSent= (BLE_GR_output_t)GestureRecognitionCode;
    BLE_GestureRecognitionUpdate(GestureRecognitionCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Gesture Recognition Code= %d\r\n",GestureRecognitionCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Sending: Gesture Recognition Code= %d\r\n",GestureRecognitionCode);
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
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);
  MotionPM_manager_run(ACC_Value_Raw);

  if((PM_DataOUTStored.Nsteps!=PM_DataOUT.Nsteps) | (PM_DataOUTStored.Cadence!=PM_DataOUT.Cadence)){
    PM_DataOUTStored = PM_DataOUT;
    PM_DataOUT_Sent.Cadence= PM_DataOUT.Cadence;
    PM_DataOUT_Sent.Nsteps= PM_DataOUT.Nsteps;
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
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);

  MotionAR_manager_run(ACC_Value_Raw, TimeStamp);

  if(ActivityCodeStored!=ActivityCode){
    ActivityCodeStored = ActivityCode;
    
    ActivityCodeSent= (BLE_AR_output_t)ActivityCode;

    BLE_ActRecUpdate(ActivityCodeSent, HAR_ALGO_IDX_NONE);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Activity Recognition Code= %d\r\n",ActivityCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Sending: Activity Recognition Code= %d\r\n",ActivityCode);
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
  MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;

  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxesRaw(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value_Raw);

  MotionID_manager_run(ACC_Value_Raw);

  if(MIDStored!=MIDCode){
    MIDStored = MIDCode;
    
    MIDCodeSent= (BLE_ID_output_t)MIDCode;

    BLE_MotionIntensityUpdate(MIDCodeSent);

    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
       BytesToWrite = sprintf((char *)BufferToWrite,"Sending: Motion Intensity Code= %d\r\n",MIDCode);
       Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Sending: Motion Intensity Code= %d\r\n",MIDCode);
    }
  }
}

/** @brief  MotionFX Working function
 * @param  None
 * @retval None
 */
static void ComputeQuaternions(void)
{
  static MOTION_SENSOR_Axes_t quat_axes[SEND_N_QUATERNIONS];
  static BLE_MOTION_SENSOR_Axes_t quat_axes_send[SEND_N_QUATERNIONS];
  static int32_t calibIndex =0;
  static int32_t CounterFX  =0;
  static int32_t CounterEC  =0;
  //MOTION_SENSOR_AxesRaw_t ACC_Value_Raw;
  MOTION_SENSOR_Axes_t ACC_Value;
  MOTION_SENSOR_Axes_t GYR_Value;
  MOTION_SENSOR_Axes_t MAG_Value;
  
  MFX_input_t data_in;
  MFX_input_t *pdata_in = &data_in;
  MFX_output_t data_out;
  MFX_output_t *pdata_out = &data_out;
  
  MFX_MagCal_input_t mag_data_in;

  /* Increment the Counter */
  if(ECompassEnabled) {
    CounterEC++;
  } else {
    CounterFX++;
  }
  
  /* Read the Acc RAW values */
  MOTION_SENSOR_GetAxes(ACCELERO_INSTANCE,MOTION_ACCELERO,&ACC_Value);
  /* Convert acceleration from [mg] to [g] */
  data_in.acc[0] = (float)ACC_Value.x * FROM_MG_TO_G;
  data_in.acc[1] = (float)ACC_Value.y * FROM_MG_TO_G;
  data_in.acc[2] = (float)ACC_Value.z * FROM_MG_TO_G;

  /* Read the Magneto values */
  MOTION_SENSOR_GetAxes(MAGNETO_INSTANCE,MOTION_MAGNETO,&MAG_Value);
  /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
  data_in.mag[0] = (float)(MAG_Value.x - MAG_Offset.x) * FROM_MGAUSS_TO_UT50;
  data_in.mag[1] = (float)(MAG_Value.y - MAG_Offset.y) * FROM_MGAUSS_TO_UT50;
  data_in.mag[2] = (float)(MAG_Value.z - MAG_Offset.z) * FROM_MGAUSS_TO_UT50;

  /* Read the Gyro values */
  MOTION_SENSOR_GetAxes(GYRO_INSTANCE,MOTION_GYRO,&GYR_Value);
  /* Convert angular velocity from [mdps] to [dps] */
  data_in.gyro[0] = (float)GYR_Value.x * FROM_MDPS_TO_DPS;
  data_in.gyro[1] = (float)GYR_Value.y * FROM_MDPS_TO_DPS;
  data_in.gyro[2] = (float)GYR_Value.z * FROM_MDPS_TO_DPS;

  /* Check if is calibrated */
  if(isCal!=0x01){
    /* Run Compass Calibration @ 25Hz */
    calibIndex++;
    if (calibIndex == 4){
      calibIndex = 0;
      mag_data_in.mag[0]= (float)MAG_Value.x * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[1]= (float)MAG_Value.y * FROM_MGAUSS_TO_UT50;
      mag_data_in.mag[2]= (float)MAG_Value.z * FROM_MGAUSS_TO_UT50;
      
      mag_data_in.time_stamp = (int)mag_time_stamp;
      mag_time_stamp += SAMPLE_PERIOD;
      
      MotionFX_manager_MagCal_run(&mag_data_in, &magOffset);
      
      /* Control the calibration status */
      if( (magOffset.cal_quality == MFX_MAGCALOK) ||
          (magOffset.cal_quality == MFX_MAGCALGOOD) ) {
        isCal= 1;
        
        MAG_Offset.x= (int32_t)(magOffset.hi_bias[0] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.y= (int32_t)(magOffset.hi_bias[1] * FROM_UT50_TO_MGAUSS);
        MAG_Offset.z= (int32_t)(magOffset.hi_bias[2] * FROM_UT50_TO_MGAUSS);
        
        /* Disable magnetometer calibration */
        MotionFX_manager_MagCal_stop(SAMPLE_PERIOD);
        
        SaveCalibrationToMemory();
      }
        
      if(isCal == 0x01){
        if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
          BytesToWrite = sprintf((char *)BufferToWrite, "Compass Calibrated\n\r");
          Term_Update(BufferToWrite,BytesToWrite);
        } else {
          MOTENV1_PRINTF("Compass Calibrated\n\r");
        }
        
        /* Switch on the Led */
        LedOnTargetPlatform();
        if(LedEnabled) {
          BLE_LedStatusUpdate(TargetBoardFeatures.LedStatus);
        }

        /* Notifications of Compass Calibration */
        Config_Update(FEATURE_MASK_SENSORFUSION_SHORT,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
        Config_Update(FEATURE_MASK_ECOMPASS,W2ST_COMMAND_CAL_STATUS,isCal ? 100: 0);
      }
    }
  } else {
    calibIndex=0;
  }
  
  MotionFX_manager_run(pdata_in, pdata_out, MOTION_FX_ENGINE_DELTATIME);

  if(ECompassEnabled) {
    /* E-Compass Updated every 0.1 Seconds*/
    if(CounterEC==10) {
      uint16_t Angle = (uint16_t)trunc(100*pdata_out->heading);
      CounterEC=0;
      BLE_ECompassUpdate(Angle);
    }
  } else {
    int32_t QuaternionNumber = (CounterFX>SEND_N_QUATERNIONS) ? (SEND_N_QUATERNIONS-1) : (CounterFX-1);

    /* Scaling quaternions data by a factor of 10000
      (Scale factor to handle float during data transfer BT) */

    /* Save the quaternions values */
    if(pdata_out->quaternion[3] < 0){
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * (-10000));
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * (-10000));
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * (-10000));
    } else {
      quat_axes[QuaternionNumber].x = (int32_t)(pdata_out->quaternion[0] * 10000);
      quat_axes[QuaternionNumber].y = (int32_t)(pdata_out->quaternion[1] * 10000);
      quat_axes[QuaternionNumber].z = (int32_t)(pdata_out->quaternion[2] * 10000);
    }
    
    quat_axes_send[QuaternionNumber].x = quat_axes[QuaternionNumber].x;
    quat_axes_send[QuaternionNumber].y = quat_axes[QuaternionNumber].y;
    quat_axes_send[QuaternionNumber].z = quat_axes[QuaternionNumber].z;
      
    /* Every QUAT_UPDATE_MUL_10MS*10 mSeconds Send Quaternions informations via bluetooth */
    if(CounterFX==QUAT_UPDATE_MUL_10MS){
      BLE_SensorFusionUpdate(quat_axes_send, SEND_N_QUATERNIONS);
      CounterFX=0;
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
void ReadEnvironmentalData(int32_t *PressToSend,uint16_t *HumToSend,int16_t *Temp1ToSend,int16_t *Temp2ToSend)
{
  float SensorValue;
  int32_t decPart, intPart;
  
  *PressToSend=0;
  *HumToSend=0;
  *Temp2ToSend=0,*Temp1ToSend=0;

  /* Read Humidity */
  ENV_SENSOR_GetValue(HUMIDITY_INSTANCE,ENV_HUMIDITY,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *HumToSend = intPart*10+decPart;

  /* Read Temperature for sensor 1 */
  ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp1ToSend = intPart*10+decPart;
  
  /* Read Pressure */
  ENV_SENSOR_GetValue(PRESSURE_INSTANCE,ENV_PRESSURE,&SensorValue);
  MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
  *PressToSend=intPart*100+decPart;

  /* Read Temperature for sensor 2 */
  ENV_SENSOR_GetValue(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE,&SensorValue);
  MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
  *Temp2ToSend = intPart*10+decPart;
}

/**
  * @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
  * @param  None
  * @retval None
  */
static void SendEnvironmentalData(void)
{
  /* Notifications of Compass Calibration status*/
  if(FirstConnectionConfig) {
    FirstConnectionConfig=0;
  
    /* Switch on/off the LED according to calibration */
    if(isCal){
        LedOnTargetPlatform();
    } else {
      LedOffTargetPlatform();
    }
  }

  /* Pressure,Humidity, and Temperatures*/
  if(EnvironmentalTimerEnabled)
  {
    int32_t PressToSend;
    uint16_t HumToSend;
    int16_t Temp2ToSend,Temp1ToSend;
    
    /* Read all the Environmental Sensors */
    ReadEnvironmentalData(&PressToSend,&HumToSend, &Temp1ToSend,&Temp2ToSend);

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
    if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
      BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
      Term_Update(BufferToWrite,BytesToWrite);
      BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
      Term_Update(BufferToWrite,BytesToWrite);
    } else {
      MOTENV1_PRINTF("Sending: ");
      MOTENV1_PRINTF("Press=%ld ",PressToSend);
      MOTENV1_PRINTF("Hum=%d ",HumToSend);
      MOTENV1_PRINTF("Temp1=%d ",Temp1ToSend);
      MOTENV1_PRINTF("Temp2=%d ",Temp2ToSend);
      MOTENV1_PRINTF("\r\n");
    }
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */
    
    BLE_EnvironmentalUpdate(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
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
static void Environmental_StartStopTimer(void)
{
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!EnvironmentalTimerEnabled) ){
        /* Start the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Start_IT(&TimEnvHandle) != HAL_OK){
          /* Starting Error */
          Error_Handler();
        }
        
    EnvironmentalTimerEnabled= 1;
  }
  
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (EnvironmentalTimerEnabled) ){
        /* Stop the TIM Base generation in interrupt mode */
        if(HAL_TIM_Base_Stop_IT(&TimEnvHandle) != HAL_OK){
          /* Stopping Error */
          Error_Handler();
        }
    
    EnvironmentalTimerEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable Accelerometer events 
  * @param  None
  * @retval None
  */
static void AccEnv_StartStop(void)
{
  if( (BLE_AccEnv_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!AccEventEnabled) ){
        EnableHWMultipleEvents();
        ResetHWPedometer();
        Config_Update(FEATURE_MASK_ACC_EVENTS,'m',1);
        AccEventEnabled= 1;
  }
  
  if( (BLE_AccEnv_NotifyEvent == BLE_NOTIFY_UNSUB) &&
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
static void TIM1_CHANNEL_1_StartStop(void)
{
  if( ((BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_SUB) || (BLE_ECompass_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_1_Enabled) ){
        
    /* Get ODR accelerometer default Value */
    MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&UsedAccelerometerDataRate);
    /* Set ODR accelerometer: >= 100 Hz*/
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, (float)ALGO_FREQ_FX);
    /* Set FS accelerometer: = <-4g, 4g> */
    Set4GAccelerometerFullScale();
    
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
    
    TIM1_CHANNEL_1_Enabled= 1;
    
    if(BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_SUB)
      SensorFusionEnabled= 1;
    
    if(BLE_ECompass_NotifyEvent == BLE_NOTIFY_SUB)
      ECompassEnabled= 1;
  }
  
  if( ((BLE_SensorFusion_NotifyEvent == BLE_NOTIFY_UNSUB) || (BLE_ECompass_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_1_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_1) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    /* Set default ODR accelerometer value */
    MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE, MOTION_ACCELERO, UsedAccelerometerDataRate);
    /* Set default FS accelerometer: = <-2g, 2g> */
    Set2GAccelerometerFullScale();
    
    TIM1_CHANNEL_1_Enabled= 0;
    SensorFusionEnabled= 0;
    ECompassEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 2 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_2_StartStop(void)
{
  if( ((BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB) ||
       (BLE_PedometerAlgorithm_NotifyEvent == BLE_NOTIFY_SUB) ) &&
      (!TIM1_CHANNEL_2_Enabled) ){
    /* Start the TIM Base generation in interrupt mode */
    if(HAL_TIM_OC_Start_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Starting Error */
      Error_Handler();
    }

    /* Set the new Capture compare value */
    {
      uint32_t uhCapture = __HAL_TIM_GET_COUNTER(&TimCCHandle);
      /* Set the Capture Compare Register value */
      __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + uhCCR2_Val));
    }
    
    Set4GAccelerometerFullScale();
    
    TIM1_CHANNEL_2_Enabled=1;
      
    if(BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_SUB)
      CarryPositionEnabled= 1;
    if(BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_SUB)
      GestureRecognitionEnabled= 1;
    if(BLE_PedometerAlgorithm_NotifyEvent == BLE_NOTIFY_SUB)
      PedometerAlgorithmEnabled= 1;
  }
  
  if( ((BLE_CarryPosition_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_GestureRecognition_NotifyEvent == BLE_NOTIFY_UNSUB) ||
       (BLE_PedometerAlgorithm_NotifyEvent == BLE_NOTIFY_UNSUB) ) &&
      (TIM1_CHANNEL_2_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_2) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    Set2GAccelerometerFullScale();
    
    TIM1_CHANNEL_2_Enabled=0;
    CarryPositionEnabled= 0;
    GestureRecognitionEnabled= 0;
    PedometerAlgorithmEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 3 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_3_StartStop(void)
{
  if( ((BLE_ActRec_NotifyEvent == BLE_NOTIFY_SUB) || (BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_SUB)) &&
      (!TIM1_CHANNEL_3_Enabled) ){
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
    
    Set4GAccelerometerFullScale();
    
    TIM1_CHANNEL_3_Enabled= 1;
    
    if(BLE_ActRec_NotifyEvent == BLE_NOTIFY_SUB)
      ActivityRecognitionEnabled= 1;
    if(BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_SUB)
      MotionIntensityEnabled= 1;
  }
  
  if( ((BLE_ActRec_NotifyEvent == BLE_NOTIFY_UNSUB) || (BLE_MotionIntensity_NotifyEvent == BLE_NOTIFY_UNSUB)) &&
      (TIM1_CHANNEL_3_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_3) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    Set2GAccelerometerFullScale();
    TIM1_CHANNEL_3_Enabled= 0;
    ActivityRecognitionEnabled= 0;
    MotionIntensityEnabled= 0;
  }
}

/**
  * @brief  Enable/Disable TIM1 Channel 4 
  * @param  None
  * @retval None
  */
static void TIM1_CHANNEL_4_StartStop(void)
{ 
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!TIM1_CHANNEL_4_Enabled) ){
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
    
    TIM1_CHANNEL_4_Enabled= 1;
    InertialTimerEnabled= 1;
  }
  
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (TIM1_CHANNEL_4_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    TIM1_CHANNEL_4_Enabled= 0;
    InertialTimerEnabled= 0;
  }
}

/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin){
  case GPIO_PIN_5:
    MEMSInterrupt=1;
    break;
  case GPIO_PIN_13:
    ButtonPressed=1;
    break;
  }
}

/**
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  /* TIM1_CH1 toggling with frequency = 100Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    
    if ((SensorFusionEnabled) || (ECompassEnabled)) {
      Quaternion=1;
    }
  }

  /* TIM1_CH2 toggling with frequency = 50Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_2, (uhCapture + DEFAULT_uhCCR2_Val));

    if(CarryPositionEnabled) {
      UpdateMotionCP=1;
    } else if(PedometerAlgorithmEnabled) {
      UpdateMotionPM=1;
    } else if(GestureRecognitionEnabled) {
      UpdateMotionGR=1;
    }
  }

  /* TIM1_CH3 toggling with frequency = 16Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_3, (uhCapture + DEFAULT_uhCCR3_Val));

    if(MotionIntensityEnabled) {
      UpdateMotionID=1;
    } else if(ActivityRecognitionEnabled) {
        UpdateMotionAR=1;
        TimeStamp += ALGO_FREQ_AR_ID;
    }
  }

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    
    if(InertialTimerEnabled){
      SendAccGyroMag=1;
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
  if(htim == (&TimEnvHandle)) {
    /* Environmental */
    if(EnvironmentalTimerEnabled)
    {
      SendEnv=1;
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
 * @brief  Test if calibration data are available
 * @param  None
 * @retval None
 */
static void MagCalibTest(void)
{
  ReCallCalibrationFromMemory();
    
  if(CalibrationData[0]== BLUEMSYS_CHECK_CALIBRATION) {
    if(CalibrationData[1] == TargetBoardFeatures.mems_expansion_board) {
      if( (CalibrationData[2] == MFX_MAGCALOK) ||
          (CalibrationData[2] == MFX_MAGCALGOOD) ) {
        MAG_Offset.x = CalibrationData[3];
        MAG_Offset.y = CalibrationData[4];
        MAG_Offset.z = CalibrationData[5];

        isCal =1;
        
        MOTENV1_PRINTF("Magneto Calibration Read\r\n");
      } else {
        isCal =0;
        MOTENV1_PRINTF("Magneto Calibration quality is not good\r\n");
      }
    } else {
      MOTENV1_PRINTF("Magneto Calibration Not correct for Current %s board\r\n",TargetBoardFeatures.mems_expansion_board ? "IKS01A3" : "IKS01A2");
      ResetCalibrationInMemory();    
      isCal=0;
    }
  } else {
    MOTENV1_PRINTF("Magneto Calibration Not present\r\n");
    isCal=0;
  }
  
  if(!isCal) {
    MAG_Offset.x = 0;
    MAG_Offset.y = 0;
    MAG_Offset.z = 0;
  }
}

/**
 * @brief  Check if there are a valid Calibration Values in Memory and read them
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char ReCallCalibrationFromMemory()
{
  /* ReLoad the Calibration Values from RAM */
  unsigned char Success=0;
  
  /* Recall the calibration Credential saved */
  MDM_ReCallGMD(GMD_MAG_CALIBRATION,(void *)&CalibrationData);
  
  return Success;
}

/**
 * @brief  Save the Magnetometer Calibration Values to Memory
 * @param uint32_t *MagnetoCalibration the Magneto Calibration
 * @retval unsigned char Success/Not Success
 */
unsigned char SaveCalibrationToMemory()
{
  unsigned char Success=1;

  /* Store in RAM */
  CalibrationData[0] = BLUEMSYS_CHECK_CALIBRATION;
  CalibrationData[1] = TargetBoardFeatures.mems_expansion_board;
  CalibrationData[2] = magOffset.cal_quality;
  CalibrationData[3] = MAG_Offset.x;
  CalibrationData[4] = MAG_Offset.y;
  CalibrationData[5] = MAG_Offset.z;

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
   BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be saved in FLASH\r\n");
   Term_Update(BufferToWrite,BytesToWrite);
  } else {
    MOTENV1_PRINTF("Magneto Calibration will be saved in FLASH\r\n");
  }

  MDM_SaveGMD(GMD_MAG_CALIBRATION,(void *)&CalibrationData);
  
  NecessityToSaveMetaDataManager=1;

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
  unsigned char Success=1;
  int32_t Counter;

  for(Counter=0;Counter<6;Counter++) {
    CalibrationData[Counter]=0x0;
    //CalibrationData[Counter]=0xFFFFFFFF;
  }

  if(BLE_StdTerm_Service==BLE_SERV_ENABLE) {
     BytesToWrite = sprintf((char *)BufferToWrite, "Magneto Calibration will be eresed in FLASH\r\n");
     Term_Update(BufferToWrite,BytesToWrite);
  } else {
    MOTENV1_PRINTF("Magneto Calibration will be eresed in FLASH\r\n");
  }
  
  MDM_SaveGMD(GMD_MAG_CALIBRATION,(void *)&CalibrationData);

  NecessityToSaveMetaDataManager=1;
  return Success;
}

/**
 * @brief  Check if there are a valid Node Name Values in Memory and read them
 * @param  None
 * @retval unsigned char Success/Not Success
 */
static unsigned char ReCallNodeNameFromMemory(void)
{
  /* ReLoad the Node Name Values from RAM */
  unsigned char Success=0;
  
  //Set the BLE Board Name 
  sprintf(BlueNRG_StackValue.BoardName,"%s%c%c%c","ME1V",
          MOTENV1_VERSION_MAJOR,
          MOTENV1_VERSION_MINOR,
          MOTENV1_VERSION_PATCH);

  /* Recall the node name Credential saved */
  MDM_ReCallGMD(GMD_NODE_NAME,(void *)&NodeName);
  
  if(NodeName[0] != 0x12)
  {
    NodeName[0]= 0x12;
    
    for(int i=0; i<7; i++)
      NodeName[i+1]= BlueNRG_StackValue.BoardName[i];
    
    MDM_SaveGMD(GMD_NODE_NAME,(void *)&NodeName);
    NecessityToSaveMetaDataManager=1;
  }
  else
  {
    for(int i=0; i<7; i++)
      BlueNRG_StackValue.BoardName[i]= NodeName[i+1];
  }

  return Success;
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
  while((HAL_GetTick() - tickstart) < Delay){
    __WFI();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

