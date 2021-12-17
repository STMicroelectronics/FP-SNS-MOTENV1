/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    main.c
  * @author  System Research & Applications Team - Agrate/Catania Lab.
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "bluenrg_utils.h"
#include "HWAdvanceFeatures.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Exported Variables -------------------------------------------------------------*/

float sensitivity;
/* Acc sensitivity multiply by FROM_MG_TO_G constant */
float sensitivity_Mul;

uint16_t PedometerStepCount= 0;

volatile uint32_t HCI_ProcessEvent=0;

uint8_t EnvironmentalTimerEnabled= 0;
uint8_t InertialTimerEnabled= 0;
uint8_t AccEventEnabled= 0;
uint8_t LedEnabled= 0;

uint8_t TIM2_CHANNEL_1_Enabled= 0;
uint8_t TIM2_CHANNEL_4_Enabled= 0;

uint32_t uhCCR1_Val = DEFAULT_uhCCR1_Val;
uint32_t uhCCR4_Val = DEFAULT_uhCCR4_Val;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static volatile int ButtonPressed        =0;
static volatile int MEMSInterrupt        =0;
static volatile uint32_t SendEnv         =0;
static volatile uint32_t SendAccGyroMag  =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void Set2GAccelerometerFullScale(void);
static void EnableDisableFeatures(void);

static void SendEnvironmentalData(void);
static void MEMSCallback(void);

static void SendMotionData(void);

static void ButtonCallback(void);

static void TIM2_CHANNEL_1_StartStop(void);
static void TIM2_CHANNEL_4_StartStop(void);

static void AccEnv_StartStop(void);
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
  __SYSCFG_CLK_ENABLE();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  InitTargetPlatform();
  MOTENV1_PRINTF("\n\t(HAL %ld.%ld.%ld_%ld)\r\n"
        "\tCompiled %s %s"
#if defined (__IAR_SYSTEMS_ICC__)
        " (IAR)\r\n"
#elif defined (__CC_ARM)
        " (KEIL)\r\n"
#elif defined (__GNUC__)
        " (STM32CubeIDE)\r\n"
#endif
         "\tSend Every %4dmS Temperature/Humidity/Pressure\r\n"
         "\tSend Every %4dmS Acc/Gyro/Magneto\r\n\n"
           ,
           HAL_GetHalVersion() >>24,
          (HAL_GetHalVersion() >>16)&0xFF,
          (HAL_GetHalVersion() >> 8)&0xFF,
           HAL_GetHalVersion()      &0xFF,
         __DATE__,__TIME__,
         ALGO_PERIOD_ENV,
         ALGO_PERIOD_ACC_GYRO_MAG );

#ifdef MOTENV1_DEBUG_CONNECTION
  MOTENV1_PRINTF("Debug Connection         Enabled\r\n");
#endif /* MOTENV1_DEBUG_CONNECTION */

#ifdef MOTENV1_DEBUG_NOTIFY_TRAMISSION
  MOTENV1_PRINTF("Debug Notify Trasmission Enabled\r\n\n");
#endif /* MOTENV1_DEBUG_NOTIFY_TRAMISSION */

  /* Initialize the BlueNRG stack and services */
  BluetoothInit();

  InitHWFeatures();

  /* Set Accelerometer Full Scale to 2G */
  if(TargetBoardFeatures.IKS01Ax_support) {
    Set2GAccelerometerFullScale();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* Led Blinking when there is not a client connected */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = DEFAULT_uhCCR1_Val;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = DEFAULT_uhCCR4_Val;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

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
  * @brief  Enable/Disable BLE Features
  * @param  None
  * @retval None
  */
static void EnableDisableFeatures(void)
{
  /* Enviromental Features */
  if(BLE_Env_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM2_CHANNEL_1_StartStop(); 
    BLE_Env_NotifyEvent = BLE_NOTIFY_NOTHING;
  }
  
  /* Inertial Features */
  if(BLE_Inertial_NotifyEvent != BLE_NOTIFY_NOTHING)
  {
    TIM2_CHANNEL_4_StartStop();   
    BLE_Inertial_NotifyEvent = BLE_NOTIFY_NOTHING;
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
}

/**********************************/
/* Characteristics Notify Service */
/**********************************/

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
static void TIM2_CHANNEL_1_StartStop(void)
{
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_SUB) &&
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
  
  if( (BLE_Env_NotifyEvent == BLE_NOTIFY_UNSUB) &&
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
  * @brief  Enable/Disable TIM1 Channel 4 
  * @param  None
  * @retval None
  */
static void TIM2_CHANNEL_4_StartStop(void)
{ 
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_SUB) &&
      (!TIM2_CHANNEL_4_Enabled) ){
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
    
    TIM2_CHANNEL_4_Enabled= 1;
    InertialTimerEnabled= 1;
  }
  
  if( (BLE_Inertial_NotifyEvent == BLE_NOTIFY_UNSUB) &&
      (TIM2_CHANNEL_4_Enabled) ){
    /* Stop the TIM Base generation in interrupt mode (for Acc/Gyro/Mag sensor) */
    if(HAL_TIM_OC_Stop_IT(&TimCCHandle, TIM_CHANNEL_4) != HAL_OK){
      /* Stopping Error */
      Error_Handler();
    }      
    
    TIM2_CHANNEL_4_Enabled= 0;
    InertialTimerEnabled= 0;
  }
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
  * @brief  Output Compare callback in non blocking mode 
  * @param  htim : TIM OC handle
  * @retval None
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
      SendEnv=1;
  }

  /* TIM1_CH4 toggling with frequency = 20 Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) {
     uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_4, (uhCapture + uhCCR4_Val));
    SendAccGyroMag=1;
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
 * @brief  BSP Push Button callback
 * @param  Button Specifies the pin connected EXTI line
 * @retval None.
 */
void BSP_PB_Callback(Button_TypeDef Button)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(Button);
  
  ButtonPressed = 1;
  
  /* This function should be implemented by the user application.
     It is called into this driver when an event on Button is triggered. */   
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
//  if(FirstConnectionConfig) {
//    FirstConnectionConfig=0;
//    LedOffTargetPlatform();
//
////    if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_LED)) {
////      LED_Update(TargetBoardFeatures.LedStatus);
////    }
//  }

  /* Pressure,Humidity, and Temperatures*/
  if(EnvironmentalTimerEnabled) {
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

