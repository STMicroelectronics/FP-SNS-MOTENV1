/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.0
  * @date    31-January-2023
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>

#include "TargetFeatures.h"
#include "BLE_Function.h"
#include "main.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim1;

TargetFeatures_t TargetBoardFeatures;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);

/**
  * @brief Led_TIM_INSTANCE Initialization Function
  * @param None
  * @retval None
  */
void Led_TIM_Init(void)
{

  /* USER CODE BEGIN Led_TIM_INSTANCE_Init 0 */

  /* USER CODE END Led_TIM_INSTANCE_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  TimLedHandle.Instance = Led_TIM_INSTANCE;
  TimLedHandle.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  TimLedHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimLedHandle.Init.Period = ((((SystemCoreClock / 10000) * ALGO_PERIOD_LED) / 1000)- 1);
  TimLedHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimLedHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TimLedHandle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TimLedHandle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TimLedHandle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Led_TIM_INSTANCE_Init 2 */

  /* USER CODE END Led_TIM_INSTANCE_Init 2 */

}

/**
  * @brief TimDistanceHandle Initialization Function
  * @param None
  * @retval None
  */
void Env_TIM_Init(void)
{

  /* USER CODE BEGIN Env_TIM_Init 0 */

  /* USER CODE END Env_TIM_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN Env_TIM_Init 1 */

  /* USER CODE END Env_TIM_Init 1 */
  TimEnvHandle.Instance = ENV_TIM_INSTANCE;
  TimEnvHandle.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  TimEnvHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimEnvHandle.Init.Period = ((((SystemCoreClock / 10000) * ALGO_PERIOD_ENV) / 1000)- 1);
  TimEnvHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimEnvHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&TimEnvHandle) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TimEnvHandle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TimEnvHandle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Env_TIM_Init 2 */

  /* USER CODE END Env_TIM_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
void MotionAlgo_TIM_Init(void)
{

  /* USER CODE BEGIN MotionAlgo_TIM_Init 0 */

  /* USER CODE END MotionAlgo_TIM_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN MotionAlgo_TIM_Init 1 */

  /* USER CODE END MotionAlgo_TIM_Init 1 */
  TimCCHandle.Instance = MOTION_ALGO_TIM_INSTANCE;
  TimCCHandle.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  TimCCHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TimCCHandle.Init.Period = 65535;
  TimCCHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TimCCHandle.Init.RepetitionCounter = 0;
  TimCCHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TimCCHandle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = (10000 / ALGO_FREQ_FX);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (10000 / ALGO_FREQ_CP_GR_PM);
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (10000 / ALGO_FREQ_AR_ID);
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (10000 / FREQ_ACC_GYRO_MAG);
  if (HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN MotionAlgo_TIM_Init 2 */

  /* USER CODE END MotionAlgo_TIM_Init 2 */
}

/**
  * @brief  Initialize all the Target platform's Features
  * @param  None
  * @retval None
  */
void InitTargetPlatform(void)
{
#ifdef MOTENV1_ENABLE_PRINTF
  /* UART Initialization */
  if(BSP_COM_Init(COM1) != BSP_ERROR_NONE) {
    Error_Handler();
  } else {
    MOTENV1_PRINTF("\033[2J\033[1;1f");
    MOTENV1_PRINTF("UART Initialized\r\n");
  }
#endif /* MOTENV1_ENABLE_PRINTF */

  /* Initialize button */
  BSP_PB_Init(BUTTON_KEY, BUTTON_MODE_EXTI);

  /* Initialize LED */
  BSP_LED_Init(LED2);

  MOTENV1_PRINTF("\r\nSTMicroelectronics %s:\r\n"
         "\tVersion %c.%c.%c\r\n"
         "\t%s board"
          "\r\n",
          MOTENV1_PACKAGENAME,
          MOTENV1_VERSION_MAJOR,MOTENV1_VERSION_MINOR,MOTENV1_VERSION_PATCH,
          BLE_STM32_BOARD);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  MOTENV1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  MOTENV1_PRINTF("\nCode compiled for X-NUCLEO-IKS01A2 board\n\r");
  TargetBoardFeatures.mems_expansion_board= _IKS01A2;

   /* Accelero & Gyro */
  if (MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE){
    MOTENV1_PRINTF("\tOK Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tOK Gyroscope Sensor\n\r");
    TargetBoardFeatures.IKS01Ax_support= 1;
  } else {
    MOTENV1_PRINTF("\tError Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tError Gyroscope Sensor\n\r");

    MOTENV1_PRINTF("\tX-NUCLEO-IKS01A2 board not present\n\r");

  }

  if(TargetBoardFeatures.IKS01Ax_support)
  {
    if(MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE){
      MOTENV1_PRINTF("\tOK Magneto Sensor\n\r");
    } else {
      MOTENV1_PRINTF("\tError Magneto Sensor\n\r");
    }

    /* Temperarure & Humidity */
    if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE| ENV_HUMIDITY)==BSP_ERROR_NONE){
      MOTENV1_PRINTF("\tOK Temperature and Humidity\t(Sensor1)\n\r");
      TargetBoardFeatures.NumTempSensors++;
    } else {
      MOTENV1_PRINTF("\tError Temperature and Humidity\t(Sensor1)\n\r");
    }

    /* Temperarure & Pressure */
    if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
      MOTENV1_PRINTF("\tOK Temperature and Pressure\t(Sensor2)\n\r");
      TargetBoardFeatures.NumTempSensors++;
    } else {
      MOTENV1_PRINTF("\tError Temperature and Pressure\t(Sensor2)\n\r");

    }

    /*  Enable all the sensors */
    if(MOTION_SENSOR_Enable(ACCELERO_INSTANCE, MOTION_ACCELERO)==BSP_ERROR_NONE)
      MOTENV1_PRINTF("\tEnabled Accelero Sensor\n\r");
    if(MOTION_SENSOR_Enable(GYRO_INSTANCE, MOTION_GYRO)==BSP_ERROR_NONE)
      MOTENV1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
    if(MOTION_SENSOR_Enable(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE)
      MOTENV1_PRINTF("\tEnabled Magneto Sensor\n\r");

    if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE)==BSP_ERROR_NONE)
      MOTENV1_PRINTF("\tEnabled Temperature\t(Sensor1)\n\r");
    if(ENV_SENSOR_Enable(HUMIDITY_INSTANCE, ENV_HUMIDITY)==BSP_ERROR_NONE)
      MOTENV1_PRINTF("\tEnabled Humidity\t(Sensor1)\n\r");

    if(TargetBoardFeatures.NumTempSensors==2) {
      if(ENV_SENSOR_Enable(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE)==BSP_ERROR_NONE)
        MOTENV1_PRINTF("\tEnabled Temperature\t(Sensor2)\n\r");
      if(ENV_SENSOR_Enable(PRESSURE_INSTANCE, ENV_PRESSURE)==BSP_ERROR_NONE)
        MOTENV1_PRINTF("\tEnabled Pressure\t(Sensor2)\n\r");
    }
  }
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  BSP_LED_On(LED2);
  TargetBoardFeatures.LedStatus=1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  BSP_LED_Off(LED2);
  TargetBoardFeatures.LedStatus=0;
}
