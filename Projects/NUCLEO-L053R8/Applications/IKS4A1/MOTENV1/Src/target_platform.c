/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    target_platform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>

#include "target_features.h"
#include "app_motenv1.h"
#include "ble_function.h"
#include "main.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
EXTI_HandleTypeDef hexti5 = {.Line = EXTI_LINE_5};
TIM_HandleTypeDef htim2;

TargetFeatures_t TargetBoardFeatures;

/* Local defines -------------------------------------------------------------*/
#define GENERIC_TIM_PERIOD 65535

/* Local function prototypes --------------------------------------------------*/
static void BSP_ACC_INT_Callback(void);
static void Init_MEM1_Sensors(void);

/**
  * @brief Register event irq handler for accelerometer interrupt pin
  * @param None
  * @retval None
  */
void SetAccIntPin_exti(void)
{
  /* register event irq handler */
  HAL_EXTI_GetHandle(&hexti5, EXTI_LINE_5);
  HAL_EXTI_RegisterCallback(&hexti5, HAL_EXTI_COMMON_CB_ID, BSP_ACC_INT_Callback);
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 4, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
}

/**
  * @brief Timer Initialization Function
  * @param None
  * @retval None
  */
void TIM_Init(void)
{

  /* USER CODE BEGIN TIM_Init 0 */

  /* USER CODE END TIM_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM_Init 1 */

  /* USER CODE END TIM_Init 1 */
  TIM_CC_HANDLE.Instance = TIM_INSTANCE;
  TIM_CC_HANDLE.Init.Prescaler = ((SystemCoreClock / 10000) - 1);
  TIM_CC_HANDLE.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_CC_HANDLE.Init.Period = GENERIC_TIM_PERIOD;
  TIM_CC_HANDLE.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_CC_HANDLE.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&TIM_CC_HANDLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_CC_HANDLE, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = (10000 / ALGO_FREQ_ENV);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (10000 / FREQ_ACC_GYRO_MAG);
  if (HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = (10000 / ALGO_FREQ_LED);
  if (HAL_TIM_OC_ConfigChannel(&TIM_CC_HANDLE, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM_Init 2 */

  /* USER CODE END TIM_Init 2 */

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
  if (BSP_COM_Init(COM1) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }
  else
  {
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
                 MOTENV1_VERSION_MAJOR, MOTENV1_VERSION_MINOR, MOTENV1_VERSION_PATCH,
                 BLE_STM32_BOARD);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Initialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  MOTENV1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS1 sensors
  * @param None
  * @retval None
  */
static void Init_MEM1_Sensors(void)
{
  MOTENV1_PRINTF("\nCode compiled for X-NUCLEO-IKS4A1 board\n\r");
  TargetBoardFeatures.mems_expansion_board = _IKS4A1;

  /* Accelero & Gyro */
  if (MOTION_SENSOR_INIT(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO) == BSP_ERROR_NONE)
  {
    MOTENV1_PRINTF("\tOK Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tOK Gyroscope Sensor\n\r");
    TargetBoardFeatures.IKS01Ax_support = 1;
  }
  else
  {
    MOTENV1_PRINTF("\tError Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tError Gyroscope Sensor\n\r");

    MOTENV1_PRINTF("\tX-NUCLEO-IKS4A1 board not present\n\r");

  }

  if (TargetBoardFeatures.IKS01Ax_support)
  {
    if (MOTION_SENSOR_INIT(MAGNETO_INSTANCE, MOTION_MAGNETO) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tOK Magneto Sensor\n\r");
    }
    else
    {
      MOTENV1_PRINTF("\tError Magneto Sensor\n\r");
    }

    /* Temperature */
    if (ENV_SENSOR_INIT(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tOK Temperature Sensor\n\r");
      TargetBoardFeatures.NumTempSensors++;
    }
    else
    {
      MOTENV1_PRINTF("\tError Temperature Sensor\n\r");
    }

    /* Humidity */
    if (ENV_SENSOR_INIT(HUMIDITY_INSTANCE, ENV_HUMIDITY) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tOK Humidity Sensor\n\r");
    }
    else
    {
      MOTENV1_PRINTF("Error Humidity Sensor\n\r");
    }

    /* Temperature & Pressure */
    if (ENV_SENSOR_INIT(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE | ENV_PRESSURE) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tOK Temperature and Pressure\n\r");
      TargetBoardFeatures.NumTempSensors++;
    }
    else
    {
      MOTENV1_PRINTF("\tError Temperature and Pressure\n\r");

    }

    /*  Enable Accelerometer sensor */
    if (MOTION_SENSOR_ENABLE(ACCELERO_INSTANCE, MOTION_ACCELERO) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tEnabled Accelero Sensor\n\r");
    }
    /*  Enable giroscope sensor */
    if (MOTION_SENSOR_ENABLE(GYRO_INSTANCE, MOTION_GYRO) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tEnabled Gyroscope Sensor\n\r");
    }
    /*  Enable magnetometer sensor */
    if (MOTION_SENSOR_ENABLE(MAGNETO_INSTANCE, MOTION_MAGNETO) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tEnabled Magneto Sensor\n\r");
    }
    /*  Enable temperature sensor */
    if (ENV_SENSOR_ENABLE(TEMPERATURE_INSTANCE_1, ENV_TEMPERATURE) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tEnabled Temperature\n\r");
    }
    /*  Enable humidity sensor */
    if (ENV_SENSOR_ENABLE(HUMIDITY_INSTANCE, ENV_HUMIDITY) == BSP_ERROR_NONE)
    {
      MOTENV1_PRINTF("\tEnabled Humidity\n\r");
    }

    if (TargetBoardFeatures.NumTempSensors == 2)
    {
      if (ENV_SENSOR_ENABLE(TEMPERATURE_INSTANCE_2, ENV_TEMPERATURE) == BSP_ERROR_NONE)
      {
        MOTENV1_PRINTF("\tEnabled Temperature\n\r");
      }
      if (ENV_SENSOR_ENABLE(PRESSURE_INSTANCE, ENV_PRESSURE) == BSP_ERROR_NONE)
      {
        MOTENV1_PRINTF("\tEnabled Pressure\n\r");
      }
    }
  }
}

/**
  * @brief  BSP ACC_INT callback
  * @param  None
  * @retval None.
  */
static void BSP_ACC_INT_Callback(void)
{
  MEMSInterrupt = 1;
}

/**
  * @brief  This function switches on the LED
  * @param  None
  * @retval None
  */
void LedOnTargetPlatform(void)
{
  /* Turns on the Green Led */
  BSP_LED_On(LED2);
  TargetBoardFeatures.LedStatus = 1;
}

/**
  * @brief  This function switches off the LED
  * @param  None
  * @retval None
  */
void LedOffTargetPlatform(void)
{
  /* Turns off the Green Led */
  BSP_LED_Off(LED2);
  TargetBoardFeatures.LedStatus = 0;
}
