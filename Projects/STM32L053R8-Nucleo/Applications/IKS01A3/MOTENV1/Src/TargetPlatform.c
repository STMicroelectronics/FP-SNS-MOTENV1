/**
  ******************************************************************************
  * @file    TargetPlatform.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Initialization of the Target Platform
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
#include <stdio.h>
#include "TargetFeatures.h"
#include "main.h"

/* Imported variables ---------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
TargetFeatures_t TargetBoardFeatures;

/* Local defines -------------------------------------------------------------*/

/* Local function prototypes --------------------------------------------------*/
static void Init_MEM1_Sensors(void);

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
    HAL_Delay(20);
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
         "\tSTM32L053R8-Nucleo board"
          "\r\n",
          MOTENV1_PACKAGENAME,
          MOTENV1_VERSION_MAJOR,MOTENV1_VERSION_MINOR,MOTENV1_VERSION_PATCH);

  /* Reset all the Target's Features */
  memset(&TargetBoardFeatures, 0, sizeof(TargetFeatures_t));
  /* Discovery and Intialize all the MEMS Target's Features */
  Init_MEM1_Sensors();
  LedOffTargetPlatform();
  MOTENV1_PRINTF("\n\r");
}

/** @brief Initialize all the MEMS1 sensors
 * @param None
 * @retval None
 */
static void Init_MEM1_Sensors(void)
{
  MOTENV1_PRINTF("\nCode compiled for X-NUCLEO-IKS01A3 board\n\r");
  //TargetBoardFeatures.mems_expansion_board= _IKS01A2;
      
   /* Accelero & Gyro */
  if (MOTION_SENSOR_Init(ACCELERO_INSTANCE, MOTION_ACCELERO | MOTION_GYRO)==BSP_ERROR_NONE){
    MOTENV1_PRINTF("\tOK Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tOK Gyroscope Sensor\n\r");
    TargetBoardFeatures.IKS01Ax_support= 1;
  } else {
    MOTENV1_PRINTF("\tError Accelero Sensor\n\r");
    MOTENV1_PRINTF("\tError Gyroscope Sensor\n\r");
    while(1);
  }

  if(MOTION_SENSOR_Init(MAGNETO_INSTANCE, MOTION_MAGNETO)==BSP_ERROR_NONE){
    MOTENV1_PRINTF("\tOK Magneto Sensor\n\r");
  } else {
    MOTENV1_PRINTF("\tError Magneto Sensor\n\r");
    while(1);
  }

  /* Temperarure & Humidity */  
  if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_1,ENV_TEMPERATURE| ENV_HUMIDITY)==BSP_ERROR_NONE){
    MOTENV1_PRINTF("\tOK Temperature and Humidity (Sensor1)\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    MOTENV1_PRINTF("Error Temperature and Humidity (Sensor1)\n\r");
  }

  /* Temperarure & Pressure */ 
  if(ENV_SENSOR_Init(TEMPERATURE_INSTANCE_2,ENV_TEMPERATURE| ENV_PRESSURE)==BSP_ERROR_NONE) {
    MOTENV1_PRINTF("\tOK Temperature and Pressure (Sensor2)\n\r");
    TargetBoardFeatures.NumTempSensors++;
  } else {
    MOTENV1_PRINTF("\tError Sensor2 Sensor\n\r");
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

