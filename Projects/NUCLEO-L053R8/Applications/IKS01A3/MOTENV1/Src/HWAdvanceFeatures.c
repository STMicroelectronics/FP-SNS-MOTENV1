/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    HWAdvanceFeatures.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 4.3.2
  * @date    19-June-2023
  * @brief   HW Advance Features API
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

#include "HWAdvanceFeatures.h"
#include "BLE_Manager.h"
#include "TargetFeatures.h"

/* Imported Variables -------------------------------------------------------------*/
extern uint16_t PedometerStepCount;

/* Exported variables ---------------------------------------------------------*/
uint32_t HWAdvanceFeaturesStatus=0;

/* Private Variables -------------------------------------------------------------*/
static float DefaultAccODR;
static uint8_t MultipleAccEventEnabled= 0;

/**
  * @brief  This function Reads the default Acceleration Output Data Rate
  * @param  None
  * @retval None
  */
void InitHWFeatures(void){
   /* Read the Default Output Data Rate for Accelerometer */
  MOTION_SENSOR_GetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,&DefaultAccODR);
}

/**
  * @brief  This function disables all the HW's Features
  * @param  None
  * @retval None
  */
void DisableHWFeatures(void)
{
  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_PEDOMETER)) {
    DisableHWPedometer();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_FREE_FALL)) {
    DisableHWFreeFall();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_DOUBLE_TAP)) {
    DisableHWDoubleTap();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_SINGLE_TAP)) {
    DisableHWSingleTap();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_WAKE_UP)) {
    DisableHWWakeUp();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_TILT)) {
    DisableHWTilt();
  }

  if(W2ST_CHECK_HW_FEATURE(W2ST_HWF_6DORIENTATION)) {
    DisableHWOrientation6D();
  }
}

/**
  * @brief  This function enables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void EnableHWOrientation6D(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Free Fall detection */
  if(MOTION_SENSOR_Enable_6D_Orientation(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!=BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling 6D Orientation\r\n");
  } else {
    MOTENV1_PRINTF("Enabled 6D Orientation\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }
}

/**
  * @brief  This function disables the HW's 6D Orientation
  * @param  None
  * @retval None
  */
void DisableHWOrientation6D(void)
{
  /* Disable Free Fall detection */
  if(MOTION_SENSOR_Disable_6D_Orientation(ACCELERO_INSTANCE)!=BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling 6D Orientation\r\n");
  } else {
    MOTENV1_PRINTF("Disabled 6D Orientation\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_6DORIENTATION);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO, DefaultAccODR);
}

/**
  * @brief  This function eturns the HW's 6D Orientation result
  * @param  None
  * @retval AccEventType 6D Orientation Found
  */
AccEventType GetHWOrientation6D(void)
{
  AccEventType OrientationResult = ACC_NOT_USED;

  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;

  if (MOTION_SENSOR_Get_6D_Orientation_XL( ACCELERO_INSTANCE, &xl ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation XL axis from LSM6DS3\r\n");
  }

  if (MOTION_SENSOR_Get_6D_Orientation_XH( ACCELERO_INSTANCE, &xh ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation XH axis from LSM6DS3\r\n");
  }

  if (MOTION_SENSOR_Get_6D_Orientation_YL( ACCELERO_INSTANCE, &yl ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation YL axis from LSM6DS3\r\n");
  }

  if (MOTION_SENSOR_Get_6D_Orientation_YH( ACCELERO_INSTANCE, &yh ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation YH axis from LSM6DS3\r\n");
  }

  if (MOTION_SENSOR_Get_6D_Orientation_ZL( ACCELERO_INSTANCE, &zl ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation ZL axis from LSM6DS3\r\n");
  }

  if (MOTION_SENSOR_Get_6D_Orientation_ZH( ACCELERO_INSTANCE, &zh ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error getting 6D orientation ZH axis from LSM6DS3\r\n");
  }

  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_RIGTH;
  } else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_TOP;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_BOTTOM;
  } else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    OrientationResult = ACC_6D_OR_LEFT;
  } else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    OrientationResult = ACC_6D_OR_UP;
  } else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ){
    OrientationResult = ACC_6D_OR_DOWN;
  } else {
    MOTENV1_PRINTF("None of the 6D orientation axes is set in LSM6DS3\r\n");
  }

  return OrientationResult;
}
/**
  * @brief  This function enables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void EnableHWTilt(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Tilt detection */
  if(MOTION_SENSOR_Enable_Tilt_Detection(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!=BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Tilt Detection\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Tilt\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_TILT);
  }
}

/**
  * @brief  This function disables the HW's Tilt Detection
  * @param  None
  * @retval None
  */
void DisableHWTilt(void)
{
  /* Disable Tilt detection */
  if(MOTION_SENSOR_Disable_Tilt_Detection(ACCELERO_INSTANCE)!=BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Tilt Detection\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Tilt\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_TILT);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void EnableHWWakeUp(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Wake up detection */
  if(MOTION_SENSOR_Enable_Wake_Up_Detection(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Wake Up Detection\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Wake Up\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }
}

/**
  * @brief  This function disables the HW's Wake Up Detection
  * @param  None
  * @retval None
  */
void DisableHWWakeUp(void)
{
  /* Disable Wake up detection */
  if(MOTION_SENSOR_Disable_Wake_Up_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Wake Up Detection\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Wake Up\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_WAKE_UP);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void EnableHWFreeFall(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Free Fall detection */
  if(MOTION_SENSOR_Enable_Free_Fall_Detection(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Free Fall Detection\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Free Fall\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_FREE_FALL);
  }
#if 0
  if(MOTION_SENSOR_Set_Free_Fall_Threshold(ACCELERO_INSTANCE,LSM6DSL_FF_TSH_250mg)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error setting Free Fall Treshold\r\n");
  }
#endif
}

/**
  * @brief  This function disables the HW's Free Fall Detection
  * @param  None
  * @retval None
  */
void DisableHWFreeFall(void)
{
  /* Disable Free Fall detection */
  if(MOTION_SENSOR_Disable_Free_Fall_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Free Fall Detection\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Free Fall\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_FREE_FALL);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWDoubleTap(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Double Tap detection */
  if(MOTION_SENSOR_Enable_Double_Tap_Detection(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Double Tap Detection\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Double Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }
}

/**
  * @brief  This function disables the HW's Double Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWDoubleTap(void)
{
  /* Disable Double Tap detection */
  if(MOTION_SENSOR_Disable_Double_Tap_Detection (ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Double Tap Detection\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Double Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_DOUBLE_TAP);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void EnableHWSingleTap(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  /* Enable Single Tap detection */
  if(MOTION_SENSOR_Enable_Single_Tap_Detection(ACCELERO_INSTANCE, MOTION_SENSOR_INT1_PIN)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Single Tap Detection\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Sigle Tap\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }
}

/**
  * @brief  This function disables the HW's Single Tap Detection
  * @param  None
  * @retval None
  */
void DisableHWSingleTap(void)
{
  /* Disable Single Tap detection */
  if(MOTION_SENSOR_Disable_Single_Tap_Detection(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Single Tap Detection\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Sigle Tap\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_SINGLE_TAP);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function enables the HW's pedometer
  * @param  None
  * @retval None
  */
void EnableHWPedometer(void)
{
  /* Disable all the HW features before */
  if(!MultipleAccEventEnabled)
    DisableHWFeatures();

  if(MOTION_SENSOR_Enable_Pedometer(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Enabling Pedometer\r\n");
  } else {
    MOTENV1_PRINTF("Enabled Pedometer\r\n");
    W2ST_ON_HW_FEATURE(W2ST_HWF_PEDOMETER);
  }
}

/**
  * @brief  This function disables the HW's pedometer
  * @param  None
  * @retval None
  */
void DisableHWPedometer(void)
{
  if(MOTION_SENSOR_Disable_Pedometer(ACCELERO_INSTANCE)!= BSP_ERROR_NONE) {
    MOTENV1_PRINTF("Error Disabling Pedometer\r\n");
  } else {
    MOTENV1_PRINTF("Disabled Pedometer\r\n");
    W2ST_OFF_HW_FEATURE(W2ST_HWF_PEDOMETER);
  }

  /* Set the Output Data Rate to Default value */
  MOTION_SENSOR_SetOutputDataRate(ACCELERO_INSTANCE,MOTION_ACCELERO,DefaultAccODR);
}

/**
  * @brief  This function resets the HW's pedometer steps counter
  * @param  None
  * @retval None
  */
void ResetHWPedometer(void)
{
  if (MOTION_SENSOR_Reset_Step_Counter( ACCELERO_INSTANCE ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error Resetting Pedometer's Counter\r\n");
  } else {
    MOTENV1_PRINTF("Reset Pedometer's Counter\r\n");
  }
}

/**
  * @brief  This function retunrs the HW's pedometer steps counter value
  * @param  None
  * @retval uint16_t Steps Counter
  */
uint16_t GetStepHWPedometer(void)
{
  uint16_t step_count=0;

  if(MOTION_SENSOR_Get_Step_Count( ACCELERO_INSTANCE, &step_count ) != BSP_ERROR_NONE ){
    MOTENV1_PRINTF("Error Reading Pedometer's Counter\r\n");
  } else {
    MOTENV1_PRINTF("Pedometer's Counter=%u\r\n",step_count);
  }

  return step_count;
}

/**
  * @brief  This function enables the multiple HW's events
  * @param  None
  * @retval None
  */
void EnableHWMultipleEvents(void)
{
  MOTENV1_PRINTF("EnableHWMultipleEvents\r\n");
  DisableHWFeatures();

  MultipleAccEventEnabled= 1;

  /* Do not change the enable sequenze of the HW events */
  /* It depends on the ODR value (from minor value to max value) */
  EnableHWPedometer();
  EnableHWTilt();
  EnableHWFreeFall();
  EnableHWSingleTap();
  EnableHWDoubleTap();
  EnableHWOrientation6D();

  PedometerStepCount=0;
  BLE_AccEnvUpdate(PedometerStepCount, 3);

  W2ST_ON_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

/**
  * @brief  This function disables the multiple HW's events
  * @param  None
  * @retval None
  */
void DisableHWMultipleEvents(void)
{
  MOTENV1_PRINTF("DisableHWMultipleEvents\r\n");
  DisableHWFeatures();
  MultipleAccEventEnabled= 0;
  W2ST_OFF_HW_FEATURE(W2ST_HWF_MULTIPLE_EVENTS);
}

