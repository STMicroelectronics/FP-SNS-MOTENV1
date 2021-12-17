/**
  ******************************************************************************
  * @file    BLE_FFT_AlarmSpeedStatus.h
  * @author  System Research & Applications Team - Agrate/Catania Lab.
  * @version 1.0.0
  * @date    18-Nov-2021
  * @brief   FFT Alarm Speed Status info services APIs.
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

/* Define to prevent recursive inclusion -------------------------------------*/  
#ifndef _BLE_FFT_ALARM_SPEED_STATUS_H_
#define _BLE_FFT_ALARM_SPEED_STATUS_H_

#ifdef __cplusplus
 extern "C" {
#endif 

/* Exported Types ----------------------------------------------------------- */
/**
* @brief  X-Y-Z Generic Value in float
*/
typedef struct
{
  float x;         //!< Generic X Value in float
  float y;         //!< Generic Y Value in float
  float z;         //!< Generic Z Value in float
} BLE_MANAGER_FFTAlarmSpeedStatusGenericValue_t;

/**
 * @brief  Warning Alarm Datatype
 */
typedef enum
{
  BLE_GOOD_SPEED_STATUS         = (uint8_t)0x00,  //!< GOOD STATUS
  BLE_WARNING_SPEED_STATUS      = (uint8_t)0x01,  //!< WARNING STATUS
  BLE_ALARM_SPEED_STATUS        = (uint8_t)0x02,  //!< ALARM STATUS
  BLE_NONE_SPEED_STATUS         = (uint8_t)0x03,  //!< RFU STATUS
} BLE_Manager_FFTAlarmSpeedStatusAlarmType_t;

/**
 * @brief  STATUS for FFT Speed Warning-Alarm
 */
typedef struct 
{
  BLE_Manager_FFTAlarmSpeedStatusAlarmType_t STATUS_AXIS_X;   //!< X STATUS ALARM
  BLE_Manager_FFTAlarmSpeedStatusAlarmType_t STATUS_AXIS_Y;   //!< Y STATUS ALARM
  BLE_Manager_FFTAlarmSpeedStatusAlarmType_t STATUS_AXIS_Z;   //!< Z STATUS ALARM
} sBLE_Manager_FFTAlarmSpeedStatusAlarm_t;

/* Exported Variables ------------------------------------------------------- */
extern BLE_NotifyEnv_t BLE_FFTAlarmSpeedStatus_NotifyEvent;

/* Exported functions ------------------------------------------------------- */

/**
 * @brief  Init FFT Alarm Speed Status info service
 * @param  None
 * @retval BleCharTypeDef* BleCharPointer: Data structure pointer for FFT Alarm Speed Status info service
 */
BleCharTypeDef* BLE_InitFFTAlarmSpeedStatusService(void);

#ifndef BLE_MANAGER_SDKV2
/**
 * @brief  Setting FFT Alarm Speed Status Advertize Data
 * @param  uint8_t *manuf_data: Advertize Data
 * @retval None
 */
void BLE_SetFFTAlarmSpeedStatusAdvertizeData(uint8_t *manuf_data);
#endif /* BLE_MANAGER_SDKV2 */

/*
 * @brief  Update FFT Alarm Speed RMS status value
 * @param  sBLE_Manager_FFTAlarmSpeedStatusAlarm_t              Alarm
 * @param  BLE_MANAGER_FFTAlarmSpeedStatusGenericValue_t        SpeedRmsValue
 * @retval tBleStatus   Status
 */
tBleStatus BLE_FFTAlarmSpeedStatusUpdate(sBLE_Manager_FFTAlarmSpeedStatusAlarm_t Alarm, BLE_MANAGER_FFTAlarmSpeedStatusGenericValue_t SpeedRmsValue);

#ifdef __cplusplus
}
#endif

#endif /* _BLE_FFT_ALARM_SPEED_STATUS_H_ */

