/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hw_advance_features.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   HW Advance Features API prototype
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_ADVANCE_FEATURES_H_
#define _HW_ADVANCE_FEATURES_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "target_features.h"
#include "ble_manager.h"

#include <stdlib.h>

/* Exported functions ------------------------------------------------------- */
extern void InitHWFeatures(void);

extern void DisableHWFeatures(void);

extern void EnableHWPedometer(void);
extern void DisableHWPedometer(void);
extern void ResetHWPedometer(void);
extern uint16_t GetStepHWPedometer(void);

extern void EnableHWFreeFall(void);
extern void DisableHWFreeFall(void);

extern void EnableHWDoubleTap(void);
extern void DisableHWDoubleTap(void);

extern void EnableHWSingleTap(void);
extern void DisableHWSingleTap(void);

extern void EnableHWWakeUp(void);
extern void DisableHWWakeUp(void);

extern void EnableHWTilt(void);
extern void DisableHWTilt(void);

extern void EnableHWOrientation6D(void);
extern void DisableHWOrientation6D(void);
extern acc_event_type_t GetHWOrientation6D(void);

extern void EnableHWMultipleEvents(void);
extern void DisableHWMultipleEvents(void);

/* Exported variables */
extern uint32_t HWAdvanceFeaturesStatus;

/* Exported defines */
#define W2ST_HWF_PEDOMETER        (1   )
#define W2ST_HWF_FREE_FALL        (1<<1)
#define W2ST_HWF_DOUBLE_TAP       (1<<2)
#define W2ST_HWF_SINGLE_TAP       (1<<3)
#define W2ST_HWF_WAKE_UP          (1<<4)
#define W2ST_HWF_TILT             (1<<5)
#define W2ST_HWF_6DORIENTATION    (1<<6)
#define W2ST_HWF_MULTIPLE_EVENTS  (1<<7)

#define W2ST_CHECK_HW_FEATURE(Feature) ((HWAdvanceFeaturesStatus&(Feature)) ? 1 : 0)
#define W2ST_ON_HW_FEATURE(Feature)    (HWAdvanceFeaturesStatus|=(Feature))
#define W2ST_OFF_HW_FEATURE(Feature)   (HWAdvanceFeaturesStatus&=(~Feature))

#ifdef __cplusplus
}
#endif

#endif /* _HW_ADVANCE_FEATURES_H_ */

