/**
  ******************************************************************************
  * @file    TemplateFlashManagement.h
  * @author  System Research & Applications Team - Catania Lab.
  * @version 1.5.0
  * @date    18-Nov-2021
  * @brief   Templete File for User specialization of Flash Management
  *          for MetaDataManager
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

#ifndef _TEMPLATE_FLASH_MANAGEMENT_H_
#define _TEMPLATE_FLASH_MANAGEMENT_H_

#ifdef __cplusplus
 extern "C" {
#endif 


/* Includes ------------------------------------------------------------------*/
#include "MetaDataManager.h"

/* Exported function prototypes -----------------------------------------------*/
extern uint32_t UserFunctionForErasingFlash(void);
extern uint32_t UserFunctionForSavingFlash(void *InitMetaDataVector,void *EndMetaDataVector);

#ifdef __cplusplus
}
#endif

#endif /* _TEMPLATE_FLASH_MANAGEMENT_H_ */

