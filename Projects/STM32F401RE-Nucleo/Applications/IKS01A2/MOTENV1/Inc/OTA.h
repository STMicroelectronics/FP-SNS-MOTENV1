/**
  ******************************************************************************
  * @file    OTA.h 
  * @author  System Research & Applications Team - Catania Lab.
  * @version V4.2.0
  * @date    03-Nov-2021
  * @brief   Over-the-Air Update API
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
#ifndef _OTA_H_
#define _OTA_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported defines ---------------------------------------------------------*/


#ifdef USE_STM32F4XX_NUCLEO
  /* 240Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x40000-0x4000-8)
#endif /* USE_STM32F4XX_NUCLEO */
   
#ifdef USE_STM32L4XX_NUCLEO
  /* 496Kbytes Max Program Size */
  #define OTA_MAX_PROG_SIZE (0x80000-0x4000-8)
#endif /* USE_STM32L4XX_NUCLEO */

/* Exported functions ---------------------------------------------------------*/

/* API for preparing the Flash for receiving the Update. It defines also the Size of the Update and the CRC value aspected */
extern void StartUpdateFWBlueMS(uint32_t SizeOfUpdate,uint32_t uwCRCValue);
/* API for storing chuck of data to Flash.
 * When it has received the total number of byte defined by StartUpdateFWBlueMS,
 * it computes the CRC value and if it matches the aspected CRC value,
 * it writes the Magic Number in Flash for BootLoader */
extern int8_t UpdateFWBlueMS(uint32_t *SizeOfUpdate,uint8_t * att_data, int32_t data_length,uint8_t WriteMagicNum);

/* API for checking the BootLoader compliance */
extern int8_t CheckBootLoaderCompliance(void);

#ifdef __cplusplus
}
#endif

#endif /* _OTA_H_ */

