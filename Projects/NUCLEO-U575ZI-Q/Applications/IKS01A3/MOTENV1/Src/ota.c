/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ota.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.0.0
  * @date    12-February-2024
  * @brief   Over-the-Air Update API implementation
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

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_motenv1.h"
#include "target_features.h"
#include "ota.h"

/* Local defines -------------------------------------------------------------*/

/* Board FW OTA Position */
/* The 2 addresses are equal due to swap address in dual boot mode */
#define OTA_ADDRESS_START_BANK (FLASH_BASE + FLASH_BANK_SIZE)

/* Uncomment the following define for enabling the PRINTF capability if it's supported */
/* #define OTA_ENABLE_PRINTF */

#ifdef OTA_ENABLE_PRINTF
/* Each application must declare it's printf implementation if it wants to use it */
/* #define OTA_PRINTF Something */
#define OTA_PRINTF MOTENV1_PRINTF
#else /* OTA_ENABLE_PRINTF */
#define OTA_PRINTF(...)
#endif /* OTA_ENABLE_PRINTF */

/* Local Macros -------------------------------------------------------------*/
#define OTA_ERROR_FUNCTION() { while(1);}

/* Private variables ---------------------------------------------------------*/
static uint32_t SizeOfUpdateBlueFW = 0;
static uint32_t AspecteduwCRCValue = 0;
static uint32_t WritingAddress;
static uint32_t BufferValueToWrite[4];
static uint32_t ValuesSavedOnBuffer = 0;
static uint8_t *PointerToBuffer = (uint8_t *) BufferValueToWrite;

/* CRC handler declaration */
CRC_HandleTypeDef   CrcHandle;

/* Exported functions  --------------------------------------------------*/

/**
  * @brief Function for Updating the Firmware
  * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes]
  * @param uint8_t *att_data attribute data
  * @param int32_t data_length length of the data
  * @param uint8_t WriteMagicNum 1/0 for writing or not the magic number
  * @retval int8_t Return value for checking purpouse (1/-1 == Ok/Error)
  */
int8_t UpdateFW(uint32_t *SizeOfUpdate, uint8_t *att_data, int32_t data_length, uint8_t WriteMagicNum)
{
  int8_t ReturnValue = 0;
  /* Save the Packed received */

  if (data_length > (*SizeOfUpdate))
  {
    /* Too many bytes...Something wrong... necessity to send it again... */
    OTA_PRINTF("OTA something wrong data_length=%lu RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n", data_length,
               (*SizeOfUpdate));
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate = 0;
  }
  else
  {
    int32_t Counter;
    int32_t FirstChunk = 0;
    int32_t LastMult16 = 0;
    int32_t Written = 0;
    /* OTA_PRINTF("OTA chunk length=%ld RemSize=%ld\r\n",data_length,(*SizeOfUpdate)); */
    /* Save the received OTA packed ad save it to flash */
    /* Disable instruction cache prior to internal cacheable memory update */
    if (HAL_ICACHE_Disable() != HAL_OK)
    {
      Error_Handler();
    }
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Fill Remaming bytes for reaching 16bytes */
    if (ValuesSavedOnBuffer != 0)
    {
      FirstChunk = (16 - ValuesSavedOnBuffer);
      if (FirstChunk > data_length)
      {
        FirstChunk = data_length;
      }
      memcpy(PointerToBuffer, att_data, FirstChunk);
      Written += FirstChunk;
      ValuesSavedOnBuffer += FirstChunk;

      /* If we have enough data */
      if (ValuesSavedOnBuffer == 16)
      {
        ValuesSavedOnBuffer = 0;
        PointerToBuffer = (uint8_t *) BufferValueToWrite;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, WritingAddress, ((uint32_t)BufferValueToWrite)) == HAL_OK)
        {
          WritingAddress += 16;
        }
        else
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error
             FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
          OTA_ERROR_FUNCTION();
        }
      }
    }

    /* We move at steps of 16 */
    LastMult16 = ((uint32_t)(data_length - FirstChunk)) & (~(((uint32_t)0xF)));
    for (Counter = FirstChunk; Counter < LastMult16; Counter += 16)
    {
      memcpy(PointerToBuffer, att_data + Counter, 16);
      Written += 16;
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, WritingAddress, ((uint32_t)BufferValueToWrite)) == HAL_OK)
      {
        WritingAddress += 16;
      }
      else
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        OTA_ERROR_FUNCTION();
      }
    }

    /* Last Section of this chunk */
    if (Written < data_length)
    {
      memcpy(PointerToBuffer, att_data + Written, (data_length - Written));
      ValuesSavedOnBuffer += (data_length - Written);
      PointerToBuffer += (data_length - Written);
    }

    /* Reduce the remaining bytes for OTA completion */
    *SizeOfUpdate -= data_length;

    if (*SizeOfUpdate == 0)
    {
      /* Check if we need to dump the last bytes */
      if (ValuesSavedOnBuffer != 0)
      {
        ValuesSavedOnBuffer = 0;
        PointerToBuffer = (uint8_t *) BufferValueToWrite;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD, WritingAddress, ((uint32_t)BufferValueToWrite)) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error
             FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
          OTA_ERROR_FUNCTION();
        }
      }

      /* We had received the whole firmware and we have saved it in Flash */
      OTA_PRINTF("OTA Update saved\r\n");

      if (WriteMagicNum)
      {
        uint32_t uwCRCValue = 0;

        /* Make the CRC integrity check */
        if (AspecteduwCRCValue)
        {
          /* Init CRC for OTA-integrity check */
          CrcHandle.Instance = CRC;
          /* The default polynomial is used */
          CrcHandle.Init.DefaultPolynomialUse    = DEFAULT_POLYNOMIAL_ENABLE;

          /* The default init value is used */
          CrcHandle.Init.DefaultInitValueUse     = DEFAULT_INIT_VALUE_ENABLE;

          /* The input data are not inverted */
          CrcHandle.Init.InputDataInversionMode  = CRC_INPUTDATA_INVERSION_NONE;

          /* The output data are not inverted */
          CrcHandle.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;

          /*  32-bit CRC length */
          CrcHandle.Init.CRCLength = CRC_POLYLENGTH_32B;

          /* The input data are 32-bit long words */
          CrcHandle.InputDataFormat              = CRC_INPUTDATA_FORMAT_WORDS;

          if (HAL_CRC_GetState(&CrcHandle) != HAL_CRC_STATE_RESET)
          {
            HAL_CRC_DeInit(&CrcHandle);
          }

          if (HAL_CRC_Init(&CrcHandle) != HAL_OK)
          {
            /* Initialization Error */
            OTA_ERROR_FUNCTION();
          }
          else
          {
            OTA_PRINTF("CRC  Initialized\n\r");
          }
          /* Compute the CRC */
          uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OTA_ADDRESS_START_BANK, SizeOfUpdateBlueFW >> 2);

          if (uwCRCValue == AspecteduwCRCValue)
          {
            ReturnValue = 1;
            OTA_PRINTF("OTA CRC-checked\r\n");
          }
          else
          {
            OTA_PRINTF("OTA Error CRC-checking\r\n");
          }
        }
        else
        {
          ReturnValue = 1;

        }
        if (ReturnValue != 1)
        {
          ReturnValue = -1;
          if (AspecteduwCRCValue)
          {
            OTA_PRINTF("Wrong CRC! Computed=%lx  expected=%lx ... Try again\r\n", uwCRCValue, AspecteduwCRCValue);
          }
        }
      }
    }

    /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
    HAL_FLASH_Lock();

    /* Re-enable instruction cache */
    if (HAL_ICACHE_Enable() != HAL_OK)
    {
      Error_Handler();
    }
  }
  return ReturnValue;
}

/**
  * @brief Start Function for Updating the Firmware
  * @param uint32_t SizeOfUpdate  size of the firmware image [bytes]
  * @param uint32_t uwCRCValue expected CRV value
  * @retval None
  */
void StartUpdateFW(uint32_t SizeOfUpdate, uint32_t uwCRCValue)
{
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  OTA_PRINTF("Start FLASH Erase\r\n");

  SizeOfUpdateBlueFW = SizeOfUpdate;
  AspecteduwCRCValue = uwCRCValue;
  ValuesSavedOnBuffer = 0;
  PointerToBuffer = (uint8_t *) BufferValueToWrite;

  WritingAddress = OTA_ADDRESS_START_BANK;

  if (CurrentActiveBank == 1)
  {
    EraseInitStruct.Banks       = FLASH_BANK_2;
  }
  else
  {
    EraseInitStruct.Banks       = FLASH_BANK_1;
  }

  EraseInitStruct.Page        = 0;
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.NbPages     = (SizeOfUpdate + 16 + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  /* Disable instruction cache prior to internal cacheable memory update */
  if (HAL_ICACHE_Disable() != HAL_OK)
  {
    Error_Handler();
  }

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  {
    /* Error occurred while sector erase.
      User can add here some code to deal with this error.
      SectorError will contain the faulty sector and then to know the code error on this sector,
      user can call function 'HAL_FLASH_GetError()'
      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
    OTA_ERROR_FUNCTION();
  }
  else
  {
    OTA_PRINTF("End FLASH Erase %lu Pages of %dBytes\r\n", EraseInitStruct.NbPages, FLASH_PAGE_SIZE);
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();

  /* Re-enable instruction cache */
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
}
