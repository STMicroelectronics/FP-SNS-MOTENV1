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

#include "target_features.h"
#include "ota.h"

/* Local types ---------------------------------------------------------------*/
typedef struct
{
  uint32_t Version;
  uint32_t MagicNum;
  uint32_t FlashDim;
  uint32_t OTAStartAdd;
  uint32_t OTADoneAdd;
  uint32_t OTAMaxSize;
  uint32_t ProgStartAdd;
  uint32_t IDE;
} BootLoaderFeatures_t;

/* Local defines -------------------------------------------------------------*/

/* Compliant BootLoader version */
#define BL_VERSION_MAJOR 3
#define BL_VERSION_MINOR 0
#define BL_VERSION_PATCH 0

/* Board  FW OTA Magic Number Position */
#define OTA_MAGIC_NUM_POS  (FLASH_BASE + FLASH_FOTA_END)

/* Board  FW OTA DONE Magic Number Position */
#define OTA_MAGIC_DONE_NUM_POS  (OTA_MAGIC_NUM_POS + 0x8)

/* Board/BlueNRG FW OTA Position */
#define OTA_ADDRESS_START  (OTA_MAGIC_NUM_POS + 0x10)

/* Board  FW OTA Magic Number */
#define OTA_MAGIC_NUM 0xDEADBEEF

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
static uint32_t WritingAddress = OTA_ADDRESS_START;
static BootLoaderFeatures_t *BootLoaderFeatures = (BootLoaderFeatures_t *)0x08003F00;
static uint64_t OTA_Buffer;
static uint32_t ValuesSavedOnBuffer = 0;
static uint8_t *PointerToBuffer = (uint8_t *) &OTA_Buffer;

/* Local function prototypes --------------------------------------------------*/
static uint32_t GetPage(uint32_t Address);
static uint32_t GetBank(uint32_t Address);

/* Exported functions  --------------------------------------------------*/
/**
  * @brief Function for Testing the BootLoader Compliance
  * @param None
  * @retval int8_t Return value for checking purpouse (0/-1 == Ok/Error)
  */
int8_t CheckBootLoaderCompliance(void)
{
  uint8_t ret = 1;

  OTA_PRINTF("Testing BootLoaderCompliance:\r\n");
  OTA_PRINTF("\tFirmware MCU STM32L476RGTX:\r\n");
  if ((FLASH_END - FLASH_BASE) != BootLoaderFeatures->FlashDim)
  {
    OTA_PRINTF("\t\tBootLoader and firmware was have build with different MCUs\r\n");
    OTA_PRINTF("\t\tFlash Dim Firmware (%ld Kbyte) != Flash Dim BootLoader (%ld Kbyte)\r\n",
               (((FLASH_END - FLASH_BASE) + 0x1) / 1024), (BootLoaderFeatures->FlashDim / 1024));
    OTA_PRINTF("\t\tMCUs check: KO\r\n");
    ret = 0;
  }
  else
  {
    OTA_PRINTF("\t\tFlash Dim=\t%ld Kbyte\r\n", (((FLASH_END - FLASH_BASE) + 0x1) / 1024));
    OTA_PRINTF("\t\tOTA Max Size=\t%ld Kbyte\r\n", ((BootLoaderFeatures->OTAMaxSize) / 1024));
    OTA_PRINTF("\t\tFirmware Start=\t0x%lx\r\n", BootLoaderFeatures->ProgStartAdd);
    OTA_PRINTF("\t\tOTA Start=\t0x%lx\r\n", BootLoaderFeatures->OTAStartAdd);
    OTA_PRINTF("\t\tMCUs check: OK\r\n");
  }

  OTA_PRINTF("\tBootLoader\r\n");
  OTA_PRINTF("\t\tVersion BL\t%ld.%ld.%ld ",
             BootLoaderFeatures->Version >> 16,
             (BootLoaderFeatures->Version >> 8) & 0xFF,
             BootLoaderFeatures->Version    & 0xFF);

  if (BootLoaderFeatures->IDE == 0)
  {
    OTA_PRINTF("(IAR)");
  }

  if (BootLoaderFeatures->IDE == 1)
  {
    OTA_PRINTF("(KEIL)");
  }

  if (BootLoaderFeatures->IDE == 2)
  {
    OTA_PRINTF("(STM32CubeIDE)");
  }

  OTA_PRINTF("\r\n");

  OTA_PRINTF("\t\tVersion Used\t%d.%d.%d\r\n", BL_VERSION_MAJOR, BL_VERSION_MINOR, BL_VERSION_PATCH);

  if (((BootLoaderFeatures->Version >> 16) != BL_VERSION_MAJOR) |
      (((BootLoaderFeatures->Version >> 8) & 0xFF) != BL_VERSION_MINOR) |
      ((BootLoaderFeatures->Version     & 0xFF) != BL_VERSION_PATCH))
  {
    OTA_PRINTF("\t\tCheck Version\tKO\r\n");
    ret = 0;
  }
  else
  {
    OTA_PRINTF("\t\tCheck Version\tOk\r\n");
  }

  if (BootLoaderFeatures->MagicNum == OTA_MAGIC_NUM)
  {
    OTA_PRINTF("\tMagicNum    OK\r\n");
  }
  else
  {
    OTA_PRINTF("\tMagicNum    KO\r\n");
    ret = 0;
  }

  if (BootLoaderFeatures->OTAStartAdd == (OTA_ADDRESS_START - 0x10))
  {
    OTA_PRINTF("\tOTAStartAdd OK\r\n");
  }
  else
  {
    OTA_PRINTF("\tOTAStartAdd KO\r\n");
    ret = 0;
  }

  if (BootLoaderFeatures->OTADoneAdd == OTA_MAGIC_DONE_NUM_POS)
  {
    OTA_PRINTF("\tOTADoneAdd  OK\r\n");
  }
  else
  {
    OTA_PRINTF("\tOTADoneAdd  KO\r\n");
    ret = 0;
  }

  return ret;
}

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
  /* OTA_PRINTF("What UpdateFW receives SizeOfUpdateBlueFW=%d InSizeOfUpdate=%d length=%d\r\n",
                 SizeOfUpdateBlueFW, *SizeOfUpdate, data_length); */
  if (data_length > (*SizeOfUpdate))
  {
    /* Too many bytes...Something wrong... necessity to send it again... */
    OTA_PRINTF("OTA something wrong data_length=%ld RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n", data_length,
               (*SizeOfUpdate));
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate = 0;
  }
  else
  {
    int32_t Counter;
    int32_t FirstChunk = 0;
    int32_t LastMult8 = 0;
    int32_t Written = 0;

    /* OTA_PRINTF("OTA chunk length=%ld RemSize=%ld\r\n",data_length,(*SizeOfUpdate)); */

    /* Save the received OTA packed ad save it to flash */

    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    /* Fill Remaming bytes for reaching 16bytes */
    if (ValuesSavedOnBuffer != 0)
    {
      FirstChunk = (8 - ValuesSavedOnBuffer);
      if (FirstChunk > data_length)
      {
        FirstChunk = data_length;
      }
      memcpy(PointerToBuffer, att_data, FirstChunk);
      Written += FirstChunk;
      ValuesSavedOnBuffer += FirstChunk;

      /* If we have enough data */
      if (ValuesSavedOnBuffer == 8)
      {
        ValuesSavedOnBuffer = 0;
        PointerToBuffer = (uint8_t *) &OTA_Buffer;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, OTA_Buffer) == HAL_OK)
        {
          WritingAddress += 8;
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

    /* We move at steps of 8 */
    LastMult8 = ((uint32_t)(data_length - FirstChunk)) & (~(((uint32_t)0x7)));
    for (Counter = FirstChunk; Counter < LastMult8; Counter += 8)
    {
      memcpy(PointerToBuffer, att_data + Counter, 8);
      Written += 8;
      if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, OTA_Buffer) == HAL_OK)
      {
        WritingAddress += 8;
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
        PointerToBuffer = (uint8_t *) &OTA_Buffer;
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, OTA_Buffer) != HAL_OK)
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

        if (AspecteduwCRCValue)
        {
          /* Make the CRC integrity check */
          /* CRC handler declaration */
          CRC_HandleTypeDef   CrcHandle;

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
          uwCRCValue = HAL_CRC_Calculate(&CrcHandle, (uint32_t *)OTA_ADDRESS_START, SizeOfUpdateBlueFW >> 2);

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
        if (ReturnValue == 1)
        {
          /* We write the Magic number for making the OTA at the next Board reset and the size of Update*/
          WritingAddress = OTA_MAGIC_NUM_POS;
          OTA_Buffer = (((uint64_t)SizeOfUpdateBlueFW) << 32) | (OTA_MAGIC_NUM);

          if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, OTA_Buffer) != HAL_OK)
          {
            /* Error occurred while writing data in Flash memory.
               User can add here some code to deal with this error
               FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
            OTA_ERROR_FUNCTION();
          }
          else
          {
            WritingAddress = OTA_MAGIC_NUM_POS + 8;
            /* Destination WritingAddress and HeaderSize==0 */
            OTA_Buffer = ((((uint64_t)(BootLoaderFeatures->ProgStartAdd)) << 32) | (0x00));

            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WritingAddress, OTA_Buffer) != HAL_OK)
            {
              /* Error occurred while writing data in Flash memory.
                User can add here some code to deal with this error
                FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
              OTA_ERROR_FUNCTION();
            }
            else
            {
              OTA_PRINTF("OTA will be installed at next board reset\r\n");
            }
          }
        }
        else
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
  WritingAddress = OTA_ADDRESS_START;
  ValuesSavedOnBuffer = 0;
  PointerToBuffer = (uint8_t *) &OTA_Buffer;

  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = GetBank(OTA_MAGIC_NUM_POS);
  EraseInitStruct.Page        = GetPage(OTA_MAGIC_NUM_POS);
  EraseInitStruct.NbPages     = (SizeOfUpdate + 16 + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

#ifdef STM32L4R9xx
  /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
  /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
  if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0)
  {
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
  }
#endif /* STM32L4R9xx */

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
    OTA_PRINTF("End FLASH Erase %ld Pages of %d KB\r\n", EraseInitStruct.NbPages, FLASH_PAGE_SIZE / 1024);
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}

/* Local functions  --------------------------------------------------*/
/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
  {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  }
  else
  {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}

/**
  * @brief  Gets the bank of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The bank of a given address
  */
static uint32_t GetBank(uint32_t Addr)
{
  uint32_t bank = 0;

  if (READ_BIT(SYSCFG->MEMRMP, SYSCFG_MEMRMP_FB_MODE) == 0)
  {
    /* No Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_1;
    }
    else
    {
      bank = FLASH_BANK_2;
    }
  }
  else
  {
    /* Bank swap */
    if (Addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
      bank = FLASH_BANK_2;
    }
    else
    {
      bank = FLASH_BANK_1;
    }
  }

  return bank;
}

