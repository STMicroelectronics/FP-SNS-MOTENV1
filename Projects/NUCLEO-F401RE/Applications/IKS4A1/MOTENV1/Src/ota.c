/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ota.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 5.1.0
  * @date    12-September-2025
  * @brief   Over-the-Air Update API implementation
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
/*---------------------------------- STM32F401xE/STM32F411xE/STM32F446xx ------------------------------*/
#if defined(STM32F401xE) || defined(STM32F411xE) || defined(STM32F446xx)
#define FLASH_SECTOR_NUMBER FLASH_SECTOR_6
#endif /* STM32F401xE/STM32F411xE/STM32F446xx */
/*-----------------------------------------------------------------------------------------------------*/

/*--------------------------------------- STM32F40xxx/STM32F41xxx -------------------------------------*/
#if defined(STM32F405xx) || defined(STM32F415xx) || defined(STM32F407xx) || defined(STM32F417xx) || \
    defined(STM32F412Zx) || defined(STM32F412Vx) || defined(STM32F412Rx) || defined(STM32F412Cx)
#define FLASH_SECTOR_NUMBER FLASH_SECTOR_8
#endif /* STM32F405xx/STM32F415xx/STM32F407xx/STM32F417xx/STM32F412Zx/STM32F412Vx/STM32F412Rx/STM32F412Cx */
/*-----------------------------------------------------------------------------------------------------*/

/*-------------------------------------- STM32F413xx/STM32F423xx --------------------------------------*/
#if defined(STM32F413xx) || defined(STM32F423xx)
#define FLASH_SECTOR_NUMBER FLASH_SECTOR_10
#endif /* STM32F413xx/STM32F423xx */
/*-----------------------------------------------------------------------------------------------------*/

/*-------------------------------------- STM32F42xxx/STM32F43xxx/STM32F469xx --------------------------*/
#if defined(STM32F427xx) || defined(STM32F437xx) || defined(STM32F429xx)|| defined(STM32F439xx) ||\
    defined(STM32F469xx) || defined(STM32F479xx)
#define FLASH_SECTOR_NUMBER FLASH_SECTOR_12
#endif /* STM32F427xx/STM32F437xx/STM32F429xx/STM32F439xx/STM32F469xx/STM32F479xx */
/*-----------------------------------------------------------------------------------------------------*/

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
  OTA_PRINTF("\tFirmware MCU STM32F401RETX:\r\n");
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
  * @param uint32_t *SizeOfUpdate Remaining size of the firmware image [bytes].
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
    /* OTA_PRINTF("OTA something wrong data_length=%ld RemSizeOfUpdate=%ld....\r\nPlease Try again\r\n",
                 data_length, (*SizeOfUpdate)); */
    ReturnValue = -1;
    /* Reset for Restarting again */
    *SizeOfUpdate = 0;
  }
  else
  {
    int32_t Counter;
    /* Save the received OTA packed ad save it to flash */
    /* Unlock the Flash to enable the flash control register access *************/
    HAL_FLASH_Unlock();

    for (Counter = 0; Counter < data_length; Counter++)
    {
      if (HAL_FLASH_Program(TYPEPROGRAM_BYTE, WritingAddress, att_data[Counter]) == HAL_OK)
      {

        WritingAddress++;
      }
      else
      {
        /* Error occurred while writing data in Flash memory.
           User can add here some code to deal with this error
           FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        OTA_ERROR_FUNCTION();
      }
    }
    /* Reduce the remaining bytes for OTA completion */
    *SizeOfUpdate -= data_length;

    if (*SizeOfUpdate == 0)
    {
      /* We had received the whole firmware and we have saved it in Flash */

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
        }
        else
        {

          ReturnValue = 1;
        }

        if (ReturnValue == 1)
        {
          /* We write the Magic number for making the OTA at the next Board reset */
          WritingAddress = OTA_MAGIC_NUM_POS;

          if (HAL_FLASH_Program(TYPEPROGRAM_WORD, WritingAddress, OTA_MAGIC_NUM) != HAL_OK)
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

  EraseInitStruct.TypeErase = TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = FLASH_SECTOR_NUMBER;
  if (SizeOfUpdate > (0x20000 - 8))
  {
    /* We need 2 sectors of 128KB */
    EraseInitStruct.NbSectors = 2;
  }
  else
  {
    /* One sector of 128KB is enough */
    EraseInitStruct.NbSectors = 1;
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
    OTA_PRINTF("End FLASH Erase %ld Sectors of 128KB\r\n", EraseInitStruct.NbSectors);
  }

  /* Lock the Flash to disable the flash control register access (recommended
  to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock();
}

