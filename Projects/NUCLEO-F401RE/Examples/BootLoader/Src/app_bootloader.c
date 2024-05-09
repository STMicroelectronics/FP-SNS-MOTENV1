/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_bootloader.c
  * @author  System Research & Applications Team - Catania Lab.
  * @version 3.0.0
  * @date    31-January-2023
  * @brief   This file provides code for FP-SNS-MOTENV1 application.
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

/* Includes ------------------------------------------------------------------*/
#include "app_bootloader.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private macro ------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Exported Variables --------------------------------------------------------*/

#include <stdio.h>
#include "main.h"

/* Private typedef -----------------------------------------------------------*/

typedef struct
{
  uint32_t Version;
  uint32_t MagicNum;
  uint32_t FlashDim;
  uint32_t FwUpdateStartAdd;
  uint32_t FwDoneAdd;
  uint32_t FwMaxSize;
  uint32_t ProgStartAdd;
  uint32_t IDE;
} BootLoaderFeatures_t;

/* Private define ------------------------------------------------------------*/
#define BL_VERSION_MAJOR 3
#define BL_VERSION_MINOR 0
#define BL_VERSION_PATCH 0

#define OTA_MAGIC_NUM ((uint32_t)0xDEADBEEF)

/* Running program Position */
#define PROG_ADDRESS_START 0x08004000

/* Middle dimension of the flash memory */
#define MIDDLE_FLASH_DIM (((FLASH_END - FLASH_BASE) + 0x1) / 0x2)

/* Maximum dimension for the firmware binary for the FOTA */
#define MAX_PROG_SIZE (MIDDLE_FLASH_DIM - 0x4000 - 0x10)

/* OTA Position - Board  FW OTA Magic Number Position */
#define OTA_ADDRESS_START (FLASH_BASE + MIDDLE_FLASH_DIM)

/* Board  FW OTA DONE Magic Number Position */
#define OTA_MAGIC_DONE_NUM_POS  OTA_ADDRESS_START + 0x8

#define OTA_NUM_SECTORS_128K  ((MIDDLE_FLASH_DIM / 1024) / 128)
/* OTA Position */
#define OTA_SECTOR_START  (FLASH_SECTOR_TOTAL - OTA_NUM_SECTORS_128K)

#define PROG_NUM_SECTORS (FLASH_SECTOR_TOTAL - OTA_NUM_SECTORS_128K - 1)
/* Running program Position */
#define PROG_SECTOR_START  FLASH_SECTOR_1

/* Private variables ---------------------------------------------------------*/
#if defined (__IAR_SYSTEMS_ICC__)
#pragma location=".version"
__root const BootLoaderFeatures_t BootLoaderFeatures =
{
  ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH),
  OTA_MAGIC_NUM,
  (FLASH_END - FLASH_BASE),
  OTA_ADDRESS_START,
  OTA_MAGIC_DONE_NUM_POS,
  MAX_PROG_SIZE,
  PROG_ADDRESS_START,
  0
};
#elif defined (__CC_ARM)
const BootLoaderFeatures_t BootLoaderFeatures __attribute__((at(0x08003F00))) =
{
  ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH),
  OTA_MAGIC_NUM,
  (FLASH_END - FLASH_BASE),
  OTA_ADDRESS_START,
  OTA_MAGIC_DONE_NUM_POS,
  MAX_PROG_SIZE,
  PROG_ADDRESS_START,
  1
};
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* For ARM Compiler 5 and 6 */
const BootLoaderFeatures_t BootLoaderFeatures __attribute__((section(".ARM.__at_0x08003F00"))) =
{
  ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH),
  OTA_MAGIC_NUM,
  (FLASH_END - FLASH_BASE),
  OTA_ADDRESS_START,
  OTA_MAGIC_DONE_NUM_POS,
  MAX_PROG_SIZE,
  PROG_ADDRESS_START,
  1
};
#elif defined (__GNUC__)
const BootLoaderFeatures_t __attribute__((section(".bootinfo"))) BootLoaderFeatures =
{
  ((BL_VERSION_MAJOR << 16) | (BL_VERSION_MINOR << 8) | BL_VERSION_PATCH),
  OTA_MAGIC_NUM,
  (FLASH_END - FLASH_BASE),
  OTA_ADDRESS_START,
  OTA_MAGIC_DONE_NUM_POS,
  MAX_PROG_SIZE,
  PROG_ADDRESS_START,
  2
};

#endif /* IDE */

/* Imported function prototypes ----------------------------------------------*/
extern void SystemClock_Config(void);

/* Private function prototypes -----------------------------------------------*/
static void BootLoader(void);

void MX_BootLoader_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN BootLoader_Init_PreTreatment */

  /* USER CODE END BootLoader_Init_PreTreatment */

  /* Initialize BOOTLOADER application */

  BootLoader();

  /* USER CODE BEGIN BootLoader_Init_PostTreatment */

  /* USER CODE END BootLoader_Init_PostTreatment */
}

/*
 * FP-SNS-MOTENV1 background task
 */
void MX_BootLoader_Process(void)
{
  /* USER CODE BEGIN BootLoader_Process_PreTreatment */

  /* USER CODE END BootLoader_Process_PreTreatment */

  /* Process of the BOOTLOADER application */

  /* USER CODE BEGIN BootLoader_Process_PostTreatment */

  /* USER CODE END BootLoader_Process_PostTreatment */
}

/**
  * @brief  Boot Loader Procedure
  * @param  None
  * @retval None
  */
static void BootLoader(void)
{
  uint32_t Address = OTA_ADDRESS_START;
  __IO uint32_t data32 = *(__IO uint32_t *) Address;

  /* Check if There is a New OTA */
  if (data32 == OTA_MAGIC_NUM)
  {
    /* Make the OTA */
    /* Configure the System clock */
    SystemClock_Config();

    /* Reset the First Flash Section */
    {
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      EraseInitStruct.TypeErase    = TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
      EraseInitStruct.Sector       = PROG_SECTOR_START;
      EraseInitStruct.NbSectors    = PROG_NUM_SECTORS;

      /* Unlock the Flash to enable the flash control register access *************/
      HAL_FLASH_Unlock();

      if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
      {
        /* Error occurred while sector erase.
          User can add here some code to deal with this error.
          SectorError will contain the faulty sector and then to know the code error on this sector,
          user can call function 'HAL_FLASH_GetError()'
          FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        while (1);
      }

      /* Lock the Flash to disable the flash control register access (recommended
      to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();
    }

    /* Make the OTA */
    {
      int32_t WriteIndex;
      /* + 16 For Skipping the Magic Number (Aligned to 8 for L4)*/
      uint32_t *OTAAddress = (uint32_t *)(OTA_ADDRESS_START + 16);
      uint32_t ProgAddress = (uint32_t) PROG_ADDRESS_START;

      /* Unlock the Flash to enable the flash control register access *************/
      HAL_FLASH_Unlock();

      for (WriteIndex = 0; WriteIndex < MAX_PROG_SIZE; WriteIndex += 4)
      {
        if (HAL_FLASH_Program(TYPEPROGRAM_WORD, ProgAddress + WriteIndex, OTAAddress[WriteIndex >> 2]) != HAL_OK)
        {
          /* Error occurred while writing data in Flash memory.
             User can add here some code to deal with this error
             FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
          while (1);
        }
      }

      /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();
    }

    /* Reset the Second Half of the Flash except where is stored the License Manager*/
    {
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      /* Unlock the Flash to enable the flash control register access *************/
      HAL_FLASH_Unlock();

      /* Reset the Second half Flash */
      EraseInitStruct.TypeErase    = TYPEERASE_SECTORS;
      EraseInitStruct.VoltageRange = VOLTAGE_RANGE_3;
      EraseInitStruct.Sector       = OTA_SECTOR_START;
      EraseInitStruct.NbSectors    = OTA_NUM_SECTORS_128K;

      if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
      {
        /* Error occurred while sector erase.
          User can add here some code to deal with this error.
          SectorError will contain the faulty sector and then to know the code error on this sector,
          user can call function 'HAL_FLASH_GetError()'
          FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError(); */
        while (1);
      }

      /* Lock the Flash to disable the flash control register access (recommended
       to protect the FLASH memory against possible unwanted operation) *********/
      HAL_FLASH_Lock();
    }

    /* System Reboot */
    HAL_NVIC_SystemReset();
  }
  else
  {
    /* Jump To Normal boot */
    typedef  void (*pFunction)(void);

    pFunction JumpToApplication;
    uint32_t JumpAddress;

    /* reset all interrupts to default */
    /* __disable_irq(); */

    /* Jump to system memory */
    JumpAddress = *(__IO uint32_t *)(PROG_ADDRESS_START + 4);
    JumpToApplication = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t *) PROG_ADDRESS_START);
    JumpToApplication();
  }
}
