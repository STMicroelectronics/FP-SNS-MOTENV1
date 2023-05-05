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

/* Maximun dimension for the firmware binary for the FOTA */
#define MAX_PROG_SIZE (MIDDLE_FLASH_DIM - 0x4000 - 0x10)

/* OTA Position - Board  FW OTA Magic Number Position */
#define OTA_ADDRESS_START (FLASH_BASE + MIDDLE_FLASH_DIM)

/* Board  FW OTA DONE Magic Number Position */
#define OTA_MAGIC_DONE_NUM_POS  OTA_ADDRESS_START + 0x8

/* remove only the Magic Number (2bytes) */
#define OTA_NUM_PAGES  1

/* Running program */
#define PROG_NUM_PAGES (MAX_PROG_SIZE / FLASH_PAGE_SIZE)

/* Private variables ---------------------------------------------------------*/
#if defined (__IAR_SYSTEMS_ICC__)
#pragma location=".version"
__root const BootLoaderFeatures_t BootLoaderFeatures={((BL_VERSION_MAJOR<<16) | (BL_VERSION_MINOR<<8) | BL_VERSION_PATCH),
                                                      OTA_MAGIC_NUM,
                                                      (FLASH_END - FLASH_BASE),
                                                      OTA_ADDRESS_START,
                                                      OTA_MAGIC_DONE_NUM_POS,
                                                      MAX_PROG_SIZE,
                                                      PROG_ADDRESS_START,
                                                      0};
#elif defined (__CC_ARM)
const BootLoaderFeatures_t BootLoaderFeatures __attribute__( (at(0x08003F00) ) ) ={((BL_VERSION_MAJOR<<16) | (BL_VERSION_MINOR<<8) | BL_VERSION_PATCH),
                                                      OTA_MAGIC_NUM,
                                                      (FLASH_END - FLASH_BASE),
                                                      OTA_ADDRESS_START,
                                                      OTA_MAGIC_DONE_NUM_POS,
                                                      MAX_PROG_SIZE,
                                                      PROG_ADDRESS_START,
                                                      1};
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050) /* For ARM Compiler 5 and 6 */
const BootLoaderFeatures_t BootLoaderFeatures __attribute__( (section(".ARM.__at_0x08003F00") ) ) ={((BL_VERSION_MAJOR<<16) | (BL_VERSION_MINOR<<8) | BL_VERSION_PATCH),
                                                      OTA_MAGIC_NUM,
                                                      (FLASH_END - FLASH_BASE),
                                                      OTA_ADDRESS_START,
                                                      OTA_MAGIC_DONE_NUM_POS,
                                                      MAX_PROG_SIZE,
                                                      PROG_ADDRESS_START,
                                                      1};
#elif defined (__GNUC__)
const BootLoaderFeatures_t __attribute__( (section (".bootinfo") ) ) BootLoaderFeatures ={((BL_VERSION_MAJOR<<16) | (BL_VERSION_MINOR<<8) | BL_VERSION_PATCH),
                                                      OTA_MAGIC_NUM,
                                                      (FLASH_END - FLASH_BASE),
                                                      OTA_ADDRESS_START,
                                                      OTA_MAGIC_DONE_NUM_POS,
                                                      MAX_PROG_SIZE,
                                                      PROG_ADDRESS_START,
                                                      2};

#endif

/* Imported function prototypes ----------------------------------------------*/
extern void SystemClock_Config(void);

/* Private function prototypes -----------------------------------------------*/
static void BootLoader(void);
static uint32_t GetPage(uint32_t Addr);

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
  uint32_t SourceAddress = OTA_ADDRESS_START;
  uint32_t data32 = *(uint32_t*) SourceAddress;

  /* Check if there is Full/Partial Firmware Update */
  if(data32==OTA_MAGIC_NUM){
    /* Make the Firmware Update*/

    /* Update Size */
    uint32_t SizeOfUpdate;

    /* Firmware Update Header Size */
    uint32_t FwHeaderSize;

    /* First Destination Addres to change */
    uint32_t FwDestAddress;

    /* This is the last Address to change */
    uint32_t LastFwDestAddress;

    uint32_t CurrentPageAddress;

    /* TmpBuffer for storing one Flash Page :
     * 0x1000 == 4096 for L4R9
     * 0x800  == 2048 for L47x */
    uint64_t FlashPageBuffer[FLASH_PAGE_SIZE>>3];
    uint8_t *FlashPageBuffer8Bit = (uint8_t *)FlashPageBuffer;

    /* Configure the System clock */
    SystemClock_Config();

    /* Update Size */
    SizeOfUpdate = *(uint32_t*) (SourceAddress+4);

    /* Dimension of Update header */
    FwHeaderSize = *(uint32_t*) (SourceAddress+8);

    /* Address of the Flash region that we need to update */
    FwDestAddress = *(uint32_t*) (SourceAddress+12);

    if(SizeOfUpdate==0) {
      /* If there is not the dimension of the Update... we set the Full program Size */
      SizeOfUpdate = MAX_PROG_SIZE;
    } else {
      /* This is the Real Size the Partial Firmware Update */
      SizeOfUpdate -=FwHeaderSize;
    }

    if(FwDestAddress==PROG_ADDRESS_START) {
      /* if the Size of Update is not present */
      if(SizeOfUpdate==0x0) {
        SizeOfUpdate  = MAX_PROG_SIZE;
      }
    }

    /* Last Destination Address that we need to change */
    LastFwDestAddress = FwDestAddress + SizeOfUpdate;

    /* Source Address */
    SourceAddress +=16+FwHeaderSize;

    /* Loop Cycle for writing the Firmare Update  */
    while(FwDestAddress<LastFwDestAddress) {
      uint32_t Counter;
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      /* These are the Address of the Current Flash page */
      CurrentPageAddress = FwDestAddress  & (~(FLASH_PAGE_SIZE-1));

      /******************* 1) Copy Data in memory *****************************/
      /* Copy the Content of the Flash page in the Tmp Buffer */
      for(Counter=0; Counter<(FLASH_PAGE_SIZE);Counter++) {
        FlashPageBuffer8Bit[Counter] =  *(((uint8_t*) CurrentPageAddress) + Counter);
      }

      /*********************** 2) Erase Flash *********************************/
      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.Banks       = FLASH_BANK_1;
      EraseInitStruct.Page        = GetPage(FwDestAddress);

      EraseInitStruct.NbPages = 1;

      /* Unlock the Flash */
      HAL_FLASH_Unlock();
#ifdef STM32L4R9xx
       /* Clear OPTVERR bit set on virgin samples */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
      /* Clear PEMPTY bit set (as the code is executed from Flash which is not empty) */
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
      }
#endif /* STM32L4R9xx */

      if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
        /* Error occurred during erase section */
        //uint32_t errorcode = HAL_FLASH_GetError();
        while(1);
      }

      /*********************** 3) Update Data in memory ***********************/

      Counter = FwDestAddress & (FLASH_PAGE_SIZE-1);
      for(;((Counter<(FLASH_PAGE_SIZE)) & (FwDestAddress<LastFwDestAddress)); Counter++) {
        FlashPageBuffer8Bit[Counter] = *((uint8_t *) (SourceAddress));
        FwDestAddress++;
        SourceAddress++;
      }

      /*********************** 4) Write Data in Flash**************************/

      for(Counter=0; Counter<(FLASH_PAGE_SIZE);Counter+=8) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, CurrentPageAddress+Counter,FlashPageBuffer[Counter>>3]) != HAL_OK){
          /* Error occurred during writing */
          //uint32_t errorcode = HAL_FLASH_GetError();
          while(1);
        }
      }

      /* Lock the Flash */
      HAL_FLASH_Lock();
    }

    /******************* Delete/Add Magic Number ***************************/
    {
      FLASH_EraseInitTypeDef EraseInitStruct;
      uint32_t SectorError = 0;

      /* Unlock the Flash */
      HAL_FLASH_Unlock();

      /* Reset the Second half Flash */
      EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
      EraseInitStruct.Banks       = FLASH_BANK_2;
      EraseInitStruct.Page        = GetPage(OTA_ADDRESS_START);
      EraseInitStruct.NbPages     = 1;
#ifdef STM32L4R9xx
      /* Clear OPTVERR bit set on virgin samples */
      __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
      /* Clear PEMPTY bit */
      if (__HAL_FLASH_GET_FLAG(FLASH_FLAG_PEMPTY) != 0) {
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_PEMPTY);
      }
#endif /* STM32L4R9xx */

      /* Delete the Magic Number Used for FOTA */
      if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        /* Error occurred during erase section */
        //uint32_t errorcode = HAL_FLASH_GetError();
        while(1);
      }

      /* Lock the Flash */
      HAL_FLASH_Lock();
    }

    /* System Reboot */
    HAL_NVIC_SystemReset();
  } else {
    /* Jump To Normal boot */
    typedef  void (*pFunction)(void);

    pFunction JumpToApplication;
    uint32_t JumpAddress;

    /* reset all interrupts to default */
    // __disable_irq();

    /* Jump to system memory */
    JumpAddress = *(__IO uint32_t*) (PROG_ADDRESS_START + 4);
    JumpToApplication = (pFunction) JumpAddress;
    /* Initialize user application's Stack Pointer */
    __set_MSP(*(__IO uint32_t*) PROG_ADDRESS_START);
    JumpToApplication();
  }
}

/**
  * @brief  Gets the page of a given address
  * @param  Addr: Address of the FLASH Memory
  * @retval The page of a given address
  */
static uint32_t GetPage(uint32_t Addr)
{
  uint32_t page = 0;

  if (Addr < (FLASH_BASE + FLASH_BANK_SIZE)) {
    /* Bank 1 */
    page = (Addr - FLASH_BASE) / FLASH_PAGE_SIZE;
  } else {
    /* Bank 2 */
    page = (Addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
  }

  return page;
}
