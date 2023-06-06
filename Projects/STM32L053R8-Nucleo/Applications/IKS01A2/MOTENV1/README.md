## <b>MOTENV1 Application Description</b>

Application for NUCLEO-L053R8 with X-NUCLEO-BNRG2A1 and X-NUCLEO-IKS01A2 expansion boards.

Example Description:

The MOTENV1 is a sample application which lets you connect your IoT node to a smartphone via BLE and use a suitable Android or iOS like the ST BLE Sensor app to view real-time environmental sensor data and motion sensor data.
 
The Example application initizializes all the Components and Library creating 3 Custom Bluetooth services:

 - The first service exposes:
   - the HW characteristics related to MEMS sensor devices: Temperature, Humidity,
     Pressure, Magnetometer, Gyroscope, Accelleromenter and LED status.
 - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose
 
This example must be used with the related ST BLE Sensor Android (Version 4.17.0 or higher) or iOS (Version 4.17.0 or higher) application available on Play/itune store,
in order to read the sent information by Bluetooth Low Energy protocol.

### <b>Important Hardware Additional Information</b>

BlueNRG-2 library does not work with the stock firmware that is loaded in the BLE module of X-NUCLEO-BNRG2A1 expansion board.
For this reason:

- first of all, it is needed to solder on X-NUCLEO-BNRG2A1, if it is not soldered, a 0 Ohm resistor at R117
- then you can use a standard ST-Link V2-1 with 5 jumper wires female-female together with STSW-BNRGFLASHER software tool
  (currently available only for Windows PC) in order to update the firmware of the BLE module of X-NUCLEO-BNRG2A1.
   
Read user manual for more details.

### <b>Keywords</b>

BLE, BLE_Manager, BlueNRG-2, SPI, I2C, MEMS, IKS01A2

### <b>Hardware and Software environment</b>

  - This example runs on STM32 Nucleo devices with:
    - BlueNRG-2 STM32 expansion board (X-NUCLEO-BNRG2A1)
	- Motion MEMS and environmental sensor expansion board (X-NUCLEO-IKS01A2) for four MEMS sensor devices:
	  - HTS221, LPS22HB, LSM6DSL, LSM303AGR
  - This example has been tested with STMicroelectronics:
    - NUCLEO-L053R8 RevC board
	
ADDITIONAL_BOARD : X-NUCLEO-BNRG2A1 https://www.st.com/en/ecosystems/x-nucleo-bnrg2a1.html

ADDITIONAL_BOARD : X-NUCLEO-IKS01A2 https://www.st.com/en/ecosystems/x-nucleo-iks01a2.html

ADDITIONAL_COMP : BlueNRG-M2SP https://www.st.com/en/wireless-connectivity/bluenrg-2.html

### <b>Known Issues</b>

- Flash Download fails with MDK-ARM Professional Version 5.37.0 on NUCLEO-L053R8 Board
- Due to flash size constraints, initialization control phase via UART is not available for Integrated Development Environment for STM32
- Due to flash size constraints (and in particularfor STM2CubeIDE), after generate the code from STM32CubeMX, in the file iks01a2_conf.h set 0 this define because the related sensor is not used:
  - USE_IKS01A2_MOTION_SENSOR_LSM303AGR_ACC_0
- Due to flash size constraints (and in particularfor STM2CubeIDE), after generate the code from STM32CubeMX, in the file MOTENV1_config.h comment this define:
  - MOTENV1_DEBUG_NOTIFY_TRAMISSION

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L0xx drivers from STM32CubeL0 V1.12.1
  
X-CUBE packages:

  - X-CUBE-BLE2 V3.3.0
  - X-CUBE-BLEMGR V2.0.0
  - X-CUBE-MEMS1 V9.5.0

### <b>How to use it?</b>

This package contains projects for 3 IDEs viz. IAR, Keil µVision 5 and Integrated Development Environment for STM32. 
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.20.1).
 - Open the IAR project file EWARM/Project.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.37.0).
 - Open the µVision project file MDK-ARM/MOTENV1.uvprojx
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.10.1).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be STM32CubeIDE). 
 - Rebuild all files and load your image into target memory.
 - Run the example.

### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2023 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.