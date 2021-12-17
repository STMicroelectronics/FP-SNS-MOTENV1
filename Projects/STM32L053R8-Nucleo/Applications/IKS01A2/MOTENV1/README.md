# MOTENV1 Application Description for STM32L053R8 with X-NUCLEO-IKS01A2 expansion board

The MOTENV1 is an STM32Cube function pack which lets you connect your IoT node to a smartphone via BLE and use a suitable AndroidT or iOST like the ST BLE Sensor app to view real-time environmental sensor data and motion sensor data.

This firmware package includes Components Device Drivers, Board Support Package and example application for the following STMicroelectronics elements:
 - X-NUCLEO-BNRG2A1 Bluetooth Low energy expansion boards
 - X-NUCLEO-IKS01A2 Expansion board for four MEMS sensor devices:
   - HTS221, LPS22HB, LSM6DSL, LSM303AGR
 - NUCLEO-L053R8 NUCLEO board
 
The Example application initizializes all the Components and Library creating 3 Custom Bluetooth services:
 - The first service exposes:
   - the HW characteristics related to MEMS sensor devices: Temperature, Humidity,
     Pressure, Magnetometer, Gyroscope, Accelleromenter and LED status.
 - The second Service exposes the console services where we have stdin/stdout and stderr capabilities
 - The last Service is used for configuration purpose

The example application allows the user to control the initialization phase via UART.
Launch a terminal application and set the UART port to 115200 bps, 8 bit, No Parity, 1 stop bit.
 
This example must be used with the related ST BLE Sensor Android (Version 4.13.0 or higher) or iOS (Version 4.11.0 or higher) application available on Play/itune store,
in order to read the sent information by Bluetooth Low Energy protocol

## Known Issues

- Flash Download fails with MDK-ARM Professional Version 5.32.0 on NUCLEO-L053R8 Board
- Due to flash size constraints, initialization control phase via UART is not available for Integrated Development Environment for STM32 (STM32CubeIDE)

## Dependencies

STM32Cube packages:
  - STM32L0xx drivers from STM32CubeL0 V1.12.0
  
X-CUBE packages:
  - X-CUBE-BLE2 V3.2.2
  - X-CUBE-MEMS1 V9.1.0

## Hardware and Software environment

- This example runs on Sensor expansion board attached to STM32L053R8 devices can be easily tailored to any other supported device and development board.
- This example must be used with the related ST BLE Sensor Android/iOS application (Version 4.13.0/4.11.0 or higher) available on Play/itune store, in order to read the sent information by Bluetooth Low Energy protocol.
- Inside the Binary Directory there are the following binaries:
  - STM32L053R8-Nucleo_IKS01A2_MOTENV1_v4.2.0.bin

## How to use it ?

This package contains projects for 3 IDEs viz. IAR, Keil µVision 5 and Integrated Development Environment for STM32. 
In order to make the  program work, you must do the following:
 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:
 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V8.50.9).
 - Open the IAR project file EWARM\Project.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:
 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.32.0).
 - Open the µVision project file MDK-ARM\STM32L053R8-Nucleo_IKS01A2_MOTENV1.uvprojx
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
For Integrated Development Environment for STM32:
 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.7.0).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be STM32CubeIDE). 
 - Rebuild all files and load your image into target memory.
 - Run the example.
