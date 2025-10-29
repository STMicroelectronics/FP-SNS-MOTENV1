## <b>BootLoader Application Description</b>

Application for NUCLEO-L476RG.

Example Description:

The Example application is a boot loader that allow Full Firmware Update of one program for NUCLEO-L476RG.
Inside this package, this program must be used like Boot Loader for the MOTENV1 application.
If there is Firmware Update, it makes the Update or, if there is not the Firwamre Update, the program works like a trompoline for executing the pre-installed firmware.
A pre-compiled version of this program is placed on Utilities/BootLoader/STM32L476RG/BootLoader_L476.bin

### <b>Keywords</b>

BootLoader

### <b>Hardware and Software environment</b>

  - This example has been tested with STMicroelectronics:
    - NUCLEO-L476RG RevC board

### <b>Very Important</b>

- After code generation from STM32CubeMX software:
  - for IAR and STM32CubeIDE and before building, replace the linker scripts with the scripts in the folder LinkerScript
  - for Keil, open the "Options for Target" and in the tab "Target" set 0x4000 as ROM size. 
- This example must run starting at the beginning of the FLASH 0x08000000 in memory and works in collaboration with another program saved on the FLASH at address 0x08004000

### <b>Known Issues</b>

Compiler warning "BootLoader.elf has a LOAD segment with RWX permissions" is generated with STM32CubeIDE.
It doesn’t affect application performances.

### <b>Dependencies</b>

STM32Cube packages:

  - STM32L4xx drivers from STM32CubeL4 v1.18.1

### <b>How to use it?</b>

This package contains projects for 3 IDEs viz. IAR, Keil µVision 5 and Integrated Development Environment for STM32. 
In order to make the  program work, you must do the following:

 - WARNING: before opening the project with any toolchain be sure your folder
   installation path is not too in-depth since the toolchain may report errors
   after building.

For IAR:

 - Open IAR toolchain (this firmware has been successfully tested with Embedded Workbench V9.60.3).
 - Open the IAR project file EWARM/Project.eww
 - Rebuild all files and load your image into target memory.
 - Run the example.

For Keil µVision 5:

 - Open Keil µVision 5 toolchain (this firmware has been successfully tested with MDK-ARM Professional Version: 5.38.0).
 - Open the µVision project file MDK-ARM/BootLoader.uvprojx
 - Rebuild all files and load your image into target memory.
 - Run the example.
 
For Integrated Development Environment for STM32:

 - Open STM32CubeIDE (this firmware has been successfully tested with Version 1.19.0).
 - Set the default workspace proposed by the IDE (please be sure that there are not spaces in the workspace path).
 - Press "File" -> "Import" -> "Existing Projects into Workspace"; press "Browse" in the "Select root directory" and choose the path where the System
   Workbench project is located (it should be STM32CubeIDE). 
 - Rebuild all files and load your image into target memory.
 - Run the example.

### <b>Author</b>

SRA Application Team

### <b>License</b>

Copyright (c) 2025 STMicroelectronics.
All rights reserved.

This software is licensed under terms that can be found in the LICENSE file
in the root directory of this software component.
If no LICENSE file comes with this software, it is provided AS-IS.