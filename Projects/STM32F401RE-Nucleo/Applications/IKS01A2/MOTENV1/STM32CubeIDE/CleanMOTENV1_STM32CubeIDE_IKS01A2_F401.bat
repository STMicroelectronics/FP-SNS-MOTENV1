@echo off
set STLINK_PATH="C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\"
set NAME_FW=Release\STM32F401RE-Nucleo_IKS01A2_MOTENV1
set BOOTLOADER="..\..\..\..\..\..\Utilities\BootLoader\STM32F4xxRE\BootLoaderF401RE-Nucleo.bin"
color 0F
echo                /******************************************/
echo                           Clean FP-SNS-MOTENV1
echo                /******************************************/
echo                              Full Chip Erase
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -c UR -HardRst -ME
echo                /******************************************/
echo                              Install BootLoader
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -P %BOOTLOADER% 0x08000000 -V "after_programming"
echo                /******************************************/
echo                           Install FP-SNS-MOTENV1
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -P %NAME_FW%.bin 0x08004000 -V "after_programming"
echo                /******************************************/
echo                      Dump FP-SNS-MOTENV1 + BootLoader
echo                /******************************************/
set offset_size=0x4000
for %%I in (%NAME_FW%.bin) do set application_size=%%~zI
echo %NAME_FW%.bin size is %application_size% bytes
set /a size=%offset_size%+%application_size%
echo Dumping %offset_size% + %application_size% = %size% bytes ...
echo ..........................
%STLINK_PATH%ST-LINK_CLI.exe -Dump 0x08000000 %size% %NAME_FW%_BL.bin
echo                /******************************************/
echo                                 Reset STM32
echo                /******************************************/
%STLINK_PATH%ST-LINK_CLI.exe -Rst
if NOT "%1" == "SILENT" pause