@echo off
set CUBE_PROG="C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set NAME_FW=Release\MOTENV1
set BOOTLOADER="..\..\..\..\..\..\Utilities\BootLoader\STM32F4xxRE\BootLoader_F401.bin"
color 0F
echo                /******************************************/
echo                           Clean FP-SNS-MOTENV1
echo                /******************************************/

echo                              Full Chip Erase
echo                /******************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -e all
echo                /******************************************/
echo                              Install BootLoader
echo                /******************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -d %BOOTLOADER% 0x08000000 -v "after_programming"
echo                /******************************************/
echo                           Install FP-SNS-MOTENV1
echo                /******************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -d %NAME_FW%.bin 0x08004000 -v "after_programming"
echo                /******************************************/
echo                      Dump FP-SNS-MOTENV1 + BootLoader
echo                /******************************************/
set offset_size=0x4000
for %%I in (%NAME_FW%.bin) do set application_size=%%~zI
echo %NAME_FW%.bin size is %application_size% bytes
set /a size=%offset_size%+%application_size%
echo Dumping %offset_size% + %application_size% = %size% bytes ...
echo ..........................
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -u 0x08000000 %size% %NAME_FW%_BL.bin
echo                /******************************************/
echo                                 Reset STM32
echo                /******************************************/
%CUBE_PROG% -c port=swd mode=UR reset=HWrst -rst
if NOT "%1" == "SILENT" pause