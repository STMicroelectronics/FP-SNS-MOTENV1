$ cat CleanMOTENV1.sh
#!/bin/bash

# control section
  : ${1?"Usage $0 BootLoaderPath BinaryPath"}
  : ${2?"Usage $0 BootLoaderPath BinaryPath"}

echo "CleanMOTENV1.sh " $1 $2

######## Modify this Section:
# 1) Set the Installation path for STM32CubeProgrammer
# example:
#CubeProg="/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI"
CubeProg="STM32_Programmer_CLI"

## Run section

echo "/******************************************/"
echo "           Clean FP-SNS-MOTENV1"
echo "/******************************************/"
echo "              Full Chip Erase"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -e all
echo "/******************************************/"
echo "              Install BootLoader"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d $1/BootLoader.bin 0x08000000 -v
echo "/******************************************/"
echo "          Install FP-SNS-MOTENV1"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -d $2/MOTENV1.bin 0x08004000 -v
echo "/******************************************/"
echo "      Dump FP-SNS-MOTENV1 + BootLoader"
echo "/******************************************/"
SizeBinBL=`ls -l $2/MOTENV1.bin | awk '{print $5+0x4000};'`
"${CubeProg}" -c port=swd mode=UR reset=HWrst -u 0x08000000 ${SizeBinBL} $2/MOTENV1_BL.bin
echo "/******************************************/"
echo "                Reset STM32"
echo "/******************************************/"
"${CubeProg}" -c port=swd mode=UR reset=HWrst -rst