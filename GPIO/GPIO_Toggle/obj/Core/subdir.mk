################################################################################
# MRS Version: 1.9.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
G:/ESC_V203/V203_bootloader_V11_PA0/SRC/Core/core_riscv.c 

OBJS += \
./Core/core_riscv.o 

C_DEPS += \
./Core/core_riscv.d 


# Each subdirectory must supply rules for building sources it contributes
Core/core_riscv.o: G:/ESC_V203/V203_bootloader_V11_PA0/SRC/Core/core_riscv.c
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -I"G:\ESC_V203\V203_bootloader_V11_PA0\SRC\Debug" -I"G:\ESC_V203\V203_bootloader_V11_PA0\SRC\Core" -I"G:\ESC_V203\V203_bootloader_V11_PA0\GPIO\GPIO_Toggle\User" -I"G:\ESC_V203\V203_bootloader_V11_PA0\SRC\Peripheral\inc" -I"G:\ESC_V203\V203_bootloader_V11_PA0\GPIO\GPIO_Toggle\User\src" -I"G:\ESC_V203\V203_bootloader_V11_PA0\GPIO\GPIO_Toggle\User\inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@	@

