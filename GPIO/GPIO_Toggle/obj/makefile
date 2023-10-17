################################################################################
# MRS Version: 1.9.0
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include User/src/subdir.mk
-include User/subdir.mk
-include Startup/subdir.mk
-include Peripheral/src/subdir.mk
-include Debug/subdir.mk
-include Core/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_FLASH += \
V203_bootloader_V11_PA0.hex \

SECONDARY_LIST += \
V203_bootloader_V11_PA0.lst \

SECONDARY_SIZE += \
V203_bootloader_V11_PA0.siz \


# All Target
all: V203_bootloader_V11_PA0.elf secondary-outputs

# Tool invocations
V203_bootloader_V11_PA0.elf: $(OBJS) $(USER_OBJS)
	@	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -Os -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized  -g -T "G:\ESC_V203\V203_bootloader_V11_PA0\SRC\Ld\Link.ld" -nostartfiles -Xlinker --gc-sections -Wl,-Map,"V203_bootloader_V11_PA0.map" --specs=nano.specs --specs=nosys.specs -o "V203_bootloader_V11_PA0.elf" $(OBJS) $(USER_OBJS) $(LIBS) -lprintf
	@	@
V203_bootloader_V11_PA0.hex: V203_bootloader_V11_PA0.elf
	@	riscv-none-embed-objcopy -O ihex "V203_bootloader_V11_PA0.elf"  "V203_bootloader_V11_PA0.hex"
	@	@
V203_bootloader_V11_PA0.lst: V203_bootloader_V11_PA0.elf
	@	riscv-none-embed-objdump --all-headers --demangle --disassemble -M xw "V203_bootloader_V11_PA0.elf" > "V203_bootloader_V11_PA0.lst"
	@	@
V203_bootloader_V11_PA0.siz: V203_bootloader_V11_PA0.elf
	@	riscv-none-embed-size --format=berkeley "V203_bootloader_V11_PA0.elf"
	@	@
# Other Targets
clean:
	-$(RM) $(ASM_UPPER_DEPS)$(OBJS)$(SECONDARY_FLASH)$(SECONDARY_LIST)$(SECONDARY_SIZE)$(ASM_DEPS)$(S_DEPS)$(S_UPPER_DEPS)$(C_DEPS) V203_bootloader_V11_PA0.elf
	-@
secondary-outputs: $(SECONDARY_FLASH) $(SECONDARY_LIST) $(SECONDARY_SIZE)

.PHONY: all clean dependents

-include ../makefile.targets
