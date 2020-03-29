################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F0xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
power_testing.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
power_testing.list \

OBJCOPY_BIN += \
power_testing.bin \


# All Target
all: power_testing.elf secondary-outputs

# Tool invocations
power_testing.elf: $(OBJS) $(USER_OBJS) Z:\documents\2020_1_Spring\ECE_49022\Cube_Projects\power_testing\STM32F051R8TX_FLASH.ld
	arm-none-eabi-gcc -o "power_testing.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"Z:\documents\2020_1_Spring\ECE_49022\Cube_Projects\power_testing\STM32F051R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="power_testing.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

power_testing.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "power_testing.list"
	@echo 'Finished building: $@'
	@echo ' '

power_testing.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "power_testing.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
