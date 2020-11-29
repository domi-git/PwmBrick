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
PwmBrick.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PwmBrick.list \


# All Target
all: PwmBrick.elf secondary-outputs

# Tool invocations
PwmBrick.elf: $(OBJS) $(USER_OBJS) /home/domi/Dropbox/Privat/Projekte/PwmBrick/Git/Design-Files/Software/uC/STM32CubeIDE/STM32F030F4PX_FLASH.ld
	arm-none-eabi-gcc -o "PwmBrick.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"/home/domi/Dropbox/Privat/Projekte/PwmBrick/Git/Design-Files/Software/uC/STM32CubeIDE/STM32F030F4PX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PwmBrick.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PwmBrick.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PwmBrick.list"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
