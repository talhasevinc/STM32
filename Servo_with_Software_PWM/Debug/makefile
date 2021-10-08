################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
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
Servo_with_Software_PWM.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
Servo_with_Software_PWM.list \


# All Target
all: Servo_with_Software_PWM.elf secondary-outputs

# Tool invocations
Servo_with_Software_PWM.elf: $(OBJS) $(USER_OBJS) C:\Users\mypc\Desktop\STM32CubeID\Servo_with_Software_PWM\STM32F407VGTX_FLASH.ld
	arm-none-eabi-gcc -o "Servo_with_Software_PWM.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\mypc\Desktop\STM32CubeID\Servo_with_Software_PWM\STM32F407VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="Servo_with_Software_PWM.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

Servo_with_Software_PWM.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "Servo_with_Software_PWM.list"
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
