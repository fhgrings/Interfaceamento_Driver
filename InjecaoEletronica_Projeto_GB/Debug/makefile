################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/subdir.mk
-include Middlewares/Third_Party/FreeRTOS/Source/subdir.mk
-include Drivers/STM32F3xx_HAL_Driver/Src/subdir.mk
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
InjecaoEletronica_Projeto_GB.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
InjecaoEletronica_Projeto_GB.list \


# All Target
all: InjecaoEletronica_Projeto_GB.elf secondary-outputs

# Tool invocations
InjecaoEletronica_Projeto_GB.elf: $(OBJS) $(USER_OBJS) C:\Users\felip\Desktop\Code\Faculdade\Interfaceamento_Driver\InjecaoEletronica_Projeto_GB\STM32F334R8TX_FLASH.ld
	arm-none-eabi-gcc -o "InjecaoEletronica_Projeto_GB.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\felip\Desktop\Code\Faculdade\Interfaceamento_Driver\InjecaoEletronica_Projeto_GB\STM32F334R8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="InjecaoEletronica_Projeto_GB.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

InjecaoEletronica_Projeto_GB.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "InjecaoEletronica_Projeto_GB.list"
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
