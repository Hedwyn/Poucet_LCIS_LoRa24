################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

OS = WIN
# OS = LINUX


RM := rm -f

# Flags defined on Makefile call
BIN = Master# bin name defaults to master



# All of the sources participating in the build are defined here
-include sources.mk
-include mbed/TARGET_NUCLEO_L432KC/TOOLCHAIN_GCC_ARM/subdir.mk
-include Utils/SX1280Lib/subdir.mk
-include Utils/subdir.mk
-include Peripherals/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: pre-build main-build

# Main-build Target
main-build: SX1280LCIS-release.elf

# Tool invocations
SX1280LCIS-release.elf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: MCU G++ Linker'
	arm-none-eabi-g++ -specs=nosys.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 -L"../mbed/TARGET_NUCLEO_L432KC/TOOLCHAIN_GCC_ARM" -u _printf_float, -Wl,--wrap=main -Wl,--wrap=_memalign_r -Wl,-n -Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -Wl,--end-group -T"../Release/linker-script-release.ld" -Wl,-Map=output.map -Wl,--gc-sections -fno-exceptions -fno-rtti -o "SX1280LCIS-release.elf" @"objects.list" $(USER_OBJS) $(LIBS) -lm
	@echo 'Finished building target: $@'
	@echo ' '
	$(MAKE) --no-print-directory post-build

# Other Targets
clean_objs:
	-$(RM) $(OBJS)

clean_bins:
	-$(RM) *.bin *.elf

clean: clean_objs clean_bins
	-@echo 'Cleaning done'

pre-build:
	-@echo 'Creating makefile.defs:'
	-echo "export PREPROC_CMD = arm-none-eabi-cpp -E -P" > ../makefile.defs
	-@echo ' '

post-build:
	-@echo 'Generating binary and Printing size information:'
	arm-none-eabi-objcopy -O binary "SX1280LCIS-release.elf" "$(BIN).bin"
	arm-none-eabi-size -B "SX1280LCIS-release.elf"
	make ldclean
	-@echo ' '

.PHONY: all clean dependents clean_objs clean_bins
.SECONDARY: main-build pre-build post-build

-include ../makefile.targets
