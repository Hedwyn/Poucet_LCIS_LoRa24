################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Utils/SX1280Lib/RangingCorrection.cpp \
../Utils/SX1280Lib/sx1280-hal.cpp \
../Utils/SX1280Lib/sx1280.cpp 

OBJS += \
./Utils/SX1280Lib/RangingCorrection.o \
./Utils/SX1280Lib/sx1280-hal.o \
./Utils/SX1280Lib/sx1280.o 

CPP_DEPS += \
./Utils/SX1280Lib/RangingCorrection.d \
./Utils/SX1280Lib/sx1280-hal.d \
./Utils/SX1280Lib/sx1280.d 


# Each subdirectory must supply rules for building sources it contributes
Utils/SX1280Lib/%.o: ../Utils/SX1280Lib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 '-DMBED_BUILD_TIMESTAMP=1568904775.85' '-D__MBED__=1' '-DDEVICE_I2CSLAVE=1' -DTARGET_LIKE_MBED '-DDEVICE_PORTINOUT=1' -DTARGET_RTOS_M4_M7 '-DDEVICE_LOWPOWERTIMER=1' '-DDEVICE_RTC=1' -D__CMSIS_RTOS -DTOOLCHAIN_GCC '-DDEVICE_CAN=1' -DTARGET_CORTEX_M -DTARGET_LIKE_CORTEX_M4 '-DDEVICE_ANALOGOUT=1' -DTARGET_M4 -DTARGET_STM32L4 '-DDEVICE_SERIAL=1' '-DDEVICE_INTERRUPTIN=1' -DTARGET_CORTEX '-DDEVICE_I2C=1' '-DDEVICE_PORTOUT=1' -D__CORTEX_M4 '-DDEVICE_STDIO_MESSAGES=1' '-D__FPU_PRESENT=1' -DTARGET_FF_ARDUINO '-DDEVICE_PORTIN=1' -DTARGET_RELEASE '-DTARGET_NAME=NUCLEO_L432KC' -DTARGET_STM -DTARGET_STM32L432KC '-DDEVICE_SERIAL_FC=1' -D__MBED_CMSIS_RTOS_CM '-DDEVICE_SLEEP=1' -DTOOLCHAIN_GCC_ARM '-DDEVICE_SPI=1' '-DDEVICE_SPISLAVE=1' '-DDEVICE_ANALOGIN=1' '-DDEVICE_PWMOUT=1' -DTARGET_NUCLEO_L432KC -DARM_MATH_CM4 -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DNDEBUG -DNDEBUG -I".." -I"../Peripherals" -I"../mbed" -I"../mbed/TARGET_NUCLEO_L432KC" -I"../mbed/TARGET_NUCLEO_L432KC/TARGET_STM/TARGET_STM32L4" -I"../mbed/TARGET_NUCLEO_L432KC/TARGET_STM/TARGET_STM32L4/TARGET_NUCLEO_L432KC" -I"../Utils" -I"../Utils/SX1280Lib" -I"../Utils/SX1280Lib/rangingCorrection"  -include../mbed_config.h -O3 -funsigned-char -fno-delete-null-pointer-checks -fomit-frame-pointer -fmessage-length=0 -w -Wall -Wextra -Wvla -Wno-unused-parameter -Wno-missing-field-initializers -ffunction-sections -fdata-sections -c -fno-exceptions -fno-rtti -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


