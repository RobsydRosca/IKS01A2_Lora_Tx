################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../X_NUCLEO_IKS01A1/Components/lsm6ds3/lsm6ds3_class.cpp 

OBJS += \
./X_NUCLEO_IKS01A1/Components/lsm6ds3/lsm6ds3_class.o 

CPP_DEPS += \
./X_NUCLEO_IKS01A1/Components/lsm6ds3/lsm6ds3_class.d 


# Each subdirectory must supply rules for building sources it contributes
X_NUCLEO_IKS01A1/Components/lsm6ds3/%.o: ../X_NUCLEO_IKS01A1/Components/lsm6ds3/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: MCU G++ Compiler'
	@echo $(PWD)
	arm-none-eabi-g++ -mcpu=cortex-m4 -mthumb -mfloat-abi=softfp -mfpu=fpv4-sp-d16 '-DDEVICE_I2C=1' '-D__MBED__=1' '-DDEVICE_I2CSLAVE=1' '-D__FPU_PRESENT=1' '-DDEVICE_PORTOUT=1' -DUSBHOST_OTHER '-DDEVICE_PORTINOUT=1' -DTARGET_RTOS_M4_M7 '-DDEVICE_LOWPOWERTIMER=1' '-DDEVICE_RTC=1' -DTOOLCHAIN_object '-DDEVICE_SERIAL_ASYNCH=1' -D__CMSIS_RTOS '-DDEVICE_ANALOGOUT=1' '-DMBED_BUILD_TIMESTAMP=1518780597.4' -DTOOLCHAIN_GCC '-DDEVICE_CAN=1' -DTARGET_STM32L476xG -DTARGET_CORTEX_M '-DDEVICE_I2C_ASYNCH=1' -DTARGET_LIKE_CORTEX_M4 -DTARGET_STM32L476RG -DTARGET_M4 -DTARGET_UVISOR_UNSUPPORTED -DTARGET_STM32L4 '-DDEVICE_SPI_ASYNCH=1' '-DDEVICE_PWMOUT=1' '-DDEVICE_INTERRUPTIN=1' -DTARGET_CORTEX -DTARGET_NUCLEO_L476RG '-DTRANSACTION_QUEUE_SIZE_SPI=2' -D__CORTEX_M4 '-DDEVICE_STDIO_MESSAGES=1' -DTARGET_FF_MORPHO -DTARGET_LIKE_MBED -DTARGET_FF_ARDUINO '-DDEVICE_PORTIN=1' -DTARGET_RELEASE -DTARGET_STM '-DDEVICE_SERIAL_FC=1' '-DDEVICE_TRNG=1' -D__MBED_CMSIS_RTOS_CM '-DDEVICE_SLEEP=1' -DTOOLCHAIN_GCC_ARM '-DDEVICE_SPI=1' '-DDEVICE_SPISLAVE=1' '-DDEVICE_ANALOGIN=1' '-DDEVICE_SERIAL=1' -DARM_MATH_CM4 -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DMBED_DEBUG '-DMBED_TRAP_ERRORS_ENABLED=1' -DNDEBUG -DNDEBUG -I"/home/rob/workspace/IKS01A2_LoRa_Tx" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/Common" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/hts221" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/lis3mdl" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/lps25h" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/lsm6ds0" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/Components/lsm6ds3" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/ST_INTERFACES" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/ST_INTERFACES/Actuators" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/ST_INTERFACES/Common" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/ST_INTERFACES/Communications" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/ST_INTERFACES/Sensors" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/X_NUCLEO_COMMON" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/X_NUCLEO_COMMON/DbgMCU" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/X_NUCLEO_COMMON/DevI2C" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/X_NUCLEO_IKS01A1/X_NUCLEO_COMMON/DevSPI" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM/TARGET_STM32L4" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM/TARGET_STM32L4/TARGET_STM32L476xG" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM/TARGET_STM32L4/TARGET_STM32L476xG/TARGET_NUCLEO_L476RG" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM/TARGET_STM32L4/TARGET_STM32L476xG/device" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TARGET_STM/TARGET_STM32L4/device" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/TARGET_NUCLEO_L476RG/TOOLCHAIN_GCC_ARM" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/drivers" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/hal" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/mbed/platform" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib/debug" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib/enums" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib/radio" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib/registers" -I"/home/rob/workspace/IKS01A2_LoRa_Tx/SX1276Lib/sx1276"  -include/home/rob/workspace/IKS01A2_LoRa_Tx/mbed_config.h -O0 -funsigned-char -fno-delete-null-pointer-checks -fomit-frame-pointer -fmessage-length=0 -fno-builtin -g3 -Wall -Wextra -Wvla -Wno-unused-parameter -Wno-missing-field-initializers -ffunction-sections -fdata-sections -c -fno-exceptions -fno-rtti -ffunction-sections -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


