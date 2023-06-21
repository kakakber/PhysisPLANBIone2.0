################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/LORALIB/LoRa.c 

OBJS += \
./Core/Src/LORALIB/LoRa.o 

C_DEPS += \
./Core/Src/LORALIB/LoRa.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/LORALIB/%.o Core/Src/LORALIB/%.su Core/Src/LORALIB/%.cyclo: ../Core/Src/LORALIB/%.c Core/Src/LORALIB/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32H7A3xxQ -c -I../Core/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../FATFS/Target -I../FATFS/App -I../Middlewares/Third_Party/FatFs/src -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Src-2f-LORALIB

clean-Core-2f-Src-2f-LORALIB:
	-$(RM) ./Core/Src/LORALIB/LoRa.cyclo ./Core/Src/LORALIB/LoRa.d ./Core/Src/LORALIB/LoRa.o ./Core/Src/LORALIB/LoRa.su

.PHONY: clean-Core-2f-Src-2f-LORALIB

