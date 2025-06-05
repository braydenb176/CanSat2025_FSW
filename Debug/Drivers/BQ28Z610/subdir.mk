################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BQ28Z610/BQ28Z610I2C.c 

OBJS += \
./Drivers/BQ28Z610/BQ28Z610I2C.o 

C_DEPS += \
./Drivers/BQ28Z610/BQ28Z610I2C.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BQ28Z610/%.o Drivers/BQ28Z610/%.su Drivers/BQ28Z610/%.cyclo: ../Drivers/BQ28Z610/%.c Drivers/BQ28Z610/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"/Users/bsibb22/Documents/CanSat2025_FSW-NO_RTOS/CanSat2025_FSW/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-BQ28Z610

clean-Drivers-2f-BQ28Z610:
	-$(RM) ./Drivers/BQ28Z610/BQ28Z610I2C.cyclo ./Drivers/BQ28Z610/BQ28Z610I2C.d ./Drivers/BQ28Z610/BQ28Z610I2C.o ./Drivers/BQ28Z610/BQ28Z610I2C.su

.PHONY: clean-Drivers-2f-BQ28Z610

