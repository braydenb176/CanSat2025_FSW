################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/MS5607/AltitudeCalculations.c \
../Drivers/MS5607/AltitudeCalculationsAccel.c \
../Drivers/MS5607/MS5607SPI.c 

OBJS += \
./Drivers/MS5607/AltitudeCalculations.o \
./Drivers/MS5607/AltitudeCalculationsAccel.o \
./Drivers/MS5607/MS5607SPI.o 

C_DEPS += \
./Drivers/MS5607/AltitudeCalculations.d \
./Drivers/MS5607/AltitudeCalculationsAccel.d \
./Drivers/MS5607/MS5607SPI.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/MS5607/%.o Drivers/MS5607/%.su Drivers/MS5607/%.cyclo: ../Drivers/MS5607/%.c Drivers/MS5607/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"/Users/bsibb22/Documents/CanSat2025_FSW-NO_RTOS/CanSat2025_FSW/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-MS5607

clean-Drivers-2f-MS5607:
	-$(RM) ./Drivers/MS5607/AltitudeCalculations.cyclo ./Drivers/MS5607/AltitudeCalculations.d ./Drivers/MS5607/AltitudeCalculations.o ./Drivers/MS5607/AltitudeCalculations.su ./Drivers/MS5607/AltitudeCalculationsAccel.cyclo ./Drivers/MS5607/AltitudeCalculationsAccel.d ./Drivers/MS5607/AltitudeCalculationsAccel.o ./Drivers/MS5607/AltitudeCalculationsAccel.su ./Drivers/MS5607/MS5607SPI.cyclo ./Drivers/MS5607/MS5607SPI.d ./Drivers/MS5607/MS5607SPI.o ./Drivers/MS5607/MS5607SPI.su

.PHONY: clean-Drivers-2f-MS5607

