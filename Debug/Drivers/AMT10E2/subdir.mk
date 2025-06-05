################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AMT10E2/AMT10E2.c 

OBJS += \
./Drivers/AMT10E2/AMT10E2.o 

C_DEPS += \
./Drivers/AMT10E2/AMT10E2.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AMT10E2/%.o Drivers/AMT10E2/%.su Drivers/AMT10E2/%.cyclo: ../Drivers/AMT10E2/%.c Drivers/AMT10E2/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G491xx -DSTM32_THREAD_SAFE_STRATEGY=2 -c -I../USB_Device/App -I../USB_Device/Target -I../Core/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/DFU/Inc -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I../Core/ThreadSafe -I"C:/Users/Tom Rice-Bladykas/Documents/CS25/CS25/Drivers" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-AMT10E2

clean-Drivers-2f-AMT10E2:
	-$(RM) ./Drivers/AMT10E2/AMT10E2.cyclo ./Drivers/AMT10E2/AMT10E2.d ./Drivers/AMT10E2/AMT10E2.o ./Drivers/AMT10E2/AMT10E2.su

.PHONY: clean-Drivers-2f-AMT10E2

