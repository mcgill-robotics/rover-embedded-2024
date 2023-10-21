################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/dwt/DWT_Delay.c 

C_DEPS += \
./Drivers/dwt/DWT_Delay.d 

OBJS += \
./Drivers/dwt/DWT_Delay.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/dwt/%.o Drivers/dwt/%.su Drivers/dwt/%.cyclo: ../Drivers/dwt/%.c Drivers/dwt/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/dwt -I../Drivers/servo -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-dwt

clean-Drivers-2f-dwt:
	-$(RM) ./Drivers/dwt/DWT_Delay.cyclo ./Drivers/dwt/DWT_Delay.d ./Drivers/dwt/DWT_Delay.o ./Drivers/dwt/DWT_Delay.su

.PHONY: clean-Drivers-2f-dwt

