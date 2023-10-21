################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/servo/SERVO.c \
../Drivers/servo/SERVO_cfg.c 

C_DEPS += \
./Drivers/servo/SERVO.d \
./Drivers/servo/SERVO_cfg.d 

OBJS += \
./Drivers/servo/SERVO.o \
./Drivers/servo/SERVO_cfg.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/servo/%.o Drivers/servo/%.su Drivers/servo/%.cyclo: ../Drivers/servo/%.c Drivers/servo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Drivers/dwt -I../Drivers/servo -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-servo

clean-Drivers-2f-servo:
	-$(RM) ./Drivers/servo/SERVO.cyclo ./Drivers/servo/SERVO.d ./Drivers/servo/SERVO.o ./Drivers/servo/SERVO.su ./Drivers/servo/SERVO_cfg.cyclo ./Drivers/servo/SERVO_cfg.d ./Drivers/servo/SERVO_cfg.o ./Drivers/servo/SERVO_cfg.su

.PHONY: clean-Drivers-2f-servo

