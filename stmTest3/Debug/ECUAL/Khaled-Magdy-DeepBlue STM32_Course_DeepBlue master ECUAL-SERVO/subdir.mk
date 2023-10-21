################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (11.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.c \
../ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.c 

C_DEPS += \
./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.d \
./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.d 

OBJS += \
./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.o \
./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.o 


# Each subdirectory must supply rules for building sources it contributes
ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.o: ../ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.c ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"ECUAL/Khaled-Magdy-DeepBlue STM32_Course_DeepBlue master ECUAL-SERVO/SERVO.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.o: ../ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.c ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L433xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"ECUAL/Khaled-Magdy-DeepBlue STM32_Course_DeepBlue master ECUAL-SERVO/SERVO_cfg.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ECUAL-2f-Khaled-2d-Magdy-2d-DeepBlue-20-STM32_Course_DeepBlue-20-master-20-ECUAL-2d-SERVO

clean-ECUAL-2f-Khaled-2d-Magdy-2d-DeepBlue-20-STM32_Course_DeepBlue-20-master-20-ECUAL-2d-SERVO:
	-$(RM) ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.cyclo ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.d ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.o ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO.su ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.cyclo ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.d ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.o ./ECUAL/Khaled-Magdy-DeepBlue\ STM32_Course_DeepBlue\ master\ ECUAL-SERVO/SERVO_cfg.su

.PHONY: clean-ECUAL-2f-Khaled-2d-Magdy-2d-DeepBlue-20-STM32_Course_DeepBlue-20-master-20-ECUAL-2d-SERVO

