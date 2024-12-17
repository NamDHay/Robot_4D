################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MOTOR_DRIVER/MOTOR_DRIVER.c 

OBJS += \
./MOTOR_DRIVER/MOTOR_DRIVER.o 

C_DEPS += \
./MOTOR_DRIVER/MOTOR_DRIVER.d 


# Each subdirectory must supply rules for building sources it contributes
MOTOR_DRIVER/%.o MOTOR_DRIVER/%.su MOTOR_DRIVER/%.cyclo: ../MOTOR_DRIVER/%.c MOTOR_DRIVER/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I../Core/Inc -I"D:/DAROBOT/Code/CodeMCU/Robot_4D/MOTOR_DRIVER" -I"D:/DAROBOT/Code/CodeMCU/Robot_4D/MOTOR_PID_CONTROL" -I"D:/DAROBOT/Code/CodeMCU/Robot_4D/PID" -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MOTOR_DRIVER

clean-MOTOR_DRIVER:
	-$(RM) ./MOTOR_DRIVER/MOTOR_DRIVER.cyclo ./MOTOR_DRIVER/MOTOR_DRIVER.d ./MOTOR_DRIVER/MOTOR_DRIVER.o ./MOTOR_DRIVER/MOTOR_DRIVER.su

.PHONY: clean-MOTOR_DRIVER

