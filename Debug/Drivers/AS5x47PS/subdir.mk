################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/AS5x47PS/AS5x47P.c 

OBJS += \
./Drivers/AS5x47PS/AS5x47P.o 

C_DEPS += \
./Drivers/AS5x47PS/AS5x47P.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/AS5x47PS/%.o Drivers/AS5x47PS/%.su: ../Drivers/AS5x47PS/%.c Drivers/AS5x47PS/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32G431xx -c -I../Core/Inc -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_SVPWM/Drivers/EncoderCalibration" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_SVPWM/Drivers/AS5x47PS" -I../Drivers/STM32G4xx_HAL_Driver/Inc -I../Drivers/STM32G4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32G4xx/Include -I../Drivers/CMSIS/Include -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_SVPWM/Drivers/Q15_math" -I"/home/harsha/STM32CubeIDE/workspace_1.10.0/gen4_motor_inverter_SVPWM/Drivers/SVPWM" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-AS5x47PS

clean-Drivers-2f-AS5x47PS:
	-$(RM) ./Drivers/AS5x47PS/AS5x47P.d ./Drivers/AS5x47PS/AS5x47P.o ./Drivers/AS5x47PS/AS5x47P.su

.PHONY: clean-Drivers-2f-AS5x47PS

