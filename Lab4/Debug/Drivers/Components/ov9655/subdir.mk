################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/ov9655/ov9655.c 

OBJS += \
./Drivers/Components/ov9655/ov9655.o 

C_DEPS += \
./Drivers/Components/ov9655/ov9655.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/ov9655/%.o Drivers/Components/ov9655/%.su Drivers/Components/ov9655/%.cyclo: ../Drivers/Components/ov9655/%.c Drivers/Components/ov9655/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components/lsm6dsl" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components/lps22hb" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components/lis3mdl" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components/hts221" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4/Drivers/Components/Common" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-ov9655

clean-Drivers-2f-Components-2f-ov9655:
	-$(RM) ./Drivers/Components/ov9655/ov9655.cyclo ./Drivers/Components/ov9655/ov9655.d ./Drivers/Components/ov9655/ov9655.o ./Drivers/Components/ov9655/ov9655.su

.PHONY: clean-Drivers-2f-Components-2f-ov9655

