################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/Components/cy8c4014lqi/cy8c4014lqi.c 

OBJS += \
./Drivers/Components/cy8c4014lqi/cy8c4014lqi.o 

C_DEPS += \
./Drivers/Components/cy8c4014lqi/cy8c4014lqi.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/Components/cy8c4014lqi/%.o Drivers/Components/cy8c4014lqi/%.su Drivers/Components/cy8c4014lqi/%.cyclo: ../Drivers/Components/cy8c4014lqi/%.c Drivers/Components/cy8c4014lqi/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4S5xx -c -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components/lsm6dsl" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components/Common" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components/hts221" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components/lis3mdl" -I"/Users/isbatos/STM32CubeIDE/workspace_1.17.0/lab4_part2/Drivers/Components/lps22hb" -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-Components-2f-cy8c4014lqi

clean-Drivers-2f-Components-2f-cy8c4014lqi:
	-$(RM) ./Drivers/Components/cy8c4014lqi/cy8c4014lqi.cyclo ./Drivers/Components/cy8c4014lqi/cy8c4014lqi.d ./Drivers/Components/cy8c4014lqi/cy8c4014lqi.o ./Drivers/Components/cy8c4014lqi/cy8c4014lqi.su

.PHONY: clean-Drivers-2f-Components-2f-cy8c4014lqi

