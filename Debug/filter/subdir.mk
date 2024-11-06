################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../filter/filter.c 

OBJS += \
./filter/filter.o 

C_DEPS += \
./filter/filter.d 


# Each subdirectory must supply rules for building sources it contributes
filter/%.o filter/%.su filter/%.cyclo: ../filter/%.c filter/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/dc_load_f103/ssd1306_oled_lib/inc" -I"E:/STM32CubeIDE/dc_load_f103/ltc2959" -I"E:/STM32CubeIDE/dc_load_f103/ad5693" -I"E:/STM32CubeIDE/dc_load_f103/filter" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-filter

clean-filter:
	-$(RM) ./filter/filter.cyclo ./filter/filter.d ./filter/filter.o ./filter/filter.su

.PHONY: clean-filter

