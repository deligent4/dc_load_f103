################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ltc4151/ltc4151.c 

OBJS += \
./ltc4151/ltc4151.o 

C_DEPS += \
./ltc4151/ltc4151.d 


# Each subdirectory must supply rules for building sources it contributes
ltc4151/%.o ltc4151/%.su ltc4151/%.cyclo: ../ltc4151/%.c ltc4151/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/dc_load_f103/ssd1306_oled_lib/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ltc4151

clean-ltc4151:
	-$(RM) ./ltc4151/ltc4151.cyclo ./ltc4151/ltc4151.d ./ltc4151/ltc4151.o ./ltc4151/ltc4151.su

.PHONY: clean-ltc4151

