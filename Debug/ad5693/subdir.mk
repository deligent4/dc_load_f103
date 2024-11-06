################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ad5693/ad5693.c 

OBJS += \
./ad5693/ad5693.o 

C_DEPS += \
./ad5693/ad5693.d 


# Each subdirectory must supply rules for building sources it contributes
ad5693/%.o ad5693/%.su ad5693/%.cyclo: ../ad5693/%.c ad5693/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F103xB -c -I../Core/Inc -I../Drivers/STM32F1xx_HAL_Driver/Inc/Legacy -I../Drivers/STM32F1xx_HAL_Driver/Inc -I../Drivers/CMSIS/Device/ST/STM32F1xx/Include -I../Drivers/CMSIS/Include -I"E:/STM32CubeIDE/dc_load_f103/ssd1306_oled_lib/inc" -I"E:/STM32CubeIDE/dc_load_f103/ltc2959" -I"E:/STM32CubeIDE/dc_load_f103/ad5693" -I"E:/STM32CubeIDE/dc_load_f103/filter" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-ad5693

clean-ad5693:
	-$(RM) ./ad5693/ad5693.cyclo ./ad5693/ad5693.d ./ad5693/ad5693.o ./ad5693/ad5693.su

.PHONY: clean-ad5693

