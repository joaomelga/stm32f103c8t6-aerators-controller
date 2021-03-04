################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/eeprom.c \
../Core/Src/fonts.c \
../Core/Src/freertos.c \
../Core/Src/main.c \
../Core/Src/ssd1306.c \
../Core/Src/stm32f1xx_hal_msp.c \
../Core/Src/stm32f1xx_hal_timebase_tim.c \
../Core/Src/stm32f1xx_it.c \
../Core/Src/syscalls.c 

OBJS += \
./Core/Src/eeprom.o \
./Core/Src/fonts.o \
./Core/Src/freertos.o \
./Core/Src/main.o \
./Core/Src/ssd1306.o \
./Core/Src/stm32f1xx_hal_msp.o \
./Core/Src/stm32f1xx_hal_timebase_tim.o \
./Core/Src/stm32f1xx_it.o \
./Core/Src/syscalls.o 

C_DEPS += \
./Core/Src/eeprom.d \
./Core/Src/fonts.d \
./Core/Src/freertos.d \
./Core/Src/main.d \
./Core/Src/ssd1306.d \
./Core/Src/stm32f1xx_hal_msp.d \
./Core/Src/stm32f1xx_hal_timebase_tim.d \
./Core/Src/stm32f1xx_it.d \
./Core/Src/syscalls.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o: ../Core/Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Core/Inc" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Drivers/CMSIS/Include" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2" -I"C:/Users/joaol/Documents/Projetos/stm32f103c8t6-aerators-controller/codes/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


