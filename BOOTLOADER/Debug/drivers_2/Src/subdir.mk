################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers_2/Src/FPEC_program.c \
../drivers_2/Src/GPIO_program.c \
../drivers_2/Src/RCC_program.c \
../drivers_2/Src/STK_program.c \
../drivers_2/Src/UART_program.c 

OBJS += \
./drivers_2/Src/FPEC_program.o \
./drivers_2/Src/GPIO_program.o \
./drivers_2/Src/RCC_program.o \
./drivers_2/Src/STK_program.o \
./drivers_2/Src/UART_program.o 

C_DEPS += \
./drivers_2/Src/FPEC_program.d \
./drivers_2/Src/GPIO_program.d \
./drivers_2/Src/RCC_program.d \
./drivers_2/Src/STK_program.d \
./drivers_2/Src/UART_program.d 


# Each subdirectory must supply rules for building sources it contributes
drivers_2/Src/%.o drivers_2/Src/%.su drivers_2/Src/%.cyclo: ../drivers_2/Src/%.c drivers_2/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"/home/richardmelvin52/Thuc_hanh_UIT/FPT_Intern_Bootloader/STM32_BOOTLOADER/BOOTLOADER/drivers/Inc" -I"/home/richardmelvin52/Thuc_hanh_UIT/FPT_Intern_Bootloader/STM32_BOOTLOADER/BOOTLOADER/drivers_2/Inc" -I"/home/richardmelvin52/Thuc_hanh_UIT/FPT_Intern_Bootloader/STM32_BOOTLOADER/BOOTLOADER/drivers_2/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers_2-2f-Src

clean-drivers_2-2f-Src:
	-$(RM) ./drivers_2/Src/FPEC_program.cyclo ./drivers_2/Src/FPEC_program.d ./drivers_2/Src/FPEC_program.o ./drivers_2/Src/FPEC_program.su ./drivers_2/Src/GPIO_program.cyclo ./drivers_2/Src/GPIO_program.d ./drivers_2/Src/GPIO_program.o ./drivers_2/Src/GPIO_program.su ./drivers_2/Src/RCC_program.cyclo ./drivers_2/Src/RCC_program.d ./drivers_2/Src/RCC_program.o ./drivers_2/Src/RCC_program.su ./drivers_2/Src/STK_program.cyclo ./drivers_2/Src/STK_program.d ./drivers_2/Src/STK_program.o ./drivers_2/Src/STK_program.su ./drivers_2/Src/UART_program.cyclo ./drivers_2/Src/UART_program.d ./drivers_2/Src/UART_program.o ./drivers_2/Src/UART_program.su

.PHONY: clean-drivers_2-2f-Src

