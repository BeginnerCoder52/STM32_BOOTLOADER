################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f103xx_gpio_driver.c \
../drivers/Src/stm32f103xx_i2c_driver.c \
../drivers/Src/stm32f103xx_rcc_driver.c \
../drivers/Src/stm32f103xx_spi_driver.c \
../drivers/Src/stm32f103xx_usart.c 

OBJS += \
./drivers/Src/stm32f103xx_gpio_driver.o \
./drivers/Src/stm32f103xx_i2c_driver.o \
./drivers/Src/stm32f103xx_rcc_driver.o \
./drivers/Src/stm32f103xx_spi_driver.o \
./drivers/Src/stm32f103xx_usart.o 

C_DEPS += \
./drivers/Src/stm32f103xx_gpio_driver.d \
./drivers/Src/stm32f103xx_i2c_driver.d \
./drivers/Src/stm32f103xx_rcc_driver.d \
./drivers/Src/stm32f103xx_spi_driver.d \
./drivers/Src/stm32f103xx_usart.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32 -DSTM32F1 -DSTM32F103C8Tx -c -I../Inc -I"/home/richardmelvin52/Thuc_hanh_UIT/FPT_Intern_Bootloader/STM32_BOOTLOADER/BOOTLOADER/drivers/Inc" -I"/home/richardmelvin52/Thuc_hanh_UIT/FPT_Intern_Bootloader/STM32_BOOTLOADER/BOOTLOADER/drivers_2/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f103xx_gpio_driver.cyclo ./drivers/Src/stm32f103xx_gpio_driver.d ./drivers/Src/stm32f103xx_gpio_driver.o ./drivers/Src/stm32f103xx_gpio_driver.su ./drivers/Src/stm32f103xx_i2c_driver.cyclo ./drivers/Src/stm32f103xx_i2c_driver.d ./drivers/Src/stm32f103xx_i2c_driver.o ./drivers/Src/stm32f103xx_i2c_driver.su ./drivers/Src/stm32f103xx_rcc_driver.cyclo ./drivers/Src/stm32f103xx_rcc_driver.d ./drivers/Src/stm32f103xx_rcc_driver.o ./drivers/Src/stm32f103xx_rcc_driver.su ./drivers/Src/stm32f103xx_spi_driver.cyclo ./drivers/Src/stm32f103xx_spi_driver.d ./drivers/Src/stm32f103xx_spi_driver.o ./drivers/Src/stm32f103xx_spi_driver.su ./drivers/Src/stm32f103xx_usart.cyclo ./drivers/Src/stm32f103xx_usart.d ./drivers/Src/stm32f103xx_usart.o ./drivers/Src/stm32f103xx_usart.su

.PHONY: clean-drivers-2f-Src

