/*
 * main.c - APP_CURRENT (Blinky LED)
 * Yêu cầu: Linker script phải set FLASH ORIGIN = 0x08002C00
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_rcc_driver.h"

#define LED_PIN     13 // PC13

// Địa chỉ thanh ghi SCB->VTOR (Vector Table Offset Register)
#define SCB_VTOR_ADDR   0xE000ED08UL
#define SCB_VTOR        (*((volatile uint32_t *)SCB_VTOR_ADDR))

// Định nghĩa offset của App (phải khớp với Bootloader)
#define APP_OFFSET      0x2C00U

/* Hàm cấu hình lại Vector Table */
void SystemInit_Custom(void) {
    // 1. Dời bảng vector ngắt về địa chỉ bắt đầu của App
    SCB_VTOR = 0x08000000U | APP_OFFSET;
}

/* Cấu hình Clock đơn giản (HSI 8MHz) cho App */
void SystemClock_Setup_App(void) {
    // Sử dụng HSI mặc định, chỉ cần đảm bảo GPIO clock chạy
}

void GPIO_Setup_App(void) {
    // Bật clock GPIOC
    RCC->APB2ENR |= (1 << 4);

    GPIO_Handle_t GpioLed;
    GpioLed.pGPIOx = GPIOC;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

    GPIO_Init(&GpioLed);
}

int main(void)
{
    // 1. Cấu hình Vector Table (Rất quan trọng!)
    SystemInit_Custom();

    // 2. Init phần cứng của App
    SystemClock_Setup_App();
    GPIO_Setup_App();

    // 3. Vòng lặp Blink LED
    while(1)
    {
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);

        // Delay ngắn (nháy nhanh)
        for(volatile int i = 0; i < 50000; i++);
    }
}
