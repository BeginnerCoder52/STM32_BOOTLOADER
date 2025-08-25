//#include "stm32f103xx.h"
//#include "main.h"
//#include "stm32f103xx_gpio_driver.h"
//#include "stm32f103xx_usart_driver.h"
//#include "stm32f103xx_flash_driver.h"
//#include "stm32f103xx_rcc_driver.h"
//#include "bootloader.h"
//
//#define _estack 0x20005000 // Top of 20KB RAM for STM32F103xx
//
//__attribute__((section(".vectors"), used))
//void (* const vector_table[])(void) = {
//    (void (*)(void))_estack, // Initial MSP
//    bootloader_main          // Initial PC (bootloader entry)
//};
//
//void SystemClock_Config(void) {
//    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//
//    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
//    RCC_OscInitStruct.PLL_State = RCC_PLL_ON;
//    RCC_OscInitStruct.PLL_Source = RCC_PLLSOURCE_HSE_DIV1;
//    RCC_OscInitStruct.PLL_Mul = RCC_PLL_MUL9;
//    RCC_OscConfig(&RCC_OscInitStruct);
//
//    RCC_ClkInitStruct.SysClkSource = RCC_SYSCLKSOURCE_PLLCLK;
//    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
//    RCC_ClockConfig(&RCC_ClkInitStruct);
//}
//
//void MX_GPIO_Init(void) {
//    GPIO_Handle_t gpioLed;
//    gpioLed.pGPIOx = LED_GPIO_Port;
//    gpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_Pin;
//    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
//    gpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = GPIO_ALT_MODE_OUT_PP;
//    GPIO_PeriClockControl(LED_GPIO_Port, ENABLE);
//    GPIO_Init(&gpioLed);
//}
//
//void MX_USART1_UART_Init(void) {
//    USART_Handle_t usart1;
//    usart1.pUSARTx = USART1;
//    usart1.USART_Config.USART_Mode = USART_MODE_TXRX;
//    usart1.USART_Config.USART_Baudrate = USART_STD_BAUD_115200;
//    usart1.USART_Config.USART_NumberOfStopBits = USART_STOPBITS_1;
//    usart1.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
//    usart1.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
//    usart1.USART_Config.USART_HWFLowControl = USART_HW_FLOW_CTRL_NONE;
//    USART_PeriClockControl(USART1, ENABLE);
//    USART_Init(&usart1);
//    // Manual enable if needed: usart1.pUSARTx->CR1 |= (1 << 13); // UE bit
//}
//
//int main(void)
//{
//    SystemClock_Config();
//    MX_GPIO_Init();
//    MX_USART1_UART_Init();
//
//    boot_ctrl_t bcb;
//    read_bcb(&bcb);
//    if (bcb.pending) {
//        bcb.pending = 0;
//        bcb.trial_count = 0;
//        write_bcb(&bcb);
//    }
//    bootloader_main();
//    while (1)
//    {
//    }
//}
//
//void Error_Handler(void) {
//    while(1);
//}
//
//
///* NVIC_SystemReset function commented out as NVIC support is not available in custom drivers
//void NVIC_SystemReset(void) {
//    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
//    while(1);
//}
//*/
#include "stm32f103xx.h"
#include <string.h>

static void uart_send_string(char *str);
static void led_init(void);
static void led_toggle(void);

int main(void)
{
    // ==== Init LED on PC13 ====
    led_init();

    // ==== Init USART1 pins and peripheral ====
    GPIO_Handle_t hgpio;

    // TX (PA9)
    hgpio.pGPIOx = GPIOA;
    hgpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
    hgpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    hgpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_TX_FULLDUP;
    hgpio.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    hgpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
    GPIO_Init(&hgpio);

    // RX (PA10)
    hgpio.pGPIOx = GPIOA;
    hgpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_10;
    hgpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    hgpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_RX_FULLDUP;
    hgpio.GPIO_PinConfig.GPIO_PinSpeed       = 0;
    hgpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
    GPIO_Init(&hgpio);

    // USART1 setup
    husart1.pUSARTx = USART1;
    husart1.USART_Config.USART_Mode             = USART_MODE_TXRX;
    husart1.USART_Config.USART_Baudrate         = USART_STD_BAUD_115200;
    husart1.USART_Config.USART_NumberOfStopBits = USART_STOPBITS_1;
    husart1.USART_Config.USART_WordLength       = USART_WORDLEN_8BITS;
    husart1.USART_Config.USART_ParityControl    = USART_PARITY_DISABLE;
    husart1.USART_Config.USART_HWFLowControl    = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&husart1);
    USART_Start(husart1.pUSARTx);

    uart_send_string("BOOTLOADER: started\r\n");

    while (1)
    {
        // Toggle LED at ~2 Hz
        led_toggle();

        // Send periodic status
        uart_send_string("BOOTLOADER: still alive\r\n");

        // crude delay ~250ms
        for (volatile uint32_t i = 0; i < 500000; i++);
    }
}

static void led_init(void)
{
    GPIO_Handle_t led;
    led.pGPIOx = GPIOC;
    led.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_13;
    led.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    led.GPIO_PinConfig.GPIO_PinCfgMode     = GPIO_CFG_OUT_GE_PP; // General purpose push-pull
    led.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_MEDIUM;  // 2 MHz is plenty for LED
    led.GPIO_PinConfig.GPIO_PinPuPdControl = 0;                  // no pull
    GPIO_Init(&led);

    // PC13 is often active low — set high to turn OFF initially
    GPIO_WritePin(GPIOC, GPIO_PIN_NO_13, 1);
}

static void led_toggle(void)
{
    GPIO_Toggle(GPIOC, GPIO_PIN_NO_13);
}

static void uart_send_string(char *str)
{
    USART_SendData(&husart1, (uint8_t *)str, strlen(str));
}



