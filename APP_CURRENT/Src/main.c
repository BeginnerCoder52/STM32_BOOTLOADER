/*
 * main.c - APP_CURRENT for STM32F103C8T6
 * - Runs at 0x08004000 (55KB code + 1KB metadata)
 * - Uses custom drivers only (no HAL/CMSIS)
 * - UART1: PA9(TX)/PA10(RX), 115200
 * - LED: PA7 blink ~0.5Hz (1s ON / 1s OFF)
 * - On OTA request (FW_REQUEST), write FW_READY to footer and reset to Bootloader
 */

#include <string.h>
#include "stm32f103xx.h"
#include "app_meta.h"

/* Handles allocated by user (extern only if defined elsewhere) */
USART_Handle_t husart1;
GPIO_Handle_t  hLedA7;

/* UART config */
#define BAUDRATE            115200U
#define CMD_FW_REQUEST_ID   2              /* byte command */
#define CMD_FW_REQUEST_STR  "FW_REQUEST"   /* ascii command */

/* APP base and VTOR */
#define APP_BASE_ADDR       0x08004000UL

/* SCB AIRCR register */
#define SCB_AIRCR           (*(volatile uint32_t*)0xE000ED0CUL)
#define AIRCR_VECTKEY       (0x5FAUL << 16)
#define SYSRESETREQ_BIT     (1UL << 2)

/* Prototypes */
static void set_vector_table(void);
static void gpio_init_pa7(void);
static void usart1_pins_init(void);
static void usart1_init(void);
static void delay_ms(uint32_t ms);
static int  uart_getc_nonblock(uint8_t *c);
static void system_soft_reset(void);
static int  check_fw_request(void);

int main(void)
{
    /* Keep default clocking (HSI 8MHz). Using RCC_DeInit to known state if needed */
    RCC_DeInit();

    set_vector_table();
    gpio_init_pa7();
    usart1_pins_init();
    usart1_init();

    const char hello[] = "APP_CURRENT running at 0x08004000\r\n";
    USART_SendData(&husart1, (uint8_t*)hello, (uint32_t)strlen(hello));

    while (1)
    {
        /* Blink PA7 at ~0.5Hz */
        GPIO_WritePin(GPIOA, GPIO_PIN_NO_7, GPIO_PIN_SET);
        delay_ms(1000);
        GPIO_WritePin(GPIOA, GPIO_PIN_NO_7, GPIO_PIN_RESET);
        delay_ms(1000);

        /* Check for OTA request (ID=2 or string "FW_REQUEST") */
        if (check_fw_request())
        {
            const char note[] = "APP_CURRENT: OTA request -> write FW_READY and reset\r\n";
            USART_SendData(&husart1, (uint8_t*)note, (uint32_t)strlen(note));

            app_meta_t meta = {
                .magic   = META_MAGIC,
                .flag    = META_FLAG_FW_READY,
                .version = 0x00010000UL,
                .crc32   = 0
            };
            app_meta_write(APP_CURRENT_META_ADDR, &meta);

            /* Short guard delay, then reset to Bootloader */
            delay_ms(50);
            system_soft_reset();
        }
    }
}

/* Set VTOR to App base so vector table points to 0x08004000 when jumped from BL */
static void set_vector_table(void)
{
    /* SCB->VTOR is at 0xE000ED08 */
    volatile uint32_t *VTOR = (volatile uint32_t*)0xE000ED08UL;
    *VTOR = APP_BASE_ADDR;
    __asm volatile ("dsb");
    __asm volatile ("isb");
}

/* Configure PA7 as push-pull output */
static void gpio_init_pa7(void)
{
    hLedA7.pGPIOx = GPIOA;
    hLedA7.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_7;
    hLedA7.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_OUT;
    hLedA7.GPIO_PinConfig.GPIO_PinCfgMode     = GPIO_CFG_OUT_GE_PP;
    hLedA7.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_MEDIUM;
    hLedA7.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
    GPIO_Init(&hLedA7);

    GPIO_WritePin(GPIOA, GPIO_PIN_NO_7, 0);
}

/* Configure USART1 pins: PA9 (TX AF-PP), PA10 (RX AF-Input floating) */
static void usart1_pins_init(void)
{
    GPIO_Handle_t gpio;

    /* TX: PA9 AF-PP, fast speed */
    gpio.pGPIOx = GPIOA;
    gpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
    gpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    gpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_TX_FULLDUP;
    gpio.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;
    gpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
    GPIO_Init(&gpio);

    /* RX: PA10 AF input floating */
    gpio.pGPIOx = GPIOA;
    gpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_10;
    gpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    gpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_RX_FULLDUP;
    gpio.GPIO_PinConfig.GPIO_PinSpeed       = 0;
    gpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;
    GPIO_Init(&gpio);
}

/* Init USART1 with custom driver (both APB1 & APB2 kept at same clock by RCC_DeInit) */
static void usart1_init(void)
{
    husart1.pUSARTx                                  = USART1;
    husart1.USART_Config.USART_Mode                  = USART_MODE_TXRX;
    husart1.USART_Config.USART_Baudrate              = BAUDRATE;
    husart1.USART_Config.USART_NumberOfStopBits      = USART_STOPBITS_1;
    husart1.USART_Config.USART_WordLength            = USART_WORDLEN_8BITS;
    husart1.USART_Config.USART_ParityControl         = USART_PARITY_DISABLE;
    husart1.USART_Config.USART_HWFLowControl         = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&husart1);
    USART_Start(husart1.pUSARTx);
}

/* Crude busy-wait delay (HSI 8MHz) */
static void delay_ms(uint32_t ms)
{
    for (volatile uint32_t i = 0; i < (ms * 8000U); i++) {
        __asm volatile ("nop");
    }
}

/* Non-blocking read: return 1 if a char is available, 0 otherwise */
static int uart_getc_nonblock(uint8_t *c)
{
    if ((husart1.pUSARTx->SR >> USART_SR_RXNE) & 1U) {
        *c = (uint8_t)(husart1.pUSARTx->DR & 0xFFU);
        return 1;
    }
    return 0;
}

/* Software reset: write SYSRESETREQ to AIRCR (no core_cm3 dependency) */
static void system_soft_reset(void)
{
    SCB_AIRCR = (AIRCR_VECTKEY | SYSRESETREQ_BIT);
    while (1) { __asm volatile ("nop"); }
}

/* Detect FW_REQUEST either as single-byte command ID=2 or ASCII "FW_REQUEST" */
static int check_fw_request(void)
{
    static char line[32];
    static uint32_t len = 0;

    uint8_t ch;
    if (!uart_getc_nonblock(&ch)) return 0;

    /* Accept binary command ID = 2 */
    if (ch == (uint8_t)CMD_FW_REQUEST_ID) {
        return 1;
    }

    /* Or accept ASCII "FW_REQUEST" terminated by newline */
    if (ch == '\r' || ch == '\n') {
        line[len] = '\0';
        int match = (strcmp(line, CMD_FW_REQUEST_STR) == 0);
        len = 0;
        return match;
    }

    if (len < (sizeof(line) - 1)) {
        line[len++] = (char)ch;
    } else {
        len = 0; /* overflow guard */
    }
    return 0;
}
