/*
 * main.c - STM32 BOOTLOADER (Fixed Driver Bugs Locally)
 * Target: STM32F103C8T6
 * Clock: 72MHz (HSE)
 * Baud: 115200 (Manually patched)
 */

#include <stdint.h>
#include <string.h>
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_usart_driver.h"
#include "stm32f103xx_rcc_driver.h"

/* --- CONFIGURATION --- */
#define APP_START_ADDRESS   0x08002C00U
#define FLASH_PAGE_SIZE     1024
#define LED_PIN             13  // PC13

/* --- PROTOCOL DEFINITIONS (HEX) --- */
#define CMD_CONNECT         0xAA 
#define CMD_INFO            0xBB 
#define CMD_DATA            0xCC 
#define CMD_END             0xDD 

#define RSP_ACK             0x06
#define RSP_NACK            0x15

/* --- FLASH DEFS --- */
typedef struct {
    volatile uint32_t ACR;
    volatile uint32_t KEYR;
    volatile uint32_t OPTKEYR;
    volatile uint32_t SR;
    volatile uint32_t CR;
    volatile uint32_t AR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WRPR;
} FLASH_TypeDef_Custom;

#define FLASH_R_BASE      0x40022000UL
#define FLASH_R           ((FLASH_TypeDef_Custom *)FLASH_R_BASE)
#define FLASH_KEY1        0x45670123UL
#define FLASH_KEY2        0xCDEF89ABUL
#define FLASH_CR_PG       (1U << 0)
#define FLASH_CR_PER      (1U << 1)
#define FLASH_CR_STRT     (1U << 6)
#define FLASH_CR_LOCK     (1U << 7)
#define FLASH_SR_BSY      (1U << 0)

USART_Handle_t usart1_handle;

/* --- FIX __set_MSP --- */
__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack) {
    __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) : "sp");
}

/* --- SYSTEM CLOCK SETUP 72MHz --- */
void SystemClock_Setup_72MHz(void) {
    // 1. Enable HSE
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17))); 

    // 2. Flash Latency = 2
    FLASH_R->ACR |= 0x02;

    // 3. PLL Config: HSE * 9 = 72MHz
    RCC->CFGR |= (1 << 16) | (0x7 << 18);

    // 4. Prescalers: AHB=1, APB2=1, APB1=2
    RCC->CFGR |= (1 << 10); 

    // 5. Enable PLL
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25))); 

    // 6. Select PLL as SYSCLK
    RCC->CFGR &= ~(0x3 << 0);
    RCC->CFGR |= (0x2 << 0);
    while ((RCC->CFGR & (0x3 << 2)) != (0x2 << 2));
}

void GPIO_Setup(void) {
    // Enable Clocks
    RCC->APB2ENR |= (1 << 2) | (1 << 4); // GPIOA, GPIOC

    // LED PC13
    GPIO_Handle_t Gpio;
    Gpio.pGPIOx = GPIOC;
    Gpio.GPIO_PinConfig.GPIO_PinNumber = LED_PIN;
    Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT_PP;
    Gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
    GPIO_Init(&Gpio);
    GPIO_WriteToOutputPin(GPIOC, LED_PIN, GPIO_PIN_SET); 

    // PA9 - TX (Alt Push Pull)
    Gpio.pGPIOx = GPIOA;
    Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALT_PP;
    Gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&Gpio);

    // PA10 - RX (Input Pull-up)
    Gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    Gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_PP;
    Gpio.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    GPIO_Init(&Gpio);

    // --- PATCH: Sửa lỗi driver GPIO không set ODR cho Pull-up ---
    // Bắt buộc set bit 10 của ODR lên 1 để kích hoạt Pull-up điện trở
    GPIOA->ODR |= (1 << 10); 
}

void USART1_Setup(void) {
    usart1_handle.pUSARTx = USART1;
    usart1_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart1_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart1_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(&usart1_handle);
    USART_PeripheralControl(USART1, ENABLE);

    // --- PATCH: Sửa lỗi driver RCC trả về sai Clock ---
    // Tự tính BRR cho 72MHz, 115200 Baud
    // DIV = 72,000,000 / (16 * 115200) = 39.0625
    // Mantissa = 39 (0x27)
    // Fraction = 0.0625 * 16 = 1
    // BRR = 0x271
    USART1->BRR = 0x271; 
}

/* --- FLASH FUNCTIONS --- */
void Flash_WaitBusy(void) { while(FLASH_R->SR & FLASH_SR_BSY); }
void Flash_Unlock(void) { if(FLASH_R->CR & FLASH_CR_LOCK) { FLASH_R->KEYR = FLASH_KEY1; FLASH_R->KEYR = FLASH_KEY2; } }
void Flash_Lock(void) { FLASH_R->CR |= FLASH_CR_LOCK; }
void Flash_ErasePage(uint32_t addr) {
    Flash_WaitBusy(); FLASH_R->CR |= FLASH_CR_PER; FLASH_R->AR = addr; FLASH_R->CR |= FLASH_CR_STRT;
    Flash_WaitBusy(); FLASH_R->CR &= ~FLASH_CR_PER;
}
void Flash_WriteHalfWord(uint32_t addr, uint16_t data) {
    Flash_WaitBusy(); FLASH_R->CR |= FLASH_CR_PG; *(__vo uint16_t*)addr = data;
    Flash_WaitBusy(); FLASH_R->CR &= ~FLASH_CR_PG;
}

/* --- UTILS --- */
void UART_SendByte(uint8_t byte) { 
    // Dùng trực tiếp thanh ghi để tránh phụ thuộc driver loop
    while(!(USART1->SR & (1<<7))); // Wait TXE
    USART1->DR = byte;
    while(!(USART1->SR & (1<<6))); // Wait TC
}

uint8_t UART_ReadByte(void) { 
    while(!(USART1->SR & (1<<5))); // Wait RXNE
    return (uint8_t)(USART1->DR & 0xFF); 
}

typedef void (*pFunction)(void);
void JumpToApplication(void) {
    uint32_t msp = *(__vo uint32_t *)APP_START_ADDRESS;
    uint32_t reset_addr = *(__vo uint32_t *)(APP_START_ADDRESS + 4);
    if ((msp & 0x2FFE0000) == 0x20000000) {
        USART_PeripheralControl(USART1, DISABLE); 
        RCC->APB2ENR &= ~(1 << 14); // Disable UART Clock
        __set_MSP(msp);
        pFunction appReset = (pFunction)reset_addr;
        appReset();
    }
}

int main(void) {
    SystemClock_Setup_72MHz();
    GPIO_Setup();
    USART1_Setup();

    // Nháy LED 6 lần báo hiệu Bootloader OK
    for(int i=0; i<6; i++) {
        GPIO_ToggleOutputPin(GPIOC, LED_PIN);
        for(volatile int d=0; d<200000; d++); 
    }
    GPIO_WriteToOutputPin(GPIOC, LED_PIN, GPIO_PIN_SET); 

    uint8_t rx_buffer[128];
    uint32_t current_addr = APP_START_ADDRESS;
    uint8_t global_xor = 0;

    while(1) {
        // Chờ nhận lệnh
        uint8_t cmd = UART_ReadByte();

        switch(cmd) {
            case CMD_CONNECT: // 0xAA
                current_addr = APP_START_ADDRESS;
                global_xor = 0;
                UART_SendByte(RSP_ACK);
                GPIO_WriteToOutputPin(GPIOC, LED_PIN, GPIO_PIN_RESET); // Sáng LED
                break;

            case CMD_INFO: // 0xBB
                // Nhận 4 byte size
                for(int i=0; i<4; i++) rx_buffer[i] = UART_ReadByte();
                uint32_t size = (rx_buffer[0]<<24)|(rx_buffer[1]<<16)|(rx_buffer[2]<<8)|rx_buffer[3];

                Flash_Unlock();
                uint32_t pages = (size / FLASH_PAGE_SIZE) + ((size % FLASH_PAGE_SIZE)?1:0);
                for(uint32_t i=0; i<pages; i++) {
                    Flash_ErasePage(APP_START_ADDRESS + i*FLASH_PAGE_SIZE);
                    GPIO_ToggleOutputPin(GPIOC, LED_PIN); // Nháy LED
                }
                GPIO_WriteToOutputPin(GPIOC, LED_PIN, GPIO_PIN_RESET);

                UART_SendByte(RSP_ACK);
                break;

            case CMD_DATA: // 0xCC
                uint8_t len = UART_ReadByte();
                for(int i=0; i<len; i++) rx_buffer[i] = UART_ReadByte();
                uint8_t xor_recv = UART_ReadByte();

                uint8_t xor_calc = 0;
                for(int i=0; i<len; i++) { xor_calc ^= rx_buffer[i]; global_xor ^= rx_buffer[i]; }

                if(xor_calc == xor_recv) {
                    for(int i=0; i<len; i+=2) {
                        uint16_t d = rx_buffer[i] | (rx_buffer[i+1] << 8);
                        Flash_WriteHalfWord(current_addr, d);
                        current_addr += 2;
                    }
                    UART_SendByte(RSP_ACK);
                } else UART_SendByte(RSP_NACK);
                break;

            case CMD_END: // 0xDD
                uint8_t g_xor_recv = UART_ReadByte();
                if(g_xor_recv == global_xor) {
                    Flash_Lock();
                    UART_SendByte(RSP_ACK);
                    for(volatile int k=0; k<1000000; k++); 
                    JumpToApplication();
                } else UART_SendByte(RSP_NACK);
                break;
        }
    }
}

/*
 * File: main.c
 * Purpose: Bootloader Logic with 8-byte Packet Protocol & Integrity Check
 */

//#include "../drivers_2/Inc/01-LIB/STD_TYPES.h"
//#include "../drivers_2/Inc/01-LIB/BIT_MATH.h"
//
//#include "../drivers_2/Inc/03-RCC/RCC_interface.h"
//#include "../drivers_2/Inc/02-GPIO/GPIO_interface.h"
//#include "../drivers_2/Inc/05-UART/UART_interface.h"
//#include "../drivers_2/Inc/04-STK/STK_interface.h"
//
///* --- ĐỊNH NGHĨA GIAO THỨC (PROTOCOL) --- */
//#define PACKET_SIZE     8
//#define START_BYTE      0xAA
//
///* Cấu trúc gói tin 8 Bytes */
//typedef struct
//{
//    u8 StartByte;   // Byte 0: Luôn là 0xAA
//    u8 Command;     // Byte 1: Lệnh
//    u8 Length;      // Byte 2: Độ dài dữ liệu
//    u8 Data[4];     // Byte 3-6: Dữ liệu Payload
//    u8 Checksum;    // Byte 7: Kiểm tra toàn vẹn
//} Packet_t;
//
///* --- HÀM HỖ TRỢ --- */
//
///* Tính Checksum đơn giản */
//u8 CalculateChecksum(u8 *buffer)
//{
//    u8 sum = 0;
//    /* Cộng dồn 7 byte đầu tiên (trừ byte checksum ở cuối) */
//    for(u8 i = 0; i < (PACKET_SIZE - 1); i++)
//    {
//        sum += buffer[i];
//    }
//    return sum;
//}
//
///* Gửi 1 gói tin 8 byte */
//void UART_SendPacket(Packet_t *pkt)
//{
//    // 1. Chuyển struct thành mảng byte
//    u8 *raw = (u8*)pkt;
//
//    // 2. Tính lại Checksum
//    pkt->Checksum = CalculateChecksum(raw);
//
//    // 3. Gửi từng byte
//    for(u8 i = 0; i < PACKET_SIZE; i++)
//    {
//        UART_voidTransmit(raw[i]); // Đảm bảo hàm này trùng tên trong UART_interface.h
//
//        // Delay nhẹ (khoảng 1ms) để ESP32 kịp xử lý
//        // Giả sử STK chạy 72MHz/8 = 9MHz -> 1ms = 9000 ticks
//        STK_voidSetBusyWait(9000);
//    }
//}
//
//int main(void)
//{
//    /* 1. Khởi tạo Clock hệ thống */
//    RCC_voidInitSysClock();
//
//    /* 2. Bật Clock ngoại vi */
//    RCC_voidEnableClock(RCC_APB2, APB2_GPIOA_EN);
//    RCC_voidEnableClock(RCC_APB2, APB2_USART1_EN);
//    RCC_voidEnableClock(RCC_APB2, APB2_AFIO_EN);
//
//    /* 3. Cấu hình GPIO UART */
//    // PA9 (TX): Output AF Push-Pull 50MHz
//    GPIO_voidSetPinDirection(GPIOA, PIN9, OUTPUT_SPEED_50MHZ_AFPP);
//    // PA10 (RX): Input Floating
//    GPIO_voidSetPinDirection(GPIOA, PIN10, INPUT_FLOATING);
//
//    /* 4. Khởi tạo UART */
//    UART_voidInit();
//
//    /* 5. Khởi tạo SysTick */
//    STK_voidInit();
//
//    /* Biến lưu trữ */
//    u8 rxBuffer[PACKET_SIZE];
//    Packet_t responsePkt;
//    u8 byteIndex = 0;
//
//    /* Gửi test message */
//    u8 hello[] = "BOOTLOADER START\r\n";
//    for(u8 i=0; i<sizeof(hello)-1; i++) UART_voidTransmit(hello[i]);
//
//    while(1)
//    {
//        /* Nhận 1 byte (Blocking) */
//        rxBuffer[byteIndex] = UART_u8Receive();
//
//        /* Kiểm tra Start Byte */
//        if (byteIndex == 0 && rxBuffer[0] != START_BYTE)
//        {
//            byteIndex = 0;
//            continue;
//        }
//
//        byteIndex++;
//
//        /* Nếu đã nhận đủ gói */
//        if (byteIndex >= PACKET_SIZE)
//        {
//            u8 calculatedCRC = CalculateChecksum(rxBuffer);
//            u8 receivedCRC = rxBuffer[PACKET_SIZE - 1];
//
//            if (calculatedCRC == receivedCRC)
//            {
//                /* CHECK OK */
//                Packet_t *rxPkt = (Packet_t*)rxBuffer;
//
//                responsePkt.StartByte = START_BYTE;
//                responsePkt.Command   = rxPkt->Command;
//                responsePkt.Length    = 1;
//
//                /* [SỬA LỖI] Thay 0xOK bằng 0x01 */
//                responsePkt.Data[0]   = 0x01; // 0x01 = OK Status
//                responsePkt.Data[1]   = 0x00;
//                responsePkt.Data[2]   = 0x00;
//                responsePkt.Data[3]   = 0x00;
//
//                UART_SendPacket(&responsePkt);
//            }
//            else
//            {
//                /* CHECK FAIL */
//                responsePkt.StartByte = START_BYTE;
//                responsePkt.Command   = 0xFF; // Error CMD
//                responsePkt.Length    = 0;
//                responsePkt.Data[0]   = 0xEE; // 0xEE = Error Code
//
//                UART_SendPacket(&responsePkt);
//            }
//
//            // Reset
//            byteIndex = 0;
//        }
//    }
//}
