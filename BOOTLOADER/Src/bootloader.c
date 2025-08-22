/*
 * Robust two-slot bootloader for STM32F103C8T6
 * - Blocking UART protocol with command IDs
 * - Length check, CRC32 check (whole image)
 * - Copy Current -> Old before update; restore on failure
 * - Footer (magic, valid, size, crc32, version), BCB (boot target/pending/trials)
 */

#include "bootloader.h"
#include "flash_if.h"
#include "bl_meta.h"
#include "stm32f103xx.h"

// ---------------- Protocol IDs ----------------
enum {
    CMD_FW_ERR        = 0,
    CMD_FW_OK         = 1,
    CMD_FW_REQUEST    = 2,
    CMD_FW_LENGTH     = 3,
    CMD_FW_RECEIVED   = 4,
    CMD_CHECKSUM_DATA = 5,
    CMD_CHECKSUM_OK   = 6,
    CMD_CHECKSUM_ERR  = 7
};

// ---------------- Tunables ----------------
//#define CHUNK_SIZE        256U
#define UART_BAUD         115200U
#define MAX_IMAGE_SIZE    (CUR_SIZE - PAGE_SIZE) // 55 KB

// Minimal SCB for VTOR
typedef struct {
    volatile uint32_t CPUID;
    volatile uint32_t ICSR;
    volatile uint32_t VTOR;
    volatile uint32_t AIRCR;
    volatile uint32_t SCR;
    volatile uint32_t CCR;
    volatile uint8_t  SHP[12];
    volatile uint32_t SHCSR;
} SCB_Type;
#define SCB ((SCB_Type*)0xE000ED00UL)

// ---------------- CRC32 (bitwise, streaming) ----------------
static uint32_t crc32_update(uint32_t crc, const uint8_t* p, uint32_t n) {
    crc ^= 0xFFFFFFFFu;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1U) crc = (crc >> 1) ^ 0xEDB88320u;
            else          crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------- Partition helpers ----------------
static inline uint32_t part_base(part_t p) {
    return (p == PART_CURRENT) ? CUR_START : OLD_START;
}

static inline uint32_t part_footer(part_t p) {
    return (p == PART_CURRENT) ? CUR_FOOTER_ADDR : OLD_FOOTER_ADDR;
}

static int read_valid_footer(part_t p, app_footer_t* f) {
    read_footer(part_footer(p), f);
    if (f->magic != FOOTER_MAGIC || f->valid == 0) return 0;
    return 1;
}

static int is_partition_valid(part_t p) {
    app_footer_t f;
    return read_valid_footer(p, &f);
}

static void invalidate_footer(part_t p) {
    app_footer_t f = {0};
    write_footer(part_footer(p), &f); // clear magic/valid
}

// Copy used image bytes from src to dst (size from src footer)
static int copy_partition(part_t src, part_t dst) {
    app_footer_t fs;
    if (!read_valid_footer(src, &fs)) return -1;

    uint32_t src_addr = part_base(src);
    uint32_t dst_addr = part_base(dst);
    uint32_t size     = fs.size;

    // Erase destination app region (excluding footer page at end)
    if (flash_erase_range(dst_addr, (dst == PART_CURRENT ? CUR_SIZE : OLD_SIZE) - PAGE_SIZE) != 0) return -1;

    uint8_t buf[CHUNK_SIZE];
    uint32_t remaining = size;
    while (remaining) {
        uint32_t len = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        for (uint32_t i = 0; i < len; i++) buf[i] = *(volatile uint8_t*)(src_addr + i);
        if (flash_write(dst_addr, buf, len) != 0) return -1;
        src_addr += len;
        dst_addr += len;
        remaining -= len;
    }

    // Copy footer (preserve validity)
    write_footer(part_footer(dst), &fs);
    return 0;
}

// ---------------- UART helpers (blocking) ----------------
USART_Handle_t husart1 = {
    .pUSARTx = USART1,
    .USART_Config = {
        .USART_Mode              = USART_MODE_TXRX,
        .USART_Baudrate          = USART_STD_BAUD_115200,
        .USART_NumberOfStopBits  = USART_STOPBITS_1,
        .USART_WordLength        = USART_WORDLEN_8BITS,
        .USART_ParityControl     = USART_PARITY_DISABLE,
        .USART_HWFLowControl     = USART_HW_FLOW_CTRL_NONE
    }
};

static void uart1_pins_init(void)
{
    GPIO_Handle_t hgpio;

    // TX: PA9, AF push-pull, 50 MHz
    hgpio.pGPIOx = GPIOA;
    hgpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_9;
    hgpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    hgpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_TX_FULLDUP; // AF out push-pull
    hgpio.GPIO_PinConfig.GPIO_PinSpeed       = GPIO_SPEED_FAST;                // 50 MHz
    hgpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;                              // not used for AF PP
    GPIO_Init(&hgpio);

    // RX: PA10, input floating (no pull)
    hgpio.pGPIOx = GPIOA;
    hgpio.GPIO_PinConfig.GPIO_PinNumber      = GPIO_PIN_NO_10;
    hgpio.GPIO_PinConfig.GPIO_PinMode        = GPIO_MODE_ALTFN;
    hgpio.GPIO_PinConfig.GPIO_PinAltFunMode  = GPIO_ALT_MODE_USART_RX_FULLDUP; // AF input floating
    hgpio.GPIO_PinConfig.GPIO_PinSpeed       = 0;                               // ignored for input
    hgpio.GPIO_PinConfig.GPIO_PinPuPdControl = 0;                               // no pull
    GPIO_Init(&hgpio);
}


static void uart1_init(void) {
    uart1_pins_init();

    husart1.pUSARTx = USART1;
    husart1.USART_Config.USART_Mode            = USART_MODE_TXRX;
    husart1.USART_Config.USART_Baudrate        = UART_BAUD;
    husart1.USART_Config.USART_WordLength      = USART_WORDLEN_8BITS;
    husart1.USART_Config.USART_ParityControl   = USART_PARITY_DISABLE;
    husart1.USART_Config.USART_NumberOfStopBits= USART_STOPBITS_1;
    husart1.USART_Config.USART_HWFLowControl   = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&husart1);
    USART_Start(husart1.pUSARTx);
}

static void uart_write_bytes(const uint8_t* p, uint32_t n) {
    USART_SendData(&husart1, (uint8_t*)p, n);
}

static void uart_write_u8(uint8_t b) {
    uart_write_bytes(&b, 1);
}

static void uart_read_bytes(uint8_t* p, uint32_t n) {
    USART_ReceiveData(&husart1, p, n);
}

static uint8_t uart_read_u8(void) {
    uint8_t b;
    uart_read_bytes(&b, 1);
    return b;
}

// Try read a byte with a crude timeout loop (approx, HSI 8MHz)
static int uart_try_read_u8(uint8_t* out, uint32_t spin_max) {
    while (spin_max--) {
        if ((husart1.pUSARTx->SR >> USART_SR_RXNE) & 1) {
            *out = (uint8_t)(husart1.pUSARTx->DR & 0xFF);
            return 1;
        }
    }
    return 0;
}

// ---------------- Jump to app ----------------
static void jump_to_app(part_t p) {
    uint32_t app_addr = part_base(p);
    uint32_t sp = *(volatile uint32_t*)(app_addr + 0);
    uint32_t rv = *(volatile uint32_t*)(app_addr + 4);

    // Deinit clocks and peripherals
    USART_Stop(husart1.pUSARTx);
    RCC_DeInit();

    // Relocate vector table
    SCB->VTOR = app_addr;

    // Disable interrupts
    __asm volatile ("cpsid i");

    // Set MSP and jump
    __asm volatile ("msr msp, %0" : : "r" (sp) : );
    ((void (*)(void))rv)();
    while(1);
}

// ---------------- OTA receive ----------------
static int receive_firmware(uint32_t fw_len, uint32_t* out_crc32) {
    // Length check
    if (fw_len == 0 || fw_len > MAX_IMAGE_SIZE) {
        uart_write_u8(CMD_FW_ERR);
        return -1;
    }

    // If Current valid, back it up to Old
    if (is_partition_valid(PART_CURRENT)) {
        if (copy_partition(PART_CURRENT, PART_OLD) != 0) {
            uart_write_u8(CMD_FW_ERR);
            return -1;
        }
    }

    // Prepare Current for new image
    // Invalidate footer first (so power-loss leaves it invalid)
    invalidate_footer(PART_CURRENT);
    if (flash_erase_range(CUR_START, CUR_SIZE - PAGE_SIZE) != 0) {
        uart_write_u8(CMD_FW_ERR);
        return -1;
    }

    uart_write_u8(CMD_FW_OK);

    uint8_t buf[CHUNK_SIZE];
    uint32_t addr = CUR_START;
    uint32_t remaining = fw_len;
    uint32_t crc = 0;

    while (remaining > 0) {
        uint32_t chunk = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;
        uart_read_bytes(buf, chunk);
        crc = crc32_update(crc, buf, chunk);

        if (flash_write(addr, buf, chunk) != 0) {
            // Restore from Old if we have a valid Old
            if (is_partition_valid(PART_OLD)) {
                copy_partition(PART_OLD, PART_CURRENT);
            }
            uart_write_u8(CMD_FW_ERR);
            return -1;
        }

        addr += chunk;
        remaining -= chunk;

        // Acknowledge chunk
        uart_write_u8(CMD_FW_RECEIVED);
    }

    // Expect checksum command next
    uint8_t cmd = uart_read_u8();
    if (cmd != CMD_CHECKSUM_DATA) {
        uart_write_u8(CMD_FW_ERR);
        return -1;
    }

    uint8_t crcraw[4];
    uart_read_bytes(crcraw, 4);
    uint32_t fw_crc = (uint32_t)crcraw[0] |
                      ((uint32_t)crcraw[1] << 8) |
                      ((uint32_t)crcraw[2] << 16) |
                      ((uint32_t)crcraw[3] << 24);

    *out_crc32 = crc;

    if (crc != fw_crc) {
        // Bad CRC: erase broken Current and restore from Old
        flash_erase_range(CUR_START, CUR_SIZE - PAGE_SIZE);
        if (is_partition_valid(PART_OLD)) {
            copy_partition(PART_OLD, PART_CURRENT);
        }
        uart_write_u8(CMD_CHECKSUM_ERR);
        return -1;
    }

    uart_write_u8(CMD_CHECKSUM_OK);

    // Finalize: write a valid footer
    app_footer_t foot = {
        .magic   = FOOTER_MAGIC,
        .valid   = 1,
        .size    = fw_len,
        .crc32   = fw_crc,
        .version = 0x00010000u
    };
    write_footer(CUR_FOOTER_ADDR, &foot);

    // Mark BCB pending/trial (optional: app should clear pending after self-test)
    boot_ctrl_t bcb;
    read_bcb(&bcb);
    if (bcb.magic != BCB_MAGIC) {
        bcb.magic = BCB_MAGIC;
    }
    bcb.boot_target = PART_CURRENT;
    bcb.pending     = 1;
    bcb.trial_count = 3;
    write_bcb(&bcb);

    return 0;
}

// ---------------- Boot selection ----------------
static void ensure_bcb(void) {
    boot_ctrl_t bcb;
    read_bcb(&bcb);
    if (bcb.magic != BCB_MAGIC) {
        bcb.magic = BCB_MAGIC;
        bcb.boot_target = PART_CURRENT;
        bcb.pending = 0;
        bcb.trial_count = 0;
        write_bcb(&bcb);
    }
}

void bootloader_main(void) {
    // Keep default HSI/8MHz. Init UART.
    uart1_init();
    ensure_bcb();

    // Optional small window to accept immediate FW_REQUEST
    uint8_t first;
    if (uart_try_read_u8(&first, 800000)) { // rough spin timeout
        if (first == CMD_FW_REQUEST) {
            // Expect FW_LENGTH + 4 bytes
            uint8_t cmd = uart_read_u8();
            if (cmd != CMD_FW_LENGTH) {
                uart_write_u8(CMD_FW_ERR);
            } else {
                uint8_t lenraw[4];
                uart_read_bytes(lenraw, 4);
                uint32_t fw_len = (uint32_t)lenraw[0] |
                                  ((uint32_t)lenraw[1] << 8) |
                                  ((uint32_t)lenraw[2] << 16) |
                                  ((uint32_t)lenraw[3] << 24);
                uint32_t crc;
                if (receive_firmware(fw_len, &crc) == 0) {
                    // Successful OTA: jump to Current
                    jump_to_app(PART_CURRENT);
                }
            }
        }
    }

    // No OTA request (or OTA failed): normal boot selection
    if (is_partition_valid(PART_CURRENT)) {
        jump_to_app(PART_CURRENT);
    } else if (is_partition_valid(PART_OLD)) {
        jump_to_app(PART_OLD);
    } else {
        // Stay in OTA mode: wait indefinitely for a proper request
        while (1) {
            uint8_t b = uart_read_u8();
            if (b == CMD_FW_REQUEST) {
                uint8_t cmd = uart_read_u8();
                if (cmd == CMD_FW_LENGTH) {
                    uint8_t lenraw[4];
                    uart_read_bytes(lenraw, 4);
                    uint32_t fw_len = (uint32_t)lenraw[0] |
                                      ((uint32_t)lenraw[1] << 8) |
                                      ((uint32_t)lenraw[2] << 16) |
                                      ((uint32_t)lenraw[3] << 24);
                    uint32_t crc;
                    if (receive_firmware(fw_len, &crc) == 0) {
                        jump_to_app(PART_CURRENT);
                    }
                } else {
                    uart_write_u8(CMD_FW_ERR);
                }
            }
        }
    }
}
