
#include "flash_if.h"
#include "stm32f103xx.h"

void flash_unlock(void) {
    FLASH->CR &= ~(1 << FLASH_CR_LOCK);
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

void flash_lock(void) {
    FLASH->CR |= (1 << FLASH_CR_LOCK);
}

int flash_erase_range(uint32_t addr, uint32_t len) {
    uint32_t page_addr = addr & ~(PAGE_SIZE - 1);
    while (page_addr < addr + len) {
        FLASH_Erase(page_addr);
        page_addr += PAGE_SIZE;
    }
    return FLASH_OK;
}

int flash_write(uint32_t addr, const uint8_t* data, uint32_t len) {
    // Convert uint8_t* to uint32_t* and adjust length for 32-bit writes
    uint32_t* data32 = (uint32_t*)data;
    uint16_t len32 = len / 4 + (len % 4 ? 1 : 0);
    return FLASH_Write_Data(addr, data32, len32) == FLASH_OK ? 0 : -1;
}

uint32_t crc32_calc(const uint8_t* p, uint32_t n) {
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return ~crc;
}

void read_bcb(boot_ctrl_t* b) {
    *b = *(boot_ctrl_t*)BCB_ADDR;
}

void write_bcb(const boot_ctrl_t* b) {
    flash_unlock();
    flash_write(BCB_ADDR, (const uint8_t*)b, sizeof(boot_ctrl_t));
    flash_lock();
}

void read_footer(uint32_t addr, app_footer_t* f) {
    *f = *(app_footer_t*)addr;
}

void write_footer(uint32_t addr, const app_footer_t* f) {
    flash_unlock();
    flash_write(addr, (const uint8_t*)f, sizeof(app_footer_t));
    flash_lock();
}