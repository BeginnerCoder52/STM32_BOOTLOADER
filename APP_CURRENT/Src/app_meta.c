/*
 * app_meta.c
 * Functions to read/write metadata flags using custom flash driver
 */

#include "app_meta.h"
#include "stm32f103xx_flash_driver.h"

static inline void footer_read(app_footer_t *out)
{
    // đọc trực tiếp flash -> struct
    // driver của bạn có hàm đọc theo word; ở đây đơn giản memcpy từ flash:
    const app_footer_t *rom = (const app_footer_t *)APP_FLAGS_ADDR;
    *out = *rom;
}

static inline void footer_write(const app_footer_t *in)
{
    // Erase nguyên 1KB page tại APP_FLAGS_ADDR, rồi ghi 8 byte đầu
    FLASH_Erase(APP_FLAGS_ADDR);
    FLASH_Write_Data(APP_FLAGS_ADDR, (uint32_t*)in, sizeof(app_footer_t)/4);
}
