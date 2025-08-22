/*
 * app_meta.c
 * Functions to read/write metadata flags using custom flash driver
 */

#include "app_meta.h"
#include "stm32f103xx_flash_driver.h"

void app_meta_write(uint32_t addr, const app_meta_t *meta)
{
    FLASH_Erase(addr); // erase the 1KB page
    FLASH_Write_Data(addr, (uint32_t*)meta, sizeof(app_meta_t) / 4);
}

void app_meta_read(uint32_t addr, app_meta_t *meta)
{
    FLASH_Read_Data(addr, (uint32_t*)meta, sizeof(app_meta_t) / 4);
}
