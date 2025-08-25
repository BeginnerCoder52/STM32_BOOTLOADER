#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include "stm32f103xx.h"
#include "bl_meta.h"

void flash_unlock(void);
void flash_lock(void);
int flash_erase_range(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, const uint8_t* data, uint32_t len);
uint32_t calculate_checksum(const uint8_t* data, uint32_t len);
void write_footer(app_footer_t* footer);
void read_footer(app_footer_t* footer);

#endif
