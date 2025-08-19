#ifndef __FLASH_IF_H
#define __FLASH_IF_H

#include <stdint.h>
#include "bl_meta.h"

#ifdef __cplusplus
extern "C" {
#endif

void flash_unlock(void);
void flash_lock(void);
int flash_erase_range(uint32_t addr, uint32_t len);
int flash_write(uint32_t addr, const uint8_t* data, uint32_t len);

uint32_t crc32_calc(const uint8_t* p, uint32_t n);

void read_bcb(boot_ctrl_t* b);
void write_bcb(const boot_ctrl_t* b);
void read_footer(uint32_t addr, app_footer_t* f);
void write_footer(uint32_t addr, const app_footer_t* f);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_IF_H */
