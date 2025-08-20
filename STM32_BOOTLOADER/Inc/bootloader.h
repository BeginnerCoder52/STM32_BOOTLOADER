#ifndef __BOOTLOADER_H
#define __BOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f103xx.h"
#include "stm32f103xx_flash_driver.h"
#include <stdint.h>
#include <stdbool.h>
#include "bl_meta.h"     // This file contains metadata definitions for the bootloader
#include "flash_if.h"    // This file contains flash interface functions
    
#define FW_REQUEST 'F'
#define FW_OK 'O'
#define FW_ERR 'E'
#define FW_ACK 'A'
#define CHUNK_SIZE 256
#define MAX_FW_SIZE (CUR_SIZE - PAGE_SIZE)  // 55KB
// ====== Define partition enum ======
//typedef enum {
//    PART_CURRENT = 0,
//    PART_OLD     = 1
//} part_t;
// da co o bl_meta.h

// ====== Main bootloader function ======
void bootloader_main(void);

// ====== OTA function (firmware update via UART/SPI) ======
void ota_receive_loop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BOOTLOADER_H */
