#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "bl_meta.h"
#include "stm32f1xx_hal.h"
#include "flash_if.h"

void flash_unlock(void){ HAL_FLASH_Unlock(); }
void flash_lock(void){ HAL_FLASH_Lock(); }

int flash_erase_range(uint32_t addr, uint32_t len){
    FLASH_EraseInitTypeDef e={0}; uint32_t err;
    flash_unlock();
    for(uint32_t a=addr; a<addr+len; a+=PAGE_SIZE){
        e.TypeErase=FLASH_TYPEERASE_PAGES; e.PageAddress=a; e.NbPages=1;
        if(HAL_FLASHEx_Erase(&e,&err)!=HAL_OK){ flash_lock(); return -1; }
    }
    flash_lock(); return 0;
}

int flash_write(uint32_t addr, const uint8_t* data, uint32_t len){
    flash_unlock();
		uint8_t verify_buf[2];
    for(uint32_t i=0;i<len;i+=2){
        uint16_t hw = data[i] | ((i+1<len?data[i+1]:0)<<8);
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr+i, hw)!=HAL_OK){
            flash_lock(); return -1;
        }
				memcpy(verify_buf, (void*)(addr + i), 2);
				if(verify_buf[0] != data[i] || (i+1<len && verify_buf[1] != data[i+1])) return -1;
    }
    flash_lock(); return 0;
}

uint32_t crc32_update(uint32_t crc, uint8_t d){
    crc^=d;
    for(int i=0;i<8;i++)
        crc = (crc&1)? (crc>>1)^0xEDB88320u : (crc>>1);
    return crc;
}

uint32_t crc32_calc(const uint8_t* p, uint32_t n){
    uint32_t crc=0xFFFFFFFFu;
    for(uint32_t i=0;i<n;i++) crc=crc32_update(crc,p[i]);
    return ~crc;
}

void read_bcb(boot_ctrl_t* b){ memcpy(b,(void*)BCB_ADDR,sizeof(*b)); }
void write_bcb(const boot_ctrl_t* b){
    flash_erase_range(BCB_ADDR, PAGE_SIZE);
    flash_write(BCB_ADDR, (const uint8_t*)b, sizeof(*b));
}

void read_footer(uint32_t addr, app_footer_t* f){
    memcpy(f,(void*)addr,sizeof(*f));
}

void write_footer(uint32_t addr, const app_footer_t* f){
    flash_erase_range(addr, PAGE_SIZE);
    flash_write(addr, (const uint8_t*)f, sizeof(*f));
}

