//#include "bootloader.h"
//#include "flash_if.h"
//#include "bl_meta.h"
//#include <string.h>
//#include "stm32f103xx_usart_driver.h"
//#include "stm32f103xx_flash_driver.h"

//extern UART_HandleTypeDef huart1;
//uint32_t crc32_update(uint32_t crc, uint8_t d);
//static void copy_partition(part_t src, part_t dst);

////======= FUNCTION TO CHECK IF PARTITION IS VALID =======
//static int is_partition_valid(part_t p) {
//    app_footer_t f;
//    uint32_t footer_addr = (p==PART_CURRENT)?CUR_FOOTER_ADDR:OLD_FOOTER_ADDR;
//    memcpy(&f, (void*)footer_addr, sizeof(f));
//    if(f.magic != FOOTER_MAGIC || f.valid == 0) return 0; // Invalid
//    return 1;
//}

//// ======= FUNCTION TO JUMP TO APP =======
//typedef void (*pFunction)(void);
//static uint32_t part_base(part_t p) {
//    return (p==PART_CURRENT)?CUR_START:OLD_START;
//}
//static void jump_to_app(part_t p) {
//    uint32_t app_addr = part_base(p);
//    uint32_t sp = *(volatile uint32_t*)(app_addr);
//    uint32_t rv = *(volatile uint32_t*)(app_addr+4);

//    HAL_RCC_DeInit();
//    HAL_DeInit();
//    SysTick->CTRL = 0;

//    __disable_irq();
//    __set_MSP(sp);
//    SCB->VTOR = app_addr;
//    __enable_irq();

//    ((pFunction)rv)(); // Jump into app
//}

//// ======= MAIN BOOTLOADER FUNCTION =======
//void bootloader_main(void) {
//    boot_ctrl_t bcb;
//    memcpy(&bcb, (void*)BCB_ADDR, sizeof(bcb));

//    if(bcb.magic != BCB_MAGIC) {
//        bcb.magic = BCB_MAGIC;
//        bcb.pending = 0;
//        bcb.boot_target = PART_CURRENT;
//        bcb.trial_count = 0;
//        flash_erase_range(BCB_ADDR, PAGE_SIZE);
//        flash_write(BCB_ADDR, (uint8_t*)&bcb, sizeof(bcb));
//    }

//    part_t target = bcb.pending ? (part_t)bcb.boot_target : PART_CURRENT;

//    if(is_partition_valid(target)) {
//        if(bcb.pending && bcb.trial_count > 0) {
//            bcb.trial_count--;
//            flash_erase_range(BCB_ADDR, PAGE_SIZE);
//            flash_write(BCB_ADDR, (uint8_t*)&bcb, sizeof(bcb));
//        }
//        jump_to_app(target);
//    }
//		
//		if (bcb.pending && bcb.trial_count == 0) {
//				bcb.pending = 0;
//				bcb.boot_target = PART_OLD;  // Assume failure on CURRENT, fallback to OLD
//				write_bcb(&bcb);
//		}
//		
//    part_t other = (target==PART_CURRENT)?PART_OLD:PART_CURRENT;
//    if(is_partition_valid(other)) {
//        bcb.pending = 0;
//        bcb.trial_count = 0;
//        bcb.boot_target = other;
//        flash_erase_range(BCB_ADDR, PAGE_SIZE);
//        flash_write(BCB_ADDR, (uint8_t*)&bcb, sizeof(bcb));
//        jump_to_app(other);
//    }

//    // If both apps are invalid, wait for OTA from ESP32
//    while(1) {
//        ota_receive_loop(); 
//    }
//}

//void ota_receive_loop(void) {
//	uint8_t cmd;
//	uint32_t fw_size, fw_crc;
//	uint8_t buf[CHUNK_SIZE + 1];  // +1 for chunk checksum
//	while(1) {
//			HAL_UART_Receive(&huart1, &cmd, 1, HAL_MAX_DELAY);
//			if(cmd != FW_REQUEST) continue;
//			HAL_UART_Receive(&huart1, (uint8_t*)&fw_size, 4, HAL_MAX_DELAY);
//			HAL_UART_Receive(&huart1, (uint8_t*)&fw_crc, 4, HAL_MAX_DELAY);
//			if(fw_size > MAX_FW_SIZE) {
//					uint8_t err = FW_ERR;
//					HAL_UART_Transmit(&huart1, &err, 1, HAL_MAX_DELAY);
//					continue;  // Keep old app
//			}
//			uint8_t ok = FW_OK;
//			HAL_UART_Transmit(&huart1, &ok, 1, HAL_MAX_DELAY);
//			flash_erase_range(CUR_START, CUR_SIZE - PAGE_SIZE);
//			uint32_t addr = CUR_START;
//			uint32_t received_crc = 0xFFFFFFFFu;  // Init for crc32_calc
//			for(uint32_t i = 0; i < fw_size; i += CHUNK_SIZE) {
//					uint32_t len = (fw_size - i < CHUNK_SIZE) ? (fw_size - i) : CHUNK_SIZE;
//					HAL_UART_Receive(&huart1, buf, len + 1, HAL_MAX_DELAY);  // Data + checksum
//					uint8_t chunk_cs = 0;
//					for(uint32_t j = 0; j < len; j++) chunk_cs ^= buf[j];
//					if(chunk_cs != buf[len]) {
//							uint8_t err = FW_ERR;
//							HAL_UART_Transmit(&huart1, &err, 1, HAL_MAX_DELAY);
//							return;  // Abort, keep old
//					}
//					for(uint32_t j = 0; j < len; j++) received_crc = crc32_update(received_crc, buf[j]);
//					flash_write(addr, buf, len);
//					if (flash_write(addr, buf, len) == -1) {
//							uint8_t err = FW_ERR;
//							HAL_UART_Transmit(&huart1, &err, 1, HAL_MAX_DELAY);
//							flash_erase_range(CUR_START, CUR_SIZE);
//							copy_partition(PART_OLD, PART_CURRENT);
//							return;
//					}
//					uint8_t ack = FW_ACK;
//					HAL_UART_Transmit(&huart1, &ack, 1, HAL_MAX_DELAY);
//					addr += len;
//			}
//			received_crc = ~received_crc;
//			if(received_crc != fw_crc) {
//					uint8_t err = FW_ERR;
//					HAL_UART_Transmit(&huart1, &err, 1, HAL_MAX_DELAY);
//					return;  // Keep old
//			}
//			app_footer_t footer = {FOOTER_MAGIC, 1, fw_size, fw_crc, 0x00010000};  // Version example
//			write_footer(CUR_FOOTER_ADDR, &footer);
//			// Set pending for FR2
//	}
//	
//	boot_ctrl_t bcb;
//	read_bcb(&bcb);
//	bcb.pending = 1;
//	bcb.boot_target = PART_CURRENT;
//	bcb.trial_count = 3;  // e.g., 3 trials
//	write_bcb(&bcb);
//	HAL_NVIC_SystemReset();  // Reset to boot new
//}

//static void copy_partition(part_t src, part_t dst) {
//    uint32_t src_addr = part_base(src);
//    uint32_t dst_addr = part_base(dst);
//    uint32_t size = (src == PART_CURRENT) ? CUR_SIZE - PAGE_SIZE : OLD_SIZE - PAGE_SIZE;
//    flash_erase_range(dst_addr, size);
//    uint8_t buf[PAGE_SIZE];
//    for(uint32_t i = 0; i < size; i += PAGE_SIZE) {
//        memcpy(buf, (void*)(src_addr + i), PAGE_SIZE);
//        flash_write(dst_addr + i, buf, PAGE_SIZE);
//    }
//    app_footer_t f;
//    read_footer((src == PART_CURRENT) ? CUR_FOOTER_ADDR : OLD_FOOTER_ADDR, &f);
//    write_footer((dst == PART_CURRENT) ? CUR_FOOTER_ADDR : OLD_FOOTER_ADDR, &f);
//}

#include "bootloader.h"
#include "stm32f103xx_usart_driver.h"
#include "stm32f103xx_flash_driver.h"

#define CHUNK_SIZE 256
void NVIC_SystemReset(void);

static uint32_t part_base(part_t p) {
    return (p == PART_CURRENT) ? CUR_START : OLD_START;
}

void bootloader_main(void) {
    uint8_t buf[CHUNK_SIZE + 1];
    boot_ctrl_t bcb;
    read_bcb(&bcb);

    if (bcb.pending) {
        app_footer_t footer;
        read_footer((bcb.boot_target == PART_CURRENT) ? CUR_FOOTER_ADDR : OLD_FOOTER_ADDR, &footer);
        if (footer.valid) {
            bcb.pending = 0;
            bcb.trial_count = 0;
            write_bcb(&bcb);
            NVIC_SystemReset();
        }
    } else {
        uint32_t fw_size, fw_crc;
        USART_Handle_t usart1 = {USART1, {0}};
        USART_ReceiveData(&usart1, (uint8_t*)&fw_size, sizeof(fw_size));
        USART_ReceiveData(&usart1, (uint8_t*)&fw_crc, sizeof(fw_crc));

        flash_erase_range(CUR_START, CUR_SIZE - PAGE_SIZE);
        uint32_t addr = CUR_START;
        uint32_t received_crc = 0xFFFFFFFF;
        for (uint32_t i = 0; i < fw_size; i += CHUNK_SIZE) {
            uint32_t len = (fw_size - i < CHUNK_SIZE) ? (fw_size - i) : CHUNK_SIZE;
            USART_ReceiveData(&usart1, buf, len + 1);
            uint8_t chunk_cs = 0;
            for (uint32_t j = 0; j < len; j++) chunk_cs ^= buf[j];
            if (chunk_cs != buf[len]) {
                uint8_t err = 'E';
                USART_SendData(&usart1, &err, 1);
                return;
            }
            for (uint32_t j = 0; j < len; j++) received_crc = crc32_calc(&buf[j], 1);
            if (flash_write(addr, buf, len) == -1) {
                uint8_t err = 'E';
                USART_SendData(&usart1, &err, 1);
                flash_erase_range(CUR_START, CUR_SIZE);
                return;
            }
            uint8_t ack = 'A';
            USART_SendData(&usart1, &ack, 1);
            addr += len;
        }
        received_crc = ~received_crc;
        if (received_crc != fw_crc) {
            uint8_t err = 'E';
            USART_SendData(&usart1, &err, 1);
            return;
        }
        app_footer_t footer = {FOOTER_MAGIC, 1, fw_size, fw_crc, 0x00010000};
        write_footer(CUR_FOOTER_ADDR, &footer);
        bcb.pending = 1;
        bcb.boot_target = PART_CURRENT;
        bcb.trial_count = 3;
        write_bcb(&bcb);
        NVIC_SystemReset();
    }
}

