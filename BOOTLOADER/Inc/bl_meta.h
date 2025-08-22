#pragma once
#include <stdint.h>

#define FLASH_BASE_ADDR   0x08000000u
#define PAGE_SIZE         1024u

#define BL_START          0x08000000u
#define BL_SIZE           (16u*1024u)
#define BCB_ADDR          (BL_START + BL_SIZE - PAGE_SIZE)

#define CUR_START         0x08004000u
#define CUR_SIZE          (56u*1024u)
#define CUR_FOOTER_ADDR   (CUR_START + CUR_SIZE - PAGE_SIZE)

#define OLD_START         0x08012000u
#define OLD_SIZE          (56u*1024u)
#define OLD_FOOTER_ADDR   (OLD_START + OLD_SIZE - PAGE_SIZE)

#define FOOTER_MAGIC 0x5A5A55AAu
#define BCB_MAGIC    0x424C424Cu

typedef struct __attribute__((packed)) {
  uint32_t magic;     // FOOTER_MAGIC
  uint32_t valid;     // 1 = verified OK
  uint32_t size;      // number of bytes in image to write
  uint32_t crc32;     // CRC32 of the image
  uint32_t version;   // e.g. 0x00030000
} app_footer_t;

typedef enum { PART_CURRENT=0, PART_OLD=1 } part_t;

typedef struct __attribute__((packed)) {
  uint32_t magic;        // BCB_MAGIC
  uint32_t boot_target;  // PART_CURRENT or PART_OLD
  uint32_t pending;      // 1 = pending update to boot
  uint32_t trial_count;  // number of trial boots remaining
} boot_ctrl_t;
