/*
 * app_meta.h
 * Shared metadata structure and constants between Bootloader, APP_CURRENT, and APP_OLD
 */
#ifndef __APP_META_H
#define __APP_META_H

#include <stdint.h>

#define APP_FLAGS_ADDR         0x08011C00UL   // 1KB cuối phân vùng APP_CURRENT
#define APP_VALID_FLAG_VALUE   0x00000001UL   // =1: app hợp lệ
#define APP_VERSION_VALUE      0x00010000UL   // ví dụ v1.0.0

typedef struct __attribute__((packed)) {
    uint32_t valid;    // 1 = hợp lệ
    uint32_t version;  // tuỳ ý
} app_footer_t;

#endif
