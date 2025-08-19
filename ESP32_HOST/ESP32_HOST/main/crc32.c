/*
 * crc32.c
 *
 *  Created on: Aug 19, 2025
 *      Author: Richard Acer
 */

#include "esp_crc.h"

uint32_t calculate_crc32(const uint8_t *data, size_t length) {
    return esp_crc32_le(0xFFFFFFFF, data, length);
}

