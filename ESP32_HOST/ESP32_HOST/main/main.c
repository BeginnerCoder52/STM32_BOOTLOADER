#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"  // Include UART driver
#include "esp_crc.h"      // Include CRC library
#include "uart_link.h"
#include "spiffs.h"

#define FW_PATH "/spiffs/firmware.bin"
#define CHUNK_SIZE 256
#define PART_CURRENT 0  // Match STM part_t
#define TRIAL_COUNT 3

void app_main(void) {
    uart_init(17, 16, 1, 115200);  // Use 1 instead of UART_NUM_1, as uart_init expects port number
    mount_spiffs();

    FILE* f = fopen(FW_PATH, "rb");
    if (!f) {
        printf("Failed to open firmware file\n");
        vTaskDelete(NULL);
    }

    fseek(f, 0, SEEK_END);
    uint32_t fw_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* buf = malloc(fw_size);
    if (!buf) {
        fclose(f);
        vTaskDelete(NULL);
    }
    fread(buf, 1, fw_size, f);
    fclose(f);

    uint32_t fw_crc = esp_crc32_le(0xFFFFFFFF, buf, fw_size);  // Use esp_crc32_le
    uint32_t version = 0x00010000;  // Example

    send_BEGIN(PART_CURRENT, fw_size, fw_crc, version);
    wait_response('O', 1000);  // Wait FW_OK

    for (uint32_t offset = 0; offset < fw_size; offset += CHUNK_SIZE) {
        uint16_t len = (fw_size - offset < CHUNK_SIZE) ? (fw_size - offset) : CHUNK_SIZE;
        uint8_t cs = 0;
        for (uint16_t i = 0; i < len; i++) cs ^= buf[offset + i];
        send_DATA(offset, &buf[offset], len);
        send_DATA(offset + len, &cs, 1);  // Send checksum after data
        wait_response('A', 1000);  // Wait ACK
    }

    send_END();
    wait_response('O', 1000);  // Assume OK for END

    send_SET_PENDING(PART_CURRENT, TRIAL_COUNT);
    send_REBOOT();

    free(buf);
    vTaskDelete(NULL);
}