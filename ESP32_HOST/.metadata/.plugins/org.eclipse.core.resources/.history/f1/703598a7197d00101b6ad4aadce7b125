#include <stdio.h>
#include <string.h>
#include "uart_link.h"
#include "esp_crc.h"
#include "spiffs.h"  // Add

#define FW_PATH "/spiffs/firmware.bin"
#define CHUNK_SIZE 256
#define PART_CURRENT 0  // Match STM part_t
#define TRIAL_COUNT 3

void app_main(void) {
    uart_init(17, 16, UART_NUM_1, 115200);  // Example pins, adjust
    mount_spiffs();

    FILE* f = fopen(FW_PATH, "rb");
    if (!f) return;

    fseek(f, 0, SEEK_END);
    uint32_t fw_size = ftell(f);
    fseek(f, 0, SEEK_SET);

    uint8_t* buf = malloc(fw_size);
    fread(buf, 1, fw_size, f);
    fclose(f);

    uint32_t fw_crc = calculate_crc32(buf, fw_size);
    uint32_t version = 0x00010000;  // Example

    send_BEGIN(PART_CURRENT, fw_size, fw_crc, version);
    wait_response('O', 1000);  // Wait FW_OK

    for (uint32_t offset = 0; offset < fw_size; offset += CHUNK_SIZE) {
        uint16_t len = (fw_size - offset < CHUNK_SIZE) ? (fw_size - offset) : CHUNK_SIZE;
        uint8_t cs = 0;
        for (uint16_t i = 0; i < len; i++) cs ^= buf[offset + i];
        send_DATA(offset, &buf[offset], len);
        send_DATA(offset + len, &cs, 1);  // Send checksum after data (adjust send_DATA if needed)
        wait_response('A', 1000);  // Wait ACK
    }

    send_END();
    wait_response('O', 1000);  // Assume OK for END

    send_SET_PENDING(PART_CURRENT, TRIAL_COUNT);
    send_REBOOT();

    free(buf);
}