#ifndef OTA_CONFIG_H_
#define OTA_CONFIG_H_

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* --- WIFI CONFIG --- */
#define ESP_WIFI_SSID       "STM32_OTA_HOST"
#define ESP_WIFI_PASS       "12345678"
#define MAX_STA_CONN        4

/* --- UART CONFIG --- */
// Đảm bảo chân 16, 17 dùng được trên board của bạn (WROOM)
#define TXD_PIN             (17) 
#define RXD_PIN             (16)
#define UART_PORT_NUM       UART_NUM_2
#define BAUD_RATE           115200
#define BUF_SIZE            1024

/* --- PROTOCOL DEFINITIONS (HEX) --- */
#define CMD_CONNECT         0xAA 
#define CMD_INFO            0xBB 
#define CMD_DATA            0xCC 
#define CMD_END             0xDD 

#define RSP_ACK             0x06
#define RSP_NACK            0x15

#define PACKET_SIZE         32 // Gửi mỗi lần 32 byte cho ổn định

/* --- FUNCTION PROTOTYPES --- */
void uart_init_custom(void);
esp_err_t start_ota_update(uint8_t *data, size_t len);

#endif