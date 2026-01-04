// #include "driver/uart.h"
// #include "esp_log.h"
// #include "ota_config.h"
// #include <string.h>

// static const char *TAG = "UART_OTA";

// void uart_init_custom(void) {
//     const uart_config_t uart_config = {
//         .baud_rate = BAUD_RATE,
//         .data_bits = UART_DATA_8_BITS,
//         .parity = UART_PARITY_DISABLE,
//         .stop_bits = UART_STOP_BITS_1,
//         .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
//         .source_clk = UART_SCLK_APB,
//     };
//     uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
//     uart_param_config(UART_PORT_NUM, &uart_config);
//     uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
//     ESP_LOGI(TAG, "UART Initialized");
// }

// esp_err_t start_ota_update(uint8_t *data, size_t length) {
//     uint8_t rx_byte;
//     uint8_t global_xor = 0;
    
//     ESP_LOGI(TAG, "Starting OTA... Size: %d bytes", length);

//     /* --- STEP 1: CONNECT --- */
//     uint8_t cmd = CMD_CONNECT;
//     uart_write_bytes(UART_PORT_NUM, (const char*)&cmd, 1);
    
//     // Chờ ACK kết nối
//     int len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(2000));
//     if (len <= 0 || rx_byte != RSP_ACK) {
//         ESP_LOGE(TAG, "Connection Failed! No ACK from STM32.");
//         return ESP_FAIL;
//     }

//     /* --- STEP 2: SEND INFO (SIZE) --- */
//     cmd = CMD_INFO;
//     uart_write_bytes(UART_PORT_NUM, (const char*)&cmd, 1);
    
//     uint8_t size_buf[4];
//     size_buf[0] = (length >> 24) & 0xFF;
//     size_buf[1] = (length >> 16) & 0xFF;
//     size_buf[2] = (length >> 8) & 0xFF;
//     size_buf[3] = (length) & 0xFF;
//     uart_write_bytes(UART_PORT_NUM, (const char*)size_buf, 4);

//     // Chờ STM32 xóa Flash (cần thời gian lâu hơn chút)
//     len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(5000));
//     if (len <= 0 || rx_byte != RSP_ACK) {
//         ESP_LOGE(TAG, "Info/Erase Failed! No ACK.");
//         return ESP_FAIL;
//     }

//     /* --- STEP 3: SEND DATA PACKETS --- */
//     size_t offset = 0;
//     int retry_count = 0;

//     while (offset < length) {
//         size_t chunk_len = (length - offset > PACKET_SIZE) ? PACKET_SIZE : (length - offset);
//         uint8_t local_xor = 0;
        
//         // Header: 'D' + Length
//         uint8_t header[2] = { CMD_DATA, (uint8_t)chunk_len };
//         uart_write_bytes(UART_PORT_NUM, (const char*)header, 2);

//         // Tính XOR và gửi Data
//         for (size_t i = 0; i < chunk_len; i++) {
//             uint8_t byte = data[offset + i];
//             local_xor ^= byte;
//             if(retry_count == 0) global_xor ^= byte; // Chỉ cộng vào global lần đầu, không cộng khi retry
//         }

//         uart_write_bytes(UART_PORT_NUM, (const char*)&data[offset], chunk_len);
//         uart_write_bytes(UART_PORT_NUM, (const char*)&local_xor, 1);

//         // Chờ ACK cho gói tin
//         len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(500));
        
//         if (len > 0 && rx_byte == RSP_ACK) {
//             offset += chunk_len;
//             retry_count = 0;
//             if(offset % 128 == 0) ESP_LOGI(TAG, "Progress: %d / %d", offset, length);
//         } else {
//             retry_count++;
//             ESP_LOGW(TAG, "NACK/Timeout at offset %d. Retrying (%d)...", offset, retry_count);
//             if(retry_count > 5) return ESP_FAIL; // Thử lại 5 lần không được thì hủy
//         }
//     }

//     /* --- STEP 4: SEND END --- */
//     cmd = CMD_END;
//     uart_write_bytes(UART_PORT_NUM, (const char*)&cmd, 1);
//     uart_write_bytes(UART_PORT_NUM, (const char*)&global_xor, 1);

//     len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(2000));
//     if (len > 0 && rx_byte == RSP_ACK) {
//         ESP_LOGI(TAG, "OTA Update Successful!");
//         return ESP_OK;
//     }

//     ESP_LOGE(TAG, "Final Checksum Failed!");
//     return ESP_FAIL;
// }

#include "driver/uart.h"
#include "esp_log.h"
#include "ota_config.h"
#include <string.h>

static const char *TAG = "UART_OTA";

void uart_init_custom(void) {
    const uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    // Tăng buffer lên để tránh tràn
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);
    uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_LOGI(TAG, "UART Init: TX=%d, RX=%d, Baud=%d", TXD_PIN, RXD_PIN, BAUD_RATE);
}

// Hàm tiện ích: Gửi 1 byte
void send_byte(uint8_t byte) {
    uart_write_bytes(UART_PORT_NUM, (const char*)&byte, 1);
}

// Hàm tiện ích: Đọc 1 byte với Timeout
esp_err_t wait_for_ack(uint32_t timeout_ms) {
    uint8_t rx_byte = 0;
    int len = uart_read_bytes(UART_PORT_NUM, &rx_byte, 1, pdMS_TO_TICKS(timeout_ms));
    if (len > 0 && rx_byte == RSP_ACK) {
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t start_ota_update(uint8_t *data, size_t length) {
    ESP_LOGI(TAG, "Starting OTA... Size: %d bytes", length);
    uart_flush(UART_PORT_NUM); // Xóa buffer trước khi bắt đầu

    /* --- STEP 1: CONNECT (0xAA) --- */
    ESP_LOGI(TAG, "Step 1: Connecting...");
    send_byte(CMD_CONNECT);
    if (wait_for_ack(2000) != ESP_OK) { // 2s timeout
        ESP_LOGE(TAG, "Connect Failed! No ACK.");
        return ESP_FAIL;
    }

    /* --- STEP 2: SEND INFO (0xBB) + SIZE --- */
    ESP_LOGI(TAG, "Step 2: Sending Info...");
    send_byte(CMD_INFO);
    
    uint8_t size_buf[4];
    size_buf[0] = (length >> 24) & 0xFF;
    size_buf[1] = (length >> 16) & 0xFF;
    size_buf[2] = (length >> 8) & 0xFF;
    size_buf[3] = (length) & 0xFF;
    uart_write_bytes(UART_PORT_NUM, (const char*)size_buf, 4);

    // Chờ 5s cho việc xóa Flash
    if (wait_for_ack(5000) != ESP_OK) {
        ESP_LOGE(TAG, "Erase Failed! No ACK.");
        return ESP_FAIL;
    }

    /* --- STEP 3: SEND DATA PACKETS (0xCC) --- */
    ESP_LOGI(TAG, "Step 3: Sending Data...");
    size_t offset = 0;
    uint8_t global_xor = 0;

    // Tính trước Global XOR
    for(size_t i=0; i<length; i++) {
        global_xor ^= data[i];
    }

    while (offset < length) {
        size_t chunk_len = (length - offset > PACKET_SIZE) ? PACKET_SIZE : (length - offset);
        uint8_t local_xor = 0;

        // Header: CMD_DATA + Length
        uint8_t header[2] = { CMD_DATA, (uint8_t)chunk_len };
        uart_write_bytes(UART_PORT_NUM, (const char*)header, 2);

        // Data & Calculate Local XOR
        for (size_t i = 0; i < chunk_len; i++) {
            local_xor ^= data[offset + i];
        }
        uart_write_bytes(UART_PORT_NUM, (const char*)&data[offset], chunk_len);

        // Checksum
        send_byte(local_xor);

        // Wait ACK (500ms)
        if (wait_for_ack(500) != ESP_OK) {
            ESP_LOGE(TAG, "Packet NACK at offset %d", offset);
            return ESP_FAIL; // Đơn giản hóa: Fail thì dừng luôn
        }

        offset += chunk_len;
        if (offset % 1024 == 0) ESP_LOGI(TAG, "Progress: %d / %d", offset, length);
    }

    /* --- STEP 4: SEND END (0xDD) --- */
    ESP_LOGI(TAG, "Step 4: Finishing...");
    send_byte(CMD_END);
    send_byte(global_xor); // Gửi Global Checksum

    if (wait_for_ack(2000) != ESP_OK) {
        ESP_LOGE(TAG, "Final Checksum/Jump Failed!");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "OTA Update SUCCESS! STM32 is rebooting...");
    return ESP_OK;
}