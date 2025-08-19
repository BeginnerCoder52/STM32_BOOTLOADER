#include "uart_link.h"
#include <esp_timer.h>  // Add at top

void wait_response(uint8_t expected, uint32_t timeout_ms) {
    uint8_t resp;
    uint32_t start = esp_timer_get_time() / 1000;
    while ((esp_timer_get_time() / 1000 - start) < timeout_ms) {
        if (uart_read_bytes(UART_NUM_1, &resp, 1, 10 / portTICK_PERIOD_MS) == 1) {
            if (resp == expected) return;
            else { /* Handle error, e.g., abort */ }
        }
    }
    /* Timeout error handling */
}

void uart_init(int tx, int rx, int uart_num, int baud){
  uart_config_t cfg={ .baud_rate=baud, .data_bits=UART_DATA_8_BITS,
    .parity=UART_PARITY_DISABLE, .stop_bits=UART_STOP_BITS_1, .flow_ctrl=UART_HW_FLOWCTRL_DISABLE };
  uart_param_config(uart_num,&cfg);
  uart_set_pin(uart_num, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, 4096, 4096, 0, NULL, 0);
}

static void write_bytes(const void* p, size_t n){ uart_write_bytes(UART_NUM_1, p, n); }

void send_BEGIN(uint8_t part, uint32_t size, uint32_t crc32, uint32_t version){
  uint8_t c='B'; write_bytes(&c,1);
  write_bytes(&part,1); write_bytes(&size,4); write_bytes(&crc32,4); write_bytes(&version,4);
}
void send_DATA(uint32_t offset, const uint8_t* data, uint16_t len){
  uint8_t c='D'; write_bytes(&c,1);
  write_bytes(&offset,4); write_bytes(&len,2); write_bytes(data,len);
}
void send_END(void){ uint8_t c='E'; write_bytes(&c,1); }
void send_SET_PENDING(uint8_t part, uint8_t trial){ uint8_t c='P'; write_bytes(&c,1); write_bytes(&part,1); write_bytes(&trial,1); }
void send_REBOOT(void){ uint8_t c='R'; write_bytes(&c,1); }
