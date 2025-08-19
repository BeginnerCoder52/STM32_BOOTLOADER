/*
 * uart_link.h
 *
 *  Created on: Aug 19, 2025
 *      Author: Richard Acer
 */

#ifndef MAIN_UART_LINK_H_
#define MAIN_UART_LINK_H_

#pragma once
#include <stdint.h>
void uart_init(int tx_gpio, int rx_gpio, int uart_num, int baud);
void send_BEGIN(uint8_t part, uint32_t size, uint32_t crc32, uint32_t version);
void send_DATA(uint32_t offset, const uint8_t* data, uint16_t len);
void send_END(void);
void send_SET_PENDING(uint8_t part, uint8_t trial);
void send_REBOOT(void);
void wait_response(uint8_t expected, uint32_t timeout_ms);


#endif /* MAIN_UART_LINK_H_ */
