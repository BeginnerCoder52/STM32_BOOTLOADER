/*********************************************************************************/
/* Author    : Islam Abdo                                                        */
/* Version   : V01                                                               */
/* Date      : 11 OCT  2020                                                      */
/*********************************************************************************/
#ifndef UART_INTERFACE_H
#define UART_INTERFACE_H



/***************   Function definition    ******************/

void printMsg_init(void );
void printMsg(uint8 *msg);

/* Include STD_TYPES để trình biên dịch hiểu u8 là gì */
#include "../01-LIB/STD_TYPES.h"

/* Khai báo nguyên mẫu hàm (Function Prototypes) */
void UART_voidInit(void);
void UART_voidTransmit(u8 Copy_u8Data);
u8   UART_u8Receive(void);

#endif // UART_INTERFACE_H
