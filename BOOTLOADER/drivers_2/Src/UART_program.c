/*********************************************************************************/
/* Author    : Islam Abdo                                                        */
/* Version   : V01                                                               */
/* Date      : 17 Jan  2021                                                      */
/*********************************************************************************/
#include "../Inc/01-LIB/STD_TYPES.h"
#include "../Inc/01-LIB/BIT_MATH.h"

/* Include theo đúng thứ tự để nhận diện macro */
#include "../Inc/05-UART/UART_private.h"
#include "../Inc/05-UART/UART_config.h"
#include "../Inc/05-UART/UART_interface.h"

/* FIX LỖI QUAN TRỌNG:
   Trong file UART_private.h cũ, địa chỉ UART đang là 0x40010000 (AFIO).
   Địa chỉ đúng của USART1 là 0x40013800.
   Ta dùng #undef để định nghĩa lại địa chỉ đúng mà không cần sửa file .h
*/
#ifdef UART
#undef UART
#endif

#define UART ((volatile UART_t *) 0x40013800)


/* --- Hàm Khởi Tạo --- */
void UART_voidInit(void)
{
    /* 1. Cấu hình Baudrate
       Lấy giá trị BAUDRATE từ UART_config.h (đã map sang hex 0x271 trong private.h)
    */
    UART->BRR = BAUDRATE;

    /* 2. Cấu hình Control Register 1 (CR1) */
    /* Reset CR1 về 0 trước để sạch sẽ */
    UART->CR1 = 0;

    /* Bật UART (UE - Bit 13) */
    SET_BIT(UART->CR1, 13);

    /* Bật Bộ Phát (TE - Bit 3) */
    SET_BIT(UART->CR1, 3);

    /* Bật Bộ Thu (RE - Bit 2) */
    SET_BIT(UART->CR1, 2);
}

/* --- Hàm Gửi Dữ Liệu --- */
void UART_voidTransmit(u8 Copy_u8Data)
{
    /* Chờ cờ TXE (Transmit Data Register Empty - Bit 7) trong thanh ghi SR */
    /* Dùng cấu trúc UART_t thay vì USART_t */
    while (GET_BIT(UART->SR, 7) == 0);

    /* Ghi dữ liệu vào thanh ghi DR */
    UART->DR = Copy_u8Data;
}

/* --- Hàm Nhận Dữ Liệu --- */
u8 UART_u8Receive(void)
{
    /* Chờ cờ RXNE (Read Data Register Not Empty - Bit 5) trong thanh ghi SR */
    while (GET_BIT(UART->SR, 5) == 0);

    /* Đọc dữ liệu từ DR và trả về */
    return (u8)(UART->DR & 0xFF);
}

