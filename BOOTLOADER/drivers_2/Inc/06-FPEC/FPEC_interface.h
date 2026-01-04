#ifndef FPEC_INTERFACE_H
#define FPEC_INTERFACE_H

/* Hàm khởi tạo (Mở khóa Flash) */
void FPEC_voidInit(void);

/* Hàm xóa 1 trang Flash (1 Page = 1KB = 1024 Bytes) */
/* PageNumber: 0 đến 63 (Với STM32F103C8T6 64KB) */
void FPEC_voidFlashPageErase(u8 Copy_u8PageNumber);

/* Hàm ghi dữ liệu (Chỉ ghi được 2 byte - Half Word một lần) */
void FPEC_voidFlashWrite(u32 Copy_u32Address, u16 Copy_u16Data);

#endif /* FPEC_INTERFACE_H */
