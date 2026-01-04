#include "../Inc/01-LIB/STD_TYPES.h"
#include "../Inc/01-LIB/BIT_MATH.h"

#include "../Inc/06-FPEC/FPEC_interface.h"
#include "../Inc/06-FPEC/FPEC_private.h"

void FPEC_voidInit(void)
{
    /* Kiểm tra xem bit LOCK có đang set không */
    if (GET_BIT(FPEC->CR, 7) == 1)
    {
        /* Mở khóa Flash bằng chuỗi Key */
        FPEC->KEYR = FPEC_KEY1;
        FPEC->KEYR = FPEC_KEY2;
    }
}

void FPEC_voidFlashPageErase(u8 Copy_u8PageNumber)
{
    /* Chờ nếu Flash đang bận (Bit BSY) */
    while (GET_BIT(FPEC->SR, 0) == 1);

    /* Bật chế độ xóa trang (Bit PER) */
    SET_BIT(FPEC->CR, 1);

    /* Tính toán địa chỉ trang cần xóa */
    /* Địa chỉ Flash bắt đầu từ 0x08000000 */
    FPEC->AR = (u32)(Copy_u8PageNumber * 1024) + 0x08000000;

    /* Bắt đầu xóa (Bit STRT) */
    SET_BIT(FPEC->CR, 6);

    /* Chờ xóa xong */
    while (GET_BIT(FPEC->SR, 0) == 1);

    /* Xóa cờ EOP (End of Operation) bằng cách ghi 1 vào nó */
    SET_BIT(FPEC->SR, 5);

    /* Tắt chế độ xóa trang */
    CLR_BIT(FPEC->CR, 1);
}

void FPEC_voidFlashWrite(u32 Copy_u32Address, u16 Copy_u16Data)
{
    /* Chờ nếu Flash đang bận */
    while (GET_BIT(FPEC->SR, 0) == 1);

    /* Bật chế độ lập trình (Bit PG - Programming) */
    SET_BIT(FPEC->CR, 0);

    /* Thực hiện ghi dữ liệu (Bắt buộc ép kiểu về u16*) */
    *((volatile u16*)Copy_u32Address) = Copy_u16Data;

    /* Chờ ghi xong */
    while (GET_BIT(FPEC->SR, 0) == 1);

    /* Xóa cờ EOP */
    SET_BIT(FPEC->SR, 5);

    /* Tắt chế độ lập trình */
    CLR_BIT(FPEC->CR, 0);
}
