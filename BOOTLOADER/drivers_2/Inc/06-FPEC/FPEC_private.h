#ifndef FPEC_PRIVATE_H
#define FPEC_PRIVATE_H

/* Định nghĩa cấu trúc thanh ghi Flash */
typedef struct
{
	volatile u32 ACR;      /* Flash access control register */
	volatile u32 KEYR;     /* Flash key register */
	volatile u32 OPTKEYR;  /* Flash option key register */
	volatile u32 SR;       /* Flash status register */
	volatile u32 CR;       /* Flash control register */
	volatile u32 AR;       /* Flash address register */
	volatile u32 RESERVED; /* Reserved */
	volatile u32 OBR;      /* Option byte register */
	volatile u32 WRPR;     /* Write protection register */
} FPEC_t;

/* Địa chỉ cơ sở của Flash Interface */
#define FPEC_BASE_ADDRESS   0x40022000
#define FPEC                ((volatile FPEC_t *)FPEC_BASE_ADDRESS)

/* Các Key để mở khóa Flash */
#define FPEC_KEY1           0x45670123
#define FPEC_KEY2           0xCDEF89AB

#endif /* FPEC_PRIVATE_H */
