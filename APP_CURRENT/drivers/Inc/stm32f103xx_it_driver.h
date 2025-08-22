/*
 * stm32f103xx_interrupt_driver.h
 *
 *  Created on: Aug 5, 2025
 *      Author: nphuc
 */

#ifndef INC_STM32F103XX_IT_DRIVER_H_
#define INC_STM32F103XX_IT_DRIVER_H_

#include "stm32f103xx.h"

#define EXTI				((EXTI_TypeDef_t*)EXTI_BASEADDR)
#define AFIO				((AFIO_TypeDef_t*)AFIO_BASEADDR)
#define NVIC				((NVIC_TypeDef_t*)NVIC_BASE_ADDR)

/*
 * Clock enable macros for AFIO peripheral
 */
#define AFIO_PCLK_EN()		(RCC->APB2ENR |= 1 << 0)

/*
 * Clock disable macros for AFIO peripheral
 */

#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51

#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53

typedef struct{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
} EXTI_TypeDef_t;

// peripheral register definition structure for AFIO
typedef struct{
	__vo uint32_t EVCR;
	__vo uint32_t MAPR;
	__vo uint32_t EXTTCR[4];
	__vo uint32_t MAPR2[2];
} AFIO_TypeDef_t;

// peripheral register definition structure for NVIC
typedef struct{
	__vo uint32_t ISER[8];
	__vo uint32_t RESERVED0[24];
	__vo uint32_t ICER[8];
	__vo uint32_t RESERVED1[24];
	__vo uint32_t ISPR[8];
	__vo uint32_t RESERVED2[24];
	__vo uint32_t ICPR[8];
	__vo uint32_t RESERVED3[24];
	__vo uint32_t IABR[8];
	__vo uint32_t RESERVED4[56];
	__vo uint32_t IPR[60];
	__vo uint32_t RESERVED5[644];
	__vo uint32_t STIR;
} NVIC_TypeDef_t;

/*
 * IQR configuring and handling
 */

void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi); // config IRQ number
void IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

#endif /* INC_STM32F103XX_IT_DRIVER_H_ */
