/*
 * stm32f103xx.h
 *
 *  Created on: Jun 10, 2025
 *      Author: nphuc
 */

#ifndef INC_STM32F103XX_H_
#define INC_STM32F103XX_H_

#define __vo				volatile
#define __weak				__attribute__((weak))

/*
 * base addresses of Flash and SRAM memories
 */
#include <stddef.h>
#include <stdint.h>

#define FLASH_BASEADDR		0x08000000U
#define SRAM_BASEADDR		0x20000000U
#define ROM_BASEADDR		0x1FFFF000U

/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHBPERIPH_BASE		0x40018000U

/*
 * Base addresses of peripherals which are hanging on AHB bus
 */

#define RCC_BASEADDR		(AHBPERIPH_BASE + 0x9000)

// Base Address of peripheral which are hanging on APB1

#define USART2_BASEADDR					(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR					(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR					(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR					(APB1PERIPH_BASE + 0x5000)

#define I2C1_BASEADDR					(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR					(APB1PERIPH_BASE + 0x5800)

#define SPI2_BASEADDR					(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR					(APB1PERIPH_BASE + 0x3C00)

// Base Address of peripheral which are hanging on APB2
#define GPIOA_BASEADDR					(APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASEADDR					(APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASEADDR					(APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASEADDR					(APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASEADDR					(APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASEADDR					(APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASEADDR					(APB2PERIPH_BASE + 0x2000)

#define USART1_BASEADDR					(APB2PERIPH_BASE + 0x3800)

#define SPI1_BASEADDR					(APB2PERIPH_BASE + 0x3000)

#define EXTI_BASEADDR					(APB2PERIPH_BASE + 0x0400)
#define AFIO_BASEADDR					(APB2PERIPH_BASE + 0x0000)

#define NVIC_BASE_ADDR					0xE000E100U

/*
 * Clock enable macros for I2Cx peripheral
 */
#define I2C1_PCLK_EN()					(RCC->APB1ENR |= 1 << 21)
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= 1 << 22)

/*
 * Clock disable macros for I2Cx peripheral
 */
#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))

#define SET_BIT(reg, pos)				(reg |= (1 << pos))
#define CLEAR_BIT(reg, pos)				(reg &= ~(1 << pos))
#define READ_BIT(reg, pos)				((reg >> pos) & 1)

#define SET_REG(reg, pos, bits)			(reg |= (bits << pos))
#define CLEAR_REG(reg, pos, bits)		(reg &= ~(bits << pos))
#define READ_REG(reg, pos, bits)		((reg >> pos) & bits)

#define ENABLE							1
#define DISABLE							0
#define SET 							ENABLE
#define RESET 							DISABLE
#define GPIO_PIN_SET 					SET
#define GPIO_PIN_RESET					RESET
#define FLAG_RESET						RESET
#define FLAG_SET						SET

#include "stm32f103xx_rcc_driver.h"
#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_it_driver.h"
#include "stm32f103xx_spi_driver.h"
#include "stm32f103xx_usart_driver.h"
#include "stm32f103xx_flash_driver.h"

#endif /* INC_STM32F103XX_H_ */
