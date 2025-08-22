/*
 * stm32f103xx_spi_driver.h
 *
 *  Created on: Jul 23, 2025
 *      Author: nphuc
 */

#ifndef INC_STM32F103XX_SPI_DRIVER_H_
#define INC_STM32F103XX_SPI_DRIVER_H_

#include "stm32f103xx.h"

//#define SPI1_Handler
//#define SPI2_Handler

/*
 * Flag to enable SPI peripherals
 */

#define SPI1							((SPI_TypeDef_t*)SPI1_BASEADDR)
#define SPI2							((SPI_TypeDef_t*)SPI2_BASEADDR)
#define SPI3							((SPI_TypeDef_t*)SPI3_BASEADDR)

/*
 * Clock enable macros for SPIx peripheral
 */
#define SPI1_PCLK_EN()					RCC->APB2ENR |= 1 << 12
#define SPI2_PCLK_EN()					RCC->APB1ENR |= 1 << 14
#define SPI3_PCLK_EN()					RCC->APB1ENR |= 1 << 15

/*
 * Clock disable macros for SPIx peripheral
 */
#define SPI1_PCLK_DI()					RCC->APB2ENR &= ~(1 << 12)
#define SPI2_PCLK_DI()					RCC->APB1ENR &= ~(1 << 14)
#define SPI3_PCLK_DI()					RCC->APB1ENR &= ~(1 << 15)

/*
 * Macro to reset SPI peripherals
 */

#define SPI1_REG_RESET() 				do{(RCC->APB2RSTR |= 1 << 12);	(RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET() 				do{(RCC->APB1RSTR |= 1 << 14);	(RCC->APB2RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET() 				do{(RCC->APB1RSTR |= 1 << 15);	(RCC->APB2RSTR &= ~(1 << 15));} while(0)

/*
 * @SPI DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER			1
#define SPI_DEVICE_MODE_SLAVE			0

/*
 * SPI_BusConfig
 */

#define SPI_BUS_CONFIG_FD				1
#define SPI_BUS_CONFIG_HD				2
#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	3
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	4

/*
 * SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BIT					0
#define SPI_DFF_16BIT					1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_HIGH					1
#define SPI_CPOL_LOW					0

/*
 * @SPI_CPHA
 */

#define SPI_CPHA_HIGH					1
#define SPI_CPHA_LOW					0

/*
 * @SPI_SSM
 */

#define SPI_SSM_EN						1
#define SPI_SSM_DI						0


// peripheral register definition structure for SPI
typedef struct{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_TypeDef_t;

typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct
{
	SPI_TypeDef_t *pSPIx;
	SPI_Config_t SPI_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLength;
	uint32_t RxLength;
	uint8_t TxState;
	uint8_t RxState;
} SPI_Handle_t;

#ifdef S1
extern SPI_Handle_t hspi1;
#endif

#ifdef S2
extern SPI_Handle_t hspi2;
#endif

/*
 * SPI related status flag definitions
 */

#define SPI_TXE_FLAG					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)

/*
 * position of SPI_CR1
 */

#define SPI_CR1_CPHA 					0
#define SPI_CR1_CPOL 					1
#define SPI_CR1_MSTR 					2
#define SPI_CR1_BR						3
#define SPI_CR1_SPE						6
#define SPI_CR1_LSBFIRST				7
#define SPI_CR1_SSI 					8
#define SPI_CR1_SSM 					9
#define SPI_CR1_RXONLY	 				10
#define SPI_CR1_DFF 					11
#define SPI_CR1_CRCNEXT 				12
#define SPI_CR1_CRCEN 					13
#define SPI_CR1_BIDIOE 					14
#define SPI_CR1_BIDIMODE 				15

/*
 * position of SPI_CR2
 */

#define SPI_CR2_RXDMAEN 				0
#define SPI_CR2_TXDMAEN 				1
#define SPI_CR2_SSOE					2
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE					6
#define SPI_CR2_TXEIE					7

/*
 * position of SPI_SR
 */

#define SPI_SR_RXNE						0
#define SPI_SR_TXE						1
#define SPI_SR_CHSIDE					2
#define SPI_SR_UDR						3
#define SPI_SR_CRCERR					4
#define SPI_SR_MODF						5
#define SPI_SR_OVR						6
#define SPI_SR_BSY						7

/*
 * SPI Interrupt State
 */

#define SPI_READY						0
#define SPI_BUSY_RX						1
#define SPI_BUSY_TX						2

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_TypeDef_t *pSPIx, uint8_t EnorDi);

/*
 * Start peripheral
 */

void SPI_Start(SPI_TypeDef_t *pSPIx);
void SPI_Stop(SPI_TypeDef_t *pSPIx);

/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_TypeDef_t *pSPIx);

/*
 * SSOE and SSI
 */

void SPI_SSOEConfig(SPI_TypeDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_TypeDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Data send and Receive
 */

void SPI_SendData(SPI_TypeDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_TypeDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length);

/*
 * ISR Handling
 */

void SPI_IRQHandling(SPI_Handle_t *pHandle); // call when IRQ occur

__weak void SPI_TransmissionEventsCallback(SPI_Handle_t *pHandle);

__weak void SPI_ReceptionEventsCallback(SPI_Handle_t *pHandle);

#endif /* INC_STM32F103XX_SPI_DRIVER_H_ */
