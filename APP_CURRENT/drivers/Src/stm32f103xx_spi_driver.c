/*
 * stm32f103xx_spi_driver.c
 *
 *  Created on: Jul 23, 2025
 *      Author: nphuc
 */
#include "stm32f103xx_spi_driver.h"

// define private function
static void SPI_TXE_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (SPI_CR1_DFF))) {
		// 16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength-= 2;
		(uint16_t*)(pSPIHandle->pTxBuffer)++;
	} else {
		// 8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
		pSPIHandle->TxLength--;
		pSPIHandle->pTxBuffer++;
	}

	if(!pSPIHandle->TxLength){
		// close the transmission
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

		pSPIHandle->pTxBuffer = NULL;
		pSPIHandle->TxLength = 0;
		pSPIHandle->TxState = SPI_READY;
		SPI_TransmissionEventsCallback(pSPIHandle);
	}
}

static void SPI_RXNE_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// 2. check the DFF bit in CR1
	if ((pSPIHandle->pSPIx->CR1 & (SPI_CR1_DFF))) {
		// 16 bit
		// 1. load the data from DR to RxBuffer
		*((uint16_t*) pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength-= 2;
		(uint16_t*)(pSPIHandle->pRxBuffer)++;
	} else {
		// 8 bit
		// 1. load the data in to the DR
		(*pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLength--;
		(pSPIHandle->pRxBuffer)++;
	}

	if(!pSPIHandle->RxLength){
		// close the spi transmission
		pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLength = 0;
		pSPIHandle->RxState = SPI_READY;
		SPI_ReceptionEventsCallback(pSPIHandle);
	}
}

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_TypeDef_t *pSPIx, uint8_t EnorDi){
	if(!EnorDi) return;
	if(pSPIx == SPI1)		SPI1_PCLK_EN();
	else if(pSPIx == SPI2)	SPI2_PCLK_EN();
	else if(pSPIx == SPI3)	SPI3_PCLK_EN();
}

/*
 * Start and Stop peripheral
 */

void SPI_Start(SPI_TypeDef_t *pSPIx){
	pSPIx->CR1 |= (1 << SPI_CR1_SPE);
}

void SPI_Stop(SPI_TypeDef_t *pSPIx){
	pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
}

/*
 * Init and De-Init
 */

void SPI_Init(SPI_Handle_t *pSPIHandle){
	// peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	// configure the SPI_CR1 register
	uint16_t temp = 0;

	// 1. configure the device mode
	temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;
	// 2. configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// bidi mode should be cleared
		temp &= ~(1 << 15);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// bidi mode should be set
		temp |= 1 << 15;
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// bidi mode should be cleared
		temp &= ~(1 << 15);
		// RX only mode should be set
		temp |= (1 << 10);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_TXONLY){
		// bidi mode should be set
		temp |= 1 << 15;
		temp |= 1 << 14;
	}
	// 3. configure the spi serial clock speed
	temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;
	// 4. configure the data format
	temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;
	// 5. configure the CPOL
	temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;
	// 6. configure the CPHA
	temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;
	// 7. configure th SSM
	temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp;
}

/*
 * SSOE and SSI
 */

void SPI_DeInit(SPI_TypeDef_t *pSPIx){
	if (pSPIx == SPI1) SPI1_REG_RESET();
	else if (pSPIx == SPI2) SPI1_REG_RESET();
	else if (pSPIx == SPI3) SPI1_REG_RESET();
}

void SPI_SSIConfig(SPI_TypeDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi) pSPIx->CR1 |=  (1 << SPI_CR1_SSI);
	else pSPIx->CR1 &=  ~(1 << SPI_CR1_SSI);
}

void SPI_SSOEConfig(SPI_TypeDef_t *pSPIx, uint8_t EnOrDi){
	if(EnOrDi) SPI1->CR2 |=  (1 << SPI_CR2_SSOE);
	else SPI1->CR2 &=  ~(1 << SPI_CR2_SSOE);
}


/*
 * Data send and Receive
 */

void SPI_SendData(SPI_TypeDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t length){
	while(length > 0){
		// 1. wait until TXE is set
		while(((pSPIx->SR >> SPI_SR_TXE) & 1) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if((pSPIx->CR1 & (SPI_CR1_DFF))){
			// 16 bit
			// 1. load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length-= 2;
			(uint16_t*)pTxBuffer++;
		}else{
			// 1. load the data in to the DR
			pSPIx->DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}
	}
}

void SPI_ReceiveData(SPI_TypeDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t length){
	while(length > 0){
		// 1. wait until RXNE is set
		while(((pSPIx->SR >> SPI_SR_RXNE) & 1) == FLAG_RESET);

		// 2. check the DFF bit in CR1
		if((pSPIx->CR1 & (SPI_CR1_DFF))){
			// 16 bit
			// 1. load the data from DR to RxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length-= 2;
			(uint16_t*)pRxBuffer++;
		}else{
			// 8 bit
			// 1. load the data in to the DR
			(*pRxBuffer) = pSPIx->DR;
			length--;
			pRxBuffer++;
		}
	}
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t length){
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_TX){
		// 1. Save the Tx buffer address and length information in global var
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLength = length;

		// 2. Mark the SPI state as busy in tranmisstion
		pSPIHandle->TxState = SPI_BUSY_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t length){
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_RX){
		// 1. Save the Rx buffer address and length information in global var
		pSPIHandle->pTxBuffer = pRxBuffer;
		pSPIHandle->RxLength = length;
		// 2. Mark the SPI state as busy in tranmisstion
		pSPIHandle->RxState = SPI_BUSY_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

void SPI_IRQHandling(SPI_Handle_t *pSPIHandle){
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = (pSPIHandle->pSPIx->SR >> SPI_SR_TXE) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> SPI_CR2_TXEIE) & 1;

	if(temp1 && temp2) SPI_TXE_Interrupt_Handle(pSPIHandle);

	// check for RXNE
	temp1 = (pSPIHandle->pSPIx->SR >> SPI_SR_RXNE) & 1;
	temp2 = (pSPIHandle->pSPIx->CR2 >> SPI_CR2_RXNEIE) & 1;

	if(temp1 && temp2) SPI_RXNE_Interrupt_Handle(pSPIHandle);
}

#ifdef SPI1_Handler
void SPI1_IRQHandler(){
	SPI_IRQHandling(&hspi1);
}
#endif

#ifdef SPI2_Handler
void SPI2_IRQHandler(){
	SPI_IRQHandling(&hspi2);
}
#endif

__weak void SPI_TransmissionEventsCallback(SPI_Handle_t *pHandle){}

__weak void SPI_ReceptionEventsCallback(SPI_Handle_t *pHandle){}