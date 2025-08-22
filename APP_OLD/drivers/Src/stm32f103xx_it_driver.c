/*
 * stm32f103xx_interrupt_driver.c
 *
 *  Created on: Aug 5, 2025
 *      Author: nphuc
 */


#include <stm32f103xx_it_driver.h>

/*
 * IQR configuring and handling
 */

// refered from cortex M3 devices generic user guide
void IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){ // config IRQ number
	if(EnorDi){
		NVIC->ISER[IRQNumber/32] |= (1 << (IRQNumber % 32));
	}else{
		NVIC->ICER[IRQNumber/32] |= (1 << (IRQNumber % 32));
	}
}

void IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority){
	uint8_t iprx				= IRQNumber / 4;
	uint8_t iprx_section		= IRQNumber % 4;
	uint8_t iprx_section_pos	= iprx_section * 8;
	uint8_t upper_order			= IRQPriority << 4;

	NVIC->IPR[iprx] = upper_order << iprx_section_pos;
}
