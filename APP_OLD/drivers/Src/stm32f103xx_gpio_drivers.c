/*
 * stm32f103xx_gpio_drivers.c
 *
 *  Created on: Jun 11, 2025
 *      Author: nphuc
 */

#include <stm32f103xx_gpio_driver.h>
/*
 * Peripheral clock setup
 */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void GPIO_PeriClockControl(GPIO_TypeDef_t *pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
	}
}

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function Initialize GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none

 */

void AlternativeMode_Init(GPIO_Handle_t *pGPIOHandle, uint8_t posPinNumber, uint32_t *reg)
{
	// configure the alt functionality
	switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode)
	{
	case GPIO_ALT_MODE_OUT_PP:
		// configure the pin as a output
		uint8_t speed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed == 0 ? 1 : pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
		(*reg) |= (speed << (4 * posPinNumber));

		// configure alternative push pull
		(*reg) |= (GPIO_CFG_OUT_AL_PP << (4 * posPinNumber + 2));
		break;
	case GPIO_ALT_MODE_OUT_OD:
		// configure the pin as a output
		(*reg) |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * posPinNumber));

		// configure alternative open drain
		*reg |= (GPIO_CFG_OUT_GE_OD << (4 * posPinNumber + 2));
		break;
	case GPIO_ALT_MODE_IN_FLOATING:
		// configure the pin as a input
		(*reg) |= (GPIO_MODE_IN << (4 * posPinNumber));

		// configure input floating
		(*reg) |= (GPIO_CFG_IN_FLOATING << (4 * posPinNumber + 2));
		break;
	case GPIO_ALT_MODE_IN_PUPD:
		break;
	default:
		break;
	}
}

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	uint8_t posPinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	uint8_t posReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;

	// enable the peripheral clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of gpio pin and speed
	switch (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
	{
	case GPIO_MODE_IN ... GPIO_MODE_OUT:
		// non interrupt
		// configure the speed
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (4 * posPinNumber));
		temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinCfgMode << (4 * posPinNumber + 2));
		pGPIOHandle->pGPIOx->CR[posReg] &= ~(3 << (4 * posPinNumber));
		pGPIOHandle->pGPIOx->CR[posReg] &= ~(3 << (4 * posPinNumber + 2));
		// configure control function (output open drain/push pull, input floating/pull up/pull down)
		pGPIOHandle->pGPIOx->CR[posReg] |= temp;
		break;
	case GPIO_MODE_IT_FT ... GPIO_MODE_IT_RFT:
		// interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinCfgMode << (4 * posPinNumber + 2));
			pGPIOHandle->pGPIOx->CR[posReg] &= ~(3 << (4 * posPinNumber + 2));
			pGPIOHandle->pGPIOx->CR[posReg] |= temp;
			// 1. configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			// clear the corresponding RTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// clear the corresponding FTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. configure FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in AFIO_EXTTCR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		AFIO_PCLK_EN();
		AFIO->EXTTCR[temp1] = (portcode << (temp2 * 4));

		// 3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		break;
	case GPIO_MODE_ALTFN:
		AlternativeMode_Init(pGPIOHandle, posPinNumber, &temp);

		pGPIOHandle->pGPIOx->CR[posReg] &= ~(3 << (4 * posPinNumber));
		pGPIOHandle->pGPIOx->CR[posReg] &= ~(3 << (4 * posPinNumber + 2));

		pGPIOHandle->pGPIOx->CR[posReg] |= temp;
		break;
	}

	temp = 0;

	// configure pull up/pull down
	temp |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle->pGPIOx->ODR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->ODR |= temp;
}

/*********************************************************************
 * @fn      		  - GPIO_De-Init
 *
 * @brief             - This function De-initialize GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none

 */

void GPIO_DeInit(GPIO_TypeDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadPin(GPIO_TypeDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
	return value;
}

uint16_t GPIO_ReadPort(GPIO_TypeDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

void GPIO_WritePin(GPIO_TypeDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		// write 1 to the output data register at the bit field corressponding to the pin number
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		// write 0 to the output data register at the bit field corressponding to the pin number
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WritePort(GPIO_TypeDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

void GPIO_Toggle(GPIO_TypeDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

void GPIO_IRQHandling(uint8_t PinNumber)
{ // call when IRQ occur
	// clear
	EXTI->PR |= (1 << PinNumber);
	GPIO_ExternalInterruptEventsCallback(PinNumber);
}

void EXTI0_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
}

void EXTI1_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_1);
}

void EXTI2_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_2);
}

void EXTI3_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_3);
}

void EXTI4_IRQHandler()
{
	GPIO_IRQHandling(GPIO_PIN_NO_4);
}

void EXTI9_5_IRQHandler()
{
	uint8_t pinNumber;
	for (uint8_t i = 5; i < 10; i++)
	{
		if ((EXTI->PR >> i) & 1)
		{
			pinNumber = i;
			break;
		}
	}

	GPIO_IRQHandling(pinNumber);
}

void EXTI15_10_IRQHandler()
{
	uint8_t pinNumber;
	for (uint8_t i = 10; i < 16; i++)
	{
		if ((EXTI->PR >> i) & 1)
		{
			pinNumber = i;
			break;
		}
	}

	GPIO_IRQHandling(pinNumber);
}

__weak void GPIO_ExternalInterruptEventsCallback(uint8_t PinNumber) {}
