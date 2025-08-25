#include "stm32f103xx_rcc_driver.h"

/* ------------------------ PRIVATE ------------------------ */
static void RCC_CIR_Reset(void) {
	CLEAR_REG(RCC->CIR, 8, 0x7F);

	CLEAR_REG(RCC->CIR, 16, 0xFF);

	while(READ_REG(RCC->CIR, 0, 0xFF));
}

static void RCC_Peripheral_Reset(void) {
	/* Reset all peripherals */
	SET_REG(RCC->APB1RSTR, 0, 0xFFFFFFFF);
	SET_REG(RCC->APB2RSTR, 0, 0xFFFFFFFF);

	/* delay */
	for(__vo uint8_t i = 0; i < 100; i++);

	/* Release peripherals */
	CLEAR_REG(RCC->APB1RSTR, 0, 0xFFFFFFFF);
	CLEAR_REG(RCC->APB2RSTR, 0, 0xFFFFFFFF);
}

static void RCC_PeripheralClock_Reset(void) {
	/* Release peripherals */
	CLEAR_REG(RCC->AHBENR, 0, 0xFFFFFFFF);
	CLEAR_REG(RCC->APB1ENR, 0, 0xFFFFFFFF);
	CLEAR_REG(RCC->APB2ENR, 0, 0xFFFFFFFF);

	for(__vo uint8_t i = 0; i < 10; i++);

	SET_BIT(RCC->AHBENR, RCC_AHBENR_SRAMEN);
	SET_BIT(RCC->AHBENR, RCC_AHBENR_FLITFEN);
}

/* --------------------------------------------------------- */

/* ------------------------ PUBLIC ------------------------ */
void RCC_DeInit(void) {
	RCC_Peripheral_Reset();
	/* Set HSION bit */
	RCC_HSI_ENABLE();

	CLEAR_REG(RCC->CFGR, 0, 0xFFFFFFFF);
	while(READ_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_MASK) != RCC_CFGR_SW_HSI);

	/* Disable HSE clock source */
	RCC_HSE_DISABLE();

	/* Disable PLL clock source */
	RCC_PLL_DISABLE();

	CLEAR_BIT(RCC->CR, RCC_CR_CSSON);

	CLEAR_BIT(RCC->CR, RCC_CR_HSEBYP);

	/* Reset HSITRIM to default value */
	SET_REG(RCC->CR, RCC_CR_HSITRIM, 0x10);

	RCC_CIR_Reset();
	RCC_PeripheralClock_Reset();
}

void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct) {
	if(RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_NONE){
		return;
	}

	if(RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_HSI) {
		switch (RCC_OscInitStruct->HSIState)
		{
			case RCC_HSI_OFF:
				RCC_HSI_DISABLE();
				break;

			case RCC_HSI_ON:
				RCC_HSI_ENABLE();
				break;

			default:
				break;
		}
	}

	if(RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_HSE) {
		switch (RCC_OscInitStruct->HSEState)
		{
			case RCC_HSE_OFF:
				RCC_HSE_DISABLE();
				break;

			case RCC_HSE_ON:
				RCC_HSE_ENABLE();
				break;

			default:
				break;
		}
	}

	if(RCC_OscInitStruct->OscillatorType & RCC_OSCILLATORTYPE_PLL) {
		switch (RCC_OscInitStruct->PLL_State)
		{
			case RCC_PLL_OFF:
				RCC_PLL_DISABLE();
				break;

			case RCC_PLL_ON:
				/* Configure PLL source and division */
				CLEAR_REG(RCC->CFGR, RCC_CFGR_PLLSRC, 0x1);
				SET_REG(RCC->CFGR, RCC_CFGR_PLLSRC, RCC_OscInitStruct->PLL_Source);

				switch (RCC_OscInitStruct->PLL_Mul)
				{
					case RCC_PLL_MUL2 ... RCC_PLL_MUL16_2:
						CLEAR_REG(RCC->CFGR, RCC_CFGR_PLLMUL, 0xF);
						SET_REG(RCC->CFGR, RCC_CFGR_PLLMUL, RCC_OscInitStruct->PLL_Mul);
						break;

					default:
						break;
				}
				RCC_PLL_ENABLE();
				break;

			default:
				break;
		}
	}
}

void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct){
	CLEAR_REG(RCC->CFGR, RCC_CFGR_SW, 0x3);
	SET_REG(RCC->CFGR, RCC_CFGR_SW, RCC_ClkInitStruct->SysClkSource);
	while (READ_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_MASK) != RCC_ClkInitStruct->SysClkSource);

	CLEAR_REG(RCC->CFGR, RCC_CFGR_HPRE, 0xF);
	SET_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_ClkInitStruct->AHBCLKDivider);

	CLEAR_REG(RCC->CFGR, RCC_CFGR_PPRE1, 0x7);
	SET_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_ClkInitStruct->APB1CLKDivider);

	CLEAR_REG(RCC->CFGR, RCC_CFGR_PPRE2, 0x7);
	SET_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_ClkInitStruct->APB2CLKDivider);
}

uint32_t RCC_GetSysClockFreq(void) {
	uint32_t sysclk_freq = RCC_HSI_FREQ;

	/* Return 8MHz if sysclk source is HSI or HSE */
	if ((READ_REG(RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_MASK) & 0x2) == 0) {
		return sysclk_freq;
	}

	/* Sysclk is divided by 2 if PLL use HSE as clock source without any division */
	if (READ_REG(RCC->CFGR, RCC_CFGR_PLLSRC, 0x3) != RCC_PLLSOURCE_HSE_DIV1) {
		sysclk_freq /= 2;
	}

	/* Calculate the PLL clock frequency */
	uint8_t mul_factor = READ_REG(RCC->CFGR, RCC_CFGR_PLLMUL, RCC_CFGR_PLLMUL_MASK);

	if(mul_factor == RCC_PLL_MUL16_2){
		return sysclk_freq * 16U;
	}

	return sysclk_freq * (2 + mul_factor);
}

uint32_t RCC_GetHClockFreq(void) {
	uint32_t sys_clk 		= RCC_GetSysClockFreq();
	uint8_t	ahb_div_factor	= READ_REG(RCC->CFGR, RCC_CFGR_HPRE, RCC_CFGR_HPRE_MASK);

	switch (ahb_div_factor)
	{
		case RCC_SYSCLK_DIV2 ... RCC_SYSCLK_DIV16:
			return sys_clk >> (ahb_div_factor - RCC_SYSCLK_DIV2 + 1);

		case RCC_SYSCLK_DIV64 ... RCC_SYSCLK_DIV512:
			return sys_clk >> (ahb_div_factor - RCC_SYSCLK_DIV64 + 6);

		default:
			return sys_clk;
	}
}

uint32_t RCC_GetPClock1Freq(void) {
	uint32_t hclk 			= RCC_GetHClockFreq();
	uint8_t	apb1_div_factor	= READ_REG(RCC->CFGR, RCC_CFGR_PPRE1, RCC_CFGR_PPRE1_MASK);

	switch (apb1_div_factor)
	{
		case RCC_HCLK_DIV2 ... RCC_HCLK_DIV16:
			return hclk >> (apb1_div_factor - RCC_HCLK_DIV2 + 1);

		default:
			return hclk;
	}
}

uint32_t RCC_GetPClock2Freq(void) {
	uint32_t hclk 			= RCC_GetHClockFreq();
	uint8_t	apb2_div_factor	= READ_REG(RCC->CFGR, RCC_CFGR_PPRE2, RCC_CFGR_PPRE2_MASK);

	switch (apb2_div_factor)
	{
		case RCC_HCLK_DIV2 ... RCC_HCLK_DIV16:
			return hclk >> (apb2_div_factor - RCC_HCLK_DIV2 + 1);

		default:
			return hclk;
	}
}

void RCC_ADCClockConfig(RCC_ADC_Divider_TypeDef_t divider) {
	CLEAR_REG(RCC->CFGR, RCC_CFGR_ADCPRE, 0x3);
	SET_REG(RCC->CFGR, RCC_CFGR_ADCPRE, divider);
}

uint32_t RCC_GetADCClockFreq(void) {
	uint32_t apb2_clk = RCC_GetPClock2Freq();
	uint8_t	div_factor = READ_REG(RCC->CFGR, RCC_CFGR_ADCPRE, RCC_CFGR_ADCPRE_MASK);

	return apb2_clk / (2 * (div_factor + 1));
}

void RCC_EnableCSS(void) {
	SET_BIT(RCC->CR, RCC_CR_CSSON);
}

void RCC_DisableCSS(void) {
	CLEAR_BIT(RCC->CR, RCC_CR_CSSON);
}

/*void RCC_MCO_Config(RCC_MCO_Src_TypeDef_t source) {
	// Step 1: Enable GPIOA clock
	RCC_MCO_CLK_ENABLE();

    // Step 2: Configure PA8 as alternate function
    // PA8 is in CRH register, bits 0-3
	CLEAR_REG(RCC_MCO_PORT->CR[1], RCC_MCO_PIN % 8, 0xF);
	SET_REG(RCC_MCO_PORT->CR[1], RCC_MCO_PIN % 8, (0x3 | (0x2 << 2)));

    // Step 3: Set MCO source
    CLEAR_REG(RCC->CFGR, RCC_CFGR_MCO, 0x7);
    SET_REG(RCC->CFGR, RCC_CFGR_MCO, source);
}

void RCC_MCO_DeInit(void) {
	RCC_MCO_CLK_DISABLE();

    // Disable MCO output
    CLEAR_REG(RCC->CFGR, RCC_CFGR_MCO, 0x7);

    // Configure PA8 as input (high impedance)
	CLEAR_REG(RCC_MCO_PORT->CR[1], RCC_MCO_PIN % 8, 0xF);
	SET_REG(RCC_MCO_PORT->CR[1], RCC_MCO_PIN % 8, 1 << 2);
}*/
/* -------------------------------------------------------- */
