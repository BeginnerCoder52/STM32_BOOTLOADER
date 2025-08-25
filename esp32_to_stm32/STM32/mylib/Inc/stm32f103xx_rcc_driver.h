#ifndef _STM32F1XX_RCC_H_
#define _STM32F1XX_RCC_H_
#include "stm32f103xx.h"

/* ------------------------ TYPE DEFINITIONS ------------------------ */
// peripheral register definition structure for RCC
typedef struct{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBSTR;
	__vo uint32_t CFGR2;
} RCC_TypeDef_t;

typedef enum {
    RCC_OSCILLATORTYPE_NONE = 0x01,
    RCC_OSCILLATORTYPE_HSE  = 0x02,
    RCC_OSCILLATORTYPE_HSI  = 0x04,
	RCC_OSCILLATORTYPE_PLL	= 0x08,
    RCC_OSCILLATORTYPE_LSE  = 0x10,
    RCC_OSCILLATORTYPE_LSI  = 0x20
} RCC_OscillatorType;

typedef enum {
    RCC_HSE_OFF = 0,
    RCC_HSE_ON,
    RCC_HSE_BYPASS
} RCC_HSEState;

typedef enum {
    RCC_HSI_OFF = 0,
    RCC_HSI_ON
} RCC_HSIState;

typedef enum {
    RCC_LSE_OFF = 0,
    RCC_LSE_ON,
    RCC_LSE_BYPASS
} RCC_LSEState;

typedef enum {
    RCC_LSI_OFF = 0,
    RCC_LSI_ON
} RCC_LSIState;

typedef enum {
    RCC_PLL_NONE = 0,
    RCC_PLL_OFF,
    RCC_PLL_ON
} RCC_PLLState;

typedef enum {
    RCC_PLLSOURCE_HSI_DIV2_0 = 0,
    RCC_PLLSOURCE_HSE_DIV1,
	RCC_PLLSOURCE_HSI_DIV2_1,
    RCC_PLLSOURCE_HSE_DIV2
} RCC_PLLSource;

typedef enum {
	RCC_PLL_MUL2 = 0,
	RCC_PLL_MUL3,
    RCC_PLL_MUL4,
    RCC_PLL_MUL5,
    RCC_PLL_MUL6,
    RCC_PLL_MUL7,
    RCC_PLL_MUL8,
    RCC_PLL_MUL9,
	RCC_PLL_MUL10,
	RCC_PLL_MUL11,
	RCC_PLL_MUL12,
	RCC_PLL_MUL13,
	RCC_PLL_MUL14,
	RCC_PLL_MUL15,
	RCC_PLL_MUL16_1,
	RCC_PLL_MUL16_2,
} RCC_PLLMul;

typedef enum {
    RCC_SYSCLKSOURCE_HSI = 0,
    RCC_SYSCLKSOURCE_HSE,
    RCC_SYSCLKSOURCE_PLLCLK
} RCC_SysClkSource;

typedef enum {
    RCC_SYSCLK_DIV1 = 0,
    RCC_SYSCLK_DIV2	= 8,
    RCC_SYSCLK_DIV4,
    RCC_SYSCLK_DIV8,
    RCC_SYSCLK_DIV16,
    RCC_SYSCLK_DIV64,
    RCC_SYSCLK_DIV128,
    RCC_SYSCLK_DIV256,
    RCC_SYSCLK_DIV512
} RCC_AHBClk_Divider;

typedef enum {
    RCC_HCLK_DIV1 = 0,
    RCC_HCLK_DIV2 = 4,
    RCC_HCLK_DIV4,
    RCC_HCLK_DIV8,
    RCC_HCLK_DIV16
} RCC_APBClk_Divider;

typedef enum {
    RCC_MCO_NOCLOCK = 0,    // No clock output
    RCC_MCO_SYSCLK  = 4,    // System clock
    RCC_MCO_HSI     = 5,    // HSI oscillator clock
    RCC_MCO_HSE     = 6,    // HSE oscillator clock
    RCC_MCO_PLLCLK_DIV2 = 7 // PLL clock divided by 2
} RCC_MCO_Src_TypeDef_t;

typedef enum {
	RCC_ADC_DIV2	= 0,
	RCC_ADC_DIV4	= 1,
	RCC_ADC_DIV6	= 2,
	RCC_ADC_DIV8	= 3
} RCC_ADC_Divider_TypeDef_t;

typedef struct {
    RCC_OscillatorType OscillatorType;
    RCC_HSEState       HSEState;
    RCC_HSIState       HSIState;
    RCC_LSEState       LSEState;
    RCC_LSIState       LSIState;
    RCC_PLLState       PLL_State;
    RCC_PLLSource      PLL_Source;
    RCC_PLLMul         PLL_Mul;
} RCC_OscInitTypeDef;

typedef struct {
    RCC_SysClkSource    SysClkSource;
    RCC_AHBClk_Divider   AHBCLKDivider;
    RCC_APBClk_Divider   APB1CLKDivider;
    RCC_APBClk_Divider   APB2CLKDivider;
} RCC_ClkInitTypeDef;

#define RCC			((RCC_TypeDef_t*)(RCC_BASEADDR))

/*
 * Bit position definitions for RCC_CR register
 */
#define RCC_CR_HSION		0
#define RCC_CR_HSIRDY		1
#define RCC_CR_HSITRIM		3
#define RCC_CR_HSICAL		8
#define RCC_CR_HSEON		16
#define RCC_CR_HSERDY		17
#define RCC_CR_HSEBYP		18
#define RCC_CR_CSSON		19
#define RCC_CR_PLLON		24
#define RCC_CR_PLLRDY		25

/*
 * Bit position definitions for RCC_CFGR register
 */
#define RCC_CFGR_SW			0
#define RCC_CFGR_SWS		2
#define RCC_CFGR_HPRE		4
#define RCC_CFGR_PPRE1		8
#define RCC_CFGR_PPRE2		11
#define RCC_CFGR_ADCPRE		14
#define RCC_CFGR_PLLSRC		16
#define RCC_CFGR_PLLXTPRE	17
#define RCC_CFGR_PLLMUL		18
#define RCC_CFGR_USBPRE		22
#define RCC_CFGR_MCO		24

/*
 * Bit masks for multi-bit fields in RCC_CFGR
 */
#define RCC_CFGR_SW_MASK	(0x3 << RCC_CFGR_SW)
#define RCC_CFGR_SWS_MASK	(0x3 << RCC_CFGR_SWS)
#define RCC_CFGR_HPRE_MASK	(0xF << RCC_CFGR_HPRE)
#define RCC_CFGR_PPRE1_MASK	(0x7 << RCC_CFGR_PPRE1)
#define RCC_CFGR_PPRE2_MASK	(0x7 << RCC_CFGR_PPRE2)
#define RCC_CFGR_ADCPRE_MASK (0x3 << RCC_CFGR_ADCPRE)
#define RCC_CFGR_PLLMUL_MASK (0xF << RCC_CFGR_PLLMUL)
#define RCC_CFGR_MCO_MASK	(0x7 << RCC_CFGR_MCO)

/*
 * Clock source selection values
 */
#define RCC_CFGR_SW_HSI		0x0
#define RCC_CFGR_SW_HSE		0x1
#define RCC_CFGR_SW_PLL		0x2

#define RCC_CFGR_SWS_HSI	RCC_CFGR_SW_HSI
#define RCC_CFGR_SWS_HSE	RCC_CFGR_SW_HSE
#define RCC_CFGR_SWS_PLL	RCC_CFGR_SW_PLL
/*
 * Bit position definitions for RCC_CIR register
 */
#define RCC_CIR_LSIRDYF		0
#define RCC_CIR_LSERDYF		1
#define RCC_CIR_HSIRDYF		2
#define RCC_CIR_HSERDYF		3
#define RCC_CIR_PLLRDYF		4
#define RCC_CIR_CSSF		7
#define RCC_CIR_LSIRDYIE	8
#define RCC_CIR_LSERDYIE	9
#define RCC_CIR_HSIRDYIE	10
#define RCC_CIR_HSERDYIE	11
#define RCC_CIR_PLLRDYIE	12
#define RCC_CIR_LSIRDYC		16
#define RCC_CIR_LSERDYC		17
#define RCC_CIR_HSIRDYC		18
#define RCC_CIR_HSERDYC		19
#define RCC_CIR_PLLRDYC		20
#define RCC_CIR_CSSC		23

/*
 * Bit position definitions for RCC_APB2RSTR register
 */
#define RCC_APB2RSTR_AFIORST	0
#define RCC_APB2RSTR_IOPARST	2
#define RCC_APB2RSTR_IOPBRST	3
#define RCC_APB2RSTR_IOPCRST	4
#define RCC_APB2RSTR_IOPDRST	5
#define RCC_APB2RSTR_IOPERST	6
#define RCC_APB2RSTR_IOPFRST	7
#define RCC_APB2RSTR_IOPGRST	8
#define RCC_APB2RSTR_ADC1RST	9
#define RCC_APB2RSTR_ADC2RST	10
#define RCC_APB2RSTR_TIM1RST	11
#define RCC_APB2RSTR_SPI1RST	12
#define RCC_APB2RSTR_TIM8RST	13
#define RCC_APB2RSTR_USART1RST	14
#define RCC_APB2RSTR_ADC3RST	15
#define RCC_APB2RSTR_TIM9RST	19
#define RCC_APB2RSTR_TIM10RST	20
#define RCC_APB2RSTR_TIM11RST	21

/*
 * Bit position definitions for RCC_APB1RSTR register
 */
#define RCC_APB1RSTR_TIM2RST	0
#define RCC_APB1RSTR_TIM3RST	1
#define RCC_APB1RSTR_TIM4RST	2
#define RCC_APB1RSTR_TIM5RST	3
#define RCC_APB1RSTR_TIM6RST	4
#define RCC_APB1RSTR_TIM7RST	5
#define RCC_APB1RSTR_TIM12RST	6
#define RCC_APB1RSTR_TIM13RST	7
#define RCC_APB1RSTR_TIM14RST	8
#define RCC_APB1RSTR_WWDGRST	11
#define RCC_APB1RSTR_SPI2RST	14
#define RCC_APB1RSTR_SPI3RST	15
#define RCC_APB1RSTR_USART2RST	17
#define RCC_APB1RSTR_USART3RST	18
#define RCC_APB1RSTR_UART4RST	19
#define RCC_APB1RSTR_UART5RST	20
#define RCC_APB1RSTR_I2C1RST	21
#define RCC_APB1RSTR_I2C2RST	22
#define RCC_APB1RSTR_USBRST		23
#define RCC_APB1RSTR_CANRST		25
#define RCC_APB1RSTR_BKPRST		27
#define RCC_APB1RSTR_PWRRST		28
#define RCC_APB1RSTR_DACRST		29

/*
 * Bit position definitions for RCC_AHBENR register
 */
#define RCC_AHBENR_DMA1EN	0
#define RCC_AHBENR_DMA2EN	1
#define RCC_AHBENR_SRAMEN	2
#define RCC_AHBENR_FLITFEN	4
#define RCC_AHBENR_CRCEN	6
#define RCC_AHBENR_FSMCEN	8
#define RCC_AHBENR_SDIOEN	10

/*
 * Bit position definitions for RCC_APB2ENR register
 */
#define RCC_APB2ENR_AFIOEN	0
#define RCC_APB2ENR_IOPAEN	2
#define RCC_APB2ENR_IOPBEN	3
#define RCC_APB2ENR_IOPCEN	4
#define RCC_APB2ENR_IOPDEN	5
#define RCC_APB2ENR_IOPEEN	6
#define RCC_APB2ENR_IOPFEN	7
#define RCC_APB2ENR_IOPGEN	8
#define RCC_APB2ENR_ADC1EN	9
#define RCC_APB2ENR_ADC2EN	10
#define RCC_APB2ENR_TIM1EN	11
#define RCC_APB2ENR_SPI1EN	12
#define RCC_APB2ENR_TIM8EN	13
#define RCC_APB2ENR_USART1EN 14
#define RCC_APB2ENR_ADC3EN	15
#define RCC_APB2ENR_TIM9EN	19
#define RCC_APB2ENR_TIM10EN	20
#define RCC_APB2ENR_TIM11EN	21

/*
 * Bit position definitions for RCC_APB1ENR register
 */
#define RCC_APB1ENR_TIM2EN	0
#define RCC_APB1ENR_TIM3EN	1
#define RCC_APB1ENR_TIM4EN	2
#define RCC_APB1ENR_TIM5EN	3
#define RCC_APB1ENR_TIM6EN	4
#define RCC_APB1ENR_TIM7EN	5
#define RCC_APB1ENR_TIM12EN	6
#define RCC_APB1ENR_TIM13EN	7
#define RCC_APB1ENR_TIM14EN	8
#define RCC_APB1ENR_WWDGEN	11
#define RCC_APB1ENR_SPI2EN	14
#define RCC_APB1ENR_SPI3EN	15
#define RCC_APB1ENR_USART2EN 17
#define RCC_APB1ENR_USART3EN 18
#define RCC_APB1ENR_UART4EN	19
#define RCC_APB1ENR_UART5EN	20
#define RCC_APB1ENR_I2C1EN	21
#define RCC_APB1ENR_I2C2EN	22
#define RCC_APB1ENR_USBEN	23
#define RCC_APB1ENR_CANEN	25
#define RCC_APB1ENR_BKPEN	27
#define RCC_APB1ENR_PWREN	28
#define RCC_APB1ENR_DACEN	29

/*
 * Bit position definitions for RCC_BDCR register
 */
#define RCC_BDCR_LSEON		0
#define RCC_BDCR_LSERDY		1
#define RCC_BDCR_LSEBYP		2
#define RCC_BDCR_RTCSEL		8
#define RCC_BDCR_RTCEN		15
#define RCC_BDCR_BDRST		16

/*
 * Bit position definitions for RCC_CSR register
 */
#define RCC_CSR_LSION		0
#define RCC_CSR_LSIRDY		1
#define RCC_CSR_RMVF		24
#define RCC_CSR_PINRSTF		26
#define RCC_CSR_PORRSTF		27
#define RCC_CSR_SFTRSTF		28
#define RCC_CSR_IWDGRSTF	29
#define RCC_CSR_WWDGRSTF	30
#define RCC_CSR_LPWRRSTF	31

#define	RCC_HSI_DISABLE()	do{CLEAR_BIT(RCC->CR, RCC_CR_HSION);} while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == SET);
#define	RCC_HSI_ENABLE()	do{SET_BIT(RCC->CR, RCC_CR_HSION);} while (READ_BIT(RCC->CR, RCC_CR_HSIRDY) == RESET);

#define	RCC_HSE_DISABLE()	do{CLEAR_BIT(RCC->CR, RCC_CR_HSEON);} while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == SET);
#define	RCC_HSE_ENABLE()	do{SET_BIT(RCC->CR, RCC_CR_HSEON);} while (READ_BIT(RCC->CR, RCC_CR_HSERDY) == RESET);

#define	RCC_PLL_DISABLE()	do{CLEAR_BIT(RCC->CR, RCC_CR_PLLON);} while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == SET);
#define	RCC_PLL_ENABLE()	do{SET_BIT(RCC->CR, RCC_CR_PLLON);} while (READ_BIT(RCC->CR, RCC_CR_PLLRDY) == RESET);

#define	RCC_HSI_FREQ		8000000U
#define RCC_HSE_FREQ		8000000U

/*#define RCC_MCO_CLK_ENABLE		GPIOA_PCLK_EN
#define	RCC_MCO_CLK_DISABLE		GPIOA_PCLK_DI
#define RCC_MCO_PORT			GPIOA
#define	RCC_MCO_PIN				GPIO_PIN_NO_8*/
/* ------------------------ INITIALIZATION APIS ------------------------ */

// System initialization
void RCC_DeInit(void);
void RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct);
void RCC_ADCClockConfig(RCC_ADC_Divider_TypeDef_t);
// /* ------------------------ CLOCK FREQUENCY FUNCTIONS ------------------------ */

// Get clock frequencies
uint32_t RCC_GetSysClockFreq(void);
uint32_t RCC_GetHClockFreq(void);
uint32_t RCC_GetPClock1Freq(void);
uint32_t RCC_GetPClock2Freq(void);
uint32_t RCC_GetADCClockFreq(void);

void RCC_EnableCSS(void);
void RCC_DisableCSS(void);

void RCC_MCO_Config(RCC_MCO_Src_TypeDef_t);
void RCC_MCO_DeInit();

#endif
