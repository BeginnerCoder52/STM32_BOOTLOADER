///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */
///* Includes ------------------------------------------------------------------*/
//#include "main.h"

///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "bootloader.h"
//#include "bl_meta.h"
//#include "flash_if.h"
///* USER CODE END Includes */

///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */

///* USER CODE END PTD */

///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */

///* USER CODE END PD */

///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */

///* USER CODE END PM */

///* Private variables ---------------------------------------------------------*/
//IWDG_HandleTypeDef hiwdg;

//UART_HandleTypeDef huart1;

///* USER CODE BEGIN PV */

///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
//static void MX_USART1_UART_Init(void);
//static void MX_IWDG_Init(void);
///* USER CODE BEGIN PFP */

///* USER CODE END PFP */

///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */

///* USER CODE END 0 */

///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{

//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */

//  /* MCU Configuration--------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  MX_IWDG_Init();
//  /* USER CODE BEGIN 2 */
//	bootloader_main();    // G?i v�o bootloader
//  /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  while (1)
//  {
//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
//  }
//  /* USER CODE END 3 */
//}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /** Initializes the RCC Oscillators according to the specified parameters
//  * in the RCC_OscInitTypeDef structure.
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }

//  /** Initializes the CPU, AHB and APB buses clocks
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

///**
//  * @brief IWDG Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_IWDG_Init(void)
//{

//  /* USER CODE BEGIN IWDG_Init 0 */

//  /* USER CODE END IWDG_Init 0 */

//  /* USER CODE BEGIN IWDG_Init 1 */

//  /* USER CODE END IWDG_Init 1 */
//  hiwdg.Instance = IWDG;
//  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
//  hiwdg.Init.Reload = 4095;
//  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN IWDG_Init 2 */

//  /* USER CODE END IWDG_Init 2 */

//}

///**
//  * @brief USART1 Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_USART1_UART_Init(void)
//{

//  /* USER CODE BEGIN USART1_Init 0 */

//  /* USER CODE END USART1_Init 0 */

//  /* USER CODE BEGIN USART1_Init 1 */

//  /* USER CODE END USART1_Init 1 */
//  huart1.Instance = USART1;
//  huart1.Init.BaudRate = 115200;
//  huart1.Init.WordLength = UART_WORDLENGTH_8B;
//  huart1.Init.StopBits = UART_STOPBITS_1;
//  huart1.Init.Parity = UART_PARITY_NONE;
//  huart1.Init.Mode = UART_MODE_TX_RX;
//  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
//  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
//  if (HAL_UART_Init(&huart1) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN USART1_Init 2 */

//  /* USER CODE END USART1_Init 2 */

//}

///**
//  * @brief GPIO Initialization Function
//  * @param None
//  * @retval None
//  */
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//  /* USER CODE BEGIN MX_GPIO_Init_1 */

//  /* USER CODE END MX_GPIO_Init_1 */

//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOD_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();

//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

//  /*Configure GPIO pin : LED_Pin */
//  GPIO_InitStruct.Pin = LED_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

//  /* USER CODE BEGIN MX_GPIO_Init_2 */

//  /* USER CODE END MX_GPIO_Init_2 */
//}

///* USER CODE BEGIN 4 */

///* USER CODE END 4 */

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */
//  __disable_irq();
//  while (1)
//  {
//  }
//  /* USER CODE END Error_Handler_Debug */
//}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */

#include "main.h"
#include "stm32f103xx.h"
#include "stm32f103xx_gpio_driver.h"
#include "stm32f103xx_usart_driver.h"
#include "stm32f103xx_flash_driver.h"
#include "stm32f103xx_rcc_driver.h"
#include "bootloader.h"
#include "core_cm3.h" // Added for SCB and related macros

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL_State = RCC_PLL_ON;
    RCC_OscInitStruct.PLL_Source = RCC_PLLSOURCE_HSE_DIV1;
    RCC_OscInitStruct.PLL_Mul = RCC_PLL_MUL9;
    RCC_OscConfig(&RCC_OscInitStruct); // Use OscConfig for oscillator setup

    RCC_ClkInitStruct.SysClkSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    RCC_ClockConfig(&RCC_ClkInitStruct); // Use ClockConfig for clock setup
}

void MX_GPIO_Init(void) {
    GPIO_Handle_t gpioLed;
    gpioLed.pGPIOx = LED_GPIO_Port;
    gpioLed.GPIO_PinConfig.GPIO_PinNumber = LED_Pin;
    gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    gpioLed.GPIO_PinConfig.GPIO_PinCfgMode = GPIO_CNF_OUT_GE_PP;
    GPIO_PeriClockControl(LED_GPIO_Port, ENABLE);
    GPIO_Init(&gpioLed);
}

void MX_USART1_UART_Init(void) {
    USART_Handle_t usart1;
    usart1.pUSARTx = USART1;
    usart1.USART_Config.USART_Mode = USART_MODE_TXRX;
    usart1.USART_Config.USART_Baudrate = USART_STD_BAUD_115200;
    usart1.USART_Config.USART_NumberOfStopBits = USART_STOPBITS_1;
    usart1.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart1.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    usart1.USART_Config.USART_HWFLowControl = USART_HW_FLOW_CTRL_NONE;
    USART_PeriClockControl(USART1, ENABLE);
    USART_Init(&usart1);
    USART_PeripheralControl(USART1, ENABLE);
}

int main(void)
{
    SystemClock_Config();
    MX_GPIO_Init();
    MX_USART1_UART_Init();

    boot_ctrl_t bcb;
    read_bcb(&bcb);
    if(bcb.pending) {
        bcb.pending = 0;
        bcb.trial_count = 0;
        write_bcb(&bcb);
    }
    bootloader_main();
    while (1)
    {
    }
}

void Error_Handler(void) {
    while(1);
}

void NVIC_SystemReset(void) {
    SCB->AIRCR = (0x5FA << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk;
    while(1);
}