//#include "stm32f1xx_hal.h"
//#include "stm32f1xx_hal_iwdg.h"
//void init_iwdg(uint32_t timeout_ms) {
//    IWDG_HandleTypeDef hiwdg;
//    hiwdg.Instance = IWDG;
//    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;  // ~32kHz /32 =1kHz
//    hiwdg.Init.Reload = (timeout_ms * 40) / 32;  // Adjust for LSI~40kHz
//    HAL_IWDG_Init(&hiwdg);
//}
//void refresh_iwdg() { HAL_IWDG_Refresh(&IWDG_HandleTypeDef{}); }  // Use global if needed