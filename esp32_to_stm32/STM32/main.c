/**
 ******************************************************************************
 * @file           : main.c
 * @author         : STM32F103C8T6 Bootloader
 * @brief          : Main program body for bootloader
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "stm32f103xx.h"
#include "bootloader.h"

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/**
 * @brief Main function - Bootloader entry point
 */
int main(void)
{
    // Initialize bootloader
    Bootloader_Init();
    // Check if there's a valid application to run
    if(Check_App_Valid(APP_CURRENT_START))
    {
        // Wait for firmware update request for 3 seconds
        uint32_t timeout_counter = 0;
        const uint32_t timeout_limit = 5000000; // Approximately 5 seconds

        while(timeout_counter < timeout_limit)
        {
            // Check for FW_REQUEST command
            if(cmd_ready)
            {
                cmd_ready = false;

                // Copy command to buffer and null terminate
                memcpy(cmd_buffer, rx_buffer, rx_index);
                cmd_buffer[rx_index] = '\0';
                rx_index = 0;

                // Check if it's firmware request
                if(strncmp((char*)cmd_buffer, CMD_FW_REQUEST, strlen(CMD_FW_REQUEST)) == 0)
                {
                    // Process firmware update
                    Process_FW_Request();
                    break; // Exit timeout loop and enter bootloader main loop
                }
            }

            timeout_counter++;

            // Toggle LED to show bootloader is waiting
            if(timeout_counter % 100000 == 0)
            {
                LED_Toggle();
            }
        }

        // If no firmware update request, jump to application
        if(timeout_counter >= timeout_limit)
        {
            Jump_To_App(APP_CURRENT_START);
        }
    }

    // If no valid application or firmware update requested, stay in bootloader
    Bootloader_Main();

    // Should never reach here
    return 0;
}
