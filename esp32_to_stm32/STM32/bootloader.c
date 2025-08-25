#include "bootloader.h"
#include "stm32f103xx_rcc_driver.h"

/* Global Variables */
USART_Handle_t husart1;
GPIO_Handle_t hgpio_led;
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t cmd_buffer[CMD_BUFFER_SIZE];
volatile uint16_t rx_index = 0;
volatile bool cmd_ready = false;
volatile Bootloader_State_t bl_state = BL_STATE_IDLE;
uint32_t fw_size = 0;
uint32_t fw_received = 0;
uint32_t fw_checksum = 0;
uint32_t calculated_checksum = 0;
uint32_t current_flash_addr = APP_CURRENT_START;

/**
 * @brief Initialize bootloader peripherals
 */
void Bootloader_Init(void)
{
    // Initialize system clock (assuming 72MHz)
    // Initialize UART1 for communication with ESP32
    UART_Init();

    // Initialize LED GPIO for status indication
    GPIO_Init_LED();

    // Enable UART1 interrupt
    IRQInterruptConfig(IRQ_NO_USART1, 1);

    // Initialize bootloader state
    bl_state = BL_STATE_IDLE;
    rx_index = 0;
    cmd_ready = false;

    // LED on to indicate bootloader is running
    GPIO_WritePin(GPIOC, GPIO_PIN_NO_13, 0); // LED on (active low)
}

/**
 * @brief Initialize UART1 for ESP32 communication
 */
void UART_Init(void)
{
    // Enable GPIOA clock for UART1 pins
    GPIOA_PCLK_EN();

    // Configure PA9 (TX) and PA10 (RX) for UART1
    GPIO_Handle_t uart_pins;

    // PA9 - UART1 TX
    uart_pins.pGPIOx = GPIOA;
    uart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
    uart_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    uart_pins.GPIO_PinConfig.GPIO_PinCfgMode = GPIO_ALT_MODE_USART_TX_FULLDUP;
    uart_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&uart_pins);

    // PA10 - UART1 RX
    uart_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    uart_pins.GPIO_PinConfig.GPIO_PinCfgMode = GPIO_ALT_MODE_USART_RX_FULLDUP;
    GPIO_Init(&uart_pins);

    // Configure UART1
    husart1.pUSARTx = USART1;
    husart1.USART_Config.USART_Baudrate = USART_STD_BAUD_115200;
    husart1.USART_Config.USART_Mode = USART_MODE_TXRX;
    husart1.USART_Config.USART_NumberOfStopBits = USART_STOPBITS_1;
    husart1.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    husart1.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    husart1.USART_Config.USART_HWFLowControl = USART_HW_FLOW_CTRL_NONE;

    USART_Init(&husart1);

    // Start UART1
    USART_Start(USART1);

    // Enable UART1 receive interrupt
    husart1.pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);
    USART_ReceiveDataIT(&husart1, rx_buffer , sizeof(rx_buffer));
}

/**
 * @brief Initialize LED GPIO for status indication
 */
void GPIO_Init_LED(void)
{
    // Enable GPIOC clock
    GPIOC_PCLK_EN();

    // Configure PC13 as output for LED
    hgpio_led.pGPIOx = GPIOC;
    hgpio_led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    hgpio_led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    hgpio_led.GPIO_PinConfig.GPIO_PinCfgMode = GPIO_CFG_OUT_GE_PP;
    hgpio_led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_MEDIUM;

    GPIO_Init(&hgpio_led);
}

/**
 * @brief Main bootloader loop
 */
void Bootloader_Main(void)
{
    while(1)
    {
        if(cmd_ready)
        {
            cmd_ready = false;
            Process_Command();
        }

        // Toggle LED to show bootloader is alive
        static uint32_t led_counter = 0;
        if(++led_counter > 100000)
        {
            led_counter = 0;
            LED_Toggle();
        }
    }
}

/**
 * @brief Process received command
 */
void Process_Command(void)
{
    // Copy command to buffer and null terminate
    memcpy(cmd_buffer, rx_buffer, rx_index);
    cmd_buffer[rx_index] = '\0';

    // Reset rx buffer
    rx_index = 0;

    // Process based on current state
    switch(bl_state)
    {
        case BL_STATE_IDLE:
            if(strncmp((char*)cmd_buffer, CMD_FW_REQUEST, strlen(CMD_FW_REQUEST)) == 0)
            {
                Process_FW_Request();
            }
            break;

        case BL_STATE_WAIT_LENGTH:
            if(strncmp((char*)cmd_buffer, CMD_FW_LENGTH, strlen(CMD_FW_LENGTH)) == 0)
            {
                Process_FW_Length((char*)cmd_buffer + strlen(CMD_FW_LENGTH));
            }
            else
            {
                Send_Response(RESP_ERROR);
                bl_state = BL_STATE_IDLE;
            }
            break;

        case BL_STATE_WAIT_DATA:
            if(strncmp((char*)cmd_buffer, CMD_FW_DATA, strlen(CMD_FW_DATA)) == 0)
            {
                uint16_t data_len = rx_index - strlen(CMD_FW_DATA);
                if(data_len > 0)
                {
                    Process_FW_Data(cmd_buffer + strlen(CMD_FW_DATA), data_len);
                }
                else
                {
                    Send_Response(RESP_ERROR);
                    bl_state = BL_STATE_IDLE;
                }
            }
            else
            {
                Send_Response(RESP_ERROR);
                bl_state = BL_STATE_IDLE;
            }
            break;

        case BL_STATE_WAIT_CHECKSUM:
            if(strncmp((char*)cmd_buffer, CMD_CHECKSUM, strlen(CMD_CHECKSUM)) == 0)
            {
                Process_FW_Checksum((char*)cmd_buffer + strlen(CMD_CHECKSUM));
            }
            else
            {
                Send_Response(RESP_ERROR);
                bl_state = BL_STATE_IDLE;
            }
            break;

        default:
            bl_state = BL_STATE_IDLE;
            break;
    }
}

/**
 * @brief Process firmware request command
 */
void Process_FW_Request(void)
{
    FW_Update_Type_t update_type = Get_Update_Type();

    if(update_type == FW_UPDATE_UPGRADE)
    {
        // Copy current app to old partition
        Copy_App_Current_To_Old();

        // Erase current app partition
        Erase_App_Current();
    }
    else if(update_type == FW_UPDATE_FIRST_TIME)
    {
        // Just erase current app partition
        Erase_App_Current();
    }

    // Reset firmware variables
    fw_size = 0;
    fw_received = 0;
    fw_checksum = 0;
    calculated_checksum = 0;
    current_flash_addr = APP_CURRENT_START;

    // Send ready response and wait for length
    Send_Response(RESP_FW_READY);
    bl_state = BL_STATE_WAIT_LENGTH;
}

/**
 * @brief Process firmware length command
 */
void Process_FW_Length(char* length_str)
{
    // Parse firmware size
    fw_size = 0;
    for(int i = 0; length_str[i] != '\0' && length_str[i] != '\r' && length_str[i] != '\n'; i++)
    {
        if(length_str[i] >= '0' && length_str[i] <= '9')
        {
            fw_size = fw_size * 10 + (length_str[i] - '0');
        }
    }

    // Validate firmware size
    if(fw_size == 0 || fw_size > APP_CURRENT_SIZE)
    {
        Send_Response(RESP_ERROR);
        bl_state = BL_STATE_IDLE;
        return;
    }

    // Send OK and wait for data
    Send_Response(RESP_FW_OK);
    bl_state = BL_STATE_WAIT_DATA;
}

/**
 * @brief Process firmware data
 */
void Process_FW_Data(uint8_t* data, uint16_t length)
{
    // Check if we have space for this data
    if(fw_received + length > fw_size)
    {
        Send_Response(RESP_ERROR);
        bl_state = BL_STATE_IDLE;
        return;
    }

    // Align length to 4 bytes for flash writing
    uint16_t aligned_length = (length + 3) & ~3;
    uint32_t aligned_data[64]; // Buffer for aligned data

    // Copy data to aligned buffer
    memset(aligned_data, 0xFF, sizeof(aligned_data)); // Fill with 0xFF (erased flash value)
    memcpy(aligned_data, data, length);

    // Write data to flash (length in words)
    if(FLASH_Write_Data(current_flash_addr, aligned_data, aligned_length/4) != FLASH_OK)
    {
        Send_Response(RESP_ERROR);
        bl_state = BL_STATE_IDLE;
        return;
    }

    // Update checksum (only for actual data, not padding)
    for(uint16_t i = 0; i < length; i++)
    {
        calculated_checksum += data[i];
    }

    // Update counters
    fw_received += length;
    current_flash_addr += aligned_length;

    // Check if we received all data
    if(fw_received >= fw_size)
    {
        Send_Response(RESP_FW_RECEIVED);
        bl_state = BL_STATE_WAIT_CHECKSUM;
    }
    else
    {
        Send_Response(RESP_FW_OK);
    }
}

/**
 * @brief Process firmware checksum
 */
void Process_FW_Checksum(char* checksum_str)
{
    // Parse received checksum
    uint32_t received_checksum = 0;
    for(int i = 0; checksum_str[i] != '\0' && checksum_str[i] != '\r' && checksum_str[i] != '\n'; i++)
    {
        if(checksum_str[i] >= '0' && checksum_str[i] <= '9')
        {
            received_checksum = received_checksum * 10 + (checksum_str[i] - '0');
        }
    }

    // Verify checksum
    if(received_checksum == calculated_checksum)
    {
        Send_Response(RESP_CHECKSUM_OK);
        bl_state = BL_STATE_COMPLETE;

        // Jump to application after short delay
        for(volatile uint32_t i = 0; i < 1000000; i++);
        Jump_To_App(APP_CURRENT_START);
    }
    else
    {
        Send_Response(RESP_CHECKSUM_ERR);
        bl_state = BL_STATE_IDLE;

        // Erase corrupted firmware
        Erase_App_Current();
    }
}

/**
 * @brief Send response via UART
 */
void Send_Response(const char* response)
{
    USART_SendData(&husart1, (uint8_t*)response, strlen(response));
}

/**
 * @brief Toggle LED
 */
void LED_Toggle(void)
{
    GPIO_Toggle(GPIOC, GPIO_PIN_NO_13);
}

/**
 * @brief Check if application is valid
 */
bool Check_App_Valid(uint32_t addr)
{
    // Read stack pointer from app start
    uint32_t stack_ptr = *((uint32_t*)addr);

    // Check if stack pointer is in valid RAM range
    if(stack_ptr >= 0x20000000 && stack_ptr <= 0x20005000)
    {
        return true;
    }

    return false;
}

/**
 * @brief Get firmware update type
 */
FW_Update_Type_t Get_Update_Type(void)
{
    bool current_valid = Check_App_Valid(APP_CURRENT_START);
    bool old_valid = Check_App_Valid(APP_OLD_START);

    if(!current_valid && !old_valid)
    {
        return FW_UPDATE_FIRST_TIME;
    }
    else
    {
        return FW_UPDATE_UPGRADE;
    }
}
/**
 * @brief Copy application from current to old partition
 */
void Copy_App_Current_To_Old(void)
{
    // First erase old partition
    for(uint32_t addr = APP_OLD_START; addr < APP_OLD_START + APP_OLD_SIZE; addr += FLASH_PAGE_SIZE)
    {
        FLASH_Erase(addr);
    }

    // Copy data from current to old
    uint32_t buffer[64]; // 256 bytes buffer
    for(uint32_t offset = 0; offset < APP_CURRENT_SIZE; offset += sizeof(buffer))
    {
        // Read from current partition
        FLASH_Read_Data(APP_CURRENT_START + offset, buffer, sizeof(buffer)/4);

        // Write to old partition
        FLASH_Write_Data(APP_OLD_START + offset, buffer, sizeof(buffer)/4);
    }
}

/**
 * @brief Erase current application partition
 */
void Erase_App_Current(void)
{
    for(uint32_t addr = APP_CURRENT_START; addr < APP_CURRENT_START + APP_CURRENT_SIZE; addr += FLASH_PAGE_SIZE)
    {
        FLASH_Erase(addr);
    }
}

/**
 * @brief Calculate checksum of memory region
 */
uint32_t Calculate_Checksum(uint32_t start_addr, uint32_t size)
{
    uint32_t checksum = 0;
    uint8_t* ptr = (uint8_t*)start_addr;

    for(uint32_t i = 0; i < size; i++)
    {
        checksum += ptr[i];
    }

    return checksum;
}

/**
 * @brief Jump to application
 */
void Jump_To_App(uint32_t addr)
{
    // Check if application is valid
    if(!Check_App_Valid(addr))
    {
        return; // Stay in bootloader
    }

    // Disable all interrupts
    __asm volatile ("cpsid i" : : : "memory");

    // Disable UART
    USART_Stop(USART1);

    // Get application stack pointer and reset handler
    uint32_t app_stack_ptr = *((uint32_t*)addr);
    uint32_t app_reset_handler = *((uint32_t*)(addr + 4));

    // Set stack pointer
    __asm volatile ("MSR msp, %0" : : "r" (app_stack_ptr) : "memory");

    // Jump to application reset handler
    void (*app_reset_handler_ptr)(void) = (void*)app_reset_handler;
    app_reset_handler_ptr();
}

/**
 * @brief UART receive interrupt callback
 */
void USART_ReceptionEventsCallback(USART_Handle_t *pUSARTHandle)
{
    if(pUSARTHandle->pUSARTx == USART1)
    {
        // Check if RXNE flag is set
        if(pUSARTHandle->pUSARTx->SR & (1 << USART_SR_RXNE))
        {
            // Read received byte
            uint8_t received_byte = (uint8_t)(pUSARTHandle->pUSARTx->DR & 0xFF);

            // Store in buffer if there's space
            if(rx_index < RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] = received_byte;

                // Check for command terminator
                if(received_byte == '\n' || received_byte == '\r')
                {
                    cmd_ready = true;
                }
            }
            else
            {
                // Buffer overflow, reset
                rx_index = 0;
            }
        }
    }
}

/**
 * @brief USART1 interrupt handler
 */
void USART1_IRQHandler(void)
{
    USART_IRQHandling(&husart1);
}

