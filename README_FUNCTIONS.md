Function Explanations for STM32 Bootloader OTA Projects
This document lists key functions in the STM32_BOOTLOADER projects (APP_CURRENT, APP_OLD, BOOTLOADER, ESP32_HOST) in table format, detailing their roles in the OTA firmware update system.
APP_CURRENT



File
Function
Description



main.c
main
Initializes STM32 peripherals (HAL, clock, GPIO) and enters an infinite loop.


system_stm32f1xx.c
-
Sets vector table at 0x08004000 for the CURRENT partition.


APP_OLD



File
Function
Description



main.c
main
Initializes STM32 peripherals, toggles LED every 100ms in an infinite loop.


system_stm32f1xx.c
-
Sets vector table at 0x08012000 for the OLD partition.


BOOTLOADER



File
Function
Description



bootloader.h
bootloader_main
Manages boot process, validates partitions, jumps to app or enters OTA mode.


bootloader.h
ota_receive_loop
Handles OTA firmware reception via UART, writes to flash, verifies integrity.


flash_if.c
flash_unlock
Unlocks STM32 flash for writing.


flash_if.c
flash_lock
Locks STM32 flash after writing.


flash_if.c
flash_erase_range
Erases specified flash range (page-aligned).


flash_if.c
flash_write
Writes data to flash with verification, in half-word increments.


flash_if.c
crc32_update
Updates CRC32 checksum for a single byte.


flash_if.c
crc32_calc
Calculates CRC32 checksum for a data buffer.


flash_if.c
read_bcb
Reads boot control block (BCB) from flash.


flash_if.c
write_bcb
Writes BCB to flash after erasing the page.


flash_if.c
read_footer
Reads application footer from specified address.


flash_if.c
write_footer
Writes application footer to flash after erasing the page.


bootloader.c
is_partition_valid
Verifies partition validity using footer’s magic number and validity flag.


bootloader.c
part_base
Returns base address of a partition (CUR_START or OLD_START).


bootloader.c
jump_to_app
Deinitializes peripherals, sets stack pointer and vector table, jumps to app.


bootloader.c
bootloader_main
Initializes BCB, selects boot target, jumps to valid partition or OTA mode.


bootloader.c
ota_receive_loop
Receives firmware via UART, verifies checksums/CRC32, writes to CURRENT.


bootloader.c
copy_partition
Copies one partition to another, including footer, for fallback.


main.c
main
Initializes STM32, clears pending BCB flags, initializes UART, calls bootloader_main.


ESP32_HOST



File
Function
Description



crc32.c
calculate_crc32
Computes CRC32 checksum of a data buffer using esp_crc32_le.


uart_link.c
uart_init
Configures UART with TX/RX pins, 115200 baud, 8N1, no flow control.


uart_link.c
write_bytes
Sends raw bytes over UART.


uart_link.c
send_BEGIN
Sends OTA start command ('B'), partition, size, CRC32, and version.


uart_link.c
send_DATA
Sends data chunk with offset, length, and payload.


uart_link.c
send_END
Sends OTA end command ('E').


uart_link.c
send_SET_PENDING
Sends command ('P') to set pending update with partition and trial count.


uart_link.c
send_REBOOT
Sends reboot command ('R').


uart_link.c
wait_response
Waits for expected UART response (e.g., 'O' for OK, 'A' for ACK).


spiffs.c
mount_spiffs
Mounts SPIFFS filesystem to store firmware binary.


main.c
app_main
Initializes UART, SPIFFS, reads firmware, sends it in chunks, sets pending/reboot.

