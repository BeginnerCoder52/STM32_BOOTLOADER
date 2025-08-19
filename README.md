STM32 Bootloader with OTA Firmware Update from ESP32

This project implements an Over-The-Air (OTA) firmware update system for an STM32 microcontroller, facilitated by an ESP32 webserver. The project is organized under the STM32_BOOTLOADER directory, containing four sub-projects: APP_CURRENT, APP_OLD, ESP32_HOST, and BOOTLOADER.
Overview

Purpose: Enable OTA firmware updates for an STM32 microcontroller using an ESP32 module as the host, communicating via UART.
Components:
APP_CURRENT: Current application firmware for STM32, stored at flash address 0x08004000.
APP_OLD: Backup (old) application firmware for STM32, stored at flash address 0x08012000.
BOOTLOADER: STM32 bootloader handling firmware updates, validation, and switching between application partitions.
ESP32_HOST: ESP32 firmware managing the webserver and UART communication to send firmware to STM32.


Key Features:
Dual-partition firmware management (CURRENT and OLD) for safe updates.
CRC32 checksum for firmware integrity.
UART-based communication protocol for OTA updates.
Fallback mechanism to boot the old firmware if the new one fails.
ESP32 webserver for firmware upload and update initiation.



Directory Structure

STM32_BOOTLOADER/
APP_CURRENT/: Current STM32 application code.
APP_OLD/: Backup STM32 application code.
BOOTLOADER/: STM32 bootloader code for OTA updates.
ESP32_HOST/: ESP32 code for hosting the webserver and managing OTA.



How It Works

ESP32 Webserver: Hosts a web interface to upload firmware (stored in SPIFFS). It calculates the CRC32 of the firmware and sends it to the STM32 via UART.
STM32 Bootloader: Receives firmware data, writes it to the CURRENT partition, verifies integrity using CRC32, and sets a boot control block (BCB) for trial boots.
Firmware Validation: The bootloader checks the validity of the CURRENT partition. If valid, it boots; otherwise, it falls back to the OLD partition.
Rollback Mechanism: If the new firmware fails after a set number of trials (e.g., 3), the bootloader switches to the OLD partition.

Prerequisites

Hardware:
STM32 microcontroller (e.g., STM32F1 series).
ESP32 module with UART connectivity to STM32.
LED for testing (optional, as in APP_OLD).


Software:
STM32CubeIDE or similar for STM32 development.
ESP-IDF for ESP32 development.
UART connection configured at 115200 baud.



Setup and Usage

Compile and Flash:
Build and flash BOOTLOADER to STM32 at 0x08000000.
Build and flash APP_CURRENT to STM32 at 0x08004000.
Build and flash APP_OLD to STM32 at 0x08012000.
Build and flash ESP32_HOST to the ESP32 module.


Upload Firmware:
Access the ESP32 webserver to upload a new firmware binary (firmware.bin).
The ESP32 sends the firmware to the STM32 via UART.


OTA Update:
The STM32 bootloader receives and writes the firmware to the CURRENT partition.
After verification, it sets the BCB to boot the new firmware with a trial count.
If the new firmware fails, the bootloader reverts to the OLD partition.



Notes

Ensure UART pins (e.g., TX: 17, RX: 16 on ESP32) match the STM32 configuration.
The APP_OLD project includes a simple LED toggle for testing.
The bootloader uses a footer and BCB for firmware validation and boot control.
