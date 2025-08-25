#include "bootloader.h"

int main(void) {
    RCC->CR |= (1 << 0);
    while (!(RCC->CR & (1 << 1)));
    RCC->CFGR = (0x1 << 0);
    RCC->APB2ENR |= (1 << 0);

    Bootloader_Main();

    while (1);
}
