#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include "stm32f4xx_hal.h"

void BL_Execute_Bootloader(void);
void Bootloader_Jump_To_User_App(void);

#endif /* BOOTLOADER_H */
