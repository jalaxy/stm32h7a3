#include "stm32h7xx_hal.h"

char touch_init(I2C_HandleTypeDef *phi2c);
char touch_pos(I2C_HandleTypeDef *phi2c, unsigned short *px, unsigned short *py, unsigned short *status);
