#include "stm32h7xx_hal.h"

#define ORIGIN_X 100
#define ORIGIN_Y 0

char touch_init(I2C_HandleTypeDef *phi2c);
int touch_pos(I2C_HandleTypeDef *phi2c, unsigned short *px, unsigned short *py, unsigned short *status);
