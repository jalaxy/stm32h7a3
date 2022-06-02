#include "touch.h"

char touch_init(I2C_HandleTypeDef *phi2c)
{
	unsigned char buf[3];
	char suc = 1;
	buf[0] = 0x00, buf[1] = 0x00;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, buf, 2, HAL_MAX_DELAY) == 0;
	buf[0] = 0xa4, buf[1] = 0x00;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, buf, 2, HAL_MAX_DELAY) == 0;
	buf[0] = 0x80, buf[1] = 0x46;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, buf, 2, HAL_MAX_DELAY) == 0;
	buf[0] = 0x88, buf[1] = 0x0c;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, buf, 2, HAL_MAX_DELAY) == 0;
	buf[0] = 0xa1, buf[1] = 0x30, buf[2] = 0x03;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, buf, 3, HAL_MAX_DELAY) == 0;
	return suc;
}

int touch_pos(I2C_HandleTypeDef *phi2c, unsigned short *px, unsigned short *py, unsigned short *status)
{
	unsigned char addr[5] = { 0x03, 0x09, 0x0f, 0x15, 0x1b }, addr_num = 0x02;
	unsigned char buf[4];
	char suc = 1;
	unsigned char n = 0;
	suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, &addr_num, 1, HAL_MAX_DELAY) == 0;
	suc &= HAL_I2C_Master_Receive(phi2c, 0x70, &n, 1, HAL_MAX_DELAY) == 0;
	if (!suc)
		n = 0;
	for (int i = 0; i < n; i++)
	{
		suc &= HAL_I2C_Master_Transmit(phi2c, 0x70, &addr[i], 1, HAL_MAX_DELAY) == 0;
		suc &= HAL_I2C_Master_Receive(phi2c, 0x70, buf, 4, HAL_MAX_DELAY) == 0;
		if (px)
			px[i] = (((buf[2] & 0x0f) << 8) | buf[3]) - ORIGIN_X;
		if (py)
			py[i] = (((buf[0] & 0x0f) << 8) | buf[1]) - ORIGIN_Y;
		if (status)
			status[i] = buf[0] >> 6;
	}
	return suc ? n : -1;
}
