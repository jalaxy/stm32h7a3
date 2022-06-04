#include "stm32h7xx_hal.h"

#define WINDOW_HEIGHT 600
#define WINDOW_WIDTH 800
#define MARGIN_X 10
#define MARGIN_Y 10
#define ORIGIN_X 100
#define ORIGIN_Y 0
#define MONO_HEIGHT 24
#define MONO_WIDTH 12
#define POS(x, y) (((unsigned int)(x) << 16) + (unsigned int)(y))
#define X(pos) ((unsigned int)(pos) >> 16)
#define Y(pos) ((unsigned int)(pos) & (unsigned int)(0xffff))
#define IN_WINDOW(x, y) ((x) < WINDOW_WIDTH && (x) >= 0 && (y) < WINDOW_HEIGHT && (y) >= 0)
#define draw_dot(x, y, c) pixels_565[y][x] = (c)
#define RGB565(r, g, b) (color_t)(((r) >> 3 << 11) | ((g) >> 2 << 5) | ((b) >> 3))
#define color_ratio(c, r) (((color_t)((((c) >> 11) & 0x1f) * (r)) << 11) | \
	((color_t)((((c) >> 5) & 0x3f) * (r)) << 5) | (color_t)(((c) & 0x1f) * (r)))

typedef unsigned int pos_t;
typedef unsigned short color_t;
extern unsigned short pixels_565[WINDOW_HEIGHT][WINDOW_WIDTH];
extern unsigned int monofont[128][144];
extern float linespace;

pos_t _putc(pos_t pos, char ch);
pos_t _puts(pos_t pos, char *s);
pos_t _putul(pos_t pos, unsigned long ul, int base);
pos_t _putl(pos_t pos, long l, int base);
pos_t _putf(pos_t pos, float f, int n);
pos_t clrscreen(color_t c);
pos_t scrollup(pos_t pos, int n);
void fill_rect(pos_t a, pos_t b, color_t c);
void draw_line(pos_t a, pos_t b, color_t c, int stroke);
void draw_ellipse(pos_t ct, pos_t r, color_t c, int stroke);
char touch_reg_init(I2C_HandleTypeDef *phi2c);
int touch_pos(I2C_HandleTypeDef *phi2c, unsigned short *px, unsigned short *py,
		unsigned short *status);
