#include "print.h"
#include "monofont.h"
#include "math.h"
#include "string.h"

unsigned short pixels_565[WINDOW_HEIGHT][WINDOW_WIDTH];
float linespace = 1.5;

pos_t _putc(pos_t pos, char ch) {
	if (ch == '\n')
		return POS(MARGIN_X, Y(pos) + linespace * MONO_HEIGHT);
	unsigned short x = X(pos), y = Y(pos);
	if (x + MONO_WIDTH > WINDOW_WIDTH - MARGIN_X) {
		x = MARGIN_X;
		y += linespace * MONO_HEIGHT;
	}
	if (y + MONO_HEIGHT > WINDOW_HEIGHT - MARGIN_Y)
		y = Y(scrollup(POS(x, y), 1));
	unsigned short (*img)[MONO_WIDTH] =
			(unsigned short (*)[MONO_WIDTH]) monofont[(int) ch];
	for (int i = 0; i < MONO_HEIGHT; i++)
		for (int j = 0; j < MONO_WIDTH; j++)
			pixels_565[y + i][x + j] = img[i][j];
	return POS(x + MONO_WIDTH, y);
}

pos_t _puts(pos_t pos, char *s) {
	while (*s)
		pos = _putc(pos, *s++);
	return pos;
}

pos_t _putul(pos_t pos, unsigned long ul, int base) {
	unsigned long div = ul / base;
	char digit = ul % base;
	digit += digit < 10 ? '0' : 'A' - 10;
	return _putc(div ? _putul(pos, div, base) : pos, digit);
}

pos_t _putl(pos_t pos, long l, int base) {
	unsigned long ul;
	if (l >= 0)
		ul = l;
	else {
		ul = -l;
		pos = _putc(pos, '-');
	}
	return _putul(pos, ul, base);
}

pos_t _putf(pos_t pos, float f, int n) {
	if (f < 0) {
		f = -f;
		pos = _putc(pos, '-');
	}
	pos = _putul(pos, (unsigned long) f, 10);
	pos = _putc(pos, '.');
	f = (f - (unsigned long) f) * 10.f;
	for (int i = 0; i < n; i++) {
		pos = _putc(pos, '0' + (unsigned long) f);
		f = (f - (unsigned long) f) * 10.f;
	}
	return pos;
}

pos_t clrscreen(color_t c) {
	for (int i = 0; i < WINDOW_HEIGHT; i++)
		for (int j = 0; j < WINDOW_WIDTH; j++)
			pixels_565[i][j] = c;
	return POS(MARGIN_X, MARGIN_Y);
}

pos_t scrollup(pos_t pos, int n) {
	int d = n * linespace * MONO_HEIGHT;
	for (int i = 0; i < WINDOW_HEIGHT - d; i++)
		for (int j = 0; j < WINDOW_WIDTH; j++)
			pixels_565[i][j] = pixels_565[i + d][j];
	for (int i = WINDOW_HEIGHT - d; i < WINDOW_HEIGHT; i++)
		for (int j = 0; j < WINDOW_WIDTH; j++)
			pixels_565[i][j] = 0xffff;
	return POS(X(pos), Y(pos) - d);
}

void draw_line(pos_t a, pos_t b, color_t c) {
	for (float t = .0f; t < 1.f; t += 1e-3) {
		int x = X(a) * t + X(b) * (1 - t), y = Y(a) * t + Y(b) * (1 - t);
		if (IN_WINDOW(x, y))
			draw_dot(POS(x, y), c);
	}
}

void draw_ellipse(pos_t ct, pos_t r, color_t c) {
	float dpi = 2 * acos(-1);
	for (float t = .0f; t < dpi; t += 1e-3) {
		int x = X(ct) + X(r) * cos(t), y = Y(ct) + Y(r) * sin(t);
		if (IN_WINDOW(x, y))
			draw_dot(POS(x, y), c);
	}
}
