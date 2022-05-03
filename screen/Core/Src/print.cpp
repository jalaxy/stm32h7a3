#include "print.h"
#include "monofont.h"

unsigned short pixels_565[WINDOW_HEIGHT][WINDOW_WIDTH];
float linespace = 1.5;

pos_t putc(pos_t pos, char ch)
{
    unsigned short x = X(pos), y = Y(pos);
    unsigned short(*img)[MONO_WIDTH] = (unsigned short(*)[MONO_WIDTH])monofont[ch];
    for (int i = 0; i < MONO_HEIGHT; i++)
        for (int j = 0; j < MONO_WIDTH; j++)
            pixels_565[x + i][y + j] = img[i][j];
    if ((x += MONO_WIDTH) >= WINDOW_WIDTH)
    {
        x = 0;
        y += linespace * MONO_HEIGHT;
        if (y >= WINDOW_HEIGHT)
            y = Y(scrollup(POS(x, y), 1));
    }
    return POS(x, y);
}

pos_t puts(pos_t pos, char *s)
{
    while (*s)
        pos = putc(pos, *s++);
    return pos;
}

pos_t scrollup(pos_t pos, int n)
{
    int d = n * linespace * MONO_HEIGHT;
    for (int i = 0; i < WINDOW_HEIGHT - d; i++)
        for (int j = 0; j < WINDOW_WIDTH; j++)
            pixels_565[i][j] = pixels_565[i + d][j];
    for (int i = WINDOW_HEIGHT - d; i < WINDOW_HEIGHT; i++)
        for (int j = 0; j < WINDOW_WIDTH; j++)
            pixels_565[i][j] = 0x0;
    return POS(X(pos), Y(pos) - d);
}
