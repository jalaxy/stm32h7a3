#define WINDOW_HEIGHT 600
#define WINDOW_WIDTH 800
#define POS(x, y) (((unsigned int)(x) << 8) + (unsigned int)(y))
#define X(pos) ((unsigned int)(pos) >> 8)
#define Y(pos) ((unsigned int)(pos) & (unsigned int)(0xffff))

typedef unsigned int pos_t;
extern unsigned short pixels_565[WINDOW_HEIGHT][WINDOW_WIDTH];
extern float linespace;

pos_t putc(pos_t pos, char ch);
pos_t puts(pos_t pos, char *s);
pos_t scrollup(pos_t pos, int n);
