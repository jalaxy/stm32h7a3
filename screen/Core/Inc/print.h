#define WINDOW_HEIGHT 600
#define WINDOW_WIDTH 800
#define MARGIN_X 10
#define MARGIN_Y 10
#define POS(x, y) (((unsigned int)(x) << 16) + (unsigned int)(y))
#define X(pos) ((unsigned int)(pos) >> 16)
#define Y(pos) ((unsigned int)(pos) & (unsigned int)(0xffff))
#define IN_WINDOW(x, y) ((x) < WINDOW_WIDTH && (x) >= 0 && (y) < WINDOW_HEIGHT && (y) >= 0)
#define draw_dot(p, c) pixels_565[Y(p)][X(p)] = (c)

typedef unsigned int pos_t;
typedef unsigned short color_t;
extern unsigned short pixels_565[WINDOW_HEIGHT][WINDOW_WIDTH];
extern float linespace;

pos_t _putc(pos_t pos, char ch);
pos_t _puts(pos_t pos, char *s);
pos_t _putul(pos_t pos, unsigned long ul, int base);
pos_t _putl(pos_t pos, long l, int base);
pos_t _putf(pos_t pos, float f, int n);
pos_t clrscreen();
pos_t scrollup(pos_t pos, int n);
void draw_line(pos_t a, pos_t b, color_t c);
void draw_ellipse(pos_t ct, pos_t r, color_t c);
