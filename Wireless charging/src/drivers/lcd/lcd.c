/*
 * 	miaofng@2010 initial version
 */

#include "config.h"
#include "lcd.h"
#include <stdlib.h>
#include <string.h>
#include "ulp/sys.h"
#include "common/bitops.h"
#include "lpt.h"

#ifdef CONFIG_FONT_TNR08X16
#include "ascii8x16.h"
#else
#include "ascii16x32.h"
#endif

struct list_head lcd_devs = LIST_HEAD_INIT(lcd_devs);

int lcd_add(const struct lcd_dev_s *dev, const char *name, int type)
{
	struct lcd_s *lcd = sys_malloc(sizeof(struct lcd_s));
	lcd -> dev = dev;
	lcd -> name = name;
	lcd -> type = 0;
	lcd -> gram = NULL;
	lcd -> font = ascii_16x32;
	lcd -> fgcolor = LCD_FGCOLOR_DEF;
	lcd -> bgcolor = LCD_BGCOLOR_DEF;
	lcd -> xres = 0;
	lcd -> yres = 0;
	lcd -> rot = LCD_ROT_DEF;
	list_add(&lcd -> list, &lcd_devs);

#ifdef CONFIG_FONT_TNR08X16
	lcd -> font = ascii_8x16;
#endif
	//lcd type
	if(type & LCD_TYPE_CHAR)
		lcd -> type |= LCD_TYPE_CHAR;
	if(type & LCD_TYPE_AUTOCLEAR) {
		lcd -> type |= LCD_TYPE_AUTOCLEAR;
		lcd ->gram = sys_malloc(dev ->xres + 1);
		memset(lcd ->gram, 0, dev ->xres + 1);
	}

	//get real resolution
	int xres, yres;
	lcd_get_res(lcd, &xres, &yres);
	lcd->xres = xres;
	lcd->yres = yres;
	return 0;
}

struct lcd_s *lcd_get(const char *name)
{
	struct list_head *pos;
	struct lcd_s *lcd;

	list_for_each(pos, &lcd_devs) {
		lcd = list_entry(pos, lcd_s, list);
		if( name == NULL ) {
			//default to the first one
			break;
		}

		if( !strcmp(lcd -> name, name) )
			break;
	}

	return lcd;
}

const char *lcd_get_font(struct lcd_s *lcd, int *w, int *h)
{
	const char *p = lcd -> font;
	*w = (int) p[0];
	*h = (int) p[1];
	return &p[2];
}

int lcd_get_res(struct lcd_s *lcd, int *x, int *y)
{
	int w, h;

	if(lcd -> xres) {
		*x = lcd -> xres;
		*y = lcd -> yres;
		return 0;
	}

	//to do calculation at the first time
	lcd_get_font(lcd, &w, &h);
	if(LCD_TYPE(lcd) == LCD_TYPE_CHAR) {
		*x = lcd -> dev -> xres * w;
		*y = lcd -> dev -> yres * h;
		return 0;
	}

	if(lcd -> rot == LCD_ROT_090 || lcd -> rot == LCD_ROT_270) {
		*x = lcd -> dev -> yres;
		*y = lcd -> dev -> xres;
	}
	else {
		*x = lcd -> dev -> xres;
		*y = lcd -> dev -> yres;
	}
	return 0;
}

int lcd_init(struct lcd_s *lcd)
{
	struct lcd_cfg_s cfg;
	cfg.rot = lcd -> rot;
#if CONFIG_CUSTOM_DISPLAY != 1
	cfg.bus = &lpt;
#endif
	int ret = lcd -> dev -> init(&cfg);
	if( !ret ) {
		ret = lcd_clear_all(lcd);
	}
	return ret;
}

/*coordinate transformation, virtual -> real, rotate angle is centered top&left of lcd, clockwise direction*/
static int __lcd_transform(int angle, int x, int y, int w, int h, int *px, int *py)
{
	int lcd_x, lcd_y, lcd_i;

	switch (angle) {
	case 90:
		lcd_x = y;
		lcd_y = w - x - 1;
		lcd_i = lcd_y * h + lcd_x;
		break;
	case 180:
		lcd_x = w - x - 1;
		lcd_y = h - y - 1;
		lcd_i = lcd_y * w + lcd_x;
		break;
	case 270:
		lcd_x = h - y - 1;
		lcd_y = x;
		lcd_i = lcd_y * h + lcd_x;
		break;
	default:
		lcd_x = x;
		lcd_y = y;
		lcd_i = lcd_y * w + lcd_x;
	}

	*px = lcd_x;
	*py = lcd_y;

	return lcd_i;
}

static int lcd_transform(struct lcd_s *lcd, int *x, int *y)
{
	int w, h;
	w = lcd->xres;
	h = lcd->yres;
	return __lcd_transform(lcd->rot * 90, *x, *y, w, h, x, y);
}

static int lcd_set_window(struct lcd_s *lcd, int x, int y, int w, int h)
{
	int x0, y0, x1, y1;

	x0 = x;
	y0 = y;
	x1 = x + w - 1;
	y1 = y + h - 1;
	if(x0 < 0 || y0 < 0 || x1 >= lcd -> xres || y1 >= lcd -> yres) {
		return -1;
	}

	//virtual -> real
	lcd_transform( lcd, &x0, &y0 );
	lcd_transform( lcd, &x1, &y1 );
	if(x0 > x1) {
		w = x0;
		x0 = x1;
		x1 = w;
	}
	if(y0 > y1) {
		w = y0;
		y0 = y1;
		y1 = w;
	}
	return lcd -> dev -> setwindow( x0, y0, x1, y1 );
}

int lcd_bitblt(struct lcd_s *lcd, const void *bits, int x, int y, int w, int h)
{
	short i, v, n = w * h;
	int _x, _y, ret = lcd_set_window(lcd, x, y, w, h);
	static short gram[32*16];

	if(n <= 32 * 16) {
		for(y = 0; y < h; y ++) {
			for(x = 0; x < w; x ++) {
				i = y * w + x;
				v = bit_get(i, bits);
				v = (v != 0) ? lcd -> fgcolor : lcd -> bgcolor;
				i = __lcd_transform(lcd->rot * 90U, x, y, w, h, &_x, &_y);
				gram[i] = v;
			}
		}

		ret = lcd ->dev ->wgram(gram, n, 0);
	}
	else {
		for(i = 0; i < n && ret == 0; i ++) {
			v = bit_get(i, bits);
			v = (v != 0) ? lcd -> fgcolor : lcd -> bgcolor;
			ret = lcd ->dev ->wgram(&v, 1, 0);
		}
	}

	return ret;
}

int lcd_imageblt(struct lcd_s *lcd, const void *image, int x, int y, int w, int h)
{
	int ret = lcd_set_window(lcd, x, y, w, h);
	if(!ret) {
		//warnning!!! be carefull of the rgb format here ...
		ret = lcd -> dev -> wgram(image, w * h, 0);
	}

	return ret;
}

static int lcd_clear_pixel(struct lcd_s *lcd, int x, int y, int w, int h)
{
	if(!lcd_set_window(lcd, x, y, w, h))
		return lcd -> dev -> wgram(NULL, w * h, lcd -> bgcolor);

	return -1;
}

static int lcd_clear_char(struct lcd_s *lcd, int x, int y, int w, int h)
{
	int ret = -1;
	int i;
	char *str;

	str = sys_malloc(w + 1);
	memset(str, ' ', w);
	str[w] = 0;

	for(i = 0; i < h; i++) {
		ret = lcd -> dev -> puts(x, y + i, str);
		if(ret)
			break;
	}

	sys_free(str);
	return ret;
}

int lcd_clear(struct lcd_s *lcd, int x, int y, int w, int h)
{
	int ret, fw, fh;
	if(lcd ->type & LCD_TYPE_AUTOCLEAR)
		return 0;

	lcd_get_font(lcd, &fw, &fh);
	if(LCD_TYPE(lcd) == LCD_TYPE_CHAR) {
		x /= fw;
		w /= fw;
		y /= fh;
		h /= fh;
		ret = lcd_clear_char(lcd, x, y, w, h);
	}
	else {
		ret = lcd_clear_pixel(lcd, x, y, w, h);
	}

	return ret;
}

int lcd_clear_all(struct lcd_s *lcd)
{
	int xres, yres;
	lcd_get_res(lcd, &xres, &yres);
	return lcd_clear(lcd, 0, 0, xres, yres);
}

/*pixel based char output*/
static int lcd_putchar_pixel(struct lcd_s *lcd, int x, int y, char c)
{
	int w, h;
	const char *font = lcd_get_font(lcd, &w, &h);

	font += (c - '!' + 1) << 6;
	return lcd_bitblt(lcd, font, x, y, w, h);
}

int lcd_puts(struct lcd_s *lcd, int x, int y, const char *str)
{
	int ret;
	int w, h;
	char *p;

	lcd_get_font(lcd, &w, &h);

	//char based lcd module
	if(LCD_TYPE(lcd) == LCD_TYPE_CHAR) {
		//convert x, y to char based coordinate system
		x /= w;
		y /= h;
		if( lcd ->type & LCD_TYPE_AUTOCLEAR) {
			p = lcd ->gram + x;
			if(!memcmp(p , str, strlen(str)))
				return 0;

			//not match
			memcpy(p, str, strlen(str));
			x = 0;
			y = 0;
			str = lcd ->gram;
		}
		return lcd -> dev -> puts(x, y, str);
	}

	//pixel based lcd module
	while (*str) {
		ret = lcd_putchar_pixel(lcd, x, y, *str);
		if(ret)
			break;
		str ++ ;
		x += w;
	}

	return ret;
}

int lcd_line(struct lcd_s *lcd, int x, int y, int dx, int dy)
{
	/*slash is not supported yet*/
	assert((dx == 0) || (dy == 0));

	if(dx < 0) {
		dx = 0 - dx;
		x += dx;
	}
	if(dy < 0) {
		dy = 0 - dy;
		y += dy;
	}

	dx += (dx == 0) ? lcd->line_width: 0;
	dy += (dy == 0) ? lcd->line_width: 0;
	lcd_set_window(lcd, x, y, dx, dy);
	lcd->dev->wgram(NULL, dx * dy, lcd->fgcolor);
	return 0;
}

int lcd_box(struct lcd_s *lcd, int x, int y, int w, int h)
{
	lcd_set_window(lcd, x, y, w, h);
	lcd->dev->wgram(NULL, w*h, lcd->fgcolor);
	return 0;
}

int lcd_rect(struct lcd_s *lcd, int x, int y, int w, int h)
{
	lcd_line(lcd, x, y, w, 0);
	lcd_line(lcd, x, y, 0, h);
	lcd_line(lcd, x, y + h - 1, w, 0);
	lcd_line(lcd, x + w - 1, y, 0, h);
	return 0;
}
