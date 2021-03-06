/*
 * 	dusk@2010 initial version
 *	miaofng@2010 reused for common lcd debug purpose
 */

#include "config.h"
#include "shell/cmd.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"
#include "pwm.h"

static int cmd_lcd_func(int argc, char *argv[])
{
	int row, col;
	int w, h;
	struct lcd_s *lcd = lcd_get(NULL);
#ifdef CONFIG_DRIVER_PWM2
	int duty;
	pwm_cfg_t cfg = PWM_CFG_DEF;
	const pwm_bus_t *pwm = &pwm23; //zf32 board, for backlight ctrl
#endif

	const char *usage = {
		"usage:\n"
#ifdef CONFIG_DRIVER_PWM2
		"lcd bl hz duty	duty: 0-9\n"
#endif
		"lcd init\n"
		"lcd puts x y str\n"
		"lcd w reg val\n"
		"lcd r reg\n"
	};

	if(argc >= 2) {
		if(argv[1][0] == 'i') { //lcd init
			if(lcd_init(lcd)) {
				printf("lcd module cann't be identified\n");
				return 0;
			}
			lcd_get_res(lcd, &row, &col);
			printf("lcd init: xres = %d, yres = %d\n", row, col);
			lcd_get_font(lcd, &row, &col);
			printf("lcd init: font width = %d, height = %d\n", row, col);
			return 0;
		}
#ifdef CONFIG_DRIVER_PWM2
		if(argv[1][0] == 'b') {
			if(argc > 3) {
				sscanf(argv[2], "%d", &cfg.hz);
				sscanf(argv[3], "%d", &duty);
			}
			else {
				cfg.hz = 500000; /*500KHz*/
				duty = 2;
			}
			cfg.fs = 10;
			pwm -> init(&cfg);
			pwm -> set(duty);
			return 0;
		}
#endif
		if(argv[1][0] == 'w') {
			//write reg
			sscanf(argv[2], "%x", &row);
			sscanf(argv[3], "%x", &col);
			lcd -> dev -> writereg(row, col);
			printf("reg[0x%x] = 0x%x\n", row, col);
			return 0;
		}

		if(argv[1][0] == 'r') {
			sscanf(argv[2], "%x", &row);
			col = lcd -> dev -> readreg(row);
			printf("reg[0x%x] = 0x%x\n", row, col);
			return 0;
		}

		col = atoi(argv[2]);
		row = atoi(argv[3]);
		//x,col means horizontal coordinate
		//y,row means vertical coordinate
		if (lcd->type == LCD_TYPE_CHAR){
			lcd_get_font(lcd, &w, &h);
			row = row * h;
			col = col * w;
		}
		lcd_puts(lcd, col, row, argv[4]);
		return 0;
	}

	printf("%s", usage);
	return 0;
}

const cmd_t cmd_lcd = {"lcd", cmd_lcd_func, "lcd driver debug"};
DECLARE_SHELL_CMD(cmd_lcd)
