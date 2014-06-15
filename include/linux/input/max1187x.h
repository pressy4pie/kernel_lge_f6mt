/* include/linux/input/max1187x.h
 *
 * Copyright (c)2012 Maxim Integrated Products, Inc.
 *
 * Driver Version: 3.0.7.1
 * Release Date: Mar 19, 2013
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MAX1187X_H
#define __MAX1187X_H

#define MAX1187X_NAME   "max1187x"
#define MAX1187X_TOUCH  MAX1187X_NAME "_touchscreen_0"
#define MAX1187X_KEY    MAX1187X_NAME "_key_0"

#define MAX_WORDS_COMMAND  9  /* command address space 0x00-0x09 minus header
				=> 9 command words maximum */
#define MAX_WORDS_REPORT   245  /* address space 0x00-0xFF minus 0x00-0x09 for
				commands minus header, maximum 1 report packet*/
#define MAX_WORDS_COMMAND_ALL  (15 * MAX_WORDS_COMMAND)  /* maximum 15 packets
				9 payload words each */

#define MAX1187X_NUM_FW_MAPPINGS_MAX    5
#define MAX1187X_TOUCH_COUNT_MAX        10
#define MAX1187X_TOUCH_REPORT_RAW       0x0800
#define MAX1187X_TOUCH_REPORT_BASIC     0x0801
#define MAX1187X_TOUCH_REPORT_EXTENDED  0x0802
/* #define MAX1187X_PROTOCOL_A */
#define MAXIM_TOUCH_REPORT_MODE 1 /* 1=basic, 2=extended */
#define MAX_REPORT_READERS		5
#define FW_DOWNLOAD_FEATURE
/* #define CONFIG_DOWNLOAD_FEATURE */
/* #define MAX11871 */
#define DEBUG_STRING_LEN_MAX 60
#define MAX_FW_RETRIES 5

#define MAX1187X_PI 205887 /* pi multiplied by 2^16 */
//start hoseong.han
#if MAXIM_TOUCH_REPORT_MODE == 2
//end hoseong.han
/* tanlist - array containing tan(i)*(2^16-1) for i=[0,45], i in degrees */
//start hoseong.han
//u16 tanlist[] = {0, 1144, 2289, 3435, 4583, 5734,
static u16 tanlist[] = {0, 1144, 2289, 3435, 4583, 5734,
//end hoseong.han
			6888, 8047, 9210, 10380, 11556, 12739,
			13930, 15130, 16340, 17560, 18792, 20036,
			21294, 22566, 23853, 25157, 26478, 27818,
			29178, 30559, 31964, 33392, 34846, 36327,
			37837, 39377, 40951, 42559, 44204, 45888,
			47614, 49384, 51202, 53069, 54990, 56969,
			59008, 61112, 63286, 65535};
//start hoseong.han
#endif
//end hoseong.han

struct max1187x_touch_report_header {
	u16 header;
	u16 report_id;
	u16 report_size;
	u16 touch_count:4;
	u16 reserved0:12;
	u16 button0:1;
	u16 button1:1;
	u16 button2:1;
	u16 button3:1;
	u16 reserved1:12;
	u16 framecounter;
};

struct max1187x_touch_report_basic {
	u16 finger_id:4;
	u16 reserved0:12;
	u16 x:12;
	u16 reserved1:4;
	u16 y:12;
	u16 reserved2:4;
	u16 reserved3:8;
	u16 z:8;
};

struct max1187x_touch_report_extended {
	u16 finger_id:4;
	u16 reserved0:12;
	u16 x:12;
	u16 reserved1:4;
	u16 y:12;
	u16 reserved2:4;
	u16 reserved3:8;
	u16 z:8;
	s16 xspeed;
	s16 yspeed;
	s8 xpixel;
	s8 ypixel;
	u16 area;
	u16 xmin;
	u16 xmax;
	u16 ymin;
	u16 ymax;
};

struct max1187x_fw_mapping {
	u32				config_id;
	u32				chip_id;
	char			*filename;
	u32				filesize;
	u32				filecrc16;
	u32				file_codesize;
};

struct max1187x_pdata {
	u32			gpio_tirq;
	u32			num_fw_mappings;
	struct max1187x_fw_mapping  fw_mapping[MAX1187X_NUM_FW_MAPPINGS_MAX];
	u32			defaults_allow;
	u32			default_chip_config;
	u32			default_chip_id;
	u32			i2c_words;
	#define MAX1187X_REVERSE_X  0x0001
	#define MAX1187X_REVERSE_Y  0x0002
	#define MAX1187X_SWAP_XY    0x0004
	u32			coordinate_settings;
	u32			panel_margin_xl;
	u32			lcd_x;
	u32			panel_margin_xh;
	u32			panel_margin_yl;
	u32			lcd_y;
	u32			panel_margin_yh;
	u32			num_rows;
	u32			num_cols;
//                                                            
	u16			button_code[4];
//                                               
//start hoseong.han
	int			(*power_func)(int on);
//end hoseong.han
};

enum{
	DEBUG_NO_MSG = 0,
	DEBUG_GET_TOUCH_DATA  = (1U << 2), // 4
	DEBUG_BASIC_INFO = (1U << 3), // 8
};

enum{
        KEY_RELEASED = 0,
        KEY_PRESSED  = 1,
        KEY_CANCLED  = 0xff,
};

enum{
        PWR_OFF = 0,
        PWR_ON,
        PWR_SLEEP,
        PWR_WAKE,
};

#endif /* __MAX1187X_H */
