/* include/linux/board_asustek.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2012, Code Aurora Forum. All rights reserved.
 * Copyright (c) 2012, LGE Inc.
 * Copyright (c) 2012-2014, ASUSTek Computer Inc.
 * Author: Paris Yeh <paris_yeh@asus.com>
 *	   Hank Lee  <hank_lee@asus.com>
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

#ifndef __ASM_INTEL_MID_BOARD_ASUSTEK_H
#define __ASM_INTEL_MID_BOARD_ASUSTEK_H
#include <linux/sfi.h>

#ifndef CONFIG_SFI_PCB
#define CONFIG_SFI_PCB
#endif
typedef enum {
	HW_REV_INVALID = -1,
	HW_REV_MAX = 8
} hw_rev;

typedef enum {
	PROJECT_ID_INVALID = -1,
	PROJECT_ID_MAX = 16,
} project_id;

typedef enum {
	SUB_PROJECT_ID_INVALID = -1,
	SUB_PROJECT_ID_MAX = 4
} sub_project_id;

typedef enum {
	LCD_TYPE_INVALID = -1,
	LCD_TYPE_MAX = 2,
	LCD_TYPE_1_MAX = 4
} lcd_type;

typedef enum {
	TP_TYPE_INVALID = -1,
	TP_TYPE_1_MAX = 2,
	TP_TYPE_MAX = 8
} tp_type;

typedef enum {
	CAM_FRONT_INVALID = -1,
	CAM_FRONT_MAX = 2
} cam_front;

typedef enum {
	CAM_REAR_INVALID = -1,
	CAM_REAR_1_MAX = 2,
	CAM_REAR_MAX = 4
} cam_rear;

typedef enum {
	RF_SKU_INVALID = -1,
	RF_SKU_MAX = 4,
	RF_SKU_1_MAX = 8
} rf_sku;

typedef enum {
	DDR_SKU_INVALID = -1,
	DDR_SKU_MAX = 2
} ddr_sku;

typedef enum {
	CARDIZ_ID_INVALID = -1,
	CARDIZ_ID_MAX = 2
} cardiz_id;

typedef enum {
	WIFI_SKU_INVALID = -1,
	WIFI_SKU_MAX = 2
} wifi_sku;

struct asustek_pcbid_platform_data {
	const char *UUID;
	struct resource *resource0;
	u32 nr_resource0;
	struct resource *resource1;
	u32 nr_resource1;
};

hw_rev asustek_get_hw_rev(void);

project_id asustek_get_project_id(void);

sub_project_id asustek_get_sub_project_id(void);

lcd_type asustek_get_lcd_type(void);

tp_type asustek_get_tp_type(void);

cam_front asustek_get_camera_front(void);

cam_rear asustek_get_camera_rear(void);

rf_sku asustek_get_rf_sku(void);

ddr_sku asustek_get_ddr_sku(void);

cardiz_id asustek_get_cardiz_id(void);

wifi_sku asustek_get_wifi_sku(void);

bool use_ov5693(void);

int sfi_parse_oem1(struct sfi_table_header *);
void sfi_parsing_done(bool);

void config_pcbid_suspend_resume(int suspend);
#endif /* __ASM_INTEL_MID_BOARD_ASUSTEK_H */
