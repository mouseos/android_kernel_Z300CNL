/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _PLATFORM_FOCAL_FT5826S_H_
#define _PLATFORM_FOCAL_FT5826S_H_

#include <linux/types.h>

/* The platform data for the focal ft5826s touchscreen driver */

struct fts_Upgrade_Info 
{
        u8 CHIP_ID;
        u8 TPD_MAX_POINTS;
        u8 AUTO_CLB;
	u16 delay_aa;	      	/*delay of write FT_UPGRADE_AA */
	u16 delay_55;	      	/*delay of write FT_UPGRADE_55 */
	u8 upgrade_id_1;      	/*upgrade id 1 */
	u8 upgrade_id_2;      	/*upgrade id 2 */
	u16 delay_readid;     	/*delay of read id */
	u16 delay_erase_flash;	/*delay of earse flash*/
};

struct fts_ts_platform_data {
	struct fts_Upgrade_Info info;
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	bool psensor_support;
	int (*power_init) (bool);
	int (*power_on) (bool);
};

#endif /* _PLATFORM_FOCAL_FT5826S_H_ */
