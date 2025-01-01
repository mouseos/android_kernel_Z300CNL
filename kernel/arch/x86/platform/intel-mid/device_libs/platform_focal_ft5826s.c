/*
 * platform_atmel_mxt1666t2.c: atmel mxt1666t2 touch platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c/focal_ft5826s.h>
#include "platform_focal_ft5826s.h"

void *ft5826s_platform_data(void *info)
{
	static struct fts_ts_platform_data ft_ts_pdata;

	ft_ts_pdata.reset_gpio = get_gpio_by_name("TS_RST");
	ft_ts_pdata.irq_gpio = get_gpio_by_name("TS_INT");
	ft_ts_pdata.irqflags = IRQF_TRIGGER_FALLING;
	ft_ts_pdata.hard_rst_dly = 20;
	ft_ts_pdata.soft_rst_dly = 200;
	ft_ts_pdata.num_max_touches = 10;
	ft_ts_pdata.x_min = 0;
	ft_ts_pdata.x_max = 1280;
	ft_ts_pdata.y_min = 0;
	ft_ts_pdata.y_max = 800;

	return &ft_ts_pdata;
}
