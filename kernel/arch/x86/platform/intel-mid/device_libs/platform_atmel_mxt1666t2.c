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
#include <linux/i2c/atmel_mxt1666t2.h>
#include "platform_atmel_mxt1666t2.h"

void *mxt1666t2_platform_data(void *info)
{
	static struct mxt_platform_data mxt_pdata;

	mxt_pdata.gpio_reset = get_gpio_by_name("TS_RST");
	mxt_pdata.gpio_int = get_gpio_by_name("TS_INT");
	mxt_pdata.irqflags = IRQF_TRIGGER_FALLING;
	mxt_pdata.input_name = "atmel_mxt_ts_T100_touchscreen";

	return &mxt_pdata;
}
