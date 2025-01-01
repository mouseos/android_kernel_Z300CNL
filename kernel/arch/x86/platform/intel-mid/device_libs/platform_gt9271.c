/*
 * platform_gt9271.c: Novatek touch platform data initilization file
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
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/gt9xx.h>
#include "platform_gt9271.h"

void *gt9271_platform_data(void *info)
{
	static struct goodix_ts_data gt_pdata;

	return &gt_pdata;
}