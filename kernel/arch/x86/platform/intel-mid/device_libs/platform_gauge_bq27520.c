/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by Piter Hsu piter_hsu@asus.com
 */

#include <linux/power/gauge_bq27520.h>
#include "platform_gauge_bq27520.h"
#include <asm/intel-mid.h>

static struct gauge_bq27520_platform_data_struct bq27520_platform_data = {
	.energy_full_design = -1,
	.voltage_max_design = -1,
	.voltage_min_design = -1,
	.ADC_alert_pin = -1
};

void *gauge_bq27520_get_platform_data(void *info)
{
	bq27520_platform_data.energy_full_design = 31;
	bq27520_platform_data.voltage_max_design = 4350;
	bq27520_platform_data.voltage_min_design = 3400;
	bq27520_platform_data.ADC_alert_pin = get_gpio_by_name("ADC_Alert");

	return &bq27520_platform_data;
}

