/*
 * platform_ov5693.c: ov5693 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include <linux/mfd/intel_mid_pmic.h>
#include "platform_camera.h"
#include "platform_ov5693.h"
#include <linux/lnw_gpio.h>

/* workround - pin defined for cht */
#define CAMERA_0_RESET	150
#define CAMERA_1P8_EN	153
#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x0
#define CLK_19P2MHz 0x1
#endif
static int camera_vprog1_on;
static int camera_reset;
static int camera_vcm_power_down;

/*
 * camera sensor - ov5693 platform data
 */

static int ov5693_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;

	pr_info("%s - E, flag: %d\n", __func__, flag);
	ret = camera_set_vprog_power(CAMERA_VPROG3, flag,
					CAMERA_2_9_VOLT);
	if (ret) {
		pr_err("%s power %s vprog3 failed\n", __func__,
				flag ? "on" : "off");
		return ret;
	}
	if (flag)
		usleep_range(1000, 1200);

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
				GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			if (flag)
				camera_set_vprog_power(CAMERA_VPROG3, !flag,
						CAMERA_2_9_VOLT);
			return ret;
		}
		camera_reset = ret;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(camera_reset, LNW_GPIO);
	}

	if (camera_vcm_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_2_POWER_DOWN,
				GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			if (flag)
				camera_set_vprog_power(CAMERA_VPROG3, !flag,
					CAMERA_2_9_VOLT);
			return ret;
		}
		camera_vcm_power_down = ret;
		/* set camera vcm power pin mode to gpio */
		lnw_gpio_set_alt(camera_vcm_power_down, LNW_GPIO);
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		gpio_set_value(camera_vcm_power_down, 1);

	} else {
		gpio_set_value(camera_reset, 0);
		gpio_free(camera_reset);
		camera_reset = -1;

		gpio_set_value(camera_vcm_power_down, 0);
		gpio_free(camera_vcm_power_down);
		camera_vcm_power_down = -1;
	}
	return 0;
}

static int ov5693_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;

	pr_info("%s - E, flag: %d\n", __func__, flag);

	if (intel_mid_identify_cpu() != INTEL_MID_CPU_CHIP_VALLEYVIEW2)
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
					     flag ? clock_khz : 0);
	return ret;
}

/*
 * The power_down gpio pin is to control OV5693's
 * internal power state.
 */

static int ov5693_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
		/* VPROG2_1V8 on */
		ret = camera_set_vprog_power(CAMERA_VPROG2, 1,
					DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog2 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}
		usleep_range(1000, 1200);
		msleep(15);

		/* VPROG1_2V8 on */
		ret = camera_set_vprog_power(CAMERA_VPROG1, 1,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog1 failed\n", __func__,
					flag ? "on" : "off");

			camera_set_vprog_power(CAMERA_VPROG2, 0,
			DEFAULT_VOLTAGE);
		}
		usleep_range(1000, 1200);
	} else {
		/* VPROG1_2V8 off */
		ret = camera_set_vprog_power(CAMERA_VPROG1, 0,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}

		msleep(15);

		/* VPROG2_1V8 off */
		ret = camera_set_vprog_power(CAMERA_VPROG2, 0,
					DEFAULT_VOLTAGE);
		if (ret)
			pr_err("%s power %s vprog2 failed\n", __func__,
					flag ? "on" : "off");
	}

	pr_info("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int ov5693_csi_configure(struct v4l2_subdev *sd, int flag)
{
	pr_info("%s - E, flag: %d\n", __func__, flag);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov5693_platform_init(struct i2c_client *client)
{
	pr_info("%s()\n", __func__);

	return 0;
}

static int ov5693_platform_deinit(void)
{

	pr_info("%s()\n", __func__);

	return 0;
}

static struct camera_sensor_platform_data ov5693_sensor_platform_data = {
	.gpio_ctrl	= ov5693_gpio_ctrl,
	.flisclk_ctrl	= ov5693_flisclk_ctrl,
	.power_ctrl	= ov5693_power_ctrl,
	.csi_cfg	= ov5693_csi_configure,
	.platform_init = ov5693_platform_init,
	.platform_deinit = ov5693_platform_deinit,
};

void *ov5693_platform_data(void *info)
{

	pr_info("%s()\n", __func__);

	camera_reset = -1;
	camera_vcm_power_down = -1;

	return &ov5693_sensor_platform_data;
}
