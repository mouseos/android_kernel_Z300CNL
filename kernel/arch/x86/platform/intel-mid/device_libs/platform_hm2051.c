/*
 * platform_hm2051.c: hm2051 platform data initilization file
 *
 * (C) Copyright 2014 Asus Corporation
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
#include "platform_hm2051.h"
#include <linux/lnw_gpio.h>
#include <linux/board_asustek.h>


static int camera_reset;

static int sub_cam_pd;

/*
 * camera sensor - hm2051 platform data
 */

static int hm2051_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;


	pr_info("%s - E, flag: %d\n", __func__, flag);
	pr_info("g_rear_cam_id =%d\n", g_rear_cam_id);
	pr_info("g_front_cam_id =%d\n", g_front_cam_id);


	if (sub_cam_pd < 0) {
		switch (project_id_for_cam) {
		case 0x8:
			if (g_rear_cam_id == 1)
				ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
							GPIOF_DIR_OUT, 0);
			else
				ret = camera_sensor_gpio(-1, GP_SUB_CAM_PD,
							GPIOF_DIR_OUT, 1);
			break;

		default:
			ret = camera_sensor_gpio(-1, GP_SUB_CAM_PD,
						GPIOF_DIR_OUT, 0);
		}

		if (ret < 0) {
			pr_info("%s(%d): camera_sensor_gpio fail\n", __func__, __LINE__);
			return ret;
		}
		sub_cam_pd = ret;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(sub_cam_pd, LNW_GPIO);
	}

	/* only front camera */
	if (g_front_cam_id == 1) {
		if (camera_reset < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			camera_reset = ret;
			lnw_gpio_set_alt(camera_reset, LNW_GPIO);
		}
	}

	if (flag) {
		gpio_set_value(sub_cam_pd, 1);
		pr_info("%s...gpio number:%d on++\n", __func__, sub_cam_pd);
		if (g_front_cam_id == 1) {
			gpio_set_value(camera_reset, 1);
			pr_info("%s...gpio number:%d on++\n", __func__, camera_reset);
		}
	} else {
		if (g_front_cam_id == 1) {
			gpio_set_value(camera_reset, 0);
			gpio_free(camera_reset);
			camera_reset = -1;
			pr_info("%s...gpio number:%d on++\n", __func__, camera_reset);
		}
		gpio_set_value(sub_cam_pd, 0);
		gpio_free(sub_cam_pd);
		sub_cam_pd = -1;
		pr_info("%s...gpio number:%d off--\n", __func__, sub_cam_pd);
	}
	return 0;
}

static int hm2051_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;

	pr_info("%s - E, flag: %d\n", __func__, flag);
	pr_info("g_rear_cam_id =%d\n", g_rear_cam_id);
	pr_info("g_front_cam_id =%d\n", g_front_cam_id);
	switch (project_id_for_cam) {
	case 0x8:
		if (g_rear_cam_id == 1) {
			pr_info("%s: CLK_CAM0\n", __func__);
			return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
						flag ? clock_khz : 0);
		} else {
			pr_info("%s: CLK_CAM1\n", __func__);
			return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
						flag ? clock_khz : 0);
		}
	default:
		pr_info("%s: CLK_CAM1\n", __func__);
		return intel_scu_ipc_osc_clk(OSC_CLK_CAM1,
					flag ? clock_khz : 0);
	}

}

/*
 * The power_down gpio pin is to control hm2051's
 * internal power state.
 */

static int hm2051_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
		/* VPROG2_1V8 on */
		pr_info("%s(%d): 1V8 on\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG2, 1,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog2 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}
		usleep_range(100, 120);

		/* VPROG1_2V8 on */
		pr_info("%s(%d): 2V8 on\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG1, 1,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power-up %s vprog1 failed\n", __func__,
					flag ? "on" : "off");
			camera_set_vprog_power(CAMERA_VPROG2, 0,
					DEFAULT_VOLTAGE);
		}
		usleep_range(100, 120);
	} else {
		/* VPROG1_2V8 off */
		pr_info("%s(%d): 2V8 off\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG1, 0,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power-down %s vprog1 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}

		usleep_range(100, 120);

		/* VPROG2_1V8 off */
		pr_info("%s(%d): 1V8 off\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG2, 0,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog2 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}
	}
	pr_info("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int hm2051_csi_configure(struct v4l2_subdev *sd, int flag)
{

	pr_info("%s(), flag: %d\n", __func__, flag);
	pr_info("g_rear_cam_id =%d\n", g_rear_cam_id);
	pr_info("g_front_cam_id =%d\n", g_front_cam_id);

	switch (project_id_for_cam) {
	case 0x8:
	if (g_rear_cam_id == 1) {
		pr_info("%s: port: PRIMARY; MIPI lane num: 1\n", __func__);
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 1,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr,
			flag);
	} else {
		pr_info("%s: port: SECONDARY; MIPI lane num: 1\n", __func__);
		return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
	}

	default:
	pr_info("%s: port: SECONDARY; MIPI lane num: 1\n", __func__);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
	}

}

static int hm2051_platform_init(struct i2c_client *client)
{
	int ret;

	/*check have camera module and sku is vaild*/
	if ((g_rear_cam_id == 0 && g_front_cam_id == 0) || (g_rear_cam_id == 1 && g_front_cam_id == 1))
		return 0;

	pr_info("%s()\n", __func__);
	pr_info("g_rear_cam_id =%d\n", g_rear_cam_id);
	pr_info("g_front_cam_id =%d\n", g_front_cam_id);

	if (sub_cam_pd < 0) {

		switch (project_id_for_cam) {
		case 0x8:
			if (g_rear_cam_id == 1) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
						GPIOF_DIR_OUT, 1);
			} else {
				ret = camera_sensor_gpio(-1, GP_SUB_CAM_PD,
							GPIOF_DIR_OUT, 1);
			}
			break;

		default:
			ret = camera_sensor_gpio(-1, GP_SUB_CAM_PD,
						GPIOF_DIR_OUT, 1);
		}

		if (ret < 0) {
			pr_info("%s(%d): camera_sensor_gpio fail\n", __func__, __LINE__);
			return ret;
		}
		sub_cam_pd = ret;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(sub_cam_pd, LNW_GPIO);
	}

	gpio_set_value(sub_cam_pd, 0);
	gpio_free(sub_cam_pd);
	sub_cam_pd = -1;

	return 0;
}

static int hm2051_platform_deinit(void)
{
	pr_info("%s()\n", __func__);

	return 0;
}

static struct camera_sensor_platform_data hm2051_sensor_platform_data = {
	.gpio_ctrl	= hm2051_gpio_ctrl,
	.flisclk_ctrl	= hm2051_flisclk_ctrl,
	.power_ctrl	= hm2051_power_ctrl,
	.csi_cfg	= hm2051_csi_configure,
	.platform_init = hm2051_platform_init,
	.platform_deinit = hm2051_platform_deinit,
};

void *hm2051_platform_data(void *info)
{

	pr_info("%s()\n", __func__);

	camera_reset = -1;
	sub_cam_pd = -1;

	return &hm2051_sensor_platform_data;
}
