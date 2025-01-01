/*
 * platform_ov5670.c: ov5670 platform data initilization file
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
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_ov2680.h"
#include <linux/lnw_gpio.h>
#include <linux/regulator/consumer.h>

#define VPROG1_VAL 2800000
#define VPROG2_VAL 1800000

static int sub_cam_pd;
static int camera_vprog1_on;
static int camera_vprog2_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

static int ov2680_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;/*Intel just can support 19.2MHz/9.6MHz/4.8MHz*/
	int ret = 0;
	v4l2_err(sd, "%s: ++\n", __func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
	return ret;
}


static int ov2680_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	pr_debug("%s: ++\n", __func__);


	if (sub_cam_pd < 0) {
		ret = camera_sensor_gpio(-1, GP_SUB_CAM_PD, GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_alert("sub_cam_pd not available.\n");
			return ret;
		}
		sub_cam_pd = ret;
	}

	if (flag) {

		gpio_set_value(sub_cam_pd, 1);
		pr_debug("<<< sub_cam_pd = 1\n");

		usleep_range(6000, 6500);

	} else {

		gpio_set_value(sub_cam_pd, 0);
		gpio_free(sub_cam_pd);
		sub_cam_pd = -1;

		udelay(1);

	}
	return 0;
}


static int ov2680_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret;

	pr_debug("%s: ++\n", __func__);


	if (flag) {

		/*turn on power 1.8V and 2.8V*/

		pr_info("%s(%d): 1V8 on\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG2, 1,
				DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog2 failed\n", __func__,
						flag ? "on" : "off");
			return ret;
		}
		usleep_range(1000, 1200);

		pr_info("%s(%d): 2V8 on\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG1, 1,
					DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power-up %s  vprog1 failed\n", __func__,
					flag ? "on" : "off");

			camera_set_vprog_power(CAMERA_VPROG2, 0,
					DEFAULT_VOLTAGE);
		}


		usleep_range(2000, 2500);/*wait vprog1 and vprog2 from enable to 90% (max:2000us)*/

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
		usleep_range(1000, 1500);

		/* VPROG2_1V8 off */
		pr_info("%s(%d): 1V8 off\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG2, 0,
						DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("%s power %s vprog2 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}
		usleep_range(1000, 1500);

	}

	return 0;
}

static int ov2680_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov2680_platform_init(struct i2c_client *client)
{
	int ret;

	pr_debug("%s: ++\n", __func__);

	/*VPROG1 for  2.8V*/
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 set failed\n");
		regulator_put(vprog1_reg);
	}

	/*VPROG2 for 1.8V*/
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "vprog2 failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog2 set failed\n");
		regulator_put(vprog2_reg);
	}

	return ret;
}

static int ov2680_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
}

static struct camera_sensor_platform_data ov2680_sensor_platform_data = {
	.gpio_ctrl	 = ov2680_gpio_ctrl,
	.flisclk_ctrl	 = ov2680_flisclk_ctrl,
	.power_ctrl	 = ov2680_power_ctrl,
	.csi_cfg	 = ov2680_csi_configure,
	.platform_init   = ov2680_platform_init,
	.platform_deinit = ov2680_platform_deinit,
};

void *ov2680_platform_data(void *info)
{
	sub_cam_pd = -1;

	return &ov2680_sensor_platform_data;
}

