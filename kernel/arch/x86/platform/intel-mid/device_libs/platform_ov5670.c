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
#include "platform_ov5670.h"
#include <linux/lnw_gpio.h>
#include <linux/regulator/consumer.h>

#define VPROG1_VAL 2800000
#define VPROG2_VAL 1800000
#define VEMMC1_VAL 2800000

static int camera_1p2_ldo;
static int camera_reset;
static int camera_vcm_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
static struct regulator *vemmc1_reg;

static int ov5670_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;/*Intel just can support 19.2MHz/9.6MHz/4.8MHz*/
	int ret = 0;
	v4l2_err(sd, "%s: ++\n", __func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
	return ret;
}


static int ov5670_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	pr_debug("%s: ++\n", __func__);

	if (camera_1p2_ldo < 0) {
		ret = camera_sensor_gpio(-1, GP_CAM_0_AF_EN, GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_alert("camera_1p2_ldo not available.\n");
			return ret;
		}
		camera_1p2_ldo = ret;
	}

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET, GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			pr_alert("camera_reset not available.\n");
			return ret;
		}
		camera_reset = ret;
	}

	if (camera_vcm_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_2_POWER_DOWN,
				GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			pr_alert("vcm_power_down not available.\n");
			return ret;
		}
		camera_vcm_power_down = ret;
		/* set camera vcm power pin mode to gpio */
		lnw_gpio_set_alt(camera_vcm_power_down, LNW_GPIO);
	}

	if (flag) {
		gpio_set_value(camera_1p2_ldo, 1);
		pr_debug("<<< GP_CAM_0_AF_EN = 1\n");

		usleep_range(1000, 1500);

		gpio_set_value(camera_reset, 1);
		pr_debug("<<< GP_CAMERA_0_RESET = 1\n");

		usleep_range(1000, 1500);

		gpio_set_value(camera_vcm_power_down, 1);
		pr_debug("<<< GP_CAMERA_2_POWER_DOWN = 1\n");

		usleep_range(6000, 6500);

	} else {
		gpio_set_value(camera_vcm_power_down, 0);
		gpio_free(camera_vcm_power_down);
		camera_vcm_power_down = -1;

		usleep_range(1000, 1500);

		gpio_set_value(camera_reset, 0);
		gpio_free(camera_reset);
		camera_reset = -1;

		usleep_range(1000, 1500);

		gpio_set_value(camera_1p2_ldo, 0);
		gpio_free(camera_1p2_ldo);
		camera_1p2_ldo = -1;

		udelay(1);

	}
	return 0;
}


static int ov5670_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret;

	pr_debug("%s: ++\n", __func__);


	if (flag) {

#if 0
		/*turn on VCM power 2.85V*/
		if (!camera_vemmc1_on) {
			camera_vemmc1_on = 1;
			reg_err = regulator_enable(vemmc1_reg);
			if (reg_err) {
				pr_alert("Failed to enable regulator vemmc1\n");
				return reg_err;
			}
			pr_debug("<<< VCM 2.85V = 1\n");
			usleep_range(1000, 1500);
		}
#endif
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

		usleep_range(1000, 1200);

		pr_info("%s(%d): vcm power on\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG3, 1,
					CAMERA_2_9_VOLT);
		if (ret) {
			pr_err("%s power-up %s	vprog3 failed\n", __func__,
					flag ? "on" : "off");

			camera_set_vprog_power(CAMERA_VPROG3, 0,
					CAMERA_2_9_VOLT);
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

		/* VPROG3_VCM off */
		pr_info("%s(%d): VCM power off\n", __func__, __LINE__);
		ret = camera_set_vprog_power(CAMERA_VPROG3, 0,
						CAMERA_2_9_VOLT);
		if (ret) {
			pr_err("%s power %s vprog3 failed\n", __func__,
					flag ? "on" : "off");
			return ret;
		}
		usleep_range(1000, 1500);

	}

	return 0;
}

static int ov5670_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
			ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov5670_platform_init(struct i2c_client *client)
{
	int ret;

	pr_debug("%s: ++\n", __func__);

	/*VPROG1 for 1.8V and 2.8V*/
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

	/*VPROG2 for 1.2V*/
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
#if 0
	/*VEMMC1 for VCM, 2.85V*/
	vemmc1_reg = regulator_get(&client->dev, "vemmc1");
	if (IS_ERR(vemmc1_reg)) {
		dev_err(&client->dev, "vemmc1 failed\n");
		return PTR_ERR(vemmc1_reg);
	}
#endif
	return ret;
}

static int ov5670_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
	regulator_put(vemmc1_reg);
}

static struct camera_sensor_platform_data ov5670_sensor_platform_data = {
	.gpio_ctrl	 = ov5670_gpio_ctrl,
	.flisclk_ctrl	 = ov5670_flisclk_ctrl,
	.power_ctrl	 = ov5670_power_ctrl,
	.csi_cfg	 = ov5670_csi_configure,
	.platform_init   = ov5670_platform_init,
	.platform_deinit = ov5670_platform_deinit,
};

void *ov5670_platform_data(void *info)
{
	camera_reset = -1;
	camera_vcm_power_down = -1;
	camera_1p2_ldo = -1;

	return &ov5670_sensor_platform_data;
}

