/*
 * Support for Himax HM2051 8M camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/io.h>
#include <linux/time.h>/*Add by marky to check sensor id*/
#include "hm2051.h"

#include <linux/debugfs.h>
#include <linux/board_asustek.h>

/* ASUS dbgfs */
static unsigned int who_am_i = 2051;
static u16 g_by_file = 0x0;
static int exposure_return0;
static char camera_module_otp[60];
static unsigned int ATD_hm2051_status;
#define HM2051_RESOLUTION "2M"
#define HM2051_MODULE "HM2051"
#define HM2051_UID "000000000000000000000000"
#define HM2051_OTP "NO available OTP on HM2051"

static int g_hm2051_facing = HM2051_FACING_UNKNOWN;




/* i2c read/write stuff */
static int hm2051_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != HM2051_8BIT && data_length != HM2051_16BIT
					&& data_length != HM2051_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	/*pr_info("    addr=0x%x\n",msg[1].addr);*/

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == HM2051_8BIT)
		*val = (u8)data[0];
	else if (data_length == HM2051_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int hm2051_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int hm2051_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != HM2051_8BIT && data_length != HM2051_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == HM2051_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* HM2051_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = hm2051_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * hm2051_write_reg_array - Initializes a list of HM2051 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __hm2051_flush_reg_array, __hm2051_buf_reg_array() and
 * __hm2051_write_reg_is_consecutive() are internal functions to
 * hm2051_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __hm2051_flush_reg_array(struct i2c_client *client,
				    struct hm2051_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return hm2051_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __hm2051_buf_reg_array(struct i2c_client *client,
				  struct hm2051_write_ctrl *ctrl,
				  const struct hm2051_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case HM2051_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case HM2051_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= HM2051_MAX_WRITE_BUF_SIZE)
		return __hm2051_flush_reg_array(client, ctrl);

	return 0;
}

static int __hm2051_write_reg_is_consecutive(struct i2c_client *client,
					     struct hm2051_write_ctrl *ctrl,
					     const struct hm2051_reg *next)
{

	return 0; /*brad cancel */


	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

static int hm2051_write_reg_array(struct i2c_client *client,
				  const struct hm2051_reg *reglist)
{
	const struct hm2051_reg *next = reglist;
	struct hm2051_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != HM2051_TOK_TERM; next++) {
		switch (next->type & HM2051_TOK_MASK) {
		case HM2051_TOK_DELAY:
			err = __hm2051_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__hm2051_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __hm2051_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			}
			err = __hm2051_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev, "%s: write reg[0x%X, 0x%X] error, aborted\n", __func__, next->reg, next->val);
				return err;
			}
			break;
		}
	}

	return __hm2051_flush_reg_array(client, &ctrl);
}
static int hm2051_g_focal(struct v4l2_subdev *sd, s32 *val)
{
/*chris TODO*/
	pr_info("%s in\n", __func__);
	*val = (HM2051_FOCAL_LENGTH_NUM << 16) | HM2051_FOCAL_LENGTH_DEM;
	return 0;
}

static int hm2051_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
/*chris TODO*/
/*const f number for imx*/
/*pr_info("marky: %s in\n", __func__); */
	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 16) | HM2051_F_NUMBER_DEM;
	return 0;
}

static int hm2051_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
/*chris TODO*/
	pr_info(" %s in\n", __func__);
	*val = (HM2051_F_NUMBER_DEFAULT_NUM << 24) |
		(HM2051_F_NUMBER_DEM << 16) |
		(HM2051_F_NUMBER_DEFAULT_NUM << 8) | HM2051_F_NUMBER_DEM;
	return 0;
}

static int hm2051_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	pr_info(" %s in\n", __func__);
	*val = hm2051_res[dev->fmt_idx].bin_factor_x - 1;

	return 0;
}

static int hm2051_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	pr_info(" %s in\n", __func__);
	*val = hm2051_res[dev->fmt_idx].bin_factor_y - 1;

	return 0;
}

static int hm2051_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct hm2051_resolution *res)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct atomisp_sensor_mode_data *buf = &info->data;

	/*u16 coarse_integration_time_min;
	  u16 coarse_integration_time_max_margin;
	  u16 fine_integration_time_min;
	  u16 fine_integration_time_max_margin;*/

	pr_info(" %s in\n", __func__);
	if (info == NULL)
		return -EINVAL;

	/* pixel clock calculattion */
  /* TODO cal this
	ret =  hm2051_read_reg(client, HM2051_8BIT,
				HM2051_VT_SYS_CLK_DIV, &vt_sys_clk_div);
	if (ret)
		return ret;

	ret =  hm2051_read_reg(client, HM2051_8BIT, HM2051_PRE_PLL_CLK_DIV, &pre_pll_clk_div);
	if (ret)
		return ret;

	ret =  hm2051_read_reg(client, HM2051_8BIT, HM2051_PLL_MULTIPLIER, &pll_multiplier);

	if (ret)
		return ret;

  vt_pix_clk_freq_mhz = ext_clk_freq_hz/((pre_pll_clk_div & 0x1F)+ 1);
  pr_info("marky :%s: vt_pix_clk_freq_mhz=%d -> MClk=%d/DivClk=%d ",__func__, vt_pix_clk_freq_mhz, ext_clk_freq_hz, ((pre_pll_clk_div & 0x1F)+ 1));

  if ((pre_pll_clk_div & 0x20) == 1) {
       vt_pix_clk_freq_mhz *= (2* (pll_multiplier&0x7F) + 1);
       pr_info("marky : %s: vt_pix_clk_freq_mhz=%d  pll_multiplier = %d",__func__, vt_pix_clk_freq_mhz, (2* (pll_multiplier&0x7F) + 1));

  }

  if ((vt_sys_clk_div & 0x08) == 0) {
       vt_pix_clk_freq_mhz /= 2;
       pr_info("marky : %s CK2CFG[3] = 0 vt_pix_clk_freq_mhz /= 2",__func__);
  }

  pr_info("marky :%s: vt_pix_clk_freq_mhz=%d",__func__, vt_pix_clk_freq_mhz);
  dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
  */

	dev->vt_pix_clk_freq_mhz = res->pix_clk_freq; /*vt_pix_clk_freq_mhz;*/
	buf->vt_pix_clk_freq_mhz = res->pix_clk_freq; /*vt_pix_clk_freq_mhz;*/

	/* get integration time */

	buf->coarse_integration_time_min = HM2051_COARSE_INTG_TIME_MIN; /*coarse_integration_time_min;*/
	buf->coarse_integration_time_max_margin = HM2051_COARSE_INTG_TIME_MAX_MARGIN;/*coarse_integration_time_max_margin;*/

	buf->fine_integration_time_min = HM2051_FINE_INTG_TIME_MIN;/* HM2051_FINE_INTG_TIME_MIN;*/
	buf->fine_integration_time_max_margin = HM2051_FINE_INTG_TIME_MAX_MARGIN;/*HM2051_FINE_INTG_TIME_MAX_MARGIN;*/

	buf->fine_integration_time_def = HM2051_FINE_INTG_TIME_MIN;/* HM2051_FINE_INTG_TIME_MIN;*/


	pr_info("%s: coarse_integration_time_min %d coarse_integration_time_max_margin %d fine_integration_time_min %d. fine_integration_time_max_margin %d\n",
		__func__, buf->coarse_integration_time_min, buf->coarse_integration_time_max_margin, buf->fine_integration_time_min, buf->fine_integration_time_max_margin);

	buf->frame_length_lines = res->lines_per_frame;
	buf->line_length_pck = res->pixels_per_line;
	buf->read_mode = res->bin_mode;

	/* get the cropping and output resolution to ISP for this mode. */
	buf->output_width = res->width;
	buf->output_height = res->height;
	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = res->bin_factor_x;
	buf->binning_factor_y = res->bin_factor_y;
	buf->crop_horizontal_start = res->horizontal_start;
	buf->crop_horizontal_end = res->horizontal_end;
	buf->crop_vertical_start = res->vertical_start;
	buf->crop_vertical_end = res->vertical_end;

	/*pr_info("%s: crop_horizontal_start %d crop_vertical_start %d crop_horizontal_end %d crop_vertical_end %d\n",__func__, buf->crop_horizontal_start, buf->crop_vertical_start, buf->crop_horizontal_end, buf->crop_vertical_end);*/

	/*pr_info("%s: output_width %d output_height %d\n",__func__, buf->output_width, buf->output_height);*/

	pr_info(" %s out\n", __func__);
	return 0;
}

static long __hm2051_set_adgain(struct v4l2_subdev *sd, int gain, int digitgain)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	/*struct hm2051_device *dev = to_hm2051_sensor(sd);*/
	int ret = 0, iGainBase = 32, iMaxGainX100 = 6169;  /*max AgainX100 (3100) * max Dgain (iMaxDgainX100) */
	int iGainX100 = (gain*100)/iGainBase;
	int iAgainReg = 0 , iFineAgainReg = 0, iFineAgainBase = 16;
	int iAgainRegPowX100 = 100, iMaxAgainReg = 3, iMaxAgainRegPowX100 = 800; /*iMaxAgainReg and iMaxAgainRegPowX10   */
	int iDgainReg = 0;
	int iMaxDgainX100 = 398, iMinDgainX100 = 100, iDGainBase = 0x40;
	int iTemp;

	if (gain < 0)
		return -EINVAL;


	/* gain = Analog gain * Digitgain*/
	/* [8*(1+15/16)]*[(3.98)] = [31/2]*[3.98] */
	/* Analog gain */
	/* get the approach Analog gain base on gain */
	/* Adgain = 2^iAgainReg*(1+iFineAgainReg/iFineAgainBase)*/
	/* pr_info("marky: %s analog gain 0x%x, iGainX100 %d\n", __func__, gain, iGainX100);*/
	iGainX100 = iGainX100 > iMaxGainX100 ? iMaxGainX100 : (iGainX100 < 100  ? 100 : iGainX100);

	iTemp = iGainX100/200;
	while (iTemp > 0) {
		iAgainReg += 1;
		iTemp >>= 1;
		iAgainRegPowX100 *= 2;
	}

  /*
	if (iAgainReg > iMaxAgainReg) {
		pr_info("marky: %s cal coarse gain error -> gain=%d, iAgainReg=%d, iAgainRegPowX100=%d\n", __func__, gain, iAgainReg, iAgainRegPowX100);
	}
  */

	iAgainReg = iAgainReg > iMaxAgainReg ? iMaxAgainReg : iAgainReg;
	iAgainRegPowX100 =  iAgainRegPowX100 > iMaxAgainRegPowX100 ? iMaxAgainRegPowX100 : iAgainRegPowX100;
	iTemp = (iFineAgainBase * iGainX100)/iAgainRegPowX100 - iFineAgainBase;

  /*
	if (iTemp  < 0 || iTemp >= iFineAgainBase) {
		pr_info("marky: %s cal final gain error -> gain=%d, iAgainReg=%d, iFineAgainReg=%d, iAgainRegPowX100=%d\n", __func__, gain, iAgainReg, iTemp, iAgainRegPowX100);
	}
  */

	iFineAgainReg = iTemp  < 0 ? 0 : (iTemp >= iFineAgainBase ? iFineAgainBase - 1 : iTemp);
	gain = (iAgainReg) + (iFineAgainReg<<4);
	/*pr_info("marky: set Again -> %s Again=0x%X, iAgainReg=%d, iFineAgainReg=%d\n", __func__, gain, iAgainReg, iFineAgainReg);*/


	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_AGAIN, gain); /*gain & 0xff*/
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_AGAIN);
		return ret;
	}

	/* Digitgain*/
	/*Digitgain = gain / Analog gain*/
	/*Digitgain = iDgainReg*(1+iFineDGainReg/iFineDGainBase)*/
	iGainX100 = (iGainX100*100) / (iAgainRegPowX100 + (iAgainRegPowX100 * iFineAgainReg) / iFineAgainBase);
	/*pr_info("marky: %s Cal Dgain : iAgainReg=0x%X, iFineAgainReg=%d, DgainX100: %d\n", __func__,  iAgainReg, iFineAgainReg, iGainX100); */

	/*
	if ( iGainX100 < iMinDgainX100 || iGainX100 > iMaxDgainX100)
		pr_info("marky: %s Dgain Error 0x%x, iGainX100 %d\n", __func__, gain, iGainX100);
	*/

	iGainX100 = iGainX100 < iMinDgainX100 ? iMinDgainX100 : (iGainX100 > iMaxDgainX100 ? iMaxDgainX100 : iGainX100);

	/*Digital gain = DGAIN[7:0] / 0x40*/
	iDgainReg = (iGainX100 * iDGainBase)/100;
	gain = iDgainReg = (iDgainReg < iDGainBase ? iDGainBase : iDgainReg);
	/*pr_info("marky:  %s set Dgain -> iGainX100=%d,  iDGainBase=0x%X,  iDgainReg=%0xX\n", __func__, iGainX100, iDGainBase, iDgainReg); */

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_DGAIN, gain);

	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_DGAIN);
		return ret;
	}

	return ret;
}

static long __hm2051_set_exposure(struct v4l2_subdev *sd, int coarse_itg,
				 int gain, int digitgain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	u16 vts, hts;
	int ret = 0;

	hts = hm2051_res[dev->fmt_idx].pixels_per_line;
	vts = hm2051_res[dev->fmt_idx].lines_per_frame;


	pr_info(" %s in\n", __func__);
	/*return ret; //dor debug */
	/* Increase the VTS to match exposure + MARGIN */
	/* data blanking*/
	if (vts < coarse_itg)
		vts = (u16) coarse_itg + HM2051_INTEGRATION_TIME_MARGIN;

	/*pr_info("marky: %s vts = %d , lines_per_frame = %d , coarse_itg = %d\n", __func__, vts , hm2051_res[dev->fmt_idx].lines_per_frame, coarse_itg); */

	#ifdef FIXED_FRAME_RATE
	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_TIMING_VTS_H, ((vts - hm2051_res[dev->fmt_idx].height) & 0xFF00) >> 8);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_TIMING_VTS_H);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT, HM2051_TIMING_VTS_L, (vts - hm2051_res[dev->fmt_idx].height) & 0x00FF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_TIMING_VTS_L);
		return ret;
	}
  #endif
	/* set exposure */

	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_H, (coarse_itg >> 8) & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_H);
		return ret;
	}

	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_EXPOSURE_L, coarse_itg & 0xFF);
	if (ret) {
		dev_err(&client->dev, "%s: write %x error, aborted\n",
			__func__, HM2051_EXPOSURE_L);
		return ret;
	}

	/* set AD gain */
	ret = __hm2051_set_adgain(sd, gain, digitgain);
	if (ret) {
		dev_err(&client->dev, "%s: Set adgain error, aborted\n",
			__func__);
		return ret;
	}


	ret = hm2051_write_reg(client, HM2051_8BIT,
			       HM2051_COMMAND_UPDATE, 1);
	if (ret) {
		dev_err(&client->dev, "%s: Write HM2051_COMMAND_UPDATE %x error, aborted\n",
			__func__, HM2051_COMMAND_UPDATE);
		return ret;
	}
	pr_info(" %s out\n", __func__);
	return ret;

}

static int hm2051_set_exposure(struct v4l2_subdev *sd, int exposure,
	int gain, int digitgain)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if (exposure_return0)
		return 0;
	mutex_lock(&dev->input_lock);
	pr_info(" %s in\n", __func__);
	ret = __hm2051_set_exposure(sd, exposure, gain, digitgain);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long hm2051_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg = exposure->integration_time[0];
	u16 analog_gain = exposure->gain[0];
	u16 digital_gain = exposure->gain[1];
	pr_info(" %s in\n", __func__);
#if 0
	/* we should not accept the invalid value below */
	if (analog_gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}
#endif
	return hm2051_set_exposure(sd, coarse_itg, analog_gain, digital_gain);
}

static long hm2051_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
  /*pr_info("marky: %s in\n", __func__);*/
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return hm2051_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int hm2051_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_v, reg_v2;
	int ret;
  /*pr_info("marky: %s in\n", __func__);*/
	/* get exposure */
	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_EXPOSURE_L,
					&reg_v);
	if (ret)
		goto err;

	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_EXPOSURE_H,
					&reg_v2);
	if (ret)
		goto err;

	/*reg_v += reg_v2 << 8;*/

	*value = reg_v + ((u32)reg_v2 << 8);
err:
	return ret;
}

struct hm2051_control hm2051_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = hm2051_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = HM2051_FOCAL_LENGTH_DEFAULT,
			.maximum = HM2051_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = HM2051_F_NUMBER_DEFAULT,
			.maximum = HM2051_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = hm2051_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = HM2051_F_NUMBER_RANGE,
			.maximum =  HM2051_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = HM2051_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = hm2051_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = HM2051_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = HM2051_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2051_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(hm2051_controls))

static struct hm2051_control *hm2051_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (hm2051_controls[i].qc.id == id)
			return &hm2051_controls[i];
	return NULL;
}

static int hm2051_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct hm2051_control *ctrl = hm2051_find_control(qc->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* imx control set/get */
static int hm2051_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *s_ctrl;
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = hm2051_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2051_control *octrl = hm2051_find_control(ctrl->id);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2051_init(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	pr_info("%s\n", __func__);
	mutex_lock(&dev->input_lock);

	/* restore settings */
	hm2051_res = hm2051_res_preview;
	N_RES = N_RES_PREVIEW;

	/*Add by marky*/
	ret = hm2051_write_reg_array(client, hm2051_global_setting);
	if (ret)
		dev_err(&client->dev, "hm5040 write register err.\n");


	mutex_unlock(&dev->input_lock);

	return ret;
}


static int power_up(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}

	pr_info("%s\n", __func__);
	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* according to DS, at least 5ms is needed between DOVDD and PWDN */
	usleep_range(5000, 6000);

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 1);
		if (ret)
			goto fail_power;
	}

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* according to DS, 20ms is needed between PWDN and i2c access */
	msleep(20);

	return 0;

fail_clk:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;


	pr_info("%s\n", __func__);

	/*eturn 0; //Add by marky to debug*/
	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}


	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "gpio failed 2\n");
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int hm2051_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	pr_info(" %s -> %din\n", __func__, on);

	if (on == 0)
		return power_down(sd);
	else {
		ret = power_up(sd);


		if (!ret)
			return hm2051_init(sd);
	}
	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 800
static int distance(struct hm2051_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13)/w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -1;
	h_ratio = ((res->height << 13) / h);
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct hm2051_resolution *tmp_res = NULL;


	for (i = 0; i < N_RES; i++) {
		tmp_res = &hm2051_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}
	pr_info(" %s idx=%d, desc=%s min_dist=%d\n", __func__, i, hm2051_res[i].desc, min_dist);
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != hm2051_res[i].width)
			continue;
		if (h != hm2051_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int hm2051_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;
	idx = nearest_resolution_index(fmt->width,
					fmt->height);


	pr_info(" %s idx %d\n", __func__, idx);
	if (idx == -1) {
		/* return the largest resolution */
	idx = 0;
	}

	fmt->width = hm2051_res[idx].width;
	fmt->height = hm2051_res[idx].height;
	pr_info(" desc is  idx %d %s", idx, hm2051_res[idx].desc);
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10; /*mod by marky*/

	return 0;
}

/* TODO: remove it. */
static int startup(struct v4l2_subdev *sd)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	pr_info(" %s in\n", __func__);
  /*
	ret = hm2051_write_reg(client, HM2051_8BIT,
					HM2051_SW_RESET, 0x00);
	if (ret) {
		dev_err(&client->dev, "hm2051 reset err.\n");
		return ret;
	}

  pr_info("marky: global_setting\n");
	ret = hm2051_write_reg_array(client, hm2051_global_setting);
	if (ret) {
		dev_err(&client->dev, "hm2051 write register err(global).\n");
		return ret;
	}
  */
	pr_info(" local_setting\n");
	ret = hm2051_write_reg_array(client, hm2051_res[dev->fmt_idx].regs);
	if (ret) {
		dev_err(&client->dev, "hm2051 write register err(fmt_idx).\n");
		return ret;
	}

	return ret;
}

static int hm2051_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *hm2051_info = NULL;
	int ret = 0;

	hm2051_info = v4l2_get_subdev_hostdata(sd);
	if (hm2051_info == NULL)
		return -EINVAL;

	pr_info("%s\n", __func__);
	mutex_lock(&dev->input_lock);
	ret = hm2051_try_mbus_fmt(sd, fmt);
	if (ret == -1) {
		dev_err(&client->dev, "try fmt fail\n");
		goto err;
	}

	dev->fmt_idx = get_resolution_index(fmt->width,
					      fmt->height);
	if (dev->fmt_idx == -1) {
		dev_err(&client->dev, "get resolution fail\n");
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	ret = startup(sd);
	if (ret)
		dev_err(&client->dev, "hm2051 startup err\n");

	ret = hm2051_get_intg_factor(client, hm2051_info,
		&hm2051_res[dev->fmt_idx]);
	if (ret) {
		dev_err(&client->dev, "failed to get integration_factor\n");
		goto err;
	}

err:
	mutex_unlock(&dev->input_lock);
	return ret;
}
static int hm2051_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = hm2051_res[dev->fmt_idx].width;
	fmt->height = hm2051_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int hm2051_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;
	pr_info(" %s in\n", __func__);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

/*chris TODO*/
#if 1
	msleep(100);  /*Add by marky to check sensor ids*/
	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_SC_CMMN_CHIP_ID_H, &high);


	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}

	ret = hm2051_read_reg(client, HM2051_8BIT,
					HM2051_SC_CMMN_CHIP_ID_L, &low);
	id = ((((u16) high) << 8) | (u16) low);

	pr_info(" sensor id: 0x%x\n", id);
	if (id != HM2051_ID) {
		ATD_hm2051_status = 0;
		dev_err(&client->dev, "sensor ID error 0x%x\n", id);
		return -ENODEV;
	} else {
		pr_info("Main sensor ID = 0x%x\n", id);
		ATD_hm2051_status = 1;
	}
#endif

	dev_dbg(&client->dev, "detect hm2051 success\n");
	return 0;
}

static int hm2051_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	pr_info(" %s ->%d\n", __func__, enable);
	ret = hm2051_write_reg_array(client, enable ? hm2051_stream_on : hm2051_stream_off);
	pr_info("%s ret ->%d\n", __func__, ret);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/* hm2051 enum frame size, frame intervals */
static int hm2051_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = hm2051_res[index].width;
	fsize->discrete.height = hm2051_res[index].height;
	fsize->reserved[0] = hm2051_res[index].used;

	return 0;
}

static int hm2051_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = hm2051_res[index].width;
	fival->height = hm2051_res[index].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = hm2051_res[index].fps;

	return 0;
}

static int hm2051_enum_mbus_fmt(struct v4l2_subdev *sd,
				unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int hm2051_s_config(struct v4l2_subdev *sd,
			   int irq, void *platform_data)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	pr_info(" %s in\n", __func__);
	if (platform_data == NULL)
		return -ENODEV;

	dev->platform_data =
		(struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	/* power off the module, then power on it in future
	 * as first power on by board may not fulfill the
	 * power on sequqence needed by the module
	 */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-off err.\n");
		goto fail_power_off;
	}

	ret = power_up(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-up err.\n");
		goto fail_power_on;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = hm2051_detect(client);
	if (ret) {
		dev_err(&client->dev, "hm2051_detect err s_config.\n");
		goto fail_csi_cfg;
	}

	/* turn off sensor, after probed */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "hm2051 power-off err.\n");
		goto fail_csi_cfg;
	}
	mutex_unlock(&dev->input_lock);

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_power_on:
	power_down(sd);
	dev_err(&client->dev, "sensor power-gating failed\n");
fail_power_off:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2051_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	pr_info(" %s in\n", __func__);
	if (!param)
		return -EINVAL;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(&client->dev,  "unsupported buffer type.\n");
		return -EINVAL;
	}

	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (dev->fmt_idx >= 0 && dev->fmt_idx < N_RES) {
		param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		param->parm.capture.timeperframe.numerator = 1;
		param->parm.capture.capturemode = dev->run_mode;
		param->parm.capture.timeperframe.denominator =
			hm2051_res[dev->fmt_idx].fps;
	}
	return 0;
}

static int hm2051_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	pr_info(" %s, run_mode: 0x%X\n", __func__, dev->run_mode);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
	pr_info(" VIDEO!\n");
		hm2051_res = hm2051_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
	pr_info(" CAPTURE!\n");
		hm2051_res = hm2051_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
	pr_info(" PREVIEW!\n");
		hm2051_res = hm2051_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int hm2051_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);

	interval->interval.numerator = 1;
	interval->interval.denominator = hm2051_res[dev->fmt_idx].fps;

	return 0;
}

static int hm2051_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}

static int hm2051_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = hm2051_res[index].width;
	fse->min_height = hm2051_res[index].height;
	fse->max_width = hm2051_res[index].width;
	fse->max_height = hm2051_res[index].height;

	return 0;

}

static struct v4l2_mbus_framefmt *
__hm2051_get_pad_format(struct hm2051_device *sensor,
			struct v4l2_subdev_fh *fh, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,
			"__hm2051_get_pad_format err. pad %x\n", pad);
		return NULL;
	}

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int hm2051_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__hm2051_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int hm2051_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct hm2051_device *snr = to_hm2051_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int hm2051_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	*frames = hm2051_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops hm2051_sensor_ops = {
	.g_skip_frames = hm2051_g_skip_frames,
};

static const struct v4l2_subdev_video_ops hm2051_video_ops = {
	.s_stream = hm2051_s_stream,
	.g_parm = hm2051_g_parm,
	.s_parm = hm2051_s_parm,
	.enum_framesizes = hm2051_enum_framesizes,
	.enum_frameintervals = hm2051_enum_frameintervals,
	.enum_mbus_fmt = hm2051_enum_mbus_fmt,
	.try_mbus_fmt = hm2051_try_mbus_fmt,
	.g_mbus_fmt = hm2051_g_mbus_fmt,
	.s_mbus_fmt = hm2051_s_mbus_fmt,
	.g_frame_interval = hm2051_g_frame_interval,
};

static const struct v4l2_subdev_core_ops hm2051_core_ops = {
	.s_power = hm2051_s_power,
	.queryctrl = hm2051_queryctrl,
	.g_ctrl = hm2051_g_ctrl,
	.s_ctrl = hm2051_s_ctrl,
	.ioctl = hm2051_ioctl,
};

static const struct v4l2_subdev_pad_ops hm2051_pad_ops = {
	.enum_mbus_code = hm2051_enum_mbus_code,
	.enum_frame_size = hm2051_enum_frame_size,
	.get_fmt = hm2051_get_pad_format,
	.set_fmt = hm2051_set_pad_format,
};

static const struct v4l2_subdev_ops hm2051_ops = {
	.core = &hm2051_core_ops,
	.video = &hm2051_video_ops,
	.pad = &hm2051_pad_ops,
	.sensor = &hm2051_sensor_ops,
};

static int hm2051_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct hm2051_device *dev = to_hm2051_sensor(sd);
	pr_info(" %s in\n", __func__);
	dev_dbg(&client->dev, "hm2051_remove...\n");

	dev->platform_data->csi_cfg(sd, 0);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);

	return 0;
}

/*++++++++++++++++dbgfs+++++++++++++++++ */
static int dbg_hm2051_vga_status_open(
	struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_hm2051_vga_status_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;


	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", camera_module_otp);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_hm2051_vga_status_fops = {
	.open		= dbg_hm2051_vga_status_open,
	.read		= dbg_hm2051_vga_status_read,
};

static ssize_t dbg_dump_hm2051_otp_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", HM2051_OTP);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_hm2051_otp_fops = {
	.open		= dbg_hm2051_vga_status_open,
	.read		= dbg_dump_hm2051_otp_read,
};


static ssize_t dbg_dump_hm2051_res_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", HM2051_RESOLUTION);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_hm2051_res_fops = {
	.open		= dbg_hm2051_vga_status_open,
	.read		= dbg_dump_hm2051_res_read,
};

static ssize_t dbg_dump_hm2051_module_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", HM2051_MODULE);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_hm2051_module_fops = {
	.open		= dbg_hm2051_vga_status_open,
	.read		= dbg_dump_hm2051_module_read,
};

static ssize_t dbg_dump_hm2051_uid_read(
	struct file *file,
	char __user *buf,
	size_t count,
	loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	pr_info("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n",
		__func__, buf, (int)count, ppos, (int)*ppos);

	if (*ppos)
		return 0;	/* the end */

	len = snprintf(bp, dlen, "%s\n", HM2051_UID);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_hm2051_uid_fops = {
	.open		= dbg_hm2051_vga_status_open,
	.read		= dbg_dump_hm2051_uid_read,
};


static int hm2051_dbgfs_init(int proj_id)
{
	struct dentry *debugfs_dir;

	switch (proj_id) {

	case 0x8:
		if (g_hm2051_facing == HM2051_FACING_REAR) {
			debugfs_dir = debugfs_create_dir("camera0", NULL);
			debugfs_create_u32("camera_status",
				0644, debugfs_dir, &ATD_hm2051_status);
		} else {
			debugfs_dir = debugfs_create_dir("camera1", NULL);
			debugfs_create_u32("vga_status",
				0644, debugfs_dir, &ATD_hm2051_status);
			(void) debugfs_create_file("CameraOTP", S_IRUGO,
			debugfs_dir, NULL, &dbg_dump_hm2051_otp_fops);
		}
		break;
	default:
		pr_info("%s: Unknown project(0x%x). Early return.\n",
			__func__, proj_id);
		return 0;
	}


	(void) debugfs_create_file("CameraOTP", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_hm2051_otp_fops);
	(void) debugfs_create_file("i2c_write", S_IRUGO,
		debugfs_dir, NULL, &dbg_hm2051_vga_status_fops);
	(void) debugfs_create_file("CameraResolution", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_hm2051_res_fops);
	(void) debugfs_create_file("CameraModule", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_hm2051_module_fops);
	(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_hm2051_uid_fops);


	debugfs_create_x16("by_file", 0644, debugfs_dir, &g_by_file);
	debugfs_create_u32("sensor_id", 0644, debugfs_dir, &who_am_i);
	debugfs_create_u32("exposure_return0", 0644, debugfs_dir, &exposure_return0);

	return 0;
}
/*---------------------dbgfs----------------------- */

static int hm2051_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct hm2051_device *dev;
	int ret;
	int project_id = PROJECT_ID_INVALID;
	int cam_rear_id = CAM_REAR_INVALID;
	int cam_front_id = CAM_FRONT_INVALID;

	pr_info(" %s in\n", __func__);

	project_id = asustek_get_project_id();
	cam_rear_id = asustek_get_camera_rear();
	cam_front_id = asustek_get_camera_front();

	if (cam_rear_id == 1)
		g_hm2051_facing = HM2051_FACING_REAR;
	else if (cam_front_id == 1)
		g_hm2051_facing = HM2051_FACING_FRONT;
	else
		g_hm2051_facing = HM2051_FACING_UNKNOWN;
	pr_info("project_id= %d\n", project_id);
	pr_info("cam_rear_id= %d\n", cam_rear_id);
	pr_info("cam_front_id= %d\n", cam_front_id);
	switch (project_id) {

	case 0x8:
	{
		pr_info("%s(): g_hm2051_facing= %s\n", __func__,
			(g_hm2051_facing == HM2051_FACING_REAR) ? "REAR" :
			((g_hm2051_facing == HM2051_FACING_FRONT) ? "FRONT" :
				"UNKNOWN"));

		if (g_hm2051_facing == HM2051_FACING_UNKNOWN) {
			pr_info("hm2051 doesn't exist. Ignore %s()\n",
				__func__);

			return -EINVAL;
		}
	}
	break;

	default:
		pr_info("hm2051 is unsupported on this board(0x%x)\n",
			project_id);
		return -EINVAL;
	}


	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &hm2051_ops);

	if (client->dev.platform_data) {
		ret = hm2051_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret)
			goto out_free;
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		hm2051_remove(client);

	hm2051_dbgfs_init(project_id);

	return ret;
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

MODULE_DEVICE_TABLE(i2c, hm2051_id);
static struct i2c_driver hm2051_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = HM2051_NAME,
	},
	.probe = hm2051_probe,
	.remove = hm2051_remove,
	.id_table = hm2051_id,
};

static int init_hm2051(void)
{
	return i2c_add_driver(&hm2051_driver);
}

static void exit_hm2051(void)
{

	i2c_del_driver(&hm2051_driver);
}

module_init(init_hm2051);
module_exit(exit_hm2051);
MODULE_AUTHOR("Chris Kan <chris.kan@intel.com>");
MODULE_DESCRIPTION("A low-level driver for Himax hm2051 sensors");
MODULE_LICENSE("GPL");
