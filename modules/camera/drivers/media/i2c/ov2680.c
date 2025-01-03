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
#include <linux/acpi.h>
#include <linux/io.h>

#include <linux/debugfs.h>
#include <linux/board_asustek.h>

#include "ov2680.h"

/* ASUS dbgfs */
static unsigned int who_am_i = 2680;
static u16 g_by_file = 0x0;
static int exposure_return0;
static char camera_module_otp[60];
static unsigned int ATD_ov2680_status;
#define OV2680_RESOLUTION "2M"
#define OV2680_MODULE "OV2680"
#define OV2680_UID "000000000000000000000000"
#define OV2680_OTP "NO available OTP on OV2680"



/* i2c read/write stuff */
static int ov2680_read_reg(struct i2c_client *client,
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

	if (data_length != OV2680_8BIT && data_length != OV2680_16BIT
					&& data_length != OV2680_32BIT) {
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
	if (data_length == OV2680_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV2680_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int ov2680_i2c_write(struct i2c_client *client, u16 len, u8 *data)
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

static int ov2680_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV2680_8BIT && data_length != OV2680_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV2680_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* ov2680_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov2680_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * ov2680_write_reg_array - Initializes a list of ov2680 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ov2680_flush_reg_array, __ov2680_buf_reg_array() and
 * __ov2680_write_reg_is_consecutive() are internal functions to
 * ov2680_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ov2680_flush_reg_array(struct i2c_client *client,
				    struct ov2680_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov2680_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ov2680_buf_reg_array(struct i2c_client *client,
				  struct ov2680_write_ctrl *ctrl,
				  const struct ov2680_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case OV2680_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case OV2680_16BIT:
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
	if (ctrl->index + sizeof(u16) >= OV2680_MAX_WRITE_BUF_SIZE)
		return __ov2680_flush_reg_array(client, ctrl);

	return 0;
}

static int __ov2680_write_reg_is_consecutive(struct i2c_client *client,
					     struct ov2680_write_ctrl *ctrl,
					     const struct ov2680_reg *next)
{
	pr_info(" %s in\n", __func__);
	return 0;
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

static int ov2680_write_reg_array(struct i2c_client *client,
				  const struct ov2680_reg *reglist)
{
	const struct ov2680_reg *next = reglist;
	struct ov2680_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	pr_info(" %s in\n", __func__);
	for (; next->type != OV2680_TOK_TERM; next++) {
		switch (next->type & OV2680_TOK_MASK) {
		case OV2680_TOK_DELAY:
			err = __ov2680_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ov2680_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ov2680_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ov2680_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ov2680_flush_reg_array(client, &ctrl);
}
static int ov2680_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	pr_info(" %s in\n", __func__);
	*val = (OV2680_FOCAL_LENGTH_NUM << 16) | OV2680_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov2680_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for imx*/
	pr_info(" %s in\n", __func__);
	*val = (OV2680_F_NUMBER_DEFAULT_NUM << 16) | OV2680_F_NUMBER_DEM;
	return 0;
}

static int ov2680_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	pr_info(" %s in\n", __func__);
	*val = (OV2680_F_NUMBER_DEFAULT_NUM << 24) |
		(OV2680_F_NUMBER_DEM << 16) |
		(OV2680_F_NUMBER_DEFAULT_NUM << 8) | OV2680_F_NUMBER_DEM;
	return 0;
}

static int ov2680_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	pr_info(" %s in\n", __func__);
	*val = ov2680_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int ov2680_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	pr_info(" %s in\n", __func__);
	*val = ov2680_res[dev->fmt_idx].bin_factor_y + 1;

	return 0;
}


static int ov2680_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct ov2680_resolution *res)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct atomisp_sensor_mode_data *buf = &info->data;
/*	const unsigned int ext_clk_freq_hz = 19200000;
	const unsigned int pll_invariant_div = 10;*/
	unsigned int pix_clk_freq_hz;
/*	u16 pre_pll_clk_div;
	u16 pll_multiplier;
	u16 op_pix_clk_div;*/
	u16 reg_val;
	int ret;
	pr_info(" %s in\n", __func__);
	if (info == NULL)
		return -EINVAL;

	/* pixel clock */
	pix_clk_freq_hz = res->pix_clk_freq * 1000000;

	dev->vt_pix_clk_freq_mhz = pix_clk_freq_hz;
	buf->vt_pix_clk_freq_mhz = pix_clk_freq_hz;


	/* get integration time */
	buf->coarse_integration_time_min = OV2680_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					OV2680_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = OV2680_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					OV2680_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = OV2680_FINE_INTG_TIME_MIN;
	buf->frame_length_lines = res->lines_per_frame;
	buf->line_length_pck = res->pixels_per_line;
	buf->read_mode = res->bin_mode;

	/* get the cropping and output resolution to ISP for this mode. */
	ret =  ov2680_read_reg(client, OV2680_16BIT,
					OV2680_H_CROP_START_H, &reg_val);
	if (ret)
		return ret;
	buf->crop_horizontal_start = reg_val;

	ret =  ov2680_read_reg(client, OV2680_16BIT,
					OV2680_V_CROP_START_H, &reg_val);
	if (ret)
		return ret;
	buf->crop_vertical_start = reg_val;

	ret = ov2680_read_reg(client, OV2680_16BIT,
					OV2680_H_CROP_END_H, &reg_val);
	if (ret)
		return ret;
	buf->crop_horizontal_end = reg_val;

	ret = ov2680_read_reg(client, OV2680_16BIT,
					OV2680_V_CROP_END_H, &reg_val);
	if (ret)
		return ret;
	buf->crop_vertical_end = reg_val;

	ret = ov2680_read_reg(client, OV2680_16BIT,
					OV2680_H_OUTSIZE_H, &reg_val);
	if (ret)
		return ret;
	buf->output_width = reg_val;

	ret = ov2680_read_reg(client, OV2680_16BIT,
					OV2680_V_OUTSIZE_H, &reg_val);
	if (ret)
		return ret;
	buf->output_height = reg_val;

	buf->binning_factor_x = res->bin_factor_x ?
					res->bin_factor_x : 1;
	buf->binning_factor_y = res->bin_factor_y ?
					res->bin_factor_y : 1;
	return 0;
}

static long __ov2680_set_exposure(struct v4l2_subdev *sd, int coarse_itg,
				 int gain, int digitgain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	u16 vts, hts;
	u16 reg,reg_5045;
	int ret, exp_val, vts_val;
	/*printk(KERN_ERR "__ov5670_set_exposure coarse_itg %d, gain %d,"
	 * "digitgain %d++\n",coarse_itg, gain, digitgain);*/
	pr_info(" %s in\n", __func__);
	if (dev->run_mode == CI_MODE_VIDEO)
		ov2680_res = ov2680_res_video;
	else if (dev->run_mode == CI_MODE_STILL_CAPTURE)
		ov2680_res = ov2680_res_still;
	else
		ov2680_res = ov2680_res_preview;

	hts = ov2680_res[dev->fmt_idx].pixels_per_line;
	vts = ov2680_res[dev->fmt_idx].lines_per_frame;

	ret = ov2680_read_reg(client, OV2680_8BIT, 0x4003, &reg);
	if(ret)
		return ret;
	ret = ov2680_read_reg(client, OV2680_8BIT, 0x5045, &reg_5045);
	reg_5045 = reg_5045|0x04;
	if(ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_8BIT,
			       0x3209, 0x00);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_8BIT,
			       0x320a, 0x01);
	if (ret)
		return ret;
ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_GROUP_ACCESS, 0x00);
	if (ret)
		return ret;

#if 1
	/*add for MWB*/

	ret = ov2680_write_reg(client, OV2680_8BIT, 0x5045, reg_5045);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_8BIT, 0x5048, reg);
	if (ret)
		return ret;
	/* Digital gain */
	dev_alert(&client->dev, "Lew:digitgain = %d", digitgain);
	if (digitgain) {
		if (digitgain > 128)
			digitgain = 128;
		ret = ov2680_write_reg(client, OV2680_16BIT,
				OV2680_MWB_RED_GAIN_H, digitgain<<4);
		if (ret)
			return ret;

		ret = ov2680_write_reg(client, OV2680_16BIT,
				OV2680_MWB_GREEN_GAIN_H, digitgain<<4);
		if (ret)
			return ret;

		ret = ov2680_write_reg(client, OV2680_16BIT,
				OV2680_MWB_BLUE_GAIN_H, digitgain<<4);
		if (ret)
			return ret;
	}
#endif

	 /* End group */
	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_GROUP_ACCESS, 0x10);
	if (ret)
		return ret;
	/* group hold */
	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_GROUP_ACCESS, 0x01);
	if (ret)
		return ret;

	/* Increase the VTS to match exposure + 8 */
	if (coarse_itg + OV2680_INTEGRATION_TIME_MARGIN > vts)
		vts_val = coarse_itg + OV2680_INTEGRATION_TIME_MARGIN;
	else
		vts_val = vts;
	{
		/*printk(KERN_ERR "__ov5670_set_exposure  0x%x,0x%x,0x%x,++\n"
		,vts_val,(vts_val & 0xFF),((vts_val >> 8) & 0xFF));*/
		ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_TIMING_VTS_H,
		(vts_val >> 8) & 0xFF);
		if (ret)
			return ret;
		ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_TIMING_VTS_L,
		vts_val & 0xFF);
		if (ret)
			return ret;
	}

	/* set exposure */
	/* Lower four bit should be 0*/
	exp_val = coarse_itg << 4;

	ret = ov2680_write_reg(client, OV2680_8BIT,
			       OV2680_EXPOSURE_L, exp_val & 0xFF);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_8BIT,
			       OV2680_EXPOSURE_M, (exp_val >> 8) & 0xFF);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_8BIT,
			       OV2680_EXPOSURE_H, (exp_val >> 16) & 0x0F);
	if (ret)
		return ret;

	/* add for MWB end. */
	/* Analog gain */
	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_AGC_L, gain & 0xff);
	if (ret){
		return ret;	
	}
	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_AGC_H, (gain >> 8) & 0xff);
	if (ret){
		return ret;
	}
	/* End group */
	ret = ov2680_write_reg(client, OV2680_8BIT,
			       OV2680_GROUP_ACCESS, 0x11);
	if (ret)
		return ret;
	ret = ov2680_write_reg(client, OV2680_8BIT,
			       0x320B, 0x15);

	if (ret)
		return ret;

	//pr_info("ov_log: ov2680 exposure:%d again:%d", exp_val, gain);
	/* Delay launch group */
	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_GROUP_ACCESS,
		0xa1);/* a0, ;e0=fast launch a0= delay launch */
	if (ret)
		return ret;

	return ret;
}

static int ov2680_set_exposure(struct v4l2_subdev *sd, int exposure,
	int gain, int digitgain)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	int ret;
	if (exposure_return0)
		return 0;
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	ret = __ov2680_set_exposure(sd, exposure, gain, digitgain);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long ov2680_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	int exp = exposure->integration_time[0];
	int gain = exposure->gain[0];
	int digitgain = exposure->gain[1];
	pr_info(" %s in\n", __func__);
	/* we should not accept the invalid value below. */
	if (gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	return ov2680_set_exposure(sd, exp, gain, digitgain);
}

static long ov2680_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov2680_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov2680_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_v, reg_v2;
	int ret;
	pr_info(" %s in\n", __func__);
	/* get exposure */
	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_EXPOSURE_L,
					&reg_v);
	if (ret)
		goto err;

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_EXPOSURE_M,
					&reg_v2);
	if (ret)
		goto err;

	reg_v += reg_v2 << 8;
	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_EXPOSURE_H,
					&reg_v2);
	if (ret)
		goto err;

	*value = reg_v + (((u32)reg_v2 << 16));
err:
	return ret;
}
struct ov2680_control ov2680_controls[] = {
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
		.query = ov2680_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV2680_FOCAL_LENGTH_DEFAULT,
			.maximum = OV2680_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV2680_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov2680_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV2680_F_NUMBER_DEFAULT,
			.maximum = OV2680_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV2680_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov2680_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV2680_F_NUMBER_RANGE,
			.maximum =  OV2680_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV2680_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov2680_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = OV2680_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov2680_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = OV2680_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov2680_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov2680_controls))

static struct ov2680_control *ov2680_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov2680_controls[i].qc.id == id)
			return &ov2680_controls[i];
	return NULL;
}

static int ov2680_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov2680_control *ctrl = ov2680_find_control(qc->id);
	struct ov2680_device *dev = to_ov2680_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* imx control set/get */
static int ov2680_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2680_control *s_ctrl;
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = ov2680_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov2680_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov2680_control *octrl = ov2680_find_control(ctrl->id);
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov2680_init(struct v4l2_subdev *sd)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);
	pr_info("ov2680 init++++\n");
	/* restore settings */
	ov2680_res = ov2680_res_preview;
	N_RES = N_RES_PREVIEW;

	ret = ov2680_write_reg(client, OV2680_8BIT,
					OV2680_SW_RESET, 0x01);
	if (ret) {
		dev_err(&client->dev, "ov2680 reset err.\n");
		pr_info("ov2680 init error\n");
		return ret;
	}
	pr_info("ov2680 init ----\n");
	mutex_unlock(&dev->input_lock);

	return 0;
}


static int power_up(struct v4l2_subdev *sd)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
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
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

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

static int ov2680_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	if (on == 0)
		return power_down(sd);
	else {
		ret = power_up(sd);
		if (!ret)
			return ov2680_init(sd);
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
static int distance(struct ov2680_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13)/w);
	unsigned int h_ratio;
	int match;
	pr_info(" %s in\n", __func__);
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
	struct ov2680_resolution *tmp_res = NULL;
	pr_info(" %s in\n", __func__);
	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov2680_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;
	pr_info(" %s in\n", __func__);

	for (i = 0; i < N_RES; i++) {
		if (w != ov2680_res[i].width)
			continue;
		if (h != ov2680_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int ov2680_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	int idx;
	pr_info(" %s in\n", __func__);

	if (!fmt)
		return -EINVAL;
	idx = nearest_resolution_index(fmt->width,
					fmt->height);

	pr_info(" %s idx %d\n", __func__, idx);
	if (idx == -1) {
		/* return the largest resolution */
		fmt->width = ov2680_res[N_RES - 1].width;
		fmt->height = ov2680_res[N_RES - 1].height;
		pr_info("select ov2680 %dx%d\n", fmt->width, fmt->height);
	} else {
		fmt->width = ov2680_res[idx].width;
		fmt->height = ov2680_res[idx].height;
		pr_info("select ov2680 %dx%d\n", fmt->width, fmt->height);
	}
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;

	return 0;
}

/* TODO: remove it. */
static int startup(struct v4l2_subdev *sd)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
/*
	ret = ov2680_write_reg(client, ov2680_8BIT,
					ov2680_SW_RESET, 0x01);
	if (ret) {
		dev_err(&client->dev, "ov2680 reset err.\n");
		return ret;
	}
*/
	ret = ov2680_write_reg_array(client, ov2680_res[dev->fmt_idx].regs);
	if (ret) {
		dev_err(&client->dev, "ov2680 write register err.\n");
		return ret;
	}

	return ret;
}

static int ov2680_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *ov2680_info = NULL;
	int ret = 0;
	pr_info(" %s in\n", __func__);
	ov2680_info = v4l2_get_subdev_hostdata(sd);
	if (ov2680_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = ov2680_try_mbus_fmt(sd, fmt);
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
/*
	dev->pixels_per_line = ov2680_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ov2680_res[dev->fmt_idx].lines_per_frame;
*/
	ret = startup(sd);
	if (ret) {
		dev_err(&client->dev, "ov2680 startup err\n");
		goto err;
	}

	ret = ov2680_get_intg_factor(client, ov2680_info,
					&ov2680_res[dev->fmt_idx]);
	if (ret)
		dev_err(&client->dev, "failed to get integration_factor\n");

err:
	mutex_unlock(&dev->input_lock);
	return ret;
}
static int ov2680_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	pr_info(" %s in\n", __func__);
	if (!fmt)
		return -EINVAL;

	fmt->width = ov2680_res[dev->fmt_idx].width;
	fmt->height = ov2680_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov2680_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_SC_CMMN_CHIP_ID_H, &high);
	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_SC_CMMN_CHIP_ID_L, &low);
	id = ((((u16) high) << 8) | (u16) low);
	pr_info(" sensor id: 0x%x\n", id);

	if (id != OV2680_ID) {
		ATD_ov2680_status = 0;
		dev_err(&client->dev, "sensor ID error\n");
		return -ENODEV;
	} else {
		pr_info("Main sensor ID = 0x%x\n", id);
		ATD_ov2680_status = 1;
	}

	dev_dbg(&client->dev, "detect ov2680 success\n");
	return 0;
}

static int ov2680_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);

	ret = ov2680_write_reg(client, OV2680_8BIT, OV2680_SW_STREAM,
				enable ? OV2680_START_STREAMING :
				OV2680_STOP_STREAMING);

	mutex_unlock(&dev->input_lock);
	return ret;
}

/* ov2680 enum frame size, frame intervals */
static int ov2680_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	pr_info(" %s in\n", __func__);
	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov2680_res[index].width;
	fsize->discrete.height = ov2680_res[index].height;
	fsize->reserved[0] = ov2680_res[index].used;

	return 0;
}

static int ov2680_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	pr_info(" %s in\n", __func__);
	if (index >= N_RES)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = ov2680_res[index].width;
	fival->height = ov2680_res[index].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov2680_res[index].fps;

	return 0;
}

static int ov2680_enum_mbus_fmt(struct v4l2_subdev *sd,
				unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	pr_info(" %s in\n", __func__);
	*code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov2680_s_config(struct v4l2_subdev *sd,
			   int irq, void *platform_data)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	pr_info(" %s in\n", __func__);
	if (platform_data == NULL)
		return -ENODEV;

	dev->platform_data =
		(struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			dev_err(&client->dev, "platform init err\n");
			goto platform_init_failed;
		}
	}

	/* power off the module, then power on it in future
	 * as first power on by board may not fulfill the
	 * power on sequqence needed by the module
	 */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "ov2680 power-off err.\n");
		goto fail_power_off;
	}

	ret = power_up(sd);
	if (ret) {
		dev_err(&client->dev, "ov2680 power-up err.\n");
		goto fail_power_on;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ov2680_detect(client);
	if (ret) {
		dev_err(&client->dev, "ov2680_detect err s_config.\n");
		goto fail_csi_cfg;
	}

	/* turn off sensor, after probed */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "ov2680 power-off err.\n");
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
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
platform_init_failed:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int ov2680_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
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
			ov2680_res[dev->fmt_idx].fps;
	}
	return 0;
}

static int ov2680_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ov2680_res = ov2680_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ov2680_res = ov2680_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ov2680_res = ov2680_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int ov2680_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	pr_info(" %s in\n", __func__);
	interval->interval.numerator = 1;
	interval->interval.denominator = ov2680_res[dev->fmt_idx].fps;

	return 0;
}

static int ov2680_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	pr_info(" %s in\n", __func__);
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	
	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ov2680_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	pr_info(" %s in\n", __func__);
	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov2680_res[index].width;
	fse->min_height = ov2680_res[index].height;
	fse->max_width = ov2680_res[index].width;
	fse->max_height = ov2680_res[index].height;

	return 0;

}

static struct v4l2_mbus_framefmt *
__ov2680_get_pad_format(struct ov2680_device *sensor,
			struct v4l2_subdev_fh *fh, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,
			"__ov2680_get_pad_format err. pad %x\n", pad);
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

static int ov2680_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov2680_device *snr = to_ov2680_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov2680_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int ov2680_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov2680_device *snr = to_ov2680_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int ov2680_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	pr_info(" %s in\n", __func__);
	mutex_lock(&dev->input_lock);
	*frames = ov2680_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops ov2680_sensor_ops = {
	.g_skip_frames	= ov2680_g_skip_frames,
};

static const struct v4l2_subdev_video_ops ov2680_video_ops = {
	.s_stream = ov2680_s_stream,
	.g_parm = ov2680_g_parm,
	.s_parm = ov2680_s_parm,
	.enum_framesizes = ov2680_enum_framesizes,
	.enum_frameintervals = ov2680_enum_frameintervals,
	.enum_mbus_fmt = ov2680_enum_mbus_fmt,
	.try_mbus_fmt = ov2680_try_mbus_fmt,
	.g_mbus_fmt = ov2680_g_mbus_fmt,
	.s_mbus_fmt = ov2680_s_mbus_fmt,
	.g_frame_interval = ov2680_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov2680_core_ops = {
	.s_power = ov2680_s_power,
	.queryctrl = ov2680_queryctrl,
	.g_ctrl = ov2680_g_ctrl,
	.s_ctrl = ov2680_s_ctrl,
	.ioctl = ov2680_ioctl,
};

static const struct v4l2_subdev_pad_ops ov2680_pad_ops = {
	.enum_mbus_code = ov2680_enum_mbus_code,
	.enum_frame_size = ov2680_enum_frame_size,
	.get_fmt = ov2680_get_pad_format,
	.set_fmt = ov2680_set_pad_format,
};

static const struct v4l2_subdev_ops ov2680_ops = {
	.core = &ov2680_core_ops,
	.video = &ov2680_video_ops,
	.pad = &ov2680_pad_ops,
	.sensor = &ov2680_sensor_ops,
};

static int ov2680_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov2680_device *dev = to_ov2680_sensor(sd);
	dev_dbg(&client->dev, "ov2680_remove...\n");

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);

	return 0;
}

/*++++++++++++++++dbgfs+++++++++++++++++ */
static int dbg_ov2680_vga_status_open(
	struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_ov2680_vga_status_read(
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

static const struct file_operations dbg_ov2680_vga_status_fops = {
	.open		= dbg_ov2680_vga_status_open,
	.read		= dbg_ov2680_vga_status_read,
};

static ssize_t dbg_dump_ov2680_otp_read(
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

	len = snprintf(bp, dlen, "%s\n", OV2680_OTP);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov2680_otp_fops = {
	.open		= dbg_ov2680_vga_status_open,
	.read		= dbg_dump_ov2680_otp_read,
};


static ssize_t dbg_dump_ov2680_res_read(
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

	len = snprintf(bp, dlen, "%s\n", OV2680_RESOLUTION);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov2680_res_fops = {
	.open		= dbg_ov2680_vga_status_open,
	.read		= dbg_dump_ov2680_res_read,
};

static ssize_t dbg_dump_ov2680_module_read(
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

	len = snprintf(bp, dlen, "%s\n", OV2680_MODULE);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov2680_module_fops = {
	.open		= dbg_ov2680_vga_status_open,
	.read		= dbg_dump_ov2680_module_read,
};

static ssize_t dbg_dump_ov2680_uid_read(
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

	len = snprintf(bp, dlen, "%s\n", OV2680_UID);

	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;

	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static const struct file_operations dbg_dump_ov2680_uid_fops = {
	.open		= dbg_ov2680_vga_status_open,
	.read		= dbg_dump_ov2680_uid_read,
};


static int ov2680_dbgfs_init(void)
{
	struct dentry *debugfs_dir;

	debugfs_dir = debugfs_create_dir("camera1", NULL);
	debugfs_create_u32("vga_status", 0644, debugfs_dir, &ATD_ov2680_status);
	(void) debugfs_create_file("CameraOTP", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_ov2680_otp_fops);
	(void) debugfs_create_file("CameraOTP", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_ov2680_otp_fops);
	(void) debugfs_create_file("i2c_write", S_IRUGO,
		debugfs_dir, NULL, &dbg_ov2680_vga_status_fops);
	(void) debugfs_create_file("CameraResolution", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_ov2680_res_fops);
	(void) debugfs_create_file("CameraModule", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_ov2680_module_fops);
	(void) debugfs_create_file("Camera_Unique_ID", S_IRUGO,
		debugfs_dir, NULL, &dbg_dump_ov2680_uid_fops);


	debugfs_create_x16("by_file", 0644, debugfs_dir, &g_by_file);
	debugfs_create_u32("sensor_id", 0644, debugfs_dir, &who_am_i);
	debugfs_create_u32("exposure_return0", 0644, debugfs_dir, &exposure_return0);

	return 0;
}
/*---------------------dbgfs----------------------- */


static int ov2680_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov2680_device *dev;
	int ret;

	pr_info("%s in\n",__func__);
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ov2680_ops);

	ret = ov2680_s_config(&dev->sd, client->irq, client->dev.platform_data);
	if (ret)
		goto out_free;

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		ov2680_remove(client);

	ov2680_dbgfs_init();

	return ret;

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

MODULE_DEVICE_TABLE(i2c, ov2680_id);
static struct i2c_driver ov2680_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV2680_NAME,
	},
	.probe = ov2680_probe,
	.remove = ov2680_remove,
	.id_table = ov2680_id,
};

static int init_ov2680(void)
{
	return i2c_add_driver(&ov2680_driver);
}

static void exit_ov2680(void)
{

	i2c_del_driver(&ov2680_driver);
}

module_init(init_ov2680);
module_exit(exit_ov2680);

MODULE_AUTHOR("Nathan Lee <nathanch_lee@asus.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision 2680 sensors");
MODULE_LICENSE("GPL");
