#include <asm/intel-mid.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>

#include "s2034.h"

static struct s2034_device s2034_dev;
static u16 vcm_position;
static int s2034_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	struct i2c_msg msg;
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;
	msg.addr = s2034_VCM_ADDR;
	msg.flags = 0;
	msg.len = s2034_REG_LENGTH + s2034_8BIT;
	msg.buf = &buf[0];

	if (i2c_transfer(client->adapter, &msg, 1) != 1)
		return -EIO;
	return 0;
}

static int s2034_i2c_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	buf[0] = reg;
	buf[1] = 0;

	msg[0].addr = s2034_VCM_ADDR;
	msg[0].flags = 0;
	msg[0].len = s2034_REG_LENGTH;
	msg[0].buf = &buf[0];

	msg[1].addr = s2034_VCM_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = s2034_8BIT;
	msg[1].buf = &buf[1];
	*val = 0;
	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -EIO;
	*val = buf[1];
	return 0;
}

int s2034_vcm_power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -EINVAL;
	char Pos[2] = { (char)((vcm_position >> 4) & 0x3F), (char)((vcm_position << 4) & 0xF0) };
	/* Enable power */
	if (s2034_dev.platform_data) {
		ret = s2034_dev.platform_data->power_ctrl(sd, 1);
		if (ret)
			return ret;
	}
	/*
	 * waiting time requested by s2034(vcm)
	 */
	usleep_range(1000, 2000);

	/*
	 * Set vcm ringing control mode.
	 */
	/*s2034 Initial setting*/
	s2034_i2c_write(client, s2034_REG_PROTECTION_OFF, 0xa3);

	s2034_i2c_write(client, s2034_REG_DLC_MCLK_CONTROL, 0x0d);

	s2034_i2c_write(client, s2034_REG_SRC, 0x08);

	s2034_i2c_write(client, s2034_REG_PROTECTION_ON, 0x51);

	/*vcm initial position*/
	s2034_i2c_write(client, Pos[0], Pos[1]);
	s2034_dev.focus = vcm_position;

	return ret;
}

int s2034_vcm_power_down(struct v4l2_subdev *sd)
{
	int ret = -ENODEV;

	if (s2034_dev.platform_data)
		ret = s2034_dev.platform_data->power_ctrl(sd, 0);

	return ret;
}

void actuator_close(struct v4l2_subdev *sd)
{
	int value=s2034_dev.focus;
	int stride=20;
	int stime=10;

	pr_info("@%s ", __func__);
	if (value <=0)
		return;

	while(value>=stride) {
		value-=stride;
		s2034_t_focus_abs(sd, value);
		msleep(stime);
	}

	if(value > 5) {
		s2034_t_focus_abs(sd, 5);
		msleep(stime);
	}

	s2034_t_focus_abs(sd, 0);
}



int s2034_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -EINVAL;
	char Pos[2] = { (char)((val >> 4) & 0x3F), (char)((val << 4) & 0xF0) };

	ret = s2034_i2c_write(client, Pos[0], Pos[1]);

	return ret;
}

int s2034_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	int ret;
	value = min(value, s2034_MAX_FOCUS_POS);
	ret = s2034_t_focus_vcm(sd, value);
	if (ret == 0) {
		s2034_dev.number_of_steps = value - s2034_dev.focus;
		s2034_dev.focus = value;
		ktime_get_ts(&s2034_dev.timestamp_t_focus_abs);
	}

	return ret;
}

int s2034_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	return s2034_t_focus_abs(sd, s2034_dev.focus + value);
}

int s2034_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	u32 status = 0;
	struct timespec temptime;
	const struct timespec timedelay = {
		0,
		min_t(u32, abs(s2034_dev.number_of_steps)*DELAY_PER_STEP_NS,
			DELAY_MAX_PER_STEP_NS),
	};

	ktime_get_ts(&temptime);

	temptime = timespec_sub(temptime, (s2034_dev.timestamp_t_focus_abs));

	if (timespec_compare(&temptime, &timedelay) <= 0)
		status = ATOMISP_FOCUS_STATUS_MOVING
			| ATOMISP_FOCUS_HP_IN_PROGRESS;
	else
		status = ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE
			| ATOMISP_FOCUS_HP_COMPLETE;

	*value = status;

	return 0;
}

int s2034_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	s32 val;

	s2034_q_focus_status(sd, &val);

	if (val & ATOMISP_FOCUS_STATUS_MOVING)
		*value  = s2034_dev.focus - s2034_dev.number_of_steps;
	else
		*value  = s2034_dev.focus;

	return 0;
}

int s2034_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	return 0;
}

int s2034_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	return 0;
}

int s2034_vcm_init(struct v4l2_subdev *sd)
{
	/* set vcm mode to DIRECT */
//	s2034_dev.vcm_mode = s2034_DIRECT;
	vcm_position = 0x10; //Default position
	s2034_dev.platform_data = camera_get_af_platform_data();

	return s2034_dev.platform_data ? 0 : -ENODEV;
}
