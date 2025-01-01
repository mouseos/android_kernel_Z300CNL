/*
 *
 * FocalTech fts TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * VERSION      	DATE			AUTHOR
 *    1.0		       2014-09			mshl
 *
 */

 /*******************************************************************************
*
* File Name: focaltech.c
*
* Author: mshl
*
* Created: 2014-09
*
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/
/*******************************************************************************
* Included header files
*******************************************************************************/
//user defined include header files
#include "focaltech_core.h"
/* Early-suspend level */
#define FTS_SUSPEND_LEVEL 1


/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_META_REGS		3
#define FTS_ONE_TCH_LEN		6
#define FTS_TCH_LEN(x)		(FTS_META_REGS + FTS_ONE_TCH_LEN * x)

#define FTS_PRESS		0x7F
#define FTS_MAX_ID		0x0F
#define FTS_TOUCH_X_H_POS	3
#define FTS_TOUCH_X_L_POS	4
#define FTS_TOUCH_Y_H_POS	5
#define FTS_TOUCH_Y_L_POS	6
#define FTS_TOUCH_PRE_POS	7
#define FTS_TOUCH_AREA_POS	8
#define FTS_TOUCH_POINT_NUM		2
#define FTS_TOUCH_EVENT_POS	3
#define FTS_TOUCH_ID_POS		5

#define FTS_TOUCH_DOWN		0
#define FTS_TOUCH_UP		1
#define FTS_TOUCH_CONTACT	2

#define POINT_READ_BUF	(3 + FTS_ONE_TCH_LEN * FTS_MAX_POINTS)

/*register address*/
#define FTS_REG_DEV_MODE	0x00
#define FTS_DEV_MODE_REG_CAL	0x02

#define FTS_REG_PMODE		0xA5

#define FTS_REG_POINT_RATE	0x88
#define FTS_REG_THGROUP		0x80

/* power register bits*/
#define FTS_PMODE_ACTIVE	0x00
#define FTS_PMODE_MONITOR	0x01
#define FTS_PMODE_STANDBY	0x02
#define FTS_PMODE_HIBERNATE	0x03

#define FTS_STATUS_NUM_TP_MASK	0x0F

#define FTS_VTG_MIN_UV		2600000
#define FTS_VTG_MAX_UV		3300000
#define FTS_I2C_VTG_MIN_UV	1800000
#define FTS_I2C_VTG_MAX_UV	1800000

#define FTS_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FTS_8BIT_SHIFT		8
#define FTS_4BIT_SHIFT		4

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
struct i2c_client *fts_i2c_client;
struct fts_ts_data *fts_wq_data;
struct input_dev *fts_input_dev;

static unsigned int buf_count_add=0;
static unsigned int buf_count_neg=0;

u8 buf_touch_data[30*POINT_READ_BUF] = { 0 };

/*******************************************************************************
* Static function prototypes
*******************************************************************************/
static int fts_ts_start(struct device *dev);
static int fts_ts_stop(struct device *dev);

/*******************************************************************************
*  Name: fts_i2c_read
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

/*******************************************************************************
*  Name: fts_i2c_write
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	mutex_lock(&i2c_rw_access);
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	mutex_unlock(&i2c_rw_access);
	
	return ret;
}

/*******************************************************************************
*  Name: fts_write_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return fts_i2c_write(client, buf, sizeof(buf));
}

/*******************************************************************************
*  Name: fts_read_reg
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return fts_i2c_read(client, &addr, 1, val, 1);
}

/*******************************************************************************
*  Name: fts_ts_interrupt
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static irqreturn_t fts_ts_interrupt(int irq, void *dev_id)
{
	struct fts_ts_data *fts_ts = dev_id;
	int ret;

	dev_dbg(&fts_i2c_client->dev, "Enter irq handler\n");

	disable_irq_nosync(fts_ts->client->irq);
	fts_wq_data->irq_status = false;

	if (!fts_ts) {
		pr_err("%s: Invalid fts_ts\n", __func__);
		enable_irq(fts_ts->client->irq);
		return IRQ_HANDLED;
	}

	ret = queue_work(fts_ts->ts_workqueue, &fts_ts->touch_event_work);
	if (!ret) {
		dev_dbg(&fts_i2c_client->dev, "queue work failed\n");
		enable_irq(fts_ts->client->irq);
	}

	dev_dbg(&fts_i2c_client->dev, "Leave irq handler\n");
	return IRQ_HANDLED;
}

/*******************************************************************************
*  Name: fts_report_value
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_report_value(struct fts_ts_data *data)
{
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	int touchs = 0;
	u8 pointid = FTS_MAX_ID;
	
	u8 buf[POINT_READ_BUF] = { 0 };

	buf_count_neg++;
	
	memcpy( buf,buf_touch_data+(((buf_count_neg-1)%30)*POINT_READ_BUF), sizeof(u8)*POINT_READ_BUF );


	memset(event, 0, sizeof(struct ts_event));

	event->point_num=buf[FTS_TOUCH_POINT_NUM] & 0x0F;
	event->touch_point = 0;
	for (i = 0; i < FTS_MAX_POINTS; i++) {
		pointid = (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		if (pointid >= FTS_MAX_ID)
			break;
		else
			event->touch_point++;
		
		event->au16_x[i] =
		    (s16) (buf[FTS_TOUCH_X_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_X_L_POS + FTS_ONE_TCH_LEN * i];
		event->au16_y[i] =
		    (s16) (buf[FTS_TOUCH_Y_H_POS + FTS_ONE_TCH_LEN * i] & 0x0F) <<
		    8 | (s16) buf[FTS_TOUCH_Y_L_POS + FTS_ONE_TCH_LEN * i];
		event->au8_touch_event[i] =
		    buf[FTS_TOUCH_EVENT_POS + FTS_ONE_TCH_LEN * i] >> 6;
		event->au8_finger_id[i] =
		    (buf[FTS_TOUCH_ID_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->area[i] =
			(buf[FTS_TOUCH_AREA_POS + FTS_ONE_TCH_LEN * i]) >> 4;
		event->pressure[i] =
			(s16) buf[FTS_TOUCH_PRE_POS + FTS_ONE_TCH_LEN * i];

		if(0 == event->area[i])
			event->area[i] = 0x09;

		if(0 == event->pressure[i])
			event->pressure[i] = 0x3f;

		if((event->au8_touch_event[i]==0 || event->au8_touch_event[i]==2)&&(event->point_num==0))
			return;
	}
	
	for (i = 0; i < event->touch_point; i++)
	{
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		
		if (event->au8_touch_event[i] == FTS_TOUCH_DOWN || event->au8_touch_event[i] == FTS_TOUCH_CONTACT)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);
			touchs |= BIT(event->au8_finger_id[i]);
			data->touchs |= BIT(event->au8_finger_id[i]);
		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			data->touchs &= ~BIT(event->au8_finger_id[i]);
		}
	}

	if(unlikely(data->touchs ^ touchs))
	{
		for(i = 0; i < FTS_MAX_POINTS; i++)
		{
			if(BIT(i) & (data->touchs ^ touchs))
			{
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);
	}
	else
	{
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}

	input_sync(data->input_dev);
}

/*******************************************************************************
*  Name: fts_touch_irq_work
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_touch_irq_work(struct work_struct *work)
{
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;

	dev_dbg(&fts_i2c_client->dev, "Enter irq work\n");

	if (fts_wq_data->suspended && (fts_wq_data->zenMotion_ctrl > 0)) {
		ret = fts_read_Gesturedata();
		if (ret < 0)
			goto err_read_data;
	} else {
		ret = fts_i2c_read(fts_wq_data->client, buf, 1, buf,
							POINT_READ_BUF);
		if (ret < 0) {
			dev_err(&fts_wq_data->client->dev,
				"%s read touchdata failed.\n", __func__);
			goto err_read_data;
		}
		buf_count_add++;
		memcpy(buf_touch_data + (((buf_count_add-1)%30)*POINT_READ_BUF),
					buf, sizeof(u8)*POINT_READ_BUF);
		fts_report_value(fts_wq_data);
	}

err_read_data:
	enable_irq(fts_wq_data->client->irq);
	fts_wq_data->irq_status = true;
	dev_dbg(&fts_i2c_client->dev, "Leave irq work\n");
}

/*******************************************************************************
*  Name: fts_gpio_configure
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_gpio_configure(struct fts_ts_data *data, bool on)
{
	int err = 0;

	if (on) {
		printk("[Focal] HW reset sequence:\n");
		printk("[Focal] irq value = %d, rst_value = %d\n",
			(gpio_get_value(data->pdata->irq_gpio) == 0)? 0 : 1,
			(gpio_get_value(data->pdata->reset_gpio) == 0)? 0 : 1);

		if (gpio_is_valid(data->pdata->irq_gpio)) {
			err = gpio_request(data->pdata->irq_gpio,
						"fts_irq_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"irq gpio request failed");
				goto err_irq_gpio_req;
			}

			err = gpio_direction_input(data->pdata->irq_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"set_direction for irq gpio failed\n");
				goto err_irq_gpio_dir;
			}
		}

		if (gpio_is_valid(data->pdata->reset_gpio)) {
			err = gpio_request(data->pdata->reset_gpio,
						"fts_reset_gpio");
			if (err) {
				dev_err(&data->client->dev,
					"reset gpio request failed");
				goto err_irq_gpio_dir;
			}

			err = gpio_direction_output(data->pdata->reset_gpio, 0);
			if (err) {
				dev_err(&data->client->dev,
				"set_direction for reset gpio failed\n");
				goto err_reset_gpio_dir;
			}
			printk("[Focal] irq value = %d, rst_value = %d\n",
			(gpio_get_value(data->pdata->irq_gpio) == 0)? 0 : 1,
			(gpio_get_value(data->pdata->reset_gpio) == 0)? 0 : 1);

			msleep(data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
			printk("[Focal] irq value = %d, rst_value = %d\n",
			(gpio_get_value(data->pdata->irq_gpio) == 0)? 0 : 1,
			(gpio_get_value(data->pdata->reset_gpio) == 0)? 0 : 1);
		}

		return 0;
	} else {
		if (gpio_is_valid(data->pdata->irq_gpio))
			gpio_free(data->pdata->irq_gpio);
		if (gpio_is_valid(data->pdata->reset_gpio)) {
			/*
			 * This is intended to save leakage current
			 * only. Even if the call(gpio_direction_input)
			 * fails, only leakage current will be more but
			 * functionality will not be affected.
			 */
			err = gpio_direction_input(data->pdata->reset_gpio);
			if (err) {
				dev_err(&data->client->dev,
					"unable to set direction for gpio "
					"[%d]\n", data->pdata->irq_gpio);
			}
			gpio_free(data->pdata->reset_gpio);
		}

		return 0;
	}

err_reset_gpio_dir:
	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);
err_irq_gpio_dir:
	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
err_irq_gpio_req:
	return err;
}

#ifdef CONFIG_PM
/*******************************************************************************
*  Name: fts_ts_start
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_start(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	msleep(data->pdata->soft_rst_dly);

	if ((data->zenMotion_ctrl) > 0)
		fts_write_reg(fts_i2c_client, 0xd0, 0x00);
	else
		enable_irq(data->client->irq);

	data->irq_status = true;
	data->suspended = false;

	pr_info("[Focal] Resume done\n");
	return 0;
}

/*******************************************************************************
*  Name: fts_ts_stop
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_stop(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2];
	int i;

	if (data->zenMotion_ctrl > 0) {
		/*IC enter gesture mode*/
		pr_info("[Focal][Touch] dclick = %d, gesture = %d\n",
			(data->zenMotion_ctrl & 0x10) >> 4,
			(data->zenMotion_ctrl & 0x20) >> 5);

		fts_write_reg(fts_i2c_client, 0xd0, 0x01);
		fts_write_reg(fts_i2c_client, 0xd1, data->zenMotion_ctrl);

		/*if gesture on, handle individual pattern*/
		if((data->zenMotion_ctrl & 0x20) > 0) {
			if (data->gesture_wec_on) {
				fts_write_reg(fts_i2c_client, 0xd2,
							data->gesture_wec);
			}
			if (data->gesture_s_on)
				fts_write_reg(fts_i2c_client, 0xd5, 0x40);
			if (data->gesture_z_on)
				fts_write_reg(fts_i2c_client, 0xd7, 0x20);
			if (data->gesture_v_on)
				fts_write_reg(fts_i2c_client, 0xd6, 0x10);
		}
		pr_info("[Focal] Enter gesture mode\n");
	} else {
		disable_irq(data->client->irq);
		data->irq_status = false;
		txbuf[0] = FTS_REG_PMODE;
		txbuf[1] = FTS_PMODE_HIBERNATE;
		fts_i2c_write(data->client, txbuf, sizeof(txbuf));
		pr_info("[Focal] Enter sleep mode\n");
	}

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	data->suspended = true;
	pr_info("[Focal] Suspend done\n");
	return 0;
}

/*******************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_ts_suspend(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	pr_info("[Focal] Start suspend\n");
	if (data->loading_fw) {
		dev_info(dev, "[Focal] Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "[Focal] Already in suspend state\n");
		return 0;
	}

	return fts_ts_stop(dev);
}

/*******************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_ts_resume(struct device *dev)
{
	struct fts_ts_data *data = dev_get_drvdata(dev);

	pr_info("[Focal] Start resume\n");
	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

	return fts_ts_start(dev);
}

static const struct dev_pm_ops fts_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = fts_ts_suspend,
	.resume = fts_ts_resume,
#endif
};
#else
/*******************************************************************************
*  Name: fts_ts_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_suspend(struct device *dev)
{
	return 0;
}
/*******************************************************************************
*  Name: fts_ts_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_resume(struct device *dev)
{
	return 0;
}
#endif

/*******************************************************************************
*  Name: fts_ts_early_suspend
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_ts_early_suspend(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);
	int err;

	err = fts_ts_suspend(&data->client->dev);
	if (err)
		pr_info("[Focal] Early suspend failed\n");
}

/*******************************************************************************
*  Name: fts_ts_late_resume
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void fts_ts_late_resume(struct early_suspend *handler)
{
	struct fts_ts_data *data = container_of(handler,
						   struct fts_ts_data,
						   early_suspend);
	int err;

	err = fts_ts_resume(&data->client->dev);
	if (err)
		pr_info("[Focal] Late resume failed\n");
}

/*******************************************************************************
*  Name: fts_debug_addr_is_valid
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static bool fts_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

/*******************************************************************************
*  Name: fts_debug_data_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_data_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_data_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_data_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr)) {
		rc = fts_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, fts_debug_data_get, fts_debug_data_set, "0x%02llX\n");

/*******************************************************************************
*  Name: fts_debug_addr_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_addr_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	if (fts_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_addr_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_addr_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (fts_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, fts_debug_addr_get, fts_debug_addr_set, "0x%02llX\n");

/*******************************************************************************
*  Name: fts_debug_suspend_set
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_suspend_set(void *_data, u64 val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		fts_ts_suspend(&data->client->dev);
	else
		fts_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

/*******************************************************************************
*  Name: fts_debug_suspend_get
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_debug_suspend_get(void *_data, u64 *val)
{
	struct fts_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, fts_debug_suspend_get, fts_debug_suspend_set, "%lld\n");

/*******************************************************************************
*  Name: fts_debug_dump_info
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
int fts_debug_dump_info(struct seq_file *m, void *v)
{
	struct fts_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

/*******************************************************************************
*  Name: debugfs_dump_info_open
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, fts_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

static ssize_t focal_show_tpfwver(struct switch_dev *sdev, char *buf)
{
	int num_read_chars = 0;
	u8 fwver = 0;

	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0) {
		pr_info("[Focal] %s: read FW fail\n", __func__);
		num_read_chars = snprintf(buf, PAGE_SIZE, "Unknown\n");
	}

	if (fwver == 255) {
		pr_info("[Focal] %s: read FW fail\n", __func__);
		num_read_chars = snprintf(buf, PAGE_SIZE, "Unknown\n");
	} else {
		pr_info("[Focal] %s: FW = 0x%02x\n", __func__, fwver);
		num_read_chars = snprintf(buf, PAGE_SIZE, "v%02x\n", fwver);
	}
	mutex_unlock(&fts_input_dev->mutex);

	return num_read_chars;
}

static void focal_cable_status(struct work_struct *work)
{
	char buf[2] = {0};
	if (fts_wq_data->init_success == 1) {
		if (fts_wq_data->usb_status) {
			/*AC plug in*/
			buf[0] = 0x8B;
			buf[1] = 0x01;
			fts_write_reg(fts_i2c_client, buf[0], buf[1]);
			pr_info("[Focal] USB charger in\n");
		} else {
			/*no AC */
			buf[0] = 0x8B;
			buf[1] = 0x00;
			fts_write_reg(fts_i2c_client, buf[0], buf[1]);
			pr_info("[Focal] no USB charger\n");
		}
	} else {
		pr_info("[Focal] init_success=%d, usb mode switched aborted\n",
						fts_wq_data->init_success);
	}
	return;
}

static int cable_status_notifier_callback(struct notifier_block *self,
					unsigned long action, void *dev)
{
	switch (action) {
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		fts_wq_data->usb_status = false;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_OTG:
		fts_wq_data->usb_status = false;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		fts_wq_data->usb_status = true;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		fts_wq_data->usb_status = true;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		fts_wq_data->usb_status = true;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		fts_wq_data->usb_status = true;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_SE1:
		fts_wq_data->usb_status = true;
		queue_work(fts_wq_data->usb_wq, &fts_wq_data->usb_detect_work);
		break;
	default:
		pr_info("[Focal] USB STATUS UNKNOWN: %d\n", (int)action);
		break;
	}
	return NOTIFY_OK;
}

/*******************************************************************************
*  Name: fts_ts_probe
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct fts_ts_platform_data *pdata;
	struct fts_ts_data *data;
	struct input_dev *input_dev;
	project_id project = asustek_get_project_id();
	lcd_type cur_lcd_type = asustek_get_lcd_type();
	hw_rev pcbid_hw_rev = asustek_get_hw_rev();
	
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

	pr_info("[Focal] project_id = %d, lcd_type = %d, hw_rev = %d\n",
				project, cur_lcd_type, pcbid_hw_rev);
	if ( project != 10 ) {
		pr_info("[Focal] Not Z300CNL project, exit probe.\n");
		return -EINVAL;
	}

	if ( cur_lcd_type != 0 ) {
		pr_info("[Focal] It's Z300CNL without BOE panel, exit probe\n");
		return -EINVAL;
	}

	printk("[Focal] Enter Focal driver probe\n");

	pdata = client->dev.platform_data;
	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data = devm_kzalloc(&client->dev, sizeof(struct fts_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	fts_wq_data = data;

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FTS_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data->tch_data_len = FTS_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev, data->tch_data_len, GFP_KERNEL);
	if (!data->tch_data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data->init_success = false;
	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;
	data->suspended = false;
	data->irq_status = true;
	data->usb_status = false;

	input_dev->name = "fts_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
				pdata->x_min, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
				pdata->y_min, pdata->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0x0f, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 0xff, 0, 0);

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}

	printk("[Focal] gpio_irq = %d, gpio_rst = %d\n",
			data->pdata->irq_gpio, data->pdata->reset_gpio);
	err = fts_gpio_configure(data, true);
	if (err < 0) {
		dev_err(&client->dev,
			"Failed to configure the gpios\n");
		goto err_gpio_req;
	}

	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	INIT_WORK(&data->touch_event_work, fts_touch_irq_work);
	data->ts_workqueue = create_workqueue(FTS_WORKQUEUE_NAME);
	if (!data->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	INIT_WORK(&data->usb_detect_work, focal_cable_status);
	data->usb_wq = create_singlethread_workqueue("focal_usb_wq");
	if (!data->usb_wq) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	data->cable_status_notif.notifier_call = cable_status_notifier_callback;
	err = cable_status_register_client(&data->cable_status_notif);
	if (err) {
		pr_info("[Focal] register cable status notifier failed\n");
		goto exit_create_singlethread;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	/* check the controller id */
	reg_addr = FTS_REG_ID;
	err = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_gpio;
	}
	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		//goto free_gpio;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data->family_id = pdata->family_id;
	client->irq = gpio_to_irq(data->pdata->irq_gpio);
	fts_i2c_client = client;
	fts_input_dev = input_dev;

	fts_get_upgrade_array();

	err = request_threaded_irq(client->irq, NULL, fts_ts_interrupt,
				pdata->irqflags | IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
				client->dev.driver->name, data);
	if (err) {
		dev_err(&client->dev, "request irq failed\n");
		goto free_gpio;
	}

	disable_irq(client->irq);
	data->irq_status = false;

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data->dir = debugfs_create_dir(FTS_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}
	
	data->ts_info = devm_kzalloc(&client->dev, FTS_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	/*get some register information */
	reg_addr = FTS_REG_POINT_RATE;
	fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	pr_info("[Focal] report rate = %dHz\n", reg_value * 10);

	reg_addr = FTS_REG_THGROUP;
	err = fts_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	pr_info("[Focal] touch threshold = %d\n", reg_value * 4);

	fts_update_fw_ver(data);
	fts_update_fw_vendor_id(data);
	fts_update_fw_panel_id(data);

	pr_info("[Focal] FW_ver = 0x%x.%x.%x, vendor_id = 0x%x, panel_id = 0x%x\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2],
		data->fw_vendor_id, data->fw_panel_id);

	FTS_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

	#ifdef FTS_APK_DEBUG
		fts_create_apk_debug_channel(client);
	#endif

	#ifdef FTS_SYSFS_DEBUG
		fts_create_sysfs(client);
	#endif
	
	#ifdef FTS_CTL_IIC
		if (fts_rw_iic_drv_init(client) < 0) {
			dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",	__func__);
		}
	#endif
	

	pr_info("[Focal] enter %s-%d\n", __func__, __LINE__);

	wake_lock_init(&data->fts_wake_lock, WAKE_LOCK_SUSPEND,
				"fts_gesture_wake_lock");

	data->zenMotion_ctrl = 0x00;
	data->gesture_wec = 0x00;
	data->gesture_wec_on = false;
	data->gesture_s_on = false;
	data->gesture_z_on = false;
	data->gesture_v_on = false;
	fts_Gesture_init(input_dev);

	#ifdef FTS_AUTO_UPGRADE
	pr_info("[Focal] *****Enter CTP Auto Upgrade*****\n");
	fts_ctpm_auto_upgrade(client);
	#endif 

	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + FTS_SUSPEND_LEVEL;
	data->early_suspend.suspend = fts_ts_early_suspend;
	data->early_suspend.resume = fts_ts_late_resume;
	register_early_suspend(&data->early_suspend);

	data->touch_sdev.name = "touch";
	data->touch_sdev.print_name = focal_show_tpfwver;
	if (switch_dev_register(&data->touch_sdev) < 0)
		pr_info("[Focal] failed to register switch_dev\n");

	pr_info("[Focal] register switch device OK\n");

	enable_irq(client->irq);
	data->irq_status = true;
	data->init_success = true;
	printk("[Focal] driver probe done\n");

	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
	
free_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);

	cable_status_unregister_client(&data->cable_status_notif);
exit_create_singlethread:
	if (data->ts_workqueue)
		destroy_workqueue(data->ts_workqueue);

	if (data->usb_wq)
		destroy_workqueue(data->usb_wq);

	i2c_set_clientdata(client, NULL);
err_gpio_req:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

/*******************************************************************************
*  Name: fts_ts_remove
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int fts_ts_remove(struct i2c_client *client)
{
	struct fts_ts_data *data = i2c_get_clientdata(client);

	cancel_work_sync(&data->touch_event_work);
	destroy_workqueue(data->ts_workqueue);

	cancel_work_sync(&data->usb_detect_work);
	destroy_workqueue(data->usb_wq);

	cable_status_unregister_client(&data->cable_status_notif);

	debugfs_remove_recursive(data->dir);

#ifdef FTS_APK_DEBUG
		fts_release_apk_debug_channel();
#endif

#ifdef FTS_SYSFS_DEBUG
		fts_remove_sysfs(fts_i2c_client);
#endif


#ifdef FTS_CTL_IIC
		fts_rw_iic_drv_exit();
#endif

	unregister_early_suspend(&data->early_suspend);

	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);

	input_unregister_device(data->input_dev);

	wake_lock_destroy(&data->fts_wake_lock);

	return 0;
}

static const struct i2c_device_id fts_ts_id[] = {
	{"fts_ts", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, fts_ts_id);

static struct i2c_driver fts_ts_driver = {
	.probe = fts_ts_probe,
	.remove = fts_ts_remove,
	.driver = {
		   .name = "fts_ts",
		   .owner = THIS_MODULE,
#ifdef CONFIG_PM
		   .pm = &fts_ts_pm_ops,
#endif
		   },
	.id_table = fts_ts_id,
};

/*******************************************************************************
*  Name: fts_ts_init
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static int __init fts_ts_init(void)
{
	printk("[Focal] enter %s-%d\n", __func__, __LINE__);
	return i2c_add_driver(&fts_ts_driver);
}

/*******************************************************************************
*  Name: fts_ts_exit
*  Brief:
*  Input:
*  Output: 
*  Return: 
*******************************************************************************/
static void __exit fts_ts_exit(void)
{
	i2c_del_driver(&fts_ts_driver);
}

module_init(fts_ts_init);
module_exit(fts_ts_exit);

MODULE_DESCRIPTION("FocalTech fts TouchScreen driver");
MODULE_LICENSE("GPL v2");
