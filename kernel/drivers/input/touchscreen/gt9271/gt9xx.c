/* drivers/input/touchscreen/gt9xx.c
 *
 * 2010 - 2013 Goodix Technology.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * Version: 2.2
 * Authors: andrew@goodix.com, meta@goodix.com
 * Release Date: 2014/01/14
 * Revision record:
 *      V1.0:
 *          first Release. By Andrew, 2012/08/31
 *      V1.2:
 *          modify gtp_reset_guitar,slot report,tracking_id & 0x0F. By Andrew, 2012/10/15
 *      V1.4:
 *          modify gt9xx_update.c. By Andrew, 2012/12/12
 *      V1.6:
 *          1. new heartbeat/esd_protect mechanism(add external watchdog)
 *          2. doze mode, sliding wakeup
 *          3. 3 more cfg_group(GT9 Sensor_ID: 0~5)
 *          3. config length verification
 *          4. names & comments
 *                  By Meta, 2013/03/11
 *      V1.8:
 *          1. pen/stylus identification
 *          2. read double check & fixed config support
 *          3. new esd & slide wakeup optimization
 *                  By Meta, 2013/06/08
 *      V2.0:
 *          1. compatible with GT9XXF
 *          2. send config after resume
 *                  By Meta, 2013/08/06
 *      V2.2:
 *          1. gt9xx_config for debug
 *          2. gesture wakeup
 *          3. pen separate input device, active-pen button support
 *          4. coordinates & keys optimization
 *                  By Meta, 2014/01/14
 *      V2.2.1:
 *          1. Fix F/W update system folder CFG memory len(memory out of size)
 *          2. Fix 2.2 modify release from HQ
 *          3. Fix reset change input after I2C address select
 */

#include <linux/irq.h>
#include <linux/gt9xx.h>
#include "gt9xx_openshort.h"

/* for PCBID */
/* #include <linux/board_asustek.h> */

static const char *goodix_ts_name = "gt9271";
static struct workqueue_struct *goodix_wq;
struct i2c_client *i2c_connect_client;
u8 config[GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH]
= { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };

static s8 gtp_i2c_test(struct i2c_client *client);
void gtp_reset_guitar(struct i2c_client *client, s32 ms);
s32 gtp_send_cfg(struct i2c_client *client);
void gtp_int_sync(s32 ms);

static ssize_t gt91xx_config_read_proc(struct file *, char __user *, size_t,
				       loff_t *);
static ssize_t gt91xx_config_write_proc(struct file *, const char __user *,
					size_t, loff_t *);
ssize_t get_touch_status(struct device *dev, struct device_attribute *attr,
			 char *buf);
ssize_t get_fw_version(struct device *dev, struct device_attribute *attr,
		       char *buf);
ssize_t get_cfg_version(struct device *dev, struct device_attribute *attr,
			char *buf);
s32 gtp_read_version(struct i2c_client *client, u16 *version);
s32 gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf,
			   int len);
ssize_t sysfs_dclick_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);
ssize_t sysfs_gesture_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);
ssize_t sysfs_flipcovermode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);

char FW_VERSION[3] = { '\0' };

u8 grp_cfg_version;
char Flipcovermode[4] = {'0'};
char Gesture_Dclick[4] = {'0'};
static char Gesture_type = 0x0;

/* static tp_type        tp_pcbid = 0; */

static int gt_status = 0x1;

static struct proc_dir_entry *gt91xx_config_proc;
static const struct file_operations config_proc_ops = {
	.owner = THIS_MODULE,
	.read = gt91xx_config_read_proc,
	.write = gt91xx_config_write_proc,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void goodix_ts_early_suspend(struct early_suspend *h);
static void goodix_ts_late_resume(struct early_suspend *h);
#endif

#if GTP_CREATE_WR_NODE
extern s32 init_wr_node(struct i2c_client *);
extern void uninit_wr_node(void);
#endif

#if GTP_AUTO_UPDATE
extern u8 gup_init_update_proc(struct goodix_ts_data *);
#endif

#if GTP_ESD_PROTECT
static struct delayed_work gtp_esd_check_work;
static struct workqueue_struct *gtp_esd_check_workqueue;
static void gtp_esd_check_func(struct work_struct *);
static s32 gtp_init_ext_watchdog(struct i2c_client *client);
void gtp_esd_switch(struct i2c_client *, s32);
#endif

#if GTP_GESTURE_WAKEUP
typedef enum {
	DOZE_DISABLED = 0,
	DOZE_ENABLED = 1,
	DOZE_WAKEUP = 2,
} DOZE_T;
static DOZE_T doze_status = DOZE_DISABLED;
static s8 gtp_enter_doze(struct goodix_ts_data *ts);
#endif

/*******************************************************
Function:
    Show gt_status
Input:
    dev
    attr
    buf
Output:
    print gt_status
*********************************************************/
ssize_t
get_touch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[GTP_DEBUG] touch status=%d. \n", gt_status);
	return sprintf(buf, "%d \n", gt_status);
}

/*******************************************************
Function:
    Show fw_version
Input:
    dev
    attr
    buf
Output:
    print fw_version
*********************************************************/
ssize_t
get_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	s32 ret = -1;
	u16 version_info;

	ret = gtp_read_version(i2c_connect_client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
		gt_status = gt_status & 0x0;
	}

	return sprintf(buf, "%02X%02X\n", FW_VERSION[0], FW_VERSION[1]);
}

/*******************************************************
Function:
    Show cfg_version
Input:
    dev
    attr
    buf
Output:
    print cfg_version
*********************************************************/
ssize_t
get_cfg_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 opr_buf[16] = { 0 };
	s32 ret = -1;

	ret =
	    gtp_i2c_read_dbl_check(i2c_connect_client, GTP_REG_CONFIG_DATA,
				   &opr_buf[0], 1);
	if (ret == SUCCESS) {
		grp_cfg_version = opr_buf[0];
		GTP_INFO("IC Config Version: %d, 0x%02X", opr_buf[0],
			 opr_buf[0]);
	}

	return sprintf(buf, "%d \n", grp_cfg_version);
}

/*******************************************************
Function:
    Read data from the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   read start address.
    buf[2~len-1]:   read data buffer.
    len:    GTP_ADDR_LENGTH + read bytes count
Output:
    numbers of i2c_msgs to transfer:
      2: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_read(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];
	/* msgs[0].scl_rate = 300 * 1000;     for Rockchip, etc.  */

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];
	/* msgs[1].scl_rate = 300 * 1000; */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5)) {

#if GTP_GESTURE_WAKEUP
		if (Gesture_type & (0x1 << GESTURE_MASK_Master)) {
			/* reset chip would quit doze mode  */
			if (DOZE_ENABLED == doze_status) {
				return ret;
			}
		}
#endif
		GTP_ERROR
		    ("I2C Read: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
		     (((u16) (buf[0] << 8)) | buf[1]), len - 2, ret);
		gtp_reset_guitar(client, 10);
	}
	return ret;
}

/*******************************************************
Function:
    Write data to the i2c slave device.
Input:
    client:     i2c device.
    buf[0~1]:   write start address.
    buf[2~len-1]:   data buffer
    len:    GTP_ADDR_LENGTH + write bytes count
Output:
    numbers of i2c_msgs to transfer:
    1: succeed, otherwise: failed
*********************************************************/
s32 gtp_i2c_write(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;
	/*msg.scl_rate = 300 * 1000;     for Rockchip, etc */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5)) {

#if GTP_GESTURE_WAKEUP
		if (Gesture_type & (0x1 << GESTURE_MASK_Master)) {
			if (DOZE_ENABLED == doze_status) {
				return ret;
			}
		}
#endif
		GTP_ERROR
		    ("I2C Write: 0x%04X, %d bytes failed, errcode: %d! Process reset.",
		     (((u16) (buf[0] << 8)) | buf[1]), len - 2, ret);
		gtp_reset_guitar(client, 10);
	}
	return ret;
}

/*******************************************************
Function:
    i2c read twice, compare the results
Input:
    client:  i2c device
    addr:    operate address
    rxbuf:   read data to store, if compare successful
    len:     bytes to read
Output:
    FAIL:    read failed
    SUCCESS: read successful
*********************************************************/
s32
gtp_i2c_read_dbl_check(struct i2c_client *client, u16 addr, u8 *rxbuf, int len)
{
	u8 buf[16] = { 0 };
	u8 confirm_buf[16] = { 0 };
	u8 retry = 0;

	while (retry++ < 3) {
		memset(buf, 0xAA, 16);
		buf[0] = (u8) (addr >> 8);
		buf[1] = (u8) (addr & 0xFF);
		gtp_i2c_read(client, buf, len + 2);

		memset(confirm_buf, 0xAB, 16);
		confirm_buf[0] = (u8) (addr >> 8);
		confirm_buf[1] = (u8) (addr & 0xFF);
		gtp_i2c_read(client, confirm_buf, len + 2);

		if (!memcmp(buf, confirm_buf, len + 2)) {
			memcpy(rxbuf, confirm_buf + 2, len);
			return SUCCESS;
		}
	}
	GTP_ERROR("I2C read 0x%04X, %d bytes, double check failed!", addr, len);
	return FAIL;
}

/*******************************************************
Function:
    Send config.
Input:
    client: i2c device.
Output:
    result of i2c write operation.
    1: succeed, otherwise: failed
*********************************************************/

s32 gtp_send_cfg(struct i2c_client *client)
{
	s32 ret = 2;

#if GTP_DRIVER_SEND_CFG
	s32 retry = 0;
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	if (ts->fixed_cfg) {
		GTP_INFO("Ic fixed config, no config sent!");
		return 0;
	} else if (ts->pnl_init_error) {
		GTP_INFO("Error occured in init_panel, no config sent");
		return 0;
	}

	GTP_INFO("Driver send config.");
	for (retry = 0; retry < 5; retry++) {
		ret =
		    gtp_i2c_write(client, config,
				  GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
		if (ret > 0) {
			break;
		}
	}
#endif
	return ret;
}

/*******************************************************
Function:
    Disable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_disable(struct goodix_ts_data *ts)
{
	unsigned long irqflags;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disable) {
		ts->irq_is_disable = 1;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Enable irq function
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
void gtp_irq_enable(struct goodix_ts_data *ts)
{
	unsigned long irqflags = 0;

	GTP_DEBUG_FUNC();

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disable) {
		enable_irq(ts->client->irq);
		ts->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
    Report touch point event
Input:
    ts: goodix i2c_client private data
    id: trackId
    x:  input x coordinate
    y:  input y coordinate
    w:  input pressure
Output:
    None.
*********************************************************/
static void
gtp_touch_down(struct goodix_ts_data *ts, s32 id, s32 x, s32 y, s32 w)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, id);
	input_mt_sync(ts->input_dev);

	GTP_DEBUG("ID:%d, X:%d, Y:%d, W:%d", id, x, y, w);
}

/*******************************************************
Function:
    Report touch release event
Input:
    ts: goodix i2c_client private data
Output:
    None.
*********************************************************/
static void gtp_touch_up(struct goodix_ts_data *ts, s32 id)
{
	input_report_key(ts->input_dev, BTN_TOUCH, 0);
}

/*******************************************************
Function:
    Goodix touchscreen work function
Input:
    work: work struct of goodix_workqueue
Output:
    None.
*********************************************************/
static void goodix_ts_work_func(struct work_struct *work)
{
	u8 end_cmd[3] = {
		GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF, 0 };
	u8 point_data[2 + 1 + 8 * GTP_MAX_TOUCH + 1] = {
		GTP_READ_COOR_ADDR >> 8, GTP_READ_COOR_ADDR & 0xFF };
	u8 touch_num = 0;
	u8 finger = 0;
	static u16 pre_touch;
	static u8 pre_key;
	u8 key_value = 0;
	u8 *coor_data = NULL;
	s32 input_x = 0;
	s32 input_y = 0;
	s32 input_w = 0;
	s32 id = 0;
	s32 i = 0;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	static int gesture_state;

#if GTP_GESTURE_WAKEUP
	u8 doze_buf[3] = { 0x81, 0x4B };
#endif

	GTP_DEBUG_FUNC();
	ts = container_of(work, struct goodix_ts_data, work);
	if (ts->enter_update) {
		wake_unlock(&ts->wake_lock);
		return;
	}
#if GTP_GESTURE_WAKEUP
	if (((Gesture_type & (0x1 << GESTURE_MASK_Master)) || Gesture_Dclick[0] == '1') && (DOZE_ENABLED == doze_status)) {
		ret = gtp_i2c_read(i2c_connect_client, doze_buf, 3);
		GTP_DEBUG("0x814B = 0x%02X", doze_buf[2]);
		if (ret > 0) {
			switch (doze_buf[2]) {
			case 'w':
				if ((Gesture_type & (0x1 << GESTURE_MASK_W)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_W, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_W, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_W) == 0)
					gtp_enter_doze(ts);
				break;
			case 's':
				if ((Gesture_type & (0x1 << GESTURE_MASK_S)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_S, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_S, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_S) == 0)
					gtp_enter_doze(ts);
				break;
			case 'e':
				if ((Gesture_type & (0x1 << GESTURE_MASK_E)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_E, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_E, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_E) == 0)
					gtp_enter_doze(ts);
				break;
			case 'c':
				if ((Gesture_type & (0x1 << GESTURE_MASK_C)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_C, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_C, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_C) == 0)
					gtp_enter_doze(ts);
				break;
			case 'z':
				if ((Gesture_type & (0x1 << GESTURE_MASK_Z)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_Z, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_Z, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_Z) == 0)
					gtp_enter_doze(ts);
				break;
			case 'v':
				if ((Gesture_type & (0x1 << GESTURE_MASK_V)) && (Gesture_type & (0x1 << GESTURE_MASK_Master))) {
					GTP_INFO("Wakeup by gesture(%c), light up the screen!",
						doze_buf[2]);
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_V, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_V, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B  */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_type & (0x1 << GESTURE_MASK_V) == 0)
					gtp_enter_doze(ts);
				break;
			case 0xCC:
				if (Gesture_Dclick[0] == '1') {
					GTP_INFO("Double click to light up the screen!");
					doze_status = DOZE_WAKEUP;
					input_report_key(ts->input_dev, KEY_GESTURE_DOUBLE_CLICK, 1);
					input_sync(ts->input_dev);
					input_report_key(ts->input_dev, KEY_GESTURE_DOUBLE_CLICK, 0);
					input_sync(ts->input_dev);
				}
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);

				if (Gesture_Dclick[0] == '0')
					gtp_enter_doze(ts);
				break;
			default:
				/* clear 0x814B */
				doze_buf[2] = 0x00;
				gtp_i2c_write(i2c_connect_client, doze_buf, 3);
				gtp_enter_doze(ts);
				break;
			}
		}
		if (ts->use_irq) {
			gtp_irq_enable(ts);
		}
		wake_unlock(&ts->wake_lock);
		return;
	}
#endif

	ret = gtp_i2c_read(ts->client, point_data, 12);
	if (ret < 0) {
		GTP_ERROR("I2C transfer error. errno:%d\n ", ret);
		if (ts->use_irq) {
			gtp_irq_enable(ts);
		}
		wake_unlock(&ts->wake_lock);
		return;
	}

	finger = point_data[GTP_ADDR_LENGTH];

	if (finger == 0x00) {
		if (ts->use_irq) {
			gtp_irq_enable(ts);
		}
		wake_unlock(&ts->wake_lock);
		return;
	}

	if ((finger & 0x80) == 0) {
		goto exit_work_func;
	}

	touch_num = finger & 0x0f;
	if (touch_num > GTP_MAX_TOUCH) {
		goto exit_work_func;
	}

	if (touch_num > 1) {
		u8 buf[8 * GTP_MAX_TOUCH] = { (GTP_READ_COOR_ADDR + 10) >> 8,
			(GTP_READ_COOR_ADDR + 10) & 0xff };

		ret = gtp_i2c_read(ts->client, buf, 2 + 8 * (touch_num - 1));
		memcpy(&point_data[12], &buf[2], 8 * (touch_num - 1));
	}

	pre_key = key_value;

	GTP_DEBUG("pre_touch:%02x, finger:%02x.", pre_touch, finger);

	if (touch_num) {
		for (i = 0; i < touch_num; i++) {
			coor_data = &point_data[i * 8 + 3];

			id = coor_data[0] & 0x0F;
			input_x = coor_data[1] | (coor_data[2] << 8);
			input_y = coor_data[3] | (coor_data[4] << 8);
			input_w = coor_data[5] | (coor_data[6] << 8);
			if (Flipcovermode[0] == '1') {
				if ((((input_x-399)*(input_x-399)) + ((input_y-293)*(input_y-293))) <= (240*240) ) {
						gtp_touch_down(ts, id, input_x, input_y, input_w);
					}
			}
			else
			{
				gtp_touch_down(ts, id, input_x, input_y, input_w);
			}
		}
	} else if (pre_touch) {
		GTP_DEBUG("Touch Release!");
		gtp_touch_up(ts, 0);
	}

	pre_touch = touch_num;

	input_sync(ts->input_dev);

 exit_work_func:
	if (!ts->gtp_rawdiff_mode) {
		ret = gtp_i2c_write(ts->client, end_cmd, 3);
		if (ret < 0) {
			GTP_INFO("I2C write end_cmd error!");
		}
	}
	if (ts->use_irq) {
		gtp_irq_enable(ts);
	}
	wake_unlock(&ts->wake_lock);
}

/*******************************************************
Function:
    Timer interrupt service routine for polling mode.
Input:
    timer: timer struct pointer
Output:
    Timer work mode.
    HRTIMER_NORESTART: no restart mode
*********************************************************/
static enum hrtimer_restart goodix_ts_timer_handler(struct hrtimer *timer)
{
	struct goodix_ts_data *ts =
	    container_of(timer, struct goodix_ts_data, timer);

	GTP_DEBUG_FUNC();

	queue_work(goodix_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, (GTP_POLL_TIME + 6) * 1000000),
		      HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

/*******************************************************
Function:
    External interrupt service routine for interrupt mode.
Input:
    irq:  interrupt number.
    dev_id: private data pointer
Output:
    Handle Result.
    IRQ_HANDLED: interrupt handled successfully
*********************************************************/
static irqreturn_t goodix_ts_irq_handler(int irq, void *dev_id)
{
	struct goodix_ts_data *ts = dev_id;

	GTP_DEBUG_FUNC();

	gtp_irq_disable(ts);
	wake_lock(&ts->wake_lock);
	queue_work(goodix_wq, &ts->work);

	return IRQ_HANDLED;
}

/*******************************************************
Function:
    Synchronization.
Input:
    ms: synchronization time in millisecond.
Output:
    None.
*******************************************************/
void gtp_int_sync(s32 ms)
{
	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(ms);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
}

/*******************************************************
Function:
    Reset chip.
Input:
    ms: reset time in millisecond
Output:
    None.
*******************************************************/
void gtp_reset_guitar(struct i2c_client *client, s32 ms)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("Guitar reset");
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);	/* begin select I2C slave addr */
	msleep(ms);		/* T2: > 10ms */
	/* HIGH: 0x28/0x29, LOW: 0xBA/0xBB */
	GTP_GPIO_OUTPUT(GTP_INT_PORT, client->addr == 0x14);

	msleep(2);		/* T3: > 100us */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	/* comment the INPUT setting since we have no external pull high. */
	/* modify sleep time according to Miles' request. */
	msleep(210);		/* T4: > 5ms */
	/*GTP_GPIO_AS_INPUT(GTP_RST_PORT);    end select I2C slave addr */

	gtp_int_sync(50);
#if GTP_ESD_PROTECT
	gtp_init_ext_watchdog(client);
#endif
}

#if GTP_GESTURE_WAKEUP
/*******************************************************
Function:
    Enter doze mode for sliding wakeup.
Input:
    ts: goodix tp private data
Output:
    1: succeed, otherwise failed
*******************************************************/
static s8 gtp_enter_doze(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = { (u8) (GTP_REG_SLEEP >> 8),
		(u8) GTP_REG_SLEEP, 8 };

	GTP_DEBUG_FUNC();

	GTP_DEBUG("Entering gesture mode.");
	while (retry++ < 5) {
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x46;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret < 0) {
			GTP_DEBUG("failed to set doze flag into 0x8046, %d",
				  retry);
			continue;
		}
		i2c_control_buf[0] = 0x80;
		i2c_control_buf[1] = 0x40;
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			doze_status = DOZE_ENABLED;
			GTP_INFO("Gesture mode enabled.");
			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send gesture cmd failed.");
	return ret;
}
#endif
/*******************************************************
Function:
    Enter sleep mode.
Input:
    ts: private data.
Output:
    Executive outcomes.
       1: succeed, otherwise failed.
*******************************************************/
static s8 gtp_enter_sleep(struct goodix_ts_data *ts)
{
	s8 ret = -1;
	s8 retry = 0;
	u8 i2c_control_buf[3] = { (u8) (GTP_REG_SLEEP >> 8),
		(u8) GTP_REG_SLEEP, 5 };

	GTP_DEBUG_FUNC();

	GTP_GPIO_OUTPUT(GTP_INT_PORT, 0);
	msleep(5);

	while (retry++ < 5) {
		ret = gtp_i2c_write(ts->client, i2c_control_buf, 3);
		if (ret > 0) {
			GTP_INFO("GTP enter sleep!");

			return ret;
		}
		msleep(10);
	}
	GTP_ERROR("GTP send sleep cmd failed.");
	return ret;
}

/*******************************************************
Function:
    Wakeup from sleep.
Input:
    ts: private data.
Output:
    Executive outcomes.
    >0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_wakeup_sleep(struct goodix_ts_data *ts)
{
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

#if GTP_POWER_CTRL_SLEEP
	while (retry++ < 5) {
		gtp_reset_guitar(ts->client, 20);

		GTP_INFO("GTP wakeup sleep.");
		return 1;
	}
#else
	while (retry++ < 10) {
#if GTP_GESTURE_WAKEUP
		if ((Gesture_type & (0x1 << GESTURE_MASK_Master)) || (Gesture_Dclick[0] == '1')) {
			if (DOZE_WAKEUP != doze_status) {
				GTP_INFO("Powerkey wakeup.");
			} else {
				GTP_INFO("Gesture wakeup.");
			}
			doze_status = DOZE_DISABLED;
			gtp_irq_disable(ts);
			gtp_reset_guitar(ts->client, 10);
			gtp_irq_enable(ts);
		} else {
			GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
			msleep(5);
		}
#else
		GTP_GPIO_OUTPUT(GTP_INT_PORT, 1);
		msleep(5);
#endif

		ret = gtp_i2c_test(ts->client);
		if (ret > 0) {
			GTP_INFO("GTP wakeup sleep.");

#if (!GTP_GESTURE_WAKEUP)
			{
				gtp_int_sync(25);
#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(ts->client);
#endif
			}
#else
			if (!(Gesture_type & (0x1 << GESTURE_MASK_Master))) {
				gtp_int_sync(25);
	#if GTP_ESD_PROTECT
				gtp_init_ext_watchdog(ts->client);
	#endif
			}
#endif
			return ret;
		}
		gtp_reset_guitar(ts->client, 20);
	}
#endif

	GTP_ERROR("GTP wakeup sleep failed.");
	return ret;
}

#if GTP_DRIVER_SEND_CFG
static s32 gtp_get_info(struct goodix_ts_data *ts)
{
	u8 opr_buf[6] = { 0 };
	s32 ret = 0;

	ts->abs_x_max = GTP_MAX_WIDTH;
	ts->abs_y_max = GTP_MAX_HEIGHT;
	ts->int_trigger_type = GTP_INT_TRIGGER;

	opr_buf[0] = (u8) ((GTP_REG_CONFIG_DATA + 1) >> 8);
	opr_buf[1] = (u8) ((GTP_REG_CONFIG_DATA + 1) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 6);
	if (ret < 0) {
		return FAIL;
	}

	ts->abs_x_max = (opr_buf[3] << 8) + opr_buf[2];
	ts->abs_y_max = (opr_buf[5] << 8) + opr_buf[4];

	opr_buf[0] = (u8) ((GTP_REG_CONFIG_DATA + 6) >> 8);
	opr_buf[1] = (u8) ((GTP_REG_CONFIG_DATA + 6) & 0xFF);

	ret = gtp_i2c_read(ts->client, opr_buf, 3);
	if (ret < 0) {
		return FAIL;
	}
	ts->int_trigger_type = opr_buf[2] & 0x03;

	GTP_INFO("X_MAX = %d, Y_MAX = %d, TRIGGER = 0x%02x",
		 ts->abs_x_max, ts->abs_y_max, ts->int_trigger_type);

	return SUCCESS;
}
#endif

/*******************************************************
Function:
    Initialize gtp.
Input:
    ts: goodix private data
Output:
    Executive outcomes.
    0: succeed, otherwise: failed
*******************************************************/
static s32 gtp_init_panel(struct goodix_ts_data *ts)
{
	s32 ret = -1;

#if GTP_DRIVER_SEND_CFG
	s32 i = 0;
	u8 check_sum = 0;
	u8 opr_buf[16] = { 0 };
	u8 sensor_id = 0;
	u8 retry = 0;

	u8 cfg_info_group1[] = CTP_CFG_GROUP1;
	u8 cfg_info_group2[] = CTP_CFG_GROUP2;
	u8 cfg_info_group3[] = CTP_CFG_GROUP3;
	u8 cfg_info_group4[] = CTP_CFG_GROUP4;
	u8 cfg_info_group5[] = CTP_CFG_GROUP5;
	u8 cfg_info_group6[] = CTP_CFG_GROUP6;
	u8 *send_cfg_buf[] = { cfg_info_group1, cfg_info_group2, cfg_info_group3,
		cfg_info_group4, cfg_info_group5, cfg_info_group6
	};
	u8 cfg_info_len[] = { CFG_GROUP_LEN(cfg_info_group1),
		CFG_GROUP_LEN(cfg_info_group2),
		CFG_GROUP_LEN(cfg_info_group3),
		CFG_GROUP_LEN(cfg_info_group4),
		CFG_GROUP_LEN(cfg_info_group5),
		CFG_GROUP_LEN(cfg_info_group6)
	};

	GTP_DEBUG_FUNC();
	GTP_DEBUG("Config Groups\' Lengths: %d, %d, %d, %d, %d, %d",
		  cfg_info_len[0], cfg_info_len[1], cfg_info_len[2],
		  cfg_info_len[3], cfg_info_len[4], cfg_info_len[5]);

	ret = gtp_i2c_read_dbl_check(ts->client, 0x41E4, opr_buf, 1);
	if (SUCCESS == ret) {
		if (opr_buf[0] != 0xBE) {
			ts->fw_error = 1;
			GTP_ERROR("Firmware error, no config sent!");
			return -1;
		}
	}

	if ((!cfg_info_len[1]) && (!cfg_info_len[2]) &&
	    (!cfg_info_len[3]) && (!cfg_info_len[4]) && (!cfg_info_len[5])) {
		sensor_id = 0;
	} else {
		ret = gtp_i2c_read_dbl_check(ts->client,
			GTP_REG_SENSOR_ID, &sensor_id, 1);
		if (SUCCESS == ret) {
			while ((sensor_id == 0xff) && (retry++ < 3)) {
				msleep(100);
				ret =
				    gtp_i2c_read_dbl_check(ts->client,
							   GTP_REG_SENSOR_ID,
							   &sensor_id, 1);
				GTP_ERROR("GTP sensor_ID read failed time %d.",
					  retry);
			}

			if (sensor_id >= 0x06) {
				GTP_ERROR
				    ("Invalid sensor_id(0x%02X), No Config Sent!",
				     sensor_id);
				ts->pnl_init_error = 1;
				return -1;
			}
		} else {
			GTP_ERROR("Failed to get sensor_id, No config sent!");
			ts->pnl_init_error = 1;
			return -1;
		}

		switch (Gesture_Dclick[2]) {
		case 'm':
			sensor_id = 0;
			break;
		case 'h':
			sensor_id = 1;
			break;
		case 'l':
			sensor_id = 2;
			break;
		default:
			sensor_id = 0;
			break;
		}

		GTP_INFO("Sensor_ID: %d", sensor_id);
	}
	ts->gtp_cfg_len = cfg_info_len[sensor_id];
	GTP_INFO("CTP_CONFIG_GROUP%d used, config length: %d", sensor_id + 1,
		 ts->gtp_cfg_len);

	if (ts->gtp_cfg_len < GTP_CONFIG_MIN_LENGTH) {
		GTP_ERROR
		    ("Config Group%d is INVALID CONFIG GROUP(Len: %d)! NO Config Sent! You need to check you header file CFG_GROUP section!",
		     sensor_id + 1, ts->gtp_cfg_len);
		ts->pnl_init_error = 1;
		return -1;
	}

	ret = gtp_i2c_read_dbl_check(ts->client, GTP_REG_CONFIG_DATA,
		&opr_buf[0], 1);

	if (ret == SUCCESS) {
		GTP_INFO("CFG_GROUP%d Config Version: %d, 0x%02X; IC Config Version: %d, 0x%02X",
			     sensor_id + 1, send_cfg_buf[sensor_id][0],
			     send_cfg_buf[sensor_id][0], opr_buf[0],
			     opr_buf[0]);

		/*  mark by Ue, just follow up F/W internal rule */
		/*  due to sometimes IC internal CFG error and version over 90 */
		/*  then driver always can't write CFG. */
		/* if (opr_buf[0] < 90) */
		if (1) {
			grp_cfg_version = send_cfg_buf[sensor_id][0];	/* backup group config version */
			/* send_cfg_buf[sensor_id][0] = 0x00;  Comment this as Miles' request.*/
			ts->fixed_cfg = 0;
		} else { /* treated as fixed config, not send config */
			GTP_INFO ("Ic fixed config with config version(%d, 0x%02X)",
			     opr_buf[0], opr_buf[0]);
			/* ts->fixed_cfg = 1; */
			/* even over 90, modify to solve the un-clear config RAM area */
			/* caused by leather cover */
			ts->fixed_cfg = 0;
			gtp_get_info(ts);

			/* even over 90, modify to solve the un-clear config RAM area */
			/* caused by leather cover */
			/* return 0; */
		}
	} else {
		GTP_ERROR("Failed to get ic config version!No config sent!");
		return -1;
	}

	memset(&config[GTP_ADDR_LENGTH], 0, GTP_CONFIG_MAX_LENGTH);
	memcpy(&config[GTP_ADDR_LENGTH], send_cfg_buf[sensor_id],
	       ts->gtp_cfg_len);

#if GTP_CUSTOM_CFG
	config[RESOLUTION_LOC] = (u8) GTP_MAX_WIDTH;
	config[RESOLUTION_LOC + 1] = (u8) (GTP_MAX_WIDTH >> 8);
	config[RESOLUTION_LOC + 2] = (u8) GTP_MAX_HEIGHT;
	config[RESOLUTION_LOC + 3] = (u8) (GTP_MAX_HEIGHT >> 8);

	if (GTP_INT_TRIGGER == 0) { /* RISING */
		config[TRIGGER_LOC] &= 0xfe;
	} else if (GTP_INT_TRIGGER == 1) {	/* FALLING */
		config[TRIGGER_LOC] |= 0x01;
	}
#endif				/* GTP_CUSTOM_CFG */

	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
		check_sum += config[i];
	}
	config[ts->gtp_cfg_len] = (~check_sum) + 1;

#else				/* driver not send config */

	ts->gtp_cfg_len = GTP_CONFIG_MAX_LENGTH;
	ret =
	    gtp_i2c_read(ts->client, config, ts->gtp_cfg_len + GTP_ADDR_LENGTH);
	if (ret < 0) {
		GTP_ERROR
		    ("Read Config Failed, Using Default Resolution & INT Trigger!");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
#endif				/* GTP_DRIVER_SEND_CFG */

	if ((ts->abs_x_max == 0) && (ts->abs_y_max == 0)) {
		ts->abs_x_max =
		    (config[RESOLUTION_LOC + 1] << 8) + config[RESOLUTION_LOC];
		ts->abs_y_max =
		    (config[RESOLUTION_LOC + 3] << 8) + config[RESOLUTION_LOC +
							       2];
		ts->int_trigger_type = (config[TRIGGER_LOC]) & 0x03;
	}

#if GTP_DRIVER_SEND_CFG
	ret = gtp_send_cfg(ts->client);
	if (ret < 0) {
		GTP_ERROR("Send config error.");
	}

	/* set config version to CTP_CFG_GROUP, for resume to send config */
	config[GTP_ADDR_LENGTH] = grp_cfg_version;
	check_sum = 0;
	for (i = GTP_ADDR_LENGTH; i < ts->gtp_cfg_len; i++) {
		check_sum += config[i];
	}
	config[ts->gtp_cfg_len] = (~check_sum) + 1;
#endif
	GTP_INFO("X_MAX: %d, Y_MAX: %d, TRIGGER: 0x%02x", ts->abs_x_max,
			 ts->abs_y_max, ts->int_trigger_type);

	msleep(10);
	return 0;
}

static ssize_t
gt91xx_config_read_proc(struct file *file, char __user *page, size_t size,
			loff_t *ppos)
{
	char *ptr = page;
	char temp_data[GTP_CONFIG_MAX_LENGTH + 2] = { 0x80, 0x47 };
	int i;

	if (*ppos) {
		return 0;
	}
	ptr += sprintf(ptr, "==== GT9XX config init value====\n");

	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		ptr += sprintf(ptr, "0x%02X ", config[i + 2]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}

	ptr += sprintf(ptr, "\n");

	ptr += sprintf(ptr, "==== GT9XX config real value====\n");
	gtp_i2c_read(i2c_connect_client, temp_data, GTP_CONFIG_MAX_LENGTH + 2);
	for (i = 0; i < GTP_CONFIG_MAX_LENGTH; i++) {
		ptr += sprintf(ptr, "0x%02X ", temp_data[i + 2]);

		if (i % 8 == 7)
			ptr += sprintf(ptr, "\n");
	}
	*ppos += ptr - page;
	return ptr - page;
}

static ssize_t
gt91xx_config_write_proc(struct file *filp, const char __user *buffer,
			 size_t count, loff_t *off)
{
	s32 ret = 0;

	GTP_DEBUG("write count %d\n", (int)count);

	if (count > GTP_CONFIG_MAX_LENGTH) {
		GTP_ERROR("size not match [%d:%d]\n", GTP_CONFIG_MAX_LENGTH,
			  (int)count);
		return -EFAULT;
	}

	if (copy_from_user(&config[2], buffer, count)) {
		GTP_ERROR("copy from user fail\n");
		return -EFAULT;
	}

	ret = gtp_send_cfg(i2c_connect_client);

	if (ret < 0) {
		GTP_ERROR("send config failed.");
	}

	return count;
}

/*******************************************************
Function:
    Read chip version.
Input:
    client:  i2c device
    version: buffer to keep ic firmware version
Output:
    read operation return.
    2: succeed, otherwise: failed
*******************************************************/
s32 gtp_read_version(struct i2c_client *client, u16 *version)
{
	s32 ret = -1;
	u8 buf[8] = { GTP_REG_VERSION >> 8, GTP_REG_VERSION & 0xff };

	GTP_DEBUG_FUNC();

	ret = gtp_i2c_read(client, buf, sizeof(buf));
	if (ret < 0) {
		GTP_ERROR("GTP read version failed");
		return ret;
	}

	if (version) {
		*version = (buf[7] << 8) | buf[6];
	}
	if (buf[5] == 0x00) {
		GTP_INFO("IC Version: %c%c%c_%02x%02x", buf[2], buf[3], buf[4],
			 buf[7], buf[6]);
	} else {
		GTP_INFO("IC Version: %c%c%c%c_%02x%02x", buf[2], buf[3],
			 buf[4], buf[5], buf[7], buf[6]);
	}

	FW_VERSION[0] = buf[7];
	FW_VERSION[1] = buf[6];

	return ret;
}

/*******************************************************
Function:
    I2c test Function.
Input:
    client:i2c client.
Output:
    Executive outcomes.
    2: succeed, otherwise failed.
*******************************************************/
static s8 gtp_i2c_test(struct i2c_client *client)
{
	u8 test[3] = { GTP_REG_CONFIG_DATA >> 8, GTP_REG_CONFIG_DATA & 0xff };
	u8 retry = 0;
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	while (retry++ < 5) {
		ret = gtp_i2c_read(client, test, 3);
		if (ret > 0) {
			return ret;
		}
		GTP_ERROR("GTP i2c test failed time %d.", retry);
		msleep(10);
	}
	return ret;
}

/*******************************************************
Function:
    Request gpio(INT & RST) ports.
Input:
    ts: private data.
Output:
    Executive outcomes.
    >= 0: succeed, < 0: failed
*******************************************************/
static s8 gtp_request_io_port(struct goodix_ts_data *ts)
{
	s32 ret = 0;

	GTP_DEBUG_FUNC();
	ret = GTP_GPIO_REQUEST(GTP_INT_PORT, "GTP_INT_IRQ");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32) GTP_INT_PORT, ret);
		ret = -ENODEV;
	} else {
		GTP_GPIO_AS_INT(GTP_INT_PORT);
		ts->client->irq = GTP_INT_IRQ;
	}

	ret = GTP_GPIO_REQUEST(GTP_RST_PORT, "GTP_RST_PORT");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d",
			  (s32) GTP_RST_PORT, ret);
		ret = -ENODEV;
	}

	GTP_GPIO_AS_INPUT(GTP_RST_PORT);

	gtp_reset_guitar(ts->client, 20);

	if (ret < 0) {
		GTP_GPIO_FREE(GTP_RST_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);
	}

	return ret;
}

/*******************************************************
Function:
    Request interrupt.
Input:
    ts: private data.
Output:
    Executive outcomes.
    0: succeed, -1: failed.
*******************************************************/
static s8 gtp_request_irq(struct goodix_ts_data *ts)
{
	s32 ret = -1;
	const u8 irq_table[] = GTP_IRQ_TAB;

	GTP_DEBUG_FUNC();
	GTP_DEBUG("INT trigger type:%x", ts->int_trigger_type);

	ret = request_irq(ts->client->irq,
			  goodix_ts_irq_handler,
			  irq_table[ts->int_trigger_type], ts->client->name,
			  ts);
	if (ret) {
		GTP_ERROR("Request IRQ failed!ERRNO:%d.", ret);
		GTP_GPIO_AS_INPUT(GTP_INT_PORT);
		GTP_GPIO_FREE(GTP_INT_PORT);

		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = goodix_ts_timer_handler;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		return -1;
	} else {
		gtp_irq_disable(ts);
		ts->use_irq = 1;
		return 0;
	}
}

/*******************************************************
Function:
    Request input device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
    0: succeed, otherwise: failed.
*******************************************************/
static s8 gtp_request_input_dev(struct goodix_ts_data *ts)
{
	s8 ret = -1;

	GTP_DEBUG_FUNC();

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		GTP_ERROR("Failed to allocate input device.");
		return -ENOMEM;
	}

	ts->input_dev->evbit[0] =
	    BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);

	ts->input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

#if GTP_GESTURE_WAKEUP
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_E);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_C);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(ts->input_dev, EV_KEY, KEY_GESTURE_DOUBLE_CLICK);
#endif

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max,
			     0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

	ts->input_dev->name = goodix_ts_name;
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0xDEAD;
	ts->input_dev->id.product = 0xBEEF;
	ts->input_dev->id.version = 10427;

	ret = input_register_device(ts->input_dev);
	if (ret) {
		GTP_ERROR("Register %s input device failed",
			  ts->input_dev->name);
		return -ENODEV;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = goodix_ts_early_suspend;
	ts->early_suspend.resume = goodix_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	return 0;
}

/*******************************************************
Function:
    Request switch device Function.
Input:
    ts:private data.
Output:
    Executive outcomes.
    0: succeed, otherwise: failed.
*******************************************************/
static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%02X%02X.%d\n", FW_VERSION[0], FW_VERSION[1],
		       grp_cfg_version);
}

static ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", gt_status);
}

static s8 gtp_request_switch_dev(struct goodix_ts_data *ts)
{
	ts->touch_sdev.name = "touch";
	ts->touch_sdev.print_name = touch_switch_name;
	ts->touch_sdev.print_state = touch_switch_state;

	if (switch_dev_register(&ts->touch_sdev) < 0) {
		GTP_ERROR("switch device register failed");
		return -1;
	}

	return 0;
}

/*******************************************************
Function:
	Gesture state sysfs store function
*******************************************************/
ssize_t
sysfs_flipcovermode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	memcpy(Flipcovermode, buf, sizeof(Flipcovermode));

	return count;
}

ssize_t sysfs_dclick_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	memcpy(Gesture_Dclick, buf, sizeof(Gesture_Dclick));

	return count;
}

ssize_t sysfs_gesture_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int i = 0;

	for (i = 0 ; i <= GESTURE_MASK_Count; i++) {
		if (buf[GESTURE_MASK_Count-i] == '0')
			Gesture_type = Gesture_type & ~(0x1 << i);
		else if (buf[GESTURE_MASK_Count-i] == '1')
			Gesture_type = Gesture_type | (0x1 << i);
	}

	return count;
}

/*******************************************************
Function:
    I2c probe.
Input:
    client: i2c device struct.
    id: device id.
Output:
    Executive outcomes.
    0: succeed.
*******************************************************/
static int
goodix_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	s32 ret = -1;
	struct goodix_ts_data *ts;
	u16 version_info;

	GTP_DEBUG_FUNC();

	/*do NOT remove these logs */
	GTP_INFO("GTP Driver Version: %s", GTP_DRIVER_VERSION);
	GTP_INFO("GTP Driver Built@%s, %s", __TIME__, __DATE__);
	GTP_INFO("GTP I2C Address: 0x%02x", client->addr);

	/* tp_pcbid = asustek_get_tp_type(); */
	/* GTP_INFO("GTP tp_pcbid is %d", tp_pcbid); */

	i2c_connect_client = client;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		GTP_ERROR("I2C check functionality failed.");
		gt_status = gt_status & 0x0;
		return -ENODEV;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		GTP_ERROR("Alloc GFP_KERNEL memory failed.");
		gt_status = gt_status & 0x0;
		return -ENOMEM;
	}

	memset(ts, 0, sizeof(*ts));
	INIT_WORK(&ts->work, goodix_ts_work_func);
	ts->client = client;
	spin_lock_init(&ts->irq_lock);	/* 2.6.39 later */
	/* ts->irq_lock = SPIN_LOCK_UNLOCKED;   2.6.39 & before  */
#if GTP_ESD_PROTECT
	ts->clk_tick_cnt = 2 * HZ;	/* HZ: clock ticks in 1 second generated by system */
	GTP_DEBUG("Clock ticks for an esd cycle: %d", ts->clk_tick_cnt);
	spin_lock_init(&ts->esd_lock);
	/* ts->esd_lock = SPIN_LOCK_UNLOCKED; */
#endif
	i2c_set_clientdata(client, ts);

	ts->gtp_rawdiff_mode = 0;

	/* Add GP56 to pull Touch_3P3_EN high */
	ret = GTP_GPIO_REQUEST(56, "Touch_3P3_EN");
	if (ret < 0) {
		GTP_ERROR("Failed to request GPIO:%d, ERRNO:%d", 56, ret);
		gt_status = gt_status & 0x0;
		ret = -ENODEV;
	}
	GTP_GPIO_OUTPUT(56, 1);

	ret = gtp_request_io_port(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request IO port failed.");
		kfree(ts);
		gt_status = gt_status & 0x0;
		return ret;
	}

	ret = gtp_i2c_test(client);
	if (ret < 0) {
		GTP_ERROR("I2C communication ERROR!");
		gt_status = gt_status & 0x0;
	}

	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
		gt_status = gt_status & 0x0;
	}

	ret = gtp_init_panel(ts);
	if (ret < 0) {
		GTP_ERROR("GTP init panel failed.");
		ts->abs_x_max = GTP_MAX_WIDTH;
		ts->abs_y_max = GTP_MAX_HEIGHT;
		ts->int_trigger_type = GTP_INT_TRIGGER;
	}
	/* Create proc file system */
#if USER_IMAGE
	gt91xx_config_proc =
	    proc_create(GT91XX_CONFIG_PROC_FILE, 0660, NULL, &config_proc_ops);
#else
	gt91xx_config_proc =
	    proc_create(GT91XX_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
#endif
	if (gt91xx_config_proc == NULL) {
		GTP_ERROR("create_proc_entry %s failed\n",
			  GT91XX_CONFIG_PROC_FILE);
		gt_status = gt_status & 0x0;
	} else {
		GTP_INFO("create proc entry %s success",
			 GT91XX_CONFIG_PROC_FILE);
	}

	ret = gtp_request_switch_dev(ts);
	if (ret < 0)
		GTP_ERROR("Create switch dev fail");

#if GTP_ESD_PROTECT
	/*modify by update fix 2.2 */
	gtp_esd_switch(client, SWITCH_ON);
#endif

#if GTP_AUTO_UPDATE
	ret = gup_init_update_proc(ts);
	if (ret < 0) {
		GTP_ERROR("Create update thread error.");
	}

	ret = gtp_read_version(client, &version_info);
	if (ret < 0) {
		GTP_ERROR("Read version failed.");
		gt_status = gt_status & 0x0;
	}
#endif

	ret = gtp_request_input_dev(ts);
	if (ret < 0) {
		GTP_ERROR("GTP request input dev failed");
	}

	ret = gtp_request_irq(ts);
	if (ret < 0) {
		GTP_INFO("GTP works in polling mode.");
	} else {
		GTP_INFO("GTP works in interrupt mode.");
	}

	if (ts->use_irq) {
		gtp_irq_enable(ts);
	}

	gtp_test_sysfs_init();

	wake_lock_init(&ts->wake_lock, WAKE_LOCK_SUSPEND, "gesture_wake_lock");

#if GTP_CREATE_WR_NODE
	init_wr_node(client);
#endif

	return 0;
}

/*******************************************************
Function:
    Goodix touchscreen driver release function.
Input:
    client: i2c device struct.
Output:
    Executive outcomes. 0---succeed.
*******************************************************/
static int goodix_ts_remove(struct i2c_client *client)
{
	struct goodix_ts_data *ts = i2c_get_clientdata(client);

	GTP_DEBUG_FUNC();
	gtp_test_sysfs_deinit();

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif

#if GTP_CREATE_WR_NODE
	uninit_wr_node();
#endif

#if GTP_ESD_PROTECT
	destroy_workqueue(gtp_esd_check_workqueue);
#endif

	if (ts) {
		if (ts->use_irq) {
			GTP_GPIO_AS_INPUT(GTP_INT_PORT);
			GTP_GPIO_FREE(GTP_INT_PORT);
			free_irq(client->irq, ts);
		} else {
			hrtimer_cancel(&ts->timer);
		}
	}

	GTP_INFO("GTP driver removing...");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/*******************************************************
Function:
    Early suspend function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_early_suspend(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	GTP_DEBUG_FUNC();

	GTP_INFO("System suspend.");

	ts->gtp_is_suspend = 1;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_OFF);
#endif

#if GTP_GESTURE_WAKEUP
	if ((Gesture_type & (0x1 << GESTURE_MASK_Master)) || (Gesture_Dclick[0] == '1')) {
		ret = gtp_enter_doze(ts);
	} else {
		if (ts->use_irq) {
			gtp_irq_disable(ts);
		} else {
			hrtimer_cancel(&ts->timer);
		}
		ret = gtp_enter_sleep(ts);
	}
#else
	if (ts->use_irq) {
		gtp_irq_disable(ts);
	} else {
		hrtimer_cancel(&ts->timer);
	}
	ret = gtp_enter_sleep(ts);
#endif
	if (ret < 0) {
		GTP_ERROR("GTP early suspend failed.");
	}
	/* to avoid waking up while not sleeping */
	/* delay 48 + 10ms to ensure reliability */
	msleep(58);
}

/*******************************************************
Function:
    Late resume function.
Input:
    h: early_suspend struct.
Output:
    None.
*******************************************************/
static void goodix_ts_late_resume(struct early_suspend *h)
{
	struct goodix_ts_data *ts;
	s8 ret = -1;
	ts = container_of(h, struct goodix_ts_data, early_suspend);

	GTP_DEBUG_FUNC();

	GTP_INFO("System resume.");

	ret = gtp_wakeup_sleep(ts);

#if GTP_GESTURE_WAKEUP
	if ((Gesture_type & (0x1 << GESTURE_MASK_Master)) || (Gesture_Dclick[0] == '1')) {
		doze_status = DOZE_DISABLED;
	}
#endif

	if (ret < 0) {
		GTP_ERROR("GTP later resume failed.");
	}
	gtp_send_cfg(ts->client);

	if (ts->use_irq) {
		gtp_irq_enable(ts);
	} else {
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	ts->gtp_is_suspend = 0;
#if GTP_ESD_PROTECT
	gtp_esd_switch(ts->client, SWITCH_ON);
#endif
}
#endif

#if GTP_ESD_PROTECT
s32 gtp_i2c_read_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msgs[2];
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr = client->addr;
	msgs[0].len = GTP_ADDR_LENGTH;
	msgs[0].buf = &buf[0];
	/* msgs[0].scl_rate = 300 * 1000;    for Rockchip, etc. */

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr = client->addr;
	msgs[1].len = len - GTP_ADDR_LENGTH;
	msgs[1].buf = &buf[GTP_ADDR_LENGTH];
	/* msgs[1].scl_rate = 300 * 1000; */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret == 2)
			break;
		retries++;
	}
	if ((retries >= 5)) {
		GTP_ERROR("I2C Read: 0x%04X, %d bytes failed, errcode: %d!",
			  (((u16) (buf[0] << 8)) | buf[1]), len - 2, ret);
	}
	return ret;
}

s32 gtp_i2c_write_no_rst(struct i2c_client *client, u8 *buf, s32 len)
{
	struct i2c_msg msg;
	s32 ret = -1;
	s32 retries = 0;

	GTP_DEBUG_FUNC();

	msg.flags = !I2C_M_RD;
	msg.addr = client->addr;
	msg.len = len;
	msg.buf = buf;
	/* msg.scl_rate = 300 * 1000;    for Rockchip, etc */

	while (retries < 5) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1)
			break;
		retries++;
	}
	if ((retries >= 5)) {
		GTP_ERROR("I2C Write: 0x%04X, %d bytes failed, errcode: %d!",
			  (((u16) (buf[0] << 8)) | buf[1]), len - 2, ret);
	}
	return ret;
}

/*******************************************************
Function:
    switch on & off esd delayed work
Input:
    client:  i2c device
    on:      SWITCH_ON / SWITCH_OFF
Output:
    void
*********************************************************/
void gtp_esd_switch(struct i2c_client *client, s32 on)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(client);
	spin_lock(&ts->esd_lock);

	if (SWITCH_ON == on) {	/* switch on esd  */
		if (!ts->esd_running) {
			ts->esd_running = 1;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd started");
			queue_delayed_work(gtp_esd_check_workqueue,
					   &gtp_esd_check_work,
					   ts->clk_tick_cnt);
		} else {
			spin_unlock(&ts->esd_lock);
		}
	} else {		/* switch off esd */
		if (ts->esd_running) {
			ts->esd_running = 0;
			spin_unlock(&ts->esd_lock);
			GTP_INFO("Esd cancelled");
			cancel_delayed_work_sync(&gtp_esd_check_work);
		} else {
			spin_unlock(&ts->esd_lock);
		}
	}
}

/*******************************************************
Function:
    Initialize external watchdog for esd protect
Input:
    client:  i2c device.
Output:
    result of i2c write operation.
    1: succeed, otherwise: failed
*********************************************************/
static s32 gtp_init_ext_watchdog(struct i2c_client *client)
{
	u8 opr_buffer[3] = { 0x80, 0x41, 0xAA };
	GTP_DEBUG("[Esd]Init external watchdog");
	return gtp_i2c_write_no_rst(client, opr_buffer, 3);
}

/*******************************************************
Function:
    Esd protect function.
    External watchdog added by meta, 2013/03/07
Input:
    work: delayed work
Output:
    None.
*******************************************************/
static void gtp_esd_check_func(struct work_struct *work)
{
	s32 i;
	s32 ret = -1;
	struct goodix_ts_data *ts = NULL;
	u8 esd_buf[5] = { 0x80, 0x40 };

	GTP_DEBUG_FUNC();

	ts = i2c_get_clientdata(i2c_connect_client);

	/* if (ts->gtp_is_suspend) , mark by Ue for ESD issue */
	if ((ts->gtp_is_suspend) || (ts->enter_update)) {
		GTP_INFO("Esd suspended or IC update firmware!");
		return;
	}

	for (i = 0; i < 3; i++) {
		ret = gtp_i2c_read_no_rst(ts->client, esd_buf, 4);

		GTP_DEBUG("[Esd]0x8040 = 0x%02X, 0x8041 = 0x%02X", esd_buf[2],
			  esd_buf[3]);
		if ((ret < 0)) {
			/* IIC communication problem */
			continue;
		} else {
			if ((esd_buf[2] == 0xAA) || (esd_buf[3] != 0xAA)) {
				/*  IC works abnormally.. */
				u8 chk_buf[4] = { 0x80, 0x40 };

				gtp_i2c_read_no_rst(ts->client, chk_buf, 4);

				GTP_DEBUG
				    ("[Check]0x8040 = 0x%02X, 0x8041 = 0x%02X",
				     chk_buf[2], chk_buf[3]);

				if ((chk_buf[2] == 0xAA)
				    || (chk_buf[3] != 0xAA)) {
					i = 3;
					break;
				} else {
					continue;
				}
			} else {
				/* IC works normally, Write 0x8040 0xAA, feed the dog */
				esd_buf[2] = 0xAA;
				gtp_i2c_write_no_rst(ts->client, esd_buf, 3);
				break;
			}
		}
	}
	if (i >= 3) {
		GTP_ERROR
		    ("IC working abnormally! Process reset guitar.");
			esd_buf[0] = 0x42;
			esd_buf[1] = 0x26;
			esd_buf[2] = 0x01;
			esd_buf[3] = 0x01;
			esd_buf[4] = 0x01;
			gtp_i2c_write_no_rst(ts->client, esd_buf, 5);
			msleep(50);
			gtp_reset_guitar(ts->client, 50);
			msleep(50);
			gtp_send_cfg(ts->client);
	}

	if (!ts->gtp_is_suspend) {
		queue_delayed_work(gtp_esd_check_workqueue, &gtp_esd_check_work,
				   ts->clk_tick_cnt);
	} else {
		GTP_INFO("Esd suspended!");
	}
	return;
}
#endif

static const struct i2c_device_id goodix_ts_id[] = {
	{GTP_I2C_NAME, 0},
	{}
};

static struct i2c_driver goodix_ts_driver = {
	.probe = goodix_ts_probe,
	.remove = goodix_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = goodix_ts_early_suspend,
	.resume = goodix_ts_late_resume,
#endif
	.id_table = goodix_ts_id,
	.driver = {
		   .name = GTP_I2C_NAME,
		   .owner = THIS_MODULE,
		   },
};

/*******************************************************
Function:
    Driver Install function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static int goodix_ts_init(void)
{
	s32 ret;

	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver installing...");
	goodix_wq = create_singlethread_workqueue("goodix_wq");
	if (!goodix_wq) {
		GTP_ERROR("Creat workqueue failed.");
		return -ENOMEM;
	}
#if GTP_ESD_PROTECT
	INIT_DELAYED_WORK(&gtp_esd_check_work, gtp_esd_check_func);
	gtp_esd_check_workqueue = create_workqueue("gtp_esd_check");
#endif

	/*
	   hw_pcbid = asustek_get_hw_rev();
	   printk("HW_PCBIS is %d \n", hw_pcbid);
	 */

	printk(KERN_INFO "[goodix touchscreen] gt9271 init\n");
	ret = i2c_add_driver(&goodix_ts_driver);
	return ret;
}

/*******************************************************
Function:
    Driver uninstall function.
Input:
    None.
Output:
    Executive Outcomes. 0---succeed.
********************************************************/
static void __exit goodix_ts_exit(void)
{
	GTP_DEBUG_FUNC();
	GTP_INFO("GTP driver exited.");
	i2c_del_driver(&goodix_ts_driver);
	if (goodix_wq) {
		destroy_workqueue(goodix_wq);
	}
}

late_initcall(goodix_ts_init);
module_exit(goodix_ts_exit);

MODULE_DESCRIPTION("GTP Series Driver");
MODULE_LICENSE("GPL");
