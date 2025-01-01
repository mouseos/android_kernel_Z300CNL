/* drivers/input/touchscreen/gt9xx_shorttp.c
 *
 * 2010 - 2012 Goodix Technology.
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
 * Version:1.0
 * Author: meta@goodix.com
 * Accomplished Date:2012/10/20
 * Revision record:
 *
 */

#include "gt9xx_openshort.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/segment.h>
#include <linux/string.h>

/* short test */
#define GTP_SHORT_GND
#define GTP_VDD         33	/* 3.3V  */

/* open test  */
u16 max_limit_value = 4418;	/* screen max limit  */
u16 min_limit_value = 2013;	/* screen min limit */
u16 max_limit_key = 1631;	/* key_val max limit */
u16 min_limit_key = 625;	/* key_val min limit */
u16 global_raw_max;
u16 global_raw_min = 65535;

extern s32 gtp_i2c_read(struct i2c_client *, u8 *, s32);
extern s32 gtp_i2c_write(struct i2c_client *, u8 *, s32);
extern void gtp_reset_guitar(struct i2c_client *, s32);
extern s32 gup_enter_update_mode(struct i2c_client *);
extern s32 gup_leave_update_mode(void);
extern s32 gtp_send_cfg(struct i2c_client *client);
extern s32 gtp_read_version(struct i2c_client *client, u16 *version);
extern void gtp_irq_disable(struct goodix_ts_data *ts);
extern void gtp_irq_enable(struct goodix_ts_data *ts);
extern s32 gup_i2c_write(struct i2c_client *client, u8 *buf, s32 len);
extern ssize_t get_touch_status(struct device *dev,
				struct device_attribute *attr, char *buf);
extern ssize_t get_fw_version(struct device *dev, struct device_attribute *attr,
			      char *buf);
extern ssize_t get_cfg_version(struct device *dev,
			       struct device_attribute *attr, char *buf);

extern struct i2c_client *i2c_connect_client;
extern u8 config[GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH];
extern ssize_t sysfs_dclick_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);
extern ssize_t sysfs_gesture_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);
extern ssize_t sysfs_flipcovermode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count);

u8 gt9xx_drv_num = MAX_DRIVER_NUM;	/* default driver and sensor number  */
u8 gt9xx_sen_num = MAX_SENSOR_NUM;
u16 gt9xx_pixel_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
u16 gt9xx_sc_pxl_cnt = MAX_DRIVER_NUM * MAX_SENSOR_NUM;
struct gt9xx_short_info *short_sum;

struct kobject *goodix_debug_kobj;
static s32 sample_set_num = 16;
static u32 default_test_types =
    _MAX_TEST | _MIN_TEST | _KEY_MAX_TEST | _KEY_MIN_TEST;
static u8 rslt_buf_idx;
static s32 *test_rslt_buf;
static struct gt9xx_open_info *touchpad_sum;
static bool irq_enable = true;
static unsigned char INPUT_ACC_DATA[30 * 18 * 5 * 4];

#define _MIN_ERROR_NUM      (sample_set_num	* 9 / 10)

static char *result_lines[200];
static char tmp_info_line[180];
static u8 RsltIndex;

static void append_info_line(void)
{
	if (strlen(tmp_info_line) != 0) {
		result_lines[RsltIndex] =
		    (char *)kzalloc(strlen(tmp_info_line), GFP_KERNEL);
		memcpy(result_lines[RsltIndex], tmp_info_line,
		       strlen(tmp_info_line));
	}
	if (RsltIndex != 199)
		++RsltIndex;
	else {
		kfree(result_lines[RsltIndex]);
	}
}

#define SET_INFO_LINE_INFO(fmt, args...)	do { memset(tmp_info_line, '\0', 90);\
							sprintf(tmp_info_line, "<Sysfs-INFO>"fmt"\n", ##args);\
							GTP_INFO(fmt, ##args);\
							append_info_line();\
						} while (0)

#define SET_INFO_LINE_ERR(fmt, args...)	do { memset(tmp_info_line, '\0', 90);\
						sprintf(tmp_info_line, "<Sysfs-ERROR>"fmt"\n", ##args);\
						GTP_ERROR(fmt, ##args);\
						append_info_line();\
					} while (0)
#define SET_LINE_INFO(fmt, args...)	do { memset(tmp_info_line, '\0', 90);\
						sprintf(tmp_info_line, fmt"\n", ##args);\
						GTP_INFO(fmt, ##args);\
						append_info_line();\
					} while (0)

static u8 cfg_drv_order[MAX_DRIVER_NUM];
static u8 cfg_sen_order[MAX_SENSOR_NUM];

/*
 * Initialize cfg_drv_order and cfg_sen_order, which is used for report short channels
 *
 */

s32 gt9xx_short_parse_cfg(void)
{
	u8 i = 0;
	u8 drv_num = 0, sen_num = 0;

	u8 config[256] = { (u8) (GTP_REG_CONFIG_DATA >> 8),
		(u8) GTP_REG_CONFIG_DATA, 0 };

	if (gtp_i2c_read
	    (i2c_connect_client, config,
	     GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0) {
		SET_INFO_LINE_ERR("Failed to read config!");
		return FAIL;
	}

	drv_num =
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT - GT9_REG_CFG_BEG] &
	     0x1F)
	    +
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 1 - GT9_REG_CFG_BEG]
	     & 0x1F);
	sen_num =
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 2 - GT9_REG_CFG_BEG]
	     & 0x0F)
	    +
	    ((config
	      [GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 2 -
	       GT9_REG_CFG_BEG] >> 4) & 0x0F);

	if (drv_num < MIN_DRIVER_NUM || drv_num > MAX_DRIVER_NUM) {
		GTP_ERROR("driver number error!");
		return FAIL;
	}
	if (sen_num < MIN_SENSOR_NUM || sen_num > MAX_SENSOR_NUM) {
		GTP_ERROR("sensor number error!");
		return FAIL;
	}
	/*  get sensor and driver order  */
	memset(cfg_sen_order, 0xFF, MAX_SENSOR_NUM);
	for (i = 0; i < sen_num; ++i) {
		cfg_sen_order[i] =
		    config[GTP_ADDR_LENGTH + GT9_REG_SEN_ORD - GT9_REG_CFG_BEG +
			   i];
	}

	memset(cfg_drv_order, 0xFF, MAX_DRIVER_NUM);
	for (i = 0; i < drv_num; ++i) {
		cfg_drv_order[i] =
		    config[GTP_ADDR_LENGTH + GT9_REG_DRV_ORD - GT9_REG_CFG_BEG +
			   i];
	}

	return SUCCESS;
}

/*
 * @param:
 *      phy_chnl: ic detected short channel, is_driver: it's driver or not
 * @Return:
 *      0xff: the ic channel is not used, otherwise: the tp short channel
 */
u8 gt9_get_short_tp_chnl(u8 phy_chnl, u8 is_driver)
{
	u8 i = 0;
	if (is_driver) {
		for (i = 0; i < MAX_DRIVER_NUM; ++i) {
			if (cfg_drv_order[i] == phy_chnl) {
				return i;
			} else if (cfg_drv_order[i] == 0xff) {
				return 0xff;
			}
		}
	} else {
		for (i = 0; i < MAX_SENSOR_NUM; ++i) {
			if (cfg_sen_order[i] == phy_chnl) {
				return i;
			} else if (cfg_sen_order[i] == 0xff) {
				return 0xff;
			}
		}
	}
	return 0xff;
}

u8 gt9xx_set_ic_msg(struct i2c_client *client, u16 addr, u8 val)
{
	s32 i = 0;
	u8 msg[3];

	msg[0] = (addr >> 8) & 0xff;
	msg[1] = addr & 0xff;
	msg[2] = val;

	for (i = 0; i < 5; i++) {
		if (gtp_i2c_write(client, msg, GTP_ADDR_LENGTH + 1) > 0) {
			break;
		}
	}

	if (i >= 5) {
		GTP_ERROR("Set data to 0x%02x%02x failed!", msg[0], msg[1]);
		return FAIL;
	}

	return SUCCESS;
}

static s32 gtp_i2c_end_cmd(struct i2c_client *client)
{
	u8 end_cmd[3] = { GTP_READ_COOR_ADDR >> 8,
		GTP_READ_COOR_ADDR & 0xFF, 0 };
	s32 ret = 0;

	ret = gtp_i2c_write(client, end_cmd, 3);
	if (ret < 0) {
		SET_INFO_LINE_INFO("I2C write end_cmd  error!");
	}
	return ret;
}

s32 gtp_parse_config(void)
{
	u8 config[256] = { (u8) (GTP_REG_CONFIG_DATA >> 8),
		(u8) GTP_REG_CONFIG_DATA, 0 };

	if (gtp_i2c_read
	    (i2c_connect_client, config,
	     GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH) <= 0) {
		SET_INFO_LINE_ERR("Failed to read config!");
		return FAIL;
	}
	/* disable hopping  */
	/* Disable hopping config reload by UE
	   if (config[GTP_ADDR_LENGTH + 0x807D - GTP_REG_CONFIG_DATA] & 0x80)
	   {
	   config[GTP_ADDR_LENGTH + 0x807D - GTP_REG_CONFIG_DATA] &= 0x7F;
	   ts = i2c_get_clientdata(i2c_connect_client);
	   for (j = 0; j < (ts->gtp_cfg_len-2); ++j)
	   {
	   chksum += config[GTP_ADDR_LENGTH + j];
	   }
	   config[ts->gtp_cfg_len] = (~chksum) + 1;

	   gup_i2c_write(i2c_connect_client, config, GTP_CONFIG_MAX_LENGTH + GTP_ADDR_LENGTH);
	   }
	 */
	gt9xx_drv_num =
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT - GT9_REG_CFG_BEG] &
	     0x1F)
	    +
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 1 - GT9_REG_CFG_BEG]
	     & 0x1F);
	gt9xx_sen_num =
	    (config[GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 2 - GT9_REG_CFG_BEG]
	     & 0x0F)
	    +
	    ((config
	      [GTP_ADDR_LENGTH + GT9_REG_SEN_DRV_CNT + 2 -
	       GT9_REG_CFG_BEG] >> 4) & 0x0F);

	if (gt9xx_drv_num < MIN_DRIVER_NUM || gt9xx_drv_num > MAX_DRIVER_NUM) {
		SET_INFO_LINE_ERR("driver number error!");
		return FAIL;
	}
	if (gt9xx_sen_num < MIN_SENSOR_NUM || gt9xx_sen_num > MAX_SENSOR_NUM) {
		SET_INFO_LINE_ERR("sensor number error!");
		return FAIL;
	}
	gt9xx_sc_pxl_cnt = gt9xx_pixel_cnt = gt9xx_drv_num * gt9xx_sen_num;

	return SUCCESS;
}

/*
 * Function:
 * 		write one byte to specified register
 * Input:
 * 		reg: the register address
 * 		val: the value to write into
 * Return:
 * 		i2c_write function return
 */
s32 gtp_write_register(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];
	buf[0] = (u8) (reg >> 8);
	buf[1] = (u8) reg;
	buf[2] = val;
	return gtp_i2c_write(client, buf, 3);
}

/*
 * Function:
 * 		read one byte from specified register into buf
 * Input:
 *		reg: the register
 * 		buf: the buffer for one byte
 * Return:
 *		i2c_read function return
 */
s32 gtp_read_register(struct i2c_client *client, u16 reg, u8 *buf)
{
	buf[0] = (u8) (reg >> 8);
	buf[1] = (u8) reg;
	return gtp_i2c_read(client, buf, 3);
}

/*
 * Function:
 * 		burn dsp_short code
 * Input:
 * 		i2c_client
 * Return:
 * 		SUCCESS: burning succeed, FAIL: burning failed
 */
s32 gtp_burn_dsp_short(struct i2c_client *client)
{
	s32 ret = 0;
	u8 *opr_buf;
	u16 i = 0;
	u16 addr = GTP_REG_DSP_SHORT;
	u16 opr_len = 0;
	u16 left = 0;

	GTP_DEBUG("Start writing dsp_short code");
	opr_buf = (u8 *) kmalloc(sizeof(u8) * (2048 + 2), GFP_KERNEL);
	if (!opr_buf) {
		SET_INFO_LINE_ERR
		    ("failed to allocate memory for check buffer!");
		return FAIL;
	}

	left = sizeof(dsp_short);
	while (left > 0) {
		opr_buf[0] = (u8) (addr >> 8);
		opr_buf[1] = (u8) (addr);
		if (left > 2048) {
			opr_len = 2048;
		} else {
			opr_len = left;
		}
		memcpy(&opr_buf[2], &dsp_short[addr - GTP_REG_DSP_SHORT],
		       opr_len);
		ret = gtp_i2c_write(client, opr_buf, 2 + opr_len);
		if (ret < 0) {
			SET_INFO_LINE_ERR("write dsp_short code failed!");
			return FAIL;
		}
		addr += opr_len;
		left -= opr_len;
	}

	/* check code: 0xC000~0xCFFF  */
	GTP_DEBUG("Start checking dsp_short code");
	addr = GTP_REG_DSP_SHORT;
	left = sizeof(dsp_short);
	while (left > 0) {
		opr_buf[0] = (u8) (addr >> 8);
		opr_buf[1] = (u8) (addr);

		msleep(20);

		if (left > 2048) {
			opr_len = 2048;
		} else {
			opr_len = left;
		}

		ret = gtp_i2c_read(client, opr_buf, opr_len + 2);
		if (ret < 0) {
			kfree(opr_buf);
			return FAIL;
		}
		for (i = 0; i < opr_len; ++i) {
			if (opr_buf[i + 2] !=
			    dsp_short[addr - GTP_REG_DSP_SHORT + i]) {
				SET_INFO_LINE_ERR
				    ("check dsp_short code failed!");
				kfree(opr_buf);
				return FAIL;
			}
		}

		addr += opr_len;
		left -= opr_len;
	}
	kfree(opr_buf);
	return SUCCESS;
}

/*
 * Function:
 * 		check the resistor between shortlike channels if less than threshold confirm as short
 * INPUT:
 *		Short like Information struct pointer
 * Returns:
 *		SUCCESS: it's shorted FAIL: otherwise
 */
s32 gtp_short_resist_check(struct gt9xx_short_info *short_node)
{
	s32 short_resist = 0;
	struct gt9xx_short_info *node = short_node;
	u8 master = node->master;
	u8 slave = node->slave;
	u8 chnnl_tx[4] = { GT9_DRV_HEAD | 13, GT9_DRV_HEAD | 28,
		GT9_DRV_HEAD | 29, GT9_DRV_HEAD | 42
	};
	s32 numberator = 0;
	u32 amplifier = 1000;	/* amplify 1000 times to emulate float computing */

	/*  Tx-ABIST & Tx_ABIST */
	if ((((master > chnnl_tx[0]) && (master <= chnnl_tx[1])) &&
	     ((slave > chnnl_tx[0]) && (slave <= chnnl_tx[1]))) ||
	    (((master > chnnl_tx[2]) && (master <= chnnl_tx[3])) &&
	     ((slave > chnnl_tx[2]) && (master <= chnnl_tx[3])))) {
		numberator = node->self_data * 40 * amplifier;
		short_resist = numberator / (node->short_code) - 40 * amplifier;
	}
	/* Receiver is Rx-odd(1,3,5) */
	else if ((node->slave & (GT9_DRV_HEAD | 0x01)) == 0x01) {
		numberator = node->self_data * 60 * amplifier;
		short_resist = numberator / node->short_code - 40 * amplifier;
	} else {
		numberator = node->self_data * 60 * amplifier;
		short_resist = numberator / node->short_code - 60 * amplifier;
	}
	GTP_DEBUG("self_data = %d", node->self_data);
	GTP_DEBUG("master = 0x%x, slave = 0x%x", node->master, node->slave);
	GTP_DEBUG("short_code = %d, short_resist = %d", node->short_code,
		  short_resist);

	if (short_resist < 0) {
		short_resist = 0;
	}

	if (short_resist < (gt900_resistor_threshold * amplifier)) {
		node->impedance = short_resist / amplifier;
		return SUCCESS;
	} else {
		return FAIL;
	}
}

/*
 * Function:
 * 		compute the result, whether there are shorts or not
 * Input:
 * 		i2c_client
 * Return:
 * 		SUCCESS
 */
s32 gtp_compute_rslt(struct i2c_client *client)
{
	u16 short_code;
	u8 i = 0, j = 0;
	u16 result_addr;
	u8 *result_buf;
	u16 *self_data;
	s32 ret = 0;
	u16 data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2;	/* a short data frame length  */
	struct gt9xx_short_info short_node;
	u16 node_idx = 0;	/* short_sum index: 0~GT9_INFO_NODE_MAX */

	u8 tx_short_num = 0;
	u8 rx_short_num = 0;

	u8 master, slave;

	self_data =
	    (u16 *) kmalloc(sizeof(u16) * ((MAX_DRIVER_NUM + MAX_SENSOR_NUM)),
			    GFP_KERNEL);
	result_buf = (u8 *) kmalloc(sizeof(u8) * (data_len + 2), GFP_KERNEL);
	short_sum =
	    (struct gt9xx_short_info *)kmalloc(sizeof(struct gt9xx_short_info) *
					       GT9_INFO_NODE_MAX, GFP_KERNEL);

	if (!self_data || !result_buf || !short_sum) {
		SET_INFO_LINE_ERR("allocate memory for short result failed!");
		return FAIL;
	}
	/*  Get Selfdata */
	result_buf[0] = 0xA4;
	result_buf[1] = 0xA1;
	gtp_i2c_read(client, result_buf, 2 + 144);
	for (i = 0, j = 0; i < 144; i += 2) {
		self_data[j++] =
		    (u16) (result_buf[i] << 8) + (u16) (result_buf[i + 1]);
	}
	GTP_DEBUG("Self Data:");
	GTP_DEBUG_ARRAY(result_buf + 2, 144);

	/*  Get TxShortNum & RxShortNum */
	result_buf[0] = 0x88;
	result_buf[1] = 0x02;
	gtp_i2c_read(client, result_buf, 2 + 2);
	tx_short_num = result_buf[2];
	rx_short_num = result_buf[3];

	GTP_DEBUG("Tx Short Num: %d, Rx Short Num: %d", tx_short_num,
		  rx_short_num);

	result_addr = 0x8860;
	data_len = 3 + (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2 + 2;
	for (i = 0; i < tx_short_num; ++i) {
		result_buf[0] = (u8) (result_addr >> 8);
		result_buf[1] = (u8) (result_addr);
		ret = gtp_i2c_read(client, result_buf, data_len + 2);
		if (ret < 0) {
			SET_INFO_LINE_ERR("read result data failed!");
		}
		GTP_DEBUG("Result Buffer: ");
		GTP_DEBUG_ARRAY(result_buf + 2, data_len);

		short_node.master_is_driver = 1;
		short_node.master = result_buf[2];

		/* Tx - Tx */
		for (j = i + 1; j < MAX_DRIVER_NUM; ++j) {
			short_code =
			    (result_buf[2 + 3 + j * 2] << 8) + result_buf[2 +
									  3 +
									  j *
									  2 +
									  1];
			if (short_code > gt900_short_threshold) {
				short_node.slave_is_driver = 1;
				short_node.slave =
				    ChannelPackage_TX[j] | GT9_DRV_HEAD;
				short_node.self_data = self_data[j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < GT9_INFO_NODE_MAX) {
						short_sum[node_idx++] =
						    short_node;
					}
				}
			}
		}
		/* Tx - Rx */
		for (j = 0; j < MAX_SENSOR_NUM; ++j) {
			short_code =
			    (result_buf[2 + 3 + 84 + j * 2] << 8) +
			    result_buf[2 + 3 + 84 + j * 2 + 1];

			if (short_code > gt900_short_threshold) {
				short_node.slave_is_driver = 0;
				short_node.slave = j | GT9_SEN_HEAD;
				short_node.self_data =
				    self_data[MAX_DRIVER_NUM + j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < GT9_INFO_NODE_MAX) {
						short_sum[node_idx++] =
						    short_node;
					}
				}
			}
		}

		result_addr += data_len;
	}

	result_addr = 0xA0D2;
	data_len = 3 + MAX_SENSOR_NUM * 2 + 2;
	for (i = 0; i < rx_short_num; ++i) {
		result_buf[0] = (u8) (result_addr >> 8);
		result_buf[1] = (u8) (result_addr);
		ret = gtp_i2c_read(client, result_buf, data_len + 2);
		if (ret < 0) {
			SET_INFO_LINE_ERR("read result data failed!");
		}

		GTP_DEBUG("Result Buffer: ");
		GTP_DEBUG_ARRAY(result_buf + 2, data_len);

		short_node.master_is_driver = 0;
		short_node.master = result_buf[2];

		/*  Rx - Rx */
		for (j = 0; j < MAX_SENSOR_NUM; ++j) {
			if ((j == i) || ((j < i) && (j & 0x01) == 0)) {
				continue;
			}
			short_code =
			    (result_buf[2 + 3 + j * 2] << 8) + result_buf[2 +
									  3 +
									  j *
									  2 +
									  1];

			if (short_code > gt900_short_threshold) {
				short_node.slave_is_driver = 0;
				short_node.slave = j | GT9_SEN_HEAD;
				short_node.self_data =
				    self_data[MAX_DRIVER_NUM + j];
				short_node.short_code = short_code;

				ret = gtp_short_resist_check(&short_node);
				if (ret == SUCCESS) {
					if (node_idx < GT9_INFO_NODE_MAX) {
						short_sum[node_idx++] =
						    short_node;
					}
				}
			}
		}

		result_addr += data_len;
	}

	if (node_idx == 0) {
		ret = SUCCESS;
	} else {
		for (i = 0, j = 0; i < node_idx; ++i) {
			if ((short_sum[i].master_is_driver)) {
				if (short_sum[i].master > (26 | GT9_DRV_HEAD)) {
					short_sum[i].master--;
				}
				master =
				    gt9_get_short_tp_chnl(short_sum[i].master -
							  GT9_DRV_HEAD, 1);

			} else {
				master =
				    gt9_get_short_tp_chnl(short_sum[i].master,
							  0);
			}

			if ((short_sum[i].slave_is_driver)) {
				if (short_sum[i].slave > (26 | GT9_DRV_HEAD)) {
					short_sum[i].slave--;
				}
				slave =
				    gt9_get_short_tp_chnl(short_sum[i].slave -
							  GT9_DRV_HEAD, 1);

			} else {
				slave =
				    gt9_get_short_tp_chnl(short_sum[i].slave,
							  0);
			}
			GTP_DEBUG("Orignal Shorted Channels: %s%d, %s%d",
				  (short_sum[i].
				   master_is_driver) ? "Drv" : "Sen", master,
				  (short_sum[i].
				   slave_is_driver) ? "Drv" : "Sen", slave);

			if (master == 255 && slave == 255) {
				GTP_DEBUG("unbonded channel (%d, %d) shorted!",
					  short_sum[i].master,
					  short_sum[i].slave);
				continue;
			} else {
				short_sum[j].slave = slave;
				short_sum[j].master = master;
				short_sum[j].slave_is_driver =
				    short_sum[i].slave_is_driver;
				short_sum[j].master_is_driver =
				    short_sum[i].master_is_driver;
				short_sum[j].impedance = short_sum[i].impedance;
				short_sum[j].self_data = short_sum[i].self_data;
				short_sum[j].short_code =
				    short_sum[i].short_code;
				++j;
			}
		}
		node_idx = j;
		if (node_idx == 0) {
			ret = SUCCESS;
		} else {
			for (i = 0; i < node_idx; ++i) {
				SET_INFO_LINE_INFO
				    ("  %s%02d & %s%02d Shorted! (R = %dKOhm)",
				     (short_sum[i].
				      master_is_driver) ? "Drv" : "Sen",
				     short_sum[i].master,
				     (short_sum[i].
				      slave_is_driver) ? "Drv" : "Sen",
				     short_sum[i].slave,
				     short_sum[i].impedance);
			}
			ret = FAIL;
		}
	}
	kfree(self_data);
	kfree(short_sum);
	kfree(result_buf);
	return ret;
}

s32 gt9_test_gnd_vdd_short(struct i2c_client *client)
{

	u8 *data;
	s32 ret = 0;
	s32 i = 0;
	u16 len = (MAX_DRIVER_NUM + MAX_SENSOR_NUM) * 2;
	u16 short_code = 0;
	s32 r = -1;
	u32 short_res = 0;
	u16 amplifier = 1000;

	data = (u8 *) kmalloc(sizeof(u8) * (len + 2), GFP_KERNEL);
	if (NULL == data) {
		SET_INFO_LINE_ERR
		    ("failed to allocate memory for gnd vdd test data buffer");
		return FAIL;
	}

	data[0] = 0xA5;
	data[1] = 0x31;
	gtp_i2c_read(client, data, 2 + len);

	GTP_DEBUG_ARRAY(data + 2, len);
	ret = SUCCESS;
	for (i = 0; i < len; i += 2) {
		short_code = (data[2 + i] << 8) + (data[2 + i + 1]);
		if (short_code == 0) {
			continue;
		}
		if ((short_code & 0x8000) == 0)	{ /* short with GND */
#ifdef GTP_SHORT_GND
			r = 5266285 * 10 / (short_code & (~0x8000)) -
			    40 * amplifier;
#endif
		} else {	/* short with VDD */
#ifdef GTP_VDD
			r = 40 * 9 * 1024 * (100 * GTP_VDD -
					     900) / ((short_code & (~0x8000)) *
						     7) - 40 * 1000;
			GTP_DEBUG("vdd short_code: %d", short_code & (~0x8000));
#endif
		}
		GTP_DEBUG("resistor: %d, short_code: %d", r, short_code);

		short_res = (r >= 0) ? r : 0xFFFF;
		if (short_res == 0xFFFF) {
		} else {
			if (short_res <
			    (gt900_gnd_resistor_threshold * amplifier)) {
				if (i < MAX_DRIVER_NUM * 2) { /* driver */
					SET_INFO_LINE_INFO
					    ("  Drv%02d & GND/VDD Shorted! (R = %dKOhm)",
					     ChannelPackage_TX[i / 2],
					     short_res / amplifier);
				} else {
					SET_INFO_LINE_INFO
					    ("  Sen%02d & GND/VDD Shorted! (R = %dKOhm)",
					     (i / 2) - MAX_DRIVER_NUM,
					     short_res / amplifier);
				}
				ret = FAIL;
			}
		}
	}
	return ret;
}

/*
 * leave short test
 */
void gt9xx_leave_short_test(struct i2c_client *client)
{
	/* boot from rom and download code from flash to ram */
	gtp_write_register(client, _rRW_MISCTL__BOOT_CTL_, 0x99);
	gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x08);

	gtp_reset_guitar(client, 20);
	msleep(100);

	gtp_send_cfg(client);
	SET_INFO_LINE_INFO("");
	SET_INFO_LINE_INFO("---gtp short test end---");
}

/*
 * Function:
 *		gt9 series ic short test function
 * Input:
 * 		I2c_client, i2c device
 * Return:
 * 		SUCCESS: test succeed, FAIL: test failed
 */
s32 gt9xx_short_test(struct i2c_client *client)
{
	s32 ret = 0;
	s32 ret2 = 0;
	u8 i = 0;
	u8 opr_buf[60] = { 0 };
	u8 retry = 0;
	u8 drv_sen_chksum = 0;
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);
	disable_irq(ts->client->irq);
#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 1;	/* suspend esd */
#endif
	/* step 1: reset guitar, delay 1ms,  hang up ss51 and dsp */
	SET_INFO_LINE_INFO("---gtp short test---");
	SET_INFO_LINE_INFO("Step 1: reset guitar, hang up ss51 dsp");

	gt9xx_short_parse_cfg();

	/* RST output low last at least 2ms */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);
	msleep(2);

	/* select I2C slave addr,INT:0--0xBA;1--0x28. */
	GTP_GPIO_OUTPUT(GTP_INT_PORT, (client->addr == 0x14));
	msleep(2);

	/* RST output high reset guitar */
	GTP_GPIO_OUTPUT(GTP_RST_PORT, 1);

	while (retry++ < 200) {
		/* Hold ss51 & dsp */
		ret = gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x0C);
		if (ret <= 0) {
			GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
			gtp_reset_guitar(client, 10);
			continue;
		}
		/* Confirm hold */
		ret =
		    gtp_read_register(client, _rRW_MISCTL__SWRST_B0_, opr_buf);
		if (ret <= 0) {
			GTP_DEBUG("Hold ss51 & dsp I2C error,retry:%d", retry);
			gtp_reset_guitar(client, 10);
			continue;
		}
		if (0x0C == opr_buf[GTP_ADDR_LENGTH]) {
			GTP_DEBUG("Hold ss51 & dsp confirm SUCCESS");
			break;
		}
		GTP_DEBUG("Hold ss51 & dsp confirm 0x4180 failed,value:%d",
			  opr_buf[GTP_ADDR_LENGTH]);
	}
	if (retry >= 200) {
		GTP_ERROR("Enter update Hold ss51 failed.");
		return FAIL;
	}
	/* DSP_CK and DSP_ALU_CK PowerOn */
	gtp_write_register(client, 0x4010, 0x00);
	SET_INFO_LINE_INFO("Enter short test mode SUCCESS.");

	/* step2: burn dsp_short code */
	SET_INFO_LINE_INFO("step 2: burn dsp_short code");
	gtp_write_register(client, _bRW_MISCTL__TMR0_EN, 0x00);	/* clear watchdog */
	gtp_write_register(client, _bRW_MISCTL__CACHE_EN, 0x00);	/* clear cache */
	gtp_write_register(client, _rRW_MISCTL__BOOTCTL_B0_, 0x02);	/* boot from sram */
	gtp_write_register(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01);	/* reset software */
	gtp_write_register(client, _bRW_MISCTL__SRAM_BANK, 0x00);	/* select bank 0 */
	gtp_write_register(client, _bRW_MISCTL__MEM_CD_EN, 0x01);	/* allow AHB bus accessing code sram */

	/* ---: burn dsp_short code */
	ret = gtp_burn_dsp_short(client);
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("step2: burn dsp_short failed!");
		goto short_test_exit;
	}
	/* step3: run dsp_short, read results */
	SET_INFO_LINE_INFO("step 3: run dsp_short code, confirm it's runnin'");
	gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x00);	/* clear dsp_short running flag */
	gtp_write_register(client, _rRW_MISCTL__BOOT_OPT_B0_, 0x03);	/* set scramble */

	ret = gt9xx_set_ic_msg(client, _bWO_MISCTL__CPU_SWRST_PULSE, 0x01);
	gtp_write_register(client, _rRW_MISCTL__SWRST_B0_, 0x08);	/* release dsp */

	msleep(80);
	/* confirm dsp is running */
	i = 0;
	while (1) {
		opr_buf[2] = 0x00;
		gtp_read_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG,
				  opr_buf);
		if (opr_buf[2] == 0xAA) {
			break;
		}
		++i;
		if (i >= 8) {
			SET_INFO_LINE_ERR("step 3: dsp is not running!");
			goto short_test_exit;
		}
		msleep(10);
	}
	/* step4: host configure ic, get test result */
	SET_INFO_LINE_INFO("Step 4: host config ic, get test result");
	/* Short Threshold */
	GTP_DEBUG(" Short Threshold: %d", gt900_short_threshold);
	opr_buf[0] = (u8) (GTP_REG_SHORT_TH >> 8);
	opr_buf[1] = (u8) GTP_REG_SHORT_TH;
	opr_buf[2] = (u8) (gt900_short_threshold >> 8);
	opr_buf[3] = (u8) (gt900_short_threshold & 0xFF);
	gtp_i2c_write(client, opr_buf, 4);

	/* ADC Read Delay */
	GTP_DEBUG(" ADC Read Delay: %d", gt900_adc_read_delay);
	opr_buf[1] += 2;
	opr_buf[2] = (u8) (gt900_adc_read_delay >> 8);
	opr_buf[3] = (u8) (gt900_adc_read_delay & 0xFF);
	gtp_i2c_write(client, opr_buf, 4);

	/* DiffCode Short Threshold */
	GTP_DEBUG(" DiffCode Short Threshold: %d",
		  gt900_diffcode_short_threshold);
	opr_buf[0] = 0x88;
	opr_buf[1] = 0x51;
	opr_buf[2] = (u8) (gt900_diffcode_short_threshold >> 8);
	opr_buf[3] = (u8) (gt900_diffcode_short_threshold & 0xFF);
	gtp_i2c_write(client, opr_buf, 4);

	/* Config Driver & Sensor Order */
#if GTP_DEBUG_ON
	printk("<<-GTP-DEBUG->>: Driver Map:\n");
	printk("IC Driver:");
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		printk(" %d", cfg_drv_order[i]);
	}
	printk("\n");
	printk("TP Driver:");
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		printk(" %d", i);
	}
	printk("\n");

	printk("<<-GTP-DEBUG->>: Sensor Map:\n");
	printk("IC Sensor:");
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		printk(" %d", cfg_sen_order[i]);
	}
	printk("\n");
	printk("TP Sensor:");
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		printk(" %d", i);
	}
	printk("\n");
#endif

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x08;
	for (i = 0; i < MAX_DRIVER_NUM; ++i) {
		opr_buf[2 + i] = cfg_drv_order[i];
		drv_sen_chksum += cfg_drv_order[i];
	}
	gtp_i2c_write(client, opr_buf, MAX_DRIVER_NUM + 2);

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x32;
	for (i = 0; i < MAX_SENSOR_NUM; ++i) {
		opr_buf[2 + i] = cfg_sen_order[i];
		drv_sen_chksum += cfg_sen_order[i];
	}
	gtp_i2c_write(client, opr_buf, MAX_SENSOR_NUM + 2);

	opr_buf[0] = 0x88;
	opr_buf[1] = 0x50;
	opr_buf[2] = drv_sen_chksum;
	gtp_i2c_write(client, opr_buf, 2 + 1);

	/* clear waiting flag, run dsp */
	gtp_write_register(client, _rRW_MISCTL__SHORT_BOOT_FLAG, 0x04);

	/* inquirying test status until it's okay */
	for (i = 0;; ++i) {
		gtp_read_register(client, 0x8800, opr_buf);
		if (opr_buf[2] == 0x88) {
			break;
		}
		msleep(50);
		if (i > 100) {
			SET_INFO_LINE_ERR
			    ("step 4: inquiry test status timeout!");
			goto short_test_exit;
		}
	}

	/* step 5: compute the result */
	/* short flag:
	   bit0: Rx & Rx
	   bit1: Tx & Tx
	   bit2: Tx & Rx
	   bit3: Tx/Rx & GND/VDD
	 */
	gtp_read_register(client, 0x8801, opr_buf);
	GTP_DEBUG("short_flag = 0x%x", opr_buf[2]);
	SET_INFO_LINE_INFO("");
	SET_INFO_LINE_INFO("Short Test Result:");
	if ((opr_buf[2] & 0x0f) == 0) {
		SET_INFO_LINE_INFO("  PASS!");
		ret = SUCCESS;
	} else {
		if ((opr_buf[2] & 0x08) == 0x08) {
			ret2 = gt9_test_gnd_vdd_short(client);

		}
		ret = gtp_compute_rslt(client);
		if (ret == SUCCESS && ret2 == SUCCESS) {
			SET_INFO_LINE_INFO("  PASS!");
		}
	}
	gt9xx_leave_short_test(client);
	enable_irq(ts->client->irq);

#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 0;	/* resume esd */
#endif
	return ret;

 short_test_exit:
	gt9xx_leave_short_test(client);
	/* gtp_irq_enable(ts); */
	enable_irq(ts->client->irq);
#if GTP_ESD_PROTECT
	ts->gtp_is_suspend = 0;	/* resume esd */
#endif
	return FAIL;
}

u32 endian_mode(void)
{
	union {
		s32 i;
		s8 c;
	} endian;

	endian.i = 1;

	if (1 == endian.c) {
		return MYBIG_ENDIAN;
	} else {
		return MYLITLE_ENDIAN;
	}
}

/*
*********************************************************************************************************
* Function:
*	send read rawdata cmd
* Input:
*	i2c_client* client: i2c device
* Return:
* 	SUCCESS: send process succeed, FAIL: failed
*********************************************************************************************************
*/
s32 gt9_read_raw_cmd(struct i2c_client *client)
{
	u8 raw_cmd[3] = { (u8) (GTP_REG_READ_RAW >> 8),
		(u8) GTP_REG_READ_RAW, 0x01 };
	s32 ret = -1;

	ret = gtp_i2c_write(client, raw_cmd, 3);
	if (ret <= 0) {
		SET_INFO_LINE_ERR("i2c write failed.");
		return FAIL;
	}
	msleep(10);
	return SUCCESS;
}

s32 gt9_read_coor_cmd(struct i2c_client *client)
{
	u8 raw_cmd[3] = { (u8) (GTP_REG_READ_RAW >> 8),
		(u8) GTP_REG_READ_RAW, 0x0 };
	s32 ret = -1;

	ret = gtp_i2c_write(client, raw_cmd, 3);
	if (ret < 0) {
		SET_INFO_LINE_ERR("i2c write coor cmd failed!");
		return FAIL;
	}
	msleep(10);
	return SUCCESS;
}

/*
*********************************************************************************************************
* Function:
*	read rawdata from ic registers
* Input:
*	u16* data: rawdata buffer
* 	i2c_client* client: i2c device
* Return:
* 	SUCCESS: read process succeed, FAIL:  failed
*********************************************************************************************************
*/
s32 gtp_read_rawdata(struct i2c_client *client, u16 *data)
{
	s32 ret = -1;
	u16 retry = 0;
	u8 read_state[3] = { (u8) (GTP_REG_RAW_READY >> 8),
		(u8) GTP_REG_RAW_READY, 0 };
	u16 i = 0, j = 0, a = 0, b = 0;
	u8 *read_rawbuf;
	u16 *reverse_data;
	u8 tail, head;

	read_rawbuf =
	    (u8 *) kmalloc(sizeof(u8) * (gt9xx_pixel_cnt * 2 + GTP_ADDR_LENGTH),
			   GFP_KERNEL);
	reverse_data =
	    (u16 *) kmalloc(sizeof(u16) * gt9xx_pixel_cnt, GFP_KERNEL);

	if ((NULL == read_rawbuf) || (NULL == reverse_data)) {
		SET_INFO_LINE_ERR("failed to allocate for read_rawbuf");
		return FAIL;
	}
	read_rawbuf[0] = (u8) (GTP_REG_RAW_DATA >> 8);
	read_rawbuf[1] = (u8) (GTP_REG_RAW_DATA);

	if (data == NULL) {
		SET_INFO_LINE_ERR("Invalid raw buffer.");
		goto have_error;
	}

 retryagain:
	msleep(10);
	while (retry++ < GTP_WAIT_RAW_MAX_TIMES) {
		ret = gtp_i2c_read(client, read_state, 3);
		if (ret <= 0) {
			SET_INFO_LINE_ERR("i2c read failed.return: %d", ret);
			continue;
		}
		if (read_state[GTP_ADDR_LENGTH] == 0x80) {
			GTP_DEBUG("Raw data is ready.");
			break;
		}
		if ((retry % 20) == 0)
			GTP_DEBUG("read_state[2] = 0x%x",
				  read_state[GTP_ADDR_LENGTH]);
		msleep(5);
	}
	if (retry >= GTP_WAIT_RAW_MAX_TIMES) {
		SET_INFO_LINE_ERR("Wait raw data ready timeout.");
		gtp_reset_guitar(client, 20);
		ret = gt9_read_raw_cmd(client);
		retry = 0;
		if (ret == FAIL) {
			SET_INFO_LINE_ERR("Send Read Rawdata Cmd failed!");
			goto have_error;
		}
		goto retryagain;
	}

	ret =
	    gtp_i2c_read(client, read_rawbuf,
			 GTP_ADDR_LENGTH +
			 ((gt9xx_drv_num * gt9xx_sen_num) * 2));
	if (ret <= 0) {
		SET_INFO_LINE_ERR("i2c read rawdata failed.");
		goto have_error;
	}
	gtp_i2c_end_cmd(client);	/* clear buffer state */

	if (endian_mode() == MYBIG_ENDIAN) {
		head = 0;
		tail = 1;
		GTP_DEBUG("Big Endian.");
	} else {
		head = 1;
		tail = 0;
		GTP_DEBUG("Little Endian.");
	}

	for (i = 0, j = 0; i < ((gt9xx_drv_num * gt9xx_sen_num) * 2); i += 2) {
		reverse_data[i / 2] =
		    (u16) (read_rawbuf[i + head + GTP_ADDR_LENGTH] << 8) +
		    (u16) read_rawbuf[GTP_ADDR_LENGTH + i + tail];
#if GTP_DEBUG_ARRAY_ON
		printk("%d ", reverse_data[i / 2]);
		++j;
		if ((j % 10) == 0)
			printk("\n");
#endif
	}

	i = 0;
	for (a = 0; a < gt9xx_drv_num; a++) {
		for (b = 0; b < gt9xx_drv_num * gt9xx_sen_num;
		     b += gt9xx_sen_num, i++) {
			data[i] = reverse_data[a + b];
		}

		if (i == gt9xx_drv_num * gt9xx_sen_num) {
			break;
		}
	}

	kfree(read_rawbuf);
	kfree(reverse_data);
	return SUCCESS;

 have_error:
	kfree(read_rawbuf);
	kfree(reverse_data);
	return FAIL;
}

/*
*********************************************************************************************************
* Function:
*	rawdata test initilization function
* Input:
*	u32 check_types: test items
*********************************************************************************************************
*/
static int gtp_raw_test_init(u32 check_types)
{
	u16 i = 0;

	test_rslt_buf =
	    (s32 *) kmalloc(sizeof(s32) * sample_set_num, GFP_ATOMIC);
	touchpad_sum =
	    (struct gt9xx_open_info *)kmalloc(sizeof(struct gt9xx_open_info) *
					      5 * _BEYOND_REC_MAX, GFP_ATOMIC);
	if (NULL == test_rslt_buf || touchpad_sum == NULL) {
		SET_INFO_LINE_ERR("Test result buffer allocate failed!");
	}
	memset(touchpad_sum, 0,
	       sizeof(struct gt9xx_open_info) * 5 * _BEYOND_REC_MAX);
	for (i = 0; i < gt9xx_drv_num * gt9xx_sen_num; i++) {
		if (i < sample_set_num) {
			test_rslt_buf[i] = _CHANNEL_PASS;
		}
	}

	return 0;
}

/*
*********************************************************************************************************
* Function:
*	touchscreen rawdata min limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static int gtp_raw_min_test(u16 *raw_buf)
{
	u16 i, j = 0;
	u8 driver, sensor;
	u8 sum_base = 1 * _BEYOND_REC_MAX;
	u8 new_flag = 0;
	global_raw_min = 65535;

	for (i = 0; i < gt9xx_sc_pxl_cnt; i++) {
		if (raw_buf[i] < min_limit_value) {
			test_rslt_buf[rslt_buf_idx] |= _BEYOND_MIN_LIMIT;
			driver = (i / gt9xx_sen_num) + 1;
			sensor = (i % gt9xx_sen_num) + 1;
			new_flag = 0;
			for (j = sum_base; j < (sum_base + _BEYOND_REC_MAX);
			     ++j) {
				if (touchpad_sum[j].beyond_type == 0) {
					new_flag = 1;
					break;
				}
				if ((driver == touchpad_sum[j].driver)
				    && (sensor == touchpad_sum[j].sensor)) {
					touchpad_sum[j].times++;
					new_flag = 0;
					break;
				}
			}
			if (new_flag) {	/* new one */
				touchpad_sum[j].driver = driver;
				touchpad_sum[j].sensor = sensor;
				touchpad_sum[j].beyond_type |=
				    _BEYOND_MIN_LIMIT;
				touchpad_sum[j].raw_val = raw_buf[i];
				touchpad_sum[j].times = 1;
			} else {
				continue;
			}
			GTP_DEBUG("[%d, %d]rawdata: %d, raw min limit: %d",
				  driver, sensor, raw_buf[i], min_limit_value);
		}

		if (raw_buf[i] < global_raw_min)
			global_raw_min = raw_buf[i];
	}

	return 0;
}

/*
*********************************************************************************************************
* Function:
*	touchscreen rawdata max limit test
* Input:
*	u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static void gtp_raw_max_test(u16 *raw_buf)
{
	u16 i, j;
	u8 driver, sensor;
	u8 sum_base = 0 * _BEYOND_REC_MAX;
	u8 new_flag = 0;
	global_raw_max = 0;

	for (i = 0; i < gt9xx_sc_pxl_cnt; i++) {
		if (raw_buf[i] > max_limit_value) {
			test_rslt_buf[rslt_buf_idx] |= _BEYOND_MAX_LIMIT;
			driver = (i / gt9xx_sen_num) + 1;
			sensor = (i % gt9xx_sen_num) + 1;
			new_flag = 0;
			for (j = sum_base; j < (sum_base + _BEYOND_REC_MAX);
			     ++j) {
				if (touchpad_sum[j].beyond_type == 0) {
					new_flag = 1;
					break;
				}
				if ((driver == touchpad_sum[j].driver)
				    && (sensor == touchpad_sum[j].sensor)) {
					touchpad_sum[j].times++;
					new_flag = 0;
					break;
				}
			}
			if (new_flag) {	/* new one */
				touchpad_sum[j].driver = driver;
				touchpad_sum[j].sensor = sensor;
				touchpad_sum[j].beyond_type |=
				    _BEYOND_MAX_LIMIT;
				touchpad_sum[j].raw_val = raw_buf[i];
				touchpad_sum[j].times = 1;
			} else {
				continue;
			}
			GTP_DEBUG("[%d, %d]rawdata: %d, raw max limit: %d",
				  driver, sensor, raw_buf[i], max_limit_value);
		}

		if (raw_buf[i] > global_raw_max)
			global_raw_max = raw_buf[i];
	}
}

/*
*********************************************************************************************************
* Function:
*   touchscreen rawdata acc limit test
* Input:
*   u16* raw_buf: rawdata buffer
*********************************************************************************************************
*/
static void gtp_raw_acc_test(u16 *raw_buf)
{
	u16 i, j;
	u8 driver, sensor;
	u8 sum_base = 4 * _BEYOND_REC_MAX;
	u8 new_flag = 0;

	u16 four_data[4], acc_data;
	u32 max_acc_data, min_acc_data;

	for (i = 0; i < gt9xx_sc_pxl_cnt; i++) {
		if (raw_buf[i] == 0) {
			acc_data = 0xFFFF;
		} else {
			/* get up/down/left/right data */
			four_data[0] = ((i < gt9xx_drv_num) ? raw_buf[i] : raw_buf[i - gt9xx_drv_num]);	/* get  up */
			four_data[1] = ((i >= (gt9xx_sc_pxl_cnt - gt9xx_drv_num)) ? raw_buf[i] : raw_buf[i + gt9xx_drv_num]);	/* get down */
			four_data[2] = ((((i + 1) % gt9xx_drv_num) == 0) ? raw_buf[i] : raw_buf[i + 1]);	/* right */
			four_data[3] = (((i % gt9xx_drv_num) == 0) ? raw_buf[i] : raw_buf[i - 1]);	/* left */

			/* find max, min */
			(four_data[0] > four_data[1]) ? (max_acc_data =
							 four_data[0],
							 min_acc_data =
							 four_data[1])
			    : (max_acc_data = four_data[1], min_acc_data =
			       four_data[0]);
			(max_acc_data < four_data[2]) ? (max_acc_data =
							 four_data[2]) : 0;
			(max_acc_data < four_data[3]) ? (max_acc_data =
							 four_data[3]) : 0;
			(min_acc_data > four_data[2]) ? (min_acc_data =
							 four_data[2]) : 0;
			(min_acc_data > four_data[3]) ? (min_acc_data =
							 four_data[3]) : 0;

			/* calculate max_, min */
			max_acc_data =
			    (((raw_buf[i] >
			       max_acc_data) ? (raw_buf[i] -
						max_acc_data) : (max_acc_data -
								 raw_buf[i])) *
			     1000) / raw_buf[i];
			min_acc_data =
			    (((raw_buf[i] >
			       min_acc_data) ? (raw_buf[i] -
						min_acc_data) : (min_acc_data -
								 raw_buf[i])) *
			     1000) / raw_buf[i];

			/* sel real max */
			acc_data =
			    (max_acc_data >
			     min_acc_data) ? max_acc_data : min_acc_data;
		}

		/* compare */
		if (acc_data > raw_acc_data[i * ACC_DATA_WIDTH + ACC_DATA_POS]) {
			test_rslt_buf[rslt_buf_idx] |= _BEYOND_ACC_LIMIT;
			driver = (i / gt9xx_sen_num);
			sensor = (i % gt9xx_sen_num);
			new_flag = 0;

			for (j = sum_base; j < (sum_base + _BEYOND_REC_MAX);
			     ++j) {
				if (touchpad_sum[j].beyond_type == 0) {
					new_flag = 1;
					break;
				}

				if ((driver == touchpad_sum[j].driver)
				    && (sensor == touchpad_sum[j].sensor)) {
					touchpad_sum[j].times++;
					new_flag = 0;
					break;
				}
			}

			if (new_flag) {	/*  new one */
				touchpad_sum[j].driver = driver;
				touchpad_sum[j].sensor = sensor;
				touchpad_sum[j].beyond_type |=
				    _BEYOND_ACC_LIMIT;
				touchpad_sum[j].raw_val = raw_buf[i];
				touchpad_sum[j].times = 1;
				GTP_DEBUG
				    ("[%d, %d]acc_data: %d, acc max limit: %d",
				     driver, sensor, acc_data,
				     raw_acc_data[i * ACC_DATA_WIDTH +
						  ACC_DATA_POS]);
			} else {
				continue;
			}
		}
	}

	return;
}

/*
*********************************************************************************************************
* Function:
*	analyse rawdata retrived from ic registers
* Input:
*	u16 *raw_buf, buffer for rawdata,
*   u32 check_types, test items
* Return:
*	SUCCESS: test process succeed, FAIL: failed
*********************************************************************************************************
*/
static u32 gtp_raw_test(u16 *raw_buf, u32 check_types)
{
	if (raw_buf == NULL) {
		GTP_DEBUG("Invalid raw buffer pointer!");
		return FAIL;
	}
	if (0 == check_types) {
		check_types = default_test_types;
	}

	if (check_types & _MAX_TEST) {
		gtp_raw_max_test(raw_buf);
	}

	if (check_types & _MIN_TEST) {
		gtp_raw_min_test(raw_buf);
	}

	if (check_types & _ACC_TEST) {
		gtp_raw_acc_test(raw_buf);
	}

	return SUCCESS;
}

/*
====================================================================================================
* Function:
* 	output the test result
* Return:
* 	return the result. if result == 0, the TP is ok, otherwise list the beyonds
====================================================================================================
*/

static s32 gtp_get_test_result(void)
{
	u16 i = 0, j = 0;
	u16 beyond_max_num = 0;	/* beyond max limit test times */
	u16 beyond_min_num = 0;	/* beyond min limit test times */
	u16 beyond_acc_num = 0;	/* beyond acc limit test times */
	s32 result = _CHANNEL_PASS;

#if GTP_DEBUG_ON
	for (i = 0; i < 5 * _BEYOND_REC_MAX; ++i) {
		printk("(%2d, %2d)[%2d] ", touchpad_sum[i].driver,
		       touchpad_sum[i].sensor, touchpad_sum[i].times);
		if (i && ((i + 1) % 5 == 0)) {
			printk("\n");
		}
	}
	printk("\n");
#endif

	for (i = 0; i < sample_set_num; ++i) {
		if (test_rslt_buf[i] & _BEYOND_MAX_LIMIT) {
			beyond_max_num++;
		}
		if (test_rslt_buf[i] & _BEYOND_MIN_LIMIT) {
			beyond_min_num++;
		}
		if (test_rslt_buf[i] & _BEYOND_ACC_LIMIT) {
			beyond_acc_num++;
		}
	}
	if (beyond_max_num > _MIN_ERROR_NUM) {
		result |= _BEYOND_MAX_LIMIT;
		j = 0;
		SET_INFO_LINE_INFO("Beyond Max Limit Points Info: ");
		for (i = 0; i < _BEYOND_REC_MAX; ++i) {
			if (touchpad_sum[i].beyond_type == 0) {
				break;
			}
			SET_INFO_LINE_INFO("  Drv: %d, Sen: %d[Times: %d]",
					   touchpad_sum[i].driver,
					   touchpad_sum[i].sensor,
					   touchpad_sum[i].times);
		}
	}
	if (beyond_min_num > _MIN_ERROR_NUM) {
		result |= _BEYOND_MIN_LIMIT;
		SET_INFO_LINE_INFO("Beyond Min Limit Points Info:");
		j = 0;
		for (i = _BEYOND_REC_MAX; i < (2 * _BEYOND_REC_MAX); ++i) {
			if (touchpad_sum[i].beyond_type == 0) {
				break;
			}
			SET_INFO_LINE_INFO("  Drv: %d, Sen: %d[Times: %d]",
					   touchpad_sum[i].driver,
					   touchpad_sum[i].sensor,
					   touchpad_sum[i].times);
		}
	}
	if (beyond_acc_num > _MIN_ERROR_NUM) {
		result |= _BEYOND_ACC_LIMIT;
		SET_INFO_LINE_INFO("Beyond ACC Limit Points Info:");
		j = 0;
		for (i = 4 * _BEYOND_REC_MAX; i < (5 * _BEYOND_REC_MAX); ++i) {
			if (touchpad_sum[i].beyond_type == 0) {
				break;
			}
			SET_INFO_LINE_INFO
			    ("  Drv: %d, Sen: %d[Times: %d][Val: %d]",
			     touchpad_sum[i].driver, touchpad_sum[i].sensor,
			     touchpad_sum[i].times, touchpad_sum[i].raw_val);
		}
	}

	if (result == 0) {
		SET_INFO_LINE_INFO("[TEST SUCCEED]: The TP is ok!");
		return result;
	}
	SET_INFO_LINE_INFO("[TEST FAILED]:");
	if (result & _BEYOND_MAX_LIMIT) {
		SET_INFO_LINE_INFO("  Beyond Raw Max Limit[Max Limit: %d]",
				   max_limit_value);
	}
	if (result & _BEYOND_MIN_LIMIT) {
		SET_INFO_LINE_INFO("  Beyond Raw Min Limit[Min Limit: %d]",
				   min_limit_value);
	}
	if (result & _BEYOND_ACC_LIMIT) {
		SET_INFO_LINE_INFO("  Beyond Raw ACC Limit");
	}

	return result;
}

/*
 ===================================================
 * Function:
 * 		test gt9 series ic open test
 * Input:
 * 		client, i2c_client
 * Return:
 * 		SUCCESS: test process success, FAIL, test process failed
 *
 ===================================================
*/

s32 gt9xx_open_test(struct i2c_client *client)
{
	u16 i = 0;
	u16 j = 0;
	s32 ret = 0;		/* SUCCESS, FAIL */
	struct goodix_ts_data *ts;
	u16 *raw_buf = NULL;
	bool print_rawdata = false;

	ts = i2c_get_clientdata(i2c_connect_client);
	gtp_irq_disable(ts);
	SET_INFO_LINE_INFO("--- gtp open test ---");
	SET_INFO_LINE_INFO("max_limit_value is %d", max_limit_value);
	SET_INFO_LINE_INFO("min_limit_value is %d", min_limit_value);

	GTP_DEBUG("Parsing configuration...");
	ret = gtp_parse_config();
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("failed to parse config...");
		goto open_test_exit;
	}
	raw_buf = (u16 *) kmalloc(sizeof(u16) * gt9xx_pixel_cnt, GFP_KERNEL);
	if (NULL == raw_buf) {
		SET_INFO_LINE_ERR("failed to allocate mem for raw_buf!");
		goto open_test_exit;
	}

	GTP_DEBUG("Step 1: Send Rawdata Cmd");

	ts->gtp_rawdiff_mode = 1;
	gtp_raw_test_init(0);
	ret = gt9_read_raw_cmd(client);
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("Send Read Rawdata Cmd failed!");
		goto open_test_exit;
	}
	GTP_DEBUG("Step 2: Sample Rawdata");
	for (i = 0; i < sample_set_num; ++i) {
		rslt_buf_idx = i;
		ret = gtp_read_rawdata(client, raw_buf);
		if (ret == FAIL) {
			SET_INFO_LINE_ERR("Read Rawdata failed!");
			goto open_test_exit;
		}
		ret = gtp_raw_test(raw_buf, 0);
		if (ret == FAIL) {
			gtp_i2c_end_cmd(client);
			continue;
		}

		if ((test_rslt_buf[rslt_buf_idx] != 0) && (!print_rawdata)) {	/* some errors happen, print current raw data. */
			for (j = 0; j < 511; j = j + 30) {
				SET_LINE_INFO
				    ("%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
				     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
				     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u",
				     raw_buf[j], raw_buf[j + 1], raw_buf[j + 2],
				     raw_buf[j + 3], raw_buf[j + 4],
				     raw_buf[j + 5], raw_buf[j + 6],
				     raw_buf[j + 7], raw_buf[j + 8],
				     raw_buf[j + 9], raw_buf[j + 10],
				     raw_buf[j + 11], raw_buf[j + 12],
				     raw_buf[j + 13], raw_buf[j + 14],
				     raw_buf[j + 15], raw_buf[j + 16],
				     raw_buf[j + 17], raw_buf[j + 18],
				     raw_buf[j + 19], raw_buf[j + 20],
				     raw_buf[j + 21], raw_buf[j + 22],
				     raw_buf[j + 23], raw_buf[j + 24],
				     raw_buf[j + 25], raw_buf[j + 26],
				     raw_buf[j + 27], raw_buf[j + 28],
				     raw_buf[j + 29]);
			}
			SET_INFO_LINE_INFO("Frame %d failed", rslt_buf_idx);
			SET_INFO_LINE_INFO("Frame Max is %u, Frame Min is %u",
					   global_raw_max, global_raw_min);
			print_rawdata = true;
		}
	}

	if (!print_rawdata) {	/* no errors happen, print last raw data. */
		for (j = 0; j < 511; j = j + 30) {
			SET_LINE_INFO
			    ("%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
			     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
			     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u",
			     raw_buf[j], raw_buf[j + 1], raw_buf[j + 2],
			     raw_buf[j + 3], raw_buf[j + 4], raw_buf[j + 5],
			     raw_buf[j + 6], raw_buf[j + 7], raw_buf[j + 8],
			     raw_buf[j + 9], raw_buf[j + 10], raw_buf[j + 11],
			     raw_buf[j + 12], raw_buf[j + 13], raw_buf[j + 14],
			     raw_buf[j + 15], raw_buf[j + 16], raw_buf[j + 17],
			     raw_buf[j + 18], raw_buf[j + 19], raw_buf[j + 20],
			     raw_buf[j + 21], raw_buf[j + 22], raw_buf[j + 23],
			     raw_buf[j + 24], raw_buf[j + 25], raw_buf[j + 26],
			     raw_buf[j + 27], raw_buf[j + 28], raw_buf[j + 29]);
		}
		SET_INFO_LINE_INFO("Last Success Frame");
		SET_INFO_LINE_INFO("Frame Max is %u, Frame Min is %u",
				   global_raw_max, global_raw_min);
	}

	GTP_DEBUG("Step 3: Analyse Result");
	SET_INFO_LINE_INFO("Total %d Sample Data", sample_set_num);
	gtp_get_test_result();

	ret = SUCCESS;
 open_test_exit:

	kfree(raw_buf);
	if (test_rslt_buf) {
		kfree(test_rslt_buf);
	}
	if (touchpad_sum) {
		kfree(touchpad_sum);
	}
	gtp_irq_enable(ts);
	ts->gtp_rawdiff_mode = 0;
	gt9_read_coor_cmd(client);	/* back to read coordinates data */
	SET_INFO_LINE_INFO("---gtp open test end---");
	gup_i2c_write(ts->client, config,
		      GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH);
	return ret;
}

/*
 ===================================================
 * Function:
 * 		test gt9 series ic acc test
 * Input:
 * 		client, i2c_client
 * Return:
 * 		SUCCESS: test process success, FAIL, test process failed
 *
 ===================================================
*/
s32 gt9xx_acc_test(struct i2c_client *client)
{
	u16 i = 0;
	u16 j = 0;
	s32 ret = 0;		/* SUCCESS, FAIL */
	struct goodix_ts_data *ts;
	u16 *raw_buf = NULL;
	bool print_rawdata = false;

	ts = i2c_get_clientdata(i2c_connect_client);
	gtp_irq_disable(ts);
	SET_INFO_LINE_INFO("---gtp acc test---");
	GTP_DEBUG("Parsing configuration...");
	ret = gtp_parse_config();
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("failed to parse config...");
		goto acc_test_exit;
	}

	raw_buf = (u16 *) kmalloc(sizeof(u16) * gt9xx_pixel_cnt, GFP_KERNEL);
	if (NULL == raw_buf) {
		SET_INFO_LINE_ERR("failed to allocate mem for raw_buf!");
		goto acc_test_exit;
	}

	GTP_DEBUG("Step 1: Send Rawdata Cmd");
	ts->gtp_rawdiff_mode = 1;
	gtp_raw_test_init(0);
	ret = gt9_read_raw_cmd(client);
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("Send Read Rawdata Cmd failed!");
		goto acc_test_exit;
	}

	GTP_DEBUG("Step 2: Sample Rawdata");
	for (i = 0; i < sample_set_num; ++i) {
		rslt_buf_idx = i;
		ret = gtp_read_rawdata(client, raw_buf);
		if (ret == FAIL) {
			SET_INFO_LINE_ERR("Read Rawdata failed!");
			goto acc_test_exit;
		}
		ret = gtp_raw_test(raw_buf, 0x0010);
		if (ret == FAIL) {
			gtp_i2c_end_cmd(client);
			continue;
		}

		if ((test_rslt_buf[rslt_buf_idx] != 0) && (!print_rawdata)) {	/* some errors happen, print current raw data. */
			for (j = 0; j < 511; j = j + 30) {
				SET_LINE_INFO
				    ("%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
				     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
				     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u",
				     raw_buf[j], raw_buf[j + 1], raw_buf[j + 2],
				     raw_buf[j + 3], raw_buf[j + 4],
				     raw_buf[j + 5], raw_buf[j + 6],
				     raw_buf[j + 7], raw_buf[j + 8],
				     raw_buf[j + 9], raw_buf[j + 10],
				     raw_buf[j + 11], raw_buf[j + 12],
				     raw_buf[j + 13], raw_buf[j + 14],
				     raw_buf[j + 15], raw_buf[j + 16],
				     raw_buf[j + 17], raw_buf[j + 18],
				     raw_buf[j + 19], raw_buf[j + 20],
				     raw_buf[j + 21], raw_buf[j + 22],
				     raw_buf[j + 23], raw_buf[j + 24],
				     raw_buf[j + 25], raw_buf[j + 26],
				     raw_buf[j + 27], raw_buf[j + 28],
				     raw_buf[j + 29]);
			}
			SET_INFO_LINE_INFO("Frame %d failed", rslt_buf_idx);
			print_rawdata = true;
		}
	}

	if (!print_rawdata) {	/* no errors happen, print last raw data. */
		for (j = 0; j < 511; j = j + 30) {
			SET_LINE_INFO
			    ("%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
			     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
			     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u",
			     raw_buf[j], raw_buf[j + 1], raw_buf[j + 2],
			     raw_buf[j + 3], raw_buf[j + 4], raw_buf[j + 5],
			     raw_buf[j + 6], raw_buf[j + 7], raw_buf[j + 8],
			     raw_buf[j + 9], raw_buf[j + 10], raw_buf[j + 11],
			     raw_buf[j + 12], raw_buf[j + 13], raw_buf[j + 14],
			     raw_buf[j + 15], raw_buf[j + 16], raw_buf[j + 17],
			     raw_buf[j + 18], raw_buf[j + 19], raw_buf[j + 20],
			     raw_buf[j + 21], raw_buf[j + 22], raw_buf[j + 23],
			     raw_buf[j + 24], raw_buf[j + 25], raw_buf[j + 26],
			     raw_buf[j + 27], raw_buf[j + 28], raw_buf[j + 29]);
		}
		SET_INFO_LINE_INFO("Last Success Frame");
	}

	GTP_DEBUG("Step 3: Analyse Result");
	SET_INFO_LINE_INFO("Total %d Sample Data", sample_set_num);
	gtp_get_test_result();

	ret = SUCCESS;

 acc_test_exit:

	kfree(raw_buf);
	if (test_rslt_buf) {
		kfree(test_rslt_buf);
	}
	if (touchpad_sum) {
		kfree(touchpad_sum);
	}
	gtp_irq_enable(ts);
	ts->gtp_rawdiff_mode = 0;
	gt9_read_coor_cmd(client);	/* back to read coordinates data */
	SET_INFO_LINE_INFO("---gtp acc test end---");
	gup_i2c_write(ts->client, config,
		      GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH);
	return ret;
}

/*
 ===================================================
 *   gtp_raw_print
 * Input:
 *   u16* raw_buf: rawdata buffer
 ===================================================
*/
s32 gtp_raw_print(struct i2c_client *client)
{
	struct goodix_ts_data *ts;

	u16 i = 0;
	s32 ret = 0;		/* SUCCESS, FAIL */
	u16 *raw_buf = NULL;

	ts = i2c_get_clientdata(i2c_connect_client);
	gtp_irq_disable(ts);

	ret = gtp_parse_config();
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("failed to parse config...");
		goto raw_print_exit;
	}

	SET_INFO_LINE_INFO("Total node number: %d", gt9xx_sc_pxl_cnt);

	raw_buf = (u16 *) kmalloc(sizeof(u16) * gt9xx_pixel_cnt, GFP_KERNEL);
	if (NULL == raw_buf) {
		SET_INFO_LINE_ERR("failed to allocate mem for raw_buf!");
		goto raw_print_exit;
	}

	GTP_DEBUG("Step 1: Send Rawdata Cmd");
	ts->gtp_rawdiff_mode = 1;
	gtp_raw_test_init(0);
	ret = gt9_read_raw_cmd(client);
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("Send Read Rawdata Cmd failed!");
		goto raw_print_exit;
	}

	GTP_DEBUG("Step 2: Sample Rawdata");

	ret = gtp_read_rawdata(client, raw_buf);
	if (ret == FAIL) {
		SET_INFO_LINE_ERR("Read Rawdata failed!");
		goto raw_print_exit;
	}

	for (i = 0; i < 511; i = i + 30) {
		SET_LINE_INFO
		    ("%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
		     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u "
		     "%04u %04u %04u %04u %04u %04u %04u %04u %04u %04u",
		     raw_buf[i], raw_buf[i + 1], raw_buf[i + 2], raw_buf[i + 3],
		     raw_buf[i + 4], raw_buf[i + 5], raw_buf[i + 6],
		     raw_buf[i + 7], raw_buf[i + 8], raw_buf[i + 9],
		     raw_buf[i + 10], raw_buf[i + 11], raw_buf[i + 12],
		     raw_buf[i + 13], raw_buf[i + 14], raw_buf[i + 15],
		     raw_buf[i + 16], raw_buf[i + 17], raw_buf[i + 18],
		     raw_buf[i + 19], raw_buf[i + 20], raw_buf[i + 21],
		     raw_buf[i + 22], raw_buf[i + 23], raw_buf[i + 24],
		     raw_buf[i + 25], raw_buf[i + 26], raw_buf[i + 27],
		     raw_buf[i + 28], raw_buf[i + 29]);
	}

 raw_print_exit:

	kfree(raw_buf);
	gtp_irq_enable(ts);
	ts->gtp_rawdiff_mode = 0;
	gt9_read_coor_cmd(client);	/* back to read coordinates data */
	gup_i2c_write(ts->client, config,
		      GTP_ADDR_LENGTH + GTP_CONFIG_MAX_LENGTH);

	return ret;
}

/*
 ===================================================
 *
 * Sysfs show and store function
 *
 ===================================================
*/
static ssize_t gtp_sysfs_shorttest_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	u8 index;
	u32 len;

	gt9xx_short_test(i2c_connect_client);

	for (index = 0, len = 0; index < RsltIndex; ++index) {
		sprintf(&buf[len], "%s", result_lines[index]);
		len += strlen(result_lines[index]);
		kfree(result_lines[index]);
	}
	RsltIndex = 0;
	return len;
}

static ssize_t gtp_sysfs_shorttest_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	return -EPERM;
}

static ssize_t gtp_sysfs_opentest_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	s32 index;
	u32 len;

	gt9xx_open_test(i2c_connect_client);

	for (index = 0, len = 0; index < RsltIndex; ++index) {
		sprintf(&buf[len], "%s", result_lines[index]);
		len += strlen(result_lines[index]);
		kfree(result_lines[index]);
	}
	RsltIndex = 0;
	return len;
}

static ssize_t gtp_sysfs_opentest_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char temp_max[20];
	char temp_min[20];
	int ret = 0;

	sscanf(buf, "%s %s", temp_max, temp_min);

	ret = kstrtou16(temp_max, 10, &max_limit_value);
	if (ret != 0)
		return -1;

	ret = kstrtou16(temp_min, 10, &min_limit_value);
	if (ret != 0)
		return -1;

	return count;
}

static ssize_t gtp_sysfs_acctest_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	s32 index;
	u32 len;

	SET_INFO_LINE_INFO("ACC_threshold 0 is [%d]", raw_acc_data[3]);
	SET_INFO_LINE_INFO("ACC_threshold 1 is [%d]", raw_acc_data[7]);
	SET_INFO_LINE_INFO("ACC_threshold 2 is [%d]", raw_acc_data[11]);

	gt9xx_acc_test(i2c_connect_client);

	for (index = 0, len = 0; index < RsltIndex; ++index) {
		sprintf(&buf[len], "%s", result_lines[index]);
		len += strlen(result_lines[index]);
		kfree(result_lines[index]);
	}
	RsltIndex = 0;
	return len;
}

static ssize_t gtp_sysfs_acctest_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	char file_path[512] = { '\0' };
	struct file *filp = NULL;
	mm_segment_t oldfs;
	int result, ret = 0;
	int i = 0, INPUT_ACC_DATA_index = 0;
	char temp_acc_char[20];
	u16 temp_acc_u16 = 0;

	if (count > 511)
		return -1;

	snprintf(file_path, count, "%s", buf);
	filp = filp_open(file_path, O_RDONLY, 0);
	if (IS_ERR(filp)) {
		GTP_INFO("Open file failed\n");
		return -1;
	}

	oldfs = get_fs();
	set_fs(get_ds());

	result =
	    filp->f_op->read(filp, INPUT_ACC_DATA, sizeof(INPUT_ACC_DATA),
			     &filp->f_pos);
	if (result < 0) {
		GTP_INFO("Read file failed\n");
		return -1;
	}

	set_fs(oldfs);
	filp_close(filp, NULL);

	GTP_DEBUG("Parsing configuration...");
	ret = gtp_parse_config();
	if (ret == FAIL) {
		return -1;
	}

	for (i = 0; i < gt9xx_pixel_cnt * 4; i++) {
		memset(temp_acc_char, '\0', 20);
		ret =
		    sscanf(&INPUT_ACC_DATA[INPUT_ACC_DATA_index], "%s",
			   temp_acc_char);
		if (ret != 1) {
			GTP_INFO("Read stop at %d\n", i);
			break;
		}

		ret = kstrtou16(temp_acc_char, 10, &temp_acc_u16);
		if (ret != 0)
			break;

		INPUT_ACC_DATA_index =
		    INPUT_ACC_DATA_index + strlen(temp_acc_char) + 1;
		raw_acc_data[i] = temp_acc_u16;
	}

	return result;
}

static ssize_t gtp_sysfs_irq_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	s32 index;
	u32 len;

	if (irq_enable)
		SET_INFO_LINE_INFO("Touch is enabled");
	else
		SET_INFO_LINE_INFO("Touch is disabled");

	for (index = 0, len = 0; index < RsltIndex; ++index) {
		sprintf(&buf[len], "%s", result_lines[index]);
		len += strlen(result_lines[index]);
		kfree(result_lines[index]);
	}
	RsltIndex = 0;
	return len;
}

static ssize_t gtp_sysfs_irq_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct goodix_ts_data *ts;

	ts = i2c_get_clientdata(i2c_connect_client);

	if (buf[0] == '1') {
		irq_enable = true;
		gtp_irq_enable(ts);
	}

	if (buf[0] == '0') {
		irq_enable = false;
		gtp_irq_disable(ts);
	}

	return count;
}

static ssize_t gtp_sysfs_rawdata_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	s32 index;
	u32 len;

	gtp_raw_print(i2c_connect_client);

	for (index = 0, len = 0; index < RsltIndex; ++index) {
		sprintf(&buf[len], "%s", result_lines[index]);
		len += strlen(result_lines[index]);
		kfree(result_lines[index]);
	}
	RsltIndex = 0;

	return len;
}

static DEVICE_ATTR(rawdata, S_IRUGO, gtp_sysfs_rawdata_show, NULL);
static DEVICE_ATTR(touch_status, S_IRUGO, get_touch_status, NULL);
static DEVICE_ATTR(fw_version, S_IRUGO, get_fw_version, NULL);
static DEVICE_ATTR(cfg_version, S_IRUGO, get_cfg_version, NULL);
static DEVICE_ATTR(shorttest, S_IRUGO | S_IWUSR, gtp_sysfs_shorttest_show,
		   gtp_sysfs_shorttest_store);
static DEVICE_ATTR(opentest, S_IRUGO | S_IWUSR, gtp_sysfs_opentest_show,
		   gtp_sysfs_opentest_store);
static DEVICE_ATTR(acctest, S_IRUGO | S_IWUSR, gtp_sysfs_acctest_show,
		   gtp_sysfs_acctest_store);
static DEVICE_ATTR(irq, S_IRUGO | S_IWUSR, gtp_sysfs_irq_show,
		   gtp_sysfs_irq_store);
static DEVICE_ATTR(dclick, S_IWUSR, NULL, sysfs_dclick_store);
static DEVICE_ATTR(gesture, S_IWUSR, NULL, sysfs_gesture_store);
static DEVICE_ATTR(flipcovermode, S_IWUSR, NULL, sysfs_flipcovermode_store);

/*******************************************************
Description:
	Goodix debug sysfs init function.

Parameter:
	none.

return:
	Executive outcomes. 0---succeed.
*******************************************************/
s32 gtp_test_sysfs_init(void)
{
	s32 ret;

	goodix_debug_kobj = kobject_create_and_add("gtp_test", NULL);
	SET_INFO_LINE_INFO("Starting initlizing gtp_debug_sysfs");
	if (goodix_debug_kobj == NULL) {
		GTP_ERROR("%s: subsystem_register failed\n", __func__);
		return -ENOMEM;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_shorttest.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_opentest.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_touch_status.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_acctest.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_irq.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_fw_version.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_cfg_version.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_rawdata.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_dclick.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_gesture.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}

	ret = sysfs_create_file(goodix_debug_kobj, &dev_attr_flipcovermode.attr);
	if (ret) {
		GTP_ERROR("%s: sysfs_create_version_file failed\n", __func__);
		return ret;
	}



	GTP_INFO("Goodix debug sysfs create success!\n");
	return 0;
}

void gtp_test_sysfs_deinit(void)
{
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_shorttest.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_opentest.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_touch_status.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_acctest.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_irq.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_fw_version.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_cfg_version.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_rawdata.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_dclick.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_gesture.attr);
	sysfs_remove_file(goodix_debug_kobj, &dev_attr_flipcovermode.attr);

	kobject_del(goodix_debug_kobj);
}
