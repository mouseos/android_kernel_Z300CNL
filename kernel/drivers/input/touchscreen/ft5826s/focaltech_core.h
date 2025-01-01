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
 */

#ifndef __LINUX_FTS_H__
#define __LINUX_FTS_H__
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
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mount.h>
#include <linux/netdevice.h>
#include <linux/unistd.h>
#include <linux/ioctl.h>
#include <linux/earlysuspend.h>
#include <linux/i2c/focal_ft5826s.h>
#include "ft_gesture_lib.h"
#include <linux/board_asustek.h>
#include <linux/switch.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
#define FTS_DRIVER_INFO  "Qualcomm_Ver 1.3.1 2015-07-06"
#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define FT6X36_ID       	0x36

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55

#define LEN_FLASH_ECC_MAX 					0xFFFE

#define FTS_MAX_POINTS                        10

#define FTS_WORKQUEUE_NAME	"fts_wq"

#define FTS_DEBUG_DIR_NAME	"fts_debug"

#define FTS_INFO_MAX_LEN		512
#define FTS_FW_NAME_MAX_LEN	50

#define FTS_REG_ID		0xA3
#define FTS_REG_FW_VER		0xA6
#define FTS_REG_FW_VENDOR_ID	0xA8
#define FTS_REG_FW_PANEL_ID	0xAC
#define FTS_REG_POINT_RATE	0x88

#define FTS_FACTORYMODE_VALUE	0x40
#define FTS_WORKMODE_VALUE	0x00

#define FTS_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FTS_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= %s\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FTS_DRIVER_INFO, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)
				

#define FTS_DBG_EN 1
#if FTS_DBG_EN
#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif


/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/
struct ts_event {
	u16 au16_x[FTS_MAX_POINTS];	/*x coordinate */
	u16 au16_y[FTS_MAX_POINTS];	/*y coordinate */
	u16 pressure[FTS_MAX_POINTS];
	u8 au8_touch_event[FTS_MAX_POINTS];	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[FTS_MAX_POINTS];	/*touch ID */
	u8 area[FTS_MAX_POINTS];
	u8 touch_point;
	u8 point_num;
};

struct fts_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	const struct fts_ts_platform_data *pdata;
	struct fts_psensor_platform_data *psensor_pdata;
	struct work_struct 	touch_event_work;
	struct workqueue_struct *ts_workqueue;
	struct regulator *vdd;
	struct regulator *vcc_i2c;
	char fw_name[FTS_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	bool irq_status;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
	u8 fw_panel_id;
	int touchs;
	struct early_suspend early_suspend;
	struct switch_dev touch_sdev;

	bool init_success;
	/*touch gesture control*/
	/*
	  write to addr 0xd1, zenMotion_ctrl bit 4 --> enable dclick
	  write to addr 0xd1, zenMotion_ctrl bit 5 --> enable gesture
	*/
	u8 zenMotion_ctrl;
	/*
	  write to addr 0xd2, gesture_wec bit 1 --> enable "w"
	  write to addr 0xd2, gesture_wec bit 3 --> enable "e"
	  write to addr 0xd2, gesture_wec bit 4 --> enable "c"
	*/
	u8 gesture_wec;
	bool gesture_wec_on;
	bool gesture_s_on;
	bool gesture_z_on;
	bool gesture_v_on;
	struct wake_lock fts_wake_lock;

	struct notifier_block	cable_status_notif;
	struct workqueue_struct *usb_wq;
	struct work_struct 	usb_detect_work;
	bool usb_status;
};

/*******************************************************************************
* Static variables
*******************************************************************************/

/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
//Function Switchs: define to open,  comment to close
#define GTP_ESD_PROTECT 0
#define FTS_APK_DEBUG
#define FTS_SYSFS_DEBUG
#define FTS_CTL_IIC
#define FTS_AUTO_UPGRADE

extern struct fts_Upgrade_Info fts_updateinfo_curr;
extern struct i2c_client *fts_i2c_client;
extern struct fts_ts_data *fts_wq_data;
extern struct input_dev *fts_input_dev;

static DEFINE_MUTEX(i2c_rw_access);

//Getstre functions
extern int fts_Gesture_init(struct input_dev *input_dev);
extern int fts_read_Gesturedata(void);
extern int fetch_object_sample(unsigned char *buf,short pointnum);
extern void init_para(int x_pixel,int y_pixel,int time_slot,int cut_x_pixel,int cut_y_pixel);

//upgrade functions
extern void fts_update_fw_panel_id(struct fts_ts_data *data);
extern void fts_update_fw_vendor_id(struct fts_ts_data *data);
extern void fts_update_fw_ver(struct fts_ts_data *data);
extern void fts_get_upgrade_array(void);
extern int fts_ctpm_auto_upgrade(struct i2c_client *client);
extern int fts_fw_upgrade(struct device *dev, bool force);
extern int fts_ctpm_auto_clb(struct i2c_client *client);
extern int fts_ctpm_fw_upgrade_with_app_file(struct i2c_client *client, char *firmware_name);
extern int fts_ctpm_fw_upgrade_with_i_file(struct i2c_client *client);
extern int fts_ctpm_get_i_file_ver(void);
extern int fts_ctpm_bl_ReadVendorID(struct i2c_client *client, u8 *ucPVendorID);
extern int fts_ctpm_fw_upgrade_with_all_file(struct i2c_client *client,
							char *firmware_name);

//Apk and functions
extern int fts_create_apk_debug_channel(struct i2c_client * client);
extern void fts_release_apk_debug_channel(void);

//ADB functions
extern int fts_create_sysfs(struct i2c_client *client);
extern int fts_remove_sysfs(struct i2c_client *client);

//char device for old apk
extern int fts_rw_iic_drv_init(struct i2c_client *client);
extern void  fts_rw_iic_drv_exit(void);

//Base functions
extern int fts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
extern int fts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
extern int fts_read_reg(struct i2c_client *client, u8 addr, u8 *val);
extern int fts_write_reg(struct i2c_client *client, u8 addr, const u8 val);

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);
/*******************************************************************************
* Static function prototypes
*******************************************************************************/
#endif
