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

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = false;
			#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
			#endif
			
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = true;
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
		
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = false;
			#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = true;
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{	
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;
	
	
	if (fwver == 255)
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
	else
	{
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	}
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	
	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	retval = strict_strtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal][test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			dev_err(dev, "[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			dev_dbg(dev, "[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);
	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	fts_wq_data->irq_status = false;
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif
	
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}
	
	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	enable_irq(client->irq);
	fts_wq_data->irq_status = true;
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	fts_wq_data->irq_status = false;
	#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
	enable_irq(client->irq);
	fts_wq_data->irq_status = true;
	
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

static ssize_t touch_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	u8 fwver = 0;
	ssize_t count = 0;

	if (fts_wq_data->suspended) {
		printk("[Focal] touch suspended\n");
		count = snprintf(buf, PAGE_SIZE, "Device suspended\n");
		return count;
	}

	mutex_lock(&fts_input_dev->mutex);

	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0) {
		pr_info("[Focal] read TP fw version failed\n");
		count = snprintf(buf, PAGE_SIZE, "0\n");
	} else {
		pr_info("[Focal] fw: 0x%02x\n", fwver);
		count = snprintf(buf, PAGE_SIZE, "1\n");
	}

	mutex_unlock(&fts_input_dev->mutex);
	return count;
}

static ssize_t touch_func_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t count = 0;
	count = snprintf(buf, PAGE_SIZE, "%d\n",
				(fts_wq_data->irq_status)? 1 : 0);
	return count;
}

static ssize_t touch_func_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	mutex_lock(&fts_input_dev->mutex);

	if (buf[0] == '0') {
		if (fts_wq_data->irq_status) {
			disable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = false;
			pr_info("[Focal] Disable touch irq\n");
		} else
			pr_info("[Focal] touch irq already disabled\n");

	} else if (buf[0] == '1') {
		if (!(fts_wq_data->irq_status)) {
			enable_irq(fts_i2c_client->irq);
			fts_wq_data->irq_status = true;
			pr_info("[Focal] Enable touch irq\n");
		} else
			pr_info("[Focal] touch irq already enabled\n");

	} else
		pr_info("[Focal] unknown parameter for touch function\n");

	mutex_unlock(&fts_input_dev->mutex);
	return count;
}

static ssize_t tp_fw_info_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 reg_addr;
	u8 fwver = 0xFF;
	u8 fw_vendor_id = 0xFF;
	u8 fw_panel_id = 0xFF;
	u8 bl_vendor_id = 0xFF;

	reg_addr = FTS_REG_FW_VER;
	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fwver, 1);
	if (err < 0) {
		dev_err(dev, "fw version read failed");
		fwver = 0xFF;
	}

	reg_addr = FTS_REG_FW_VENDOR_ID;
	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fw_vendor_id, 1);
	if (err < 0) {
		dev_err(dev, "fw vendor id read failed");
		fw_vendor_id = 0xFF;
	}

	reg_addr = FTS_REG_FW_PANEL_ID;
	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fw_panel_id, 1);
	if (err < 0) {
		dev_err(dev, "fw panel id read failed");
		fw_panel_id = 0xFF;
	}

	err = fts_ctpm_bl_ReadVendorID(fts_i2c_client, &bl_vendor_id);
	if (err < 0) {
		dev_err(dev, "bootloader vendorl id read failed");
		bl_vendor_id = 0xFF;
	}

	pr_info("[Focal] FW_ver = %x, FW_vid = %x, FW_pid=%d, bl_vid = %x\n",
			fwver, fw_vendor_id, fw_panel_id, bl_vendor_id);
	return sprintf(buf, "%x-%x-%x-%x\n",
			fwver, fw_vendor_id, fw_panel_id, bl_vendor_id);
}

static ssize_t fts_fwupgradeall_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	int ret = 0;

	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count - 1] = '\0';

	mutex_lock(&fts_input_dev->mutex);

	disable_irq(client->irq);
	fts_wq_data->irq_status = false;

	#if GTP_ESD_PROTECT
	apk_debug_flag = 1;
	#endif

	ret = fts_ctpm_fw_upgrade_with_all_file(client, fwname);
	if (ret)
		pr_info("[Focal] fw upgrade all.bin failed\n");
	else
		pr_info("[Focal] fw upgrade all.bin successful\n");

	#if GTP_ESD_PROTECT
	apk_debug_flag = 0;
	#endif

	enable_irq(client->irq);
	fts_wq_data->irq_status = true;

	mutex_unlock(&fts_input_dev->mutex);

	return count;
}

static ssize_t tp_vendor_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	int err = 0;
	u8 reg_addr;
	u8 fw_vendor_id = 0xFF;
	u8 fw_panel_id = 0xFF;
	char vendor[20] = {0};

	reg_addr = FTS_REG_FW_VENDOR_ID;
	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fw_vendor_id, 1);
	if (err < 0) {
		dev_err(dev, "fw vendor id read failed");
		fw_vendor_id = 0xFF;
	}

	reg_addr = FTS_REG_FW_PANEL_ID;
	err = fts_i2c_read(fts_i2c_client, &reg_addr, 1, &fw_panel_id, 1);
	if (err < 0) {
		dev_err(dev, "fw panel id read failed");
		fw_panel_id = 0xFF;
	}

	if (fw_vendor_id == 0x3b && fw_panel_id == 0x00)
		strncpy(vendor, "BIEL_GFF", 20);
	else if (fw_vendor_id == 0x3b && fw_panel_id == 0x01)
		strncpy(vendor, "BIEL_OGS", 20);
	else if (fw_vendor_id == 0x55 && fw_panel_id == 0x01)
		strncpy(vendor, "LAIBAO_OGS", 20);
	else
		strncpy(vendor, "UNKNOWN", 20);


	pr_info("[Focal] FW_vid = %x, FW_pid=%d\n", fw_vendor_id, fw_panel_id);
	return sprintf(buf, "%s\n", vendor);
}

static ssize_t touch_dclick_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "dclick master control = %d\n",
			(fts_wq_data->zenMotion_ctrl & 0x10) >> 4);
}

static ssize_t touch_dclick_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (fts_wq_data->suspended || !fts_wq_data->init_success) {
		pr_info("[Focal][Touch] gesture mode setting aborted\n");
		return count;
	}

	if (buf[0] == '1') {
		/* set bit 4 to 1, for enable double click*/
		fts_wq_data->zenMotion_ctrl |= 0x10;
		pr_info("[Focal][Touch] enable double click.\n");
	} else if (buf[0] == '0') {
		/* set bit 4 to 0, for disable double click
		   and don't change bit 5 */
		fts_wq_data->zenMotion_ctrl &= 0x20;
		pr_info("[Focal][Touch] disable double click.\n");
	} else {
		/* set bit 4 to 0, for disable double click
		   and don't change bit 5 */
		fts_wq_data->zenMotion_ctrl &= 0x20;
		pr_info("[Focal][Touch] Unknown parameter to dclick store.\n");
	}
	return count;
}

static ssize_t touch_gesture_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf,
		"[Focal][Touch] gesture mctl= %d, wseczv= %d %d %d %d %d %d\n",
				(fts_wq_data->zenMotion_ctrl & 0x20) >> 5,
				(fts_wq_data->gesture_wec & 0x02) >> 1,
				fts_wq_data->gesture_s_on,
				(fts_wq_data->gesture_wec & 0x08) >> 3,
				(fts_wq_data->gesture_wec & 0x10) >> 4,
				fts_wq_data->gesture_z_on,
				fts_wq_data->gesture_v_on);
}

static ssize_t touch_gesture_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (fts_wq_data->suspended || !fts_wq_data->init_success) {
		pr_info("[Focal][Touch] gesture mode setting aborted\n");
		return count;
	}

	/*
	  buf [0]  [1][2][3][4][5][6] mapping to
	     [ctrl][w][s][e][c][z][v]
	*/
	/*store master control*/
	if (buf[0] == '1') {
		/* set bit 5 to 1, for enable gesture*/
		fts_wq_data->zenMotion_ctrl |= 0x20;
	} else if (buf[0] == '0') {
		/* set bit 5 to 0, for disable gesture
		   and don't change bit 4 */
		fts_wq_data->zenMotion_ctrl &= 0x10;
	} else {
		/* set bit 5 to 0, for disable gesture
		   and don't change bit 4 */
		fts_wq_data->zenMotion_ctrl &= 0x10;
		pr_info("[Focal][Touch] Unknown parameter to gesture store.\n");
	}

	/*store w, e and c gesture*/
	fts_wq_data->gesture_wec = 0x00;
	fts_wq_data->gesture_wec_on = false;
	fts_wq_data->gesture_s_on = false;
	fts_wq_data->gesture_z_on = false;
	fts_wq_data->gesture_v_on = false;

	if (buf[1] == '1') {
		fts_wq_data->gesture_wec_on = true;
		fts_wq_data->gesture_wec |= (0x1 << 1);
	}
	if (buf[3] == '1') {
		fts_wq_data->gesture_wec_on = true;
		fts_wq_data->gesture_wec |= (0x1 << 3);
	}
	if (buf[4] == '1') {
		fts_wq_data->gesture_wec_on = true;
		fts_wq_data->gesture_wec |= (0x1 << 4);
	}

	/*store s gesture*/
	if (buf[2] == '1')
	    fts_wq_data->gesture_s_on = true;

	/*store z gesture*/
	if (buf[5] == '1')
	    fts_wq_data->gesture_z_on = true;

	/*store v gesture*/
	if (buf[6] == '1')
	    fts_wq_data->gesture_v_on = true;

	pr_info("[Focal][Touch] gesture mctl= %d, wseczv= %d %d %d %d %d %d\n",
				(fts_wq_data->zenMotion_ctrl & 0x20) >> 5,
				(fts_wq_data->gesture_wec & 0x02) >> 1,
				fts_wq_data->gesture_s_on,
				(fts_wq_data->gesture_wec & 0x08) >> 3,
				(fts_wq_data->gesture_wec & 0x10) >> 4,
				fts_wq_data->gesture_z_on,
				fts_wq_data->gesture_v_on);

	return count;
}
/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);

static DEVICE_ATTR(touch_status, S_IRUGO, touch_status_show, NULL);
static DEVICE_ATTR(touch_function, S_IRUGO | S_IWUSR, touch_func_show,
						touch_func_store);
static DEVICE_ATTR(tp_fw_info, S_IRUGO, tp_fw_info_show, NULL);
static DEVICE_ATTR(ftsfwupgradeall, S_IWUSR, NULL, fts_fwupgradeall_store);
static DEVICE_ATTR(tp_vendor, S_IRUGO, tp_vendor_show, NULL);
static DEVICE_ATTR(touch_dclick, S_IRUGO | S_IWUSR, touch_dclick_show,
					touch_dclick_store);
static DEVICE_ATTR(touch_gesture, S_IRUGO | S_IWUSR, touch_gesture_show,
					touch_gesture_store);

/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_touch_function.attr,
	&dev_attr_tp_fw_info.attr,
	&dev_attr_ftsfwupgradeall.attr,
	&dev_attr_tp_vendor.attr,
	&dev_attr_touch_dclick.attr,
	&dev_attr_touch_gesture.attr,
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
