/*
  * drivers/power/Smb347-charger.c
  *
  * Charger driver for Summit SMB347
  *
  * Copyright (c) 2012, ASUSTek Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful, but WITHOUT
  * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  * more details.
  */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/smb347-charger.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/usb/otg.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/ioctl.h>
#include <linux/board_asustek.h>
#include <linux/earlysuspend.h>

#include <asm/uaccess.h>

#define smb347_CHARGE		0x00
#define smb347_CHRG_CRNTS	0x01
#define smb347_VRS_FUNC		0x02
#define smb347_FLOAT_VLTG	0x03
#define smb347_CHRG_CTRL	0x04
#define smb347_STAT_TIME_CTRL	0x05
#define smb347_PIN_CTRL		0x06
#define smb347_THERM_CTRL	0x07
#define smb347_SYSOK_USB3	0x08
#define smb347_CTRL_REG		0x09

#define smb347_OTG_TLIM_REG	0x0A
#define smb347_HRD_SFT_TEMP	0x0B
#define smb347_FAULT_INTR	0x0C
#define smb347_STS_INTR_1	0x0D
#define smb347_I2C_ADDR		0x0E
#define smb347_IN_CLTG_DET	0x10
#define smb347_STS_INTR_2	0x11

/* Command registers */
#define smb347_CMD_REG		0x30
#define smb347_CMD_REG_B	0x31
#define smb347_CMD_REG_c	0x33

/* Interrupt Status registers */
#define smb347_INTR_STS_A	0x35
#define smb347_INTR_STS_B	0x36
#define smb347_INTR_STS_C	0x37
#define smb347_INTR_STS_D	0x38
#define smb347_INTR_STS_E	0x39
#define smb347_INTR_STS_F	0x3A

/* Status registers */
#define smb347_STS_REG_A	0x3B
#define smb347_STS_REG_B	0x3C
#define smb347_STS_REG_C	0x3D
#define smb347_STS_REG_D	0x3E
#define smb347_STS_REG_E	0x3F

/* GPIO pin definition */
#define APQ_AP_CHAR             22
#define APQ_AP_ACOK             179

#define smb347_ENABLE_WRITE	1
#define smb347_DISABLE_WRITE	0
#define ENABLE_WRT_ACCESS	0x80
#define ENABLE_APSD		0x04
#define PIN_CTRL		0x10
#define PIN_ACT_LOW		0x20
#define ENABLE_CHARGE		0x02
#define USBIN			0x80
#define APSD_OK			0x08
#define APSD_RESULT		0x07
#define FLOAT_VOLT_MASK		0x3F
#define ENABLE_PIN_CTRL_MASK	0x60
#define HOT_LIMIT_MASK		0x30
#define BAT_OVER_VOLT_MASK	0x40
#define STAT_OUTPUT_EN		0x20
#define GPIO_AC_OK		APQ_AP_ACOK
#define BAT_Cold_Limit		0
#define BAT_Hot_Limit		55
#define BAT_Mid_Temp_Wired	50
#define FLOAT_VOLT		0x2A
#define FLOAT_VOLT_LOW		0x1E
#define FLOAT_VOLT_43V		0x28
#define FLOAT_VOLT_LOW_DECIMAL	4110000
#define THERMAL_RULE1		1
#define THERMAL_RULE2		2

/* Z300CL setting definition */
#define TERMINATION_CURRENT		0x07
#define FLOAT_VOLT_410V			0x1D
#define FLOAT_VOLT_411V			0x1E
#define FLOAT_VOLT_42V			0x23
#define FLOAT_VOLT_434V			0x2A
#define FLOAT_VOLT_435V			0x2B
#define VBATT_100MV			0x80
#define VFLT_240MV			0x18
#define SOFT_HOT_LIMIT_BEHAVIOR_NO_RESPONSE	0x03
#define SOFT_HOT_LIMIT_BEHAVIOR_COMPENSATION	0x02
#define MAX_DCIN				1
#define MAX_USBIN				0
#define REVERTBYTE(a)   (char)((((a)>>4) & 0x0F) | (((a)<<4) & 0xF0))
#define USB_TO_HC_MODE				0x03
#define USB_to_REGISTER_CONTROL			0x10
#define AC_CURRENT				1200
#define AC_CURRENT_Z300CNL			900
#define USB_CURRENT				500
#define HARD_HOT_59_SOFT_HOT_53_MASK		0xCC
#define HARD_HOT_59_SOFT_HOT_53			0x12
#define USBIN_INPUT_CURRENT_MASK		0x0F
#define USBIN_INPUT_CURRENT_LIMIT_500		0x01
#define USBIN_INPUT_CURRENT_LIMIT_900		0x03
#define USBIN_INPUT_CURRENT_LIMIT_1200		0x04

/* Project definition */
#define Z300CL	8
#define Z300CNL 10

/* Functions declaration */
extern int  bq27320_battery_callback(unsigned usb_cable_state);
//extern void touch_callback(unsigned cable_status);
static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf);
int AICL_read_per_min(void);

/* Global variables */
static struct smb347_charger *charger;
static struct workqueue_struct *smb347_wq;
struct wake_lock charger_wakelock;
struct wake_lock COS_wakelock;
static int charge_en_flag = 1;
bool otg_on = false;
bool JEITA_early_suspend = false;
extern int ac_on;
extern int usb_on;
static bool disable_DCIN;
static unsigned int cable_state_detect = CHARGER_NONE;
extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);
extern unsigned int query_cable_status(void);
unsigned int smb347_charger_status = 0;
static bool smb347init = false;
static bool JEITA_init = false;
static int JEITA_flag = 0;
extern bool smb347_shutdown_value = false;
static project_id projectID = PROJECT_ID_INVALID;
static DEFINE_MUTEX(smb347_shutdown_mutex);
extern int RSOC;

static enum JEITA_flag {
	JEITA_flag_1 = 1,
	JEITA_flag_2 = 2,
	JEITA_flag_3 = 3,
	JEITA_flag_4 = 4,
	JEITA_flag_5 = 5,
};

/* ATD commands */
static ssize_t smb347_input_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t show_smb347_charger_status(struct device *dev, struct device_attribute *attr, char *buf);

/* Sysfs interface */
static DEVICE_ATTR(reg_status, S_IWUSR | S_IRUGO, smb347_reg_show, NULL);
static DEVICE_ATTR(input_AICL_result, S_IWUSR | S_IRUGO, smb347_input_AICL_result_show, NULL);
static DEVICE_ATTR(smb347_status, S_IWUSR | S_IRUGO, show_smb347_charger_status, NULL);

static struct attribute *smb347_attributes[] = {
	&dev_attr_reg_status.attr,
	&dev_attr_input_AICL_result.attr,
	&dev_attr_smb347_status.attr,
NULL
};

static const struct attribute_group smb347_group = {
	.attrs = smb347_attributes,
};

extern char androidboot_mode[32] = {0,};
int __init asustek_androidboot_mode(char *s)
{
	int n;

	if (*s == '=')
		s++;
	n = snprintf(androidboot_mode, sizeof(androidboot_mode), "%s", s);
	androidboot_mode[n] = '\0';

	return 1;
}
__setup("androidboot.mode", asustek_androidboot_mode);

static int smb347_read(struct i2c_client *client, int reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_write(struct i2c_client *client, int reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int smb347_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval | value);
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

static int smb347_clear_reg(struct i2c_client *client, int reg, u8 value)
{
	int ret, retval;

	retval = smb347_read(client, reg);
	if (retval < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, retval);
		return retval;
	}

	ret = smb347_write(client, reg, retval & (~value));
	if (ret < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return ret;
	}

	return ret;
}

int smb347_volatile_writes(struct i2c_client *client, uint8_t value)
{
	int ret = 0;

	if (value == smb347_ENABLE_WRITE) {
		/* Enable volatile write to config registers */
		ret = smb347_update_reg(client, smb347_CMD_REG, ENABLE_WRT_ACCESS);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG);
			return ret;
		}
	} else {
		ret = smb347_read(client, smb347_CMD_REG);
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}

		ret = smb347_write(client, smb347_CMD_REG, ret & (~(1<<7)));
		if (ret < 0) {
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return ret;
		}
	}
	return ret;
}

int smb347_setting_series(struct i2c_client *client)
{
	u8 ret = 0, setting;

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in smb347 volatile writes \n", __func__);
		goto error;
	}

	/* Change Input Source Priority to USBIN -> 02[2] = "1" */
	ret = smb347_update_reg(client, smb347_VRS_FUNC, 0x04);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in Priority to USBIN \n", __func__);
		goto error;
	}

	/* set Fast Charger current 2A... -> 00h = 0xAD */
	ret = smb347_write(client, smb347_CHARGE, 0xAD);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in set Fast Charger \n", __func__);
		goto error;
	}

	/* Set cold soft limit current = 900 mA -> 0Ah[7:6] = "10"*/
	setting = smb347_read(client, smb347_OTG_TLIM_REG);
	if (setting < 0) {
		dev_err(&client->dev, "%s() error in set cold soft limit \n", __func__);
		goto error;
	}

	ret = smb347_write(client, smb347_OTG_TLIM_REG, (setting & 0x3F) | 0x80);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in set cold soft limit \n", __func__);
		goto error;
	}

	/* set battery OV does not end charge cycle -> 02h[1] = "0" */
	ret = smb347_clear_reg(client, smb347_VRS_FUNC, 0x02);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in set battery OV \n", __func__);
		goto error;
	}

	/* Set Float voltage = 4.34 V -> 03h[5:0] = "101010" */
	setting = smb347_read(client, smb347_FLOAT_VLTG);
	if (setting < 0) {
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);
		goto error;
	}

	ret = smb347_write(client, smb347_FLOAT_VLTG, (setting & 0xC0) | 0x2A);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);
		goto error;
	}

	/* remove hw recharge */
	ret = smb347_update_reg(client, smb347_CHRG_CTRL, 0x80);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in Priority to USBIN \n", __func__);
		goto error;
	}

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in smb347 volatile writes \n", __func__);
		goto error;
	}

error:
	return ret;
}

/* set USB to HC mode and USB5/1/HC to register control */
int smb347_setting_USB_to_HCmode(struct i2c_client *client)
{
	u8 ret = 0;

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in smb347 volatile writes \n", __func__);
		goto error;
	}

	ret = smb347_update_reg(client, smb347_CMD_REG_B, USB_TO_HC_MODE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in set USB to HC mode \n", __func__);
		goto error;
	}

	ret = smb347_clear_reg(client, smb347_PIN_CTRL, USB_to_REGISTER_CONTROL);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);
		goto error;
        }

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in smb347 volatile writes \n", __func__);
		goto error;
	}

error:
	return ret;
}

static int smb347_pin_control(bool state)
{
	struct i2c_client *client = NULL;
	u8 ret = 0;

	mutex_lock(&charger->pinctrl_lock);

	if (charger == NULL || smb347init == false)
		return 0;

	client = charger->client;

	if (state) {
		/*Pin Controls - active low */
		ret = smb347_update_reg(client, smb347_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"enable charger\n", __func__);
		}
	} else {
		/*Pin Controls - active high */
		ret = smb347_clear_reg(client, smb347_PIN_CTRL, PIN_ACT_LOW);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed to"
						"disable charger\n", __func__);
		}
	}

	mutex_unlock(&charger->pinctrl_lock);
	return ret;
}

int smb347_charger_enable(bool state)
{
	u8 ret = 0;
	struct i2c_client *client = NULL;

	if (charger == NULL || smb347init == false)
		return 0;
	client = charger->client;

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}
	charge_en_flag = state;
	smb347_pin_control(state);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n",
								__func__);
		goto error;
	}

error:
	return ret;
}
EXPORT_SYMBOL_GPL(smb347_charger_enable);

int
smb347_set_InputCurrentlimit(struct i2c_client *client, u32 current_setting, int usb_dc)
{
	int ret = 0, retval;
	u8 setting, tempval = 0;
	project_id project = PROJECT_ID_INVALID;
	hw_rev hw_version = HW_REV_INVALID;
	hw_version = asustek_get_hw_rev();
	project = asustek_get_project_id();

	wake_lock(&charger_wakelock);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}

	if (charge_en_flag)
		smb347_pin_control(0);

	retval = smb347_read(client, smb347_VRS_FUNC);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x", __func__, smb347_VRS_FUNC);
		goto error;
	}

	setting = retval & (~(BIT(4)));
	SMB_NOTICE("Disable AICL, retval=%x setting=%x\n", retval, setting);
	ret = smb347_write(client, smb347_VRS_FUNC, setting);

	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_VRS_FUNC);
		goto error;
	}

	if (current_setting != 0) {
		retval = smb347_read(client, smb347_CHRG_CRNTS);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_CHRG_CRNTS);
			goto error;
		}

		if (usb_dc == MAX_DCIN)
			tempval = REVERTBYTE(retval);
		else if (usb_dc == MAX_USBIN)
			tempval = retval;

		/*smb347 current setting*/
		setting = tempval & 0xF0;
		if (current_setting == 2000)
			setting |= 0x07;
		else if (current_setting == 1800)
			setting |= 0x06;
		else if (current_setting == 1200)
			setting |= 0x04;
		else if (current_setting == 900)
			setting |= 0x03;
		else if (current_setting == 700)
			setting |= 0x02;
		else if (current_setting == 500)
			setting |= 0x01;

		if (usb_dc == MAX_DCIN)
			setting = REVERTBYTE(setting);

		SMB_NOTICE("Set ICL=%u retval =%x setting=%x\n", current_setting, retval, setting);

		ret = smb347_write(client, smb347_CHRG_CRNTS, setting);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
				"0x%02x\n", __func__, setting, smb347_CHRG_CRNTS);
			goto error;
		}

		if (current_setting == 2000)
			charger->curr_limit = 2000;
		else if (current_setting == 1800)
			charger->curr_limit = 1800;
		else if (current_setting == 1200)
			charger->curr_limit = 1200;
		else if (current_setting == 900)
			charger->curr_limit = 900;
		else if (current_setting == 700)
			charger->curr_limit = 700;
		else if (current_setting == 500)
			charger->curr_limit = 500;

		if (current_setting > 900)
			charger->time_of_1800mA_limit = jiffies;
		else
			charger->time_of_1800mA_limit = 0;

		retval = smb347_read(client, smb347_VRS_FUNC);
		if (retval < 0) {
			dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_VRS_FUNC);
			goto error;
		}
	}

	setting = retval | BIT(4);
	SMB_NOTICE("Re-enable AICL, setting=%x\n", setting);
	msleep(20);
	ret = smb347_write(client, smb347_VRS_FUNC, setting);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_VRS_FUNC);
		goto error;
	}

	if (charge_en_flag)
		smb347_pin_control(1);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() error in configuring charger..\n", __func__);
		goto error;
	}

error:
	wake_unlock(&charger_wakelock);
	return ret;
}

/*
static irqreturn_t smb347_inok_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}
*/

void smb347_recheck_charging_type(void)
{
	int retval;
	struct i2c_client *client = NULL;

	if (charger == NULL)
		return;
	client = charger->client;

	if (ac_on) {
		retval = AICL_read_per_min();
		if ((retval & USBIN_INPUT_CURRENT_MASK) <= USBIN_INPUT_CURRENT_LIMIT_500) {
			SMB_NOTICE("try read AICL result after 5 secs\n");
			queue_delayed_work(smb347_wq, &charger->AICL_read_work, 5*HZ);
			if ((retval & USBIN_INPUT_CURRENT_MASK) <= USBIN_INPUT_CURRENT_LIMIT_500) {
				SMB_NOTICE("reconfig input current\n");
				if (projectID == Z300CL)
					smb347_set_InputCurrentlimit(client, AC_CURRENT, MAX_USBIN);
				else
					smb347_set_InputCurrentlimit(client, AC_CURRENT_Z300CNL, MAX_USBIN);
			}
		}
	}
}
EXPORT_SYMBOL(smb347_recheck_charging_type);

static int smb347_inok_irq(struct smb347_charger *smb)
{

	/* cable detect from usb drivers */
	/*
	int err = 0 ;
	unsigned gpio = GPIO_AC_OK;
	unsigned irq_num = gpio_to_irq(gpio);

	err = gpio_request(gpio, "smb347_inok");
	if (err) {
		SMB_ERR("gpio %d request failed \n", gpio);
	}

	err = gpio_direction_input(gpio);
	if (err) {
		SMB_ERR("gpio %d unavaliable for input \n", gpio);
	}

	err = request_irq(irq_num, smb347_inok_isr, IRQF_TRIGGER_FALLING |IRQF_TRIGGER_RISING|IRQF_SHARED,
		"smb347_inok", smb);
	if (err < 0) {
		SMB_ERR("%s irq %d request failed \n","smb347_inok", irq_num);
		goto fault ;
	}
	enable_irq_wake(irq_num);
	SMB_NOTICE("GPIO pin irq %d requested ok, smb347_INOK = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");
	return 0;

fault:
	gpio_free(gpio);
	return err;
	*/
	return 0;
}

static int smb347_configure_otg(struct i2c_client *client)
{
	int ret = 0;
	project_id project = PROJECT_ID_INVALID;
	hw_rev hw_version = HW_REV_INVALID;
	hw_version = asustek_get_hw_rev();
	project = asustek_get_project_id();

	/* Allow Violate Register can be written and disable OTG */
	ret = smb347_update_reg(client, smb347_CMD_REG, 0x80);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_CMD_REG);
		goto error;
	}

	ret = smb347_clear_reg(client, smb347_CMD_REG, 0x10);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_CMD_REG);
		goto error;
	}

	/* Switching Frequency change to 1.5Mhz */
	ret = smb347_update_reg(client, smb347_THERM_CTRL, 0x80);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_THERM_CTRL);
		goto error;
	}

	/* 09h OTG/ID pin control */
	ret = smb347_clear_reg(client, smb347_CTRL_REG, 0xC0);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_CTRL_REG);
		goto error;
	}

	/* Change "OTG output current limit" to 250mA */
	ret = smb347_write(client, smb347_OTG_TLIM_REG, 0x34);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_OTG_TLIM_REG);
		goto error;
	}

	/* Enable OTG */
	ret = smb347_update_reg(client, smb347_CMD_REG, 0x10);
	if (ret < 0) {
		dev_err(&client->dev, "%s: Failed in writing register"
			"0x%02x\n", __func__, smb347_CMD_REG);
		goto error;
	}

	/* Change "OTG output current limit" from 250mA to 500mA */
	ret = smb347_write(client, smb347_OTG_TLIM_REG, 0x0C);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_OTG_TLIM_REG);
		goto error;
	}

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
		goto error;
	}

	return 0;

error:
	return ret;
}

void smb347_vbus_status(bool on)
{
	int ret;
	struct i2c_client *client = NULL;

	if (charger == NULL) {
		return;
	}
	client = charger->client;

	ret = smb347_update_reg(client, smb347_CMD_REG, 0x80);

	if (on) {
		ret = smb347_update_reg(client, smb347_CMD_REG, 0x04);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG);
			ret = smb347_clear_reg(client, smb347_CMD_REG, 0x80);
			return;
		}
	} else {
		ret = smb347_clear_reg(client, smb347_CMD_REG, 0x04);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing"
				"register 0x%02x\n", __func__, smb347_CMD_REG);
			ret = smb347_clear_reg(client, smb347_CMD_REG, 0x80);
			return;
		}
	}
	ret = smb347_clear_reg(client, smb347_CMD_REG, 0x80);
	return;
}

void smb347_otg_status(bool on)
{
	int ret;
	struct i2c_client *client = NULL;

	otg_on = on;
	if (charger == NULL) {
		return;
	}
	client = charger->client;

	SMB_NOTICE("otg function: %s\n", on ? "on" : "off");

	if (on) {
		otg_on = true;
		/* ENABLE OTG */
		ret = smb347_configure_otg(client);
		if (ret < 0) {
			dev_err(&client->dev, "%s() error in configuring"
				"otg..\n", __func__);
			return;
		}
	} else
		otg_on = false;

}
EXPORT_SYMBOL_GPL(smb347_otg_status);

int smb347_otg_complete(struct i2c_client *client)
{
	int ret;

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	/* Change "OTG output current limit" to 250mA */
	ret = smb347_clear_reg(client, smb347_OTG_TLIM_REG, 0x0c);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_OTG_TLIM_REG);
		goto error;
	}

	smb347_otg_status(false);

	/* IC Reset to default */
	ret = smb347_update_reg(client, smb347_CMD_REG_B, 0x80);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_OTG_TLIM_REG);
		goto error;
	}

	/* Disable volatile writes to registers */
	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s error in configuring OTG..\n",
								__func__);
		goto error;
	}
	return 0;
error:
	return ret;
}

static void workqueue_setting_input_current(struct work_struct *work)
{
	struct smb347_charger *charger = container_of(work, struct smb347_charger, cable_det_work.work);
	struct i2c_client *client = charger->client;
	int retval = 0;

	if (cable_state_detect == CHARGER_NONE) {
		smb347_vbus_status(false);
		if (otg_on == true)
			smb347_otg_complete(client);
	} else if (cable_state_detect == CHARGER_CDP || cable_state_detect == CHARGER_DCP || cable_state_detect == CHARGER_ACA) {
#ifndef FACTORY_IMAGE
		smb347_setting_series(client);
#endif
		charger->cur_cable_type = ac_cable;

		/* porting guide IUSB_IN (01h[3:0]) < 1200 mA */
		retval = smb347_read(client, smb347_CHRG_CRNTS);

                if (retval < 0)
			dev_err(&client->dev, "%s() IUSB_IN reads fail \n", __func__);

		retval = (retval & USBIN_INPUT_CURRENT_MASK);

		if (projectID == Z300CL) {
			if (retval != USBIN_INPUT_CURRENT_LIMIT_1200) {
				SMB_NOTICE("Z300CL: IUSB_IN=%x\n", retval);
				smb347_set_InputCurrentlimit(client, AC_CURRENT, MAX_USBIN);
			}
		} else {
                        if (retval != USBIN_INPUT_CURRENT_LIMIT_900) {
                                SMB_NOTICE("Z300CNL: IUSB_IN=%x\n", retval);
                                smb347_set_InputCurrentlimit(client, AC_CURRENT_Z300CNL, MAX_USBIN);
                        }
		}
//		smb347_setting_USB_to_HCmode(client);
	} else if (cable_state_detect == CHARGER_SDP) {

#ifndef FACTORY_IMAGE
		smb347_setting_series(client);
#endif
		charger->cur_cable_type = usb_cable;
		smb347_set_InputCurrentlimit(client, USB_CURRENT, MAX_USBIN);
	} else if (cable_state_detect == CHARGER_OTG)
		smb347_otg_status(true);
}

int smb347_float_volt_set(unsigned int val)
{
	struct i2c_client *client = charger->client;
	int ret = 0, retval;

	if (val > 4500 || val < 3500)
		SMB_ERR("%s(): val=%d is out of range !\n",__func__, val);

	SMB_NOTICE("%s(): val=%d\n", __func__, val);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0) {
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);
		goto fault;
	}

	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_FLOAT_VLTG);
		goto fault;
	}
	retval = retval & (~FLOAT_VOLT_MASK);
	val = clamp_val(val, 3500, 4500) - 3500;
	val /= 20;
	retval |= val;
	ret = smb347_write(client, smb347_FLOAT_VLTG, retval);
	if (ret < 0) {
		dev_err(&client->dev, "%s(): Failed in writing"
			"register 0x%02x\n", __func__, smb347_FLOAT_VLTG);
		goto fault;
	}

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger disable write error..\n", __func__);
	return 0;
fault:
	return ret;
}
EXPORT_SYMBOL_GPL(smb347_float_volt_set);

int cable_status_notify2(struct notifier_block *self, unsigned long action, void *dev)
{
	struct i2c_client *client = NULL;
	int  success = 0, ret;

	if (charger == NULL)
		return 0;
	client = charger->client;

	ret = cancel_delayed_work(&charger->cable_det_work);
	if (ret < 0)
		dev_err(&client->dev, "%s cancel delay work fail\n", __func__);

	switch (action) {
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		printk(KERN_INFO "%s CHRG_UNKNOWN !!!\n", __func__);
		cable_state_detect = CHARGER_NONE;
		charger->cur_cable_type = unknow_cable;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.01*HZ);
		success = bq27320_battery_callback(non_cable);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		printk(KERN_INFO "%s CHRG_SDP !!!\n", __func__);
		cable_state_detect = CHARGER_SDP;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		success = bq27320_battery_callback(usb_cable);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		printk(KERN_INFO "%s CHRG_CDP !!!\n", __func__);
		cable_state_detect = CHARGER_CDP;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		success =  bq27320_battery_callback(ac_cable);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		printk(KERN_INFO "%s CHRG_DCP !!!\n", __func__);
		cable_state_detect = CHARGER_DCP;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		success =  bq27320_battery_callback(ac_cable);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		printk(KERN_INFO "%s CHRG_ACA !!!\n", __func__);
		cable_state_detect = CHARGER_ACA;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		success =  bq27320_battery_callback(ac_cable);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_OTG:
		printk(KERN_INFO "%s CHRG_OTG !!!\n", __func__);
		cable_state_detect = CHARGER_OTG;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		break;
	case POWER_SUPPLY_CHARGER_TYPE_SE1:
		printk(KERN_INFO "%s CHRG_SE1 !!!\n", __func__);
		cable_state_detect = CHARGER_DCP;
		queue_delayed_work(smb347_wq, &charger->cable_det_work, 0.5*HZ);
		success =  bq27320_battery_callback(ac_cable);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}
EXPORT_SYMBOL_GPL(cable_status_notify2);

static struct notifier_block cable_status_notifier2 = {
        .notifier_call = cable_status_notify2,
};

int cable_detect_callback(unsigned int cable_state)
{
	cable_state_detect = cable_state;
	if (charger->cur_cable_type == unknow_cable)
		cable_state_detect = CHARGER_NONE;
	return 0;
}
EXPORT_SYMBOL_GPL(cable_detect_callback);


int AICL_read_per_min(void)
{
	int retval;
	struct i2c_client *client = NULL;

	if (charger == NULL)
		return 0;

	client = charger->client;
	retval = smb347_read(client, smb347_STS_REG_E);

	if (retval < 0)
		dev_err(&client->dev, "%s() AICL result reading fail \n", __func__);

	SMB_NOTICE("smb347 AICL result = %x\n", retval);

	return retval;
}

/* Sysfs function */
static ssize_t smb347_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	char tmp_buf[64];
	int i,ret;

	sprintf(tmp_buf, "SMB347 Configuration Registers Detail\n"
						"==================\n");
	strcpy(buf, tmp_buf);

	for (i = 0; i <= 14; i++) {
		ret = smb347_read(client, smb347_CHARGE+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", smb347_CHARGE+i, ret);
		strcat(buf, tmp_buf);
	}
	for (i = 0; i <= 1; i++) {
		ret = smb347_read(client, smb347_CMD_REG+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", smb347_CMD_REG+i, ret);
		strcat(buf, tmp_buf);
	}
	for (i = 0; i <= 10; i++) {
		ret = smb347_read(client, smb347_INTR_STS_A+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", smb347_INTR_STS_A+i, ret);
		strcat(buf, tmp_buf);
	}

	return strlen(buf);
}

void recharge_function(void)
{
	int retval;
	struct i2c_client *client = NULL;

	if (charger == NULL)
		return;

	client = charger->client;

	retval = smb347_read(client, smb347_STS_REG_C);
	if (retval < 0) {
		dev_err(&client->dev, "%s() recharge read fail\n", __func__);
	}

	SMB_NOTICE("RSOC = %d, 3Dh = 0x%02x\n", RSOC, retval);
	/* At least one charging cycle has terminated 3dh[5]=1 */
        if ((RSOC <= 98) && ((retval & 0x20) == 0x20)) {
		SMB_NOTICE("recharge on, RSOC = %d, 3Dh = 0x%02x\n", RSOC, retval);
		smb347_charger_enable(false);
		msleep(200);
		smb347_charger_enable(true);

	}
}

int smb347_config_thermal_limit(void)
{
	struct i2c_client *client = NULL;
	int ret = 0;

	if (charger == NULL)
		return 0;

	client = charger->client;

	/* SOC Control JEITA Function & Charger do second protection */
	/* Set Hard Hot Limit = 72 Deg. C -> 0Bh[5:4]="11" */
	ret = smb347_update_reg(client, smb347_HRD_SFT_TEMP, 0x30);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Hard Hot Limit = 72\n", __func__);

	/* Set Soft Hot Limit Behavior = No Response -> 07h[1:0]="00" */
	ret = smb347_clear_reg(client, smb347_THERM_CTRL, 0x03);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Hard Hot Limit = 72\n", __func__);

	/* Set Soft Cold temp limit = No Response -> 07h[3:2]="00 */
	ret = smb347_clear_reg(client, smb347_THERM_CTRL, 0x0C);
	if (ret < 0)
	        dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	return 0;
}

void JEITA_rule_1(void)
{
	/* temp < 1.5 || temp >= 55 */
	/* Vchg=4.34V, Charging Disable, Set Fast Charge Current = 2000 mA */
	struct i2c_client *client = charger->client;
	int ret = 0, retval, setting = 0;

	JEITA_flag = JEITA_flag_1;
	mutex_lock(&smb347_shutdown_mutex);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	/* JEITA flowchart start */
	smb347_config_thermal_limit();

	/*control float voltage*/
	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
			__func__, smb347_FLOAT_VLTG);
	}

	setting = retval & FLOAT_VOLT_MASK;
	if (setting != FLOAT_VOLT_434V) {
		setting = retval & (~FLOAT_VOLT_MASK);
		setting |= FLOAT_VOLT_434V;
		SMB_NOTICE("Set Float Volt, retval=%x setting=%x\n", retval, setting);
		ret = smb347_write(client, smb347_FLOAT_VLTG, setting);
		if (ret < 0) {
		        dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_FLOAT_VLTG);
		}
	} else
		SMB_NOTICE("Bypass set Float Volt=%x\n", retval);

	/* Set Fast Charge Current = 2000 mA */
	setting = smb347_read(client, smb347_CHARGE);
	if (setting < 0)
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);

	ret = smb347_write(client, smb347_CHARGE, (setting & 0x1F) | 0xA0);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	mutex_unlock(&smb347_shutdown_mutex);

	/* Charging Disable */
	smb347_charger_enable(false);
	SMB_NOTICE("Charger disable JEITA rule 1\n");
}

void JEITA_rule_2(void)
{
	/* 1.5 <= temp < 15 */
	/* Vchg=4.34V , Charging Enable, Set Fast Charge Current = 900 mA */
	struct i2c_client *client = charger->client;
	int ret = 0, retval, setting = 0;

	JEITA_flag = JEITA_flag_2;
	mutex_lock(&smb347_shutdown_mutex);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	/* JEITA flowchart start */
	smb347_config_thermal_limit();

	/*control float voltage*/
	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
			__func__, smb347_FLOAT_VLTG);
	}

	setting = retval & FLOAT_VOLT_MASK;
	if (setting != FLOAT_VOLT_434V) {
		setting = retval & (~FLOAT_VOLT_MASK);
		setting |= FLOAT_VOLT_434V;
		SMB_NOTICE("Set Float Volt, retval=%x setting=%x\n", retval, setting);
		ret = smb347_write(client, smb347_FLOAT_VLTG, setting);
		if (ret < 0) {
		        dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_FLOAT_VLTG);
		}
	} else
		SMB_NOTICE("Bypass set Float Volt=%x\n", retval);

	/* Set Fast Charge Current = 900 mA */
	setting = smb347_read(client, smb347_CHARGE);
	if (setting < 0)
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);

	ret = smb347_write(client, smb347_CHARGE, (setting & 0x1F) | 0x20);

	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	mutex_unlock(&smb347_shutdown_mutex);

	/* Charging enable */
	smb347_charger_enable(true);
	SMB_NOTICE("Charger enable JEITA rule 2\n");
	recharge_function();
}

void JEITA_rule_3(void)
{
	/* 15 <= temp < 50 */
	/* Vchg=4.34V , Charging Enable, Set Fast Charge Current = 2000 mA*/

	struct i2c_client *client = charger->client;
	int ret = 0, retval, setting = 0;

	JEITA_flag = JEITA_flag_3;
	mutex_lock(&smb347_shutdown_mutex);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	/* JEITA flowchart start */
	smb347_config_thermal_limit();

	/*control float voltage*/
	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_FLOAT_VLTG);
	}

	setting = retval & FLOAT_VOLT_MASK;
	if (setting != FLOAT_VOLT_434V) {
		setting = retval & (~FLOAT_VOLT_MASK);
		setting |= FLOAT_VOLT_434V;
		SMB_NOTICE("Set Float Volt, retval=%x setting=%x\n", retval, setting);
		ret = smb347_write(client, smb347_FLOAT_VLTG, setting);
		if (ret < 0) {
		        dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_FLOAT_VLTG);
		}
	} else
		SMB_NOTICE("Bypass set Float Volt=%x\n", retval);

	/* Set Fast Charge Current = 2000 mA */
	setting = smb347_read(client, smb347_CHARGE);
	if (setting < 0)
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);

	ret = smb347_write(client, smb347_CHARGE, (setting & 0x1F) | 0xA0);

	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);

        ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
        if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	mutex_unlock(&smb347_shutdown_mutex);

	/* Charging enable */
	smb347_charger_enable(true);
	SMB_NOTICE("Charger enable JEITA rule 3\n");
	recharge_function();
}

void JEITA_rule_4(void)
{
	/* 50 <= temp < 55 */
	/* Vchg=4.11V , Charging Enable,  Set Fast Charge Current = 2000 mA*/
	/* Vrech = Vflt - 100mV */

	struct i2c_client *client = charger->client;
	int ret = 0, retval, setting = 0;

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	JEITA_flag = JEITA_flag_4;
	mutex_lock(&smb347_shutdown_mutex);

	/*control float voltage*/
	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_FLOAT_VLTG);
	}

	/* JEITA flowchart start */
	smb347_config_thermal_limit();

	setting = retval & FLOAT_VOLT_MASK;
	if (setting != FLOAT_VOLT_410V) {
		setting = retval & (~FLOAT_VOLT_MASK);
		setting |= FLOAT_VOLT_410V;
		SMB_NOTICE("Set Float Volt, retval=%x setting=%x\n", retval, setting);
		ret = smb347_write(client, smb347_FLOAT_VLTG, setting);
		if (ret < 0) {
			dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_FLOAT_VLTG);
		}
	} else
		SMB_NOTICE("Bypass set Float Volt=%x\n", retval);
	/* Set Fast Charge Current = 2000 mA */
	setting = smb347_read(client, smb347_CHARGE);
	if (setting < 0)
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);

	ret = smb347_write(client, smb347_CHARGE, (setting & 0x1F) | 0xA0);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);

	/* Vrech = Vflt - 100mV -> 04h[3] = 1*/
	ret = smb347_update_reg(client, smb347_CHRG_CTRL, 0x08);

	if (ret < 0)
		dev_err(&client->dev, "%s() error in Vrech\n", __func__);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	mutex_unlock(&smb347_shutdown_mutex);

	/* Charging enable */
	smb347_charger_enable(true);
	SMB_NOTICE("Charger enable JEITA rule 4\n");
}

void JEITA_rule_5(void)
{
	/* 50 <= temp < 55 */
	/* Vchg=4.35V , Charging Disable, Set Fast Charge Current = 2000 mA*/

	struct i2c_client *client = charger->client;
	int ret = 0, retval, setting = 0;

	JEITA_flag = JEITA_flag_5;
	mutex_lock(&smb347_shutdown_mutex);

	ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	/* JEITA flowchart start */
	smb347_config_thermal_limit();

	/*control float voltage*/
	retval = smb347_read(client, smb347_FLOAT_VLTG);
	if (retval < 0) {
		dev_err(&client->dev, "%s(): Failed in reading 0x%02x",
				__func__, smb347_FLOAT_VLTG);
	}

	setting = retval & FLOAT_VOLT_MASK;
	if (setting != FLOAT_VOLT_434V) {
		setting = retval & (~FLOAT_VOLT_MASK);
		setting |= FLOAT_VOLT_434V;
		SMB_NOTICE("Set Float Volt, retval=%x setting=%x\n", retval, setting);
		ret = smb347_write(client, smb347_FLOAT_VLTG, setting);
		if (ret < 0) {
		        dev_err(&client->dev, "%s(): Failed in writing 0x%02x to register"
			"0x%02x\n", __func__, setting, smb347_FLOAT_VLTG);
		}
	} else
		SMB_NOTICE("Bypass set Float Volt=%x\n", retval);

	/* Set Fast Charge Current = 2000 mA */
	setting = smb347_read(client, smb347_CHARGE);

	if (setting < 0)
		dev_err(&client->dev, "%s() error in smb347 read \n", __func__);

	ret = smb347_write(client, smb347_CHARGE, (setting & 0x1F) | 0xA0);
	if (ret < 0)
		dev_err(&client->dev, "%s() error in Set Float voltage \n", __func__);

	ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
	if (ret < 0)
		dev_err(&client->dev, "%s() charger enable write error..\n", __func__);

	mutex_unlock(&smb347_shutdown_mutex);

	/* Charging enable */
	smb347_charger_enable(false);
	SMB_NOTICE("Charger disable JEITA rule 5\n");
}

int smb347_config_thermal_charging(int temp, int volt, int rule)
{
	struct i2c_client *client = NULL;
	int ret = 0, retval, setting = 0;

	if (JEITA_early_suspend == true) {
		SMB_NOTICE("JEITA_early_suspend is true, pass JEITA rule setting\n");
		JEITA_early_suspend = false;
		return 0;
	}

	if (charger == NULL)
		return 0;

	client = charger->client;

	SMB_NOTICE("temp=%d, volt=%d\n", temp, volt);

	if (JEITA_init == false) {
		if (temp < 1.5 && (JEITA_flag != JEITA_flag_1))
			JEITA_rule_1();
		else if (temp >= 1.5 && temp < 10 && (JEITA_flag != JEITA_flag_2))
			JEITA_rule_2();
		else if (temp >= 15 && temp < 50 && (JEITA_flag != JEITA_flag_3))
			JEITA_rule_3();
		else if (temp >= 50 && temp < 55) {
			ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
			if (ret < 0)
				dev_err(&client->dev, "%s() charger enable write error..\n", __func__);
			/*control float voltage*/
			retval = smb347_read(client, smb347_FLOAT_VLTG);
			if (ret < 0)
				dev_err(&client->dev, "%s(): Failed in reading 0x%02x", __func__, smb347_FLOAT_VLTG);
			setting = retval & FLOAT_VOLT_MASK;
			ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
			if (ret < 0)
				dev_err(&client->dev, "%s() charger disable write error..\n", __func__);
			if (setting == FLOAT_VOLT_434V && volt >= 4100000 && (JEITA_flag != JEITA_flag_5))
				JEITA_rule_5();
			else if (JEITA_flag != JEITA_flag_4)
				JEITA_rule_4();
		} else if (temp >= 55 && (JEITA_flag != JEITA_flag_1))
			JEITA_rule_1();
		JEITA_init = true;
	} else {
		if (temp < 1.5 && (JEITA_flag != JEITA_flag_1))
			JEITA_rule_1();
		else if (temp > 4.5 && temp < 15 && (JEITA_flag != JEITA_flag_2))
			JEITA_rule_2();
		else if (temp > 18 && temp < 47 && (JEITA_flag != JEITA_flag_3))
			JEITA_rule_3();
		else if (temp >= 50 && temp < 52) {
			ret = smb347_volatile_writes(client, smb347_ENABLE_WRITE);
			if (ret < 0)
				dev_err(&client->dev, "%s() charger enable write error..\n", __func__);
			/*control float voltage*/
			retval = smb347_read(client, smb347_FLOAT_VLTG);
			if (ret < 0)
				dev_err(&client->dev, "%s(): Failed in reading 0x%02x", __func__, smb347_FLOAT_VLTG);
			setting = retval & FLOAT_VOLT_MASK;
			ret = smb347_volatile_writes(client, smb347_DISABLE_WRITE);
			if (ret < 0)
				dev_err(&client->dev, "%s() charger disable write error..\n", __func__);
			if (setting == FLOAT_VOLT_434V && volt >= 4100000 && (JEITA_flag != JEITA_flag_5))
				JEITA_rule_5();
			else if (JEITA_flag != JEITA_flag_4)
				JEITA_rule_4();
		} else if (temp >= 55 && (JEITA_flag != JEITA_flag_1))
			JEITA_rule_1();
	}
	SMB_NOTICE("JEITA monitor : temp=%d, volt=%d, JEITA flag=%d\n", temp, volt, JEITA_flag);
	return 0;
}
EXPORT_SYMBOL(smb347_config_thermal_charging);

static ssize_t smb347_input_AICL_result_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger->client;
	int ret, index, result = 0;

	ret = i2c_smbus_read_byte_data(client, smb347_STS_REG_E);
	if (ret < 0)
		return sprintf(buf, "i2c read register fail\n");

	if (ret & 0x01) {
		index = ret & 0x0F;
		switch (index) {
		case 0:
			result = 300;
			break;
		case 1:
			result = 500;
			break;
		case 2:
			result = 700;
			break;
		case 3:
			result = 900;
			break;
		case 4:
			result = 1200;
			break;
		case 5:
			result = 1300;
			break;
		case 6:
			result = 1800;
			break;
		case 7:
			result = 2000;
			break;
		case 8:
			result = 2000;
			break;
		default:
			result = 2000;
		}
	} else
		return sprintf(buf, "AICL is not completed\n");

	return sprintf(buf, "%d\n", result);
}

static ssize_t show_smb347_charger_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	if (!smb347_charger_status)
		return sprintf(buf, "%d\n", 0);
	else
		return sprintf(buf, "%d\n", 1);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
void smb347_early_suspend(struct early_suspend *h)
{
	SMB_NOTICE("smb347_early_suspend+\n");
	flush_workqueue(smb347_wq);
	JEITA_early_suspend = true;
	SMB_NOTICE("smb347_early_suspend-\n");
}

void smb347_late_resume(struct early_suspend *h)
{
	SMB_NOTICE("smb347_late_resume+\n");
	JEITA_early_suspend = false;
	SMB_NOTICE("smb347_late_resume-\n");
}
#endif

static int smb347_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	project_id project = PROJECT_ID_INVALID;
	hw_rev hw_version = HW_REV_INVALID;
	hw_version = asustek_get_hw_rev();
	project = asustek_get_project_id();
	projectID = project;

	printk("smb347 probe init\n");

	SMB_NOTICE("smb347 project = %d, hw_version =%d\n", project, hw_version);

	charger = kzalloc(sizeof(*charger), GFP_KERNEL);
	if (!charger)
		return -ENOMEM;

	printk("smb347_probe+\n");
	charger->client = client;
	charger->dev = &client->dev;
	i2c_set_clientdata(client, charger);

	ret = sysfs_create_group(&client->dev.kobj, &smb347_group);
	if (ret) {
		SMB_ERR("Unable to create the sysfs\n");
		goto error;
	}

	mutex_init(&charger->apsd_lock);
	mutex_init(&charger->usb_lock);
	mutex_init(&charger->pinctrl_lock);

	smb347_wq = create_singlethread_workqueue("smb347_wq");

	INIT_DELAYED_WORK(&charger->AICL_read_work, AICL_read_per_min);
	INIT_DELAYED_WORK(&charger->cable_det_work, workqueue_setting_input_current);
	cable_status_register_client(&cable_status_notifier2);

	wake_lock_init(&charger_wakelock, WAKE_LOCK_SUSPEND,
			"charger_configuration");
	wake_lock_init(&COS_wakelock, WAKE_LOCK_SUSPEND, "COS_wakelock");
	charger->curr_limit = UINT_MAX;
	charger->cur_cable_type = non_cable;
	charger->old_cable_type = non_cable;
	disable_DCIN = false;
	smb347_charger_status = 1;
#ifdef CONFIG_HAS_EARLYSUSPEND
	charger->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	charger->early_suspend.suspend = smb347_early_suspend;
	charger->early_suspend.resume = smb347_late_resume;
	register_early_suspend(&charger->early_suspend);
#endif

	ret = smb347_inok_irq(charger);
	if (ret) {
		SMB_ERR("Failed in requesting ACOK# pin isr\n");
		goto error;
	}

        if (strcmp(androidboot_mode, "charger") == 0) {
		SMB_NOTICE("COS: wake lock in charger\n");
		wake_lock(&COS_wakelock);
        }

	printk("smb347_probe-\n");
	smb347init = true;

	return 0;
error:
	kfree(charger);
	return ret;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *charger = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&charger->early_suspend);
#endif
	kfree(charger);
	return 0;
}

void smb347_shutdown(struct i2c_client *client)
{
	mutex_lock(&smb347_shutdown_mutex);
	smb347_shutdown_value = true;
	printk("smb347_shutdown+\n");
	//kfree(charger);
	//charger = NULL;
	printk("smb347_shutdown-\n");
	mutex_unlock(&smb347_shutdown_mutex);
}

static const struct i2c_device_id smb347_id[] = {
	{ "smb347_charger", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

static struct i2c_driver smb347_i2c_driver = {
	.driver	= {
		.name	= "smb347_charger",
	},
	.probe		= smb347_probe,
	.remove		= smb347_remove,
	.shutdown	= smb347_shutdown,
	.id_table	= smb347_id,
};

static int __init smb347_init(void)
{
	printk("smb347 init\n");
	i2c_add_driver(&smb347_i2c_driver);
	return 0;
};
late_initcall(smb347_init);

static void __exit smb347_exit(void)
{
	printk("smb347 exit\n");
	i2c_del_driver(&smb347_i2c_driver);
}
module_exit(smb347_exit);

MODULE_DESCRIPTION("smb347 Battery-Charger");
MODULE_LICENSE("GPL");
