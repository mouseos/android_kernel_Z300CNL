/*
 * drivers/power/bq27320_battery.c
 *
 * Gas Gauge driver for TI's BQ27320
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/board_asustek.h>
#include <linux/switch.h>

#define GAUGE_UPDATE_DELAY       20
#define GAUGE_FILE	         "/system/etc/firmware/Z300_MP_ATL_4890_v10.df.fs"
#define GAUGE_FW                 0x3031
#define GAUGE_Z300CNL_LG_FILE    "/system/etc/firmware/Z300CN_MP_LGC_4680_3725_V1.2_20161115.df.fs"
#define GAUGE_Z300CNL_LG_FW      0x3231
#define GAUGE_Z300CNL_SDI_FILE   "/system/etc/firmware/Z300CN_MP_SDI_4680_3676_V1.1_20161114.df.fs"
#define GAUGE_Z300CNL_SDI_FW     0x3131
#define LG_UPPER		 12000
#define LG_LOWER		 8000
#define SDI_UPPER		 2500
#define SDI_LOWER		 500

#define SMBUS_RETRY                                     (0)
#define GPIOPIN_LOW_BATTERY_DETECT	  29
#define BATTERY_POLLING_RATE	       60
#define BATTERY_POLLING_RATE_FIRST     5
#define DELAY_FOR_CORRECT_CHARGER_STATUS	(5)
#define TEMP_KELVIN_TO_CELCIUS		(2731)

#define MAXIMAL_VALID_BATTERY_TEMP	(1200)
#define BATTERY_PROTECTED_VOLT	(2800)
#define USB_NO_Cable	0
#define USB_DETECT_CABLE	1
#define USB_SHIFT	0
#define AC_SHIFT	1
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
#define DOCK_AC 4

#define BATTERY_MANUFACTURER_SIZE	12
#define BATTERY_NAME_SIZE		8

/* Battery flags bit definitions */
#define BATT_REG_CNTL	        0x00
#define BATT_SUBCMD_FW_VER      0x0002
#define BATT_STS_DSG		0x0001
#define BATT_STS_SOCF		0x0002
#define BATT_STS_FC		0x0200

/* Battery Control Subcommands */
#define CONTROL_STATUS		0x0000
#define DEVICE_TYPE		0x0001
#define FW_VERSION		0x0002
#define DEVICE_NAME		0x004A

#define THERMAL_RULE1 1
#define THERMAL_RULE2 2

/* Project definition */
#define ME581CL 0
#define FE375CG 1
#define ME572C  2
#define ME572CL 3
#define Z300CL  8
#define Z300CNL 10

/* Debug Message */
#define BAT_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "%s " format , __FUNCTION__ , ## arg)

#define BAT_ERR(format, arg...)		\
	printk(KERN_ERR format , ## arg)

/* Battery state-of-health: bq27320-G1*/
#define FCC    0x12
#define DC     0x3C
#define RM     0x10
#define CC     0x2A
#define SOH    0x2E

/* Global variable */
unsigned bq27320_battery_cable_status = 0;
unsigned bq27320_battery_driver_ready = 0;
int ac_on;
int usb_on;
int RSOC;
extern int stand_capacity;
int last_stand_capacity;
//extern bool dock_ac;
//extern bool dock_in;
extern bool otg_on;
static unsigned int	battery_current;
static unsigned int  battery_remaining_capacity;
struct workqueue_struct *bq27320_battery_work_queue = NULL;
static unsigned int battery_check_interval = BATTERY_POLLING_RATE;
static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
extern bool smb347_shutdown_value;
extern char androidboot_mode[32];

int reboot_cmd = 0;
int old_cable_status;

project_id project;

/* Functions declaration */
static int bq27320_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);
static int bq27320_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
extern unsigned  get_usb_cable_status(void);
extern int smb347_config_thermal_charging(int temp, int volt, int rule);
extern void smb347_recheck_charging_type(void);
extern unsigned int query_cable_status(void);
extern int smb347_charger_enable(bool enable);
//extern int stand_polling(void);
static int auto_flash_gauge_check();
extern int pmic_get_battery_id(int *batt_id);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#define bq27320_DATA(_psp, _addr, _min_value, _max_value)	\
	{								\
		.psp = POWER_SUPPLY_PROP_##_psp,	\
		.addr = _addr,				\
		.min_value = _min_value,		\
		.max_value = _max_value,	\
	}

enum gaugeIC_update_ret_value{
        error_sleep = -4,
        error_check = -3,
        error_write = -2,
        error_read = -1,
        update_success = 0,
};

enum gaugeIC_mode_enum{
        may_broken = 0,
        ROM_mode = 1,
        normal_mode = 2,
};

enum gaugeIC_update_command{
        update_command_read = 0x00,
        update_command_write = 0x01,
        update_command_sleep = 0x02,
};

enum {
       REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_PACK_BAT,
	Charger_Type_PACK_AC,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static struct bq27320_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27320_data[] = {
       [REG_MANUFACTURER_DATA]	= bq27320_DATA(PRESENT, 0, 0, 65535),
       [REG_STATE_OF_HEALTH]		= bq27320_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE]			= bq27320_DATA(TEMP, 0x06, 0, 65535),
	[REG_VOLTAGE]				= bq27320_DATA(VOLTAGE_NOW, 0x08, 0, 6000),
	[REG_CURRENT]				= bq27320_DATA(CURRENT_NOW, 0x14, -32768, 32767),
	[REG_TIME_TO_EMPTY]			= bq27320_DATA(TIME_TO_EMPTY_AVG, 0x16, 0, 65535),
	[REG_TIME_TO_FULL]			= bq27320_DATA(TIME_TO_FULL_AVG, 0x18, 0, 65535),
	[REG_STATUS]				= bq27320_DATA(STATUS, 0x0a, 0, 65535),
	[REG_CAPACITY]				= bq27320_DATA(CAPACITY, 0x2c, 0, 100),
};

static enum power_supply_property bq27320_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static enum power_supply_property pack_bat_properties[] = {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_CAPACITY,
};

unsigned get_cable_status(void)
{
	return bq27320_battery_cable_status;
}

void bq27320_check_cabe_type(void)
{
      if(bq27320_battery_cable_status == USB_AC_Adapter) {
		ac_on  = 1;
	        usb_on = 0;
	} else if (bq27320_battery_cable_status == USB_Cable) {
		ac_on = 0;
		usb_on = 1;
	} else {
		ac_on = 0;
		usb_on = 0;
	}
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#if 0
static int pack_bat_get_property(struct power_supply *psy,
        enum power_supply_property psp, union power_supply_propval *val)
{
        int ret=0;
        switch (psp) {

	case POWER_SUPPLY_PROP_STATUS:
/*
                if (dock_in)
                        if (stand_capacity == 100) {
                                val->intval = POWER_SUPPLY_STATUS_FULL;
                        } else {
                                val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
                        }
                } else if (dock_ac && dock_in) {
			if (stand_capacity == 100) {
				val->intval = POWER_SUPPLY_STATUS_FULL;
			} else {
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			}
                }
		break;
*/
        case POWER_SUPPLY_PROP_PRESENT:
                if(psy->type == POWER_SUPPLY_TYPE_PACK_BAT && dock_in)
                        val->intval = 1;
                else
                        val->intval = 0;
                break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = last_stand_capacity;
		if(psy->type == POWER_SUPPLY_TYPE_PACK_BAT && dock_in)
			val->intval = last_stand_capacity = stand_capacity;
		else
			val->intval = last_stand_capacity;
		break;
        default:
                return -EINVAL;
        }
        return ret;
}
#endif

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	int ret=0;
	switch (psp) {

	case POWER_SUPPLY_PROP_ONLINE:
		if(psy->type == POWER_SUPPLY_TYPE_MAINS && ac_on)
			val->intval = 1;
		else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval = 1;
//		else if (psy->type == POWER_SUPPLY_TYPE_PACK_AC && dock_ac)
//			val->intval = 1;
		else
			val->intval = 0;
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

static char *supply_list[] = {
	"battery",
	"ac",
	"usb",
	"pack_bat"
	"pack_ac",
};

static struct power_supply bq27320_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= bq27320_properties,
		.num_properties = ARRAY_SIZE(bq27320_properties),
		.get_property	= bq27320_get_property,
	},
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties     = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property	= power_get_property,
	},
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties     = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property   = power_get_property,
	},
/*	{
                .name           = "pack_bat",
                .type           = POWER_SUPPLY_TYPE_PACK_BAT,
                .supplied_to    = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties     = pack_bat_properties,
                .num_properties = ARRAY_SIZE(pack_bat_properties),
                .get_property   = pack_bat_get_property,
        },
        {
                .name           = "pack_ac",
                .type           = POWER_SUPPLY_TYPE_PACK_AC,
                .supplied_to    = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties     = power_properties,
                .num_properties = ARRAY_SIZE(power_properties),
                .get_property   = power_get_property,
        },
*/
};

static DEFINE_MUTEX(lock);

static struct bq27320_device_info {
	struct i2c_client *client;
	struct i2c_client *ROM_mode_client;
	struct delayed_work battery_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct delayed_work gauge_update_work;
	struct miscdevice battery_misc;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_wake_lock;
	struct switch_dev gauge_sdev;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
	int bat_limit_off;
	int bat_capacity_zero_count;
	int bat_APD_check;
	unsigned int old_capacity;
	unsigned int cap_err;
	int old_temperature;
	bool temp_err;
	unsigned int prj_id;
	spinlock_t lock;
} *bq27320_device;

static int bq27320_read_i2c(int reg, int *rt_value, int b_single)
{
	struct i2c_client *client = bq27320_device->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
	return err;
}

int bq27320_smbus_read_data(int reg_offset,int byte,int *rt_value)
{
     s32 ret=-EINVAL;
     int count=0;

	mutex_lock(&lock);
	do {
		ret = bq27320_read_i2c(reg_offset, rt_value, byte);
	} while((ret<0)&&(++count<=SMBUS_RETRY));
	mutex_unlock(&lock);
	return ret;
}

static int close_file(struct file *fp)
{
        filp_close(fp,NULL);

        return 0;
}

static struct file *open_file(char *path,int flag,int mode)
{
        struct file *fp;

        fp = filp_open(path, flag, 0);

        if(fp) return fp;
        else return NULL;
}

static void exit_kernel_to_read_file(mm_segment_t origin_fs)
{
        set_fs(origin_fs);
}

static mm_segment_t enable_kernel_to_read_file_and_return_origin_fs(void)
{
        mm_segment_t old_fs;

        old_fs = get_fs();

        set_fs(KERNEL_DS);

        return old_fs;
}

int bq27320_smbus_write_data(int reg_offset, int byte, unsigned int value)
{
     s32 ret = -EINVAL;
     int count=0;

	mutex_lock(&lock);
	do{
		if(byte){
			ret = i2c_smbus_write_byte_data(bq27320_device->client, reg_offset, value&0xFF);
		}
		else{
			ret = i2c_smbus_write_word_data(bq27320_device->client, reg_offset, value&0xFFFF);
		}
	mutex_unlock(&lock);
	}while((ret<0) && (++count<=SMBUS_RETRY));
	return ret;
}

static int __write_or_read_or_sleep_gauge_firmware_data(u8 dffs_command, u8 buff[], u8 buff_length)
{
        struct i2c_client *client = NULL;
        u8 data_length = buff_length - 2;               // 16 00 13 (address reg data) data_length = 3 - 2
        u8 read_buff[data_length];
        int i;
        int sleep_time;

        if(dffs_command != update_command_sleep){
                if(buff[0] == 0xAA)             // normail mode address
                        client = bq27320_device->client;
                else if(buff[0] == 0x16)        // rom mode address
                        client = bq27320_device->ROM_mode_client;
                else                                            // invalid address
                        return -EINVAL;
        }

        switch(dffs_command){
                case update_command_read:       //read command
                        if(i2c_smbus_read_i2c_block_data(client, buff[1], data_length, read_buff) < 0){
                                BAT_NOTICE("error readr: i2c_smbus_read_i2c_block_data return error\n");
                                return -EINVAL;
                        }

                        BAT_NOTICE("gauge check return data: ");
                        for(i = 0; i < data_length; i++)
                                BAT_NOTICE("%02x", read_buff[i]);
                        BAT_NOTICE("\n");

                        for(i = 0; i < data_length; i++){               // check the data of the check sum
                                if(read_buff[i] != buff[2+i]){
                                        BAT_NOTICE("error check: i = %d, command data = %02x, gauge return data = %02x", i, buff[2+i], read_buff[i]);
                                        return -EINVAL;
                                }
                        }

                        break;
                case update_command_write:      // write command
                        if(i2c_smbus_write_i2c_block_data(client, buff[1], data_length, &buff[2]) < 0){
                                BAT_NOTICE("error write: i2c_smbus_write_i2c_block_data error\n");
                                return -EINVAL;
                        }

                        break;
                case update_command_sleep:      // sleep command
                        if(buff_length == 2){
                                sleep_time = buff[0] * 256 + buff[1];
                                BAT_NOTICE("sleep time = %d\n", sleep_time);
                                msleep(sleep_time);
                        }else{
                                BAT_NOTICE("error: invalid sleep buff length\n");
                                return -EINVAL;
                        }

                        break;
                default:
                        BAT_NOTICE("error: invalid dffs_command\n");
                        return -EINVAL;
                        break;
        }

        return 0;
}

static enum gaugeIC_update_ret_value analyze_dffs_string_to_hexadecimal_and_send_data(
        char dffs_string[], int string_length)
{
        char temp_dffs_string[string_length];
        char *temp_dffs_string_for_strsep_use;
        char *delim_token = " ";
        char *temp_char;
        u8 temp_result_array[string_length];
        int send_byte_count = 0;
        int j = 0;
        int wait_time = 0;
        int EC_return_vaule = 0;
        int re_send_to_EC_count = 0;
        enum gaugeIC_update_ret_value gaugeIC_update_return_value;
        #define RE_SEND_TO_EC_TIMES (3)

       memcpy(temp_dffs_string, dffs_string, sizeof(temp_dffs_string));

       switch(temp_dffs_string[0]){
               case '#':
                        // dffs's comment format is like:: # Date_2013_6_11
                        // abstract the string Date_2013_6_11

                        temp_dffs_string_for_strsep_use = temp_dffs_string;

                        temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                        temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);

                        BAT_NOTICE("dffs_string: comment: %s\n", temp_char);

                        break;

                case 'W':
                        // dffs's write format is like:: W: 16 00 08
                        temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

                        send_byte_count = 0;
                        for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                                temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
                                temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

                                send_byte_count++;
                        }

                        BAT_NOTICE("dffs_string: write %d bytes: ", send_byte_count);
                        j = 0;
                        for(j = 0; j < send_byte_count; j++)
                                printk("%02x ", temp_result_array[j]);

                        BAT_NOTICE("\n");

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_write, temp_result_array, send_byte_count);
                                re_send_to_EC_count++;
                        } while((EC_return_vaule <0) && (re_send_to_EC_count < RE_SEND_TO_EC_TIMES));

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_write : update_success;

                        return gaugeIC_update_return_value;

                        //gaugeIC_update_return_value = change_EC_ret_value_to_gaugeIC_update_ret_value(EC_return_vaule);
                        //if(gaugeIC_update_return_value != 0)
                        //      return EC_return_vaule;

                        break;

                case 'C':

                        // dffs's check format is like:: C: 16 04 34 11 3F A7

                        temp_dffs_string_for_strsep_use = &temp_dffs_string[3];

                        send_byte_count = 0;
                        for(temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token);
                                temp_char != NULL; temp_char = strsep(&temp_dffs_string_for_strsep_use, delim_token)){
                                temp_result_array[send_byte_count] = (u8)simple_strtol(temp_char, NULL, 16);

                                send_byte_count++;
                        }

                        BAT_NOTICE("dffs_string: check %d bytes: ", send_byte_count);
                        j = 0;
                        for(j = 0; j < send_byte_count; j++)
                                BAT_NOTICE("%02x ", temp_result_array[j]);

                        BAT_NOTICE("\n");

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_read, temp_result_array,send_byte_count);
                                re_send_to_EC_count++;
                        }while((EC_return_vaule <0) && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_read : update_success;

                        return gaugeIC_update_return_value;

                        break;

                case 'X':
                        // dffs's check format is like:: X: 170

                        temp_dffs_string_for_strsep_use = temp_dffs_string;

                        temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);
                        temp_char = strsep(&temp_dffs_string_for_strsep_use,delim_token);

                        wait_time = (int)simple_strtol(temp_char, NULL, 10);
                        temp_result_array[0] = wait_time / 256;
                        temp_result_array[1] = wait_time % 256;
                        BAT_NOTICE("dffs_string: wait: %02x, %02x\n", temp_result_array[0], temp_result_array[1]);

                        do{
                                EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_sleep, temp_result_array, 2);
                                re_send_to_EC_count++;
                        }while(EC_return_vaule < 0 && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

                        gaugeIC_update_return_value = (EC_return_vaule <0) ? error_sleep : update_success;

                        return gaugeIC_update_return_value;

                        break;
                default:
                        BAT_NOTICE("error\n");

                        break;
        }

        return 0;
}

static int gaugeIC_exit_ROM_mode(void)
{
        int max_dffs_string_length = 256;
        char dffs_string[max_dffs_string_length];

        BAT_NOTICE("gaugeIC: exit ROM mode...\n");

        strcpy(dffs_string, "W: 16 00 0F");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: 16 00 0F update success\n");

        strcpy(dffs_string, "W: 16 64 0F 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: 16 64 0F 00 update success\n");

        strcpy(dffs_string, "X: 4000");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 4000 update success\n");

        strcpy(dffs_string, "W: AA 00 20 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 20 00");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: exit ROM mode: X: 20 update success\n");

        BAT_NOTICE("gaugeIC: exit ROM mode success\n");

        return 0;
}

static int gaugeIC_enter_ROM_mode(void)
{
	int max_dffs_string_length = 256;
        char dffs_string[max_dffs_string_length];

        BAT_NOTICE("gaugeIC: enter ROM mode...\n");

        strcpy(dffs_string, "W: AA 00 14 04");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 14 04 update success\n");

        strcpy(dffs_string, "W: AA 00 72 16");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 72 16 update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 FF FF");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

        strcpy(dffs_string, "W: AA 00 FF FF");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        strcpy(dffs_string, "W: AA 00 00 0F");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: W: AA 00 00 0F update success\n");

        strcpy(dffs_string, "X: 20");
        if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string, max_dffs_string_length) != update_success)
                return -EINVAL;
        BAT_NOTICE("gauge: enter ROM mode: X: 20 update success\n");

        BAT_NOTICE("gaugeIC: enter ROM mode success\n");

        return 0;
}

static int get_gaugeIC_mode(void)
{
        // normal mode: if we can get battery information(such as capacity).
        // ROM mode:    if we can get ROM mode data(we choose 0x66, since we found some check sum address is 0x66).
        // may broken:   we can't get battery information and can't get ROM mode data.
        int return_value;
        u8 ROM_mode_read_buff[1];
        int retry_times = 0;
        #define MAX_RETRY_TIME 3

	do {
                if(bq27320_smbus_read_data(bq27320_data[REG_CAPACITY].addr, 0, &return_value) >= 0)
                return normal_mode;

                if(i2c_smbus_read_i2c_block_data(bq27320_device->ROM_mode_client, 0x66, 1, ROM_mode_read_buff) >= 0)
                return ROM_mode;

                retry_times++;
	} while(retry_times <= MAX_RETRY_TIME);

	return may_broken;
}

static bool check_if_gaugeIC_in_ROM_mode(void)
{
	enum gaugeIC_mode_enum gaugeIC_mode = may_broken;
	gaugeIC_mode = get_gaugeIC_mode();

	return (gaugeIC_mode == normal_mode) ? false : true;  // since sometimes it will check error, so if not normal, feedback it is in rom mode
}

static int __update_gaugeIC_firmware_data_flash_blocks(struct file *fp)
{
        int max_dffs_string_length = 256;
        int i = 0;
        int dffs_line_count = 0;
        int ret;
        int gaugeIC_check_error_count = 0;
        char temp_char;
        char dffs_string[max_dffs_string_length];
        enum gaugeIC_update_ret_value gaugeIC_update_return_value;
        #define F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR (0)
        #define GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES (5)

        BAT_NOTICE("gaugeIC: update data flash blocks...\n");

        if (!(fp->f_op) || !(fp->f_op->read)){
                BAT_NOTICE("gaugeIC: update data flash blocks error: no dffs file operation\n");
                return -EINVAL;
        }

        do {
                ret = fp->f_op->read(fp,&temp_char,1, &fp->f_pos);
                if (ret > 0){
                        if (temp_char != '\n'){
                                dffs_string[i] = temp_char;

                                i++;
                        } else {
                                dffs_string[i] = '\0';

                                BAT_NOTICE("line %d: %s\n", dffs_line_count, dffs_string);

                                gaugeIC_update_return_value =analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
                                                                                                max_dffs_string_length);

                                switch(gaugeIC_update_return_value){
                                        case error_sleep:
                                        case error_check:
                                        case error_write:
                                        case error_read:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
                                                fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
                                                gaugeIC_check_error_count++;
                                                i = 0;
                                                dffs_line_count = 0;

                                                if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
                                                        return -EINVAL;

                                                break;

                                        //case EC_error_unknow:
                                        //      printk("gauge_bq27520: gaugeIC firmware update: Major errors\n");
                                        //      return -EINVAL;
                                        //      break;

                                        case update_success:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: success\n");
                                                i = 0;
                                                dffs_line_count++;
                                                break;

                                        default:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: no this case\n");
                                                return -EINVAL;
                                                break;
                                }

                        }
                } else {  // last line
                        if (i != 0) {
                                dffs_string[i] = '\0';

                                BAT_NOTICE("line %d: %s\n", dffs_line_count, dffs_string);

                                gaugeIC_update_return_value = analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
                                                                                                max_dffs_string_length);

                                switch(gaugeIC_update_return_value){
                                        case error_sleep:
                                        case error_check:
                                        case error_write:
                                        case error_read:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
                                                fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
                                                gaugeIC_check_error_count++;
                                                i = 0;
                                                dffs_line_count = 0;
                                                ret = 1;

                                                if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
                                                        return -EINVAL;

                                                break;

                                        //case EC_error_unknow:
                                        //      printk("gauge_bq27520: gaugeIC firmware update: Major errors\n");
                                        //      return -EINVAL;
                                        //      break;
                                        case update_success:
                                                BAT_NOTICE("gauge: gaugeIC firmware update: success\n");
                                                i = 0;
                                                dffs_line_count++;
                                                break;

                                        default:
						BAT_NOTICE("gauge: gaugeIC firmware update: no this case\n");
						return -EINVAL;
						break;
				}
			}
		}
	} while(ret > 0);


	BAT_NOTICE("gaugeIC: update data flash blocks success\n");

	return 0;
}

static int update_gaugeIC_firmware(struct file *fp)
{
	int ret = 0;

	if (!check_if_gaugeIC_in_ROM_mode())
		ret = gaugeIC_enter_ROM_mode();

	if (ret != 0)
		return -EINVAL;

	ret = __update_gaugeIC_firmware_data_flash_blocks(fp);

	if (ret != 0)
		return -EINVAL;

	if (project == FE375CG)
		ret = gaugeIC_exit_ROM_mode();

	return ret;
}

#ifdef NOT_USER
static ssize_t show_battery_BSOC(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	int value;
	ret = bq27320_smbus_read_data(0x74, 0, &value);
	if (ret) {
		BAT_ERR("error in reading battery BSOC= %x\n", ret);
		return 0;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_RSOC(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	int value;
	ret = bq27320_smbus_read_data(0x2C, 0, &value);
	if (ret) {
		BAT_ERR("error in reading battery RSOC= %x\n", ret);
		return 0;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_RM(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	int value;
	ret = bq27320_smbus_read_data(0x10, 0, &value);
	if (ret) {
		BAT_ERR("error in reading battery RM = %x\n", ret);
		return 0;
	}

	return sprintf(buf, "%d\n", value);
}

static ssize_t show_battery_FCC(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	int value;
	ret = bq27320_smbus_read_data(0x12, 0, &value);
	if (ret) {
		BAT_ERR("error in reading battery FCC = %x\n", ret);
		return 0;
	}

	return sprintf(buf, "%d\n", value);
}
#endif

static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int status = !bq27320_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}

static ssize_t show_battery_APD_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", bq27320_device->bat_APD_check);
}

static ssize_t battery_APD_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	if (buf[0] == '1') {
		bq27320_device->bat_APD_check = 1;
		BAT_NOTICE("enable bat_APD_check\n");
	}
	if (buf[0] == '0') {
		bq27320_device->bat_APD_check = 0;
		BAT_NOTICE("disable bat_APD_check\n");
	}
	return count;
}

#ifdef FACTORY_IMAGE
static ssize_t battery_limit_off_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
        if (buf[0] == '1') {
		bq27320_device->bat_limit_off = 0;
		BAT_NOTICE("enable ATD battery limit\n");
	}
        if (buf[0] == '0') {
		bq27320_device->bat_limit_off = 1;
		BAT_NOTICE("disable ATD battery limit\n");
	}
	return count;
}

static ssize_t battery_limit_off_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "bat_limit_off = %d, echo 1(off) or 0(on) > battery_limit_off\n", bq27320_device->bat_limit_off);
}

#endif

static ssize_t get_gauge_FW(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int ret;
	int temp = 0;

	bq27320_smbus_write_data(0x00, 0, DEVICE_NAME);
	msleep(10);
	ret = bq27320_smbus_read_data(0x45, 0, &temp);
	if (ret < 0)
		return sprintf(buf, "0\n");
	return sprintf(buf, "V:1.%x\n", temp);

}

static ssize_t show_battery_INFO(struct device *dev, struct device_attribute *devattr, char *buf)
{
	char tmp_buf[64];
        int ret;
	int temp;

	bq27320_smbus_write_data(0x00, 0, DEVICE_NAME);
	msleep(10);
	ret = bq27320_smbus_read_data(0x45, 0, &temp);

        if (ret < 0)
                return sprintf(buf, "DF_VERSION fails\n");

	sprintf(tmp_buf, "%x\n", temp);
	strcat(buf, tmp_buf);
	return strlen(buf);
}

static ssize_t show_battery_soh(struct device *dev, struct device_attribute *devattr, char *buf)
{

	int ret, r_FCC, r_DC, r_RM, r_temp, r_volt, r_cur, r_CC, r_SOH;

	ret = bq27320_smbus_read_data(FCC, 0, &r_FCC);
	if (ret) {
		BAT_ERR("error in reading battery FCC = %x\n", ret);
		return sprintf(buf, "Fail\n");
	}

	ret = bq27320_smbus_read_data(DC, 0, &r_DC);
	if (ret) {
		BAT_ERR("error in reading battery DC = %x\n", ret);
		return sprintf(buf, "Fail\n");
	}

	ret = bq27320_smbus_read_data(RM, 0, &r_RM);
	if (ret) {
		BAT_ERR("error in reading battery RM = %x\n", ret);
		return sprintf(buf, "Fail\n");
	}

	ret = bq27320_smbus_read_data(CC, 0, &r_CC);
	if (ret) {
		BAT_ERR("error in reading battery CC = %x\n", ret);
		return sprintf(buf, "Fail\n");
	}

	ret = bq27320_smbus_read_data(SOH, 0, &r_SOH);
	if (ret) {
		BAT_ERR("error in reading battery SOH = %x\n", ret);
		return sprintf(buf, "Fail\n");
	}

	r_temp = bq27320_device->old_temperature/10;
	r_volt = bq27320_device->bat_vol/1000;
	r_cur = bq27320_device->bat_current;

	return sprintf(buf, "FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(¢XC),VOLT=%d(mV),CUR=%d(mA),CC=%d,SOH=%d(%)\n",
                        r_FCC, r_DC, r_RM, r_temp, r_volt, r_cur, r_CC, r_SOH);

}

#ifdef NOT_USER
static ssize_t gauge_mode_show(struct device *dev,
        struct device_attribute *devattr, char *buf)
{
        // 0: not normal mode and not rom mode. the gauge may broken.
        // 1: gaugeIC in rom mode.
        // 2: gaugeIC in normal mode.

        enum gaugeIC_mode_enum gaugeIC_mode = may_broken;
        char gaugeIC_mode_in_sentence[15];

        gaugeIC_mode = get_gaugeIC_mode();
        BAT_NOTICE("gaugeIC mode: %d\n", gaugeIC_mode);
        switch(gaugeIC_mode){
                case may_broken:                // not normal mode and not rom mode. the gauge may broken.
                        strcpy(gaugeIC_mode_in_sentence, "may broken");
                        break;
                case ROM_mode:          // gaugeIC in rom mode.
                        strcpy(gaugeIC_mode_in_sentence, "rom mode");
                        break;
                case normal_mode:       // gaugeIC in normal mode.
                        strcpy(gaugeIC_mode_in_sentence, "normal mode");
                        break;
                default:                                // no this case
                        strcpy(gaugeIC_mode_in_sentence, "no this case");
        }

        return sprintf(buf, "gaugeIC mode: %s\n", gaugeIC_mode_in_sentence);
}

static ssize_t gauge_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int temp,ret;
	struct file *fp;
	mm_segment_t old_fs;
	int val;

	pmic_get_battery_id(&val);

	if (buf[0] == '1') {
		temp = get_gaugeIC_mode();
		BAT_NOTICE("gaugeIC_mode = %d\n", temp);
	} else if (buf[0] == '2') {
		gaugeIC_enter_ROM_mode();
		BAT_NOTICE("enter ROM mode\n");
	} else if (buf[0] == '3') {
		gaugeIC_exit_ROM_mode();
		BAT_NOTICE("exit ROM mode\n");
	} else if (buf[0] == '4') {
		BAT_NOTICE("update\n");
			old_fs = enable_kernel_to_read_file_and_return_origin_fs();
		if (project == Z300CL)
			fp = open_file(GAUGE_FILE, O_RDONLY, 0);
		else if (project == Z300CNL) {
			if ((val <= SDI_UPPER) && (val >= SDI_LOWER))
				fp = open_file(GAUGE_Z300CNL_SDI_FILE, O_RDONLY, 0);
			else if ((val <= LG_UPPER) && (val >= LG_LOWER))
				fp = open_file(GAUGE_Z300CNL_LG_FILE, O_RDONLY, 0);
		}
		if (!IS_ERR(fp) && (fp != NULL)) {
			ret = update_gaugeIC_firmware(fp);
			if (ret == 0)
				printk("===bq27320 firware update success===\n");
			else
				printk("===bq27320 firmware update fail===\n");
			close_file(fp);
		} else
			printk("===bq27320 firmware update fail: no file===\n");
		exit_kernel_to_read_file(old_fs);
	}

	return count;
}
#endif

static DEVICE_ATTR(battery_APD, S_IWUSR | S_IRUGO, show_battery_APD_status, battery_APD_store);
static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status,NULL);
static DEVICE_ATTR(battery_INFO, S_IWUSR | S_IRUGO, show_battery_INFO,NULL);
static DEVICE_ATTR(battery_soh, S_IWUSR | S_IRUGO, show_battery_soh, NULL);
#ifdef FACTORY_IMAGE
static DEVICE_ATTR(battery_limit_off, S_IWUSR | S_IRUGO, battery_limit_off_show, battery_limit_off_store);
#endif

#ifdef NOT_USER
static DEVICE_ATTR(battery_FCC, S_IWUSR | S_IRUGO, show_battery_FCC, NULL);
static DEVICE_ATTR(battery_RM, S_IWUSR | S_IRUGO, show_battery_RM, NULL);
static DEVICE_ATTR(battery_RSOC, S_IWUSR | S_IRUGO, show_battery_RSOC, NULL);
static DEVICE_ATTR(battery_BSOC, S_IWUSR | S_IRUGO, show_battery_BSOC, NULL);
static DEVICE_ATTR(gauge_mode, S_IWUSR | S_IRUGO, gauge_mode_show, NULL);
static DEVICE_ATTR(gauge_debug, S_IWUSR | S_IRUGO, NULL, gauge_store);
#endif

static struct attribute *battery_smbus_attributes[] = {

	&dev_attr_battery_smbus.attr,
	&dev_attr_battery_INFO.attr,
	&dev_attr_battery_soh.attr,
	&dev_attr_battery_APD.attr,
#ifdef FACTORY_IMAGE
	&dev_attr_battery_limit_off.attr,
#endif

#ifdef NOT_USER
	&dev_attr_battery_FCC.attr,
	&dev_attr_battery_RM.attr,
	&dev_attr_battery_RSOC.attr,
	&dev_attr_battery_BSOC.attr,
	&dev_attr_gauge_debug.attr,
	&dev_attr_gauge_mode.attr,
#endif
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

static int bq27320_battery_current(void)
{
	int ret;
	int curr = 0;

	ret = bq27320_smbus_read_data(bq27320_data[REG_CURRENT].addr, 0, &curr);
	if (ret) {
		BAT_ERR("error reading current ret = %x\n", ret);
		return 0;
	}

	curr = (s16)curr;

	if (curr >= bq27320_data[REG_CURRENT].min_value &&
		curr <= bq27320_data[REG_CURRENT].max_value) {
		return curr;
	} else
		return 0;
}

static void battery_status_poll(struct work_struct *work)
{
       struct bq27320_device_info *batt_dev = container_of(work, struct bq27320_device_info, status_poll_work.work);

	if(!bq27320_battery_driver_ready)
		BAT_NOTICE("battery driver not ready\n");

	power_supply_changed(&bq27320_supply[Charger_Type_Battery]);

#ifndef FACTORY_IMAGE
	if (!bq27320_device->temp_err && !(smb347_shutdown_value == true)) {
		smb347_config_thermal_charging(bq27320_device->old_temperature/10, bq27320_device->bat_vol, THERMAL_RULE1);
	}

	smb347_recheck_charging_type();
#endif
#if 0
	/* stand polling */
	if (dock_in) {
		stand_polling();
	}
#endif
	/* Schedule next polling */
	queue_delayed_work(bq27320_battery_work_queue, &batt_dev->status_poll_work, battery_check_interval*HZ);
}

static void auto_flash_gauge(struct work_struct *work)
{
	struct bq27320_device_info *batt_dev = container_of(work, struct bq27320_device_info, gauge_update_work.work);
	int ret;
	struct file *fp;
	mm_segment_t old_fs;
	int val;

	pmic_get_battery_id(&val);
	BAT_NOTICE("ac_on=%d RSOC=%d ROM_mode=%d battery_id=%d\n", ac_on, RSOC, check_if_gaugeIC_in_ROM_mode(), val);

	if (ac_on || (RSOC >= 99) || check_if_gaugeIC_in_ROM_mode()) {
		old_fs = enable_kernel_to_read_file_and_return_origin_fs();

		if (project == Z300CL)
			fp = open_file(GAUGE_FILE, O_RDONLY, 0);
		else if (project == Z300CNL) {
			if ((val <= SDI_UPPER) && (val >= SDI_LOWER))
				fp = open_file(GAUGE_Z300CNL_SDI_FILE, O_RDONLY, 0);
			else if ((val <= LG_UPPER) && (val >= LG_LOWER))
				fp = open_file(GAUGE_Z300CNL_LG_FILE, O_RDONLY, 0);
		}

		if (!IS_ERR(fp) && (fp != NULL)) {
			ret = update_gaugeIC_firmware(fp);
			if (ret == 0)
				printk("===bq27320 firware update success===\n");
			else
				printk("===bq27320 firmware update fail===\n");
			close_file(fp);
		} else 
			printk("===bq27320 firmware update fail: no file===\n");

		exit_kernel_to_read_file(old_fs);
		smb347_charger_enable(true);
	}
}

int auto_flash_gauge_check_Z300CNL(void)
{
	int ret;
	int temp;
	int val;

	pmic_get_battery_id(&val);
	BAT_NOTICE("Z300CL auto flash gauge check, battery id %d\n", val);

	bq27320_smbus_write_data(0x00, 0, DEVICE_NAME);
	msleep(10);
	ret = bq27320_smbus_read_data(0x45, 0, &temp);
	BAT_NOTICE("gauge fw %x\n", temp);

	if (val <= SDI_UPPER && val >= SDI_LOWER) {
		if (temp == GAUGE_Z300CNL_SDI_FW) {
			BAT_NOTICE("SDI battery checking pass\n");
			return 1;
		} else {
			cancel_delayed_work(&bq27320_device->status_poll_work);
			INIT_DELAYED_WORK(&bq27320_device->gauge_update_work, auto_flash_gauge);
			smb347_charger_enable(false);
			queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->gauge_update_work, GAUGE_UPDATE_DELAY*HZ);
		}

	} else if (val <= LG_UPPER && val >= LG_LOWER) {
		if (temp == GAUGE_Z300CNL_LG_FW) {
			BAT_NOTICE("LG battery checking pass\n");
			return 1;
		} else {
			cancel_delayed_work(&bq27320_device->status_poll_work);
			INIT_DELAYED_WORK(&bq27320_device->gauge_update_work, auto_flash_gauge);
			smb347_charger_enable(false);
			queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->gauge_update_work, GAUGE_UPDATE_DELAY*HZ);
		}
	}
}

int auto_flash_gauge_check(void)
{
	int ret;
	int temp;

	BAT_NOTICE("Z300CL auto flash gauge check\n");
	bq27320_smbus_write_data(0x00, 0, DEVICE_NAME);
	msleep(10);
	ret = bq27320_smbus_read_data(0x45, 0, &temp);
	BAT_NOTICE("gauge fw %x\n", temp);

	if (temp == GAUGE_FW)
		return 1;
	else {
		cancel_delayed_work(&bq27320_device->status_poll_work);
		INIT_DELAYED_WORK(&bq27320_device->gauge_update_work, auto_flash_gauge);
		smb347_charger_enable(false);
		queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->gauge_update_work, GAUGE_UPDATE_DELAY*HZ);
	}

}

/*
static void low_low_battery_check(struct work_struct *work)
{
	printk("lowlowbattery check");
	cancel_delayed_work(&bq27320_device->status_poll_work);
	queue_delayed_work(bq27320_battery_work_queue,&bq27320_device->status_poll_work, 0.1*HZ);
	msleep(2000);
	enable_irq(bq27320_device->irq_low_battery_detect);
}

static irqreturn_t low_battery_detect_isr(int irq, void *dev_id)
{
	printk("low_low_battery check");
	disable_irq_nosync(bq27320_device->irq_low_battery_detect);
	bq27320_device->low_battery_present = gpio_get_value(bq27320_device->gpio_low_battery_detect);
	BAT_NOTICE("gpio LL_BAT_T30=%d\n", bq27320_device->low_battery_present);
	wake_lock_timeout(&bq27320_device->low_battery_wake_lock, 10*HZ);
	queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->low_low_bat_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int setup_low_battery_irq(void)
{
	unsigned gpio = GPIOPIN_LOW_BATTERY_DETECT;
	int ret, irq = gpio_to_irq(gpio);

	ret = gpio_request(gpio, "low_battery_detect");
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 request failed\n");
		goto fail;
	}

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 unavaliable for input\n");
		goto fail_gpio;
	}

	ret = request_irq(irq, low_battery_detect_isr, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			"bq27320-battery (low battery)", NULL);
	if (ret < 0) {
		BAT_ERR("gpio LL_BAT_T30 irq request failed\n");
		goto fail_irq;
	}

	bq27320_device->low_battery_present = gpio_get_value(gpio);
	bq27320_device->irq_low_battery_detect = gpio_to_irq(gpio);
	BAT_NOTICE("irq=%d, LL_BAT_T30=%d\n", irq, bq27320_device->low_battery_present);

	enable_irq_wake(bq27320_device->irq_low_battery_detect);
	return ret;
fail_irq:
fail_gpio:
	gpio_free(gpio);
fail:
	return ret;
}
*/

int bq27320_battery_callback(unsigned usb_cable_state)
{
	if (!bq27320_battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return 0;
	}

	old_cable_status = bq27320_battery_cable_status;
	bq27320_battery_cable_status = usb_cable_state;

	printk("========================================================\n");
	printk("bq27320_battery_callback  usb_cable_state = %x\n", usb_cable_state);
	printk("========================================================\n");

	if (old_cable_status != bq27320_battery_cable_status) {
		printk(KERN_INFO"battery_callback cable_wake_lock 5 sec...\n ");
		wake_lock_timeout(&bq27320_device->cable_wake_lock, 5*HZ);
	}

	bq27320_check_cabe_type();

	if (!bq27320_battery_cable_status) {
		if (old_cable_status == USB_AC_Adapter)
			power_supply_changed(&bq27320_supply[Charger_Type_AC]);
		else if (old_cable_status == USB_Cable)
			power_supply_changed(&bq27320_supply[Charger_Type_USB]);
		else if (bq27320_battery_cable_status == USB_Cable)
			power_supply_changed(&bq27320_supply[Charger_Type_USB]);
		else if (bq27320_battery_cable_status == USB_AC_Adapter)
			power_supply_changed(&bq27320_supply[Charger_Type_AC]);
	}

	if (strcmp(androidboot_mode, "main") == 0) {
		cancel_delayed_work(&bq27320_device->status_poll_work);
		queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->status_poll_work, 2*HZ);
		battery_check_interval = BATTERY_POLLING_RATE_FIRST;
	}

	return 1;

}
EXPORT_SYMBOL(bq27320_battery_callback);

#if 0
int bq27320_pack_bat_callback(unsigned pack_bat_state)
{
	printk(KERN_NOTICE "========================================================\n");
	printk(KERN_NOTICE "pack_bat_callback  bat_state = %x old_cable = %x\n", pack_bat_state, old_cable_status) ;
	printk(KERN_NOTICE "========================================================\n");

	if (pack_bat_state == 5)
		power_supply_changed(&bq27320_supply[Charger_Type_PACK_BAT]);
	else {
		if (old_cable_status == USB_AC_Adapter)
			power_supply_changed(&bq27320_supply[Charger_Type_AC]);
		else if (old_cable_status == USB_Cable)
			power_supply_changed(&bq27320_supply[Charger_Type_USB]);
		else
			power_supply_changed(&bq27320_supply[Charger_Type_Battery]);
	}

	cancel_delayed_work(&bq27320_device->status_poll_work);
	queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->status_poll_work, 2*HZ);

	return 1;
}
EXPORT_SYMBOL(bq27320_pack_bat_callback);

int bq27320_pack_ac_callback(unsigned pack_ac_state)
{
	printk(KERN_NOTICE "========================================================\n");
	printk(KERN_NOTICE "bq27320_pack_ac_callback  pack_ac_state = %x\n", pack_ac_state);
	printk(KERN_NOTICE "========================================================\n");

	if (pack_ac_state == 4) {
		power_supply_changed(&bq27320_supply[Charger_Type_PACK_AC]);
	} else {
		power_supply_changed(&bq27320_supply[Charger_Type_PACK_BAT]);
	}

	cancel_delayed_work(&bq27320_device->status_poll_work);
	queue_delayed_work(bq27320_battery_work_queue,
	&bq27320_device->status_poll_work, 2*HZ);

	return 1;
}
EXPORT_SYMBOL(bq27320_pack_ac_callback);
#endif

static int bq27320_get_health(enum power_supply_property psp,
	union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_PRESENT)
		val->intval = 1;
	else if (psp == POWER_SUPPLY_PROP_HEALTH)
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int bq27320_get_psp(int reg_offset, enum power_supply_property psp,
	union power_supply_propval *val)
{
	s32 ret;
	int rt_value=0;

	bq27320_device->smbus_status = bq27320_smbus_read_data(bq27320_data[reg_offset].addr, 0, &rt_value);

	if ((bq27320_device->smbus_status < 0) && (psp != POWER_SUPPLY_PROP_TEMP)) {
		dev_err(&bq27320_device->client->dev, "%s: i2c read for %d failed\n", __func__, reg_offset);
		return -EINVAL;
	}

	if (psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
		if (rt_value >= bq27320_data[REG_VOLTAGE].min_value &&
			rt_value <= bq27320_data[REG_VOLTAGE].max_value) {
			if (rt_value > BATTERY_PROTECTED_VOLT)
				val->intval = bq27320_device->bat_vol = rt_value*1000;
			else
				val->intval = bq27320_device->bat_vol;
		} else
			val->intval = bq27320_device->bat_vol;
		BAT_NOTICE("voltage_now= %u uV\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_STATUS) {
		ret = bq27320_device->bat_status = rt_value;

//		if (ac_on || usb_on || dock_ac) {/* Charging detected */
		if (ac_on || usb_on) {
			if (bq27320_device->old_capacity == 100)
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else if ((ret & BATT_STS_SOCF) || (ret & BATT_STS_DSG))		/* Fully Discharged detected */
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

		BAT_NOTICE("status: %s ret= 0x%04x\n", status_text[val->intval], ret);

	} else if (psp == POWER_SUPPLY_PROP_TEMP) {
		ret = bq27320_device->bat_temp = rt_value;
		if (bq27320_device->smbus_status >= 0) {
			if (rt_value >= bq27320_data[REG_TEMPERATURE].min_value &&
				rt_value <= bq27320_data[REG_TEMPERATURE].max_value) {
				if (rt_value >= 2530  && rt_value <= 3530) {
					ret = rt_value - 2730;
					if (bq27320_device->temp_err)
						bq27320_device->temp_err = false;
				} else
					bq27320_device->temp_err = true;
			} else
				bq27320_device->temp_err = true;
		} else
			bq27320_device->temp_err = true;

		if (bq27320_device->temp_err) {
			BAT_NOTICE("error: temperature ret=%d, old_temp=%d \n",
				rt_value, bq27320_device->old_temperature);
			if (bq27320_device->old_temperature != 0xFF)
				ret = bq27320_device->old_temperature;
			else
				return -EINVAL;
		}

		bq27320_device->old_temperature = val->intval = ret;
		BAT_NOTICE("temperature= %d (0.1¢XC)\n", val->intval);
	}
	if (psp == POWER_SUPPLY_PROP_CURRENT_NOW) {
		val->intval = bq27320_device->bat_current
			= bq27320_battery_current();
		BAT_NOTICE("current = %d mA\n", val->intval);
	}
	return 0;
}

static int enableADF_check(void)
{
	char buf[4] = {0, 0, 0, 0};
	struct file *fd;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	fd = filp_open("/ADF/ADF", O_RDONLY, 0);

	if (!IS_ERR(fd)) {
		kernel_read(fd, fd->f_pos, buf, 4);
		filp_close(fd, NULL);
	} else {
		set_fs(old_fs);
		pr_info("open /ADF/ADF fail\n");
		return 0;
	}

	set_fs(old_fs);
	pr_info("***** check ADF: %d\n", buf[3]);
	if (buf[3] == 1 || buf[3] == 2) {
		pr_info("enableADF_check pass %d\n", buf[3]);
		return 1;
	} else {
		pr_info("enableADF_check fail %d\n", buf[3]);
		return 0;
	}
}

static int bq27320_get_capacity(union power_supply_propval *val)
{
	s32 ret;
	s32 temp_capacity;
	int smb_retry = 0;
	bool check_cap = false;
	int smb_retry_max = (SMBUS_RETRY + 2);

	do {
		bq27320_device->smbus_status = bq27320_smbus_read_data(bq27320_data[REG_CAPACITY].addr, 0, &bq27320_device->bat_capacity);
		if ((bq27320_device->bat_capacity <= 0) || (bq27320_device->bat_capacity > 100))
			check_cap = true;

	} while(((bq27320_device->smbus_status < 0) || check_cap) && ( ++smb_retry <= smb_retry_max));

	if (bq27320_device->smbus_status < 0) {
		dev_err(&bq27320_device->client->dev, "%s: i2c read for %d "
			"failed bq27320_device->cap_err=%u\n", __func__, REG_CAPACITY, bq27320_device->cap_err);

		if (bq27320_device->cap_err > 5 || (bq27320_device->old_capacity == 0xFF))
			return -EINVAL;
		else {
			val->intval = bq27320_device->old_capacity;
			bq27320_device->cap_err++;
			BAT_NOTICE("cap_err=%u use old capacity=%u\n", bq27320_device->cap_err, val->intval);
			return 0;
		}
	}

	RSOC = temp_capacity = ret = bq27320_device->bat_capacity;

	if (!(bq27320_device->bat_capacity >= bq27320_data[REG_CAPACITY].min_value &&
			bq27320_device->bat_capacity <= bq27320_data[REG_CAPACITY].max_value)) {
		val->intval = bq27320_device->old_capacity;
		BAT_NOTICE("use old capacity=%u\n", bq27320_device->old_capacity);
		return 0;
	}

	/* start: for mapping %99 to 100%. Lose 84%*/
	if (temp_capacity == 99)
		temp_capacity = 100;
	if (temp_capacity >= 84 && temp_capacity <= 98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	/* lose 26% 47% 58%,69%,79% */
	if (temp_capacity > 70 && temp_capacity < 80)
		temp_capacity -= 1;
	else if (temp_capacity > 60 && temp_capacity <= 70)
		temp_capacity -= 2;
	else if (temp_capacity > 50 && temp_capacity <= 60)
		temp_capacity -= 3;
	else if (temp_capacity > 30 && temp_capacity <= 50)
		temp_capacity -= 4;
	else if (temp_capacity >= 0 && temp_capacity <= 30)
		temp_capacity -= 5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity < 0) ? 0 : temp_capacity);

	/* 572 series gauge wrokaround TT464876 and TT463786 */
	if (project == Z300CL || project == Z300CNL) {
		if (temp_capacity == 0 && bq27320_device->bat_vol == 3600000) {
			BAT_NOTICE("TI gauge workaround\n");
			temp_capacity = bq27320_device->old_capacity;
		}
	}

	val->intval = temp_capacity;

	if (strcmp(androidboot_mode, "main") == 0) {
		if ((temp_capacity == 0) &&
			(bq27320_battery_cable_status)) {
			bq27320_device->bat_capacity_zero_count++;
			if (bq27320_device->bat_capacity_zero_count >= 6) {
				bq27320_device->bat_capacity_zero_count = 0;
				BAT_NOTICE("pretend no charging type to shutdown\n");
				reboot_cmd = 1;
				bq27320_battery_callback(0);
			} else
				BAT_NOTICE("bat_capacity_zero_count = %d",
				bq27320_device->bat_capacity_zero_count);
		} else
			bq27320_device->bat_capacity_zero_count = 0;

		if (temp_capacity < 5 && battery_check_interval != 10) {
			battery_check_interval = 10;
			BAT_NOTICE("battery_check_interval = %d\n",
				battery_check_interval);
		} else if (temp_capacity >= 5 &&
			battery_check_interval != BATTERY_POLLING_RATE) {
			battery_check_interval = BATTERY_POLLING_RATE;
			BAT_NOTICE("battery_check_interval = %d\n",
				battery_check_interval);
		}
	}

	bq27320_device->old_capacity = val->intval;
	bq27320_device->cap_err = 0;

	/* persist.sys.enableAPD check*/
	if (bq27320_device->bat_APD_check && enableADF_check()) {
		if (val->intval >= 60) {
			smb347_charger_enable(false);
			BAT_NOTICE("smb347 charger disable due to enable APD & ADF\n");
		} else if (val->intval <= 55) {
			smb347_charger_enable(true);
			BAT_NOTICE("smb347 charger enable due to capacity <= 55\n");
		}
	}

#ifdef FACTORY_IMAGE
	if (bq27320_device->bat_limit_off == 0) {
		if (val->intval <= 40) {
			BAT_NOTICE("smb347 charger enable\n");
			smb347_charger_enable(true);
		} else if (val->intval >= 60) {
			BAT_NOTICE("smb347 charger disable\n");
			smb347_charger_enable(false);
		}
	} else {
		BAT_NOTICE("smb347 charger enable\n");
		smb347_charger_enable(true);
	}

#endif

	BAT_NOTICE("= %u%% ret= %u\n", val->intval, ret);
	return 0;
}

static int bq27320_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	u8 count;
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_HEALTH:
		if (bq27320_get_health(psp, val))
			goto error;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		if (bq27320_get_capacity(val))
			goto error;
		break;

	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		for (count = 0; count < REG_MAX; count++) {
			if (psp == bq27320_data[count].psp)
				break;
		}

		if (bq27320_get_psp(count, psp, val))
			return -EINVAL;
		break;

	default:
		dev_err(&bq27320_device->client->dev,
			"%s: INVALID property psp=%u\n", __func__, psp);
		return -EINVAL;
	}

	return 0;

error:

	return -EINVAL;
}

static int bq27320_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret, i = 0;
	struct device *dev = &client->dev;

	printk("bq27320 probe init\n");

	project = asustek_get_project_id();
	BAT_NOTICE("project: %d\n", project);

	BAT_NOTICE("+ client->addr= %02x\n", client->addr);
	bq27320_device = kzalloc(sizeof(*bq27320_device), GFP_KERNEL);
	if (!bq27320_device)
		return -ENOMEM;

	memset(bq27320_device, 0, sizeof(*bq27320_device));
	bq27320_device->client = client;
	i2c_set_clientdata(client, bq27320_device);
	bq27320_device->smbus_status = 0;
	bq27320_device->cap_err = 0;
	bq27320_device->temp_err = false;
	bq27320_device->old_capacity = 0xFF;
	bq27320_device->old_temperature = 0xFF;
	bq27320_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;
	bq27320_device->bat_limit_off = 0;
	bq27320_device->bat_capacity_zero_count = 0;

	for (i = 0; i < ARRAY_SIZE(bq27320_supply); i++) {
		ret = power_supply_register(&client->dev, &bq27320_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply\n");
			while (i--)
				power_supply_unregister(&bq27320_supply[i]);
				kfree(bq27320_device);
			return ret;
		}
	}

	bq27320_battery_work_queue = create_singlethread_workqueue("bq27320_battery_work_queue");
	INIT_DELAYED_WORK(&bq27320_device->status_poll_work, battery_status_poll);
//	INIT_DELAYED_WORK(&bq27320_device->low_low_bat_work, low_low_battery_check);
	cancel_delayed_work(&bq27320_device->status_poll_work);

	spin_lock_init(&bq27320_device->lock);
	wake_lock_init(&bq27320_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&bq27320_device->cable_wake_lock, WAKE_LOCK_SUSPEND, "cable_state_changed");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret)
		dev_err(&client->dev, "bq27320_probe: unable to create the sysfs\n");

//	setup_low_battery_irq();

	bq27320_device->ROM_mode_client = kzalloc(sizeof(*client), GFP_KERNEL);
	*bq27320_device->ROM_mode_client = *bq27320_device->client;
	bq27320_device->ROM_mode_client->addr = 0x0B;
	bq27320_battery_driver_ready = 1;
	queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->status_poll_work, 15*HZ);
	BAT_NOTICE("- %s driver registered\n", client->name);
	bq27320_device->gauge_sdev.name = "battery";
	bq27320_device->gauge_sdev.print_name = get_gauge_FW;

	if (switch_dev_register(&bq27320_device->gauge_sdev) < 0)
		dev_info(&client->dev, "switch_dev_register for gauge failed!\n");

	switch_set_state(&bq27320_device->gauge_sdev, 0);

	/* auto gauge fw update check */
	if (project == Z300CL)
		auto_flash_gauge_check();
	else if (project == Z300CNL)
		auto_flash_gauge_check_Z300CNL();

	return 0;
}

static int bq27320_remove(struct i2c_client *client)
{
	struct bq27320_device_info *bq27320_device;
	int i = 0;

	bq27320_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(bq27320_supply); i++)
		power_supply_unregister(&bq27320_supply[i]);

	if (bq27320_device) {
		wake_lock_destroy(&bq27320_device->low_battery_wake_lock);
		wake_lock_destroy(&bq27320_device->cable_wake_lock);
		kfree(bq27320_device);
		bq27320_device = NULL;
	}
	return 0;
}

static int bq27320_shutdown(struct i2c_client *client)
{
	BAT_NOTICE("shutdown ++\n");

	if (strcmp(androidboot_mode, "main") == 0) {
		cancel_delayed_work(&bq27320_device->status_poll_work);
		destroy_workqueue(bq27320_battery_work_queue);
	}
	BAT_NOTICE("shutdown --\n");
	return 0;

}

static int bq27320_suspend(struct i2c_client *client, pm_message_t state)
{
	if (strcmp(androidboot_mode, "main") == 0) {
		cancel_delayed_work_sync(&bq27320_device->status_poll_work);
		flush_workqueue(bq27320_battery_work_queue);
	}
	return 0;
}

/* any smbus transaction will wake up pad */
static int bq27320_resume(struct i2c_client *client)
{
	if (strcmp(androidboot_mode, "main") == 0) {
		cancel_delayed_work(&bq27320_device->status_poll_work);
		queue_delayed_work(bq27320_battery_work_queue, &bq27320_device->status_poll_work, 0.1*HZ);
	}
	return 0;
}

static const struct i2c_device_id bq27320_id[] = {
	{ "hpa02254_batt", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, bq27320_id);

static struct i2c_driver bq27320_battery_driver = {
	.probe		= bq27320_probe,
	.remove		= bq27320_remove,
	.shutdown	= bq27320_shutdown,
	.suspend	= bq27320_suspend,
	.resume		= bq27320_resume,
	.id_table	= bq27320_id,
	.driver = {
		.name = "hpa02254_batt",
	},
};
static int __init bq27320_battery_init(void)
{
	int ret;
	printk("bq27320 init\n");

	ret = i2c_add_driver(&bq27320_battery_driver);
	if (ret)
		dev_err(&bq27320_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);
	return ret;
}
late_initcall(bq27320_battery_init);

static void __exit bq27320_battery_exit(void)
{
	i2c_del_driver(&bq27320_battery_driver);
}
module_exit(bq27320_battery_exit);

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("bq27320 battery monitor driver");
MODULE_LICENSE("GPL");
