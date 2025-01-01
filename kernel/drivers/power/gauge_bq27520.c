/*
 * drivers/power/gauge_bq27520.c
 **
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
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
#include <linux/wakelock.h>

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <asm/uaccess.h>
#include <linux/switch.h>


#include <linux/power/gauge_bq27520.h>

// vvvv get battery ID from pmic vvvv
#include <asm/intel_scu_pmic.h>
// ^^^^ get battery ID from pmic ^^^^



#define GPIO_PIN_LOW_BATTERY_DETECT TEGRA_GPIO_PR4
#define SMBUS_RETRY (0)
#define KELVIN_BASE 2730
#define INITIAL_CAPACITY_VALUE	50
#define INITIAL_TEMPERATURE_VALUE	250
#define PROTECT_TEMPERATURE_IN_CELSIUS_HIGH 1200
#define PROTECT_TEMPERATURE_IN_CELSIUS_LOW (-200)
#define PROPERTY_ERROR_RESET_VALUE_CAPACITY	0
#define BATTERY_POLLING_RATE 60
#define RETRY_TIMES_IF_EC_FEEDBACK_ERROR	(3)
#define EC_FEEDBACK_STATUS_DISCHARGING		0x0040
#define EC_FEEDBACK_STATUS_FULLY_CHARGED	0x0020
#define EC_FEEDBACK_STATUS_FULLY_DISCHARGED	0x0010
#define PMIC_BATTID_MSB_RESULT_REG     0x5EEC //note that the datasheet said 10EC, but intel matt said have to transfer to this
#define PMIC_BATTID_LSB_RESULT_REG     0x5EED
#define HAYDN_GPIO_AC_OK TEGRA_GPIO_PV1

#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_VOLTAGE 				4000
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CAPACITY_IN_PERCENT	INITIAL_CAPACITY_VALUE
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_TEMPERATURE			(KELVIN_BASE + INITIAL_TEMPERATURE_VALUE)
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CURRENT				0
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_FULL_CAPACITY_IN_MAH	3190
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CAPACITY_IN_MAH		1000
#define GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_TIME_TO_EMPTY			0

#define _PROPERTY_AND_REG_SPEC(_addr, _min_value, _max_value) { \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}

/* Battery state-of-health */
#define SOH_TEMP   0x06
#define SOH_VOLT   0x08
#define SOH_CUR    0x14
#define SOH_FCC    0x12
#define SOH_DC     0x2E
#define SOH_RM     0x10
#define SOH_CC     0x1E
#define SOH    0x1C
#define SOH_READY_MASK  0x300
#define SOH_READY_SHIFT 8
#define SOH_READY       0x3
#define SOH_PER_MASK    0xFF
#define SOH_PER_SHIFT   0

static bool gauge_FW_is_updating = false;
static bool gauge_bq27520_driver_ready = false;
#ifdef USER_IMAGE
static bool pad_gaugeIC_firmware_is_updated = true;
static bool image_is_user = true;
#else
static bool pad_gaugeIC_firmware_is_updated = false;
static bool image_is_user = false;
#endif
static struct gauge_bq27520_struct *gauge_bq27520 = NULL;
static struct workqueue_struct *gauge_bq27520_workqueue = NULL;
char gaugeIC_firmware_message[100];
static int delay_time = 0x07;

static bool piter_test_bit = false;

static DEFINE_MUTEX(read_mutex_lock);

enum which_battery_enum{
	dock_device_battery = 0,
	pad_device_battery,
};

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

enum bq27520_REGs{
       REG_CONTROL = 0,
	REG_AT_RATE,
	REG_AT_RATE_TIME_TO_EMPTY,
	REG_TEMPERATURE,						//temperature
	REG_VOLTAGE,							//voltage
	REG_FLAGS,
	REG_NOMINAL_AVAILABLE_CAPACTY,
	REG_FULL_AVAILABLE_CAPACITY,
	REG_REMAINING_CAPACITY,				//capacity_in_mAh
	REG_FULL_CHARGE_CAPACITY,
	REG_AVERAGE_CURRENT,					//current
	REG_TIME_TO_EMPTY,
	REG_STANDBY_CURRENT,
	REG_STANDBY_TIME_TO_EMPTY,			//time_to_empty
	REG_STATE_OF_HEALTH,
	REG_CYCLE_COUNT,
	REG_STATE_OF_CHARGE,					//capacity_in_percent
	REG_INSTANTANEOUS_CURRENT,
	REG_INTERNAL_TEMPERATURE,
	REG_RESISTANCE_SCALE,
	REG_OPERATION_CONFIGURATION,
	REG_DESIGN_CAPACITY,
	REG_RM,
	REG_FCC,
	REG_BSOC,
};

struct gauge_bq27520_struct{
	struct i2c_client		*client;
	struct i2c_client		*ROM_mode_client;
	struct gauge_bq27520_platform_data_struct *gauge_bq27520_platform_data;
	struct delayed_work	battery_status_reupdate_at_booting;
	struct wake_lock gaugeIC_updating;
	struct switch_dev gauge_sdev;
};

static struct property_and_reg_spec {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} bq27520_property_and_reg_spec[] = {
      [REG_CONTROL] = 						_PROPERTY_AND_REG_SPEC(0x00, 0, 65535),
	[REG_AT_RATE] = 						_PROPERTY_AND_REG_SPEC(0x02, 0, 65535),
	[REG_AT_RATE_TIME_TO_EMPTY] = 		_PROPERTY_AND_REG_SPEC(0x04, 0, 65535),
	[REG_TEMPERATURE] = 				_PROPERTY_AND_REG_SPEC(0x06, 0, 65535),
	[REG_VOLTAGE] = 						_PROPERTY_AND_REG_SPEC(0x08, 0, 65535),
	[REG_FLAGS] = 						_PROPERTY_AND_REG_SPEC(0x0A, 0, 65535),
	[REG_NOMINAL_AVAILABLE_CAPACTY] = 	_PROPERTY_AND_REG_SPEC(0x0C, 0, 65535),
	[REG_FULL_AVAILABLE_CAPACITY] = 		_PROPERTY_AND_REG_SPEC(0x0E, 0, 65535),
	[REG_REMAINING_CAPACITY] = 			_PROPERTY_AND_REG_SPEC(0x10, 0, 65535),
	[REG_FULL_CHARGE_CAPACITY] = 		_PROPERTY_AND_REG_SPEC(0x12, 0, 65535),
	[REG_AVERAGE_CURRENT] = 			_PROPERTY_AND_REG_SPEC(0x14, 0, 65535),
	[REG_TIME_TO_EMPTY] = 				_PROPERTY_AND_REG_SPEC(0x16, 0, 65535),
	[REG_STANDBY_CURRENT] = 			_PROPERTY_AND_REG_SPEC(0x18, 0, 65535),
	[REG_STANDBY_TIME_TO_EMPTY] = 		_PROPERTY_AND_REG_SPEC(0x1A, 0, 65535),
	[REG_STATE_OF_HEALTH] = 				_PROPERTY_AND_REG_SPEC(0x1C, 0, 65535),
	[REG_CYCLE_COUNT] = 				_PROPERTY_AND_REG_SPEC(0x1E, 0, 65535),
	[REG_STATE_OF_CHARGE] = 			_PROPERTY_AND_REG_SPEC(0x20, 0, 65535),
	[REG_INSTANTANEOUS_CURRENT] = 		_PROPERTY_AND_REG_SPEC(0x22, 0, 65535),
	[REG_INTERNAL_TEMPERATURE] = 		_PROPERTY_AND_REG_SPEC(0x28, 0, 65535),
	[REG_RESISTANCE_SCALE] = 			_PROPERTY_AND_REG_SPEC(0x2A, 0, 65535),
	[REG_OPERATION_CONFIGURATION] = 	_PROPERTY_AND_REG_SPEC(0x2C, 0, 65535),
	[REG_DESIGN_CAPACITY] = 				_PROPERTY_AND_REG_SPEC(0x2E, 0, 65535),
	[REG_RM] = 							_PROPERTY_AND_REG_SPEC(0x6C, 0, 65535),
	[REG_FCC] = 							_PROPERTY_AND_REG_SPEC(0x12, 0, 65535),
	[REG_BSOC] = 						_PROPERTY_AND_REG_SPEC(0x74, 0, 65535),
};

static int _gauge_bq27520_read_i2c(u8 reg, int *return_value, int is_byte)
{
	struct i2c_client *client = gauge_bq27520->client;
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
		if (!is_byte)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!is_byte)
				*return_value = get_unaligned_le16(data);
			else
				*return_value = data[0];

			return 0;
		}
	}
	return err;
}

static int gauge_bq27520_smbus_read_data(int reg_offset,int is_byte,int *return_value)
{
     s32 ret=-EINVAL;
     int count=0;

	mutex_lock(&read_mutex_lock);

	do {
		ret = _gauge_bq27520_read_i2c(bq27520_property_and_reg_spec[reg_offset].addr, return_value, is_byte);
	} while((ret<0)&&(++count<=SMBUS_RETRY));

	mutex_unlock(&read_mutex_lock);
	return ret;
}

static int gauge_bq27520_smbus_write_data(int reg_offset,int is_byte, unsigned int value)
{
     s32 ret = -EINVAL;
     int count=0;

	do{
		if(is_byte){
			ret = i2c_smbus_write_byte_data(gauge_bq27520->client,bq27520_property_and_reg_spec[reg_offset].addr,value&0xFF);
		}
		else{
			ret = i2c_smbus_write_word_data(gauge_bq27520->client,bq27520_property_and_reg_spec[reg_offset].addr,value&0xFFFF);
		}
	}while((ret<0) && (++count<=SMBUS_RETRY));

	return ret;
}

static int bq27520_get_control_register_data(int sub_command)
{
	int return_value;

	if(gauge_bq27520_smbus_write_data(REG_CONTROL, 0, sub_command) < 0){
		printk("get FW device type write error\n");
		return -EINVAL;
	}

	msleep(50);	// for gaugeIC buffter time

	if(gauge_bq27520_smbus_read_data(REG_CONTROL, 0, &return_value) < 0){
		printk("get FW device type read error\n");
		return -EINVAL;
	}

	return return_value;
}

u16 bq27520_get_gauge_FW_control_status(void)
{
	#define CONTROL_STATUS 0x0000

	return bq27520_get_control_register_data(CONTROL_STATUS);
}

u16 bq27520_get_gauge_FW_device_type(void)
{
	#define DEVICE_TYPE 0x0001

	return bq27520_get_control_register_data(DEVICE_TYPE);
}

u16 bq27520_get_gauge_FW_firmware_version(void)
{
	#define FW_VERSION 0x0002

	return bq27520_get_control_register_data(FW_VERSION);
}

u16 bq27520_get_gauge_FW_chemical_ID(void)
{
	#define CHEM_ID 0x0008

	return bq27520_get_control_register_data(CHEM_ID);
}

u16 bq27520_get_gauge_FW_DF_version(void)
{
	#define DF_VERSION 0x001f

	return bq27520_get_control_register_data(DF_VERSION);
}

int bq27520_get_gauge_ADC_alert_pin(void)
{
	return (gauge_bq27520->gauge_bq27520_platform_data->ADC_alert_pin < 0) ?
		-EINVAL : gauge_bq27520->gauge_bq27520_platform_data->ADC_alert_pin;
}

int bq27520_get_battery_voltage_min_design(void)
{
	return (gauge_bq27520->gauge_bq27520_platform_data->voltage_min_design < 0) ?
		0 : gauge_bq27520->gauge_bq27520_platform_data->voltage_min_design;
}

int bq27520_get_battery_voltage_max_design(void)
{
	return (gauge_bq27520->gauge_bq27520_platform_data->voltage_max_design < 0) ?
		0 : gauge_bq27520->gauge_bq27520_platform_data->voltage_max_design;
}

int bq27520_get_battery_energy_full_in_watt_design(void)
{
	return (gauge_bq27520->gauge_bq27520_platform_data->energy_full_design < 0) ?
		0 : gauge_bq27520->gauge_bq27520_platform_data->energy_full_design;
}

int bq27520_get_battery_flags(void)
{
	int return_value;

	if(gauge_bq27520_smbus_read_data(REG_FLAGS, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_cycle_count(void)
{
	int return_value;

	if(gauge_bq27520_smbus_read_data(REG_CYCLE_COUNT, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_full_capacity_in_mAh(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_FULL_CAPACITY_IN_MAH;

	if(gauge_bq27520_smbus_read_data(REG_FULL_CHARGE_CAPACITY, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_BSOC(void)
{
	int return_value;

	if(gauge_bq27520_smbus_read_data(REG_BSOC, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_FCC(void)
{
	int return_value;

	if(gauge_bq27520_smbus_read_data(REG_FCC, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}
EXPORT_SYMBOL(bq27520_get_battery_FCC);

int bq27520_get_battery_RM(void)
{
	int return_value;

	if(gauge_bq27520_smbus_read_data(REG_RM, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_capacity_in_mAh(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CAPACITY_IN_MAH;

	if(gauge_bq27520_smbus_read_data(REG_REMAINING_CAPACITY, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_capacity_in_percent(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CAPACITY_IN_PERCENT;

	if(gauge_bq27520_smbus_read_data(REG_STATE_OF_CHARGE, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_time_to_empty(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_TIME_TO_EMPTY;

	if(gauge_bq27520_smbus_read_data(REG_TIME_TO_EMPTY, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_current(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_CURRENT;

	if(gauge_bq27520_smbus_read_data(REG_AVERAGE_CURRENT, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_voltage(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_VOLTAGE;

	if(gauge_bq27520_smbus_read_data(REG_VOLTAGE, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_temperature(void)
{
	int return_value;

	if(gauge_FW_is_updating)
		return GAUGE_FW_IS_UPDATING_RETURN_CONSTANT_TEMPERATURE;

	if(gauge_bq27520_smbus_read_data(REG_TEMPERATURE, 0, &return_value) < 0)
		return -EINVAL;

	return return_value;
}

int bq27520_get_battery_health_or_present(void)
{
	return POWER_SUPPLY_HEALTH_GOOD;
}

int bq27520_get_battery_charging_status(void)
{
	//0x0080 INITIALIZED: set means can get battery status, or it may be in initialling
	//0x0040 DISCHARGING: set means battery is being discharging; note cleared doesn't mean battery is being charging.
	//0x0020 FULLY_CHARGED:
	//0x0010 FULLY_DISCHARGED:
	int temp_return_value, gauge_return_value;
	#define FULL_CHARGED		0x0200
	#define FULL_DISCHARGED	0x0002
	#define DISCHARGING			0x0001

	temp_return_value = 0x0000;

	if(gauge_bq27520_smbus_read_data(REG_FLAGS, 0, &gauge_return_value) >= 0)
		temp_return_value |= 0x0080;
	else
		return -EINVAL;

	if(gauge_return_value & FULL_CHARGED)
		temp_return_value |= 0x0020;

	if(gauge_return_value & FULL_DISCHARGED)
		temp_return_value |= 0x0010;

	if(gauge_return_value & DISCHARGING)
		temp_return_value |= 0x0040;

	return temp_return_value;
}

int __battery_current_source_from_pmic_to_realistic_battery_current_source(u8 battery_current_source)
{
	// return 0 if the data of battery_current_source is wrong
	int realistic_battery_current_source = 0;

        switch(battery_current_source){
                case 0x01:
                        realistic_battery_current_source = 1.125;
                        break;

                case 0x02:
                        realistic_battery_current_source = 2.25;
                        break;

                case 0x03:
                        realistic_battery_current_source = 4.5;
                        break;

                case 0x04:
                        realistic_battery_current_source = 9;
                        break;

                case 0x05:
                        realistic_battery_current_source = 18;
                        break;

                case 0x06:
                        realistic_battery_current_source = 36;
                        break;

                case 0x07:
                        realistic_battery_current_source = 72;
                        break;

                case 0x08:
                        realistic_battery_current_source = 144;
                        break;

                default:
                        realistic_battery_current_source = 0;
                        break;
        }

	return realistic_battery_current_source;
}

int __transform_battery_resistance_from_pmic_to_realistic_resistance(u8 MSB_data, u8 LSB_data)
{
	// refer to intel moorefield PMIC document
	// BATTERY_CURSRC = MSB_data >> 4;
	// batteryID_ADC_code = (MSB_data & 0x0f) << 8 + LSB_data;
	// Resistance = (batteryID_ADC_code * 293uV) / current source
	//
	// return 0 resistance if there's some error
	u8 battery_current_source;
	u16 batteryID_ADC_code;
	int realistic_battery_current_source = 0;
	int realistic_batteryID_ACD_code = 0;
	int realistic_battery_resistance = 0;

	battery_current_source = MSB_data >> 4;
	batteryID_ADC_code = ((MSB_data & 0x0f) << 8) + LSB_data;


	// battery_current_source -> realistic_battery_current_source
	realistic_battery_current_source = __battery_current_source_from_pmic_to_realistic_battery_current_source(battery_current_source);
	if(realistic_battery_current_source == 0){
		printk("gauge_bq27520: get battery_current_source fail\n");
		return 0;
	}


	// batteryID_ADC_code -> realistic_batteryID_ACD_code
        realistic_batteryID_ACD_code = (int)batteryID_ADC_code;


	// Resistance = (batteryID_ADC_code * 293uV) / current source
	// we have avoid the case of realistic_battery_current_source = 0 above
	realistic_battery_resistance = (realistic_batteryID_ACD_code * 293) / realistic_battery_current_source;


	printk("gauge_bq27520: realistic battery resistance: %d\n", realistic_battery_resistance);
	printk("gauge_bq27520: battery_current_source: 0x%02x, batteryID_ADC_code: 0x%04x\n", battery_current_source, batteryID_ADC_code);
	printk("gauge_bq27520: realistic_battery_current_source: %d, realistic_batteryID_ACD_code: %d\n", realistic_battery_current_source, realistic_batteryID_ACD_code);

	return realistic_battery_resistance;
}

int bq27520_get_battery_resistance_from_pmic(void)
{
	u8 MSB_data;
	u8 LSB_data;
	int battery_resistance = 0;

	intel_scu_ipc_ioread8(PMIC_BATTID_MSB_RESULT_REG, &MSB_data);
	intel_scu_ipc_ioread8(PMIC_BATTID_LSB_RESULT_REG, &LSB_data);
	printk("piter test: batterID_MSB: 0x%02x, batteryID_LSB: 0x%02x\n", MSB_data, LSB_data);

	battery_resistance = __transform_battery_resistance_from_pmic_to_realistic_resistance(MSB_data, LSB_data);

	printk("gauge_bq27520: battery_resistance: %d\n", battery_resistance);

	return battery_resistance;
}

u16 bq27520_get_batteryID_calculate_from_pmic(void)
{
	// the resistance of battery in ASUS definition are 10k, 51k, 75k, 100k
	// return 0 if it is not ASUS's battery
	u16 batteryID_calculate_from_pmic = 0x0000;
	int battery_resistance = 0;

	battery_resistance = bq27520_get_battery_resistance_from_pmic();

	if(battery_resistance <= 11000 && battery_resistance >= 9000)
		batteryID_calculate_from_pmic = 0x0010;		// 10k battery
	else if(battery_resistance <= 56000 && battery_resistance >= 46000)
		batteryID_calculate_from_pmic = 0x0051;		// 51k battery
	else if(battery_resistance <= 83000 && battery_resistance >= 67000)
		batteryID_calculate_from_pmic = 0x0075;		// 75k battery
	else if(battery_resistance <= 110000 && battery_resistance >= 90000)
		batteryID_calculate_from_pmic = 0x0100;		// 100k battery
	else
		batteryID_calculate_from_pmic = 0x0000;

	printk("gauge_bq27520: batteryID_from_pmic: 0x%04x\n", batteryID_calculate_from_pmic);

	return batteryID_calculate_from_pmic;
}

int bq27520_get_if_pad_gaugeIC_firmware_is_updated(void)
{
	return pad_gaugeIC_firmware_is_updated;
}

static bool bq27520_check_if_pad_gaugeIC_firmware_is_updated(void)
{
	u16 gaugeIC_firmware_info;
	int retry_times = 0;
	enum which_battery_enum which_battery = pad_device_battery;
	#define MAX_RETRY_TIME 3

	do{
		printk("bq27520: check if pad gaugeIC's firmware is updated...\n");
		printk("bq27520: origin gaugeIC is updated status: %d\n", pad_gaugeIC_firmware_is_updated);

		pad_gaugeIC_firmware_is_updated = true;

		gaugeIC_firmware_info = bq27520_get_gauge_FW_device_type();
		printk("gaugeIC: %x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info != 0x0520){
			printk("DEVICE_TYPE not 0x520\n");
			pad_gaugeIC_firmware_is_updated = false;
		}

		gaugeIC_firmware_info = bq27520_get_gauge_FW_firmware_version();
		printk("FW_VERSION: %x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info != 0x0329){
			printk("FW_VERSION not 0x0329\n");
			pad_gaugeIC_firmware_is_updated = false;
		}

		//gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		//if(!(gaugeIC_firmware_info == 0x0000 || gaugeIC_firmware_info == 0x0001))
		//	pad_gaugeIC_firmware_is_updated = false;

		gaugeIC_firmware_info = bq27520_get_gauge_FW_DF_version();
		printk("DF_VERSION: %x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info < 0x0001 || gaugeIC_firmware_info >= 0x0500){
			printk("DF_VERSION is too old\n");
			pad_gaugeIC_firmware_is_updated = false;
		}

		printk("bq27520: pad gaugeIC's firmware is updated: %d,\n", pad_gaugeIC_firmware_is_updated);

		retry_times++;
	}while((pad_gaugeIC_firmware_is_updated == false) && retry_times <= MAX_RETRY_TIME);

	return pad_gaugeIC_firmware_is_updated;
}

static int __write_or_read_or_sleep_gauge_firmware_data(u8 dffs_command, u8 buff[], u8 buff_length, bool pad)
{
	struct i2c_client *client = NULL;
	u8 data_length = buff_length - 2;		// 16 00 13 (address reg data) data_length = 3 - 2
	u8 read_buff[data_length];
	int i;
	int sleep_time;

	if(dffs_command != update_command_sleep){
		if(buff[0] == 0xAA)		// normail mode address
			client = gauge_bq27520->client;
		else if(buff[0] == 0x16)	// rom mode address
			client = gauge_bq27520->ROM_mode_client;
		else						// invalid address
			return -EINVAL;
	}

	switch(dffs_command){
		case update_command_read:	//read command
			if(i2c_smbus_read_i2c_block_data(client, buff[1], data_length, &read_buff) < 0){
				printk("error readr: i2c_smbus_read_i2c_block_data return error\n");
				return -EINVAL;
			}

			printk("gauge check return data: ");
			for(i = 0; i < data_length; i++)
				printk("%02x", read_buff[i]);
			printk("\n");

			for(i = 0; i < data_length; i++){		// check the data of the check sum
				if(read_buff[i] != buff[2+i]){
					printk("error check: i = %d, command data = %02x, gauge return data = %02x", i, buff[2+i], read_buff[i]);
					return -EINVAL;
				}
			}

			break;
		case update_command_write:	// write command
			if(i2c_smbus_write_i2c_block_data(client, buff[1], data_length, &buff[2]) < 0){
				printk("error write: i2c_smbus_write_i2c_block_data error\n");
				return -EINVAL;
			}

			break;
		case update_command_sleep:	// sleep command
			if(buff_length == 2){
				sleep_time = buff[0] * 256 + buff[1];
				printk("sleep time = %d\n", sleep_time);
				msleep(sleep_time);
			}else{
				printk("error: invalid sleep buff length\n");
				return -EINVAL;
			}

			break;
		default:
			printk("error: invalid dffs_command\n");
			return -EINVAL;
			break;
	}

	return 0;
}

static enum gaugeIC_update_ret_value analyze_dffs_string_to_hexadecimal_and_send_data(
	char dffs_string[], int string_length, enum which_battery_enum which_battery)
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

			printk("dffs_string: comment: %s\n", temp_char);

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

			printk("dffs_string: write %d bytes: ", send_byte_count);
			j = 0;
			for(j = 0; j < send_byte_count; j++)
				printk("%02x ", temp_result_array[j]);

			printk("\n");

			do{
				EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_write, temp_result_array, send_byte_count,
					which_battery);
				re_send_to_EC_count++;
			} while((EC_return_vaule <0) && (re_send_to_EC_count < RE_SEND_TO_EC_TIMES));

			gaugeIC_update_return_value = (EC_return_vaule <0) ? error_write : update_success;

			return gaugeIC_update_return_value;

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

			printk("dffs_string: check %d bytes: ", send_byte_count);
			j = 0;
			for(j = 0; j < send_byte_count; j++)
				printk("%02x ", temp_result_array[j]);

			printk("\n");

			do{
				EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_read, temp_result_array,send_byte_count,
					which_battery);
				re_send_to_EC_count++;
			}while((EC_return_vaule <0) && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

			gaugeIC_update_return_value = (EC_return_vaule < 0) ? error_read : update_success;

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
			printk("dffs_string: wait: %02x, %02x\n", temp_result_array[0], temp_result_array[1]);

			do{
				EC_return_vaule = __write_or_read_or_sleep_gauge_firmware_data(update_command_sleep, temp_result_array, 2,
					which_battery);
				re_send_to_EC_count++;
			}while(EC_return_vaule < 0 && re_send_to_EC_count < RE_SEND_TO_EC_TIMES);

			gaugeIC_update_return_value = (EC_return_vaule < 0) ? error_sleep : update_success;

			return gaugeIC_update_return_value;

			break;
		default:
			printk("error\n");

			break;
	}

	return 0;
}

static int __update_gaugeIC_firmware_data_flash_blocks(struct file *fp,
	enum which_battery_enum which_battery)
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

	printk("gaugeIC: update data flash blocks...\n");

	if (!(fp->f_op) || !(fp->f_op->read)){
		printk("gaugeIC: update data flash blocks error: no dffs file operation\n");
		return -EINVAL;
	}

	do{
		ret = fp->f_op->read(fp,&temp_char,1, &fp->f_pos);
		if(ret > 0){
			if(temp_char != '\n'){
				dffs_string[i] = temp_char;

				i++;
			}else{
				dffs_string[i] = '\0';

				printk("line %d: %s\n", dffs_line_count, dffs_string);

				gaugeIC_update_return_value =analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
												max_dffs_string_length, which_battery);

				switch(gaugeIC_update_return_value){
					case error_sleep:
					case error_check:
					case error_write:
					case error_read:
						printk("gauge_bq27520: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
						fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
						gaugeIC_check_error_count++;
						i = 0;
						dffs_line_count = 0;

						if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
							return -EINVAL;

						break;

					case update_success:
						printk("gauge_bq27520: gaugeIC firmware update: success\n");
						i = 0;
						dffs_line_count++;
						break;

					default:
						printk("gauge_bq27520: gaugeIC firmware update: no this case\n");
						return -EINVAL;
						break;
				}

			}
		}else{	// last line
			if(i != 0){
				dffs_string[i] = '\0';

				printk("line %d: %s\n", dffs_line_count, dffs_string);

				gaugeIC_update_return_value = analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,
												max_dffs_string_length, which_battery);

				switch(gaugeIC_update_return_value){
					case error_sleep:
					case error_check:
					case error_write:
					case error_read:
						printk("gauge_bq27520: gaugeIC firmware update: error = %d , error time = %d\n", gaugeIC_update_return_value, gaugeIC_check_error_count);
						fp->f_pos = F_POS_WHEN_GAUGEIC_UPDATE_CHECK_ERROR;
						gaugeIC_check_error_count++;
						i = 0;
						dffs_line_count = 0;
						ret = 1;

						if(gaugeIC_check_error_count> GAUGEIC_CHECK_ERROR_RELOAD_DFFS_TIMES)
							return -EINVAL;

						break;

					case update_success:
						printk("gauge_bq27520: gaugeIC firmware update: success\n");
						i = 0;
						dffs_line_count++;
						break;

					default:
						printk("gauge_bq27520: gaugeIC firmware update: no this case\n");
						return -EINVAL;
						break;
				}

			}
		}
	}while(ret > 0);


	printk("gaugeIC: update data flash blocks success\n");

	return 0;
}

static int gaugeIC_exit_ROM_mode(enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	char dffs_string[max_dffs_string_length];

	printk("gaugeIC: exit ROM mode...\n");

	strcpy(dffs_string, "W: 16 00 0F");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: W: 16 00 0F update success\n");

	strcpy(dffs_string, "W: 16 64 0F 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: W: 16 64 0F 00 update success\n");

	strcpy(dffs_string, "X: 4000");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: X: 4000 update success\n");

	strcpy(dffs_string, "W: AA 00 20 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: X: 20 update success\n");

	strcpy(dffs_string, "W: AA 00 20 00");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: W: AA 00 20 00 update success\n");

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: exit ROM mode: X: 20 update success\n");

	printk("gaugeIC: exit ROM mode success\n");

	return 0;
}

static int gaugeIC_enter_ROM_mode(enum which_battery_enum which_battery)
{
	int max_dffs_string_length = 256;
	char dffs_string[max_dffs_string_length];

	printk("gaugeIC: enter ROM mode...\n");

	strcpy(dffs_string, "W: AA 00 14 04");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: W: AA 00 14 04 update success\n");


	strcpy(dffs_string, "W: AA 00 72 16");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: W: AA 00 72 16 update success\n");

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: X: 20 update success\n");

	strcpy(dffs_string, "W: AA 00 FF FF");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

	strcpy(dffs_string, "W: AA 00 FF FF");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: W: AA 00 FF FF update success\n");

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: X: 20 update success\n");

	strcpy(dffs_string, "W: AA 00 00 0F");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: W: AA 00 00 0F update success\n");

	strcpy(dffs_string, "X: 20");
	if(analyze_dffs_string_to_hexadecimal_and_send_data(dffs_string,	max_dffs_string_length,
		which_battery) != update_success)
		return -EINVAL;
	printk("gauge: enter ROM mode: X: 20 update success\n");

	printk("gaugeIC: enter ROM mode success\n");

	return 0;
}

static enum gaugeIC_mode_enum get_gaugeIC_mode(void)
{
	// normal mode: if we can get battery information(such as capacity).
	// ROM mode:    if we can get ROM mode data(we choose 0x66, since we found some check sum address is 0x66).
	// may broken:   we can't get battery information and can't get ROM mode data.
	int return_value;
	u8 ROM_mode_read_buff[1];
	int retry_times = 0;
	#define MAX_RETRY_TIME 3

	do{
		if(gauge_bq27520_smbus_read_data(REG_REMAINING_CAPACITY, 0, &return_value) >= 0)
		return normal_mode;

		if(i2c_smbus_read_i2c_block_data(gauge_bq27520->ROM_mode_client, 0x66, 1, &ROM_mode_read_buff) >= 0)
		return ROM_mode;

		retry_times++;
	}while(retry_times <= MAX_RETRY_TIME);

	return may_broken;
}

static bool check_if_gaugeIC_in_ROM_mode(enum which_battery_enum which_battery)
{
	enum gaugeIC_mode_enum gaugeIC_mode = may_broken;

	gaugeIC_mode = get_gaugeIC_mode();

	return (gaugeIC_mode == normal_mode) ? false : true;	// since sometimes it will check error, so if not normal, feedback it is in rom mode
}

static int update_gaugeIC_firmware(struct file *fp, enum which_battery_enum which_battery)
{
	int ret = 0;

	if(!check_if_gaugeIC_in_ROM_mode(which_battery))
		ret = gaugeIC_enter_ROM_mode(which_battery);

	if(ret !=0)
		return -EINVAL;

	ret = __update_gaugeIC_firmware_data_flash_blocks(fp, which_battery);

	if(ret != 0)
		return -EINVAL;

	ret = gaugeIC_exit_ROM_mode(which_battery);

	return ret;
}

static bool check_if_can_update_gaugeIC_firmware(enum which_battery_enum which_battery)
{
	u16 gaugeIC_firmware_info;
	bool can_update_gaugeIC_firmware = true;
	int retry_times = 0;
	#define MAX_RETRY_TIME 3

	if(check_if_gaugeIC_in_ROM_mode(which_battery))
		return true;

	do{
		gaugeIC_firmware_info = bq27520_get_gauge_FW_device_type();
		printk("gaugeIC:%x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info != 0x0520)
			can_update_gaugeIC_firmware = false;

		gaugeIC_firmware_info = bq27520_get_gauge_FW_firmware_version();
		printk("FW_VERSION:%x\n", gaugeIC_firmware_info);
		if(gaugeIC_firmware_info != 0x0329)
			can_update_gaugeIC_firmware = false;
	/*	piter: have to add it after battery ID can get from pin
		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
		printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
		if(!(gaugeIC_firmware_info == 0x0000 || gaugeIC_firmware_info == 0x0001))
			can_update_gaugeIC_firmware = false;
	*/
		retry_times++;
	}while((can_update_gaugeIC_firmware == false) && retry_times <= MAX_RETRY_TIME);

	return can_update_gaugeIC_firmware;
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

static ssize_t switch_get_gauge_FW_DF_version__print_name(struct device *dev, struct device_attribute *devattr, char *buf)
{
	u16 gauge_DF_version;

	gauge_DF_version = bq27520_get_gauge_FW_DF_version();
	printk("gauge bq27520: DF_VERSION: %x\n", gauge_DF_version);
	if(gauge_DF_version < 0x0001 || gauge_DF_version >= 0x0500){
		printk("DF_VERSION is too old\n");
		gauge_DF_version = 0x0;
	}

	return sprintf(buf, "V%x\n", gauge_DF_version);
}

static ssize_t sysfs_gauge_bq27520_I2C_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	// 0: i2c communicate fail
	// 1: i2c communicate success
	int i2c_status = 0;
	int ret = 0;

	i2c_status = bq27520_get_battery_capacity_in_percent();

	ret = (i2c_status < 0) ? 0 : 1;

	return sprintf(buf, "%d\n", ret);
}
static ssize_t sysfs_gauge_bq27520_get_pad_gaugeIC_mode_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	// 0: not normal mode and not rom mode. the gauge may broken.
	// 1: gaugeIC in rom mode.
	// 2: gaugeIC in normal mode.

	enum gaugeIC_mode_enum gaugeIC_mode = may_broken;
	char gaugeIC_mode_in_sentence[15];

	gaugeIC_mode = get_gaugeIC_mode();
	printk("gaugeIC mode: %d\n", gaugeIC_mode);
	switch(gaugeIC_mode){
		case may_broken:		// not normal mode and not rom mode. the gauge may broken.
			strcpy(gaugeIC_mode_in_sentence, "may broken");
			break;
		case ROM_mode:		// gaugeIC in rom mode.
			strcpy(gaugeIC_mode_in_sentence, "rom mode");
			break;
		case normal_mode:	// gaugeIC in normal mode.
			strcpy(gaugeIC_mode_in_sentence, "normal mode");
			break;
		default:				// no this case
			strcpy(gaugeIC_mode_in_sentence, "no this case");
	}

	return sprintf(buf, "gaugeIC mode: %s\n", gaugeIC_mode_in_sentence);
}

static ssize_t sysfs_gauge_bq27520_gaugeIC_FW_version_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	u16 gaugeIC_firmware_info;
	char gaugeIC_name[10];
	char gaugeIC_FW_version[10];
	u16 gaugeIC_DF_version;
	u16 gaugeIC_battery_ID_calculate_from_pmic;
	u16 gaugeIC_battery_ID_from_gaugeIC;
	u16 cycle_count;

	gaugeIC_firmware_info = bq27520_get_gauge_FW_device_type();
	printk("gaugeIC:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0520)
		strcpy(gaugeIC_name, "bq27520");
	else
		strcpy(gaugeIC_name, "unknow");

	gaugeIC_firmware_info = bq27520_get_gauge_FW_firmware_version();
	printk("FW_VERSION:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0329)
		strcpy(gaugeIC_FW_version, "G4");
	else
		strcpy(gaugeIC_FW_version, "unknow");

	gaugeIC_DF_version = bq27520_get_gauge_FW_DF_version();
	printk("DF_VERSION:%x\n", gaugeIC_DF_version);

	gaugeIC_firmware_info = bq27520_get_batteryID_calculate_from_pmic();
	printk("BATTERY_ID_CALCULATE_FROM_PMIC:%x\n", gaugeIC_firmware_info);
	if(gaugeIC_firmware_info == 0x0010)
		gaugeIC_battery_ID_calculate_from_pmic = 0x0010;
	else if(gaugeIC_firmware_info == 0x0051)
		gaugeIC_battery_ID_calculate_from_pmic = 0x0051;
	else if(gaugeIC_firmware_info == 0x0075)
		gaugeIC_battery_ID_calculate_from_pmic = 0x0075;
	else if(gaugeIC_firmware_info == 0x0100)
		gaugeIC_battery_ID_calculate_from_pmic = 0x0100;
	else
		gaugeIC_battery_ID_calculate_from_pmic = 0xFFFF;

	gaugeIC_battery_ID_from_gaugeIC = bq27520_get_gauge_FW_chemical_ID();
	printk("BATTERY_ID_FROM_GAUGEIC:%x\n", gaugeIC_battery_ID_from_gaugeIC);

	cycle_count = bq27520_get_battery_cycle_count();
	printk("cycle_count = %d", cycle_count);

	return sprintf(buf, "gaugeIC: %s_%s, DF_version: %x, battery ID: %x, battery ID in gaugeIC: %x, cycle count = %d\n", gaugeIC_name, gaugeIC_FW_version,
			gaugeIC_DF_version, gaugeIC_battery_ID_calculate_from_pmic, gaugeIC_battery_ID_from_gaugeIC, cycle_count);
}

static ssize_t sysfs_gauge_bq27520_battery_full_capacity_in_mAh_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int capacity;

	capacity = bq27520_get_battery_full_capacity_in_mAh();

	return sprintf(buf, "%d\n", capacity);
}

static ssize_t sysfs_gauge_bq27520_battery_BSOC(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int BSOC;

	BSOC = bq27520_get_battery_BSOC();

	return sprintf(buf, "%d\n", BSOC);
}

static ssize_t sysfs_gauge_bq27520_battery_FCC(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int FCC;

	FCC = bq27520_get_battery_FCC();

	return sprintf(buf, "%d\n", FCC);
}

static ssize_t sysfs_gauge_bq27520_battery_RM(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int RM;

	RM = bq27520_get_battery_RM();

	return sprintf(buf, "%d\n", RM);
}


static ssize_t sysfs_gauge_bq27520_battery_capacity_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int capacity;

	capacity = bq27520_get_battery_capacity_in_percent();

	return sprintf(buf, "%d\n", capacity);
}

static ssize_t sysfs_gauge_bq27520_battery_current_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int current_now;

	current_now = bq27520_get_battery_current();

	return sprintf(buf, "%d\n", current_now);
}

static ssize_t sysfs_gauge_bq27520_bq27520_FW_update_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long buf_type_in_num;
	int ret;
	struct file *fp;
	mm_segment_t old_fs;
	int max_dffs_string_length = 256;

	enum which_battery_enum which_battery = pad_device_battery;

	enum gaugeIC_mode_enum gaugeIC_mode = may_broken;

	if ( strict_strtol(buf, 0, &buf_type_in_num)){
		printk("gauge_bq27520: string to num fail\n");
		return -EINVAL;
	}


	if (buf_type_in_num == 1){
		piter_test_bit = !piter_test_bit;
		printk("test_bit: %d", piter_test_bit);
	}

	if (buf_type_in_num == 2){
		bq27520_check_if_pad_gaugeIC_firmware_is_updated();
	}


	if (buf_type_in_num == 7){
		gaugeIC_mode = get_gaugeIC_mode();

		printk("gaugeIC_mode = %d\n", gaugeIC_mode);
	}

	if (buf_type_in_num == 5){
		// update pad gaugeIC's firmware by dffs file in system image

		gauge_FW_is_updating = true;

		which_battery = pad_device_battery;

		printk("Battery: pad gaugeIC firmware updating...\n");

		if(!check_if_can_update_gaugeIC_firmware(which_battery)){
			printk("wrong gaugeIC, cannot update firmware\n");
			strcpy(gaugeIC_firmware_message, "wrong gaugeIC, cannot update firmware\n");
			gauge_FW_is_updating = false;
			return count;
		}

		wake_lock_timeout(&gauge_bq27520->gaugeIC_updating, 20*HZ);

		// piter: may add some function to let other stop to get battery information.

		old_fs = enable_kernel_to_read_file_and_return_origin_fs();

//		gaugeIC_firmware_info = asuspec_gauge_ic_monitor("BATTERY_ID_FROM_PIN", which_battery);
//		printk("BATTERY_ID_FROM_PIN:%x\n", gaugeIC_firmware_info);
//		if(gaugeIC_firmware_info == 0x0001)
			fp = open_file("/system/etc/firmware/battery_gauge/ME375_0368.dffs", O_RDONLY, 0);
//		else if(gaugeIC_firmware_info == 0x0000)
//			fp = open_file("/system/etc/firmware/battery_gauge/mozart_pad_0368.dffs", O_RDONLY, 0);
//		else
//			fp = NULL;

		if (!IS_ERR(fp) && fp!=NULL){
			ret = update_gaugeIC_firmware(fp, which_battery);
			if(ret == 0){
				printk("gaugeIC firware update success\n");
				strcpy(gaugeIC_firmware_message, "pad gaugeIC firware update success\n");
				pad_gaugeIC_firmware_is_updated = true;
			}
			else{
				printk("gaugeIC firmware update fail\n");
				strcpy(gaugeIC_firmware_message, "pad gaugeIC firmware update fail\n");
				pad_gaugeIC_firmware_is_updated = false;
			}

			close_file(fp);
		}else{
			printk("pad gaugeIC firmware update fail: no file\n");
			strcpy(gaugeIC_firmware_message, "gaugeIC firmware update fail: no file\n");
		}

		exit_kernel_to_read_file(old_fs);

		// piter: may add some function to let other start to get battery information.

		gauge_FW_is_updating = false;
	}

/*

	if (buf_type_in_num == 12){
		int return_value = bq27520_get_battery_current();
		printk("piter: currnet = %d\n", return_value);
		printk("piter: %d%02x\n", buf_type_in_num, delay_time);
	}
*/
	return count;
}

static ssize_t sysfs_gauge_bq27520_bq27520_FW_update_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	strcpy(gaugeIC_firmware_message, "gaugeIC firmware update fail: no file\n");
	return scnprintf(buf, PAGE_SIZE, "%s\n", gaugeIC_firmware_message);
}

static ssize_t sysfs_gauge_bq27520_battery_soh_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        int ret, r_FCC, r_DC, r_RM, r_temp, r_volt, r_cur, r_CC, r_SOH;

        ret = _gauge_bq27520_read_i2c(SOH_FCC, &r_FCC, 0);
        if (ret) {
                printk("error in reading battery FCC = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }

        ret = _gauge_bq27520_read_i2c(SOH_DC, &r_DC, 0);
        if (ret) {
                printk("error in reading battery DC = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }

        ret = _gauge_bq27520_read_i2c(SOH_RM, &r_RM, 0);
        if (ret) {
                printk("error in reading battery RM = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }

        ret = _gauge_bq27520_read_i2c(SOH_CC, &r_CC, 0);
        if (ret) {
                printk("error in reading battery CC = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }

        ret = _gauge_bq27520_read_i2c(SOH, &r_SOH, 0);
        if (ret) {
                printk("error in reading battery SOH = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }
        if ((r_SOH & SOH_READY_MASK) == (SOH_READY << SOH_READY_SHIFT))
                r_SOH &= (SOH_PER_MASK << SOH_PER_SHIFT);
        else
                r_SOH = -1;

        ret = _gauge_bq27520_read_i2c(SOH_TEMP, &r_temp, 0);
        if (ret) {
                printk("error in reading battery TEMP = %x\n", ret);
                return sprintf(buf, "Fail\n");
        } else {
		r_temp -= 2730;
		r_temp /= 10;
	}

        ret = _gauge_bq27520_read_i2c(SOH_VOLT, &r_volt, 0);
        if (ret) {
                printk("error in reading battery VOLT = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }

        ret = _gauge_bq27520_read_i2c(SOH_CUR, &r_cur, 0);
        if (ret) {
                printk("error in reading battery CUR = %x\n", ret);
                return sprintf(buf, "Fail\n");
        }
	r_cur = (s16)r_cur;

        if (r_SOH == -1)
		return sprintf(buf, "FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(°C),VOLT=%d(mV),CUR=%d(mA),CC=%d,SOH=invalid\n",
                        r_FCC, r_DC, r_RM, r_temp, r_volt, r_cur, r_CC);
        else
		return sprintf(buf, "FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(°C),VOLT=%d(mV),CUR=%d(mA),CC=%d,SOH=%d(%)\n",
                        r_FCC, r_DC, r_RM, r_temp, r_volt, r_cur, r_CC, r_SOH);
}

static ssize_t sysfs_gauge_bq27520_get_battery_all_information_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int battery_charging_status, battery_voltage, battery_capacity, battery_temperature, battery_current;
	int battery_capacity_in_mAh, battery_full_capacity_in_mAh, battery_time_to_empty, battery_cycle_count;
	int battery_flags, battery_heath_or_present;
	u16 fw_control_status, fw_device_type, fw_firmware_version, fw_chemical_ID, fw_DF_version;
	char temp_char[64];

	battery_charging_status = 		bq27520_get_battery_charging_status();
	battery_voltage = 				bq27520_get_battery_voltage();
	battery_capacity = 			bq27520_get_battery_capacity_in_percent();
	battery_temperature = 			bq27520_get_battery_temperature();
	battery_current = 				bq27520_get_battery_current();
	battery_capacity_in_mAh = 		bq27520_get_battery_capacity_in_mAh();
	battery_full_capacity_in_mAh =	bq27520_get_battery_full_capacity_in_mAh();
	battery_time_to_empty = 		bq27520_get_battery_time_to_empty();
	battery_heath_or_present = 		bq27520_get_battery_health_or_present();
	battery_flags =				bq27520_get_battery_flags();
	battery_cycle_count =			bq27520_get_battery_cycle_count();
	fw_control_status =			bq27520_get_gauge_FW_control_status();
	fw_device_type =				bq27520_get_gauge_FW_device_type();
	fw_firmware_version =			bq27520_get_gauge_FW_firmware_version();
	fw_chemical_ID =				bq27520_get_gauge_FW_chemical_ID();
	fw_DF_version =				bq27520_get_gauge_FW_DF_version();


	sprintf(temp_char, "charging status = 0x%x\n", battery_charging_status);
	strcpy(buf, temp_char);
	sprintf(temp_char, "voltage = %d\n", battery_voltage);
	strcat(buf, temp_char);
	sprintf(temp_char, "capacity = %d\n", battery_capacity);
	strcat(buf, temp_char);
	sprintf(temp_char, "temperature = %d\n", battery_temperature);
	strcat(buf, temp_char);
	sprintf(temp_char, "current = %d\n", battery_current);
	strcat(buf, temp_char);
	sprintf(temp_char, "capacity in mAh = %d\n", battery_capacity_in_mAh);
	strcat(buf, temp_char);
	sprintf(temp_char, "full capacity in mAh = %d\n", battery_full_capacity_in_mAh);
	strcat(buf, temp_char);
	sprintf(temp_char, "avg time to empty = %d\n", battery_time_to_empty);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery present = %d\n", battery_heath_or_present);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery Flags = 0x%x\n", battery_flags);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery cycle count = %d\n", battery_cycle_count);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery fw control status = 0x%x\n", fw_control_status);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery fw device type = 0x%x\n", fw_device_type);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery fw firmware version = 0x%x\n", fw_firmware_version);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery fw chemical ID= 0x%x\n", fw_chemical_ID);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery fw DF version = 0x%x\n", fw_DF_version);
	strcat(buf, temp_char);

	return strlen(buf);
}


static DEVICE_ATTR(gauge_bq27520_I2C_status, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_I2C_status_show, NULL);
static DEVICE_ATTR(gauge_bq27520_get_pad_gaugeIC_mode, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_get_pad_gaugeIC_mode_show, NULL);
static DEVICE_ATTR(gauge_bq27520_gaugeIC_FW_version, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_gaugeIC_FW_version_show, NULL);
static DEVICE_ATTR(gauge_bq27520_get_battery_all_information, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_get_battery_all_information_show, NULL);
static DEVICE_ATTR(gauge_bq27520_battery_capacity, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_capacity_show, NULL);
static DEVICE_ATTR(gauge_bq27520_battery_BSOC, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_BSOC, NULL);
static DEVICE_ATTR(gauge_bq27520_battery_RM, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_RM, NULL);
static DEVICE_ATTR(gauge_bq27520_battery_FCC, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_FCC, NULL);
static DEVICE_ATTR(gauge_bq27520_battery_current, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_current_show, NULL);
static DEVICE_ATTR(gauge_bq27520_bq27520_FW_update_enable, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_bq27520_FW_update_enable_show,
	sysfs_gauge_bq27520_bq27520_FW_update_enable_store);
static DEVICE_ATTR(gauge_bq27520_battery_soh, S_IWUSR | S_IRUGO,
	sysfs_gauge_bq27520_battery_soh_show, NULL);

static struct attribute *gauge_bq27520_attributes[] = {
	&dev_attr_gauge_bq27520_I2C_status.attr,
	&dev_attr_gauge_bq27520_get_pad_gaugeIC_mode.attr,
	&dev_attr_gauge_bq27520_gaugeIC_FW_version.attr,
	&dev_attr_gauge_bq27520_get_battery_all_information.attr,
	&dev_attr_gauge_bq27520_battery_capacity.attr,
	&dev_attr_gauge_bq27520_battery_BSOC.attr,
	&dev_attr_gauge_bq27520_battery_RM.attr,
	&dev_attr_gauge_bq27520_battery_FCC.attr,
	&dev_attr_gauge_bq27520_battery_current.attr,
	&dev_attr_gauge_bq27520_bq27520_FW_update_enable.attr,
	&dev_attr_gauge_bq27520_battery_soh.attr,
	NULL
};

static const struct attribute_group gauge_bq27520_group = {
	.attrs = gauge_bq27520_attributes,
};

static int gauge_bq27520_probe(struct i2c_client *client,	const struct i2c_device_id *id)
{
	char ROM_mode_name[20] = "bq27520_ROM_mode";

	printk("gauge_bq27520 probe++\n");

	gauge_bq27520 = devm_kzalloc(&client->dev, sizeof(*gauge_bq27520), GFP_KERNEL);
	if(!gauge_bq27520)
		return -ENOMEM;

	gauge_bq27520->client = client;

	gauge_bq27520->gauge_bq27520_platform_data = client->dev.platform_data;


	gauge_bq27520->ROM_mode_client = kzalloc(sizeof *client, GFP_KERNEL);
	*gauge_bq27520->ROM_mode_client = *gauge_bq27520->client;
	strlcpy(gauge_bq27520->ROM_mode_client->name, ROM_mode_name, gauge_bq27520->ROM_mode_client->name);
	gauge_bq27520->ROM_mode_client->addr = 0x0B;

//	INIT_DELAYED_WORK(&gauge_bq27520->check_pad_gaugeIC_firmware_is_updated,
//		check_if_pad_gaugeIC_firmware_is_updated_work_func);

	if(!image_is_user){
		printk("gauge_bq27520: it's not user image, thus check if gaugeIC is updated\n");
		pad_gaugeIC_firmware_is_updated = bq27520_check_if_pad_gaugeIC_firmware_is_updated();
	}else{
		pad_gaugeIC_firmware_is_updated = true;
		printk("it's user image, so assume gaugeIC is update:%d\n", pad_gaugeIC_firmware_is_updated);
	}

	wake_lock_init(&gauge_bq27520->gaugeIC_updating,	WAKE_LOCK_SUSPEND, "gaugeIC updating");

	i2c_set_clientdata(client,gauge_bq27520);


	if(sysfs_create_group(&client->dev.kobj, &gauge_bq27520_group))
		dev_err(&client->dev, "gauge_bq27520_probe: unable to create the sysfs\n");


	gauge_bq27520->gauge_sdev.name = "battery";
	gauge_bq27520->gauge_sdev.print_name = switch_get_gauge_FW_DF_version__print_name;
	if(switch_dev_register(&gauge_bq27520->gauge_sdev) < 0){
		dev_info(&client->dev, "switch_dev_register for gauge failed!\n");
	}

	switch_set_state(&gauge_bq27520->gauge_sdev, 0);


	gauge_bq27520_driver_ready = true;

	printk("gauge_bq27520 probe--\n");
	return 0;
}

static int gauge_bq27520_remove(struct i2c_client *client)
{
	wake_lock_destroy(&gauge_bq27520->gaugeIC_updating);
	return 0;
}

#if defined (CONFIG_PM)
static int gauge_bq27520_suspend(struct i2c_client *client, pm_message_t state)
{
	printk("pad battery by EC suspend+\n");
	printk("pad battery by EC suspend-\n");
	return 0;
}

static int gauge_bq27520_resume(struct i2c_client *client)
{
	printk("pad battery by EC resume+\n");
	printk("pad battery by EC resume-\n");
	return 0;
}
#endif


static const struct dev_pm_ops gauge_bq27520_pm_ops = {
	.suspend = gauge_bq27520_suspend,
	.resume = gauge_bq27520_resume,
};

static const struct i2c_device_id gauge_bq27520_id[] = {
	//{ "gauge_bq27520", 0 },
	{"hpa02254_batt", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, gauge_bq27520_id);

static struct i2c_driver gauge_bq27520_driver = {
	.driver = {
		.name = "hpa02254_batt",
		.owner = THIS_MODULE,
		#if defined (CONFIG_PM)
		.pm = &gauge_bq27520_pm_ops,
		#endif
	},
	.probe        = gauge_bq27520_probe,
	.remove      = gauge_bq27520_remove,
	.id_table     = gauge_bq27520_id,
};

static int __init gauge_bq27520_init(void)
{
	printk("gauge_bq27520: init.\n");
	return i2c_add_driver(&gauge_bq27520_driver);
}
module_init(gauge_bq27520_init);

static void __exit gauge_bq27520_exit(void)
{
	printk("gauge_bq27520: exit.\n");
	return i2c_del_driver(&gauge_bq27520_driver);
}
module_exit(gauge_bq27520_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS gauge bq27520 G4");
MODULE_AUTHOR("Piter Hsu <piter_hsu@asus.com>");
