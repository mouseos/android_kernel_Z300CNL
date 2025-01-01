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
#include <linux/usb/penwell_otg.h>

#include <linux/power/gauge_bq27520.h>
#include <linux/power/charger_smb358.h>

#include <linux/earlysuspend.h>
#include <linux/board_asustek.h>

// vvv piter test ctp vvv
#include <asm/intel-mid.h>
// ^^^ piter test ctp  ^^^


// vvvv piter test vvvv
#define ASUS_BATTERY_I2C_BUS 2
// ^^^ piter test


//#define GPIO_PIN_LOW_BATTERY_DETECT TEGRA_GPIO_PR4		test ctp
#define SMBUS_RETRY (0)
#define KELVIN_BASE 2730
#define INITIAL_CAPACITY_VALUE	50
#define INITIAL_TEMPERATURE_VALUE	(250+KELVIN_BASE)
#define PROTECT_TEMPERATURE_IN_CELSIUS_HIGH 600		// android shutdown temperature
#define PROTECT_TEMPERATURE_IN_CELSIUS_LOW (-200)
#define PROPERTY_ERROR_RESET_VALUE_CAPACITY	0
#define PROPERTY_ERROR_RESET_VALUE_TEMPERATURE 1200
#define BATTERY_POLLING_RATE 60
#define RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR	(5)
#define GAUGE_FEEDBACK_STATUS_DISCHARGING		0x0040
#define GAUGE_FEEDBACK_STATUS_FULLY_CHARGED	0x0020
#define GAUGE_FEEDBACK_STATUS_FULLY_DISCHARGED	0x0010
//#define GPIO_AC_OK TEGRA_GPIO_PV1			test ctp
#define _PROPERTY_AND_REG_SPEC(_psp, _addr, _min_value, _max_value) { \
	.psp = _psp, \
	.addr = _addr, \
	.min_value = _min_value, \
	.max_value = _max_value, \
}
#ifdef REMOVE_USB_POWER_SUPPLY
static bool can_power_supply_by_usb = false;
#else
static bool can_power_supply_by_usb = true;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static bool config_has_early_suspend = true;
#else
static bool config_has_early_suspend = false;
#endif

static bool asus_battery_driver_ready = false;

static bool pad_gaugeIC_firmware_is_updated = true;

static bool project_is_haydn = false;
static struct asus_battery_struct *asus_battery = NULL;
static struct workqueue_struct *asus_battery_workqueue = NULL;
char androidboot_mode[10];
EXPORT_SYMBOL(androidboot_mode);
static bool system_out_late_resume = true;

static project_id projectID;

static int asus_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
extern int cable_status_register_client(struct notifier_block *nb);
extern unsigned int query_cable_status(void);
extern int bq27520_get_battery_FCC(void);

static bool piter_test_bit = false;
static int piter_test_capacity = 30;

enum usb_connect_type {
        CONNECT_TYPE_NONE,
        CONNECT_TYPE_SDP,
        CONNECT_TYPE_DCP,
        CONNECT_TYPE_CDP,
};

enum projectID_num {
	PCBID_FE375CL = 0,
	PCBID_ME375CL,
};

enum which_battery_enum{
	dock_device_battery = 0,
	pad_device_battery,
};

enum asus_battery_cable_type {
	non_cable =0,
	usb_cable,
	unknow_cable,
	ac_cable,
	otg_cable,
};

static enum asus_battery_cable_type last_time_cable_type = non_cable;
static enum asus_battery_cable_type this_time_cable_type = non_cable;

struct gauge_ADC_alert_struct{
	int	GPIO_pin;
	unsigned int irq;
	struct wake_lock ADC_alert_wake_lock;
	struct delayed_work ADC_alert_work;
};

struct cable_type_struct{
	enum asus_battery_cable_type this_time_cable_type;
	enum asus_battery_cable_type last_time_cable_type;
	struct wake_lock cable_type_change_event_wake_lock;
};

struct AC_ok_struct{
	unsigned	GPIO_pin;
	unsigned int irq;
	struct wake_lock AC_ok_wake_lock;
};

struct asus_battery_struct{
	struct mutex			mutex_lock;
	struct i2c_client		*client;
	struct power_supply	ac;
	struct power_supply	usb;
	struct power_supply	battery;
	struct delayed_work	battery_status_polling_work;
	struct delayed_work	battery_status_reupdate_at_booting;
	struct delayed_work	AC_ok_work;
	struct delayed_work	charger_enable_AC_work;
	struct delayed_work	charger_enable_USB_work;
	struct gauge_ADC_alert_struct gauge_ADC_alert;
	struct cable_type_struct cable_type;
	struct AC_ok_struct AC_ok;
	struct early_suspend early_suspend;
	struct wake_lock charging_mode_wake_lock;
	int (*get_battery_charging_status_from_gauge)(void);
	int (*get_battery_voltage_from_gauge)(void);
	int (*get_battery_capacity_in_percent_from_gauge)(void);
	int (*get_battery_temperature_from_gauge)(void);
	int (*get_battery_current_from_gauge)(void);
	int (*get_battery_capacity_in_mAh_from_gauge)(void);
	int (*get_battery_time_to_empty_from_gauge)(void);
	int (*get_battery_energy_full_in_watt_design_from_gauge)(void);
	int (*get_battery_voltage_max_design_from_gauge)(void);
	int (*get_battery_voltage_min_design_from_gauge)(void);
	int (*charger_enable_AC_charging_and_set_AC_current_limit)(int AC_current_limit);
	int (*charger_enable_USB_charging)(void);
	int (*charger_disable_AC_and_USB_charging)(void);
	int (*charger_enable_otg)(void);
	int (*charger_disable_otg)(void);
	int (*charger_set_AC_current_limit)(int AC_current_limit);
	int (*charger_get_AICL_results)(void);
	int (*charger_JEITA_rule_for_system_run)(int battery_temperature, int battery_voltage_in_mV);
	int (*charger_JEITA_rule_for_sleeping_or_shutdown)(void);
	int (*factory_temp_charging_enable_or_disable)(bool charging_enable);
	int (*force_enable_or_disable_charging)(bool charging_enable);
	int capacity_in_percent_this_time;
	int capacity_in_percent_this_time_user_viewed;	// since we have modified the capacity, this is to store the capacity that user viewed
	int capacity_in_percent_last_time;
	int temperature_this_time;
	int temperature_last_time;
	int capacity_get_error_times;
	int temperature_get_error_times;
	bool factory_charging_limit_enable;
	bool chargerIC_is_work;
	bool AC_in;
	bool batteryID_is_correct;
};

static enum power_supply_property asus_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN,
};

static enum power_supply_property ac_and_usb_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static char *ac_and_usb_supply_list[] = {"battery",};

#define CONFIG_POWER_SUPPLY_BATTERY(ps_battery)					\
do{																	\
	ps_battery.name		= "battery";									\
	ps_battery.type		= POWER_SUPPLY_TYPE_BATTERY;				\
	ps_battery.properties	= asus_battery_properties;				\
	ps_battery.num_properties = ARRAY_SIZE(asus_battery_properties); \
	ps_battery.get_property	= asus_battery_get_property;					\
}while(0)

#define CONFIG_POWER_SUPPLY_AC(ps_ac)							\
do{																	\
	ps_ac.name		= "ac";												\
	ps_ac.type		= POWER_SUPPLY_TYPE_MAINS;							\
	ps_ac.supplied_to	= ac_and_usb_supply_list;											\
	ps_ac.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	ps_ac.properties = ac_and_usb_power_properties;							\
	ps_ac.num_properties = ARRAY_SIZE(ac_and_usb_power_properties);			\
	ps_ac.get_property	= power_get_property;								\
}while(0)

#define CONFIG_POWER_SUPPLY_USB(ps_USB)							\
do{																	\
	ps_USB.name		= "usb";												\
	ps_USB.type		= POWER_SUPPLY_TYPE_USB;							\
	ps_USB.supplied_to	= ac_and_usb_supply_list;											\
	ps_USB.num_supplicants = ARRAY_SIZE(ac_and_usb_supply_list);												\
	ps_USB.properties = ac_and_usb_power_properties;							\
	ps_USB.num_properties = ARRAY_SIZE(ac_and_usb_power_properties);			\
	ps_USB.get_property = power_get_property;								\
}while(0)

static enum asus_battery_cable_type get_this_time_cable_type(void)
{
	return this_time_cable_type;
}

static enum asus_battery_cable_type change_tegra_conntect_type_to_asus_battery_cable_type(
	unsigned long usb_cable_state)
{
	// may modify with the return type of usb
	switch (usb_cable_state) {
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			return non_cable;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			return usb_cable;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
	  		return ac_cable;
			break;
		case POWER_SUPPLY_CHARGER_TYPE_OTG:
		case POWER_SUPPLY_CHARGER_TYPE_B_DEVICE:
	  		return otg_cable;
		default:
			return non_cable;
			break;
	}
}

static int using_queue_work_to_communicate_charger_workaround(struct work_struct *work)
{
	// using queue work to communicat to charger is a workaround
	// or it may have a bug message: BUG: sleeping function called from invalid context at
	// rtmutex.c (happened in intel chip with charger connecting to PMIC)

	int re_try_count = 0;
	int ret =0;
	static char *ASUSEvt_cable_type_text[] = {"None", "USB", "UNKNOWN", "ASUS AC", "OTG"};

	ASUSEvtlog("[USB] set_chg_mode: %s\n", ASUSEvt_cable_type_text[this_time_cable_type]);

	do{
		if(re_try_count != 0){
			printk("asus_battery: using_queue_work_to_communicate_charger_workaround. re_try_count: %d\n", re_try_count);
			msleep(50);
		}

		if(this_time_cable_type== unknow_cable) {
			// reserve
		}else if (this_time_cable_type == usb_cable && can_power_supply_by_usb){
			ret = asus_battery->charger_enable_USB_charging();
			smb358_get_register_value(0x00);
			smb358_get_register_value(0x01);
			smb358_get_register_value(0x02);
			smb358_get_register_value(0x03);
			smb358_get_register_value(0x06);
			smb358_get_register_value(0x0A);
			smb358_get_register_value(0x30);
			smb358_get_register_value(0x31);
			power_supply_changed(&asus_battery->usb);
		}else if (this_time_cable_type == ac_cable){
			ret = asus_battery->charger_enable_AC_charging_and_set_AC_current_limit(1200);
			smb358_get_register_value(0x00);
			smb358_get_register_value(0x01);
			smb358_get_register_value(0x02);
			smb358_get_register_value(0x03);
			smb358_get_register_value(0x06);
			smb358_get_register_value(0x0A);
			smb358_get_register_value(0x30);
			smb358_get_register_value(0x31);
			power_supply_changed(&asus_battery->ac);
		}else if (this_time_cable_type == non_cable){
			if(last_time_cable_type == otg_cable){
				ret = asus_battery->charger_disable_otg();
				smb358_get_register_value(0x0A);
				smb358_get_register_value(0x31);
			}else{
				ret = asus_battery->charger_disable_AC_and_USB_charging();
				smb358_get_register_value(0x00);
				smb358_get_register_value(0x01);
				smb358_get_register_value(0x02);
				smb358_get_register_value(0x03);
				smb358_get_register_value(0x06);
				smb358_get_register_value(0x0A);
				smb358_get_register_value(0x30);
				smb358_get_register_value(0x31);
				power_supply_changed(&asus_battery->battery);
			}
		}else if(this_time_cable_type == otg_cable){
			ret = asus_battery->charger_enable_otg();
			smb358_get_register_value(0x09);
			smb358_get_register_value(0x0A);
			smb358_get_register_value(0x30);
		}

	}while(ret < 0 && ++re_try_count < 5);

	return 0;
}

int for_udc_call_me_back_if_cable_type_change(struct notifier_block *self,
	unsigned long cable_type, void *dev)
{
	static char *cable_type_text[] = {"non cable", "usb cable", "unknow cable", "ac cable", "otg cable"};

/*
	if(!asus_battery_driver_ready) {
		printk("asus_battery: battery driver not ready\n");
		return 1;
	}
*/
	last_time_cable_type = this_time_cable_type;
	this_time_cable_type = change_tegra_conntect_type_to_asus_battery_cable_type(cable_type);

	if(this_time_cable_type == last_time_cable_type){
		printk("asus battery: this time cable type (ret = %x) equal to last time cable type\n", cable_type);
		return 0;
	}

	wake_lock_timeout(&asus_battery->cable_type.cable_type_change_event_wake_lock, 5*HZ);

	cancel_delayed_work(&asus_battery->battery_status_polling_work);


	printk("=========================================================\n");
	printk("asus_battery:_callback from udc notify, usb_cable_state = %s, ret = %x\n", cable_type_text[this_time_cable_type], cable_type) ;
	printk("=========================================================\n");

	queue_work(asus_battery_workqueue, &asus_battery->charger_enable_USB_work);
/*
	if(this_time_cable_type== unknow_cable) {
		if (old_cable_status == ac_cable){
			asus_battery->charger_enable_AC_charging_and_set_AC_current_limit(1200);
			smb358_get_register_value(0x00);
			smb358_get_register_value(0x01);
			smb358_get_register_value(0x02);
			smb358_get_register_value(0x03);
			smb358_get_register_value(0x06);
			smb358_get_register_value(0x0A);
			smb358_get_register_value(0x30);
			smb358_get_register_value(0x31);
			power_supply_changed(&asus_battery->ac);
		}else if (old_cable_status == usb_cable && can_power_supply_by_usb){
			queue_work(asus_battery_workqueue, &asus_battery->charger_enable_USB_work);
			//asus_battery->charger_enable_USB_charging();
			smb358_get_register_value(0x00);
			smb358_get_register_value(0x01);
			smb358_get_register_value(0x02);
			smb358_get_register_value(0x03);
			smb358_get_register_value(0x06);
			smb358_get_register_value(0x0A);
			smb358_get_register_value(0x30);
			smb358_get_register_value(0x31);
			power_supply_changed(&asus_battery->usb);

		}
	}else if (this_time_cable_type == usb_cable && can_power_supply_by_usb){
		queue_work(asus_battery_workqueue, &asus_battery->charger_enable_USB_work);
		//asus_battery->charger_enable_USB_charging();
		smb358_get_register_value(0x00);
		smb358_get_register_value(0x01);
		smb358_get_register_value(0x02);
		smb358_get_register_value(0x03);
		smb358_get_register_value(0x06);
		smb358_get_register_value(0x0A);
		smb358_get_register_value(0x30);
		smb358_get_register_value(0x31);
		power_supply_changed(&asus_battery->usb);
	}else if (this_time_cable_type == ac_cable){
		asus_battery->charger_enable_AC_charging_and_set_AC_current_limit(1200);
		smb358_get_register_value(0x00);
		smb358_get_register_value(0x01);
		smb358_get_register_value(0x02);
		smb358_get_register_value(0x03);
		smb358_get_register_value(0x06);
		smb358_get_register_value(0x0A);
		smb358_get_register_value(0x30);
		smb358_get_register_value(0x31);
		power_supply_changed(&asus_battery->ac);
	}else if (this_time_cable_type == non_cable){
		asus_battery->charger_disable_AC_and_USB_charging();
		smb358_get_register_value(0x00);
		smb358_get_register_value(0x01);
		smb358_get_register_value(0x02);
		smb358_get_register_value(0x03);
		smb358_get_register_value(0x06);
		smb358_get_register_value(0x0A);
		smb358_get_register_value(0x30);
		smb358_get_register_value(0x31);
		power_supply_changed(&asus_battery->battery);
	}
*/
	queue_delayed_work(asus_battery_workqueue, &asus_battery->battery_status_polling_work, 2*HZ);

	return 0;
}
EXPORT_SYMBOL(for_udc_call_me_back_if_cable_type_change);

static struct notifier_block usb_cable_changed_callback_notifier = {
	.notifier_call = for_udc_call_me_back_if_cable_type_change,
};

static int get_battery_voltage_min_design(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_voltage_min_design_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value;

	return 0;
}

static int get_battery_voltage_max_design(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_voltage_max_design_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value;

	return 0;
}

static int get_battery_energy_full_in_watt_design(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_energy_full_in_watt_design_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value;

	return 0;
}

static int get_battery_capacity_in_mAh(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_capacity_in_mAh_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value;

	return 0;
}

static void modify_real_capacity_for_user_read(int *capacity_value)
{
	int temp_capacity;
	temp_capacity = ((*capacity_value >= 100) ? 100 : *capacity_value);

	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

	*capacity_value = temp_capacity;
}

static int get_battery_capacity_in_percent(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;
	bool error = false;
	bool gaugeIC_firmware_is_updated;

	if(which_battery == pad_device_battery){
		gaugeIC_firmware_is_updated = bq27520_get_if_pad_gaugeIC_firmware_is_updated();
		asus_battery->capacity_in_percent_last_time = asus_battery->capacity_in_percent_this_time;
	}

	if(gaugeIC_firmware_is_updated){
		do{
			if(gauge_feedback_error_times !=0){
				printk("asus_battery: it may get wrong capacity, times: %d, gauge_return_value: %d, last time capacity: %d\n",
					gauge_feedback_error_times, gauge_return_value, asus_battery->capacity_in_percent_last_time);

				msleep(100);
			}

			gauge_return_value = asus_battery->get_battery_capacity_in_percent_from_gauge();
		}while((gauge_return_value <= 5 || gauge_return_value > 100 ||
				abs(gauge_return_value - asus_battery->capacity_in_percent_last_time) > 5) &&
				++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

		printk("un-modified battery capacity: %d\n", gauge_return_value);

		if(gauge_return_value < 0 || gauge_return_value > 100)
			error = true;

		if(which_battery == pad_device_battery){
			if(error == true){
				printk("battery: read pad capacity fail times = %d\n", asus_battery->capacity_get_error_times);
				asus_battery->capacity_get_error_times++;
				if(asus_battery->capacity_get_error_times > 5){
					val->intval = PROPERTY_ERROR_RESET_VALUE_CAPACITY;
					asus_battery->capacity_in_percent_this_time = val->intval;
					return 0;
				}

				gauge_return_value = asus_battery->capacity_in_percent_last_time;
			}else{
				asus_battery->capacity_get_error_times = 0;

				if(which_battery == pad_device_battery)
					asus_battery->capacity_in_percent_this_time = gauge_return_value;

			}
		}

		modify_real_capacity_for_user_read(&gauge_return_value);
		asus_battery->capacity_in_percent_this_time_user_viewed = gauge_return_value;

		val->intval = gauge_return_value;

	}else{
		do{
			gauge_return_value = asus_battery->get_battery_voltage_from_gauge();
		}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

		asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

		if(gauge_return_value >= 4250)
			val->intval = 100;
		else if(gauge_return_value >= 4100 && gauge_return_value < 4250)
			val->intval = 70;
		else if(gauge_return_value >= 4000 && gauge_return_value < 4100)
			val->intval = 50;
		else if(gauge_return_value >= 3900 && gauge_return_value < 4000)
			val->intval = 30;
		else if(gauge_return_value >= 3800 && gauge_return_value < 3900)
			val->intval = 20;
		else if(gauge_return_value < 3800)
			val->intval = 6;

		if(val->intval > 100)
			val->intval = 100;

		if(val->intval < 6)
			val->intval = 6;

		if(which_battery == pad_device_battery)
			asus_battery->capacity_in_percent_this_time = val->intval;

	}


	#ifdef FACTORY_IMAGE
		if(asus_battery->factory_charging_limit_enable){
			if(piter_test_bit)
				val->intval = piter_test_capacity;

			if(projectID != PCBID_ME375CL){
	                    if(val->intval <= 40){
					printk("asus_battery: FE375CL factory charging limit enable: capacity < 40, start charging\n");
					asus_battery->factory_temp_charging_enable_or_disable(true);
				}else if(val->intval >= 60){
					printk("asus_battery: FE375CL factory charging limit enable: capacity > 60, stop charging\n");
					asus_battery->factory_temp_charging_enable_or_disable(false);
				}

			}else{
	                    if(val->intval <= 40){
					printk("asus_battery: ME375CL factory charging limit enable: capacity < 40, start charging\n");
					asus_battery->factory_temp_charging_enable_or_disable(true);
				}else if(val->intval >= 70){
					printk("asus_battery: ME375CL factory charging limit enable: capacity > 70, stop charging\n");
					asus_battery->factory_temp_charging_enable_or_disable(false);
				}
			}
		}else{
			printk("asus_battery: factory charging limit not enable.\n");
			asus_battery->factory_temp_charging_enable_or_disable(true);
		}

	#endif

	return 0;
}

static int get_battery_time_to_empty(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	int gauge_return_value ;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_time_to_empty_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value;

	return 0;
}

static int get_battery_current(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int gauge_return_value ;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_current_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = (int)(s16)gauge_return_value;

	return 0;
}

static int get_battery_voltage(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;

	do{
		gauge_return_value = asus_battery->get_battery_voltage_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		return -EINVAL;

	val->intval = gauge_return_value * 1000;

	return 0;
}

static int get_battery_temperature(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int gauge_return_value;
	int gauge_feedback_error_times = 0;
	bool error = false;

	asus_battery->temperature_last_time= asus_battery->temperature_this_time;

	do{
		gauge_return_value = asus_battery->get_battery_temperature_from_gauge();
	}while(gauge_return_value < 0 && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value < 0 ? false : true;

	if(gauge_return_value < 0)
		error = true;

	if(gauge_return_value > (PROTECT_TEMPERATURE_IN_CELSIUS_HIGH - 1 + KELVIN_BASE) ||
		gauge_return_value < PROTECT_TEMPERATURE_IN_CELSIUS_LOW + KELVIN_BASE )
		error = true;

	if(which_battery == pad_device_battery){
		if(error == true){
			printk("battery: read pad temperature fail times = %d, temp: %d\n", asus_battery->temperature_get_error_times, gauge_return_value);
			asus_battery->temperature_get_error_times++;
			if(asus_battery->temperature_get_error_times > 3){
				val->intval = PROPERTY_ERROR_RESET_VALUE_TEMPERATURE;
				asus_battery->temperature_this_time = val->intval + KELVIN_BASE;
				return 0;
			}

			gauge_return_value = asus_battery->temperature_last_time;
		}else{
			asus_battery->temperature_get_error_times = 0;

			if(which_battery == pad_device_battery)
				asus_battery->temperature_this_time = gauge_return_value;

		}
	}

	val->intval= gauge_return_value -KELVIN_BASE;

	return 0;
}

static int get_battery_health_or_present(union power_supply_propval *val,
	enum which_battery_enum which_battery)
{
	//val->intval = bq27520_get_battery_health_or_present();

	val->intval = 1;		// ctp test
	return 0;
}

static int get_battery_status(union power_supply_propval *val, enum which_battery_enum which_battery)
{
	int gauge_return_value_status;
	int gauge_feedback_error_times = 0;
	enum asus_battery_cable_type cable_type_now;

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;

	if(asus_battery->batteryID_is_correct == false)
		return 0;		// if battery is not for this project, than battery status is unknow

	do{
		gauge_return_value_status = asus_battery->get_battery_charging_status_from_gauge();
	}while((gauge_return_value_status < 0) && ++gauge_feedback_error_times < RETRY_TIMES_IF_GAUGE_FEEDBACK_ERROR);

	asus_battery->chargerIC_is_work = gauge_return_value_status < 0 ? false : true;

	if(gauge_return_value_status < 0){
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		return -EINVAL;
	}

	cable_type_now = get_this_time_cable_type();
	gauge_feedback_error_times = 0;

	printk("asus_battery: battery status: cable_now = %d, gauge_return_value_status = %x\n", cable_type_now, gauge_return_value_status);
	if(cable_type_now == ac_cable || cable_type_now == usb_cable){
		val->intval = asus_battery->capacity_in_percent_this_time_user_viewed == 100 ?
			POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
	}else if((gauge_return_value_status & GAUGE_FEEDBACK_STATUS_DISCHARGING) ||
		(gauge_return_value_status & GAUGE_FEEDBACK_STATUS_FULLY_DISCHARGED)){
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
	}else if(gauge_return_value_status & GAUGE_FEEDBACK_STATUS_FULLY_CHARGED){
		val->intval = POWER_SUPPLY_STATUS_FULL;
	}else{
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;		// avoid showing ? battery icon
	}

/*	// may think the factory
	if(gauge_return_value_status & GAUGE_FEEDBACK_STATUS_DISCHARGING ||
		gauge_return_value_status & GAUGE_FEEDBACK_STATUS_FULLY_DISCHARGED){
		val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	 }

	if(gauge_return_value_status & GAUGE_FEEDBACK_STATUS_FULLY_CHARGED){
		val->intval = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if(!(gauge_return_value_status & GAUGE_FEEDBACK_STATUS_DISCHARGING)){
		if(which_battery == pad_device_battery){
			if(cable_type_now == ac_cable)
				val->intval = asus_battery->capacity_in_percent_this_time == 100 ?
				POWER_SUPPLY_STATUS_FULL : POWER_SUPPLY_STATUS_CHARGING;
			else
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;

			return 0;
		}
	}

	val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
*/
	return 0;
}

static int asus_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	union power_supply_propval ps_temp_value;
	union power_supply_propval ps_temp_for_JEITA;
	enum which_battery_enum which_battery;
	static char *which_battery_text[] = {"Dock", "Pad"};
	int re_try_count = 0;
	int ret =0;

	if (!strcmp(psy->name, "dock_battery"))
		which_battery = dock_device_battery;
	else
		which_battery = pad_device_battery;


	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (get_battery_health_or_present(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			ps_temp_value.intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			if (get_battery_capacity_in_percent(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery capacity[%s] = %d (%)\n",
				which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_ENERGY_NOW:
			if(get_battery_capacity_in_mAh(&ps_temp_value, which_battery) < 0 )
				return -EINVAL;
			break;

		case POWER_SUPPLY_PROP_STATUS:
			if(get_battery_status(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			static char *charging_status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};
			printk("battery charging status[%s] = %s\n", which_battery_text[which_battery],
				charging_status_text[ps_temp_value.intval]);
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
			if(get_battery_voltage(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery voltage[%s] = %d (uV)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_CURRENT_NOW:
			if(get_battery_current(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery current[%s] = %d (mA)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_CURRENT_AVG:
			if(get_battery_current(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery current[%s] = %d (mA)\n", which_battery_text[which_battery], ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_TEMP:
			if(get_battery_temperature(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery temperature[%s] = %d (¢XC)\n", which_battery_text[which_battery],
				(ps_temp_value.intval / 10));

			if(!system_out_late_resume){
				printk("asus_battery: system didn't out of late resume, thus do not use system run JEITA rule\n");
				smb358_get_register_value(0x0B);
				smb358_get_register_value(0x07);
				smb358_get_register_value(0x06);
				break;
			}

			if(this_time_cable_type == ac_cable || this_time_cable_type == usb_cable){
				if(this_time_cable_type == ac_cable){
					if(asus_battery->charger_get_AICL_results() <= 500){
						printk("asus_battery: AICL_result <= 500, thus resetting\n");

						re_try_count = 0;
						do{
							ret = asus_battery->charger_enable_AC_charging_and_set_AC_current_limit(1200);

							if(re_try_count != 0){
								printk("asus_battery: charger_enable_AC_charging_and_set_AC_current_limit. re_try_count: %d\n", re_try_count);
								msleep(50);
							}
						}while(ret < 0 && ++re_try_count < 5);
					}
				}

				if(get_battery_voltage(&ps_temp_for_JEITA, which_battery) < 0)
					return -EINVAL;

				re_try_count = 0;
				do{
					ret = asus_battery->charger_JEITA_rule_for_system_run((ps_temp_value.intval)/10, (ps_temp_for_JEITA.intval)/1000);

					if(re_try_count != 0){
						printk("asus_battery: charger_JEITA_rule_for_system_run. re_try_count: %d\n", re_try_count);
						msleep(50);
					}
					//smb358_get_register_value(0x00);
					//smb358_get_register_value(0x01);
					//smb358_get_register_value(0x03);
					smb358_get_register_value(0x06);
				}while(ret < 0 && ++re_try_count < 5);
			}
			break;

		case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
			if(get_battery_time_to_empty(&ps_temp_value, which_battery) < 0)
				return -EINVAL;
			printk("battery time to empty[%s] = %d (min)\n", which_battery_text[which_battery],
				ps_temp_value.intval);
			break;

		case POWER_SUPPLY_PROP_ENERGY_FULL_DESIGN:
			ps_temp_value.intval = 8180;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
			ps_temp_value.intval = 4350;
			break;

		case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
			ps_temp_value.intval = 3400;
			break;

	       case POWER_SUPPLY_PROP_CURRENT_MAX:
			ps_temp_value.intval = 1800;;
			break;

		default:
			dev_err(&asus_battery->client->dev,
				"%s: INVALID property psp[%s] = %u\n", __func__, which_battery_text[which_battery], psp);
			return -EINVAL;
	}

	val->intval = ps_temp_value.intval;
	return 0;
}

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	union power_supply_propval dock_ps_value;
	enum which_battery_enum which_battery = dock_device_battery;

	get_battery_status(&dock_ps_value, which_battery);

	if(!project_is_haydn){
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  this_time_cable_type == ac_cable)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_USB && this_time_cable_type == usb_cable)
				val->intval =  1;
			else
				val->intval = 0;
			break;

		default:
			return -EINVAL;
		}

		return 0;
	}else{
		switch (psp) {
		case POWER_SUPPLY_PROP_ONLINE:
			if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  asus_battery->AC_in == true)
				val->intval =  1;
			else if (psy->type == POWER_SUPPLY_TYPE_USB && this_time_cable_type == usb_cable)
				val->intval =  1;
			else
				val->intval = 0;
			break;

		default:
			return -EINVAL;
		}

		return 0;
	}

}

static void battery_status_reupdate_at_booting_work_func(struct work_struct *work)
{
	// since the power consumption of T40 is too large, so re-update the status.
	// ex: when with dock, the net current may negative at booting, even the dock is charging
	// the pad, so re-update the battery status,
	// or it may show the wrong status at the first glance.

	printk("battery status update at booting\n");
	power_supply_changed(&asus_battery->battery);
}

static void battery_status_polling_work_func(struct work_struct *work)
{
       struct asus_battery_struct *battery_gauge = container_of(work, struct asus_battery_struct, battery_status_polling_work.work);
       static char *ASUSEvt_cable_type_text[] = {"None", "USB", "UNKNOWN", "ASUS AC", "OTG"};

	printk("battery polling work\n");

	if(!asus_battery_driver_ready)
		printk("battery driver not ready\n");

	power_supply_changed(&battery_gauge->battery);

	ASUSEvtlog("[BAT][Ser]report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, Temp:%dC, Cable:(%s)\n",
		asus_battery->get_battery_capacity_in_percent_from_gauge(),
		bq27520_get_battery_FCC(),
		asus_battery->get_battery_capacity_in_percent_from_gauge(),
		asus_battery->get_battery_voltage_from_gauge(),
		(int)(s16)asus_battery->get_battery_current_from_gauge(),
		(asus_battery->get_battery_temperature_from_gauge() - KELVIN_BASE)/10,
		ASUSEvt_cable_type_text[this_time_cable_type]);

	/* Schedule next polling */
	queue_delayed_work(asus_battery_workqueue, &battery_gauge->battery_status_polling_work, BATTERY_POLLING_RATE*HZ);
}

static void asus_battery_gauge_ADC_alert_work_func(struct work_struct *work)
{
	cancel_delayed_work(&asus_battery->battery_status_polling_work);
	printk("un-modified battery capacity: %d, voltage: %d, temperature: %d\n",
		asus_battery->get_battery_capacity_in_percent_from_gauge(),
		asus_battery->get_battery_voltage_from_gauge(),
		asus_battery->get_battery_temperature_from_gauge());

	msleep(500);

	queue_delayed_work(asus_battery_workqueue,&asus_battery->battery_status_polling_work, 0.1*HZ);

	enable_irq(asus_battery->gauge_ADC_alert.irq);
}

static irqreturn_t asus_battery_gauge_ADC_alert_detect_isr(int irq, void *dev_id)
{
	static char *ADC_alert_text[] = {"ADC alert", "default"};
	int gauge_ADC_alert_value = 0;

	disable_irq_nosync(asus_battery->gauge_ADC_alert.irq);

	gauge_ADC_alert_value = gpio_get_value(asus_battery->gauge_ADC_alert.GPIO_pin);
	gauge_ADC_alert_value = (gauge_ADC_alert_value > 0) ? 1 : 0;
	printk("gpio gauge ADC alert : %s\n", ADC_alert_text[gauge_ADC_alert_value]);

	wake_lock_timeout(&asus_battery->gauge_ADC_alert.ADC_alert_wake_lock, 10*HZ);
	queue_delayed_work(asus_battery_workqueue, &asus_battery->gauge_ADC_alert.ADC_alert_work, 0.1*HZ);
	return IRQ_HANDLED;
}

static int asus_battery_setup_gauge_ADC_alert_detect_irq(void)
{
	if(gpio_request(asus_battery->gauge_ADC_alert.GPIO_pin , "gauge_ADC_alert_detect") < 0){
		printk("asus_battery error: gpio gauge ADC alert request failed\n");
		return -EINVAL;
	}

	if(gpio_direction_input(asus_battery->gauge_ADC_alert.GPIO_pin) < 0){
		printk("asus_battery error: gpio gauge ADC alert unavailable for input\n");
		return -EINVAL;
	}

	asus_battery->gauge_ADC_alert.irq = gpio_to_irq(asus_battery->gauge_ADC_alert.GPIO_pin);
	printk("piter debug: gauge ADC_alert irq%d\n:", asus_battery->gauge_ADC_alert.irq);

	if(request_irq(asus_battery->gauge_ADC_alert.irq, asus_battery_gauge_ADC_alert_detect_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "gauge_ADC_alert", NULL) < 0){
		printk("asus_battery error: gpio gauge ADC alert irq request failed\n");
		return -EINVAL;
	}

	return 0;
}

static void asus_battery_AC_ok_work_func(struct work_struct *work)
{
	printk("asus battery: AC OK work function\n");

	cancel_delayed_work(&asus_battery->battery_status_polling_work);

	printk("========================================================\n");
	printk("         asus_battery: AC in status: %s\n", asus_battery->AC_in? "AC in":"without AC");
       printk("========================================================\n");

	if(asus_battery->AC_in)
		power_supply_changed(&asus_battery->ac);

	power_supply_changed(&asus_battery->battery);

	queue_delayed_work(asus_battery_workqueue,&asus_battery->battery_status_polling_work, 2*HZ);

	enable_irq(asus_battery->AC_ok.irq);
}

static irqreturn_t asus_battery_setup_AC_ok_isr(int irq, void *dev_id)
{
	disable_irq_nosync(asus_battery->AC_ok.irq);

	asus_battery->AC_in = !gpio_get_value(asus_battery->AC_ok.GPIO_pin);
	printk("AC in = %d, gpio value: %d\n", asus_battery->AC_in, gpio_get_value(asus_battery->AC_ok.GPIO_pin));

	wake_lock_timeout(&asus_battery->AC_ok.AC_ok_wake_lock, 10*HZ);
	queue_delayed_work(asus_battery_workqueue, &asus_battery->AC_ok_work, 1.8*HZ);
	return IRQ_HANDLED;
}

static int asus_battery_setup_AC_ok_irq(void)
{
	if(gpio_request(asus_battery->AC_ok.GPIO_pin, "battery_AC_ok") < 0){
		printk("asus_battery error: gpio AC_ok request failed\n");
		return -EINVAL;
	}

	if(gpio_direction_input(asus_battery->AC_ok.GPIO_pin) < 0){
		printk("asus_battery error: gpio AC_ok unavailable for input\n");
		return -EINVAL;
	}

	if(request_irq(asus_battery->AC_ok.irq, asus_battery_setup_AC_ok_isr,
		IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "battery_AC_ok", NULL) < 0){
		printk("asus_battery error: gpio AC_ok irq request failed\n");
		return -EINVAL;
	}

	return 0;
}

#ifdef FACTORY_IMAGE
static ssize_t sysfs_asus_battery_charging_limit_enable_store(struct device *dev, struct device_attribute *devattr, const char *buf, size_t count)
{
	long buf_type_in_num;

	if ( strict_strtol(buf, 0, &buf_type_in_num)){
		printk("gauge_bq27520: string to num fail\n");
		return -EINVAL;
	}

	if (buf_type_in_num == 0){
		asus_battery->factory_charging_limit_enable = false;
		printk("factory: charging limit not enable\n");
		power_supply_changed(&asus_battery->battery);
	}

	if (buf_type_in_num == 1){
		asus_battery->factory_charging_limit_enable = true;
		printk("factory: charging limit enable\n");
		power_supply_changed(&asus_battery->battery);
	}


	if (buf_type_in_num == 7){
		piter_test_bit = !	piter_test_bit;
		printk("piter_test_bit: %d\n", piter_test_bit);
	}

	if (buf_type_in_num == 8){
		piter_test_capacity = piter_test_capacity + 5;
		printk("piter_test_capacity: %d\n", piter_test_capacity);
	}
	if (buf_type_in_num == 9){
		piter_test_capacity = piter_test_capacity - 5;
		printk("piter_test_capacity: %d\n", piter_test_capacity);
	}

	if (buf_type_in_num == 30){
		piter_test_capacity = 30;
		printk("piter_test_capacity: %d\n", piter_test_capacity);
	}

	if (buf_type_in_num == 50){
		piter_test_capacity = 50;
		printk("piter_test_capacity: %d\n", piter_test_capacity);
	}

	if (buf_type_in_num == 70){
		piter_test_capacity = 70;
		printk("piter_test_capacity: %d\n", piter_test_capacity);
	}

	return count;
}

static ssize_t sysfs_asus_battery_charging_limit_enable_show(struct device *dev, struct device_attribute *devattr, char *buf)
{
	static char *charging_limit_enable_text[] = {"charging limit not enable", "charging limit enable"};
	return sprintf(buf, "factory charging limit enable: %s\n", charging_limit_enable_text[asus_battery->factory_charging_limit_enable]);
}
#endif

static ssize_t sysfs_asus_battery_battery_capacity_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	int capacity;

	capacity = asus_battery->get_battery_capacity_in_percent_from_gauge();

	return sprintf(buf, "%d\n", capacity);
}

static ssize_t sysfs_asus_battery_chargerIC_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", asus_battery->chargerIC_is_work);
}

static ssize_t sysfs_asus_battery_battery_current_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery = pad_device_battery;

	get_battery_current(&ps_temp_value, which_battery);

	return sprintf(buf, "%d\n", ps_temp_value.intval);
}

static ssize_t sysfs_asus_battery_charge_status_show(struct device *dev,
	struct device_attribute *devattr, char *buf)
{
	union power_supply_propval ps_temp_value;
	enum which_battery_enum which_battery = pad_device_battery;

	get_battery_status(&ps_temp_value, which_battery);

	switch(ps_temp_value.intval){
		case POWER_SUPPLY_STATUS_UNKNOWN:
			return sprintf(buf, "%d\n", 1);
			break;

		case POWER_SUPPLY_STATUS_CHARGING:
			return sprintf(buf, "%d\n", 2);
			break;

		case POWER_SUPPLY_STATUS_DISCHARGING:
			return sprintf(buf, "%d\n", 3);
			break;

		case POWER_SUPPLY_STATUS_NOT_CHARGING:
			return sprintf(buf, "%d\n", 4);
			break;

		case POWER_SUPPLY_STATUS_FULL:
			return sprintf(buf, "%d\n", 5);
			break;

		default:
			return sprintf(buf, "%d\n", 1);
			break;

	}

}

static ssize_t sysfs_asus_battery_get_battery_all_information_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int battery_charging_status, battery_voltage, battery_capacity, battery_temperature, battery_current;
	int battery_capacity_in_mAh, battery_time_to_empty, battery_heath_or_present;
	int battery_energy_full_design, battery_voltage_max, battery_voltage_min;
	char temp_char[64];

	battery_charging_status = 		asus_battery->get_battery_charging_status_from_gauge();
	battery_voltage = 				asus_battery->get_battery_voltage_from_gauge();
	battery_capacity = 			asus_battery->get_battery_capacity_in_percent_from_gauge();
	battery_temperature = 			asus_battery->get_battery_temperature_from_gauge();
	battery_current = 				asus_battery->get_battery_current_from_gauge();
	battery_capacity_in_mAh = 		asus_battery->get_battery_capacity_in_mAh_from_gauge();
	battery_time_to_empty = 		asus_battery->get_battery_time_to_empty_from_gauge();
	battery_energy_full_design = 	asus_battery->get_battery_energy_full_in_watt_design_from_gauge();
	battery_voltage_max = 			asus_battery->get_battery_voltage_max_design_from_gauge();
	battery_voltage_min = 			asus_battery->get_battery_voltage_min_design_from_gauge();

	sprintf(temp_char, "charging status = 0x%x\n", battery_charging_status);
	strcpy(buf, temp_char);
	sprintf(temp_char, "voltage = %d(mV)\n", battery_voltage);
	strcat(buf, temp_char);
	sprintf(temp_char, "capacity = %d(%)\n", battery_capacity);
	strcat(buf, temp_char);
	sprintf(temp_char, "temperature = %d\n", battery_temperature);
	strcat(buf, temp_char);
	sprintf(temp_char, "current = %d\n", battery_current);
	strcat(buf, temp_char);
	sprintf(temp_char, "capacity in mAh= %d\n", battery_capacity_in_mAh);
	strcat(buf, temp_char);
	sprintf(temp_char, "avg time to empty = %d\n", battery_time_to_empty);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery energy full design = %d(W)\n", battery_energy_full_design);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery voltage max = %d(mV)\n", battery_voltage_max);
	strcat(buf, temp_char);
	sprintf(temp_char, "battery voltage min = %d(mV)\n", battery_voltage_min);
	strcat(buf, temp_char);

	return strlen(buf);
}

static DEVICE_ATTR(asus_battery_get_battery_all_information, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_get_battery_all_information_show, NULL);
static DEVICE_ATTR(asus_battery_battery_capacity, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_battery_capacity_show, NULL);
static DEVICE_ATTR(asus_battery_chargerIC_status, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_chargerIC_status_show, NULL);
static DEVICE_ATTR(asus_battery_battery_current, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_battery_current_show, NULL);
static DEVICE_ATTR(asus_battery_charge_status, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_charge_status_show, NULL);
#ifdef FACTORY_IMAGE
static DEVICE_ATTR(asus_battery_charging_limit_enable, S_IWUSR | S_IRUGO,
	sysfs_asus_battery_charging_limit_enable_show,
	sysfs_asus_battery_charging_limit_enable_store);
#endif

static struct attribute *asus_battery_attributes[] = {
	&dev_attr_asus_battery_get_battery_all_information.attr,
	&dev_attr_asus_battery_battery_capacity.attr,
	&dev_attr_asus_battery_chargerIC_status.attr,
	&dev_attr_asus_battery_battery_current.attr,
	&dev_attr_asus_battery_charge_status.attr,
	#ifdef FACTORY_IMAGE
	&dev_attr_asus_battery_charging_limit_enable.attr,
	#endif
	NULL
};

static const struct attribute_group asus_battery_group = {
	.attrs = asus_battery_attributes,
};


void asus_battery_early_suspend(struct early_suspend *h)
{
	int re_try_count = 0;
	int ret =0;

	printk("asus_battery early suspend++\n");

	if(this_time_cable_type == ac_cable || this_time_cable_type == usb_cable){
		re_try_count = 0;

		do{
			ret = asus_battery->charger_JEITA_rule_for_sleeping_or_shutdown();

			if(re_try_count != 0){
				printk("asus_battery: charger_JEITA_rule_for_sleeping_or_shutdown. re_try_count: %d\n", re_try_count);
				msleep(50);
			}
		}while(ret < 0 && ++re_try_count < 5);

		smb358_get_register_value(0x0B);
		smb358_get_register_value(0x07);
		smb358_get_register_value(0x06);
	}

	system_out_late_resume = false;

	printk("asus_battery early suspend--\n");
}

void asus_battery_late_resume(struct early_suspend *h)
{
	printk("asus battery late_resume++\n");

	if(this_time_cable_type == ac_cable || this_time_cable_type == usb_cable){
		asus_battery->charger_JEITA_rule_for_system_run(25, 4000);	// arbitraily choose, since it will polling soon.
		smb358_get_register_value(0x03);
		smb358_get_register_value(0x06);
		smb358_get_register_value(0x07);
		smb358_get_register_value(0x0B);
	}

	system_out_late_resume = true;

	printk("asus battery late resume--\n");
}

static int asus_battery_probe(struct i2c_client *client,	const struct i2c_device_id *id)
{
	project_id project;

	printk("asus_battery probe++\n");

	asus_battery = devm_kzalloc(&client->dev, sizeof(*asus_battery), GFP_KERNEL);
	if(!asus_battery)
		return -ENOMEM;

	project = asustek_get_project_id();
	if (project == 0){
		projectID = PCBID_FE375CL;
		printk("asus_battery: project is FE375CL\n");
	}else if (project == 1){
		projectID = PCBID_ME375CL;
		printk("asus_battery: project is ME375CL\n");
	}else{
		printk("asus_battery: wrong projectID: %d\n", projectID);
	}

	mutex_init(&asus_battery->mutex_lock);
	asus_battery->client = client;
	asus_battery->capacity_in_percent_this_time = INITIAL_CAPACITY_VALUE;
	asus_battery->capacity_in_percent_this_time_user_viewed = INITIAL_CAPACITY_VALUE;
	asus_battery->capacity_in_percent_last_time = INITIAL_CAPACITY_VALUE;
	asus_battery->temperature_this_time = INITIAL_TEMPERATURE_VALUE;
	asus_battery->temperature_last_time = INITIAL_TEMPERATURE_VALUE;
	asus_battery->capacity_get_error_times = 0;
	asus_battery->temperature_get_error_times = 0;
	asus_battery->cable_type.this_time_cable_type = non_cable;
	asus_battery->cable_type.last_time_cable_type = non_cable;
	asus_battery->chargerIC_is_work = false;
	asus_battery->factory_charging_limit_enable = true;
	asus_battery->batteryID_is_correct = true;
	//asus_battery->AC_in = false;
	//asus_battery->AC_ok.GPIO_pin= GPIO_AC_OK;
	//asus_battery->AC_ok.irq= gpio_to_irq(asus_battery->AC_ok.GPIO_pin);



	asus_battery->gauge_ADC_alert.GPIO_pin = 							bq27520_get_gauge_ADC_alert_pin();
	asus_battery->get_battery_charging_status_from_gauge = 				bq27520_get_battery_charging_status;
	asus_battery->get_battery_voltage_from_gauge = 						bq27520_get_battery_voltage;
	asus_battery->get_battery_capacity_in_percent_from_gauge = 			bq27520_get_battery_capacity_in_percent;
	asus_battery->get_battery_temperature_from_gauge = 					bq27520_get_battery_temperature;
	asus_battery->get_battery_current_from_gauge = 						bq27520_get_battery_current;
	asus_battery->get_battery_capacity_in_mAh_from_gauge = 				bq27520_get_battery_capacity_in_mAh;
	asus_battery->get_battery_time_to_empty_from_gauge = 				bq27520_get_battery_time_to_empty;
	asus_battery->get_battery_energy_full_in_watt_design_from_gauge =		bq27520_get_battery_energy_full_in_watt_design;
	asus_battery->get_battery_voltage_max_design_from_gauge = 			bq27520_get_battery_voltage_max_design;
	asus_battery->get_battery_voltage_min_design_from_gauge = 			bq27520_get_battery_voltage_min_design;

	asus_battery->charger_enable_AC_charging_and_set_AC_current_limit =	smb358_AC_charging_enable_and_set_AC_current_limit;
	asus_battery->charger_enable_USB_charging =						smb358_USB_charging_enable;
	asus_battery->charger_disable_AC_and_USB_charging =				smb358_usb_and_AC_charging_disable;
	asus_battery->charger_enable_otg =								smb358_enable_otg;
	asus_battery->charger_disable_otg =								smb358_disable_otg;
	asus_battery->charger_set_AC_current_limit =						smb358_disable_AICL_set_AC_current_limit_enable_AICL;
	asus_battery->charger_get_AICL_results =							smb358_get_AICL_results_return_in_mA;
	asus_battery->charger_JEITA_rule_for_system_run =					smb358_JEITA_rule_for_system_run;
	asus_battery->charger_JEITA_rule_for_sleeping_or_shutdown =			smb358_JEITA_rule_for_sleeping_or_shutdown;

	asus_battery->factory_temp_charging_enable_or_disable =				smb358_temp_charging_enable_or_disable;
	asus_battery->force_enable_or_disable_charging = 					smb358_temp_charging_enable_or_disable;

	if(config_has_early_suspend){
		asus_battery->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
		asus_battery->early_suspend.suspend = asus_battery_early_suspend;
		asus_battery->early_suspend.resume = asus_battery_late_resume;
		register_early_suspend(&asus_battery->early_suspend);
	}

	pad_gaugeIC_firmware_is_updated = bq27520_get_if_pad_gaugeIC_firmware_is_updated();
	printk("gaugeIC is update: %d\n", pad_gaugeIC_firmware_is_updated);

	CONFIG_POWER_SUPPLY_BATTERY(asus_battery->battery);
	CONFIG_POWER_SUPPLY_AC(asus_battery->ac);
	CONFIG_POWER_SUPPLY_USB(asus_battery->usb);

	if(power_supply_register(&client->dev, &asus_battery->battery ) < 0)
		return -EINVAL;

	if(power_supply_register(&client->dev, &asus_battery->ac) < 0){
		power_supply_unregister(&asus_battery->battery);
		return -EINVAL;
	}

	if(power_supply_register(&client->dev, &asus_battery->usb) < 0){
		power_supply_unregister(&asus_battery->battery );
		power_supply_unregister(&asus_battery->ac);
		return -EINVAL;
	}

	asus_battery_workqueue = create_singlethread_workqueue("asus_battery_workqueue");

	INIT_DELAYED_WORK(&asus_battery->battery_status_polling_work, battery_status_polling_work_func);
	INIT_DELAYED_WORK(&asus_battery->gauge_ADC_alert.ADC_alert_work, asus_battery_gauge_ADC_alert_work_func);
	INIT_DELAYED_WORK(&asus_battery->battery_status_reupdate_at_booting,
		battery_status_reupdate_at_booting_work_func);
	INIT_DELAYED_WORK(&asus_battery->AC_ok_work, asus_battery_AC_ok_work_func);

	INIT_DELAYED_WORK(&asus_battery->charger_enable_AC_work, asus_battery->charger_enable_AC_charging_and_set_AC_current_limit);
	INIT_DELAYED_WORK(&asus_battery->charger_enable_USB_work, using_queue_work_to_communicate_charger_workaround);

	wake_lock_init(&asus_battery->gauge_ADC_alert.ADC_alert_wake_lock,
		WAKE_LOCK_SUSPEND, "gauge_ADC_alert_detection");
	wake_lock_init(&asus_battery->cable_type.cable_type_change_event_wake_lock,
		WAKE_LOCK_SUSPEND, "battery_cable_type_changed_event");
	wake_lock_init(&asus_battery->AC_ok.AC_ok_wake_lock, WAKE_LOCK_SUSPEND, "charger_AC_ok_wakelock");

	wake_lock_init(&asus_battery->charging_mode_wake_lock, WAKE_LOCK_SUSPEND, "charging_mode_wakelock");

	i2c_set_clientdata(client,asus_battery);


	printk("piter debug: gauge ADC_alert pin: %d\n", asus_battery->gauge_ADC_alert.GPIO_pin);
	if(asus_battery->gauge_ADC_alert.GPIO_pin >= 0)
		if(asus_battery_setup_gauge_ADC_alert_detect_irq() < 0)
			printk("asus_battery: setup gauge ADC alert fail\n");

//	if(asus_battery_setup_AC_ok_irq() < 0)
//		printk("asus_battery: setup AC ok fail\n");

//	asus_battery->AC_in = !gpio_get_value(asus_battery->AC_ok.GPIO_pin);

//	printk("asus_battery:  AC_in: %s\n", asus_battery->AC_in? "with AC ":"without AC");


	if(sysfs_create_group(&client->dev.kobj, &asus_battery_group))
		dev_err(&client->dev, "asus_battery_probe: unable to create the sysfs\n");

	cable_status_register_client(&usb_cable_changed_callback_notifier);


	asus_battery_driver_ready = true;

	if(bq27520_get_batteryID_calculate_from_pmic() == 0){
		printk("asus_battery: the batteryID is not in ASUS's battery, thus stop charging\n");
		asus_battery->force_enable_or_disable_charging(false);	// disable charging
		asus_battery->batteryID_is_correct = false;
	}


	for_udc_call_me_back_if_cable_type_change(NULL, query_cable_status(), &client->dev);


	queue_delayed_work(asus_battery_workqueue,
		&asus_battery->battery_status_reupdate_at_booting, 14*HZ);
	queue_delayed_work(asus_battery_workqueue, &asus_battery->battery_status_polling_work, 20*HZ);


	if (!strcmp(androidboot_mode, "charger")){
		// don't let device suspend when device in charging mode, or it will get fail power key status when device in suspend.
		// this will cause device can't reboot to android when we log press the power button.
		// without doing this, we have to short press the power key the wake device up from suspend,
		// then log press the power key to reboot the device to android.
		printk("asus_battery:: androidboot mode: charging mode, thus hold wake lock\n");

		wake_lock(&asus_battery->charging_mode_wake_lock);
	}

	printk("asus_battery probe--\n");
	return 0;

}

static int asus_battery_shutdown(struct i2c_client *client)
{
	int re_try_count = 0;
	int ret =0;

	printk("asus battery shutdown+\n");

	if(this_time_cable_type == ac_cable || this_time_cable_type == usb_cable){
		re_try_count = 0;

		do{
			ret = asus_battery->charger_JEITA_rule_for_sleeping_or_shutdown();

			if(re_try_count != 0){
				printk("asus_battery: shutdown: charger_JEITA_rule_for_sleeping_or_shutdown. re_try_count: %d\n", re_try_count);
				msleep(50);
			}
		}while(ret < 0 && ++re_try_count < 5);

		smb358_get_register_value(0x0B);
		smb358_get_register_value(0x07);
	}

	cancel_delayed_work_sync(&asus_battery->battery_status_polling_work);
	wake_lock_destroy(&asus_battery->gauge_ADC_alert.ADC_alert_wake_lock);
	wake_lock_destroy(&asus_battery->cable_type.cable_type_change_event_wake_lock);
	wake_lock_destroy(&asus_battery->AC_ok.AC_ok_wake_lock);

	if(config_has_early_suspend)
		unregister_early_suspend(&asus_battery->early_suspend);

	printk("asus battery shutdown--\n");
}

static int asus_battery_remove(struct i2c_client *client)
{
	power_supply_unregister(&asus_battery->battery);
	power_supply_unregister(&asus_battery->ac);
	power_supply_unregister(&asus_battery->usb);
	wake_lock_destroy(&asus_battery->gauge_ADC_alert.ADC_alert_wake_lock);
	wake_lock_destroy(&asus_battery->cable_type.cable_type_change_event_wake_lock);
	wake_lock_destroy(&asus_battery->AC_ok.AC_ok_wake_lock);

	if(config_has_early_suspend)
		unregister_early_suspend(&asus_battery->early_suspend);

	return 0;
}

#if defined (CONFIG_PM)
static int asus_battery_suspend(struct device *dev)
{
	printk("asus battery suspend++\n");

	cancel_delayed_work_sync(&asus_battery->battery_status_polling_work);
	flush_workqueue(asus_battery_workqueue);
	enable_irq_wake(asus_battery->gauge_ADC_alert.irq);
	enable_irq_wake(asus_battery->AC_ok.irq);

	printk("asus battery suspend--\n");
	return 0;
}

static int asus_battery_resume(struct device *dev)
{
	printk("asus battery resume++\n");

	cancel_delayed_work(&asus_battery->battery_status_polling_work);
	queue_delayed_work(asus_battery_workqueue, &asus_battery->battery_status_polling_work, 1.5*HZ);
	disable_irq_wake(asus_battery->gauge_ADC_alert.irq);
	disable_irq_wake(asus_battery->AC_ok.irq);

	printk("asus battery resume--\n");
	return 0;
}
#endif

static const struct dev_pm_ops asus_battery_pm_ops = {
	.suspend = asus_battery_suspend,
	.resume = asus_battery_resume,
};

static const struct i2c_device_id asus_battery_id[] = {
	{ "asus_battery", 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, asus_battery_id);

static struct i2c_driver asus_battery_driver = {
	.driver = {
		.name = "asus_battery",
		.owner = THIS_MODULE,
		#if defined (CONFIG_PM)
		.pm = &asus_battery_pm_ops,
		#endif
	},
	.probe        = asus_battery_probe,
	.remove      = asus_battery_remove,
	.shutdown	= asus_battery_shutdown,
	.id_table     = asus_battery_id,
};

static struct i2c_board_info i2c_asus_battery_board_info[] = {
	{
		I2C_BOARD_INFO("asus_battery", 0x07),
	},
};

static int __init asus_battery_init(void)
{
	int ret = 0;

	struct i2c_adapter *adapter = NULL;
	struct i2c_client *client   = NULL;

	printk("asus_battery: init++\n");

	adapter = i2c_get_adapter(ASUS_BATTERY_I2C_BUS);
	if(adapter == NULL){
		printk("asus battery error: can't get i2c adapter\n");
		return 0;
		//return -ENODEV;
	}

	client = i2c_new_device(adapter, i2c_asus_battery_board_info);
	if(client == NULL){
		printk("asus battery error: allocate i2c client failed\n");
		return 0;
		//return -ENOMEM;
	}

	i2c_put_adapter(adapter);

	printk("asus_battery: init--\n");

	ret = i2c_add_driver(&asus_battery_driver);
	if(ret != 0){
		printk("asus battery error: i2c add driver failed\n");
	}


	//return i2c_add_driver(&asus_battery_driver);

	return 0;
}
late_initcall(asus_battery_init);

static void __exit asus_battery_exit(void)
{
	printk("asus_battery: exit\n");
	return i2c_del_driver(&asus_battery_driver);
}
module_exit(asus_battery_exit);

int __init get_androidboot_mode(char *s)
{
	int n;

	n = snprintf(androidboot_mode, sizeof(androidboot_mode), "%s", s);
	androidboot_mode[n] = '\0';

	printk("asus_battery: piter test: androidboot_mode: %s", androidboot_mode);

	return 1;
}
__setup("androidboot.mode=", get_androidboot_mode);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS battery");
MODULE_AUTHOR("Piter Hsu <piter_hsu@asus.com>");
