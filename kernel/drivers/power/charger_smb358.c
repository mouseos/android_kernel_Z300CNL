#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>

#include <linux/power/charger_smb358.h>

/* Register definitions */
#define REG_CHARGE_CURRENT						0x00
#define REG_INPUT_CURRENT_LIMIT					0x01
#define REG_VARIOUS_FUNCTIONS						0x02
#define REG_FLOAT_VOLTAGE							0x03
#define REG_CHARGE_CTRL							0x04
#define REG_STATUS_TIMER_CTRL						0x05
#define REG_PIN_ENABLE_CTRL						0x06
#define REG_THERM_SYSTEM_CTRL_A					0x07
#define REG_SYSOK_USB3_SELECTION					0x08
#define REG_OTHER_CTRL_A							0x09
#define REG_OTG_TLIM_THERM_CTRL					0x0A
#define REG_HARD_SOFT_LIMIT_CELL_TEMP_MONITOR	0x0B
#define REG_FAULT_INTERRUPT						0x0C
#define REG_STATUS_INTERRUPT						0x0D
#define REG_I2C_BUS_SLAVE_ADDRESS					0x0E
#define REG_COMMAND_A								0x30
#define REG_COMMAND_B								0x31
#define REG_COMMAND_C								0x33
#define REG_INTERRUPT_A							0x35
#define REG_INTERRUPT_B							0x36
#define REG_INTERRUPT_C							0x37
#define REG_INTERRUPT_D							0x38
#define REG_INTERRUPT_E							0x39
#define REG_INTERRUPT_F							0x3A
#define REG_STATUS_A								0x3B
#define REG_STATUS_B								0x3C
#define REG_STATUS_C								0x3D
#define REG_STATUS_D								0x3E
#define REG_STATUS_E								0x3F

static struct charger_smb358_struct *charger_smb358 = NULL;

enum system_mode_enum{
	SYSTEM_RUN,
	SLEEPING_OR_SHUT_DOWN,
	OTG,
	SYSTEM_AT_BOOTING,
};

struct charger_smb358_struct {
	struct i2c_client	*client;
	struct mutex		mutex_lock;
	enum system_mode_enum system_mode;
	unsigned int curr_limit;
	bool force_charging_disable;
};

static int smb358_read(struct i2c_client *client, int reg)
{
	int return_value, i;
	int reread_times = 3;

	for(i = 0; i < reread_times; i ++){
		return_value = i2c_smbus_read_byte_data(client, reg);

		if(return_value >= 0)
			break;
	}

	if (return_value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, return_value);

	return return_value;
}

static int smb358_write(struct i2c_client *client, int reg, u8 value)
{
	int return_value, i;
	int rewrite_times = 3;

	for(i = 0; i < rewrite_times; i ++){
		return_value = i2c_smbus_write_byte_data(client, reg, value);

		if(return_value >= 0)
			break;
	}

	if (return_value < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, return_value);

	return return_value;
}

static int smb358_update_reg(struct i2c_client *client, int reg, u8 value)
{
	int return_value;

	return_value = smb358_read(client, reg);
	if (return_value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, return_value);
		return -EINVAL;
	}

	if(smb358_write(client, reg, return_value | value) < 0){
		dev_err(&client->dev, "%s(): Failed in writing register 0x%02x\n", __func__, reg);
		return -EINVAL;
	}

	return 0;
}

static int smb358_clear_reg(struct i2c_client *client, int reg, u8 value)
{
	int return_value;

	return_value = smb358_read(client, reg);
	if (return_value < 0) {
		dev_err(&client->dev, "%s: err %d\n", __func__, return_value);
		return -EINVAL;
	}

	if(smb358_write(client, reg, return_value & (~value)) < 0){
		dev_err(&client->dev, "%s(): Failed in writing register 0x%02x\n", __func__, reg);
		return -EINVAL;
	}

	return 0;
}

static int smb358_volatile_writes(struct i2c_client *client, bool writable)
{
	int ret;

	#define ALLOW_VOLATILE_WRITE	BIT(7)

	ret = smb358_read(client, REG_COMMAND_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= ALLOW_VOLATILE_WRITE;
	else
		ret &= ~ALLOW_VOLATILE_WRITE;

	return smb358_write(client, REG_COMMAND_A, ret);
}

static int smb358_register_volatile_write_enable(void)
{
	if(smb358_volatile_writes(charger_smb358->client, true) < 0)
		return -EINVAL;

	return 0;
}

static int smb358_register_volatile_write_disable(void)
{
	if(smb358_volatile_writes(charger_smb358->client, false) < 0)
		return -EINVAL;

	return 0;
}

static void smb358_clear_interrupts(struct i2c_client *client)
{
	uint8_t buf[6];

	// when the status bit is read, the corresponding signal is cleared
	if(i2c_smbus_read_i2c_block_data(client, REG_INTERRUPT_A, 6, buf) < 0)
		dev_err(&client->dev, "%s(): Failed in clearing interrupts\n",	__func__);
}

int smb358_temp_charging_enable_or_disable(bool charging_enable)
{
	struct i2c_client *client = charger_smb358->client;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	if(charging_enable){
		/* charging enable */
		if(smb358_update_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_PIN_ENABLE_CTRL);
			return -EFAULT;
		}

		charger_smb358->force_charging_disable = false;
	}else{
		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_PIN_ENABLE_CTRL);
			return -EFAULT;
		}

		charger_smb358->force_charging_disable = true;
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return 0;
}

int smb358_JEITA_rule_for_system_run(int battery_temperature, int battery_voltage_in_mV)
{
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	if(charger_smb358->force_charging_disable){
		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_PIN_ENABLE_CTRL);
			return -EFAULT;
		}

		/* Disable volatile writes to registers */
		if(smb358_register_volatile_write_disable() < 0){
			dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
			return -EFAULT;
		}

		printk("smb358: JEITA_rule_for_system_run: since force_charging disable so disable charging and return\n");

		return 0;
	}

	/* Set Hard Hot Limit = 72 Deg.C. */
	if(smb358_update_reg(client, REG_HARD_SOFT_LIMIT_CELL_TEMP_MONITOR, BIT(5)|BIT(4)) < 0){
		dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}

	/* Set Soft Hot Limit Behavior = No Response ,  Set Soft Cold temp limit = No Response */
	if(smb358_clear_reg(client, REG_THERM_SYSTEM_CTRL_A, BIT(0)|BIT(1)|BIT(2)|BIT(3)) < 0){
		dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}


	if(battery_temperature < 1.5){
		/* Vchg=4.34V */
		ret = smb358_read(client, REG_FLOAT_VOLTAGE);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0xc0) | 0x2A;

		ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
			return -EFAULT;
		}

		/* Set Fast Charge Current = 600 mA */
		ret = smb358_read(client, REG_CHARGE_CURRENT);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x1f) | 0x40;

		ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		printk("smb358: temperature < 0, stop charging\n");
	}else if(battery_temperature >= 1.5 && battery_temperature < 10){
		/* Vchg=4.34V */
		ret = smb358_read(client, REG_FLOAT_VOLTAGE);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0xc0) | 0x2A;

		ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		/* charging enable */
		if(smb358_update_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
			return -EFAULT;
		}

		/* Set Fast Charge Current = 600 mA */
		ret = smb358_read(client, REG_CHARGE_CURRENT);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x1f) | 0x40;

		ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}
	}else if(battery_temperature >= 10 && battery_temperature < 50){
		/* Vchg=4.34V */
		ret = smb358_read(client, REG_FLOAT_VOLTAGE);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0xc0) | 0x2A;

		ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		/* charging enable */
		if(smb358_update_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
			return -EFAULT;
		}

		/* Set Fast Charge current=1800mA */
		ret = smb358_read(client, REG_CHARGE_CURRENT);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x1f) | 0xc0;

		ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}
	}else if(battery_temperature >= 50 && battery_temperature < 55){
		/* get Vchg */
		ret = smb358_read(client, REG_FLOAT_VOLTAGE);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = ret & 0x3F;		// only get Vchg

		if((ret == 0x2A) && battery_voltage_in_mV >= 4100){
			/* Vchg=4.34V */
			ret = smb358_read(client, REG_FLOAT_VOLTAGE);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			ret = (ret & 0xc0) | 0x2A;

			ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			/* charging disable */
			if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
				dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
				return -EFAULT;
			}

			/* Set Fast Charge current=1800mA */
			ret = smb358_read(client, REG_CHARGE_CURRENT);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			ret = (ret & 0x1f) | 0xc0;

			ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			printk("smb358: 50 <= Vtemp < 55 && Vchg == 4.34V && VBAT >= 4.1V, stop charging\n");
		}else{
			/* Vchg=4.1V */
			ret = smb358_read(client, REG_FLOAT_VOLTAGE);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			ret = (ret & 0xc0) | 0x1E;

			ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			/* charging enable */
			if(smb358_update_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
				dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
				return -EFAULT;
			}

			/* Set Fast Charge current=1800mA */
			ret = smb358_read(client, REG_CHARGE_CURRENT);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}

			ret = (ret & 0x1f) | 0xc0;

			ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
			if(ret < 0){
				dev_err(&client->dev, "%s: err %d\n", __func__, ret);
				return -EFAULT;
			}
		}

	}else if(battery_temperature >= 55){
		/* Vchg=4.34V */
		ret = smb358_read(client, REG_FLOAT_VOLTAGE);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0xc0) | 0x2A;

		ret = smb358_write(client, REG_FLOAT_VOLTAGE, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
			return -EFAULT;
		}

		/* Set Fast Charge current=1800mA */
		ret = smb358_read(client, REG_CHARGE_CURRENT);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x1f) | 0xc0;

		ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		printk("smb358: temperature >= 55, stop charging\n");
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return 0;
}

int smb358_JEITA_rule_for_sleeping_or_shutdown(void)
{
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	//* SOC Do Not Control JEITA Function & Charger Control JEITA Protection *//


	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	/* Set Hard Hot Limit = 53 Deg.C. */
	ret = smb358_read(client, REG_HARD_SOFT_LIMIT_CELL_TEMP_MONITOR);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0xcf) | 0x00;

	ret = smb358_write(client, REG_HARD_SOFT_LIMIT_CELL_TEMP_MONITOR, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}


	/* Set Soft Hot Limit Behavior = Float Voltage Compensation
	     Set Soft Cold temp limit = Charge Current Compensation */
	ret = smb358_read(client, REG_THERM_SYSTEM_CTRL_A);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0xf0) | 0x06;

	ret = smb358_write(client, REG_THERM_SYSTEM_CTRL_A, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	/* charging enable */
	if(smb358_update_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
		dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}


	if(charger_smb358->force_charging_disable){
		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_PIN_ENABLE_CTRL);
			return -EFAULT;
		}
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

//	charger_smb358->system_mode = SLEEPING_OR_SHUT_DOWN;
	return 0;
}

static int __smb358_set_current_limit(struct i2c_client *client, int current_limit)
{
	int ret = 0, set_value = 0;

	if(current_limit < 500)
		set_value = 0x00;		// CURRENT_LIMIT_300
	else if(current_limit >= 500 && current_limit < 700)
		set_value = 0x10;		// CURRENT_LIMIT_500
	else if(current_limit >= 700 && current_limit < 1000)
		set_value = 0x20;		// CURRENT_LIMIT_700
	else if(current_limit >= 1000 && current_limit < 1200)
		set_value = 0x30;		// CURRENT_LIMIT_1000
	else if(current_limit >= 1200 && current_limit < 1500)
		set_value = 0x40;		// CURRENT_LIMIT_1200
	else if(current_limit >= 1500 && current_limit < 1800)
		set_value = 0x50;		// CURRENT_LIMIT_1500
	else if(current_limit >= 1800 && current_limit < 2000)
		set_value = 0x60;		// CURRENT_LIMIT_1800
	else if(current_limit >= 2000)
		set_value = 0x70;		// CURRENT_LIMIT_2000

	ret = smb358_read(client, REG_INPUT_CURRENT_LIMIT);

	ret = (ret & 0x0f) | set_value;

       ret = smb358_write(client, REG_INPUT_CURRENT_LIMIT, ret);
       if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
       }

	return ret;
}

int smb358_disable_AICL_set_AC_current_limit_enable_AICL(int AC_current_limit)
{
	struct i2c_client *client = charger_smb358->client;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	/* Disable AICL */
	if(smb358_clear_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
	      	dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}

	/* Set I_USB_IN=current_limit mA */
	__smb358_set_current_limit(client,  AC_current_limit);

	/* enable AICL */
	if(smb358_update_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
		dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return 0;
}

int smb358_get_AICL_results_return_in_mA(void)
{
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	/* read AICL results */
	ret = smb358_read(client, REG_STATUS_E);
       if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
       }

	ret = ret & 0x0f;

	if(ret == 0x00)
		return 300;
	else if(ret == 0x01)
		return 500;
	else if(ret == 0x02)
		return 700;
	else if(ret == 0x03)
		return 1000;
	else if(ret == 0x04)
		return 1200;
	else if(ret == 0x05)
		return 1300;
	else if(ret == 0x06)
		return 1800;
	else if(ret == 0x07)
		return 2000;
	else if(ret >= 0x08 && ret <= 0x0f)
		return 2000;
	else
		return -EFAULT;
}

static int smb358_configure_otg(bool enable)
{
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	if(enable){
		/* Disable OTG */
		if(smb358_clear_reg(client, REG_COMMAND_A, BIT(4)) < 0){
	       	dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_COMMAND_A);
			return -EFAULT;
		}

		/* Change "OTG output current limit" to 250mA*/
		if(smb358_clear_reg(client, REG_OTG_TLIM_THERM_CTRL, BIT(2)|BIT(3)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_OTG_TLIM_THERM_CTRL);
			return -EFAULT;
		}

		/* Enable OTG Function */
		ret = smb358_read(client, REG_OTHER_CTRL_A);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x3f) | 0x40;

		ret = smb358_write(client, REG_OTHER_CTRL_A, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}


		/* Set OTG current limit = 500mA */
		ret = smb358_read(client, REG_OTG_TLIM_THERM_CTRL);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0xf3) | 0x04;

		ret = smb358_write(client, REG_OTG_TLIM_THERM_CTRL, ret);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

	}else{
		// do nothing
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return ret;
}

int smb358_enable_otg(void)
{
	return smb358_configure_otg(true);
}

int smb358_disable_otg(void)
{
	return smb358_configure_otg(false);
}

static int __smb358_ACOK_charging_presetting(struct i2c_client *client)
{
	int ret = 0;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in enable volatile writes to registers\n", __func__);
		return -EFAULT;
	}

	/* Set Fast Charge current=1800mA */
	ret = smb358_read(client, REG_CHARGE_CURRENT);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0x1f) | 0xc0;

	ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	/* Set Termination current = 200 mA */
	ret = smb358_read(client, REG_CHARGE_CURRENT);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0xf8) | 0x07;

	ret = smb358_write(client, REG_CHARGE_CURRENT, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	/* Set cold soft limit current=600mA */
	ret = smb358_read(client, REG_OTG_TLIM_THERM_CTRL);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0x3f) | 0x80;

	ret = smb358_write(client, REG_OTG_TLIM_THERM_CTRL, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	/* Recharge Voltage = Vflt-100mV */
	ret = smb358_read(client, REG_INPUT_CURRENT_LIMIT);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	ret = (ret & 0xf3) | 0x04;

	ret = smb358_write(client, REG_INPUT_CURRENT_LIMIT, ret);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	/*disable volatile writes to registers*/
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in disable volatile writes to registers\n", __func__);
		return -EFAULT;
	}

	return ret;
}

static int smb358_usb_or_AC_charging_enable(bool is_AC, int current_limit)
{
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	if(__smb358_ACOK_charging_presetting(client) < 0){
		dev_err(&client->dev, "%s error in __smb358_ACOK_charging_presetting.\n", __func__);
		return -EFAULT;
	}

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in smb358_register_volatile_write_enable\n", __func__);
		return -EFAULT;
	}

	if(is_AC){
		/* Check if IUSB_IN (01h[7:4]) < 1200mA  */
		ret = smb358_read(client, REG_INPUT_CURRENT_LIMIT);
		if(ret < 0){
			dev_err(&client->dev, "%s: err %d\n", __func__, ret);
			return -EFAULT;
		}

		ret = (ret & 0x70);

		if(ret < 0x40){
			/* Disable AICL */
			if(smb358_clear_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
				dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
				return -EFAULT;
			}

			/* Set I_USB_IN=current_limit mA */
			if(__smb358_set_current_limit(client, current_limit) < 0){
				dev_err(&client->dev, "%s error in __smb358_set_current_limit.\n", __func__);
				return -EFAULT;
			}

			/* enable AICL */
			if(smb358_update_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
				dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
				return -EFAULT;
			}

			printk("smb358: Iusb_in ret < 40\n", ret);
		}else{
			if(smb358_get_AICL_results_return_in_mA() <= 500){
				printk("smb358: AICL <= 500\n", ret);

				/* Disable AICL */
				if(smb358_clear_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
					dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
					return -EFAULT;
				}

				/* Set I_USB_IN=current_limit mA */
				if(__smb358_set_current_limit(client, current_limit) < 0){
					dev_err(&client->dev, "%s error in __smb358_set_current_limit.\n", __func__);
					return -EFAULT;
				}

				/* enable AICL */
				if(smb358_update_reg(client, REG_VARIOUS_FUNCTIONS, BIT(4)) < 0){
					dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
					return -EFAULT;
				}
			}
		}
	}else{	// usb
		// charger team said don't have to do anything, since there are some default parameters presetting to charger IC.
		// but later on, we found when booting if NVM override with 1200mA then we have to override again.

		/* Set I_USB_IN=current_limit mA */
		if(__smb358_set_current_limit(client, current_limit) < 0){
			dev_err(&client->dev, "%s error in __smb358_set_current_limit.\n", __func__);
			return -EFAULT;
		}

	}

	if(charger_smb358->force_charging_disable){
		/* charging disable */
		if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
			dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_PIN_ENABLE_CTRL);
			return -EFAULT;
		}
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return ret;
}

int smb358_AC_charging_enable_and_set_AC_current_limit(int AC_current_limit)
{
	return smb358_usb_or_AC_charging_enable(true, AC_current_limit);
}

int smb358_USB_charging_enable(void)
{
	return smb358_usb_or_AC_charging_enable(false, 500);
}

int smb358_usb_and_AC_charging_disable(void)
{
	struct i2c_client *client = charger_smb358->client;

	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	/* charging disable */
	if(smb358_clear_reg(client, REG_PIN_ENABLE_CTRL, BIT(6)|BIT(5)) < 0){
		dev_err(&client->dev, "%s: Failed in writing register 0x%02x\n", __func__, REG_VARIOUS_FUNCTIONS);
		return -EFAULT;
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	return 0;
}

int smb358_get_register_value(int reg)
{
	int ret;
	struct i2c_client *client = charger_smb358->client;

	ret = smb358_read(client, reg);

	printk("smb358: Reg[%02x]=0x%02x\n", reg, ret);

	return ret;
}

static ssize_t sysfs_charger_smb358_get_AICL_in_mA_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int AICL_value_in_mA = 0;

	AICL_value_in_mA = smb358_get_AICL_results_return_in_mA();

	return sprintf(buf, "%d\n", AICL_value_in_mA);
}

static ssize_t sysfs_charger_smb358_I2C_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	// 0: i2c communicate fail
	// 1: i2c communicate success
	int i2c_status = 0;
	int ret = 0;
	struct i2c_client *client = charger_smb358->client;

	i2c_status = smb358_read(client, REG_CHARGE_CURRENT);

	ret = (i2c_status < 0) ? 0 : 1;

	return sprintf(buf, "%d\n", ret);
}

static ssize_t sysfs_charger_smb358_reg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = charger_smb358->client;
	char tmp_buf[64];
	int i,ret;

       sprintf(tmp_buf, "SMB358 Configuration Registers Show\n"
						"==================\n");
       strcpy(buf, tmp_buf);

	for (i = 0; i <= 14; i++) {
		ret = smb358_read(client, REG_CHARGE_CURRENT+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", REG_CHARGE_CURRENT+i,ret);
		strcat(buf, tmp_buf);
	}

	for (i = 0; i <= 1; i++) {
		ret = smb358_read(client, REG_COMMAND_A+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", REG_COMMAND_A+i,ret);
		strcat(buf, tmp_buf);
	}

	for (i = 0; i <= 10; i++) {
		ret = smb358_read(client, REG_INTERRUPT_A+i);
		sprintf(tmp_buf, "Reg%02xh:\t0x%02x\n", REG_INTERRUPT_A+i,ret);
		strcat(buf, tmp_buf);
	}

	return strlen(buf);
}

static ssize_t sysfs_charger_smb358_set_reg_value_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	long buf_type_in_num;
	u8 reg, reg_value;
	int ret = 0;
	int temp;
	struct i2c_client *client = charger_smb358->client;

	/* e.g: echo 0x1234 means set the data 0x34 to register 12h */
	if ( strict_strtol(buf, 0, &buf_type_in_num)){
		printk("charger smb358: string to num fail\n");
		return -EINVAL;
	}

	// vvvv for test vvvv
	if(buf_type_in_num == 1){
		smb358_JEITA_rule_for_system_run(-1, 4000);
		return count;
	}else if(buf_type_in_num == 2){
		smb358_JEITA_rule_for_system_run(5, 4000);
		return count;
	}else if(buf_type_in_num == 3){
		smb358_JEITA_rule_for_system_run(35, 4000);
		return count;
	}else if(buf_type_in_num == 4){
		smb358_JEITA_rule_for_system_run(52, 4000);
		return count;
	}else if(buf_type_in_num == 5){
		smb358_JEITA_rule_for_system_run(52, 4200);
		return count;
	}else if(buf_type_in_num == 6){
		smb358_JEITA_rule_for_system_run(55, 4000);
		return count;
	}else if(buf_type_in_num == 7){
		return count;
	}else if(buf_type_in_num == 8){
		return count;
	}else if(buf_type_in_num == 9){
		return count;
	}
	// ^^^^ for test ^^^^

	reg = (u8)(buf_type_in_num >> 8);
	reg_value = (u8)(buf_type_in_num & 0xff);


	/* before write reg and reg value */
	ret = smb358_read(client, reg);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	printk("before write: reg = 0x%02x, reg_value = 0x%02x\n", reg, ret);


	printk("write command: reg = 0x%02x, reg_value = 0x%02x\n", reg, reg_value);


	/*Enable volatile writes to registers*/
	if(smb358_register_volatile_write_enable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}

	/* write */
	if(smb358_write(client, reg, reg_value) < 0){
		dev_err(&client->dev, "%s(): Failed in writing register 0x%02x\n", __func__, REG_COMMAND_A);
		return -EFAULT;
	}

	/* Disable volatile writes to registers */
	if(smb358_register_volatile_write_disable() < 0){
		dev_err(&client->dev, "%s error in configuring otg..\n", __func__);
		return -EFAULT;
	}


	/* read */
	ret = smb358_read(client, reg);
	if(ret < 0){
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
		return -EFAULT;
	}

	printk("after write: reg = 0x%02x, reg_value = 0x%02x\n", reg, ret);


	return count;
}

static ssize_t sysfs_charger_smb358_set_reg_value_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "piter\n");
}

static DEVICE_ATTR(charger_smb358_get_AICL_in_mA, S_IWUSR | S_IRUGO, sysfs_charger_smb358_get_AICL_in_mA_show, NULL);
static DEVICE_ATTR(charger_smb358_I2C_status, S_IWUSR | S_IRUGO, sysfs_charger_smb358_I2C_status_show, NULL);
static DEVICE_ATTR(charger_smb358_reg_status, S_IWUSR | S_IRUGO, sysfs_charger_smb358_reg_show, NULL);
static DEVICE_ATTR(charger_smb358_set_reg_value, S_IWUSR | S_IRUGO,
	sysfs_charger_smb358_set_reg_value_show,
	sysfs_charger_smb358_set_reg_value_store);

static struct attribute *smb358_attributes[] = {
	&dev_attr_charger_smb358_get_AICL_in_mA.attr,
	&dev_attr_charger_smb358_I2C_status.attr,
	&dev_attr_charger_smb358_reg_status.attr,
	&dev_attr_charger_smb358_set_reg_value.attr,
	NULL
};

static const struct attribute_group charger_smb358_group = {
	.attrs = smb358_attributes,
};


static int charger_smb358_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	printk("charger smb358 probe++\n");

//	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
//		printk("piter debug1 smb358\n");
//		return -EIO;
//	}

	charger_smb358 = kzalloc(sizeof(*charger_smb358), GFP_KERNEL);
	if (!charger_smb358){
		printk("smb358 error: probe, ENOMEM\n");
		return -ENOMEM;
	}
	charger_smb358->client = client;
	charger_smb358->system_mode = SYSTEM_AT_BOOTING;
	charger_smb358->force_charging_disable = false;

	mutex_init(&charger_smb358->mutex_lock);

	i2c_set_clientdata(client, charger_smb358);

	if(sysfs_create_group(&client->dev.kobj, &charger_smb358_group))
		dev_err(&client->dev, "charger smb358 probe: unable to create the sysfs\n");


	printk("charger smb358 probe--\n");
	return 0;
}

static int charger_smb358_remove(struct i2c_client *client)
{
	kfree(charger_smb358);
	return 0;
}

static void charger_smb358_shutdown(struct i2c_client *client)
{
	printk("charger smb358 shutdown++\n");
	printk("charger smb358 shutdown--\n");
	return 0;
}

static int charger_smb358_suspend(struct device *dev)
{
	printk("charger smb358 suspend++\n");
	printk("charger smb358 suspend--\n");
	return 0;
}

static int charger_smb358_resume(struct device *dev)
{
	printk("charger smb358 resume++\n");
	printk("charger smb358 resume--\n");
	return 0;
}

static const struct dev_pm_ops charger_smb358_pm_ops = {
        .suspend = charger_smb358_suspend,
        .resume = charger_smb358_resume,
};

static const struct i2c_device_id charger_smb358_id[] = {
        {"smb347_charger", 0},
        {},
};

MODULE_DEVICE_TABLE(i2c, charger_smb358_id);

static struct i2c_driver charger_smb358_driver = {
        .driver = {
                .name = "smb347_charger",
                .owner = THIS_MODULE,
                #if defined (CONFIG_PM)
                .pm = &charger_smb358_pm_ops,
                #endif
        },
        .probe		= charger_smb358_probe,
        .remove		= charger_smb358_remove,
        .shutdown	= charger_smb358_shutdown,
        .id_table	= charger_smb358_id,
};

static int __init smb358_init(void)
{
	printk("charger smb358: init\n");
	return i2c_add_driver(&charger_smb358_driver);
}
module_init(smb358_init);

static void __exit smb358_exit(void)
{
	printk("charger smb358: exit\n");
	i2c_del_driver(&charger_smb358_driver);
}
module_exit(smb358_exit);

MODULE_AUTHOR("Piter Hsu <piter_hsu@asus.com>");
MODULE_DESCRIPTION("Charger smb358");
MODULE_LICENSE("GPL");
