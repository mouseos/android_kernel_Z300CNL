#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>

#define LP8557I_I2C_ADDR 0x2C
#define LP8557I_I2C_ADAPTER 2

static struct i2c_client *lp8557i_client;

static u8 value_0x11;
static u8 value_0x12;
static u8 value_0x13;
static u8 value_0x15;

int lp8557i_enable_backlight()
{
	int err;
	pr_info("%s\n", __func__);

	if(!lp8557i_client) {
		pr_err("no lp8557i_client\n");
		return -1;
	}
	//initial setting
	err = i2c_smbus_write_byte_data(lp8557i_client, 0x10, 0x84);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	//Full-scale current: 23mA
	err = i2c_smbus_write_byte_data(lp8557i_client, 0x11, value_0x11);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x12, value_0x12);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x13, value_0x13);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x15, value_0x15);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	/*++ set OVP to 22V ++*/
	err = i2c_smbus_write_byte_data(lp8557i_client, 0x7F, 0x21);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x7A, 0x88);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x16, 0x60);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}
	/*-- set OVP to 22V --*/

	/*Enable Channel 1~4*/
	err = i2c_smbus_write_byte_data(lp8557i_client, 0x14, 0x8F);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	//enable backlight
	//enable SREN and SSEN for EMI reduction
	err = i2c_smbus_write_byte_data(lp8557i_client, 0x00, 0x07);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	return 0;
}

int lp8557i_disable_backlight()
{
	int err;
	pr_info("%s\n", __func__);

	if(!lp8557i_client) {
		pr_err("no lp8557i_client\n");
		return -1;
	}

	err = i2c_smbus_write_byte_data(lp8557i_client, 0x00, 0x00);
	if (err < 0) {
		dev_err(&lp8557i_client->dev,
				"%s:i2c transfer FAIL: err=%d\n", __func__, err);
		return -1;
	}

	return 0;
}

static int lp8557i_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	pr_info("%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
		pr_err("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	lp8557i_client = client;
	return 0;
}

static int lp8557i_i2c_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id lp8557i_id[] = {
	{"lp8557i", 0},
	{}
};
/*MODULE_DEVICE_TABLE(i2c, lp8557i_id);*/

static struct i2c_driver lp8557i_i2c_driver = {
	.driver = {
		.name = "lp8557i",
	},
	.id_table = lp8557i_id,
	.probe = lp8557i_i2c_probe,
	.remove = lp8557i_i2c_remove,
};
static lp8557i_create_i2c_device(void)
{
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	struct i2c_board_info info = {
		.type = "lp8557i",
		.addr = LP8557I_I2C_ADDR,
	};

	pr_info("%s ++\n", __func__);

	adapter = i2c_get_adapter(LP8557I_I2C_ADAPTER);
	if (!adapter) {
		pr_err("%s: i2c_get_adapter(%d) failed\n", __func__,
				LP8557I_I2C_ADAPTER);
		return -EINVAL;
	}

	client = i2c_new_device(adapter, &info);
	if (!client) {
		pr_err("%s: i2c_new_device() failed\n", __func__);
		i2c_put_adapter(adapter);
		return -EINVAL;
	}

	struct dentry *dent = debugfs_create_dir("lp8557i", NULL);

	debugfs_create_u8("11h", 0777, dent, &value_0x11);
	value_0x11 = 0x06;

	debugfs_create_u8("12h", 0777, dent, &value_0x12);
	value_0x12 = 0x2C;

	debugfs_create_u8("13h", 0777, dent, &value_0x13);
	value_0x13 = 0x02;

	debugfs_create_u8("15h", 0777, dent, &value_0x15);
	value_0x15 = 0xC3;

	return 0;
}


int __init lp8557i_i2c_init(void)
{

	lp8557i_create_i2c_device(); //tmp to do this

	return i2c_add_driver(&lp8557i_i2c_driver);
}

void __exit lp8557i_i2c_exit(void)
{

	i2c_del_driver(&lp8557i_i2c_driver);

}

