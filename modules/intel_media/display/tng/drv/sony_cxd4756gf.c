#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/board_asustek.h>

#include "sony_cxd4756gf.h"
#include "cadiz_ioctl.h"

#define CADIZ_LCM_EN	189
#define CADIZ_RESET	188
#define CADIZ_SYSCLK	177
#define CADIZ_I2C_ADDR 0x10 //SLAVEAD pin low
#define CADIZ_I2C_ADAPTER 5
#define CADIZ_I2C_DEV_NAME "cxd4756gf"

#define DEBUG                1
#define DEBUG_BOOT_REGISTERS 0
#define DEBUG_VERBOSE 0

#if DEBUG
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>

static char* boot1_path = "/data/Cadiz_boot1.csv";
static char* boot2_path = "/data/Cadiz_i2c_boot2.csv";
static char* boot3_path = "/data/Cadiz_boot3.csv";
static char* boot4_path = "/data/Cadiz_boot4.csv";
static char* ipc_ibc_path = "/data/Cadiz_ipc_ibc.csv";

static u16 dbg_read_reg;
#endif

static int reinit;
static int init_done = 0;
static int green_light = 0;

static int cadiz_lcm_en_gpio;
static int cadiz_reset_gpio;
static int cadiz_sysclk_gpio;
static struct i2c_client *cadiz_client;

static struct reg_val* p_boot1;
static struct reg_val* p_boot2;
static struct reg_val* p_boot3;
static struct reg_val* p_boot4;
static struct reg_val* p_ipc_ibc;

static int len_boot1;
static int len_boot2;
static int len_boot3;
static int len_boot4;
static int len_ipc_ibc;
static int check_tables_exist(void);
static int save_tables_and_pass_regs(struct cadiz_regs*, int save);

static struct miscdevice cadiz_dev;
static struct cadiz_regs cadiz_regs;

static project_id project;

int cadiz_power(int val)
{
	if(!init_done) {
		return 0;
	}

	if(project == 8 || project == 10)
		return 0;

	pr_info("%s: val=%d\n", __func__, val);

	if (val > 0)
		gpio_set_value_cansleep(cadiz_lcm_en_gpio, 1);
	else
		gpio_set_value_cansleep(cadiz_lcm_en_gpio, 0);
	return 0;
}

int cadiz_reset(int val)
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s: val=%d\n", __func__, val);

	if (val > 0) {
		gpio_set_value_cansleep(cadiz_sysclk_gpio, 1);
		usleep_range(5000, 5100); //5ms
		gpio_set_value_cansleep(cadiz_reset_gpio, 1);
	} else {
		gpio_set_value_cansleep(cadiz_reset_gpio, 0);
		usleep_range(100, 110); //100us
		gpio_set_value_cansleep(cadiz_sysclk_gpio, 0);
	}
	return 0;
}

static int cadiz_i2c_reg_read(struct i2c_client *client, u16 reg, u8 *value)
{
	if (!client) {
		pr_err("no cadiz i2c\n");
		return -1;
	}

	int r;
	u8 tx_data[] = {
		reg >> 8 ,      //SubAddress(MSB) 8bit
		reg & 0xff,     //SubAddress(LSB) 8bit
	};

	u8 rx_data[1]={0};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d\n", __func__,
				reg, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
				reg, r);
		return -EAGAIN;
	}
#if DEBUG && DEBUG_VERBOSE
	pr_info("%s: 0x%04xh = 0x%02x (r = %d)\n", __func__, reg, rx_data[0], r);
#endif
	*value = rx_data[0];

	return r;
}

static int cadiz_i2c_reg_write(struct i2c_client *client, u16 reg, u8 value)
{
	if (!client) {
		pr_err("no cadiz i2c\n");
		return -1;
	}

	int r;
	u8 tx_data[] = {
		reg >> 8 ,      //SubAddress(MSB) 8bit
		reg & 0xff,     //SubAddress(LSB) 8bit
		value & 0xff,   //Data 8bit
	};

	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%02x error %d\n",
				__func__, reg, value, r);
		return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%02x msgs %d\n",
				__func__, reg, value, r);
		return -EAGAIN;
	}
#if DEBUG && DEBUG_VERBOSE
	pr_info("%s: 0x%04x, 0x%02x ===> 0x%02x, 0x%02x, 0x%02x\n", __func__, reg, value, tx_data[0], tx_data[1], tx_data[2]);
#endif
	return r;
}

int cadiz_enable_i2c(void)
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
	return cadiz_i2c_reg_write(cadiz_client, 0x0830, 0x00);
}

int cadiz_wait_tx_init_done(void)
{
#define CADIZ_POLLING_COUNT 10
#define MAGIC_REG	0x0207
#define MAGIC_VALUE 0x40
	int cnt, r;
	u8 rdata;

	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);

	for(cnt = 0; cnt < CADIZ_POLLING_COUNT; cnt++) {
		r = cadiz_i2c_reg_read(cadiz_client, MAGIC_REG, &rdata);

		if (r < 0) {
			break;
		} else if (rdata != MAGIC_VALUE) {
			usleep_range(10000, 10000);
			continue;
		}else {
			//pr_info("wait count = %d\n", cnt);
			break;
		}
	}

	if (rdata!= MAGIC_VALUE) {
		pr_err("%s: Polling FAIL :0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n"
			, __func__, MAGIC_REG, rdata, r, MAGIC_VALUE);
		return -EIO;
	} else {
		pr_info("%s: Polling SUCCESS\n", __func__);
	}
	return r;
#undef CADIZ_POLLING_COUNT
#undef MAGIC_REG
#undef MAGIC_VALUE
}

int cadiz_wait_trans_complete(void)
{
#define CADIZ_POLLING_COUNT 50
#define MAGIC_REG	0x0404
#define MAGIC_VALUE 0x38
	int cnt, r;
	u8 rdata;

	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);

	for(cnt = 0; cnt < CADIZ_POLLING_COUNT; cnt++) {
		r = cadiz_i2c_reg_read(cadiz_client, MAGIC_REG, &rdata);

		if (r < 0) {
			break;
		} else if (rdata != MAGIC_VALUE) {
			usleep_range(10000, 10000);
			continue;
		}else {
			//pr_info("wait count = %d\n", cnt);
			break;
		}
	}

	if (rdata!= MAGIC_VALUE) {
		pr_err("%s: Polling FAIL :0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n"
			, __func__, MAGIC_REG, rdata, r, MAGIC_VALUE);
		return -EIO;
	} else {
		pr_info("%s: Polling SUCCESS\n", __func__);
	}
	return r;
#undef CADIZ_POLLING_COUNT
#undef MAGIC_REG
#undef MAGIC_VALUE
}



#if DEBUG
static u8 AscIItoByte(u8 Ascii)
{
	u8 B;
	if (Ascii >= 0x61)
		B = Ascii-0x57;
	else
		B = Ascii >= 0x41 ? Ascii-0x37 : Ascii-0x30;
	return B;
}

static int read_registers_from_file(char* path, struct reg_val* a)
{
	mm_segment_t old_fs;
	struct file *fp;
	int f_size, i;
	int num_head_line_skip = 2;
	int line_cnt = 1;
	int start_read = 0;
	u16 read_reg;
	u8 read_val;
	u8 *pbuf;

	fp = filp_open(path, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(fp)) {
		pr_info("%s opened success\n", path);
		f_size = fp->f_dentry->d_inode->i_size;
		pr_info("f_size=%d\n", f_size);
		pbuf = kmalloc(f_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if (fp->f_op && fp->f_op->read) {
			pr_info("Start to read\n");
			int byte_cnt;
			byte_cnt = fp->f_op->read(fp, pbuf, f_size, &fp->f_pos);
			if(byte_cnt <= 0) {
				pr_err("EOF or error. last byte_cnt= %d;\n", byte_cnt);
				kfree(pbuf);
				return -1;
			}
			for (i = 0; i < f_size; i++) {
#if 0
				pr_info("pbuf[%d]: c=%c, d=%d\n", i, pbuf[i], pbuf[i]);
#endif
				if(pbuf[i] == 0xA) { //new line
					line_cnt ++;
					start_read = 1;
					continue;
				}
				if(line_cnt <= num_head_line_skip) {
					start_read = 0;
					continue;
				}
				if(start_read) {
					start_read = 0;
					read_reg = AscIItoByte(pbuf[i]) << 12 |
						AscIItoByte(pbuf[++i]) << 8 |
						AscIItoByte(pbuf[++i]) << 4 |
						AscIItoByte(pbuf[++i]);
					if (pbuf[++i] != ',') {
						pr_err("invalid format pbuf[%d] = %c, stop\n", i, pbuf[i]);
						return -1;
					}
					read_val = AscIItoByte(pbuf[++i]) << 4 |
						   AscIItoByte(pbuf[++i]);
#if 0
					pr_info("line_cnt=%d, read_reg=0x%04x, read_val=0x%02x\n"
						, line_cnt, read_reg, read_val);
#endif
					a[line_cnt - num_head_line_skip - 1].reg = read_reg;
					a[line_cnt - num_head_line_skip - 1].val = read_val;
				}
			}
			pr_info("End of read\n");
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else {
		pr_err("%s opened fail\n", path);
		return -1;
	}
	return 0;
}
static ssize_t dbg_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int dbg_reg_write(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char dbg_buf[20];
	u16 dbg_reg;
	u8 dbg_val;
	int r, j;
	struct cadiz_regs cadiz_regs;

	if(check_tables_exist())
		return -EPERM;
	if(count > sizeof(dbg_buf))
		return -EINVAL;
	if(copy_from_user(dbg_buf, buf, count))
		return -EINVAL;
	dbg_buf[count] = '\0';

	//sscanf(dbg_buf, "%4x[ ]%2x", &dbg_reg, &dbg_val);
	dbg_reg = AscIItoByte(dbg_buf[0]) << 12 |
		  AscIItoByte(dbg_buf[1]) << 8 |
		  AscIItoByte(dbg_buf[2]) << 4 |
		  AscIItoByte(dbg_buf[3]);
	dbg_val = AscIItoByte(dbg_buf[5]) << 4 |
		  AscIItoByte(dbg_buf[6]);

	pr_info("%s: reg=0x%x, val=0x%x"
		, __func__, dbg_reg, dbg_val);

	cadiz_regs.len = 1;
	cadiz_regs.regs[0][0] = dbg_reg;
	cadiz_regs.regs[0][1] = dbg_val;
	r = save_tables_and_pass_regs(&cadiz_regs, 0);
	if(r < 0)
		return r;

	return count;
}

static const struct file_operations dbg_reg_fops = {
	.open		= dbg_reg_open,
	.write		= dbg_reg_write,
};

static int dbg_read_reg_write(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char dbg_buf[20];
	u16 dbg_reg;
	int r;

	if(count > sizeof(dbg_buf))
		return -EINVAL;
	if(copy_from_user(dbg_buf, buf, count))
		return -EINVAL;
	dbg_buf[count] = '\0';

	//sscanf(dbg_buf, "%4x[ ]%2x", &dbg_reg, &dbg_val);
	dbg_reg = AscIItoByte(dbg_buf[0]) << 12 |
		  AscIItoByte(dbg_buf[1]) << 8 |
		  AscIItoByte(dbg_buf[2]) << 4 |
		  AscIItoByte(dbg_buf[3]);

	pr_info("%s: to read reg=0x%04x\n"
		, __func__, dbg_reg);

	dbg_read_reg = dbg_reg;

	return count;
}

static ssize_t dbg_read_reg_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	char buf[20];
	u16 reg = dbg_read_reg;
	u8 val;
	int r;

	r = cadiz_i2c_reg_read(cadiz_client, reg, &val);
	if (r < 0)
		snprintf(buf, 20, "%s\n", "i2c read fail...");
	else
		snprintf(buf, 20, "0x%04x 0x%02x\n", reg, val);

	return simple_read_from_buffer(user_buf, count, ppos,
			buf, strlen(buf));
}

static const struct file_operations dbg_read_reg_fops = {
	.open		= simple_open,
	.write		= dbg_read_reg_write,
	.read		= dbg_read_reg_read,
};

static int dbg_read_ipc_ibc_show(struct seq_file *s, void *unused)
{
	int i;

	pr_info("%s\n", __func__);

	if(!p_ipc_ibc)
		return 0;

	for(i = 0; i < len_ipc_ibc; i++)
		seq_printf(s, "%d:reg(0x%04x) val(0x%02x)\n"
			, i, p_ipc_ibc[i].reg, p_ipc_ibc[i].val);

	return 0;
}

static int dbg_read_ipc_ibc_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_read_ipc_ibc_show, inode->i_private);
}

static const struct file_operations dbg_read_ipc_ibc_fops = {
	.open		= dbg_read_ipc_ibc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


static int dbg_check_tcon_status_show(struct seq_file *s, void *unused)
{
#define CADIZ_POLLING_COUNT 10
	int i, r;
	u8 rdata = 0,value1 = 0, value2 = 0,value3 = 0,value4 = 0,value5 = 0;

	//step 1:
	struct reg_val step1[] = {
		{0x096c, 0x00},
		{0x0960, 0x40},
		{0x0962, 0xf8},
		{0x096d, 0x00},
		{0x0961, 0xfc},
		{0x0963, 0xfe},
		{0x0964, 0x0a}, //0x0A is only for DCS short without parameter.
		{0x0965, 0x00},
		{0x096c, 0x01},
		{0x096c, 0x00},
	};

	pr_info("%s ++\n", __func__);

	for(i = 0; i < ARRAY_SIZE(step1); i++) {
		r = cadiz_i2c_reg_write(cadiz_client, step1[i].reg, step1[i].val);
		if (r < 0)
			return -EIO;
	}

	usleep_range(60000, 65000); //60ms , > 3 frames

	//step 2:
	for(i = 0; i < CADIZ_POLLING_COUNT; i++) {
		r = cadiz_i2c_reg_read(cadiz_client, 0x040C, &rdata);

		if (r < 0) {
			break;
		} else if (rdata != 0x02) {
			usleep_range(10000, 10000);
			continue;
		}else {
			pr_info("0x040C wait count = %d\n", i);
			break;
		}
	}

	cadiz_i2c_reg_write(cadiz_client, 0x040C, 0x00);

	//step 3: read results
	cadiz_i2c_reg_read(cadiz_client, 0x0410, &value1);

	cadiz_i2c_reg_read(cadiz_client, 0x0204, &value2);
	cadiz_i2c_reg_read(cadiz_client, 0x0205, &value3);
	cadiz_i2c_reg_read(cadiz_client, 0x0206, &value4);
	cadiz_i2c_reg_read(cadiz_client, 0x0207, &value5);

	//Error report form tcon as:
	//0x87
	//0x21 [value1] 0x00 [ECC]
	//0x02 [value2] [value3, value5] [ECC]
	seq_printf(s, "reg(0x%04x) val(0x%02x)\n"
		, 0x0410, value1);
	seq_printf(s, "reg(0x%04x) val(0x%02x)\n"
		, 0x0204, value2);
	seq_printf(s, "reg(0x%04x) val(0x%02x)\n"
		, 0x0205, value3);
	seq_printf(s, "reg(0x%04x) val(0x%02x)\n"
		, 0x0206, value4);
	seq_printf(s, "reg(0x%04x) val(0x%02x)\n"
		, 0x0207, value5);

	pr_info("%s --\n", __func__);
	return 0;
#undef CADIZ_POLLING_COUNT
}

static int dbg_check_tcon_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_check_tcon_status_show, inode->i_private);
}

static const struct file_operations dbg_check_tcon_status_fops = {
	.open		= dbg_check_tcon_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif

int cadiz_setup_boot1()
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
#if DEBUG && DEBUG_BOOT_REGISTERS
	struct reg_val debug_boot1[173];
	int i, r;
	int ret;
	
	ret = read_registers_from_file(boot1_path, debug_boot1);
	if(ret < 0) {
		pr_err("read fail\n");
		return -EAGAIN;
	}

	len = sizeof(debug_boot1)/sizeof(debug_boot1[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot1[i].reg, debug_boot1[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	int i, r;

	if(!p_boot1) {
		pr_err("no boot1\n");
		return -1;
	}

	for(i = 0; i < len_boot1; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, p_boot1[i].reg, p_boot1[i].val);
		if (r < 0)
			return -EIO;
	}
#endif
	return 0;
}

int cadiz_setup_boot2()
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
#if DEBUG && DEBUG_BOOT_REGISTERS
	struct reg_val debug_boot2[12];
	int i, r;
	int ret;
	
	ret = read_registers_from_file(boot2_path, debug_boot2);
	if(ret < 0) {
		pr_err("read fail\n");
		return -EAGAIN;
	}

	len = sizeof(debug_boot2)/sizeof(debug_boot2[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot2[i].reg, debug_boot2[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	int i, r;

	if(!p_boot2) {
		pr_err("no boot2\n");
		return -1;
	}

	for(i = 0; i < len_boot2; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, p_boot2[i].reg, p_boot2[i].val);
		if (r < 0)
			return -EIO;
	}
#endif
	return 0;
}

int cadiz_setup_boot3()
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
#if DEBUG && DEBUG_BOOT_REGISTERS
	struct reg_val debug_boot3[3];
	int i, r;
	int ret;
	
	ret = read_registers_from_file(boot3_path, debug_boot3);
	if(ret < 0) {
		pr_err("read fail\n");
		return -EAGAIN;
	}

	len = sizeof(debug_boot3)/sizeof(debug_boot3[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot3[i].reg, debug_boot3[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	int i, r;

	if(!p_boot3) {
		pr_err("no boot3\n");
		return -1;
	}

	for(i = 0; i < len_boot3; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, p_boot3[i].reg, p_boot3[i].val);
		if (r < 0)
			return -EIO;
	}
#endif
	return 0;
}

int cadiz_setup_boot4()
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
#if DEBUG && DEBUG_BOOT_REGISTERS
	struct reg_val debug_boot4[4];
	int i, r;
	int ret;
	
	ret = read_registers_from_file(boot4_path, debug_boot4);
	if(ret < 0) {
		pr_err("read fail\n");
		return -EAGAIN;
	}

	len = sizeof(debug_boot4)/sizeof(debug_boot4[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot4[i].reg, debug_boot4[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	int i, r;

	if(!p_boot4) {
		pr_err("no boot4\n");
		return -1;
	}

	for(i = 0; i < len_boot4; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, p_boot4[i].reg, p_boot4[i].val);
		if (r < 0)
			return -EIO;
	}
#endif
	return 0;
}

int cadiz_setup_ipc_ibc()
{
	if(!init_done) {
		return 0;
	}

	pr_info("%s\n", __func__);
#if DEBUG && DEBUG_BOOT_REGISTERS
	struct reg_val debug_ipc_ibc[232];
	int i, r;
	int ret;
	
	ret = read_registers_from_file(ipc_ibc_path, debug_ipc_ibc);
	if(ret < 0) {
		pr_err("read fail\n");
		return -EAGAIN;
	}

	len = sizeof(debug_ipc_ibc)/sizeof(debug_ipc_ibc[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_ipc_ibc[i].reg, debug_ipc_ibc[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	int i, r;

	if(!p_ipc_ibc) {
		pr_err("no ipc_ibc\n");
		return -1;
	}

	for(i = 0; i < len_ipc_ibc; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, p_ipc_ibc[i].reg, p_ipc_ibc[i].val);
		if (r < 0)
			return -EIO;
	}
#endif
	return 0;
}

int cadiz_setup_regs(struct reg_val* regs, int len)
{
	int i, r;

	if(!init_done) {
		return 0;
	}

	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, regs[i].reg, regs[i].val);
		if (r < 0)
			return -EIO;
	}
	return 0;
}

void cadiz_pass_tables(struct reg_val* b1,  int b1_len,
		       struct reg_val* b2,  int b2_len,
		       struct reg_val* b3,  int b3_len,
		       struct reg_val* b4,  int b4_len,
		       struct reg_val* ipb, int ipb_len)
{
	if(!init_done) {
		return;
	}

	pr_info("%s\n", __func__);

	p_boot1 = b1;
	len_boot1 = b1_len;

	p_boot2 = b2;
	len_boot2 = b2_len;

	p_boot3 = b3;
	len_boot3 = b3_len;

	p_boot4 = b4;
	len_boot4 = b4_len;

	p_ipc_ibc = ipb;
	len_ipc_ibc = ipb_len;
}

static int check_tables_exist(void)
{
	if(!p_boot1 || !p_boot2 || !p_boot3 || !p_boot4 || !p_ipc_ibc)
		return -1;
	else
		return 0;
}

static int save_tables_and_pass_regs(struct cadiz_regs* regs, int save)
{
	int i, j, r, len;
	u16 t_reg;
	u8  t_val;

	len = regs->len;

	for(i = 0; i < len; i++) {

		t_reg = (u16)(regs->regs[i][0]);
		t_val = (u8)(regs->regs[i][1]);

		if(!save) {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;
			continue;
		}

		//1. check if belong to ipc_ibc
		for(j = 0; j < len_ipc_ibc; j++)
			if(t_reg == p_ipc_ibc[j].reg)
				break;
		if(j == len_ipc_ibc) {
#if DEBUG && DEBUG_VERBOSE
			dev_info(&cadiz_client->dev,
					"reg: 0x%04x does not belong to ipc_ibc\n", t_reg);
#endif
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;

			//save the change
			p_ipc_ibc[j].reg = t_reg;
			p_ipc_ibc[j].val = t_val;
			continue;
		}
		//Most of changes will be expected in ipc_ibc, if not, check further.
		//2. if not, check boot1
		for(j = 0; j < len_boot1; j++)
			if(t_reg == p_boot1[j].reg)
				break;
		if(j == len_boot1) {
#if DEBUG && DEBUG_VERBOSE
			dev_info(&cadiz_client->dev,
					"reg: 0x%04x does not belong to boot1\n", t_reg);
#endif
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;

			//save the change
			p_boot1[j].reg = t_reg;
			p_boot1[j].val = t_val;
			continue;
		}
/*
		//3. if not, check boot2
		for(j = 0; j < len_boot2; j++)
			if(t_reg == p_boot2[j].reg)
				break;
		if(j == len_boot2) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot2\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;

			//save the change
			p_boot2[j].reg = t_reg;
			p_boot2[j].val = t_val;
			continue;
		}

		//4. if not, check boot3
		for(j = 0; j < len_boot3; j++)
			if(t_reg == p_boot3[j].reg)
				break;
		if(j == len_boot3) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot3\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;

			//save the change
			p_boot3[j].reg = t_reg;
			p_boot3[j].val = t_val;
			continue;
		}

		//5. if not, check boot4
		for(j = 0; j < len_boot4; j++)
			if(t_reg == p_boot4[j].reg)
				break;
		if(j == len_boot4) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot4\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EAGAIN;

			//save the change
			p_boot4[j].reg = t_reg;
			p_boot4[j].val = t_val;
			continue;
		}
*/
		//the change does not belong to all tables
		dev_err(&cadiz_client->dev,
				"invalid reg: 0x%04x(0x%02x)\n", t_reg, t_val);
		return -EINVAL;
	}
	return 0;
}

void cadiz_set_accessible(int yes)
{
	struct cadiz_regs cadiz_regs;

	green_light = yes;

	if(!yes) {
		cadiz_regs.len = 1;
		cadiz_regs.regs[0][0] = 0x0841;
		cadiz_regs.regs[0][1] = 0x7B;
		save_tables_and_pass_regs(&cadiz_regs, 1);
	}
}

static long cadiz_dev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
#if DEBUG && DEBUG_VERBOSE
	dev_info(&cadiz_client->dev, "%s\n", __func__);
#endif
	if(!green_light || !init_done) {
		dev_err(&cadiz_client->dev, "ERROR:not accessible\n", __func__);
		return -EAGAIN;
	}

	switch(cmd) {
	case CADIZ_IOCTL_SET_REGISTERS:
	{
		struct cadiz_regs cadiz_regs;
		int r;

		if(check_tables_exist())
			return -EPERM;

		memset(&cadiz_regs, 0, sizeof(cadiz_regs));
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EINVAL;
		}

		if(cadiz_regs.len > MAX_CADIZ_REGS || cadiz_regs.len <= 0) {
			dev_err(&cadiz_client->dev, "invalid len:%d\n", cadiz_regs.len);
			return -EINVAL;
		}

		r =  save_tables_and_pass_regs(&cadiz_regs, 1);
		if(r < 0)
			return r;

		break;
	}
	case CADIZ_IOCTL_GET_REGISTERS:
	{
		struct cadiz_regs cadiz_regs;
		u16 cur_reg;
		u8  cur_val;
		int i, r;

		memset(&cadiz_regs, 0, sizeof(cadiz_regs));
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EINVAL;
		}

		if(cadiz_regs.len > MAX_CADIZ_REGS || cadiz_regs.len <= 0) {
			dev_err(&cadiz_client->dev, "invalid len:%d\n", cadiz_regs.len);
			return -EINVAL;
		}

		for(i = 0; i< cadiz_regs.len; i++) {
			cur_reg = (u16)cadiz_regs.regs[i][0];
			cur_val = 0;
			r = cadiz_i2c_reg_read(cadiz_client, cur_reg, &cur_val);
			if (r < 0)
				return -EAGAIN;

			 cadiz_regs.regs[i][1] = (int)cur_val;
		}

		if (copy_to_user((void __user *)arg, &cadiz_regs,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "pass arg error\n");
			return -EINVAL;
		}

		break;
	}
	default:
		dev_err(&cadiz_client->dev, "%s:unknown cmd=%d\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
}

static int cadiz_dev_open(struct inode *inode, struct file *filp)
{
	dev_info(&cadiz_client->dev, "%s\n", __func__);
	return nonseekable_open(inode, filp);
}

static const struct file_operations cadiz_dev_fops = {
	.owner = THIS_MODULE,
	.open  = cadiz_dev_open,
	.unlocked_ioctl = cadiz_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cadiz_dev_ioctl,
#endif
};

int cadiz_need_reinit(void)
{
	return reinit;
}

static int cadiz_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	int r;
	u8 rdata_1, rdata_2, rdata_3;

	reinit = 0;

	cadiz_client = NULL;
	cadiz_lcm_en_gpio = 0;
	cadiz_reset_gpio = 0;

	p_boot1   = NULL;
	p_boot2   = NULL;
	p_boot3   = NULL;
	p_boot4   = NULL;
	p_ipc_ibc = NULL;

	len_boot1   = 0;
	len_boot2   = 0;
	len_boot3   = 0;
	len_boot4   = 0;
	len_ipc_ibc = 0;

	project = asustek_get_project_id();

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	cadiz_client = client;

	/* WAR: check if cadiz pwm status is abnormal*/
	r = cadiz_i2c_reg_read(cadiz_client, 0x0C28, &rdata_1);
	if (r < 0)
		return -EIO;

	r = cadiz_i2c_reg_read(cadiz_client, 0x0C3E, &rdata_2);
	if (r < 0)
		return -EIO;

	r = cadiz_i2c_reg_read(cadiz_client, 0x0C3F, &rdata_3);
	if (r < 0)
		return -EIO;

	pr_info("%s:0x0C28(%02x), 0x0C3E(%02x), 0x0C3F(%02x)\n"
			, __func__, rdata_1, rdata_2, rdata_3);

	if(rdata_1 != 0 && rdata_2 == 0 && rdata_3 == 0) {
		pr_err("%s: found 0x0C28(%02x), 0x0C3E(%02x), 0x0C3F(%02x)\n"
			, __func__, rdata_1, rdata_2, rdata_3);
		reinit = 1;
	}

	//No need if cadiz power en is same gpio as panel en
	if(project != 8 && project != 10) {
		cadiz_lcm_en_gpio = CADIZ_LCM_EN;
		if (gpio_request(cadiz_lcm_en_gpio, "cadiz_lcm_en")) {
			pr_err("failed to requeset cadiz_lcm_en\n");
			return -EINVAL;
		}

		if(reinit)
			gpio_direction_output(cadiz_lcm_en_gpio, 0);
		else
			gpio_direction_output(cadiz_lcm_en_gpio, 1);
	}

	cadiz_reset_gpio = CADIZ_RESET;
	if (gpio_request(cadiz_reset_gpio, "cadiz_rst")) {
		pr_err("failed to requeset cadiz_rst\n");
		return -EINVAL;
	}
	if(reinit)
		gpio_direction_output(cadiz_reset_gpio, 0);
	else
		gpio_direction_output(cadiz_reset_gpio, 1);

	cadiz_sysclk_gpio = CADIZ_SYSCLK;
	if (gpio_request(cadiz_sysclk_gpio, "cadiz_sysclk")) {
		pr_err("failed to requeset cadiz_sysclk\n");
		return -EINVAL;
	}

	if(reinit)
		gpio_direction_output(cadiz_sysclk_gpio, 0);
	else
		gpio_direction_output(cadiz_sysclk_gpio, 1);

	cadiz_dev.minor = MISC_DYNAMIC_MINOR;
	cadiz_dev.name = "cadiz";
	cadiz_dev.fops = &cadiz_dev_fops;
	if(misc_register(&cadiz_dev)) {
		dev_err(&client->dev, "%s:fail to register misc device\n", __func__);
	}

#if DEBUG
	struct dentry *dent = debugfs_create_dir("cadiz", NULL);
	debugfs_create_file("write_reg", S_IRUGO | S_IWUSR, dent, NULL, &dbg_reg_fops);
	debugfs_create_file("read_reg", S_IRUGO | S_IWUSR, dent, NULL, &dbg_read_reg_fops);
	debugfs_create_file("show_ipc_ibc", S_IRUGO, dent, NULL, &dbg_read_ipc_ibc_fops);
	debugfs_create_file("check_tcon_status", S_IRUGO, dent, NULL, &dbg_check_tcon_status_fops);
#endif

	init_done = 1;
	green_light = 1;

	pr_info("%s --\n", __func__);
	return 0;

}

static int cadiz_i2c_remove(struct i2c_client *client)
{
        return 0;
}

static const struct i2c_device_id cadiz_id[] = {
       {CADIZ_I2C_DEV_NAME, 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, cadiz_id);

static struct i2c_driver cadiz_i2c_driver = {
       .driver = {
               .name = CADIZ_I2C_DEV_NAME,
       },
       .id_table = cadiz_id,
       .probe = cadiz_i2c_probe,
       .remove = cadiz_i2c_remove,
};

int __init cadiz_i2c_init(void)
{
	return i2c_add_driver(&cadiz_i2c_driver);
}

void __exit cadiz_i2c_exit(void)
{
        i2c_del_driver(&cadiz_i2c_driver);
}
