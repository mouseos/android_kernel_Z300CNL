/*
 * pmic_ccsm.c - Intel MID PMIC Charger Driver
 *
 * Copyright (C) 2011 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Jenny TC <jenny.tc@intel.com>
 * Author: Yegnesh Iyer <yegnesh.s.iyer@intel.com>
 */

/* Includes */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/param.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/rpmsg.h>
#include <linux/version.h>
#include <asm/intel_basincove_gpadc.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
#include <linux/iio/consumer.h>
#else
#include "../../../kernel/drivers/staging/iio/consumer.h"
#endif
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/pm_runtime.h>
#include <linux/sfi.h>
#include <linux/async.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/power/battery_id.h>
#include <linux/gpio.h>
#include <linux/board_asustek.h>
#include "pmic_ccsm.h"
#include <asm/spid.h>

/* Macros */
#define DRIVER_NAME "pmic_ccsm"
#define PMIC_SRAM_INTR_ADDR 0xFFFFF616
#define ADC_TO_TEMP 1
#define TEMP_TO_ADC 0

#ifdef PMIC_A0_WAR
#define AS_DOCK_IN 46
#define USB_PATH_SEL 48
#define USBIN_ACOK_N 52
#define DCIN_ACOK_N 54
#define CHG_ACOK_N 179
static struct workqueue_struct *pmic_gpio_acok_wq;
static struct workqueue_struct *as_gpio_dock_in_wq;
#endif

#define is_valid_temp(tmp)\
	(!(tmp > chc.pdata->adc_tbl[0].temp ||\
	tmp < chc.pdata->adc_tbl[chc.pdata->max_tbl_row_cnt - 1].temp))
#define is_valid_adc_code(val)\
	(!(val < chc.pdata->adc_tbl[0].adc_val ||\
	val > chc.pdata->adc_tbl[chc.pdata->max_tbl_row_cnt - 1].adc_val))
#define CONVERT_ADC_TO_TEMP(adc_val, temp)\
	adc_temp_conv(adc_val, temp, ADC_TO_TEMP)
#define CONVERT_TEMP_TO_ADC(temp, adc_val)\
	adc_temp_conv(temp, adc_val, TEMP_TO_ADC)
#define NEED_ZONE_SPLIT(bprof)\
	 ((bprof->temp_mon_ranges < MIN_BATT_PROF))

#define USB_WAKE_LOCK_TIMEOUT	(5 * HZ)
#define ACOK_WAKE_LOCK_TIMEOUT	(2 * HZ)

/* 100mA value definition for setting the inlimit in bq24261 */
#define USBINPUTICC100VAL	100

#define OHM_MULTIPLIER		10

/* Type definitions */
static void pmic_bat_zone_changed(void);
static void pmic_battery_overheat_handler(bool);

static void handle_acok_falling_interrupt();

/* Extern definitions */

/* Global declarations */
static project_id project = 0;
static int pmic_acok = CHG_ACOK_N;
static DEFINE_MUTEX(pmic_lock);
static struct pmic_chrgr_drv_context chc;
static struct interrupt_info chgrirq0_info[] = {
	{
		CHGIRQ0_BZIRQ_MASK,
		0,
		"Battery temperature zone changed",
		NULL,
		NULL,
		pmic_bat_zone_changed,
		NULL,
	},
	{
		CHGIRQ0_BAT_CRIT_MASK,
		SCHGIRQ0_SBAT_CRIT_MASK,
		NULL,
		"Battery Over heat exception",
		"Battery Over heat exception Recovered",
		NULL,
		pmic_battery_overheat_handler
	},
	{
		CHGIRQ0_BAT0_ALRT_MASK,
		SCHGIRQ0_SBAT0_ALRT_MASK,
		NULL,
		"Battery0 temperature inside boundary",
		"Battery0 temperature outside boundary",
		NULL,
		pmic_battery_overheat_handler
	},
	{
		CHGIRQ0_BAT1_ALRT_MASK,
		SCHGIRQ0_SBAT1_ALRT_MASK,
		NULL,
		"Battery1 temperature inside boundary",
		"Battery1 temperature outside boundary",
		NULL,
		NULL
	},
};

u16 pmic_inlmt[][2] = {
	{ 100, CHGRCTRL1_FUSB_INLMT_100},
	{ 150, CHGRCTRL1_FUSB_INLMT_150},
	{ 500, CHGRCTRL1_FUSB_INLMT_500},
	{ 900, CHGRCTRL1_FUSB_INLMT_900},
	{ 1500, CHGRCTRL1_FUSB_INLMT_1500},
	{ 2000, CHGRCTRL1_FUSB_INLMT_1500},
	{ 2500, CHGRCTRL1_FUSB_INLMT_1500},
};


static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}


/* Function definitions */
static void lookup_regval(u16 tbl[][2], size_t size, u16 in_val, u8 *out_val)
{
	int i;
	for (i = 1; i < size; ++i)
		if (in_val < tbl[i][0])
			break;

	*out_val = (u8)tbl[i-1][1];
}

static int interpolate_y(int dx1x0, int dy1y0, int dxx0, int y0)
{
	return y0 + DIV_ROUND_CLOSEST((dxx0 * dy1y0), dx1x0);
}

static int interpolate_x(int dy1y0, int dx1x0, int dyy0, int x0)
{
	return x0 + DIV_ROUND_CLOSEST((dyy0 * dx1x0), dy1y0);
}

static int adc_temp_conv(int in_val, int *out_val, int conv)
{
	int tbl_row_cnt, i;
	struct temp_lookup *adc_temp_tbl;

	if (!chc.pdata) {
		dev_err(chc.dev, "ADC-lookup table not yet available\n");
		return -ERANGE;
	}

	tbl_row_cnt = chc.pdata->max_tbl_row_cnt;
	adc_temp_tbl = chc.pdata->adc_tbl;

	if (conv == ADC_TO_TEMP) {
		if (!is_valid_adc_code(in_val))
			return -ERANGE;

		if (in_val == adc_temp_tbl[tbl_row_cnt-1].adc_val)
			i = tbl_row_cnt - 1;
		else {
			for (i = 0; i < tbl_row_cnt; ++i)
				if (in_val < adc_temp_tbl[i].adc_val)
					break;
		}

		*out_val =
		    interpolate_y((adc_temp_tbl[i].adc_val
					- adc_temp_tbl[i - 1].adc_val),
				  (adc_temp_tbl[i].temp
				   - adc_temp_tbl[i - 1].temp),
				  (in_val - adc_temp_tbl[i - 1].adc_val),
				  adc_temp_tbl[i - 1].temp);
	} else {
		if (!is_valid_temp(in_val))
			return -ERANGE;

		if (in_val == adc_temp_tbl[tbl_row_cnt-1].temp)
			i = tbl_row_cnt - 1;
		else {
			for (i = 0; i < tbl_row_cnt; ++i)
				if (in_val > adc_temp_tbl[i].temp)
					break;
		}

		*((short int *)out_val) =
		    interpolate_x((adc_temp_tbl[i].temp
					- adc_temp_tbl[i - 1].temp),
				  (adc_temp_tbl[i].adc_val
				   - adc_temp_tbl[i - 1].adc_val),
				  (in_val - adc_temp_tbl[i - 1].temp),
				  adc_temp_tbl[i - 1].adc_val);
	}
	return 0;
}

static int pmic_read_reg(u16 addr, u8 *val)
{
	int ret;

	ret = intel_scu_ipc_ioread8(addr, val);
	if (ret) {
		dev_err(chc.dev,
			"Error in intel_scu_ipc_ioread8 0x%.4x\n", addr);
		return -EIO;
	}
	return 0;
}


static int __pmic_write_tt(u8 addr, u8 data)
{
	int ret;

	ret = intel_scu_ipc_iowrite8(CHRTTADDR_ADDR, addr);
	if (unlikely(ret))
		return ret;

	return intel_scu_ipc_iowrite8(CHRTTDATA_ADDR, data);
}

static inline int pmic_write_tt(u8 addr, u8 data)
{
	int ret;

	mutex_lock(&pmic_lock);
	ret = __pmic_write_tt(addr, data);
	mutex_unlock(&pmic_lock);

	/* If access is blocked return success to avoid additional
	*  error handling at client side
	*/
	if (ret == -EACCES) {
		dev_warn(chc.dev, "IPC write blocked due to unsigned kernel/invalid battery\n");
		ret = 0;
	}
	return ret;
}

static int __pmic_read_tt(u8 addr, u8 *data)
{
	int ret;

	ret = intel_scu_ipc_iowrite8(CHRTTADDR_ADDR, addr);
	if (ret)
		return ret;

	usleep_range(2000, 3000);

	return intel_scu_ipc_ioread8(CHRTTDATA_ADDR, data);
}

static inline int pmic_read_tt(u8 addr, u8 *data)
{
	int ret;

	mutex_lock(&pmic_lock);
	ret = __pmic_read_tt(addr, data);
	mutex_unlock(&pmic_lock);

	return ret;
}

static int pmic_update_tt(u8 addr, u8 mask, u8 data)
{
	u8 tdata;
	int ret;

	mutex_lock(&pmic_lock);
	ret = __pmic_read_tt(addr, &tdata);
	if (unlikely(ret))
		goto exit;

	tdata = (tdata & ~mask) | (data & mask);
	ret = __pmic_write_tt(addr, tdata);
exit:
	mutex_unlock(&pmic_lock);
	return ret;
}


/**
 * pmic_read_adc_val - read ADC value of specified sensors
 * @channel: channel of the sensor to be sampled
 * @sensor_val: pointer to the charger property to hold sampled value
 * @chc :  battery info pointer
 *
 * Returns 0 if success
 */
static int pmic_read_adc_val(int channel, int *sensor_val,
			      struct pmic_chrgr_drv_context *chc)
{
	int val;
	int ret;
	struct iio_channel *indio_chan = NULL;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	switch (channel) {
	case GPADC_BATTEMP0:
		indio_chan = iio_st_channel_get("BATTEMP", "BATTEMP0");
		break;
	case GPADC_BATID:
		indio_chan = iio_st_channel_get("BATID", "BATID");
		break;
	default:
		dev_err(chc->dev, "invalid sensor%d", channel);
		ret = -EINVAL;
	}
#else
	switch (channel) {
	case GPADC_BATTEMP0:
		indio_chan = iio_channel_get(NULL, "BATTEMP0");
		break;
	case GPADC_BATID:
		indio_chan = iio_channel_get(NULL, "BATID");
		break;
	default:
		dev_err(chc->dev, "invalid sensor%d", channel);
		ret = -EINVAL;
	}
#endif
	if (IS_ERR_OR_NULL(indio_chan)) {
		ret = PTR_ERR(indio_chan);
		goto exit;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	ret = iio_st_read_channel_raw(indio_chan, &val);
#else
	ret = iio_read_channel_raw(indio_chan, &val);
#endif
	if (ret) {
		dev_err(chc->dev, "IIO channel read error\n");
		goto err_exit;
	}

	switch (channel) {
	case GPADC_BATTEMP0:
		ret = CONVERT_ADC_TO_TEMP(val, sensor_val);
		break;
	case GPADC_BATID:
		*sensor_val = val;
		break;
	default:
		dev_err(chc->dev, "invalid sensor%d", channel);
		ret = -EINVAL;
	}
	dev_dbg(chc->dev, "pmic_ccsm pmic_ccsm.0: %s adc val=%x, %d sensorval=%d\n",
		__func__, val, val, *sensor_val);

err_exit:
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	iio_st_channel_release(indio_chan);
#else
	iio_channel_release(indio_chan);
#endif
exit:
	return ret;
}

int pmic_get_battery_pack_temp(int *temp)
{
	if (chc.invalid_batt)
		return -ENODEV;
	return pmic_read_adc_val(GPADC_BATTEMP0, temp, &chc);
}

int pmic_get_battery_id(int *batt_id)
{
	int retval = 0;
	if (chc.invalid_batt)
		return -ENODEV;

	/* Toggle BATTRMSRC to enable VREFB*/
	retval = intel_scu_ipc_iowrite8(BATTDETCTRL_ADDR, 0x05);
	if (unlikely(retval)) {
		dev_err(chc.dev, "%s: Unable to enable VREFB", __func__);
		return -ENODEV;
	}

	/* Expend TLP delay time to get correct battery id value*/
	intel_scu_ipc_iowrite8(TLP2INSTMEMADDR_REG, TLP2INSTMEMADDR_VALUE);
	intel_scu_ipc_iowrite8(TLP2INSTMEMDATA_REG, TLP2INSTMEMDATA_VALUE);

	/* Take and discard a few samples to increase reliability */
	pmic_read_adc_val(GPADC_BATID, batt_id , &chc);
	pmic_read_adc_val(GPADC_BATID, batt_id , &chc);
	pmic_read_adc_val(GPADC_BATID, batt_id , &chc);
	pmic_read_adc_val(GPADC_BATID, batt_id , &chc);

	/* Read BATT_ID resistance value from GPADC Channel 1 */
	if (!pmic_read_adc_val(GPADC_BATID, batt_id , &chc)) {
		/* Toggle BATTRMSRC to turn off VREFB*/
		retval = intel_scu_ipc_iowrite8(BATTDETCTRL_ADDR, 0x01);
		if (unlikely(retval)) {
			dev_err(chc.dev, "%s: Unable to disable VREFB", __func__);
			return -ENODEV;
		}
	} else {
		dev_err(chc.dev, "%s: Unable to read BATTID", __func__);
		return -ENODEV;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(pmic_get_battery_id);

#ifdef CONFIG_DEBUG_FS
static int pmic_chrgr_reg_show(struct seq_file *seq, void *unused)
{
	int ret;
	u16 addr;
	u16 val1;
	u8 val;

	addr = *((u8 *)seq->private);

	if (addr == CHRGRIRQ1_ADDR) {
		val1 = ioread16(chc.pmic_intr_iomap);
		val = (u8)(val1 >> 8);
	} else if (addr == CHGRIRQ0_ADDR) {
		val1 = ioread16(chc.pmic_intr_iomap);
		val = (u8)val1;
	} else {
		ret = pmic_read_reg(addr, &val);
		if (ret != 0) {
			dev_err(chc.dev,
				"Error reading tt register 0x%2x\n",
				addr);
			return -EIO;
		}
	}

	seq_printf(seq, "0x%x\n", val);
	return 0;
}

static int pmic_chrgr_tt_reg_show(struct seq_file *seq, void *unused)
{
	int ret;
	u8 addr;
	u8 val;

	addr = *((u8 *)seq->private);

	ret = pmic_read_tt(addr, &val);
	if (ret != 0) {
		dev_err(chc.dev,
			"Error reading tt register 0x%2x\n",
			addr);
		return -EIO;
	}

	seq_printf(seq, "0x%x\n", val);
	return 0;
}

static int pmic_adc_show(struct seq_file *seq, void *unused)
{
	int val;
	int channel;

	channel = (int)seq->private;
	switch (channel) {
	case GPADC_BATTEMP0:
		if (pmic_get_battery_pack_temp(&val)) {
			seq_printf(seq, "Error reading pack temperature\n");
			return -EIO;
		}
		seq_printf(seq, "Battery Pack Temperature: %d\n", val);
		break;
	case GPADC_BATID:
		if (pmic_get_battery_id(&val)) {
			seq_printf(seq, "Error reading battid\n");
			return -EIO;
		}
		seq_printf(seq, "%d\n", val);
		break;
	default:
		seq_printf(seq, "Invalid ADC Channel %d\n", channel);
		break;
	}
	return 0;
}

static int pmic_chrgr_tt_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_chrgr_tt_reg_show, inode->i_private);
}

static int pmic_chrgr_reg_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_chrgr_reg_show, inode->i_private);
}

static int pmic_adc_open(struct inode *inode, struct file *file)
{
	return single_open(file, pmic_adc_show, inode->i_private);
}

static struct dentry *charger_debug_dir;
static struct pmic_regs_def pmic_regs_bc[] = {
	PMIC_REG_DEF(PMIC_ID_ADDR),
	PMIC_REG_DEF(IRQLVL1_ADDR),
	PMIC_REG_DEF(IRQLVL1_MASK_ADDR),
	PMIC_REG_DEF(CHGRIRQ0_ADDR),
	PMIC_REG_DEF(SCHGRIRQ0_ADDR),
	PMIC_REG_DEF(MCHGRIRQ0_ADDR),
	PMIC_REG_DEF(LOWBATTDET0_ADDR),
	PMIC_REG_DEF(LOWBATTDET1_ADDR),
	PMIC_REG_DEF(BATTDETCTRL_ADDR),
	PMIC_REG_DEF(VBUSDETCTRL_ADDR),
	PMIC_REG_DEF(VDCINDETCTRL_ADDR),
	PMIC_REG_DEF(CHRGRIRQ1_ADDR),
	PMIC_REG_DEF(SCHGRIRQ1_ADDR),
	PMIC_REG_DEF(MCHGRIRQ1_ADDR),
	PMIC_REG_DEF(CHGRCTRL0_ADDR),
	PMIC_REG_DEF(CHGRCTRL1_ADDR),
	PMIC_REG_DEF(CHGRSTATUS_ADDR),
	PMIC_REG_DEF(CHRLEDCTRL_ADDR),
	PMIC_REG_DEF(CHRLEDFSM_ADDR),
	PMIC_REG_DEF(CHRLEDPWM_ADDR),
	PMIC_REG_DEF(USBIDCTRL_ADDR),
	PMIC_REG_DEF(USBIDSTAT_ADDR),
	PMIC_REG_DEF(WAKESRC_ADDR),
	PMIC_REG_DEF(THRMBATZONE_ADDR_BC),
	PMIC_REG_DEF(THRMZN0L_ADDR_BC),
	PMIC_REG_DEF(THRMZN0H_ADDR_BC),
	PMIC_REG_DEF(THRMZN1L_ADDR_BC),
	PMIC_REG_DEF(THRMZN1H_ADDR_BC),
	PMIC_REG_DEF(THRMZN2L_ADDR_BC),
	PMIC_REG_DEF(THRMZN2H_ADDR_BC),
	PMIC_REG_DEF(THRMZN3L_ADDR_BC),
	PMIC_REG_DEF(THRMZN3H_ADDR_BC),
	PMIC_REG_DEF(THRMZN4L_ADDR_BC),
	PMIC_REG_DEF(THRMZN4H_ADDR_BC),
};

static struct pmic_regs_def pmic_regs_sc[] = {
	PMIC_REG_DEF(PMIC_ID_ADDR),
	PMIC_REG_DEF(IRQLVL1_ADDR),
	PMIC_REG_DEF(IRQLVL1_MASK_ADDR),
	PMIC_REG_DEF(CHGRIRQ0_ADDR),
	PMIC_REG_DEF(SCHGRIRQ0_ADDR),
	PMIC_REG_DEF(MCHGRIRQ0_ADDR),
	PMIC_REG_DEF(LOWBATTDET0_ADDR),
	PMIC_REG_DEF(LOWBATTDET1_ADDR),
	PMIC_REG_DEF(BATTDETCTRL_ADDR),
	PMIC_REG_DEF(VBUSDETCTRL_ADDR),
	PMIC_REG_DEF(VDCINDETCTRL_ADDR),
	PMIC_REG_DEF(CHRGRIRQ1_ADDR),
	PMIC_REG_DEF(SCHGRIRQ1_ADDR),
	PMIC_REG_DEF(MCHGRIRQ1_ADDR),
	PMIC_REG_DEF(CHGRCTRL0_ADDR),
	PMIC_REG_DEF(CHGRCTRL1_ADDR),
	PMIC_REG_DEF(CHGRSTATUS_ADDR),
	PMIC_REG_DEF(USBIDCTRL_ADDR),
	PMIC_REG_DEF(USBIDSTAT_ADDR),
	PMIC_REG_DEF(WAKESRC_ADDR),
	PMIC_REG_DEF(USBPHYCTRL_ADDR),
	PMIC_REG_DEF(DBG_USBBC1_ADDR),
	PMIC_REG_DEF(DBG_USBBC2_ADDR),
	PMIC_REG_DEF(DBG_USBBCSTAT_ADDR),
	PMIC_REG_DEF(USBPATH_ADDR),
	PMIC_REG_DEF(USBSRCDETSTATUS_ADDR),
	PMIC_REG_DEF(THRMBATZONE_ADDR_SC),
	PMIC_REG_DEF(THRMZN0L_ADDR_SC),
	PMIC_REG_DEF(THRMZN0H_ADDR_SC),
	PMIC_REG_DEF(THRMZN1L_ADDR_SC),
	PMIC_REG_DEF(THRMZN1H_ADDR_SC),
	PMIC_REG_DEF(THRMZN2L_ADDR_SC),
	PMIC_REG_DEF(THRMZN2H_ADDR_SC),
	PMIC_REG_DEF(THRMZN3L_ADDR_SC),
	PMIC_REG_DEF(THRMZN3H_ADDR_SC),
	PMIC_REG_DEF(THRMZN4L_ADDR_SC),
	PMIC_REG_DEF(THRMZN4H_ADDR_SC),
};

static struct pmic_regs_def pmic_tt_regs[] = {
	PMIC_REG_DEF(TT_I2CDADDR_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT0OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT1OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT2OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT3OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT4OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT5OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT6OS_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT7OS_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICCOS_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICCMASK_ADDR),
	PMIC_REG_DEF(TT_CHRCVOS_ADDR),
	PMIC_REG_DEF(TT_CHRCVMASK_ADDR),
	PMIC_REG_DEF(TT_CHRCCOS_ADDR),
	PMIC_REG_DEF(TT_CHRCCMASK_ADDR),
	PMIC_REG_DEF(TT_LOWCHROS_ADDR),
	PMIC_REG_DEF(TT_LOWCHRMASK_ADDR),
	PMIC_REG_DEF(TT_WDOGRSTOS_ADDR),
	PMIC_REG_DEF(TT_WDOGRSTMASK_ADDR),
	PMIC_REG_DEF(TT_CHGRENOS_ADDR),
	PMIC_REG_DEF(TT_CHGRENMASK_ADDR),
	PMIC_REG_DEF(TT_CUSTOMFIELDEN_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT0VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT1VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT2VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT3VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT4VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT5VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT6VAL_ADDR),
	PMIC_REG_DEF(TT_CHGRINIT7VAL_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICC100VAL_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICC150VAL_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICC500VAL_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICC900VAL_ADDR),
	PMIC_REG_DEF(TT_USBINPUTICC1500VAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVEMRGLOWVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVCOLDVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVCOOLVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVWARMVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVHOTVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCVEMRGHIVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCEMRGLOWVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCCOLDVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCCOOLVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCWARMVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCHOTVAL_ADDR),
	PMIC_REG_DEF(TT_CHRCCEMRGHIVAL_ADDR),
	PMIC_REG_DEF(TT_LOWCHRENVAL_ADDR),
	PMIC_REG_DEF(TT_LOWCHRDISVAL_ADDR),
};

void dump_pmic_regs(void)
{
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;
	u32 pmic_reg_cnt = 0;
	u32 reg_index;
	u8 data;
	int retval;
	struct pmic_regs_def *pmic_regs = NULL;

	if (vendor_id == BASINCOVE_VENDORID) {
		pmic_reg_cnt = ARRAY_SIZE(pmic_regs_bc);
		pmic_regs = pmic_regs_bc;
	} else if (vendor_id == SHADYCOVE_VENDORID) {
		pmic_reg_cnt = ARRAY_SIZE(pmic_regs_sc);
		pmic_regs = pmic_regs_sc;
	}

	dev_info(chc.dev, "PMIC Register dump\n");
	dev_info(chc.dev, "====================\n");

	for (reg_index = 0; reg_index < pmic_reg_cnt; reg_index++) {

		retval = intel_scu_ipc_ioread8(pmic_regs[reg_index].addr,
				&data);
		if (retval)
			dev_err(chc.dev, "Error in reading %x\n",
				pmic_regs[reg_index].addr);
		else
			dev_info(chc.dev, "0x%x=0x%x\n",
				pmic_regs[reg_index].addr, data);
	}
	dev_info(chc.dev, "====================\n");
}

void dump_pmic_tt_regs(void)
{
	u32 pmic_tt_reg_cnt = ARRAY_SIZE(pmic_tt_regs);
	u32 reg_index;
	u8 data;
	int retval;

	dev_info(chc.dev, "PMIC CHRGR TT dump\n");
	dev_info(chc.dev, "====================\n");

	for (reg_index = 0; reg_index < pmic_tt_reg_cnt; reg_index++) {

		retval = pmic_read_tt(pmic_tt_regs[reg_index].addr, &data);
		if (retval)
			dev_err(chc.dev, "Error in reading %x\n",
				pmic_tt_regs[reg_index].addr);
		else
			dev_info(chc.dev, "0x%x=0x%x\n",
				pmic_tt_regs[reg_index].addr, data);
	}

	dev_info(chc.dev, "====================\n");
}
static const struct file_operations pmic_chrgr_reg_fops = {
	.open = pmic_chrgr_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations pmic_chrgr_tt_reg_fops = {
	.open = pmic_chrgr_tt_reg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static const struct file_operations pmic_adc_fops = {
	.open = pmic_adc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release
};

static void pmic_debugfs_init(void)
{
	struct dentry *fentry;
	struct dentry *pmic_regs_dir;
	struct dentry *pmic_tt_regs_dir;

	u32 reg_index;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;
	u32 pmic_reg_cnt = 0;
	u32 pmic_tt_reg_cnt = ARRAY_SIZE(pmic_tt_regs);
	char name[PMIC_REG_NAME_LEN] = {0};
	struct pmic_regs_def *pmic_regs = NULL;

	if (vendor_id == BASINCOVE_VENDORID) {
		pmic_reg_cnt = ARRAY_SIZE(pmic_regs_bc);
		pmic_regs = pmic_regs_bc;
	} else if (vendor_id == SHADYCOVE_VENDORID) {
		pmic_reg_cnt = ARRAY_SIZE(pmic_regs_sc);
		pmic_regs = pmic_regs_sc;
	}

	/* Creating a directory under debug fs for charger */
	charger_debug_dir = debugfs_create_dir(DRIVER_NAME , NULL) ;
	if (charger_debug_dir == NULL)
		goto debugfs_root_exit;

	/* Create files for printing battery ID and pack temp */
	fentry = debugfs_create_file("battery_id",
				S_IRUGO,
				charger_debug_dir,
				(void *)GPADC_BATID,
				&pmic_adc_fops);

	fentry = debugfs_create_file("pack_temperature",
			S_IRUGO,
			charger_debug_dir,
			(void *)GPADC_BATTEMP0,
			&pmic_adc_fops);

	/* Create a directory for pmic charger registers */
	pmic_regs_dir = debugfs_create_dir("pmic_ccsm_regs",
			charger_debug_dir);

	if (pmic_regs_dir == NULL)
		goto debugfs_err_exit;

	for (reg_index = 0; reg_index < pmic_reg_cnt; reg_index++) {

		snprintf(name, PMIC_REG_NAME_LEN, "%s",
			pmic_regs[reg_index].reg_name);

		fentry = debugfs_create_file(name,
				S_IRUGO,
				pmic_regs_dir,
				&pmic_regs[reg_index].addr,
				&pmic_chrgr_reg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	/* Create a directory for pmic tt charger registers */
	pmic_tt_regs_dir = debugfs_create_dir("pmic_ccsm_tt_regs",
			charger_debug_dir);

	if (pmic_tt_regs_dir == NULL)
		goto debugfs_err_exit;

	for (reg_index = 0; reg_index < pmic_tt_reg_cnt; reg_index++) {

		snprintf(name, PMIC_REG_NAME_LEN, "%s",
			pmic_tt_regs[reg_index].reg_name);

		fentry = debugfs_create_file(name,
				S_IRUGO,
				pmic_tt_regs_dir,
				&pmic_tt_regs[reg_index].addr,
				&pmic_chrgr_tt_reg_fops);

		if (fentry == NULL)
			goto debugfs_err_exit;
	}

	dev_dbg(chc.dev, "Debugfs created successfully!!");
	return;

debugfs_err_exit:
	debugfs_remove_recursive(charger_debug_dir);
debugfs_root_exit:
	dev_err(chc.dev, "Error creating debugfs entry!!");
	return;
}

static void pmic_debugfs_exit(void)
{
	if (charger_debug_dir != NULL)
		debugfs_remove_recursive(charger_debug_dir);
}
#endif

static void pmic_get_bat_zone(int *bat_zone)
{
	u8 data = 0;
	u16 addr = 0;
	int vendor_id, ret;

	vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;
	if (vendor_id == BASINCOVE_VENDORID)
		addr = THRMBATZONE_ADDR_BC;
	else if (vendor_id == SHADYCOVE_VENDORID)
		addr = THRMBATZONE_ADDR_SC;

	ret = intel_scu_ipc_ioread8(addr, &data);
	if (ret) {
		dev_err(chc.dev, "Error:%d in reading battery zone\n", ret);
		/* Return undetermined zone in case of IPC failure */
		*bat_zone = PMIC_BZONE_UNKNOWN;
		return;
	}

	*bat_zone = (data & THRMBATZONE_MASK);
}

static void pmic_bat_zone_changed(void)
{
	int retval;
	int cur_zone, temp = 0;
	u16 addr = 0;
	u8 data = 0;
	struct power_supply *psy_bat;
	int vendor_id;

	pmic_get_bat_zone(&cur_zone);
	pmic_get_battery_pack_temp(&temp);
	dev_info(chc.dev, "Battery Zone changed. Current zone:%d, temp:%d\n",
			cur_zone, temp);

	/* if current zone is the top and bottom zones then report OVERHEAT
	 */
	if ((cur_zone == PMIC_BZONE_LOW) || (cur_zone == PMIC_BZONE_HIGH))
		chc.health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (cur_zone == PMIC_BZONE_UNKNOWN)
		chc.health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else
		chc.health = POWER_SUPPLY_HEALTH_GOOD;

	psy_bat = get_psy_battery();

	/* Trigger charging algo to get new parameters */
	power_supply_changed(psy_bat);

	if (psy_bat && psy_bat->external_power_changed)
		psy_bat->external_power_changed(psy_bat);

	return;
}

static void pmic_battery_overheat_handler(bool stat)
{
	if (stat)
		chc.health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else
		chc.health = POWER_SUPPLY_HEALTH_GOOD;
	return;
}

int pmic_get_health(void)
{
	return chc.health;
}

int pmic_enable_vbus(bool enable)
{
	int ret = 0;

	if (enable)
		ret = intel_scu_ipc_update_register(CHGRCTRL0_ADDR,
				WDT_NOKICK_ENABLE, CHGRCTRL0_WDT_NOKICK_MASK);
	else
		ret = intel_scu_ipc_update_register(CHGRCTRL0_ADDR,
				WDT_NOKICK_DISABLE, CHGRCTRL0_WDT_NOKICK_MASK);

	/* If access is blocked return success to avoid additional
	*  error handling at client side
	*/
	if (ret == -EACCES) {
		dev_warn(chc.dev, "IPC blocked due to unsigned kernel/invalid battery\n");
		ret = 0;
	}

	return ret;
}

int pmic_handle_otgmode(bool enable)
{
	int ret = 0;
	int vendor_id;

	vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	if (vendor_id != SHADYCOVE_VENDORID) {
		dev_err(chc.dev, "Ignore otg-mode event received\n");
		return 0;
	}

	if (enable)
		ret = intel_scu_ipc_update_register(CHGRCTRL1_ADDR,
				CHGRCTRL1_OTGMODE_MASK,
				CHGRCTRL1_OTGMODE_MASK);
	else
		ret = intel_scu_ipc_update_register(CHGRCTRL1_ADDR,
				0x0, CHGRCTRL1_OTGMODE_MASK);

	/* If access is blocked return success to avoid additional
	*  error handling at client side
	*/
	if (ret == -EACCES) {
		dev_warn(chc.dev, "IPC blocked due to unsigned kernel/invalid battery\n");
		ret = 0;
	}

	return ret;
}

int pmic_enable_charging(bool enable)
{
	int ret;
	u8 val;

	if (enable) {
		ret = intel_scu_ipc_update_register(CHGRCTRL1_ADDR,
			CHGRCTRL1_FTEMP_EVENT_MASK, CHGRCTRL1_FTEMP_EVENT_MASK);
		if (ret)
			return ret;
	}

	val = (enable) ? 0 : EXTCHRDIS_ENABLE;

	ret = intel_scu_ipc_update_register(CHGRCTRL0_ADDR,
			val, CHGRCTRL0_EXTCHRDIS_MASK);
	/* If access is blocked return success to avoid additional
	*  error handling at client side
	*/
	if (ret == -EACCES) {
		dev_warn(chc.dev, "IPC blocked due to unsigned kernel/invalid battery\n");
		ret = 0;
	}

	return ret;
}

static inline int update_zone_cc(int zone, u8 reg_val)
{
	u8 addr_cc = TT_CHRCCHOTVAL_ADDR - zone;
	dev_dbg(chc.dev, "%s:%X=%X\n", __func__, addr_cc, reg_val);
	return pmic_write_tt(addr_cc, reg_val);
}

static inline int update_zone_cv(int zone, u8 reg_val)
{
	u8 addr_cv = TT_CHRCVHOTVAL_ADDR - zone;
	dev_dbg(chc.dev, "%s:%X=%X\n", __func__, addr_cv, reg_val);
	return pmic_write_tt(addr_cv, reg_val);
}

/**
 * get_scove_tempzone_val - get tempzone register val for a particular zone
 * @adc_val: adc_value passed for zone temp
 * @temp: zone temperature
 *
 * Returns temp zone alert value
 */
static u16 get_scove_tempzone_val(u16 resi_val, int temp)
{
	u8 cursel = 0, hys = 0;
	u16 trsh = 0, count = 0, bsr_num = 0;
	u16 adc_thold = 0, tempzone_val = 0;
	s16 hyst = 0;
	int retval;

	/* multiply to convert into Ohm*/
	resi_val *= OHM_MULTIPLIER;

	/* CUR = max(floor(log2(round(ADCNORM/2^5)))-7,0)
	 * TRSH = round(ADCNORM/(2^(4+CUR)))
	 * HYS = if(∂ADCNORM>0 then max(round(∂ADCNORM/(2^(7+CUR))),1) else 0
	 */

	/*
	 * while calculating the CUR[2:0], instead of log2
	 * do a BSR (bit scan reverse) since we are dealing with integer values
	 */
	bsr_num = resi_val;
	bsr_num /= (1 << 5);

	while (bsr_num >>= 1)
		count++;

	/* cursel = max((count - 7), 0);
	 * Clamp cursel to 3-bit value
	 */
	cursel = clamp_t(s8, (count-7), 0, 7);

	/* calculate the TRSH[8:0] to be programmed */
	trsh = ((resi_val) / (1 << (4 + cursel)));

	/* calculate HYS[3:0] */
	/* add the temp hysteresis depending upon the zones */
	if (temp <= 0 || temp >= 60)
		temp += 1;
	else
		temp += 2;

	/* retrieve the resistance corresponding to temp with hysteresis */
	retval = CONVERT_TEMP_TO_ADC(temp, (int *)&adc_thold);
	if (unlikely(retval)) {
		dev_err(chc.dev,
			"Error converting temperature for zone\n");
		return retval;
	}

	/* multiply to convert into Ohm*/
	adc_thold *= OHM_MULTIPLIER;

	hyst = (resi_val - adc_thold);

	if (hyst > 0)
		hys = max((hyst / (1 << (7 + cursel))), 1);
	else
		hys = 0;
	/* Clamp hys to 4-bit value */
	hys = clamp_t(u8, hys, 0, 15);

	tempzone_val = (hys << 12) | (cursel << 9) | trsh;

	return tempzone_val;
}

static inline int update_zone_temp(int zone, u16 adc_val)
{
	int ret;
	u16 addr_tzone;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	if (vendor_id == BASINCOVE_VENDORID)
		addr_tzone = THRMZN4H_ADDR_BC - (2 * zone);
	else if (vendor_id == SHADYCOVE_VENDORID) {
		/* to take care of address-discontinuity of zone-registers */
		int offset_zone = zone;
		if (zone >= 3)
			offset_zone += 1;

		addr_tzone = THRMZN4H_ADDR_SC - (2 * offset_zone);
	} else {
		dev_err(chc.dev, "%s: invalid vendor id %X\n", __func__, vendor_id);
		return -EINVAL;
	}

	ret = intel_scu_ipc_iowrite8(addr_tzone, (u8)(adc_val >> 8));
	if (unlikely(ret))
		return ret;
	dev_dbg(chc.dev, "%s:%X:%X=%X\n", __func__, addr_tzone,
				(addr_tzone+1), adc_val);

	return intel_scu_ipc_iowrite8(addr_tzone+1, (u8)(adc_val & 0xFF));
}

int pmic_set_cc(int new_cc)
{
	struct ps_pse_mod_prof *bcprof = chc.actual_bcprof;
	struct ps_pse_mod_prof *r_bcprof = chc.runtime_bcprof;
	int temp_mon_ranges;
	int new_cc1;
	int ret;
	int i, cur_zone;
	u8 reg_val = 0;

	pmic_get_bat_zone(&cur_zone);
	dev_info(chc.dev, "%s: Battery Zone:%d\n", __func__, cur_zone);

	/* No need to write PMIC if CC = 0 */
	if (!new_cc)
		return 0;

	temp_mon_ranges = min_t(u16, bcprof->temp_mon_ranges,
			BATT_TEMP_NR_RNG);

	for (i = 0; i < temp_mon_ranges; ++i) {
		new_cc1 = min_t(int, new_cc,
				bcprof->temp_mon_range[i].full_chrg_cur);

		if (new_cc1 != r_bcprof->temp_mon_range[i].full_chrg_cur) {
			if (chc.pdata->cc_to_reg) {
				chc.pdata->cc_to_reg(new_cc1, &reg_val);
				ret = update_zone_cc(i, reg_val);
				if (unlikely(ret))
					return ret;
			}
			r_bcprof->temp_mon_range[i].full_chrg_cur = new_cc1;
		}
	}

	/* send the new CC and CV */
	intel_scu_ipc_update_register(CHGRCTRL1_ADDR,
		CHGRCTRL1_FTEMP_EVENT_MASK, CHGRCTRL1_FTEMP_EVENT_MASK);

	return 0;
}

int pmic_set_cv(int new_cv)
{
	struct ps_pse_mod_prof *bcprof = chc.actual_bcprof;
	struct ps_pse_mod_prof *r_bcprof = chc.runtime_bcprof;
	int temp_mon_ranges;
	int new_cv1;
	int ret;
	int i, cur_zone;
	u8 reg_val = 0;

	pmic_get_bat_zone(&cur_zone);
	dev_info(chc.dev, "%s: Battery Zone:%d\n", __func__, cur_zone);

	/* No need to write PMIC if CV = 0 */
	if (!new_cv)
		return 0;

	temp_mon_ranges = min_t(u16, bcprof->temp_mon_ranges,
			BATT_TEMP_NR_RNG);

	for (i = 0; i < temp_mon_ranges; ++i) {
		new_cv1 = min_t(int, new_cv,
				bcprof->temp_mon_range[i].full_chrg_vol);

		if (new_cv1 != r_bcprof->temp_mon_range[i].full_chrg_vol) {
			if (chc.pdata->cv_to_reg) {
				chc.pdata->cv_to_reg(new_cv1, &reg_val);
				ret = update_zone_cv(i, reg_val);
				if (unlikely(ret))
					return ret;
			}
			r_bcprof->temp_mon_range[i].full_chrg_vol = new_cv1;
		}
	}

	/* send the new CC and CV */
	intel_scu_ipc_update_register(CHGRCTRL1_ADDR,
		CHGRCTRL1_FTEMP_EVENT_MASK, CHGRCTRL1_FTEMP_EVENT_MASK);

	return 0;
}

int pmic_set_ilimma(int ilim_ma)
{
	u8 reg_val;
	int ret;

	if (ilim_ma >= 1500) {
		if (chc.pdata->inlmt_to_reg)
			chc.pdata->inlmt_to_reg(ilim_ma, &reg_val);

		ret = pmic_write_tt(TT_USBINPUTICC1500VAL_ADDR, reg_val);
		if (ret) {
			dev_err(chc.dev, "Error in updating TT-reg(%x): %d\n",
					TT_USBINPUTICC1500VAL_ADDR, ret);
			return ret;
		}
	}

	lookup_regval(pmic_inlmt, ARRAY_SIZE(pmic_inlmt),
			ilim_ma, &reg_val);
	dev_dbg(chc.dev, "Setting inlmt %d in register %x=%x\n", ilim_ma,
		CHGRCTRL1_ADDR, reg_val);
	ret = intel_scu_ipc_iowrite8(CHGRCTRL1_ADDR, reg_val);

	/* If access is blocked return success to avoid additional
	*  error handling at client side
	*/
	if (ret == -EACCES) {
		dev_warn(chc.dev, "IPC blocked due to unsigned kernel/invalid battery\n");
		ret = 0;
	}

	return ret;
}

static bool is_hvdcp_charging_enabled(int mask)
{
	int ret;
	u8 val;

	if (mask) {
		/* Enable VDPSRC so that it stays on when switch is closed */
		ret = intel_scu_ipc_update_register(DBG_USBBC2_ADDR,
				DBG_USBBC2_EN_VDPSRC_MASK,
				DBG_USBBC2_EN_VDPSRC_MASK);
		if (ret) {
			dev_err(chc.dev,
				"Error updating DBG_USBBC2-register 0x%3x\n",
				DBG_USBBC2_ADDR);
			return false;
		}

		/* Enable SW control of the USB charger type detection */
		ret = intel_scu_ipc_update_register(DBG_USBBC1_ADDR,
			DBG_USBBC1_SWCTRL_EN_MASK | DBG_USBBC1_EN_CMP_DM_MASK |
			DBG_USBBC1_EN_CMP_DP_MASK | DBG_USBBC1_EN_CHG_DET_MASK,
			DBG_USBBC1_SWCTRL_EN_MASK | DBG_USBBC1_EN_CMP_DM_MASK |
			DBG_USBBC1_EN_CMP_DP_MASK |
			DBG_USBBC1_EN_CHG_DET_MASK);
		if (ret) {
			dev_err(chc.dev,
				"Error updating DBG_USBBC1-register 0x%3x\n",
				DBG_USBBC1_ADDR);
			return false;
		}

		/* Wait >1.5 sec */
		msleep(HVDCPDET_SLEEP_TIME);

		/* Check if DM<VDATDET.  If a HVDCP is attached, it will remove
		 * the DP/DM short & enable Rdm_dwn, pulling down DM to 0V.
		 * DCP will keep DM at 0.6V
		 */
		ret = pmic_read_reg(DBG_USBBCSTAT_ADDR, &val);
		if (ret) {
			dev_err(chc.dev,
				"Error reading DBG_USBBCSTAT-register 0x%3x\n",
				DBG_USBBCSTAT_ADDR);
			return false;
		}

		/* If “0”, HVDCP detected */
		dev_info(chc.dev,
				"DBG_USBBCSTAT-register 0x%3x: %x\n",
				DBG_USBBCSTAT_ADDR, val);
		if (!(val & DBG_USBBCSTAT_CMP_DM_MASK)) {
			/* Enable HVDCP 12V charging  by enabling VDMSRC */
			ret = intel_scu_ipc_update_register(DBG_USBBC2_ADDR,
					DBG_USBBC2_EN_VDMSRC_MASK,
					DBG_USBBC2_EN_VDMSRC_MASK);
			if (ret) {
				dev_err(chc.dev,
				"Error updating DBG_USBBC2-register 0x%3x\n",
				DBG_USBBC2_ADDR);
			} else {
				/* HVDCP detection completed successfully */
				dev_info(chc.dev,
					"HVDCP 12V charging enabled\n");
				return true;
			}
		}
	}

	/* Cleanup on either HVDCP-not-detected or HVDCP-disconnected */
	dev_info(chc.dev, "HVDCP 12V charging disabled");
	/* Open PMIC switch */
	intel_scu_ipc_iowrite8(USBPHYCTRL_ADDR, 0x00);
	/* Disable VDMSRC & VDPSRC */
	intel_scu_ipc_iowrite8(DBG_USBBC2_ADDR, 0x00);
	/* Disable SW control of the USB charger type detection */
	intel_scu_ipc_iowrite8(DBG_USBBC1_ADDR, 0x00);

	return false;
}

static int scove_get_usbid(void)
{
	int ret;
	struct iio_channel *indio_chan;
	int rid, id = RID_UNKNOWN;
	u8 val;

	ret = pmic_read_reg(SCHGRIRQ1_ADDR, &val);
	if (ret) {
		dev_err(chc.dev,
			"Error reading SCHGRIRQ1-register 0x%2x\n",
			SCHGRIRQ1_ADDR);
		return ret;
	}

	/* SCHGRIRQ1_REG SUSBIDDET bit definition:
	 * 00 = RID_A/B/C ; 01 = RID_GND ; 10 = RID_FLOAT */
	if ((val & SCHRGRIRQ1_SUSBIDGNDDET_MASK) == SHRT_FLT_DET)
		return RID_FLOAT;
	else if ((val & SCHRGRIRQ1_SUSBIDGNDDET_MASK) == SHRT_GND_DET)
		return RID_GND;

	indio_chan = iio_channel_get(NULL, "USBID");
	if (IS_ERR_OR_NULL(indio_chan)) {
		dev_err(chc.dev, "Failed to get IIO channel USBID\n");
		ret = PTR_ERR(indio_chan);
		goto exit;
	}

	ret = iio_read_channel_raw(indio_chan, &rid);
	if (ret) {
		dev_err(chc.dev, "IIO channel read error for USBID\n");
		goto err_exit;
	}

	if ((rid > 11150) && (rid < 13640))
		id = RID_A;
	else if ((rid > 6120) && (rid < 7480))
		id = RID_B;
	else if ((rid > 3285) && (rid < 4015))
		id = RID_C;

err_exit:
	iio_channel_release(indio_chan);
exit:
	return id;
}

static int get_charger_type(void)
{
	int ret, i = 0;
	u8 val;
	int chgr_type, rid;

	do {
		ret = pmic_read_reg(USBSRCDETSTATUS_ADDR, &val);
		if (ret != 0) {
			dev_err(chc.dev,
				"Error reading USBSRCDETSTAT-register 0x%2x\n",
				USBSRCDETSTATUS_ADDR);
			return 0;
		}

		i++;
		dev_info(chc.dev, "Read USBSRCDETSTATUS val: %x\n", val);

		if (val & USBSRCDET_SUSBHWDET_DETSUCC)
			break;
		else
			msleep(USBSRCDET_SLEEP_TIME);
	} while (i < USBSRCDET_RETRY_CNT);

	if (!(val & USBSRCDET_SUSBHWDET_DETSUCC)) {
		dev_err(chc.dev, "Charger detection unsuccessful after %dms\n",
			i * USBSRCDET_SLEEP_TIME);
		return 0;
	}

	chgr_type = (val & USBSRCDET_USBSRCRSLT_MASK) >> 2;
	dev_info(chc.dev, "Charger type after detection complete: %d\n",
			(val & USBSRCDET_USBSRCRSLT_MASK) >> 2);

	rid = scove_get_usbid();
	if ((project == 8 || project == 10) && gpio_get_value(USBIN_ACOK_N) && !gpio_get_value(CHG_ACOK_N) && rid != RID_GND) {
		if (chc.is_eng_test_as){
			dev_info(chc.dev, "eng request: Usb status is SDP: 1\n");
			return POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
		} else {
			dev_info(chc.dev, "Charger type get false, Usb status is None: 0\n");
			return POWER_SUPPLY_CHARGER_TYPE_NONE;
		}
	}
	switch (chgr_type) {
	case PMIC_CHARGER_TYPE_SDP:
	case PMIC_CHARGER_TYPE_FLOAT_DP_DN:
		return POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
	case PMIC_CHARGER_TYPE_DCP:
		return POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
	case PMIC_CHARGER_TYPE_CDP:
		return POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
	case PMIC_CHARGER_TYPE_ACA:
		if (rid == RID_A)
			return POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK;
		else if (rid != RID_UNKNOWN)
			return POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
	case PMIC_CHARGER_TYPE_SE1:
		return POWER_SUPPLY_CHARGER_TYPE_SE1;
	case PMIC_CHARGER_TYPE_MHL:
		return POWER_SUPPLY_CHARGER_TYPE_MHL;
	default:
		return POWER_SUPPLY_CHARGER_TYPE_NONE;
	}
}

void trigger_charger_redetect()
{
	u8 val;
	int ret;

	dev_info(chc.dev, "Toggle USBPHYRSTB to force charger re-detection\n");

	ret = pmic_read_reg(USBPHYCTRL_ADDR, &val);
	if (ret != 0) {
		dev_err(chc.dev, "Error reading USBPHYCTRL-register 0x%02x\n", USBPHYCTRL_ADDR);
		return;
	}

	intel_scu_ipc_update_register(USBPHYCTRL_ADDR, USBPHYRSTB, USBPHYCTRL_USBPHYRSTB_MASK);

	do {
		ret = pmic_read_reg(USBPHYCTRL_ADDR, &val);
		if (ret != 0) {
			dev_err(chc.dev, "Error reading USBPHYCTRL-register 0x%02x\n", USBPHYCTRL_ADDR);
			return;
		}
	} while ((val & USBPHYRSTB) == 0);

	intel_scu_ipc_update_register(USBPHYCTRL_ADDR, 0x0, USBPHYCTRL_USBPHYRSTB_MASK);

	msleep(50);
	handle_acok_falling_interrupt();
}

static void handle_internal_usbphy_notifications(int mask)
{
	struct power_supply_cable_props cap = {0};
	bool hvdcp_chgr = false;

	if (mask) {
		cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cap.chrg_type = get_charger_type();
		chc.charger_type = cap.chrg_type;
	} else {
		cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		cap.chrg_type = chc.charger_type;
		chc.charger_type = get_charger_type();
	}

	if (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_ACA && chc.aca_oneshot == 0 && mask == 1) {
		dev_info(chc.dev, "get ACA type, re-detect type\n");
		chc.aca_oneshot = 1;
		trigger_charger_redetect();
		return;
	}

	if (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP)
		cap.ma = 0;
	else if ((cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_DCP)
			|| (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_CDP)
			|| (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_SE1)
			|| (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_ACA)
			|| (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK))
		cap.ma = 1500;

	dev_info(chc.dev, "Notifying OTG ev:%d, evt:%d, chrg_type:%d, mA:%d\n",
			USB_EVENT_CHARGER, cap.chrg_evt, cap.chrg_type,
			cap.ma);
	dev_info(chc.dev, "chc.chrg_type:%d", chc.charger_type);
	atomic_notifier_call_chain(&chc.otg->notifier,
			USB_EVENT_CHARGER, &cap);

	if (cap.chrg_type == POWER_SUPPLY_CHARGER_TYPE_USB_DCP && (project != 8 || project != 10)) {
		hvdcp_chgr = is_hvdcp_charging_enabled(mask);
		if (hvdcp_chgr && mask) {
			cap.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_UPDATE;
			cap.ma = 2000;

			dev_info(chc.dev, "Notifying OTG ev:%d, evt:%d,"
					" chrg_type:%d, mA:%d\n",
					USB_EVENT_CHARGER, cap.chrg_evt,
					cap.chrg_type, cap.ma);
			atomic_notifier_call_chain(&chc.otg->notifier,
					USB_EVENT_CHARGER, &cap);
		}
	}
}

/* ShadyCove-WA for VBUS removal detect issue */
int pmic_handle_low_supply(void)
{
	int ret;
	u8 val;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	dev_info(chc.dev, "Low-supply event received from external-charger\n");
	if (vendor_id == BASINCOVE_VENDORID || !chc.vbus_connect_status) {
		dev_err(chc.dev, "Ignore Low-supply event received\n");
		return 0;
	}

	msleep(200);
	ret = pmic_read_reg(SCHGRIRQ1_ADDR, &val);
	if (ret) {
		dev_err(chc.dev,
			"Error reading SCHGRIRQ1-register 0x%2x\n",
			SCHGRIRQ1_ADDR);
		return ret;
	}

	//if (!(val & SCHRGRIRQ1_SVBUSDET_MASK)) {
	int mask = 0;
	chc.aca_oneshot = 0;

	dev_info(chc.dev, "USB VBUS Removed. Notifying OTG driver\n");
	mutex_lock(&chc.evt_queue_lock);
	chc.vbus_connect_status = false;
	mutex_unlock(&chc.evt_queue_lock);

	if (chc.is_internal_usb_phy && !chc.otg_mode_enabled)
		handle_internal_usbphy_notifications(mask);
	else {
		atomic_notifier_call_chain(&chc.otg->notifier,
				USB_EVENT_VBUS, &mask);
		mutex_lock(&chc.evt_queue_lock);
		chc.otg_mode_enabled = false;
		mutex_unlock(&chc.evt_queue_lock);
	}
	//}

	return ret;
}

static void handle_level0_interrupt(u8 int_reg, u8 stat_reg,
				struct interrupt_info int_info[],
				int int_info_size)
{
	int i;
	bool int_stat;
	char *log_msg;

	for (i = 0; i < int_info_size; ++i) {

		/*continue if interrupt register bit is not set */
		if (!(int_reg & int_info[i].int_reg_mask))
			continue;

		/*log message if interrupt bit is set */
		if (int_info[i].log_msg_int_reg_true)
			dev_err(chc.dev, "%s",
					int_info[i].log_msg_int_reg_true);

		/* interrupt bit is set.call int handler. */
		if (int_info[i].int_handle)
			int_info[i].int_handle();

		/* continue if stat_reg_mask is zero which
		 *  means ignore status register
		 */
		if (!(int_info[i].stat_reg_mask))
			continue;

		dev_dbg(chc.dev,
				"stat_reg=%X int_info[i].stat_reg_mask=%X",
				stat_reg, int_info[i].stat_reg_mask);

		/* check if the interrupt status is true */
		int_stat = (stat_reg & int_info[i].stat_reg_mask);

		/* log message */
		log_msg = int_stat ? int_info[i].log_msg_stat_true :
			int_info[i].log_msg_stat_false;

		if (log_msg)
			dev_err(chc.dev, "%s", log_msg);

		/* call status handler function */
		if (int_info[i].stat_handle)
			int_info[i].stat_handle(int_stat);

	}

	return ;
}

static void handle_acok_falling_interrupt()
{
	int mask = 0;
	u8 val;
	int ret;

	if (!gpio_get_value(pmic_acok))
		mask = 1;

	if (mask) {
		 /* This is to handle the scenario where a connect event
		 * is received with already connect-status for USB.
		 * Previous disconnect event could have been missed due
		 * to synchronization issues of low-supply fault or
		 * late update to SCHRGRIRQ1_SVBUSDET
		 */
		if (chc.vbus_connect_status) {
			int rmv_mask = 0;
			dev_info(chc.dev, "Previous USB VBUS removal not received\n");
			dev_info(chc.dev, "USB VBUS removal forced. Notifying OTG driver\n");

			if (chc.is_internal_usb_phy && !chc.otg_mode_enabled)
				handle_internal_usbphy_notifications(rmv_mask);
			else {
				atomic_notifier_call_chain(&chc.otg->notifier, USB_EVENT_VBUS, &rmv_mask);
				chc.otg_mode_enabled = false;
			}
		}

		dev_info(chc.dev, "USB VBUS Detected. Notifying OTG driver\n");
		chc.vbus_connect_status = true;

		ret = pmic_read_reg(CHGRCTRL1_ADDR, &val);
		if (ret != 0) {
			dev_err(chc.dev, "Error reading CHGRCTRL1-register 0x%2x\n", CHGRCTRL1_ADDR);
			return;
		}

		if (val & CHGRCTRL1_OTGMODE_MASK)
			chc.otg_mode_enabled = true;
	} else {
		dev_info(chc.dev, "USB VBUS Removed. Notifying OTG driver\n");
		chc.vbus_connect_status = false;
	}

	pr_info("%s, pmic_gpio_ACOK = %d\n", __func__, gpio_get_value(pmic_acok)? 1 : 0);
	pr_info("%s, otg_mode_enabled = %d\n", __func__, chc.otg_mode_enabled? 1 : 0);
	pr_info("%s, mask = %d\n", __func__, mask);

	/* Avoid charger-detection flow in case of host-mode */
	if (chc.is_internal_usb_phy && !chc.otg_mode_enabled)
		handle_internal_usbphy_notifications(mask);
	else {
		atomic_notifier_call_chain(&chc.otg->notifier, USB_EVENT_VBUS, &mask);
		if (!mask)
			chc.otg_mode_enabled = false;
	}
}

static void handle_as_dock_in_interrupt(bool state) {
	int mask;
	u8 val;
	int ret;

	if (state) {
		/*dock plug in*/
		int rid = scove_get_usbid();
		if (rid == RID_GND) {
			pr_info("%s, AS Dock plug in but ID is grounded! Skip!\n", __func__);
		} else if (chc.charger_type == POWER_SUPPLY_CHARGER_TYPE_USB_SDP && !gpio_get_value(pmic_acok)) {
			pr_info("%s, AS Dock plug in but usb is in SDP mode! Skip!\n", __func__);
		} else if (!gpio_get_value(AS_DOCK_IN)) {
			if (!gpio_get_value(USB_PATH_SEL)) {
				if (chc.is_eng_test_as) {
				/*factory request SDP*/
					pr_info("%s, Test cable plug in. Notifying OTG driver!\n", __func__);
					gpio_direction_output(USB_PATH_SEL, 1);
					chc.vbus_connect_status = true;
					ret = pmic_read_reg(CHGRCTRL1_ADDR, &val);
					if (ret != 0) {
					        dev_err(chc.dev, "Error reading CHGRCTRL1-register 0x%2x\n", CHGRCTRL1_ADDR);
						gpio_direction_output(USB_PATH_SEL, 0);
						return;
					}
					mask = 1;
					handle_internal_usbphy_notifications(mask);
				} else {
				/*None, DCP or others*/
					pr_info("%s, AS Dock plug in. Notifying OTG driver switch to HOST mode!\n", __func__);
					gpio_direction_output(USB_PATH_SEL, 1);
					mask = 1;
					pmic_handle_otgmode(true);
					atomic_notifier_call_chain(&chc.otg->notifier,
						USB_EVENT_ID, &mask);
				}
			}
		} else {
		/*As Dock not exist*/
			pr_info("%s, As Dock not exist\n", __func__);
			gpio_direction_output(USB_PATH_SEL, 0);
		}
	} else {
		/*dock plug out*/
		if (gpio_get_value(USB_PATH_SEL)) {
			if (chc.is_eng_test_as) {
				/*factory request SDP*/
				pr_info("%s, Eng test cable out. Notifying OTG driver!\n", __func__);
				gpio_direction_output(USB_PATH_SEL, 0);
				mask = 0;
				handle_internal_usbphy_notifications(mask);
				chc.is_eng_test_as = false;
			} else {
				/*None, DCP or others*/
				pr_info("%s, AS Dock plug out. Notifying OTG driver!\n", __func__);
				mask = 0;
				pmic_handle_otgmode(false);
				atomic_notifier_call_chain(&chc.otg->notifier, USB_EVENT_ID, &mask);
				msleep(50);
				gpio_direction_output(USB_PATH_SEL, 0);
			}
		}
	}
	pr_info("%s, as_gpio_dock_in = %s\n", __func__, gpio_get_value(AS_DOCK_IN)? "H" : "L");
	pr_info("%s, usb_gpio_path_sel = %s\n", __func__, gpio_get_value(USB_PATH_SEL)? "H" : "L");
}

static void handle_level1_interrupt(u8 int_reg, u8 stat_reg)
{
	int mask;
	u8 val;
	int ret;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	if (!int_reg)
		return;

	if (vendor_id == SHADYCOVE_VENDORID) {
		if (int_reg & CHRGRIRQ1_SUSBIDFLTDET_MASK)
			dev_info(chc.dev,
				"USBID-FLT interrupt received\n");

		mask = ((stat_reg & SCHRGRIRQ1_SUSBIDGNDDET_MASK)
				== SHRT_GND_DET) ? 1 : 0;
		if (int_reg & CHRGRIRQ1_SUSBIDGNDDET_MASK) {
			if (mask) {
				dev_info(chc.dev,
				"USBID-GND Detected. Notifying OTG\n");
				//JEREMY1
				if ((project == 8 || project == 10) && gpio_get_value(USB_PATH_SEL)) {
					/*OTG plug in when AS dock exist*/
					gpio_direction_output(USB_PATH_SEL, 0);
				}
				pmic_handle_otgmode(true);
			} else {
				dev_info(chc.dev,
				"USBID-GND Removed. Notifying OTG\n");
				pmic_handle_otgmode(false);
			}

			atomic_notifier_call_chain(&chc.otg->notifier,
					USB_EVENT_ID, &mask);
		}
	}

#if 0
	mask = !!(int_reg & stat_reg);
	if ((vendor_id == BASINCOVE_VENDORID) &&
			(int_reg & CHRGRIRQ1_SUSBIDDET_MASK)) {
		if (mask)
			dev_info(chc.dev,
				"USB ID Detected. Notifying OTG driver\n");
		else
			dev_info(chc.dev,
				"USB ID Removed. Notifying OTG driver\n");

		atomic_notifier_call_chain(&chc.otg->notifier,
				USB_EVENT_ID, &mask);
	}
#endif
	return;
}

#ifdef PMIC_A0_WAR
static irqreturn_t pmic_gpio_acok_isr(int irq, void *dev_id)
{
	wake_lock_timeout(&chc.wakelock, ACOK_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(pmic_gpio_acok_wq, &chc.acok_irq_work, 0.5*HZ);

	return IRQ_HANDLED;
}

static void acok_irq_work_function(struct work_struct *work)
{
	printk(KERN_INFO "%s, pmic_gpio_ACOK = %s\n", __func__, gpio_get_value(pmic_acok)? "H":"L");

	if (gpio_get_value(pmic_acok))
		if ((project == 8 || project == 10) && !gpio_get_value(AS_DOCK_IN)) {
			/*OTG, SDP, DCP plug out when AS dock exist*/
			pmic_handle_low_supply();
			msleep(200);
			handle_as_dock_in_interrupt(true);
		} else
			pmic_handle_low_supply();
	else {
		/*SDP, DCP plug in when AS dock exist*/
		int rid = scove_get_usbid();
		if ((project == 8 || project == 10) && gpio_get_value(USB_PATH_SEL) && rid != RID_GND) {
			handle_as_dock_in_interrupt(false);
			msleep(300);
			handle_acok_falling_interrupt();
			msleep(300);
			handle_as_dock_in_interrupt(true);
		} else
			handle_acok_falling_interrupt();
	}
}

static int pmic_gpio_acok_init(struct platform_device *pdev)
{
	int err = 0 ;
	unsigned gpio = pmic_acok;
	unsigned irq_num = gpio_to_irq(gpio);

	pmic_gpio_acok_wq = create_singlethread_workqueue("pmic_gpio_acok_wq");
	INIT_DELAYED_WORK(&chc.acok_irq_work, acok_irq_work_function);

	err = gpio_request(gpio, "pmic_gpio_acok");
	if (err){
		printk(KERN_INFO"gpio %d request failed \n", gpio);
		goto err_gpio;
	}
	err = gpio_direction_input(gpio);
	if (err){
		printk(KERN_INFO"gpio %d unavaliable for input \n", gpio);
		goto err_free_gpio;
	}
	err = request_irq(irq_num, pmic_gpio_acok_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_SHARED,
		"pmic_gpio_acok", pdev);
	if (err < 0){
		printk(KERN_INFO "%s irq %d request failed \n","pmic_gpio_acok", irq_num);
		goto err_free_gpio;
	}
	printk(KERN_INFO "GPIO pin irq %d requested ok, pmic_acok = %s\n", irq_num, gpio_get_value(gpio)? "H":"L");
	return 0;

err_free_gpio:
	if (gpio_is_valid(gpio))
		gpio_free(gpio);;
err_gpio:
	return err;

}

static irqreturn_t as_gpio_dock_in_isr(int irq, void *dev_id)
{
	wake_lock_timeout(&chc.wakelock, ACOK_WAKE_LOCK_TIMEOUT);
	queue_delayed_work(as_gpio_dock_in_wq, &chc.as_irq_work, 0.5*HZ);

	return IRQ_HANDLED;
}

static void as_irq_work_function(struct work_struct *work)
{
	pr_info("%s, as_gpio_dock_in = %s\n", __func__, gpio_get_value(AS_DOCK_IN)? "H":"L");

	if (!gpio_get_value(AS_DOCK_IN))
		handle_as_dock_in_interrupt(true);
	else
		handle_as_dock_in_interrupt(false);
}

static int as_gpio_dock_in_init(struct platform_device *pdev)
{
	int err = 0;
	unsigned gpio = AS_DOCK_IN;
	unsigned irq_num = gpio_to_irq(gpio);

	as_gpio_dock_in_wq = create_singlethread_workqueue("as_gpio_dock_in_wq");
	INIT_DELAYED_WORK(&chc.as_irq_work, as_irq_work_function);

	err = gpio_request(AS_DOCK_IN, "as_gpio_dock_in");
	if (err && err != -EBUSY) {
		printk(KERN_INFO"gpio %d request failed err:%d\n", AS_DOCK_IN, err);
		goto err_gpio_as;
	}
	err = gpio_request(USB_PATH_SEL, "usb_gpio_path_sel");
	if (err && err != -EBUSY) {
		printk(KERN_INFO"gpio %d request failed err:%d\n", USB_PATH_SEL, err);
		goto err_free_gpio_as;
	}
	err = gpio_request(CHG_ACOK_N, "chg_gpio_acok");
	if (err && err != -EBUSY) {
		printk(KERN_INFO"gpio %d request failed err:%d\n", CHG_ACOK_N, err);
		goto err_free_gpio_as;
	}

	err = gpio_direction_input(AS_DOCK_IN);
	if (err) {
		printk(KERN_INFO"gpio %d unavaliable for input err:%d\n", AS_DOCK_IN, err);
		goto err_free_gpio_as;
	}
	err = gpio_direction_output(USB_PATH_SEL, 0);
	if (err) {
		printk(KERN_INFO"gpio %d unavaliable for output err:%d\n", USB_PATH_SEL, err);
		goto err_free_gpio_as;
	}
	err = gpio_direction_input(CHG_ACOK_N);
	if (err) {
		printk(KERN_INFO"gpio %d unavaliable for input err:%d\n", CHG_ACOK_N, err);
		goto err_free_gpio_as;
	}

	err = request_irq(irq_num, as_gpio_dock_in_isr, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_SHARED,
		"as_gpio_dock_in", pdev);
	if (err < 0) {
		printk(KERN_INFO "%s irq %d request failed, err:%d \n","as_gpio_dock_in", irq_num, err);
		goto err_free_gpio_as;
	}

	printk(KERN_INFO "GPIO pin irq %d requested ok, AS_DOCK_IN = %s\n", irq_num, gpio_get_value(AS_DOCK_IN)? "H":"L");
	printk(KERN_INFO "GPIO pin requested ok, USB_PATH_SEL = %s\n", gpio_get_value(USB_PATH_SEL)? "H":"L");
	printk(KERN_INFO "GPIO pin requested ok, CHG_ACOK_N  = %s\n", gpio_get_value(CHG_ACOK_N)? "H":"L");
	return 0;

err_free_gpio_as:
	if (gpio_is_valid(AS_DOCK_IN))
		gpio_free(AS_DOCK_IN);
	if (gpio_is_valid(USB_PATH_SEL))
		gpio_free(USB_PATH_SEL);
	if (gpio_is_valid(CHG_ACOK_N))
		gpio_free(CHG_ACOK_N);
err_gpio_as:
	return err;
}

static ssize_t req_as_test_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
        return sprintf(buf, "req_as_test = %d\n", chc.is_eng_test_as);
}

static ssize_t req_as_test_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t size)
{
        int value;
        sscanf(buf, "%d", &value);
	dev_info(chc.dev,"req_as_test = %d\n", value);
	if (value == 0 || value == 1)
		chc.is_eng_test_as = value;
        return size;
}
static DEVICE_ATTR(req_as_test, 0666, req_as_test_show, req_as_test_store);

#endif

static void pmic_event_worker(struct work_struct *work)
{
	struct pmic_event *evt, *tmp;

	dev_dbg(chc.dev, "%s\n", __func__);

	mutex_lock(&chc.evt_queue_lock);
	list_for_each_entry_safe(evt, tmp, &chc.evt_queue, node) {
		list_del(&evt->node);

		dev_info(chc.dev, "CHGRIRQ0=%X SCHGRIRQ0=%X CHGRIRQ1=%x SCHGRIRQ1=%X\n",
				evt->chgrirq0_int, evt->chgrirq0_stat,
				evt->chgrirq1_int, evt->chgrirq1_stat);
		if (evt->chgrirq0_int)
			handle_level0_interrupt(evt->chgrirq0_int,
				evt->chgrirq0_stat, chgrirq0_info,
				ARRAY_SIZE(chgrirq0_info));

		if (evt->chgrirq1_stat)
			handle_level1_interrupt(evt->chgrirq1_int,
							evt->chgrirq1_stat);
		kfree(evt);
	}

	mutex_unlock(&chc.evt_queue_lock);
}

static irqreturn_t pmic_isr(int irq, void *data)
{
	u16 pmic_intr;
	u8 chgrirq0_int;
	u8 chgrirq1_int;
	u8 mask = 0;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	if (vendor_id == BASINCOVE_VENDORID)
		mask = ((CHRGRIRQ1_SVBUSDET_MASK) |
				(CHRGRIRQ1_SUSBIDDET_MASK));
	else if (vendor_id == SHADYCOVE_VENDORID)
		mask = ((CHRGRIRQ1_SVBUSDET_MASK) |
				(CHRGRIRQ1_SUSBIDFLTDET_MASK) |
				(CHRGRIRQ1_SUSBIDGNDDET_MASK));

	pmic_intr = ioread16(chc.pmic_intr_iomap);
	chgrirq0_int = (u8)pmic_intr;
	chgrirq1_int = (u8)(pmic_intr >> 8);

	if (!chgrirq1_int && !(chgrirq0_int & PMIC_CHRGR_INT0_MASK))
		return IRQ_NONE;

	if ((chgrirq1_int & mask) && (!wake_lock_active(&chc.wakelock))) {
		/*
		Setting the Usb wake lock hold timeout to a safe value of 5s.
		*/
		wake_lock_timeout(&chc.wakelock, USB_WAKE_LOCK_TIMEOUT);
	}

	dev_dbg(chc.dev, "%s", __func__);

	return IRQ_WAKE_THREAD;
}
static irqreturn_t pmic_thread_handler(int id, void *data)
{
	u16 pmic_intr;
	struct pmic_event *evt;
	int ret;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (evt == NULL) {
		dev_dbg(chc.dev, "Error allocating evt structure in fn:%s\n",
			__func__);
		return IRQ_NONE;
	}

	pmic_intr = ioread16(chc.pmic_intr_iomap);
	evt->chgrirq0_int = (u8)pmic_intr;
	evt->chgrirq1_int = (u8)(pmic_intr >> 8);
	dev_dbg(chc.dev, "irq0=%x irq1=%x\n",
		evt->chgrirq0_int, evt->chgrirq1_int);

	/*
	In case this is an external charger interrupt, we are
	clearing the level 1 irq register and let external charger
	driver handle the interrupt.
	 */

	if (!(evt->chgrirq1_int) &&
		!(evt->chgrirq0_int & PMIC_CHRGR_CCSM_INT0_MASK)) {
		intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
				IRQLVL1_CHRGR_MASK);
		if ((chc.invalid_batt) &&
			(evt->chgrirq0_int & PMIC_CHRGR_EXT_CHRGR_INT_MASK)) {
			dev_dbg(chc.dev, "Handling external charger interrupt!!\n");
			kfree(evt);
			return IRQ_HANDLED;
		}
		kfree(evt);
		dev_dbg(chc.dev, "Unhandled interrupt!!\n");
		return IRQ_NONE;
	}

	if (evt->chgrirq0_int & PMIC_CHRGR_CCSM_INT0_MASK) {
		ret = intel_scu_ipc_ioread8(SCHGRIRQ0_ADDR,
				&evt->chgrirq0_stat);
		if (ret) {
			dev_err(chc.dev,
				"%s: Error(%d) in intel_scu_ipc_ioread8. Faile to read SCHGRIRQ0_ADDR\n",
					__func__, ret);
			kfree(evt);
			goto end;
		}
	}
	if (evt->chgrirq1_int) {
		ret = intel_scu_ipc_ioread8(SCHGRIRQ1_ADDR,
				&evt->chgrirq1_stat);
		if (ret) {
			dev_err(chc.dev,
				"%s: Error(%d) in intel_scu_ipc_ioread8. Faile to read SCHGRIRQ1_ADDR\n",
					__func__, ret);
			kfree(evt);
			goto end;
		}
	}

	INIT_LIST_HEAD(&evt->node);

	mutex_lock(&chc.evt_queue_lock);
	list_add_tail(&evt->node, &chc.evt_queue);
	mutex_unlock(&chc.evt_queue_lock);

	queue_work(system_nrt_wq, &chc.evt_work);

end:
	/*clear first level IRQ */
	dev_dbg(chc.dev, "Clearing IRQLVL1_MASK_ADDR\n");
	intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
			IRQLVL1_CHRGR_MASK);

	return IRQ_HANDLED;
}

static int pmic_init(void)
{
	int ret = 0, i, temp_mon_ranges;
	u16 adc_val;
	u8 reg_val;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;
	struct ps_pse_mod_prof *bcprof = chc.actual_bcprof;

	temp_mon_ranges = min_t(u16, bcprof->temp_mon_ranges,
			BATT_TEMP_NR_RNG);
	for (i = 0; i < temp_mon_ranges; ++i) {
		ret =
		CONVERT_TEMP_TO_ADC(bcprof->temp_mon_range[i].temp_up_lim,
				(int *)&adc_val);
		if (unlikely(ret)) {
			dev_err(chc.dev,
				"Error converting temperature for zone %d!!\n",
				i);
			return ret;
		}

		if (vendor_id == SHADYCOVE_VENDORID) {
			/* Values obtained from lookup-table are resistance values.
			 * Convert these to raw adc-codes
			 */
			adc_val = get_scove_tempzone_val(adc_val,
					bcprof->temp_mon_range[i].temp_up_lim);
			dev_info(chc.dev,
					"adc-val:%x configured for temp:%d\n",
					adc_val,
					bcprof->temp_mon_range[i].temp_up_lim);
		}

		ret = update_zone_temp(i, adc_val);
		if (unlikely(ret)) {
			dev_err(chc.dev,
				"Error updating zone temp for zone %d\n",
				i);
			return ret;
		}

		if (chc.pdata->cc_to_reg)
			chc.pdata->cc_to_reg(bcprof->temp_mon_range[i].
					full_chrg_cur, &reg_val);

		ret = update_zone_cc(i, reg_val);
		if (unlikely(ret)) {
			dev_err(chc.dev,
				"Error updating zone cc for zone %d\n",
				i);
			return ret;
		}

		if (chc.pdata->cv_to_reg)
			chc.pdata->cv_to_reg(bcprof->temp_mon_range[i].
					full_chrg_vol, &reg_val);

		ret = update_zone_cv(i, reg_val);
		if (unlikely(ret)) {
			dev_err(chc.dev,
				"Error updating zone cv for zone %d\n",
				i);
			return ret;
		}

		/* Write lowest temp limit */
		if (i == (bcprof->temp_mon_ranges - 1)) {
			ret = CONVERT_TEMP_TO_ADC(bcprof->temp_low_lim,
							(int *)&adc_val);
			if (unlikely(ret)) {
				dev_err(chc.dev,
					"Error converting low lim temp!!\n");
				return ret;
			}

			if (vendor_id == SHADYCOVE_VENDORID) {
				adc_val = get_scove_tempzone_val(adc_val,
						bcprof->temp_low_lim);
				dev_info(chc.dev,
					"adc-val:%x configured for temp:%d\n",
					adc_val, bcprof->temp_low_lim);
			}

			ret = update_zone_temp(i+1, adc_val);

			if (unlikely(ret)) {
				dev_err(chc.dev,
					"Error updating last temp for zone %d\n",
					i+1);
				return ret;
			}
		}
	}
	ret = pmic_update_tt(TT_CUSTOMFIELDEN_ADDR,
				TT_HOT_COLD_LC_MASK,
				TT_HOT_COLD_LC_DIS);

	if (unlikely(ret)) {
		dev_err(chc.dev, "Error updating TT_CUSTOMFIELD_EN reg\n");
		return ret;
	}

	if (chc.pdata->inlmt_to_reg)
		chc.pdata->inlmt_to_reg(USBINPUTICC100VAL, &reg_val);

	ret = pmic_write_tt(TT_USBINPUTICC100VAL_ADDR, reg_val);
	return ret;
}

static inline void print_ps_pse_mod_prof(struct ps_pse_mod_prof *bcprof)
{
	int i, temp_mon_ranges;

	dev_info(chc.dev, "ChrgProf: batt_id:%s\n", bcprof->batt_id);
	dev_info(chc.dev, "ChrgProf: battery_type:%u\n", bcprof->battery_type);
	dev_info(chc.dev, "ChrgProf: capacity:%u\n", bcprof->capacity);
	dev_info(chc.dev, "ChrgProf: voltage_max:%u\n", bcprof->voltage_max);
	dev_info(chc.dev, "ChrgProf: chrg_term_ma:%u\n", bcprof->chrg_term_ma);
	dev_info(chc.dev, "ChrgProf: low_batt_mV:%u\n", bcprof->low_batt_mV);
	dev_info(chc.dev, "ChrgProf: disch_tmp_ul:%d\n", bcprof->disch_tmp_ul);
	dev_info(chc.dev, "ChrgProf: disch_tmp_ll:%d\n", bcprof->disch_tmp_ll);
	dev_info(chc.dev, "ChrgProf: temp_mon_ranges:%u\n",
			bcprof->temp_mon_ranges);
	temp_mon_ranges = min_t(u16, bcprof->temp_mon_ranges,
			BATT_TEMP_NR_RNG);

	for (i = 0; i < temp_mon_ranges; ++i) {
		dev_info(chc.dev, "ChrgProf: temp_up_lim[%d]:%d\n",
				i, bcprof->temp_mon_range[i].temp_up_lim);
		dev_info(chc.dev, "ChrgProf: full_chrg_vol[%d]:%d\n",
				i, bcprof->temp_mon_range[i].full_chrg_vol);
		dev_info(chc.dev, "ChrgProf: full_chrg_cur[%d]:%d\n",
				i, bcprof->temp_mon_range[i].full_chrg_cur);
		dev_info(chc.dev, "ChrgProf: maint_chrgr_vol_ll[%d]:%d\n",
				i, bcprof->temp_mon_range[i].maint_chrg_vol_ll);
		dev_info(chc.dev, "ChrgProf: maint_chrgr_vol_ul[%d]:%d\n",
				i, bcprof->temp_mon_range[i].maint_chrg_vol_ul);
		dev_info(chc.dev, "ChrgProf: maint_chrg_cur[%d]:%d\n",
				i, bcprof->temp_mon_range[i].maint_chrg_cur);
	}
	dev_info(chc.dev, "ChrgProf: temp_low_lim:%d\n", bcprof->temp_low_lim);
}

static int find_tempzone_index(short int *interval,
				int *num_zones,
				short int *temp_up_lim)
{
	struct ps_pse_mod_prof *bprof = chc.sfi_bcprof->batt_prof;
	int up_lim_index = 0, low_lim_index = -1;
	int diff = 0;
	int i;

	*num_zones = MIN_BATT_PROF - bprof->temp_mon_ranges + 1;
	if ((*num_zones) <= 0)
		return 0;

	for (i = 0 ; i < bprof->temp_mon_ranges ; i++) {
		if (bprof->temp_mon_range[i].temp_up_lim == BATT_TEMP_WARM)
			up_lim_index = i;
	}

	low_lim_index = up_lim_index + 1;

	if (low_lim_index == bprof->temp_mon_ranges)
		diff = bprof->temp_low_lim -
			bprof->temp_mon_range[up_lim_index].temp_up_lim;
	else
		diff = bprof->temp_mon_range[low_lim_index].temp_up_lim -
			bprof->temp_mon_range[up_lim_index].temp_up_lim;

	*interval = diff / (*num_zones);
	*temp_up_lim = bprof->temp_mon_range[up_lim_index].temp_up_lim;

	return up_lim_index;
}


static void set_pmic_batt_prof(struct ps_pse_mod_prof *new_prof,
				struct ps_pse_mod_prof *bprof)
{
	int num_zones;
	int split_index;
	int i, j = 0;
	short int temp_up_lim;
	short int interval;

	if ((new_prof == NULL) || (bprof == NULL))
		return;

	if (!NEED_ZONE_SPLIT(bprof)) {
		dev_info(chc.dev, "No need to split the zones!!\n");
		memcpy(new_prof, bprof, sizeof(struct ps_pse_mod_prof));
		return;
	}

	strcpy(&(new_prof->batt_id[0]), &(bprof->batt_id[0]));
	new_prof->battery_type = bprof->battery_type;
	new_prof->capacity = bprof->capacity;
	new_prof->voltage_max =  bprof->voltage_max;
	new_prof->chrg_term_ma = bprof->chrg_term_ma;
	new_prof->low_batt_mV =  bprof->low_batt_mV;
	new_prof->disch_tmp_ul = bprof->disch_tmp_ul;
	new_prof->disch_tmp_ll = bprof->disch_tmp_ll;

	split_index = find_tempzone_index(&interval, &num_zones, &temp_up_lim);

	for (i = 0 ; i < bprof->temp_mon_ranges; i++) {
		if ((i == split_index) && (num_zones > 0)) {
			for (j = 0; j < num_zones; j++,
					temp_up_lim += interval) {
				memcpy(&new_prof->temp_mon_range[i+j],
					&bprof->temp_mon_range[i],
					sizeof(bprof->temp_mon_range[i]));
				new_prof->temp_mon_range[i+j].temp_up_lim =
					temp_up_lim;
			}
			j--;
		} else {
			memcpy(&new_prof->temp_mon_range[i+j],
				&bprof->temp_mon_range[i],
				sizeof(bprof->temp_mon_range[i]));
		}
	}

	new_prof->temp_mon_ranges = i+j;
	new_prof->temp_low_lim = bprof->temp_low_lim;

	return;
}


static int pmic_check_initial_events(void)
{
	struct pmic_event *evt;
	int ret;
	u8 mask = (CHRGRIRQ1_SVBUSDET_MASK), chgrirq1_stat;
	int vendor_id = chc.pmic_id & PMIC_VENDOR_ID_MASK;

	evt = kzalloc(sizeof(struct pmic_event), GFP_KERNEL);
	if (evt == NULL) {
		dev_dbg(chc.dev, "Error allocating evt structure in fn:%s\n",
			__func__);
		return -1;
	}

	ret = intel_scu_ipc_ioread8(SCHGRIRQ0_ADDR, &evt->chgrirq0_stat);
	evt->chgrirq0_int = evt->chgrirq0_stat;
	ret = intel_scu_ipc_ioread8(SCHGRIRQ1_ADDR, &chgrirq1_stat);
	evt->chgrirq1_stat = chgrirq1_stat;
	evt->chgrirq1_int = chgrirq1_stat;

	/* For ShadyCove, CHGRIRQ1_REG & SCHGRIRQ1_REG cannot be directly
	 * mapped. If status has (01 = Short to ground detected), it means
	 * USBIDGNDDET should be handled. If status has (10 = Floating pin
	 * detected), it means USBIDFLTDET should be handled.
	 */
	if (vendor_id == SHADYCOVE_VENDORID) {
		if ((chgrirq1_stat & SCHRGRIRQ1_SUSBIDGNDDET_MASK)
				== SHRT_FLT_DET) {
			evt->chgrirq1_int |= CHRGRIRQ1_SUSBIDFLTDET_MASK;
			evt->chgrirq1_int &= ~CHRGRIRQ1_SUSBIDGNDDET_MASK;
		} else if ((chgrirq1_stat & SCHRGRIRQ1_SUSBIDGNDDET_MASK)
				== SHRT_GND_DET)
			evt->chgrirq1_int |= CHRGRIRQ1_SUSBIDGNDDET_MASK;
	}

	if (chgrirq1_stat || evt->chgrirq0_int) {
		INIT_LIST_HEAD(&evt->node);
		mutex_lock(&chc.evt_queue_lock);
		list_add_tail(&evt->node, &chc.evt_queue);
		mutex_unlock(&chc.evt_queue_lock);
		schedule_work(&chc.evt_work);
	}

	if ((chgrirq1_stat & mask) && !wake_lock_active(&chc.wakelock)) {
		/*
		Setting the Usb wake lock hold timeout to a safe value of 5s.
		*/
		wake_lock_timeout(&chc.wakelock, USB_WAKE_LOCK_TIMEOUT);
	}

	pmic_bat_zone_changed();

	if (!gpio_get_value(pmic_acok))
		queue_delayed_work(pmic_gpio_acok_wq, &chc.acok_irq_work, 0.5*HZ);

	if ((project == 8 || project == 10) && !gpio_get_value(AS_DOCK_IN))
		queue_delayed_work(as_gpio_dock_in_wq, &chc.as_irq_work, 0.5*HZ);

	return ret;
}

static ssize_t attr_led_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 ctrl, fsm, mode;
	int ret;
	ssize_t len = 0;

	ret = intel_scu_ipc_ioread8(CHRLEDCTRL_ADDR, &ctrl);
	if (ret)
		return -EIO;

	ret = intel_scu_ipc_ioread8(CHRLEDFSM_ADDR, &fsm);
	if (ret)
		return -EIO;

	/* report mode based on CHRLEDFN, SWLEDON, LEDFSMDIS and LEDEFF */
	if (ctrl & CHRLEDFN_SW_CONTROL) {
		if (ctrl & SWLEDON_ENABLE) {
			if ((fsm & LEDFSMDIS_DISABLED) == 0) {
				u8 mode = fsm & CHRLEDFSM_LEDEFF_MASK;

				if (mode == LEDEFF_OFF) {
					len = sprintf(buf, "off\n");
				} else if (mode == LEDEFF_ON) {
					len = sprintf(buf, "on\n");
				} else if (mode == LEDEFF_BLINK) {
					len = sprintf(buf, "blink\n");
				} else if (mode == LEDEFF_BREATHE) {
					len = sprintf(buf, "breathe\n");
				}
			} else {
				len = sprintf(buf, "manual\n");
			}
		} else {
			len = sprintf(buf, "off\n");
		}
	} else {
		len = sprintf(buf, "auto\n");
	}

	return len;
}

static ssize_t attr_led_ctrl_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;

	if (!strncmp(buf, "on", 2)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL | SWLEDON_ENABLE,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;

		ret = intel_scu_ipc_update_register(CHRLEDFSM_ADDR,
				LEDEFF_ON,
				CHRLEDFSM_LEDFSMDIS_MASK | CHRLEDFSM_LEDEFF_MASK);
		if (ret)
			return -EIO;
	} else if (!strncmp(buf, "off", 3)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;
	} else if (!strncmp(buf, "auto", 4)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				0, CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;
	} else if (!strncmp(buf, "blink", 5)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL | SWLEDON_ENABLE,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;

		ret = intel_scu_ipc_update_register(CHRLEDFSM_ADDR,
				LEDEFF_BLINK,
				CHRLEDFSM_LEDFSMDIS_MASK | CHRLEDFSM_LEDEFF_MASK);
		if (ret)
			return -EIO;
	} else if (!strncmp(buf, "manual", 6)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL | SWLEDON_ENABLE,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;

		ret = intel_scu_ipc_update_register(CHRLEDFSM_ADDR,
				LEDFSMDIS_DISABLED,
				CHRLEDFSM_LEDFSMDIS_MASK | CHRLEDFSM_LEDEFF_MASK);
		if (ret)
			return -EIO;
	} else if (!strncmp(buf, "breathe", 7)) {
		ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL | SWLEDON_ENABLE,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (ret)
			return -EIO;

		ret = intel_scu_ipc_update_register(CHRLEDFSM_ADDR,
				LEDEFF_BREATHE,
				CHRLEDFSM_LEDFSMDIS_MASK | CHRLEDFSM_LEDEFF_MASK);
		if (ret)
			return -EIO;
	} else {
		return -EINVAL;
	}

	return size;
}

static ssize_t attr_led_duty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data;
	int ret;

	ret = intel_scu_ipc_ioread8(CHRLEDPWM_ADDR, &data);
	if (ret)
		return -EIO;

	return sprintf(buf, "%u\n", data);
}

static ssize_t attr_led_duty_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	int ret;
	unsigned value;

	ret = sscanf(buf, "%u", &value);
	if (ret != 1)
		return -EINVAL;

	if (value > 255)
		value = 255;

	ret = intel_scu_ipc_iowrite8(CHRLEDPWM_ADDR, value);
	if (ret)
		return -EIO;

	return size;
}

static ssize_t attr_led_freq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 data, mode;
	ssize_t len = 0;
	int ret;

	ret = intel_scu_ipc_ioread8(CHRLEDCTRL_ADDR, &data);
	if (ret)
		return -EIO;

	mode = data & CHRLEDCTRL_CHRLEDF_MASK;

	if (mode == CHRLEDF_QUARTER_HZ) {
		len = sprintf(buf, "quarter\n");
	} else if (mode == CHRLEDF_HALF_HZ) {
		len = sprintf(buf, "half\n");
	} else if (mode == CHRLEDF_ONE_HZ) {
		len = sprintf(buf, "one\n");
	} else if (mode == CHRLEDF_TWO_HZ) {
		len = sprintf(buf, "two\n");
	}

	return len;
}

static ssize_t attr_led_freq_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	u8 mode;
	int ret;

	if (!strncmp(buf, "quarter", 7)) {
		mode = CHRLEDF_QUARTER_HZ;
	} else if (!strncmp(buf, "half", 4)) {
		mode = CHRLEDF_HALF_HZ;
	} else if (!strncmp(buf, "one", 3)) {
		mode = CHRLEDF_ONE_HZ;
	} else if (!strncmp(buf, "two", 3)) {
		mode = CHRLEDF_TWO_HZ;
	} else {
		return -EINVAL;
	}

	ret = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
			mode, CHRLEDCTRL_CHRLEDF_MASK);
	if (ret)
		return -EIO;

	return size;
}

static DEVICE_ATTR(led_ctrl, S_IRUSR|S_IWUSR, attr_led_ctrl_show, attr_led_ctrl_store);
static DEVICE_ATTR(led_duty, S_IRUSR|S_IWUSR, attr_led_duty_show, attr_led_duty_store);
static DEVICE_ATTR(led_freq, S_IRUSR|S_IWUSR, attr_led_freq_show, attr_led_freq_store);

static struct attribute *pmic_ccsm_attrs[] = {
	&dev_attr_led_ctrl.attr,
	&dev_attr_led_duty.attr,
	&dev_attr_led_freq.attr,
	NULL,
};

static const struct attribute_group pmic_ccsm_attr_group = {
	.attrs = pmic_ccsm_attrs,
};

/**
 * pmic_charger_probe - PMIC charger probe function
 * @pdev: pmic platform device structure
 * Context: can sleep
 *
 * pmic charger driver initializes its internal data
 * structure and other  infrastructure components for it
 * to work as expected.
 */
static int pmic_chrgr_probe(struct platform_device *pdev)
{
	int retval = 0;
	u8 val;

	if (!pdev)
		return -ENODEV;

	project = asustek_get_project_id();
	chc.health = POWER_SUPPLY_HEALTH_UNKNOWN;
	chc.aca_oneshot = 0;
	chc.dev = &pdev->dev;
	chc.irq = platform_get_irq(pdev, 0);
	chc.pdata = pdev->dev.platform_data;
	platform_set_drvdata(pdev, &chc);

	if (chc.pdata == NULL) {
		dev_err(chc.dev, "Platform data not initialized\n");
		return -EFAULT;
	}

	retval = intel_scu_ipc_ioread8(PMIC_ID_ADDR, &chc.pmic_id);
	if (retval) {
		dev_err(chc.dev,
			"Error reading PMIC ID register\n");
		return retval;
	}

	dev_info(chc.dev, "PMIC-ID: %x\n", chc.pmic_id);
	if ((chc.pmic_id & PMIC_VENDOR_ID_MASK) == SHADYCOVE_VENDORID) {
		retval = pmic_read_reg(USBPATH_ADDR, &val);
		if (retval) {
			dev_err(chc.dev,
				"Error reading CHGRSTATUS-register 0x%2x\n",
				CHGRSTATUS_ADDR);
			return retval;
		}

		if (val & USBPATH_USBSEL_MASK) {
			dev_info(chc.dev, "SOC-Internal-USBPHY used\n");
			chc.is_internal_usb_phy = true;
		} else
			dev_info(chc.dev, "External-USBPHY used\n");
	}

	chc.sfi_bcprof = kzalloc(sizeof(struct ps_batt_chg_prof),
				GFP_KERNEL);
	if (chc.sfi_bcprof == NULL) {
		dev_err(chc.dev,
			"Error allocating memeory SFI battery profile\n");
		return -ENOMEM;
	}

	retval = get_batt_prop(chc.sfi_bcprof);
	if (retval) {
		dev_err(chc.dev,
			"Error reading battery profile from battid frmwrk\n");
		kfree(chc.sfi_bcprof);
		chc.invalid_batt = true;
		chc.sfi_bcprof = NULL;
	}

	retval = intel_scu_ipc_update_register(CHGRCTRL0_ADDR, SWCONTROL_ENABLE | CHGRCTRL0_CCSM_OFF_MASK,
			CHGRCTRL0_SWCONTROL_MASK | CHGRCTRL0_CCSM_OFF_MASK);
	if (retval)
		dev_err(chc.dev, "Error enabling sw control. Charging may continue in h/w control mode\n");

	if (!chc.invalid_batt) {
		chc.actual_bcprof = kzalloc(sizeof(struct ps_pse_mod_prof),
					GFP_KERNEL);
		if (chc.actual_bcprof == NULL) {
			dev_err(chc.dev,
				"Error allocating mem for local battery profile\n");
			kfree(chc.sfi_bcprof);
			return -ENOMEM;
		}

		chc.runtime_bcprof = kzalloc(sizeof(struct ps_pse_mod_prof),
					GFP_KERNEL);
		if (chc.runtime_bcprof == NULL) {
			dev_err(chc.dev,
			"Error allocating mem for runtime batt profile\n");
			kfree(chc.sfi_bcprof);
			kfree(chc.actual_bcprof);
			return -ENOMEM;
		}

		set_pmic_batt_prof(chc.actual_bcprof,
				chc.sfi_bcprof->batt_prof);
		print_ps_pse_mod_prof(chc.actual_bcprof);
		retval = pmic_init();
		if (retval)
			dev_err(chc.dev, "Error in Initializing PMIC. Continue in h/w charging mode\n");

		memcpy(chc.runtime_bcprof, chc.actual_bcprof,
			sizeof(struct ps_pse_mod_prof));
	}

	chc.pmic_intr_iomap = ioremap_nocache(PMIC_SRAM_INTR_ADDR, 8);
	if (!chc.pmic_intr_iomap) {
		dev_err(&pdev->dev, "ioremap Failed\n");
		retval = -ENOMEM;
		goto ioremap_failed;
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
	chc.otg = usb_get_transceiver();
#else
	chc.otg = usb_get_phy(USB_PHY_TYPE_USB2);
#endif
	if (!chc.otg || IS_ERR(chc.otg)) {
		dev_err(&pdev->dev, "Failed to get otg transceiver!!\n");
		retval = -ENOMEM;
		goto otg_req_failed;
	}

	INIT_WORK(&chc.evt_work, pmic_event_worker);
	INIT_LIST_HEAD(&chc.evt_queue);
	mutex_init(&chc.evt_queue_lock);
	wake_lock_init(&chc.wakelock, WAKE_LOCK_SUSPEND, "pmic_wakelock");

	/* register interrupt */
	retval = request_threaded_irq(chc.irq, pmic_isr,
			pmic_thread_handler,
			IRQF_SHARED|IRQF_NO_SUSPEND ,
			DRIVER_NAME, &chc);
	if (retval) {
		dev_err(&pdev->dev,
			"Error in request_threaded_irq(irq(%d)!!\n",
			chc.irq);
		goto otg_req_failed;
	}

#ifdef PMIC_A0_WAR
	if (project == 8 || project == 10) {
		pmic_acok = USBIN_ACOK_N;
		as_gpio_dock_in_init(pdev);
		device_create_file(&pdev->dev, &dev_attr_req_as_test);
		chc.is_eng_test_as = false;
	} else
		pmic_acok = CHG_ACOK_N;
	pmic_gpio_acok_init(pdev);
#endif

	retval = pmic_check_initial_events();
	if (unlikely(retval)) {
		dev_err(&pdev->dev,
			"Error posting initial events\n");
		goto req_irq_failed;
	}

	/* unmask charger interrupts in second level IRQ register*/
	retval = intel_scu_ipc_update_register(MCHGRIRQ0_ADDR, 0x00,
			PMIC_CHRGR_INT0_MASK);
	/* unmask charger interrupts in second level IRQ register*/
	retval = intel_scu_ipc_iowrite8(MCHGRIRQ1_ADDR, 0x00);
	if (unlikely(retval))
		goto unmask_irq_failed;


	/* unmask IRQLVL1 register */
	retval = intel_scu_ipc_update_register(IRQLVL1_MASK_ADDR, 0x00,
			IRQLVL1_CHRGR_MASK);
	if (unlikely(retval))
		goto unmask_irq_failed;

	retval = intel_scu_ipc_update_register(USBIDCTRL_ADDR,
			 ACADETEN_MASK | USBIDEN_MASK,
			ACADETEN_MASK | USBIDEN_MASK);
	if (unlikely(retval))
		goto unmask_irq_failed;

	chc.health = POWER_SUPPLY_HEALTH_GOOD;
#ifdef CONFIG_DEBUG_FS
	pmic_debugfs_init();
#endif

	retval = sysfs_create_group(&pdev->dev.kobj, &pmic_ccsm_attr_group);
	if (unlikely(retval)) {
		dev_err(&pdev->dev, "Failed to create sysfs group: %d\n", retval);
		goto sysfs_create_failed;
	}
	if ((INTEL_MID_BOARD(2, PHONE, MRFL, BTNS, PRO) || INTEL_MID_BOARD(2, PHONE, MRFL, BTNS, ENG))) {
		/* turn on the PMIC power LED */
		retval = intel_scu_ipc_update_register(CHRLEDCTRL_ADDR,
				CHRLEDFN_SW_CONTROL | SWLEDON_ENABLE,
				CHRLEDCTRL_SWLEDON_MASK | CHRLEDCTRL_CHRLEDFN_MASK);
		if (unlikely(retval))
			goto set_led_failed;

		retval = intel_scu_ipc_update_register(CHRLEDFSM_ADDR,
				LEDEFF_ON,
				CHRLEDFSM_LEDFSMDIS_MASK | CHRLEDFSM_LEDEFF_MASK);
		if (unlikely(retval))
			goto set_led_failed;

		return 0;
	} else {
		return 0;
	}
set_led_failed:
	sysfs_remove_group(&pdev->dev.kobj, &pmic_ccsm_attr_group);
sysfs_create_failed:
unmask_irq_failed:
req_irq_failed:
	free_irq(chc.irq, &chc);
otg_req_failed:
	iounmap(chc.pmic_intr_iomap);
ioremap_failed:
	kfree(chc.sfi_bcprof);
	kfree(chc.actual_bcprof);
	kfree(chc.runtime_bcprof);
	return retval;
}

static void pmic_chrgr_do_exit_ops(struct pmic_chrgr_drv_context *chc)
{
	/*TODO:
	 * If charger is connected send IPC message to SCU to continue charging
	 */
#ifdef CONFIG_DEBUG_FS
	pmic_debugfs_exit();
#endif
}

/**
 * pmic_charger_remove - PMIC Charger driver remove
 * @pdev: PMIC charger platform device structure
 * Context: can sleep
 *
 * PMIC charger finalizes its internal data structure and other
 * infrastructure components that it initialized in
 * pmic_chrgr_probe.
 */
static int pmic_chrgr_remove(struct platform_device *pdev)
{
	struct pmic_chrgr_drv_context *chc = platform_get_drvdata(pdev);

	if (chc) {
		pmic_chrgr_do_exit_ops(chc);
		wake_lock_destroy(&chc->wakelock);
		free_irq(chc->irq, chc);
		iounmap(chc->pmic_intr_iomap);
		kfree(chc->sfi_bcprof);
		kfree(chc->actual_bcprof);
		kfree(chc->runtime_bcprof);
	}

	gpio_free(pmic_acok);

	return 0;
}

#ifdef CONFIG_PM
static int pmic_chrgr_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int pmic_chrgr_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#endif

#ifdef CONFIG_PM_RUNTIME
static int pmic_chrgr_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int pmic_chrgr_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int pmic_chrgr_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}
#endif

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

static const struct dev_pm_ops pmic_chrgr_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pmic_chrgr_suspend,
				pmic_chrgr_resume)
	SET_RUNTIME_PM_OPS(pmic_chrgr_runtime_suspend,
				pmic_chrgr_runtime_resume,
				pmic_chrgr_runtime_idle)
};

static struct platform_driver pmic_chrgr_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .pm = &pmic_chrgr_pm_ops,
		   },
	.probe = pmic_chrgr_probe,
	.remove = pmic_chrgr_remove,
};

static int pmic_chrgr_init(void)
{
	return platform_driver_register(&pmic_chrgr_driver);
}

static void pmic_chrgr_exit(void)
{
	platform_driver_unregister(&pmic_chrgr_driver);
}

static int pmic_ccsm_rpmsg_probe(struct rpmsg_channel *rpdev)
{
	int ret = 0;

	if (rpdev == NULL) {
		pr_err("rpmsg channel not created\n");
		ret = -ENODEV;
		goto out;
	}

	dev_info(&rpdev->dev, "Probed pmic_ccsm rpmsg device\n");

	ret = pmic_chrgr_init();

out:
	return ret;
}

static void pmic_ccsm_rpmsg_remove(struct rpmsg_channel *rpdev)
{
	pmic_chrgr_exit();
	dev_info(&rpdev->dev, "Removed pmic_ccsm rpmsg device\n");
}

static void pmic_ccsm_rpmsg_cb(struct rpmsg_channel *rpdev, void *data,
					int len, void *priv, u32 src)
{
	dev_warn(&rpdev->dev, "unexpected, message\n");

	print_hex_dump(KERN_DEBUG, __func__, DUMP_PREFIX_NONE, 16, 1,
		       data, len,  true);
}

static struct rpmsg_device_id pmic_ccsm_rpmsg_id_table[] = {
	{ .name	= "rpmsg_pmic_ccsm" },
	{ },
};
MODULE_DEVICE_TABLE(rpmsg, pmic_ccsm_rpmsg_id_table);

static struct rpmsg_driver pmic_ccsm_rpmsg = {
	.drv.name	= KBUILD_MODNAME,
	.drv.owner	= THIS_MODULE,
	.id_table	= pmic_ccsm_rpmsg_id_table,
	.probe		= pmic_ccsm_rpmsg_probe,
	.callback	= pmic_ccsm_rpmsg_cb,
	.remove		= pmic_ccsm_rpmsg_remove,
};

static int __init pmic_ccsm_rpmsg_init(void)
{
	return register_rpmsg_driver(&pmic_ccsm_rpmsg);
}

static void __exit pmic_ccsm_rpmsg_exit(void)
{
	return unregister_rpmsg_driver(&pmic_ccsm_rpmsg);
}
/* Defer init call so that dependant drivers will be loaded. Using  async
 * for parallel driver initialization */
late_initcall(pmic_ccsm_rpmsg_init);
module_exit(pmic_ccsm_rpmsg_exit);

MODULE_AUTHOR("Jenny TC <jenny.tc@intel.com>");
MODULE_DESCRIPTION("PMIC Charger  Driver");
MODULE_LICENSE("GPL");
