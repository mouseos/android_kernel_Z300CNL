/*
 * rt5648_ioctl.h  --  RT5648 ALSA SoC audio driver IO control
 *
 * Copyright 2012 Realtek Microelectronics
 * Author: Bard <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/spi/spi.h>
#include <sound/soc.h>
#include "rt_codec_ioctl.h"
#include "rt5648_ioctl.h"
#include "rt5648.h"

const hweq_t hweq_param[] = {
	{/* NORMAL */
		{0},
		0x0000,
	},
	{/* CLUB */
		{0x1bbc, 0x0c73, 0x030b, 0xff24, 0x1ea6, 0xe4d9, 0x1c98, 0x589d,
		 0x01bd, 0xd344, 0x01ca, 0x2940, 0xd8cb, 0x1bbc, 0x0000, 0xef01,
		 0x1bbc, 0x0000, 0xef01, 0x1bbc, 0x0000, 0x0257, 0x0000, 0x1cc9,
		 0x02eb, 0x1cee, 0x0800, 0x0800},
		0x0082,
	},
	{/* SPK */
		{0x1c10, 0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4,
		 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4,
		 0x2000, 0x0000, 0x2000},
		0x0000,
	},
	{/* HP */
		{0x1c10, 0x01f4, 0xc5e9, 0x1a98, 0x1d2c, 0xc882, 0x1c10, 0x01f4,
		 0xe904, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4,
		 0x2000, 0x0000, 0x2000},
		0x0000,
	},
	{/* SPK L hpf 200 hz only */
		{0x1c10, 0x01f4, 0x03dd, 0x1884, 0x1e18, 0x8fbd,
		 0x1c14, 0x6a69, 0x01f6, 0x1424, 0x01f8, 0x9bba,
		 0xc882, 0x1f10, 0x01f4, 0xe904, 0x1c10, 0x01f4,
		 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1f68,
		 0x0094, 0x1f69, 0x0800, 0x0800},
		0x0040,
	},
	{/* SPK R hpf 200 hz only*/
		{0x1c10, 0x01f4, 0x03d9, 0x1f84, 0x1e20, 0x4e03,
		 0x1c17, 0x2079, 0x01ef, 0x8cea, 0x01f7, 0xf168,
		 0xc882, 0x1c10, 0x01f4, 0xe904, 0x1c10, 0x01f4,
		 0xe904, 0x1c10, 0x01f4, 0x1c10, 0x01f4, 0x1f68,
		 0x0094, 0x1f69, 0x0800, 0x0800},
		0x0040,
	},
	{/* Z300CL SPK, LR symmetric. */
		{0x1c10, 0x0000, 0x0379, 0x54a4, 0x1e3d, 0x9ac3,
		 0x1c86, 0xab5d, 0x01c2, 0x9ac2, 0x0200, 0x0000,
		 0xd588, 0x1c10, 0x0000, 0xe904, 0x1c10, 0x0000,
		 0x0000, 0x1c10, 0x0000, 0x0436, 0x0000, 0x1fa5,
		 0x0059, 0x1fa5, 0x0800, 0x0800},
		0x0040,
	},
	{/* Z300CL MIC, on-board mic and headset mic shared. */
		{0x1c10, 0x0000, 0xc883, 0x1c10, 0x0000, 0xd824,
		 0x12c1, 0xf405, 0xced6, 0x1df8, 0x0000, 0x0000,
		 0x1cd0, 0x0c73, 0x1ec4, 0x0000, 0x0800, 0x0800},
		0x00a4,
	},
};
#define RT5648_HWEQ_LEN ARRAY_SIZE(hweq_param)

const int eqreg[EQ_CH_NUM][EQ_REG_NUM] = {
	/* Register for DACL */
	{0xa4, 0xa5, 0xeb, 0xec, 0xed, 0xee,
	 0xe7, 0xe8, 0xe9, 0xea, 0xe5, 0xe6,
	 0xae, 0xaf, 0xb0, 0xb4, 0xb5, 0xb6,
	 0xba, 0xbb, 0xbc, 0xc0, 0xc1, 0xc4,
	 0xc5, 0xc6, 0xca, 0xcc},
	/* Register for DACR */
	{0xa6, 0xa7, 0xf5, 0xf6, 0xf7, 0xf8,
	 0xf1, 0xf2, 0xf3, 0xf4, 0xef, 0xf0,
	 0xb1, 0xb2, 0xb3, 0xb7, 0xb8, 0xb9,
	 0xbd, 0xbe, 0xbf, 0xc2, 0xc3, 0xc7,
	 0xc8, 0xc9, 0xcb, 0xcd},
	/* Register for ADC */
	{0xce, 0xcf, 0xd0, 0xd1, 0xd2, 0xd3,
	 0xd4, 0xd5, 0xd6, 0xd7, 0xd8, 0xd9,
	 0xda, 0xdb, 0xdc, 0xdd, 0xe1, 0xe2,
	 /* Redundant ADC EQ register index */
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

int rt5648_update_eqmode(
	struct snd_soc_codec *codec, int channel, int mode)
{
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
	int i, upd_reg, reg, mask;

	if (codec == NULL ||  mode >= RT5648_HWEQ_LEN)
		return -EINVAL;

	dev_dbg(codec->dev, "%s(): mode=%d\n", __func__, mode);
	if (mode != NORMAL) {
		const int *regs = eqreg[channel];
		const int *vals = hweq_param[mode].value;
		for (i = 0; i < EQ_REG_NUM; i++) {
			if (regs[i])
				ioctl_ops->index_write(codec, regs[i], vals[i]);
			else
				break;
		}
	}
	switch (channel) {
	case EQ_CH_DACL:
		reg = RT5648_EQ_CTRL2;
		mask = 0x11fe;
		upd_reg = RT5648_EQ_CTRL1;
		break;
	case EQ_CH_DACR:
		reg = RT5648_EQ_CTRL2;
		mask = 0x22fe;
		upd_reg = RT5648_EQ_CTRL1;
		break;
	case EQ_CH_ADC:
		reg = RT5648_ADC_EQ_CTRL2;
		mask = 0x01bf;
		upd_reg = RT5648_ADC_EQ_CTRL1;
		break;
	default:
		printk(KERN_ERR "Invalid EQ channel\n");
		return -EINVAL;
	}
	snd_soc_update_bits(codec, reg, mask, hweq_param[mode].ctrl);
	snd_soc_update_bits(codec, upd_reg,
		RT5648_EQ_UPD, RT5648_EQ_UPD);
	snd_soc_update_bits(codec, upd_reg, RT5648_EQ_UPD, 0);

	return 0;
}

int rt5648_ioctl_common(struct snd_hwdep *hw, struct file *file,
			unsigned int cmd, unsigned long arg)
{
	struct snd_soc_codec *codec = hw->private_data;
	struct rt_codec_cmd __user *_rt_codec = (struct rt_codec_cmd *)arg;
	struct rt_codec_cmd rt_codec;
	/* struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops(); */
	int *buf;
	static int eq_mode[EQ_CH_NUM];

	if (copy_from_user(&rt_codec, _rt_codec, sizeof(rt_codec))) {
		dev_err(codec->dev, "copy_from_user faild\n");
		return -EFAULT;
	}
	dev_dbg(codec->dev, "%s(): rt_codec.number=%d, cmd=%d\n",
			__func__, rt_codec.number, cmd);
	buf = kmalloc(sizeof(*buf) * rt_codec.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt_codec.buf, sizeof(*buf) * rt_codec.number))
		goto err;

	switch (cmd) {
	case RT_SET_CODEC_HWEQ_IOCTL:
		if (eq_mode == *buf)
			break;
		eq_mode[*buf] = *(buf + 1);
		rt5648_update_eqmode(codec, eq_mode[*buf], *buf);
		break;

	case RT_GET_CODEC_ID:
		*buf = snd_soc_read(codec, RT5648_VENDOR_ID2);
		if (copy_to_user(rt_codec.buf, buf, sizeof(*buf) * rt_codec.number))
			goto err;
		break;
	default:
		break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;
}
EXPORT_SYMBOL_GPL(rt5648_ioctl_common);
