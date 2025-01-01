/*
 *  Headset device detection driver.
 *
 * Copyright (C) 2011 ASUSTek Corporation.
 *
 * Authors:
 *  Jason Cheng <jason4_cheng@asus.com>
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
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/switch.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include "rt5648.h"

#define TAG "HEADSET"

/*----------------------------------------------------------------------------
 ** FUNCTION DECLARATION
 **----------------------------------------------------------------------------*/
static int detection_work(void);
static int headset_get_jack_status(void);
static int headset_get_hook_status(void);
static int headset_disable_button_irq(void);
int hs_micbias_power(int on);
int hs_micbias_switch(int on);
int hs_get_status(void);
bool hs_need_micbias(void);
int hs_resume(void);
int hs_suspend(void);

/*----------------------------------------------------------------------------
 ** GLOBAL VARIABLES
 **----------------------------------------------------------------------------*/
enum {
	HS_JACK = 0,
};

struct snd_soc_jack_gpio hs_gpio[] = {
	[HS_JACK] = {
		.name			= "JACK_DET_AND_HOOK_DET",
		.gpio			= 14,
		.report			= SND_JACK_HEADSET |
					  SND_JACK_HEADPHONE |
					  SND_JACK_LINEOUT |
					  SND_JACK_BTN_0,
		.debounce_time		= 0,
		.jack_status_check	= detection_work,
		.irq_flags		= IRQF_TRIGGER_FALLING |
					    IRQF_TRIGGER_RISING,
	},
};
EXPORT_SYMBOL(hs_gpio);

static struct snd_soc_jack_gpio *jack_gpio;

enum {
	OFF,
	ON,
};

struct button_work {
	struct delayed_work work;
	int key;
};

struct headset_data {
	struct workqueue_struct *btn_wq;
	struct switch_dev sdev;
	struct snd_soc_jack *jack;
	struct button_work btn_works[8];
	struct delayed_work jack_work;
	struct delayed_work btn_enable_work;

	bool enable_detect;
	bool detecting;
	bool enable_btn;
};
static struct headset_data hs_data;
const int NUM_BUTTON_WORKS = sizeof(hs_data.btn_works) / sizeof(hs_data.btn_works[0]);


extern struct snd_soc_codec *audio_codec;

int hs_get_status(void)
{
	const struct snd_soc_jack *jack = jack_gpio[HS_JACK].jack;
	if (jack)
		return (jack->status & SND_JACK_HEADSET);
	return 0;
}

bool hs_need_micbias(void) {
	return (hs_get_status() == SND_JACK_HEADSET) || (hs_data.detecting);
}
EXPORT_SYMBOL(hs_need_micbias);

void cancel_all_works()
{
	int i;
	/* possible panic when suspending */
	/* drain_workqueue not working?? */
	for (i = 0; i < NUM_BUTTON_WORKS; ++i)
		cancel_delayed_work_sync(hs_data.btn_works + i);
	cancel_delayed_work_sync(&hs_data.jack_work);
}

int hs_suspend(void)
{
	struct snd_soc_jack *jack = jack_gpio[HS_JACK].jack;
	hs_data.enable_detect = false;
	hs_data.enable_btn = false;
	cancel_all_works();

	hs_data.jack = jack;
	jack->status = 0;
	headset_disable_button_irq();
	snd_soc_jack_free_gpios(jack, 1, &jack_gpio[HS_JACK]);

	return 0;
}
EXPORT_SYMBOL(hs_suspend);

int hs_resume(void)
{
	int ret = 0;

	ret = snd_soc_jack_add_gpios(hs_data.jack, 1, &jack_gpio[HS_JACK]);

	if (ret)
		pr_err("%s: add jack gpio failed\n", TAG);
	hs_data.enable_detect = true;
	detection_work();

	return ret;

}
EXPORT_SYMBOL(hs_resume);

static int headset_get_jack_status(void)
{
	return rt5648_get_gpio_value(RT5648_GPIO_JD1_1);
}

static int headset_get_hook_status(void)
{
	return rt5648_get_gpio_value(RT5648_GPIO_6);
}

static int headset_enable_button_irq(void)
{
	int ret = 0;

	ret = rt5648_enable_jd1_irq();
	ret = rt5648_set_jd1_irq_source(RT5648_GPIO_6);

	return ret;
}

static int headset_disable_button_irq(void)
{
	int ret = 0;

	ret = rt5648_disable_jd1_irq();
	ret = rt5648_set_jd1_irq_source(0);

	return ret;
}

static ssize_t headset_name_show(struct switch_dev *sdev, char *buf)
{
	const char* device = "NO DEVICE";
	switch (hs_get_status()) {
	case SND_JACK_HEADSET:
		device = "HEADSET";
		break;
	case SND_JACK_HEADPHONE:
		device = "HEADPHONE";
	}
	return sprintf(buf, "%s\n", device);
}

static ssize_t headset_state_show(struct switch_dev *sdev, char *buf)
{
	int device = 0;
	switch (hs_get_status()) {
	case SND_JACK_HEADSET:
		device = 1;
		break;
	case SND_JACK_HEADPHONE:
		device = 2;
	}
	return sprintf(buf, "%d\n", device);
}

int hs_micbias_switch(int status)
{
	int ret = -1;
	unsigned bits;
	struct snd_soc_codec *codec = audio_codec;

	if (codec == NULL)
		goto failed;

	pr_debug("%s: Turn %s micbias power\n", TAG,
			(status == ON) ? "on" : "off");

	if (status == OFF &&
			codec->dapm.bias_level != SND_SOC_BIAS_OFF) {
		pr_debug("%s: Codec turned it on, leave as was.\n", TAG);
		return 1;
	}

	bits = RT5648_PWR_BG | RT5648_PWR_VREF2 |
		RT5648_PWR_VREF1 | RT5648_PWR_MB;
	ret = snd_soc_update_bits(codec, RT5648_PWR_ANLG1, bits,
			status == ON ? bits : 0);
	if (ret < 0)
		goto failed;

	bits = RT5648_PWR_MB1;
	ret = snd_soc_update_bits(codec, RT5648_PWR_ANLG2, bits,
			status == ON ? bits : 0);
	if (ret < 0)
		goto failed;
	return ret;
failed:
	pr_err("%s: Codec update failure!\n", TAG);
	return ret;
}
EXPORT_SYMBOL(hs_micbias_switch);

static bool is_btn_available(int status)
{
	int jk_status = headset_get_jack_status();

	pr_debug("%s: jk_status 0x%x, status 0x%x enable_btn %d\n",
			TAG, jk_status, status, hs_data.enable_btn);
	return hs_data.enable_detect && hs_data.enable_btn && jk_status == 0 && ((status & SND_JACK_HEADSET) == SND_JACK_HEADSET);
}

static void button_work(void)
{
	const int TIME_TO_REPORT = 800;
	struct snd_soc_jack *jack = hs_gpio[HS_JACK].jack;
	int status = jack->status;
	static int next_bw_idx = 0;
	struct button_work*bw = NULL;
	/* Check for headset status before processing interrupt */

	if (is_btn_available(status)) {
		bw = hs_data.btn_works + next_bw_idx;
		next_bw_idx = (next_bw_idx + 1) % NUM_BUTTON_WORKS;
		bw->key = headset_get_hook_status();
		queue_delayed_work(hs_data.btn_wq, &(bw->work),
				msecs_to_jiffies(TIME_TO_REPORT));
	} else {
		pr_debug("Not a case for button.\n");
	}
}

static void button_work_late(struct work_struct*w)
{
	struct snd_soc_jack *jack = hs_gpio[HS_JACK].jack;
	int status = jack->status;
	struct delayed_work* dw = container_of(w, struct delayed_work, work);
	struct button_work* bw = container_of(dw, struct button_work, work);

	if (is_btn_available(status)) {
		if (bw->key) { // set btn bit.
			status |= SND_JACK_BTN_0;
		} else { // clear btn bit.
			status &= ~SND_JACK_BTN_0;
		}
		if (status != jack->status) {
			snd_soc_jack_report(jack, status, SND_JACK_BTN_0);
			pr_info("%s: %s\n", TAG, (status & SND_JACK_BTN_0)?"Pressed":"Released");
		}
	}
}

static int hot_retry(int (func)(void))
{
	const int INTERVAL = 8;
	const int EXPIRED = 1024;
	const int MAX_CONFIRM = 20;

	int status, temp;
	int confirm = 0;
	int time_elapsed = 0;

	status = func();
	do {
		msleep(INTERVAL);
		time_elapsed += INTERVAL;
		temp = func();
		if (status != temp) {
			confirm = 0;
			status = temp;
		} else {
			++confirm;
		}
	} while ((time_elapsed < EXPIRED) && (confirm < MAX_CONFIRM));

	pr_debug("%s: val %d given in %d with confidence %d\n",
			TAG, status, time_elapsed, confirm);
	return status;
}

static void jack_work_late(struct work_struct*w)
{
	int status = 0, i, jk_status;
	struct snd_soc_jack *jack = hs_gpio[HS_JACK].jack;

	status = hs_get_status();
	jk_status = hot_retry(headset_get_jack_status);
	switch (jk_status) {
	case 0:
		if (!status) {
			if (hot_retry(headset_get_hook_status)) {
				pr_info("%s: headphone\n", TAG);
				status = SND_JACK_HEADPHONE;
			} else {
				pr_info("%s: headset\n", TAG);
				status = SND_JACK_HEADSET;
			}
		}
		break;
	case 1:
		pr_info("%s: remove headset\n", TAG);
	default:
		status = 0;
		break;
	}
	hs_data.detecting = false;

	if (status != hs_get_status())
		snd_soc_jack_report(jack, status,
				SND_JACK_HEADSET | SND_JACK_HEADPHONE);

	if (hs_get_status() != SND_JACK_HEADSET) {
		hs_micbias_switch(OFF);
		headset_disable_button_irq();
		hs_data.enable_btn = false;
		pr_info("%s: btn lock.\n", TAG);
		for (i = 0; i < NUM_BUTTON_WORKS; ++i)
			cancel_delayed_work_sync(hs_data.btn_works + i);
	} else {
		headset_enable_button_irq();
		if (!hs_data.enable_btn)
			queue_delayed_work(hs_data.btn_wq, &hs_data.btn_enable_work,
					msecs_to_jiffies(1000));
	}
}

static int detection_work(void)
{
	unsigned long debouncing;

	const int INSERT_JK_DEBOUNCING = 300; // This value must shorter than button.
	const int REMOVAL_JK_DEBOUNCING = 0;

	/*Avoid unnecessary headset detection after resuming codec*/
	if (!hs_data.enable_detect)
		return hs_gpio[HS_JACK].jack->status;

	button_work();
	/* prevent system from falling asleep again as we sleep */
	hs_data.detecting = true;
	hs_micbias_switch(ON);
	if (hs_get_status() == 0)
		debouncing = msecs_to_jiffies(INSERT_JK_DEBOUNCING);
	else
		debouncing = msecs_to_jiffies(REMOVAL_JK_DEBOUNCING);
	schedule_delayed_work(&hs_data.jack_work, debouncing);
	return hs_gpio[HS_JACK].jack->status;
}

static void button_enable_work(struct work_struct* w)
{
	if (hs_get_status() == SND_JACK_HEADSET) {
		hs_data.enable_btn = true;
		pr_info("%s: btn unlock.\n", TAG);
	} else
		pr_debug("%s: We're late; HS has gone...\n", TAG);
}


/**********************************************************
 **  Function: Headset driver init function
 **  Parameter: none
 **  Return value: none
 **
 ************************************************************/
int headset_init(void)
{
	int ret, i;
	pr_info("%s+ #####\n", __func__);

	hs_data.btn_wq = create_workqueue("button_work_late");
	for (i = 0; i < NUM_BUTTON_WORKS; ++i) {
		INIT_DELAYED_WORK(&(hs_data.btn_works[i].work), button_work_late);
		hs_data.btn_works[i].key = 0;
	}
	INIT_DELAYED_WORK(&hs_data.jack_work, jack_work_late);
	INIT_DELAYED_WORK(&hs_data.btn_enable_work, button_enable_work);
	hs_data.sdev.name = "h2w";
	hs_data.sdev.print_name = headset_name_show;
	hs_data.sdev.print_state = headset_state_show;
	hs_data.enable_detect = true;
	hs_data.detecting = false;

	jack_gpio = hs_gpio;

	ret = switch_dev_register(&hs_data.sdev);
	if (ret < 0)
		pr_err("Warning: test node creation failed.");

	pr_info("%s- #####\n", __func__);
	return 0;
}

/**********************************************************
 **  Function: Headset driver exit function
 **  Parameter: none
 **  Return value: none
 **
 ************************************************************/
void headset_exit(void)
{
	switch_dev_unregister(&hs_data.sdev);
	destroy_workqueue(hs_data.btn_wq);
}

