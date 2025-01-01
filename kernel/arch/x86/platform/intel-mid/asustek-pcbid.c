/*
 * arch/x86/platform/intel-mid/asustek-pcbid.c
 *
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/board_asustek.h>
#include <linux/lnw_gpio.h>

#include <asm/intel_scu_flis.h>
#include <asm/intel-mid.h>
#include <linux/delay.h>

#define PCBID_VALUE_INVALID 0x4E2F4100 /* N/A */

enum {
	DEBUG_STATE = 1U << 0,
	DEBUG_VERBOSE = 1U << 1,
};

static int debug_mask = DEBUG_STATE;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static unsigned int asustek_pcbid = PCBID_VALUE_INVALID;
static const char *asustek_chipid;
static unsigned int hw_rev_pcbid[] = {0, 1, 2};
static unsigned int project_id_pcbid[] = {3, 4, 5, 6};
static unsigned int lcd_type_pcbid[] = {7};
static unsigned int tp_type_pcbid[] = {8, 9, 10};
static unsigned int cam_front_pcbid[] = {11};
static unsigned int cam_rear_pcbid[] = {12, 13};
static unsigned int rf_sku_pcbid[] = {14, 15};
static unsigned int ddr_sku_pcbid[] = {16};
static unsigned int sub_project_id_pcbid[] = {6, 7};
static unsigned int lcd_type_pcbid_1[] = {8, 9};
static unsigned int tp_type_pcbid_1[] = {10};
static unsigned int cardiz_pcbid_1[] = {11};
static unsigned int cam_rear_pcbid_1[] = {12};
static unsigned int cam_front_pcbid_1[] = {13};
static unsigned int wifi_pcbid_1[] = {14};
static unsigned int rf_sku_pcbid_1[] = {15, 16, 17};

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
static bool sfi_pcbid_initialized = false;
static unsigned int asustek_pcbid_oem1 = PCBID_VALUE_INVALID;
#endif

struct pcbid_maps {
	unsigned char name[16];
	unsigned int *pcbid;
	unsigned int pcbid_num;
} asustek_pcbid_maps[] = {
	{"HW_REV", hw_rev_pcbid, ARRAY_SIZE(hw_rev_pcbid)},
	{"PROJECT_ID", project_id_pcbid, ARRAY_SIZE(project_id_pcbid)},
	{"LCD_TYPE", lcd_type_pcbid, ARRAY_SIZE(lcd_type_pcbid)},
	{"TP_TYPE", tp_type_pcbid, ARRAY_SIZE(tp_type_pcbid)},
	{"CAM_FRONT", cam_front_pcbid, ARRAY_SIZE(cam_front_pcbid)},
	{"CAM_REAR", cam_rear_pcbid, ARRAY_SIZE(cam_rear_pcbid)},
	{"RF_SKU", rf_sku_pcbid, ARRAY_SIZE(rf_sku_pcbid)},
	{"DDR_SKU", ddr_sku_pcbid, ARRAY_SIZE(ddr_sku_pcbid)},
	{"SUB_PROJECT_ID", sub_project_id_pcbid, ARRAY_SIZE(sub_project_id_pcbid)},
	{"LCD_TYPE_1", lcd_type_pcbid_1, ARRAY_SIZE(lcd_type_pcbid_1)},
	{"TP_TYPE_1", tp_type_pcbid_1, ARRAY_SIZE(tp_type_pcbid_1)},
	{"CARDIZ_ID_1", cardiz_pcbid_1, ARRAY_SIZE(cardiz_pcbid_1)},
	{"CAM_REAR_1", cam_rear_pcbid_1, ARRAY_SIZE(cam_rear_pcbid_1)},
	{"CAM_FRONT_1", cam_front_pcbid_1, ARRAY_SIZE(cam_front_pcbid_1)},
	{"WIFI_SKU_1", wifi_pcbid_1, ARRAY_SIZE(wifi_pcbid_1)},
	{"RF_SKU_1", rf_sku_pcbid_1, ARRAY_SIZE(rf_sku_pcbid_1)},
};

#define NUM_MAPS (sizeof(asustek_pcbid_maps) / sizeof(asustek_pcbid_maps[0]))

int get_pcbid_type(const char *func)
{
	int i = 0, ret = 0;
	struct pcbid_maps *map = NULL;

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
	if (!sfi_pcbid_initialized && (asustek_pcbid == PCBID_VALUE_INVALID)) {
#else
	if (asustek_pcbid == PCBID_VALUE_INVALID) {
#endif
		pr_err("ASUSTek PCBID was invalid\n");
		return -ENODEV;
	}

	for (i = 0; i < NUM_MAPS; i++) {
		if (!strcmp(func, asustek_pcbid_maps[i].name)) {
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("%s was found\n", func);

			map = &asustek_pcbid_maps[i];
			break;
		}
	}

	if (map) {
		/* found */
		for (i = 0; i < map->pcbid_num; i++) {
#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
			if (asustek_pcbid == PCBID_VALUE_INVALID)
				ret += asustek_pcbid_oem1 & BIT(map->pcbid[i]);
			else
#endif
				ret += asustek_pcbid & BIT(map->pcbid[i]);
		}
		ret = ret >> map->pcbid[0];
	} else
		ret = -ENODEV;

	return ret;
}

hw_rev asustek_get_hw_rev(void)
{
	hw_rev ret = HW_REV_INVALID;

	ret = get_pcbid_type("HW_REV");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= HW_REV_MAX))
		ret = HW_REV_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_hw_rev);

project_id asustek_get_project_id(void)
{
	project_id ret = PROJECT_ID_INVALID;

	ret = get_pcbid_type("PROJECT_ID");

	if ((ret & 0x07) == 0x06)
		ret = asustek_get_sub_project_id() + 0x08;

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= PROJECT_ID_MAX))
		ret = PROJECT_ID_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_project_id);

sub_project_id asustek_get_sub_project_id(void)
{
	sub_project_id ret = SUB_PROJECT_ID_INVALID;

	ret = get_pcbid_type("SUB_PROJECT_ID");

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= SUB_PROJECT_ID_MAX))
		ret = SUB_PROJECT_ID_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_sub_project_id);

lcd_type asustek_get_lcd_type(void)
{
	lcd_type ret = LCD_TYPE_INVALID;
	lcd_type max = LCD_TYPE_MAX;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("LCD_TYPE");
		break;
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("LCD_TYPE_1");
		max = LCD_TYPE_1_MAX;
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= max))
		ret = LCD_TYPE_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_lcd_type);

tp_type asustek_get_tp_type(void)
{
	tp_type ret = TP_TYPE_INVALID;
	tp_type max = TP_TYPE_MAX;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("TP_TYPE");
		break;
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("TP_TYPE_1");
		max = TP_TYPE_1_MAX;
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= max))
		ret = TP_TYPE_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_tp_type);

cam_front asustek_get_camera_front(void)
{
	cam_front ret = CAM_FRONT_INVALID;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("CAM_FRONT");
		break;
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("CAM_FRONT_1");
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= CAM_FRONT_MAX))
		ret = CAM_FRONT_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_camera_front);

cam_rear asustek_get_camera_rear(void)
{
	cam_rear ret = CAM_REAR_INVALID;
	cam_rear max = CAM_REAR_MAX;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("CAM_REAR");
		break;
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("CAM_REAR_1");
		max = CAM_REAR_1_MAX;
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= max))
		ret = CAM_REAR_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_camera_rear);

bool asustek_isOV5693(void)
{
	project_id pcbid = asustek_get_project_id();
	if (pcbid == 0x08 || pcbid == 0x0a)
		return true;
	return false;
}
EXPORT_SYMBOL(asustek_isOV5693);

bool asustek_isMT9m114(void)
{
	project_id pcbid = asustek_get_project_id();
	if (pcbid == 0x08 || pcbid == 0x0a)
		return true;
	return false;
}
EXPORT_SYMBOL(asustek_isMT9m114);

rf_sku asustek_get_rf_sku(void)
{
	rf_sku ret = RF_SKU_INVALID;
	rf_sku max = RF_SKU_MAX;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("RF_SKU");
		break;
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("RF_SKU_1");
		max = RF_SKU_1_MAX;
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= max))
		ret = RF_SKU_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_rf_sku);

ddr_sku asustek_get_ddr_sku(void)
{
	ddr_sku ret = DDR_SKU_INVALID;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x00:
		ret = get_pcbid_type("DDR_SKU");
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= DDR_SKU_MAX))
		ret = DDR_SKU_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_ddr_sku);

cardiz_id asustek_get_cardiz_id(void)
{
	cardiz_id ret = CARDIZ_ID_INVALID;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("CARDIZ_ID_1");
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= CARDIZ_ID_MAX))
		ret = CARDIZ_ID_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_cardiz_id);

wifi_sku asustek_get_wifi_sku(void)
{
	wifi_sku ret = WIFI_SKU_INVALID;
	project_id prj = PROJECT_ID_INVALID;

	prj = asustek_get_project_id();
	switch (prj) {
	case 0x08:
	case 0x0a:
		ret = get_pcbid_type("WIFI_SKU_1");
		break;
	default:
		break;
	}

	if (debug_mask & DEBUG_VERBOSE)
		pr_info("%s: %d\n", __func__, ret);

	if ((ret == -ENODEV) || (ret >= WIFI_SKU_MAX))
		ret = WIFI_SKU_INVALID;

	return ret;
}
EXPORT_SYMBOL(asustek_get_wifi_sku);

bool use_ov5693(void)
{
        project_id prj = PROJECT_ID_INVALID;

        prj = asustek_get_project_id();
        if (prj == 0x00)
                return true;

        pr_info("ov5693 is unsupported on this board (0x%x)\n", prj);
        return false;
}
EXPORT_SYMBOL(use_ov5693);

#define ASUSTEK_PCBID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

static ssize_t asustek_pcbid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%05x\n", asustek_pcbid);
}

static ssize_t asustek_projectid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%02x\n", asustek_get_project_id());
}

static ssize_t asustek_hardwareid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%02x\n", asustek_get_hw_rev());
}

static ssize_t asustek_chipid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", asustek_chipid);
}

static ssize_t asustek_rfid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_rf_sku());
}

static ssize_t asustek_ddrid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_ddr_sku());
}

static ssize_t asustek_lcdtype_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_lcd_type());
}

static ssize_t asustek_tptype_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_tp_type());
}

static ssize_t asustek_frontcameraid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_camera_front());
}

static ssize_t asustek_rearcameraid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_camera_rear());
}

static ssize_t asustek_wifiid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%x\n", asustek_get_wifi_sku());
}

ASUSTEK_PCBID_ATTR(asustek_pcbid);
ASUSTEK_PCBID_ATTR(asustek_projectid);
ASUSTEK_PCBID_ATTR(asustek_hardwareid);
ASUSTEK_PCBID_ATTR(asustek_chipid);
ASUSTEK_PCBID_ATTR(asustek_rfid);
ASUSTEK_PCBID_ATTR(asustek_ddrid);
ASUSTEK_PCBID_ATTR(asustek_lcdtype);
ASUSTEK_PCBID_ATTR(asustek_tptype);
ASUSTEK_PCBID_ATTR(asustek_frontcameraid);
ASUSTEK_PCBID_ATTR(asustek_rearcameraid);
ASUSTEK_PCBID_ATTR(asustek_wifiid);

static struct attribute *attr_list[] = {
	&asustek_pcbid_attr.attr,
	&asustek_projectid_attr.attr,
	&asustek_hardwareid_attr.attr,
	&asustek_chipid_attr.attr,
	&asustek_rfid_attr.attr,
	&asustek_ddrid_attr.attr,
	&asustek_lcdtype_attr.attr,
	&asustek_tptype_attr.attr,
	&asustek_frontcameraid_attr.attr,
	&asustek_rearcameraid_attr.attr,
	&asustek_wifiid_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attr_list,
};

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
int sfi_parse_oem1(struct sfi_table_header *table)
{
	struct sfi_table_oem1 *oem1;
	struct sfi_table_oem1_1 *oem1_1;
	u8 sig[SFI_SIGNATURE_SIZE + 1] = {'\0'};
	u8 oem_id[SFI_OEM_ID_SIZE + 1] = {'\0'};
	u8 oem_table_id[SFI_OEM_TABLE_ID_SIZE + 1] = {'\0'};

	oem1 = (struct sfi_table_oem1 *) table;

	if (!oem1) {
		pr_err("%s: fail to read SFI OEM1 Layout\n",
			__func__);
		return -ENODEV;
	}

	snprintf(sig, (SFI_SIGNATURE_SIZE + 1), "%s", oem1->header.sig);
	snprintf(oem_id, (SFI_OEM_ID_SIZE + 1), "%s", oem1->header.oem_id);
	snprintf(oem_table_id, (SFI_OEM_TABLE_ID_SIZE + 1), "%s",
		 oem1->header.oem_table_id);

	pr_info("SFI OEM1 Layout\n");
	pr_info("\tOEM1 signature               : %s\n"
		"\tOEM1 length                  : %d\n"
		"\tOEM1 revision                : %d\n"
		"\tOEM1 checksum                : 0x%X\n"
		"\tOEM1 oem_id                  : %s\n"
		"\tOEM1 oem_table_id            : %s\n"
		"\tOEM1 ifwi_rc                 : 0x%02X\n"
		"\tPCBID hardware_id            : 0x%02X\n"
		"\tPCBID project_id             : 0x%02X\n",
		sig,
		oem1->header.len,
		oem1->header.rev,
		oem1->header.csum,
		oem_id,
		oem_table_id,
		oem1->ifwi_rc,
		oem1->hardware_id,
		oem1->project_id
		);

	switch (oem1->project_id) {
	case 0x00:
		pr_info("\tPCBID lcd_id                 : 0x%02X\n"
			"\tPCBID touch_id               : 0x%02X\n"
			"\tPCBID camera_front_id        : 0x%02X\n"
			"\tPCBID camera_rear_id         : 0x%02X\n"
			"\tPCBID rf_sku                 : 0x%02X\n"
			"\tPCBID ddr_id                 : 0x%02X\n",
			oem1->lcd_id,
			oem1->touch_id,
			oem1->camera_front_id,
			oem1->camera_rear_id,
			oem1->rf_sku,
			oem1->ddr_id
		);

		asustek_pcbid_oem1 = oem1->hardware_id |
			oem1->project_id << 3 |
			oem1->lcd_id << 7 |
			oem1->touch_id << 8 |
			oem1->camera_front_id << 11 |
			oem1->camera_rear_id << 12 |
			oem1->rf_sku << 14 |
			oem1->ddr_id << 16;
		break;

	case 0x06:
	case 0x08:
	case 0x0a:
		oem1_1 = (struct sfi_table_oem1_1 *) table;
		pr_info("\tPCBID sub_project_id         : 0x%02X\n"
			"\tPCBID lcd_id                 : 0x%02X\n"
			"\tPCBID touch_id               : 0x%02X\n"
			"\tPCBID cardiz_id              : 0x%02X\n"
			"\tPCBID camera_front_id        : 0x%02X\n"
			"\tPCBID camera_rear_id         : 0x%02X\n"
			"\tPCBID wifi_id                : 0x%02X\n"
			"\tPCBID rf_sku                 : 0x%02X\n",
			oem1_1->sub_project_id,
			oem1_1->lcd_id,
			oem1_1->touch_id,
			oem1_1->cardiz_id,
			oem1_1->camera_front_id,
			oem1_1->camera_rear_id,
			oem1_1->wifi_id,
			oem1_1->rf_sku
		);
		asustek_pcbid_oem1 = oem1_1->hardware_id | 0x6 << 3 |
			oem1_1->sub_project_id << 6 | oem1_1->lcd_id << 8 |
			oem1_1->touch_id << 10 | oem1_1->cardiz_id << 11 |
			oem1_1->camera_front_id << 12 |
			oem1_1->camera_rear_id << 13 |
			oem1_1->wifi_id << 14 | oem1_1->rf_sku << 15;
		break;
	default:
		pr_err("%s: Unknow project id: %02x\n",
					__func__, oem1->project_id);
		break;
	}

	return 0;
}
EXPORT_SYMBOL(sfi_parse_oem1);

void sfi_parsing_done(bool initialized)
{
	sfi_pcbid_initialized = initialized;
}
EXPORT_SYMBOL(sfi_parsing_done);
#endif

void config_pcbid_suspend_resume(int suspend)
{
	int i, index, prj;
	int hw_project_pinindex[] = {235, 241, 242, 243, 244, 245, 148};
	int project0_pinindex[] = {225, 146, 147, 200, 149, 150};
	int project1_pinindex[] = {225, 248, 249, 226, 146, 147, 200, 149, 150};
	prj = ((asustek_pcbid >> 3) & 0xF);

	if (prj == 0x0 || prj == 0x1) {
		for (i = 0; i < ARRAY_SIZE(hw_project_pinindex); i++) {
			index = hw_project_pinindex[i];
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("config gpio index = %d\n", index);
			if (suspend) {
				if (config_pin_flis(index, PULL, DOWN_50K))
					pr_err("Failed to configure gpio index %d with pull down\n", index);
			}
			else {
				if (config_pin_flis(index, PULL, UP_50K))
					pr_err("Failed to configure gpio index %d with pull up\n", index);
			}
		}

		for (i = 0; i < ARRAY_SIZE(project0_pinindex); i++) {
			if (prj == 0x0)
				index = project0_pinindex[i];
			else if (prj == 0x1)
				index = project1_pinindex[i];
			if (debug_mask & DEBUG_VERBOSE)
				pr_info("config gpio index = %d\n", index);
			if (suspend) {
				if (config_pin_flis(index, PULL, DOWN_50K))
					pr_err("Failed to configure gpio index %d with pull down\n", index);
			}
			else {
				if (config_pin_flis(index, PULL, UP_50K))
					pr_err("Failed to configure gpio index %d with pull up\n", index);
			}
		}
		udelay(200);
	}
}
EXPORT_SYMBOL(config_pcbid_suspend_resume);

static int __init pcbid_driver_probe(struct platform_device *pdev)
{
	int i, j, ret = 0;
	struct resource *res;
	struct asustek_pcbid_platform_data *pdata;
	unsigned int value;
	int gpio;
	int prj = 0;
	struct resource *pdata_res;
	int nr_pdata_res;

	if (!pdev)
		return -EINVAL;

	pdata = pdev->dev.platform_data;

	if (pdata)
		asustek_chipid = kstrdup(pdata->UUID, GFP_KERNEL);

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
	if (!sfi_pcbid_initialized) {
		pr_info("ASUSTek: Cannot parse PCB_ID layout from SFI OEM1 table\n");
		asustek_pcbid_oem1 = 0;
	}
#endif

	asustek_pcbid = 0;

	//manipulate pcbid of project & revision
	for (i = 0; i < pdev->num_resources; i++) {
		if (i == 6 && ((asustek_pcbid >> 3) & 0x7) == 6)
			break;

		res = platform_get_resource(pdev, IORESOURCE_IO, i);
		if (!res)
			return -ENODEV;

		gpio = res->start;
		/*
		 * change necessary GPIO pin mode for PCB_ID module working.
		 * This is something should be done in IA firmware.
		 * But, anyway, just do it here in case IA firmware
		 * forget to do so.
		 */
		lnw_gpio_set_alt(gpio, LNW_GPIO);

		/* set pin to none pull-strength and open-drain disabled */
		#if 0
		/*
		 * FIXME: config_pin_flis() is error prone because
		 * 1) Not all gpio number could be matched with corresponding
		 * 	pin list index. Ex: GPIO154 ~ 161, GPIO163, and GPIO97.
		 * 2) CONFIG_X86_MRFLD is not set so that bitwise operation
		 * 	applys CTP's setup for ANNIEDALE's manipulation.
		 * 3) The first parameter is pin list index not gpio number used
		 *	as below.
		 */
		if (config_pin_flis(gpio, PULL, NONE) ||
			config_pin_flis(gpio, OPEN_DRAIN, OD_DISABLE)) {
			pr_err("ASUSTek: Failed to configure gpio%d with"
					"no-pull/open-drain disabled\n", gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			res = NULL;
			break;
		}
		#endif

		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Requesting gpio%d\n", gpio);

		ret = gpio_request(gpio, res->name);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to request gpio%d\n", gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			res = NULL;
			break;
		}

		ret = gpio_direction_input(gpio);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to configure direction for gpio%d\n",
					gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			res = NULL;
			break;
		}

		/* read input value through gpio library directly */
		value = gpio_get_value(gpio) ? 1 : 0;
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Input value of gpio%d is %s\n", gpio,
					value ? "high" : "low");

		asustek_pcbid |= value << i;
	}

	//manipulate rest of pcbid
	prj = ((asustek_pcbid >> 3) & 0xF);
	switch (prj) {
	case 0x00:
		pdata_res = pdata->resource0;
		nr_pdata_res = pdata->nr_resource0;
		break;
	case 0x06:
		pdata_res = pdata->resource1;
		nr_pdata_res = pdata->nr_resource1;
		break;
	default:
		break;
	}

	if (!pdata_res)
		return -ENODEV;

	for (j = 0; j < nr_pdata_res; j++) {
		gpio = (&pdata_res[j])->start;

		lnw_gpio_set_alt(gpio, LNW_GPIO);

		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Requesting gpio%d\n", gpio);

		ret = gpio_request(gpio, (&pdata_res[j])->name);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to request gpio%d\n", gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			break;
		}

		ret = gpio_direction_input(gpio);
		if (ret) {
			/* indicate invalid pcbid value when error happens */
			pr_err("ASUSTek: Failed to configure direction for gpio%d\n",
					gpio);
			asustek_pcbid = PCBID_VALUE_INVALID;
			break;
		}

		/* read input value through gpio library directly */
		value = gpio_get_value(gpio) ? 1 : 0;
		if (debug_mask & DEBUG_VERBOSE)
			pr_info("ASUSTek: Input value of gpio%d is %s\n", gpio,
					value ? "high" : "low");

		asustek_pcbid |= value << (j + i);
	}

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
	if (sfi_pcbid_initialized && (asustek_pcbid_oem1 != asustek_pcbid)) {
		if (debug_mask && DEBUG_STATE)
			pr_info("ASUSTek: OEM1 PCBID=%05x\n", asustek_pcbid_oem1);
		WARN_ON(1);
	}
#endif

	if (asustek_pcbid == PCBID_VALUE_INVALID) {

#if defined(CONFIG_SFI) && defined(CONFIG_SFI_PCB)
		asustek_pcbid = asustek_pcbid_oem1;
#endif

		/* error handler to free allocated gpio resources */
		while (i >= 0) {
			res = platform_get_resource(pdev, IORESOURCE_IO, i);
			if (!res)
				return -ENODEV;

			if (debug_mask & DEBUG_VERBOSE)
				pr_info("ASUSTek: Freeing gpio%d\n", gpio);

			gpio_free(gpio);
			i--;
		}
		while (j >= 0) {
			gpio = (&pdata_res[j])->start;

			if (debug_mask & DEBUG_VERBOSE)
				pr_info("ASUSTek: Freeing gpio%d\n", gpio);

			gpio_free(gpio);
			j--;
		}
	} else {
		/* report pcbid info to dmesg */
		if (debug_mask && DEBUG_STATE)
			pr_info("ASUSTek: PCBID=%05x\n", asustek_pcbid);

		/* create a sysfs interface */
		ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);

		if (ret)
			pr_err("ASUSTek: Failed to create sysfs group\n");
	}

	return ret;
}

static int pcbid_driver_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver asustek_pcbid_driver __refdata = {
	.probe = pcbid_driver_probe,
	.remove = pcbid_driver_remove,
	.driver = {
		.name = "asustek_pcbid",
		.owner = THIS_MODULE,
	},
};

static int asustek_pcbid_init(void)
{
	return platform_driver_register(&asustek_pcbid_driver);
}

rootfs_initcall(asustek_pcbid_init);

MODULE_DESCRIPTION("ASUSTek PCBID driver");
MODULE_AUTHOR("Paris Yeh <paris_yeh@asus.com>");
MODULE_LICENSE("GPL");
