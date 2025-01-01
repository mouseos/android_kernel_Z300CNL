/*
 * Support for Himax HM2051 8M camera sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __HM2051_H__
#define __HM2051_H__
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>

#include <linux/atomisp_platform.h>

#define HM2051_NAME		"hm2051"

/* Defines for register writes and register array processing */
#define I2C_MSG_LENGTH		0x2
#define I2C_RETRY_COUNT		5

#define HM2051_FOCAL_LENGTH_NUM	334	/*3.34mm*/
#define HM2051_FOCAL_LENGTH_DEM	100
#define HM2051_F_NUMBER_DEFAULT_NUM	24
#define HM2051_F_NUMBER_DEM	10

#define MAX_FMTS		1

#define HM2051_INTEGRATION_TIME_MARGIN	5

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define HM2051_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define HM2051_F_NUMBER_RANGE 0x180a180a
#define HM2051_ID	0x2051


#define HM2051_BIN_FACTOR_MAX 4
#define HM2051_FINE_INTG_TIME_MIN 0
#define HM2051_FINE_INTG_TIME_MAX_MARGIN 65535
#define HM2051_COARSE_INTG_TIME_MIN 0
#define HM2051_COARSE_INTG_TIME_MAX_MARGIN 6/*(0xffff - 6)*/
/*
 * HM2051 System control registers
 */
#define HM2051_SW_RESET					0x0022
#define HM2051_SW_STREAM				0x0005

#define HM2051_SC_CMMN_CHIP_ID_H		0x0001
#define HM2051_SC_CMMN_CHIP_ID_L		0x0002

#define HM2051_PRE_PLL_CLK_DIV			0x002A
#define HM2051_PLL_MULTIPLIER			  0x002B
/*#define HM2051_VT_PIX_CLK_DIV			0x0301*/
#define HM2051_VT_SYS_CLK_DIV			  0x0026


#define HM2051_COMMAND_UPDATE     0x0100

#define HM2051_EXPOSURE_H				0x0015
#define HM2051_EXPOSURE_L				0x0016
#define HM2051_AGAIN            0x0018
#define HM2051_DGAIN            0x001D


#define HM2051_HORIZONTAL_START		0x0344
#define HM2051_VERTICAL_START			0x0346
#define HM2051_HORIZONTAL_END			0x0348
#define HM2051_VERTICAL_END				0x034a



#define HM2051_TIMING_VTS_H				0x0010
#define HM2051_TIMING_VTS_L				0x0011



#define HM2051_START_STREAMING			0x03
#define HM2051_STOP_STREAMING		    0x02

/*#define HM2051_PCLK             88533333 //70400000*/

#define FULL_SPEED_HM2051       1
#ifdef FULL_SPEED_HM2051
#define HM2051_PCLK             88533333
#define HM2051_CLK_DIV          0x00
#define HM2051_MIPI_DRIVING     0x12
#else
#define HM2051_PCLK             44266667
#define HM2051_CLK_DIV          0x05
#define HM2051_MIPI_DRIVING     0x13     /* strong driving*/
#endif

enum hm2051_facing_type {
	HM2051_FACING_REAR,
	HM2051_FACING_FRONT,
	HM2051_FACING_UNKNOWN,
};


struct regval_list {
	u16 reg_num;
	u8 value;
};

struct hm2051_resolution {
	u8 *desc;
	const struct hm2051_reg *regs;
	int res;
	int width;
	int height;
	int pix_clk_freq;
	int fps;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
	bool used;
	int skip_frames;
	int horizontal_start;
	int horizontal_end;
	int vertical_start;
	int vertical_end;
};

struct hm2051_format {
	u8 *desc;
	u32 pixelformat;
	struct hm2051_reg *regs;
};

struct hm2051_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

/*
 * HM2051 device structure.
 */
struct hm2051_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct mutex input_lock;

	struct camera_sensor_platform_data *platform_data;
	int vt_pix_clk_freq_mhz;
	int fmt_idx;
	int run_mode;
	u8 res;
	u8 type;
};

enum hm2051_tok_type {
	HM2051_8BIT  = 0x0001,
	HM2051_16BIT = 0x0002,
	HM2051_32BIT = 0x0004,
	HM2051_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	HM2051_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	HM2051_TOK_MASK = 0xfff0
};

/**
 * struct hm2051_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct hm2051_reg {
	enum hm2051_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

#define to_hm2051_sensor(x) container_of(x, struct hm2051_device, sd)

#define HM2051_MAX_WRITE_BUF_SIZE	30

struct hm2051_write_buffer {
	u16 addr;
	u8 data[HM2051_MAX_WRITE_BUF_SIZE];
};

struct hm2051_write_ctrl {
	int index;
	struct hm2051_write_buffer buffer;
};


#define PREVIEW_CAP 1
#define SW_STD 0
/*ULPM -> PREVIEW_CAP 0 and SW_STD 0  */

/*previe <-> full*/
#if PREVIEW_CAP == 1
static struct hm2051_reg const hm2051_stream_on[] = {
	{HM2051_8BIT, 0x0000, 0x01},
	{HM2051_8BIT, 0x0100, 0x01},
	{HM2051_8BIT, 0x0101, 0x01},
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x0005, 0x03},	/*Turn on rolling shutter   */
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_stream_off[] = {
	{HM2051_8BIT, 0x0005, 0x02},	/*Turn on rolling shutter*/
	{HM2051_8BIT, 0x4B20, 0xDE},
	/*{HM2051_TOK_DELAY, 0, 100},*/
	{HM2051_TOK_TERM, 0, 0}
};
#elif SW_STD == 1

/*SW_STANDBY*/
static struct hm2051_reg const hm2051_stream_on[] = {
	{HM2051_8BIT, 0x0025, 0x80},
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x0005, 0x02},
	/*{HM2051_8BIT, 0x4B23, 0x04},*/
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_8BIT, 0x0005, 0x03},
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_stream_off[] = {
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x4B20, 0xDE},
	{HM2051_TOK_DELAY, 0, 150},
	{HM2051_8BIT, 0x0005, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};

#else /*#define ULPM 0*/

static struct hm2051_reg const hm2051_stream_on[] = {

	{HM2051_8BIT, 0x0025, 0x80},
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x4B23, 0x04},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_8BIT, 0x0005, 0x03},
  /*{HM2051_TOK_DELAY,0,150},       */
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_stream_off[] = {
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_TOK_DELAY, 0, 40},
	{HM2051_8BIT, 0x4B20, 0xDE},
	{HM2051_8BIT, 0x4B23, 0x01},
	{HM2051_8BIT, 0x0005, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};
#endif



static const struct i2c_device_id hm2051_id[] = {
	{HM2051_NAME, 0},
	{}
};

static struct hm2051_reg const hm2051_global_setting[] = {

/*---------------------------------------------------*/
/* Initial*/
/*---------------------------------------------------*/
	/*{HM2051_8BIT,0x0005,0x02},*/
	/*{HM2051_TOK_DELAY,0,100},  */
	/*------------------------------*/
	{HM2051_8BIT, 0x0022, 0x00},/* RESET*/
	/* ---------------------------------------------------*/
	/* CMU update*/
	/* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	{HM2051_8BIT, 0x0005, 0x02},/* power up*/
	{HM2051_TOK_DELAY, 0, 50},

	{HM2051_8BIT, 0x0026, 0x08},/* PLL1, mipi pll_pre_div*/
	{HM2051_8BIT, 0x002A, 0x52},/* PLL1, mipi pll_div (pclk=pktclk= 002A + 1)*/


	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode    */
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/

    /*---------------------------------------------------*/
    /* Analog*/
    /*---------------------------------------------------*/


	{HM2051_8BIT, 0x002B, HM2051_CLK_DIV},/*  clk divider  //00->05*/
	{HM2051_8BIT, 0x002C, 0x06},/* PLL cfg: CP, LPF, use PLL clk*/

	{HM2051_8BIT, 0x0040, 0x00}, /* BLC  //0A->00*/
	{HM2051_8BIT, 0x0044, 0x03},/* enable BLC, enable BLC IIR*/
	{HM2051_8BIT, 0x0045, 0x63},/* CFPN cfg, 0x65=repeat 32 times, 0x55=repeat 16 times*/
	{HM2051_8BIT, 0x0046, 0x5F},/* CFPN cfg, enable IIR, weight=1/4 new, CFPN applied to BLC*/
	{HM2051_8BIT, 0x0049, 0xC0},/*improve BLC_hunting*/
	{HM2051_8BIT, 0x004B, 0x03},/*improve BLC_hunting*/
	{HM2051_8BIT, 0x0070, 0x2F},/* ADD  0923*/
	{HM2051_8BIT, 0x0072, 0xFB},/*8F -> FB  0923*/
	{HM2051_8BIT, 0x0073, 0x77},/* ADD  0923 for 30 fps*/
	{HM2051_8BIT, 0x0075, 0x40},/* Negative CP is controlled by INT*/
	{HM2051_8BIT, 0x0078, 0x65},/* ADD  0923 for 30 fps*/

	{HM2051_8BIT, 0x0080, 0x98},/* fd_clamp_en_d=1, tg_boost_en_d=1  //90 -> 98  0923*/
	{HM2051_8BIT, 0x0082, 0x09},/* fd_clamp_en_d=1, tg_boost_en_d=1*/
	{HM2051_8BIT, 0x0083, 0x3C},/* VRPP=avdd+1.36V, VRNP=-1V, w/ lowest pump clk freq.*/
	{HM2051_8BIT, 0x0087, 0x41},/* disable dsun clamp first  31 -> 41 0923*/
	{HM2051_8BIT, 0x008D, 0x20},/* pix_disc_diff_en_d=1*/
	{HM2051_8BIT, 0x008E, 0x30},

	{HM2051_8BIT, 0x009D, 0x11},/* Nramp_rst1,2*/
	{HM2051_8BIT, 0x009E, 0x12},/* Nramp_rst3,4*/

	{HM2051_8BIT, 0x0090, 0x00},/* gain table*/
	{HM2051_8BIT, 0x0091, 0x01},/* gain table*/
	{HM2051_8BIT, 0x0092, 0x02},/* gain table*/
	{HM2051_8BIT, 0x0093, 0x03},/* gain table*/

	{HM2051_8BIT, 0x00C0, 0x64},/* col_ldo setting*/
	{HM2051_8BIT, 0x00C1, 0x15},/* pvdd_refsel=5h(for power noise), pvdd_lorefsel*/
	{HM2051_8BIT, 0x00C2, 0x00},/* ramp power consumption*/

	{HM2051_8BIT, 0x00C3, 0x02},/* comp1,2,3_bias_sel*/
	{HM2051_8BIT, 0x00C4, 0x0B},/* column ADC cfg*/
	{HM2051_8BIT, 0x00C6, 0x83},/* sf_srcdr_shrt_en_right_d=1, sf_always_on_d=0(improve noise)  93 -> 83 0923*/
	{HM2051_8BIT, 0x00C7, 0x02},/* sr_sel_sh_d (reduce CFPN @ high AVDD) ADD 0923*/
	{HM2051_8BIT, 0x00CC, 0x00},/* mipi skew[5:0]*/

	{HM2051_8BIT, 0x4B3B, HM2051_MIPI_DRIVING},/* MIPI analog setting (strong driving )*/
	{HM2051_8BIT, 0x4B41, 0x10},/* HS0_D=1, prevent enter test mode(clk lane=0)*/

	/*Star of BPC setting*/
	{HM2051_8BIT, 0x0165, 0x03},/*[1:0]0:24 1:32, 2:48, 3:80*/
	{HM2051_8BIT, 0x018C, 0x00},/*[7:6]Dark_raw_enable*/

	{HM2051_8BIT, 0x0195, 0x06},/*X_offset[2:0]*/
	{HM2051_8BIT, 0x0196, 0x4F},/*X_offset[7:0]*/
	{HM2051_8BIT, 0x0197, 0x04},/*Y_offset[2:0]*/
	{HM2051_8BIT, 0x0198, 0xBF},/*Y_offset[7:0]*/

	{HM2051_8BIT, 0x0144, 0x10},/*BPC_HOT_TH[8],[1]Median Filter with current pixel*/
	{HM2051_8BIT, 0x0140, 0x20},/*BPC_HOT_TH[7:0]*/
	{HM2051_8BIT, 0x015A, 0x4B},/*BPC_HOT_2*/
	{HM2051_8BIT, 0x015D, 0x20},/*BPC_HOT_3*/
	{HM2051_8BIT, 0x0160, 0x61},/*[0]hot_replace[1]cold_replace[3:2]Max1_Max2[4]correct_all[5]Dynamic[6]Static[7]no write back */

	/* for default d gain*/
	{HM2051_8BIT, 0x001D, 0x40},

	{HM2051_TOK_TERM, 0, 0}
};


static struct hm2051_reg const hm2051_1616x1216_30fps[] = {

	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000D, 0x00},/* [0] vsub2*/
	{HM2051_8BIT, 0x000E, 0x00},/* [0] hsub2   */
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode    */
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/

	/*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x65},  /*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0x00},  /*static BPC windows offset*/

	/* ---------------------------------------------------*/
	/* mipi-tx settings*/
	/* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0123, 0xC5},/* [4] digital window off*/
	{HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb 09->08*/
	{HM2051_8BIT, 0x4B51, 0xE2},/* pre_h Lb 22->B2  //B2 -> E2 0923*/
	{HM2051_8BIT, 0x4B0A, 0x06},
	{HM2051_8BIT, 0x4B0B, 0x50},

	/* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},


	#ifdef FULL_SPEED_HM2051
		{HM2051_8BIT, 0x4B02, 0x04},
		{HM2051_8BIT, 0x4B03, 0x09},
		{HM2051_8BIT, 0x4B04, 0x05},
		{HM2051_8BIT, 0x4B05, 0x0C},
		{HM2051_8BIT, 0x4B06, 0x06},
		{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
		{HM2051_8BIT, 0x4B3F, 0x12},
		{HM2051_8BIT, 0x4B42, 0x05},
		{HM2051_8BIT, 0x4B43, 0x00},
	#else
		{HM2051_8BIT, 0x4B02, 0x02},
		{HM2051_8BIT, 0x4B03, 0x07},
		{HM2051_8BIT, 0x4B04, 0x02},
		{HM2051_8BIT, 0x4B05, 0x0C},
		{HM2051_8BIT, 0x4B06, 0x04},
		{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
		{HM2051_8BIT, 0x4B3F, 0x0C},
		{HM2051_8BIT, 0x4B42, 0x02},
		{HM2051_8BIT, 0x4B43, 0x00},
	#endif

	/* ---------------------------------------------------*/
    /* CMU update*/
    /* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
    /*---------------------------------------------------*/
    /* Turn on rolling shutter*/
    /*---------------------------------------------------*/
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};


static struct hm2051_reg const hm2051_1616x916_30fps[] = {

	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000D, 0x00},/* [0] vsub2*/
	{HM2051_8BIT, 0x000E, 0x00},/* [0] hsub2    */
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode    */
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/

    /*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x65},  /*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0x00},  /*static BPC windows offset*/

    /* ---------------------------------------------------*/
    /* mipi-tx settings*/
    /* ---------------------------------------------------*/

	{HM2051_8BIT, 0x0123, 0xD5},
	{HM2051_8BIT, 0x0660, 0x00},  /*win x_st Hb*/
	{HM2051_8BIT, 0x0661, 0x00},  /*win x_st Lb*/
	{HM2051_8BIT, 0x0662, 0x06},  /*win x_ed Hb*/
	{HM2051_8BIT, 0x0663, 0x4F},  /*win x_ed Lb*/
	{HM2051_8BIT, 0x0664, 0x00},  /*win y_st Hb*/
	{HM2051_8BIT, 0x0665, 0x96},  /*win y_st Lb*/
	{HM2051_8BIT, 0x0666, 0x04},  /*win y_ed Hb*/
	{HM2051_8BIT, 0x0667, 0x29},  /*win y_ed Lb*/

	{HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb 09->08*/
	{HM2051_8BIT, 0x4B51, 0xE2},/* pre_h Lb 22->B2  //B2 -> E2 0923*/
	{HM2051_8BIT, 0x4B0A, 0x06},
	{HM2051_8BIT, 0x4B0B, 0x50},


	/* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},

	/* ---------------------------------------------------*/
    /* CMU update*/
    /* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},

	/*---------------------------------------------------*/
	/* Turn on rolling shutter*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};


static struct hm2051_reg const hm2051_1096x736_48fps[] = {

	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x04},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000D, 0x00},/* [0] vsub2*/
	{HM2051_8BIT, 0x000E, 0x00},/* [0] hsub2    */
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode*/
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x1B},/* disable output black rows*/

	/*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x65},/*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0xF0},/*static BPC windows offset*/

	/* ---------------------------------------------------*/
	/* mipi-tx settings*/
	/* ---------------------------------------------------*/

	{HM2051_8BIT, 0x0123, 0xD5},
	{HM2051_8BIT, 0x0660, 0x01},  /*win x_st Hb*/
	{HM2051_8BIT, 0x0661, 0x04},  /*win x_st Lb*/
	{HM2051_8BIT, 0x0662, 0x05},  /*win x_ed Hb*/
	{HM2051_8BIT, 0x0663, 0x4B},  /*win x_ed Lb*/
	{HM2051_8BIT, 0x0664, 0x00},  /*win y_st Hb*/
	{HM2051_8BIT, 0x0665, 0x2E},  /*win y_st Lb*/
	{HM2051_8BIT, 0x0666, 0x03},  /*win y_ed Hb*/
	{HM2051_8BIT, 0x0667, 0x0D},  /*win y_ed Lb*/

	{HM2051_8BIT, 0x4B50, 0x00},/* pre_h Hb*/
	{HM2051_8BIT, 0x4B51, 0xE0},/* pre_h Lb  CD->AF  => EO -> C2   //keep E0 workaround*/
	{HM2051_8BIT, 0x4B0A, 0x04},
	{HM2051_8BIT, 0x4B0B, 0x48},

	/* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},

	/* ---------------------------------------------------*/
	/* CMU update*/
    /* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},

	/*---------------------------------------------------*/
    /* Turn on rolling shutter*/
    /*---------------------------------------------------*/
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_1296x736_48fps[] = {

    /*---------------------------------------------------*/
    /* Digital function*/
    /*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x04},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000D, 0x00},/* [0] vsub2*/
	{HM2051_8BIT, 0x000E, 0x00},/* [0] hsub2    */
	{HM2051_8BIT, 0x000F, 0x00},/* [1] fixed frame rate mode, [0] non fixed frame rate mode*/
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x1B},/* disable output black rows*/

    /*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x65},  /*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0xF0},  /*static BPC windows offset    */

	/* ---------------------------------------------------*/
	/* mipi-tx settings*/
	/* ---------------------------------------------------*/

	{HM2051_8BIT, 0x0123, 0xD5},
	{HM2051_8BIT, 0x0660, 0x00},  /*win x_st Hb*/
	{HM2051_8BIT, 0x0661, 0xA0},  /*win x_st Lb*/
	{HM2051_8BIT, 0x0662, 0x05},  /*win x_ed Hb*/
	{HM2051_8BIT, 0x0663, 0xAF},  /*win x_ed Lb*/
	{HM2051_8BIT, 0x0664, 0x00},  /*win y_st Hb*/
	{HM2051_8BIT, 0x0665, 0x2E},  /*win y_st Lb*/
	{HM2051_8BIT, 0x0666, 0x03},  /*win y_ed Hb*/
	{HM2051_8BIT, 0x0667, 0x0D},  /*win y_ed Lb*/

	{HM2051_8BIT, 0x4B50, 0x00},/* pre_h Hb*/
	{HM2051_8BIT, 0x4B51, 0xAF},/* pre_h Lb  CD->AF*/
	{HM2051_8BIT, 0x4B0A, 0x05},
	{HM2051_8BIT, 0x4B0B, 0x10},


	/* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},

	/*---------------------------------------------------*/
	/* CMU update*/
	/* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},

	/*---------------------------------------------------*/
	/* Turn on rolling shutter*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0005, 0x02},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};

static struct hm2051_reg const hm2051_SUB2_808x608_56sfps[] = {
	/*{HM2051_8BIT,0x0005, 0x02},*/
	/*{HM2051_TOK_DELAY,0,100},*/

	/*---------------------------------------------------*/
	/* Digital function*/
	/*---------------------------------------------------*/
	{HM2051_8BIT, 0x0006, 0x00},/* [1] hmirror, [0] vflip*/
	{HM2051_8BIT, 0x000D, 0x01},/* [0] vsub2*/
	{HM2051_8BIT, 0x000E, 0x01},/* [0] hsub2*/
	{HM2051_8BIT, 0x0024, 0x40},/* mipi enable*/
	{HM2051_8BIT, 0x0027, 0x23},/* OUTFMT, after BPC, [7] pre_vsync_en*/
	{HM2051_8BIT, 0x0065, 0x01},/* Hscan_delay (default=1)*/
	{HM2051_8BIT, 0x0074, 0x13},/* disable output black rows*/

	/*static dymaic BPC*/
	{HM2051_8BIT, 0x0160, 0x25},  /*static BPC switch*/
	{HM2051_8BIT, 0x0183, 0x00},  /*static BPC windows offset       */

	/* ---------------------------------------------------*/
	/* mipi-tx settings*/
	/* --------------------------------------------------- */

	/* window off (1616x1216)*/
	{HM2051_8BIT, 0x0123, 0xC5},/* [4] digital window offs   */
	{HM2051_8BIT, 0x4B50, 0x08},/* pre_h Hb 09->08*/
	{HM2051_8BIT, 0x4B51, 0xE2},/* pre_h Lb 22->B2  //E6 -> E2    */
	{HM2051_8BIT, 0x4B0A, 0x03},  /*HSIZE (800)*/
	{HM2051_8BIT, 0x4B0B, 0x28},

	/* window on (default 1600x1200)*/
	{HM2051_8BIT, 0x4B20, 0x9E},
	{HM2051_8BIT, 0x4B07, 0xBD},/* MARK1 width*/
	{HM2051_8BIT, 0x4B30, 0x0E},
	{HM2051_8BIT, 0x4B30, 0x0F},

	/* ---------------------------------------------------*/
    /* CMU update*/
    /* ---------------------------------------------------*/
	{HM2051_8BIT, 0x0000, 0x00},
	{HM2051_8BIT, 0x0100, 0x00},
	{HM2051_8BIT, 0x0101, 0x00},
	/*---------------------------------------------------*/
	/* Turn on rolling shutter*/
	/*---------------------------------------------------    */
	{HM2051_8BIT, 0x0005, 0x03},
	{HM2051_8BIT, 0x0025, 0x00},
	{HM2051_TOK_TERM, 0, 0}
};


/*
preview: 16:9
capture: 16:9 4:3
video:   16:9 3:2
*/

struct hm2051_resolution hm2051_res_preview[] = {
	{
		.desc = "THIS IS PREVIEW:hm2051_1616x1216_30fps",
		.width = 1616,
		.height = 1216,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1616x1216_30fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1615,
		.vertical_start = 0,
		.vertical_end = 1215,
	},

	{
		.desc = "THIS IS PREVIEW:hm2051_1616x916_30fps",
		.width = 1616,
		.height = 916,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1616x916_30fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1615,
		.vertical_start = 0,
		.vertical_end = 915,
	},

};
#define N_RES_PREVIEW (ARRAY_SIZE(hm2051_res_preview))

struct hm2051_resolution hm2051_res_still[] = {
	{
		.desc = "THIS IS STILL:hm2051_1616x1216_30fps",
		.width = 1616,
		.height = 1216,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1616x1216_30fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1615,
		.vertical_start = 0,
		.vertical_end = 1215,
	},

	{
		.desc = "THIS IS STILL:hm2051_1616x916_30fps",
		.width = 1616,
		.height = 916,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1616x916_30fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1615,
		.vertical_start = 0,
		.vertical_end = 915,
	},


#if 0
	{
		.desc = "THIS IS PREVIEW SUB2 :hm2051_808x608_56fps",
		.width = 808,
		.height = 608,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 56,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 659,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.bin_mode = 1,
		.regs = hm2051_SUB2_808x608_56sfps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 807,/*1615,*/
		.vertical_start = 0,
		.vertical_end = 607,/*735,  */
	},
#endif
};
#define N_RES_STILL (ARRAY_SIZE(hm2051_res_still))

struct hm2051_resolution hm2051_res_video[] = {

	{
		.desc = "THIS IS VIDEO:hm2051_1616x1216_30fps",
		.width = 1616,
		.height = 1216,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 30,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 1266,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1616x1216_30fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1615,
		.vertical_start = 0,
		.vertical_end = 1215,

	},

	{
		.desc = "THIS IS VIDEO:hm2051_1296x736_48fps",
		.width = 1296,
		.height = 736,
		.pix_clk_freq = HM2051_PCLK,
		.fps = 48,
		.used = 0,
		.pixels_per_line = 2310,
		.lines_per_frame = 786,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.bin_mode = 0,
		.regs = hm2051_1296x736_48fps,
		.skip_frames = 0,
		.horizontal_start = 0,
		.horizontal_end = 1295,
		.vertical_start = 0,
		.vertical_end = 735,
	},

};
#define N_RES_VIDEO (ARRAY_SIZE(hm2051_res_video))

static struct hm2051_resolution *hm2051_res = hm2051_res_preview;
static int N_RES = N_RES_PREVIEW;
#endif
