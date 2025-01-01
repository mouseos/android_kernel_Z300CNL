#include <video/mipi_display.h>
#include <linux/debugfs.h>
//backlight pwm from soc +
#include <linux/lnw_gpio.h>
#include <linux/intel_mid_pm.h>
//backlight pwm from soc -
#include <linux/board_asustek.h>

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/boe_tv101wxmnu1.h"
#include "sony_cxd4756gf.h"

#define CADIZ_IBC_ON 1
#define USE_CADIZ_PWM 1

#define TV101WXMNU1_WIDTH 135
#define TV101WXMNU1_HEIGHT 217

#define TV101WXMNU1_AVDD_GPIO 189
#define Z300CNL 1

static int panel_en_gpio;
static int panel_reset_gpio;

static int bist_enable;

static int cadiz_exist;

#if !USE_CADIZ_PWM
//backlight pwm from soc +
#define TV101WXMNU1_BL_PWM_GPIO  183
#define PWMCTRL_REG 0xFF013C00
static int bl_pwm_gpio;
static u32 __iomem *pwmctrl_mmio;
union pwmctrl_reg {
        struct {
                u32 pwmtd:8;
                u32 pwmbu:22;
                u32 pwmswupdate:1;
                u32 pwmenable:1;
        } part;
        u32 full;
};
//backlight pwm from soc -
#endif

extern int lp8557i_enable_backlight(void);
extern int lp8557i_disable_backlight(void);
int boe_lp8557i_enable;

/*
Cadiz register settings:
(using Cadiz_register_generator_ver1.09_0515.xlsm)
Resolution 800x1280
HSYNC      16
HBP        48
HACT       800
HFP        16
VSYNC      4
VBP        12
VACT       1280
VFP        16

LANE            4
HS bitrate      418
Rx sync mode    Event
Tx sync mode    Event
Tx EOT          0
TLPX                    2(55.78ns)
THS-PREPARE             0(108.80ns)
TCLK-PREPARE            0(72.92ns)
THS-ZERO                12(192.20ns)
TCLK-ZERO               AUTO(328.57ns)

Autual:
THS-PREPARE              108.80
TCLK-PREPARE             72.92
THS-PREPARE + THS-ZERO   301.00
TCLK-PREPARE + TCLK-ZERO 401.48

SYSCLK[MHz]     19.2
Modesel0        1

IPC             ON
COM Unsed Mode  Enable
ICE             ON
COM             OFF
SPC             ON
GRC             ON
PQ setting      STD_2
Effect Area     1

IBC                  ON
PWM frequency setting 1		2
PWM frequency			1200
PWM frequency			19.35kHz
------------
manual change
a. cadiz mipi tx discontinuous mode and EOTp:
   0x025C, 0x05 -> 0x06
*/
static struct reg_val boot1[] = {
	{0x0830,0x00},
	{0x0200,0x00},
	{0x0201,0x00},
	{0x0202,0x00},
	{0x0203,0x00},
	{0x0274,0x00},
	{0x0275,0x00},
	{0x0276,0x00},
	{0x0277,0x00},
	{0x0600,0x00},
	{0x0601,0x00},
	{0x0602,0x00},
	{0x0603,0x00},
	{0x0820,0x32},
	{0x0821,0x04},
	{0x0822,0x33},
	{0x0823,0x33},
	{0x0824,0x33},
	{0x0825,0x03},
	{0x0826,0x00},
	{0x0827,0x00},
	{0x0840,0x40},
	{0x0841,0x7B},
	{0x0842,0x14},
	{0x0843,0x00},
	{0x0900,0x06},
	{0x0901,0x01},
	{0x0902,0x20},
	{0x0903,0x03},
	{0x0904,0x00},
	{0x0905,0x05},
	{0x0906,0x40},
	{0x0907,0x00},
	{0x0908,0x00},
	{0x0909,0x00},
	{0x090A,0x18},
	{0x090B,0x19},
	{0x090C,0x00},
	{0x090D,0x00},
	{0x090E,0x00},
	{0x090F,0x00},
	{0x0914,0x01},
	{0x0915,0x10},
	{0x0916,0xE0},
	{0x0917,0x06},
	{0x0940,0x05},
	{0x0941,0x00},
	{0x0942,0x06},
	{0x0943,0x00},
	{0x020C,0x00},
	{0x020D,0x62},
	{0x020E,0x00},
	{0x020F,0x00},
	{0x0220,0x20},
	{0x0221,0x03},
	{0x0222,0x00},
	{0x0223,0x05},
	{0x0228,0x0C},
	{0x0229,0x00},
	{0x022A,0x00},
	{0x022B,0x00},
	{0x022C,0x24},
	{0x022D,0x00},
	{0x022E,0x00},
	{0x022F,0x00},
	{0x0230,0x0C},
	{0x0231,0x00},
	{0x0232,0x00},
	{0x0233,0x00},
	{0x0234,0x58},
	{0x0235,0x02},
	{0x0236,0x00},
	{0x0237,0x00},
	{0x0238,0x04},
	{0x0239,0x00},
	{0x023A,0x00},
	{0x023B,0x00},
	{0x023C,0x0C},
	{0x023D,0x00},
	{0x023E,0x00},
	{0x023F,0x00},
	{0x0240,0x10},
	{0x0241,0x00},
	{0x0242,0x00},
	{0x0243,0x00},
	{0x0244,0x18},
	{0x0245,0x00},
	{0x0246,0x00},
	{0x0247,0x00},
	{0x0258,0x02},
	{0x0259,0x00},
	{0x025A,0x00},
	{0x025B,0x00},
	{0x025C,0x06}, //cadiz TX discontinuous, and EOT packet
	{0x025D,0x00},
	{0x025E,0x00},
	{0x025F,0x00},
	{0x0260,0x0C},
	{0x0261,0x00},
	{0x0262,0x00},
	{0x0263,0x00},
	{0x0264,0x0F},
	{0x0265,0x00},
	{0x0266,0x20},
	{0x0267,0x00},
	{0x0268,0x02},
	{0x0269,0x00},
	{0x026A,0x00},
	{0x026B,0x00},
	{0x026C,0x00},
	{0x026D,0x0C},
	{0x026E,0x04},
	{0x026F,0x08},
	{0x0270,0x00},
	{0x0271,0x13},
	{0x0272,0x04},
	{0x0273,0x08},
	{0x0278,0x1A},
	{0x0279,0x04},
	{0x027A,0x84},
	{0x027B,0x3B},
	{0x027C,0x46},
	{0x027D,0x38},
	{0x027E,0x32},
	{0x027F,0xB5},
	{0x0280,0x00},
	{0x0281,0x00},
	{0x0282,0x40},
	{0x0283,0x10},
	{0x0284,0x00},
	{0x0285,0x40},
	{0x0286,0x44},
	{0x0287,0x02},
	{0x02A8,0x01},
	{0x02A9,0x00},
	{0x02AA,0x00},
	{0x02AB,0x00},
	{0x000C,0x60},
	{0x000D,0x00},
	{0x000E,0x00},
	{0x000F,0x00},
	{0x0024,0x01},
	{0x0025,0x00},
	{0x0026,0x00},
	{0x0027,0x00},
	{0x002C,0x05},
	{0x002D,0xFF},
	{0x002E,0x00},
	{0x002F,0x00},
	{0x0034,0x1A},
	{0x0035,0x84},
	{0x0036,0x04},
	{0x0037,0x3A},
	{0x0038,0x46},
	{0x0039,0x38},
	{0x003A,0x32},
	{0x003B,0xB5},
	{0x003C,0x1C},
	{0x003D,0x04},
	{0x003E,0x00},
	{0x003F,0x00},
	{0x0040,0x00},
	{0x0041,0x00},
	{0x0042,0x44},
	{0x0043,0x02},
	{0x0058,0x10},
	{0x0059,0x00},
	{0x005A,0x04},
	{0x005B,0x00},
	{0x090C,0x00},
	{0x090D,0x00},
	{0x090E,0x00},
	{0x090F,0x07},
};

static struct reg_val boot2[] = {
	{0x001C,0x05},
	{0x001D,0x00},
	{0x001E,0x00},
	{0x001F,0x00},
	{0x0000,0x01},
	{0x0001,0x00},
	{0x0002,0x00},
	{0x0003,0x00},
	{0x0940,0x05},
	{0x0941,0x06},
	{0x0942,0x06},
	{0x0943,0x00},
};

static struct reg_val boot3[] = {
	{0x0274,0x01},
	{0x0200,0x01},
	{0x0006,0xFF},
};

static struct reg_val boot4[] = {
	{0x0000,0x01},
	{0x0941,0x07},
	{0x0941,0x01},
	{0x0005,0xFF},
};

static struct reg_val ipc_ibc[] = {
	{0x0C0A,0x00},
	{0x0C0B,0xFE},
	{0x0C14,0x00},
	{0x0C15,0x00},
	{0x0C16,0x00},
	{0x0C17,0x02},
	{0x0C20,0x00},
	{0x0C21,0x02},
	{0x0C22,0x00},
	{0x0C23,0x01},
	{0x0C34,0x10},
	{0x0C35,0x82},
	{0x0C36,0x05},
	{0x0C37,0x51},
	//{0x0C28,0x80},
	{0x0C29,0x00},
	{0x0C2A,0x02},
	{0x0C2B,0x00},
	{0x0C2C,0xB0},
	{0x0C2D,0x04},
	{0x0C2E,0x00},
	{0x0C2F,0x00},
	{0x0C30,0x00},
	{0x0C31,0x00},
	{0x0C32,0xC0},
	{0x0C33,0x12},
	{0x0A00,0x80},
	{0x0A01,0x00},
	{0x0A02,0x04},
	{0x0A03,0x00},
	{0x0A04,0x20},
	{0x0A05,0x03},
	{0x0A06,0x00},
	{0x0A07,0x00},
	{0x0A08,0x00},
	{0x0A09,0x05},
	{0x0A0A,0x00},
	{0x0A0B,0x00},
	{0x0A0C,0x00},
	{0x0A0D,0x00},
	{0x0A0E,0x00},
	{0x0A0F,0x00},
	{0x0A10,0x00},
	{0x0A11,0x00},
	{0x0A12,0x00},
	{0x0A13,0x88},
	{0x0A14,0x01},
	{0x0A15,0x00},
	{0x0A16,0x00},
	{0x0A17,0x00},
	{0x0A18,0x80},
	{0x0A19,0x80},
	{0x0A1A,0x80},
	{0x0A1B,0x80},
	{0x0A1C,0x00},
	{0x0A1D,0x00},
	{0x0A1E,0x02},
	{0x0A1F,0x08},
	{0x0A20,0x0A},
	{0x0A21,0x06},
	{0x0A22,0x1F},
	{0x0A23,0x3F},
	{0x0A24,0x16},
	{0x0A25,0x16},
	{0x0A26,0x04},
	{0x0A27,0x04},
	{0x0A28,0x0A},
	{0x0A29,0x00},
	{0x0A2A,0x04},
	{0x0A2B,0x04},
	{0x0A2C,0x04},
	{0x0A2D,0x04},
	{0x0A2E,0x02},
	{0x0A2F,0x04},
	{0x0A30,0x04},
	{0x0A31,0x01},
	{0x0A32,0x30},
	{0x0A33,0xFF},
	{0x0A34,0x80},
	{0x0A35,0x30},
	{0x0A36,0x37},
	{0x0A37,0xFF},
	{0x0A38,0x00},
	{0x0A39,0x00},
	{0x0A3A,0x68},
	{0x0A3B,0x00},
	{0x0A3C,0x58},
	{0x0A3D,0x02},
	{0x0A3E,0xA0},
	{0x0A3F,0x00},
	{0x0A40,0xC0},
	{0x0A41,0x03},
	{0x0A42,0x06},
	{0x0A43,0x00},
	{0x0A44,0x85},
	{0x0A45,0x00},
	{0x0A46,0x02},
	{0x0A47,0x00},
	{0x0A48,0xD5},
	{0x0A49,0x00},
	{0x0A4A,0x3A},
	{0x0A4B,0x00},
	{0x0A4C,0x93},
	{0x0A4D,0x04},
	{0x0A4E,0xEC},
	{0x0A4F,0x01},
	{0x0A50,0x33},
	{0x0A51,0x01},
	{0x0A52,0x78},
	{0x0A53,0x68},
	{0x0A54,0x2C},
	{0x0A55,0x71},
	{0x0A56,0x84},
	{0x0A57,0x03},
	{0x0A58,0x24},
	{0x0A59,0x00},
	{0x0A5A,0x00},
	{0x0A5B,0x67},
	{0x0A5C,0xC0},
	{0x0A5D,0x88},
	{0x0A5E,0xB6},
	{0x0A5F,0xAF},
	{0x0A60,0x00},
	{0x0A61,0xFF},
	{0x0A62,0xF0},
	{0x0A63,0x88},
	{0x0A64,0x22},
	{0x0A65,0x3A},
	{0x0A66,0xD6},
	{0x0A67,0xA7},
	{0x0A68,0x60},
	{0x0A69,0xC5},
	{0x0A6A,0xB0},
	{0x0A6B,0x20},
	{0x0A6C,0xCC},
	{0x0A6D,0x8A},
	{0x0A6E,0x8A},
	{0x0A6F,0x83},
	{0x0A70,0x84},
	{0x0A71,0x86},
	{0x0A72,0x89},
	{0x0A73,0x83},
	{0x0A74,0x91},
	{0x0A75,0x8B},
	{0x0A76,0x01},
	{0x0A77,0x00},
	{0x0A80,0x00},
	{0x0A81,0x00},
	{0x0A82,0x04},
	{0x0A83,0x00},
	{0x0A84,0x20},
	{0x0A85,0x03},
	{0x0A86,0x04},
	{0x0A87,0x00},
	{0x0A88,0x00},
	{0x0A89,0x05},
	{0x0A8A,0x60},
	{0x0A8B,0x00},
	{0x0A8C,0x65},
	{0x0A8D,0x04},
	{0x0A8E,0x42},
	{0x0A8F,0x00},
	{0x0A90,0x09},
	{0x0A91,0x3D},
	{0x0A92,0x00},
	{0x0A93,0x00},
	{0x0A94,0x20},
	{0x0A95,0x00},
	{0x0A96,0x64},
	{0x0A97,0x00},
	{0x0A98,0xA0},
	{0x0A99,0x00},
	{0x0A9A,0x00},
	{0x0A9B,0x10},
	{0x0A9C,0x00},
	{0x0A9D,0x00},
	{0x0A9E,0x01},
	{0x0A9F,0x00},
	{0x0AA0,0xF5},
	{0x0AA1,0x04},
	{0x0AA2,0x08},
	{0x0AA3,0x00},
	{0x0AA8,0x00},
	{0x0AA9,0x00},
	{0x0AAA,0x78},
	{0x0AAB,0x00},
	{0x0AAC,0x30},
	{0x0AAD,0x02},
	{0x0AAE,0x26},
	{0x0AAF,0x00},
	{0x0AB0,0x80},
	{0x0AB1,0x04},
	{0x0AB2,0x00},
	{0x0AB3,0x00},
	{0x0AB4,0x00},
	{0x0AB5,0x2A},
	{0x0AB6,0x00},
	{0x0AB7,0x00},
	{0x0AB8,0x21},
	{0x0AB9,0x3F},
	{0x0ABA,0x00},
	{0x0ABB,0x01},
	{0x0ABC,0x0F},
	{0x0ABD,0x80},
	{0x0ABE,0xB8},
	{0x0ABF,0x1F},
	{0x0AC0,0xC0},
	{0x0AC1,0x64},
	{0x0AC2,0x02},
	{0x0AC3,0x80},
	{0x0AC4,0x26},
	{0x0AC5,0x00},
	{0x0AC6,0x88},
	{0x0AC7,0x20},
	{0x0AC8,0x00},
	{0x0AC9,0x40},
	{0x0ACA,0x00},
	{0x0ACB,0x3F},
	{0x0ACC,0x00},
	{0x0ACD,0x43},
	{0x0ACE,0x00},
	{0x0ACF,0x4A},
	{0x0AD0,0x20},
	{0x0AD1,0x33},
	{0x0AD2,0x7F},
	{0x0AD3,0x96},
	{0x0AD4,0x06},
	{0x0AD5,0x0F},
	{0x0AD6,0x21},
	{0x0AD7,0x28},
	{0x0AD8,0x2C},
	{0x0AD9,0x7F},
	{0x0ADA,0xC8},
	{0x0ADB,0x96},
	{0x0ADC,0x0F},
	{0x0ADD,0x19},
	{0x0ADE,0x28},
	{0x0ADF,0x2C},
	{0x0AE0,0x00},
	{0x0AE1,0x00},
	{0x0AE2,0x00},
	{0x0AE3,0x00},
	{0x0AFC,0x00},
	{0x0AFD,0x00},
	{0x0AFE,0x00},
	{0x0AFF,0x08},
};

/*
from CalculateDPHY.apk

HActive 800
Hsync 16
HBP 48
HFP 16

VActive 1280
Vsync 4
VBP 12
VFP 16

Horizontal image size 800
Vertical image size 1280

Frame Rate 60
Number of Lane 4

THsPrepare 77
THsPrepareThsZero 173
TClkPrepareTClkZero 300
TClkTrail 60
*/
static void
boe_tv101wxmnu1_vid_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx;

	pr_info("%s +\n", __func__);

	if (!dsi_config || !(&dsi_config->dsi_hw_context)) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	hw_ctx = &dsi_config->dsi_hw_context;
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xFFFF;
	hw_ctx->turn_around_timeout = 0x3F;
	//hw_ctx->high_low_switch_count = 0x1a;
	hw_ctx->high_low_switch_count = 0x17;
	//hw_ctx->clk_lane_switch_time_cnt = 0x21000e;
	hw_ctx->clk_lane_switch_time_cnt = 0x1e000d;
	hw_ctx->lp_byteclk = 0x3;
	//hw_ctx->dphy_param = 0x170e3412;
	hw_ctx->dphy_param = 0x140d2f11;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dsi_func_prg = (RGB_888_FMT << FMT_DPI_POS) |
		dsi_config->lane_count;
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xE;

	pr_info("%s -\n", __func__);
}

static int boe_tv101wxmnu1_vid_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	pr_info("%s +\n", __func__);

	if (!panel_en_gpio || !panel_reset_gpio)
		return -EINVAL;

	gpio_set_value_cansleep(panel_en_gpio, 0);
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	gpio_set_value_cansleep(panel_en_gpio, 1);

	pr_info("%s -\n", __func__);

	return 0;
}

static int boe_tv101wxmnu1_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 data;

	pr_info("%s +\n", __func__);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	mdfld_dsi_read_mcs_lp(sender, 0x0A, &data, 1);

	usleep_range(1000, 1100);
	//cadiz_power(1); //same gpio as panel en
	cadiz_reset(1);
	usleep_range(200, 210);  //over 100us
	cadiz_enable_i2c();
	cadiz_wait_tx_init_done();

	usleep_range(10000,10500); //10ms
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(100, 150); //100us
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(100, 150); //100us
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(100000, 105000); //100ms FIXME:need this?

	//LP command ++
	//mipi settle time +
	{
		u8 buf[] = {0xFF, 0xAA, 0x55, 0x25, 0x01}; //RFF AA 55 25 01
		mdfld_dsi_send_gen_long_lp(sender, buf, 5,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xFC, 0x08, 2, //RFC 08
			MDFLD_DSI_SEND_PACKAGE);

	usleep_range(1000, 1100); //delay 1

	mdfld_dsi_send_gen_short_lp(sender, 0xFC, 0x00, 2, //RFC 00
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xFF, 0xAA, 0x55, 0x25, 0x00}; //RFF AA 55 25 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 5,
				MDFLD_DSI_SEND_PACKAGE);
	}

	usleep_range(20000, 20500); //delay 20

	//mipi settle time -
	//page0 +
	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}; //RF0 55 AA 52 08 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB1, 0x68, 0x01};
		mdfld_dsi_send_gen_long_lp(sender, buf, 3, //RB1 68 01
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB8, 0x01, 0x02, 0x02, 0x02}; //RB8 01 02 02 02
		mdfld_dsi_send_gen_long_lp(sender, buf, 5,
				MDFLD_DSI_SEND_PACKAGE);
	} //EQ control function for source

	mdfld_dsi_send_gen_short_lp(sender, 0xB6, 0x11, 2, //RB6 11
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xBB, 0x63, 0x63}; //RBB 63 63
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	} //source driver control

	{
		u8 buf[] = {0xBC, 0x00, 0x00}; //RBC 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBD, 0x02, 0xBB, 0x11, 0x10, 0x00}; //RBD 02 BB 11 10 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xC8, 0x80, 2, //RC8 80
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01}; //RF0 55 AA 52 08 01
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB3, 0x28, 0x28}; //RB3 28 28
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB4, 0x14, 0x14}; //RB4 14 14
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB9, 0x44, 0x44}; //RB9 44 44
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBA, 0x24, 0x24}; //RBA 24 24
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBC, 0x70, 0x00}; //RBC 70 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBD, 0x70, 0x00}; //RBD 70 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x02}; //RF0 55 AA 52 08 02
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}


	mdfld_dsi_send_gen_short_lp(sender, 0xEE, 0x01, 2, //REE 01
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xB0, 0x00, 0x16, 0x00, 0x26, 0x00, //RB0 00 16 00 26 00 45 00 5F 00 72 00 96 00 B3 00 E3
			    0x45, 0x00, 0x5F, 0x00, 0x72, 0x00,
			    0x96, 0x00, 0xB3, 0x00, 0xE3};
		mdfld_dsi_send_gen_long_lp(sender, buf, 17,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB1, 0x01, 0x0A, 0x01, 0x47, 0x01, //RB1 01 0A 01 47 01 77 01 C3 01 FF 02 00 02 36 02 70
			    0x77, 0x01, 0xC3, 0x01, 0xFF, 0x02,
			    0x00, 0x02, 0x36, 0x02, 0x70};
		mdfld_dsi_send_gen_long_lp(sender, buf, 17,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB2, 0x02, 0x93, 0x02, 0xC8, 0x02, //RB2 02 93 02 C8 02 EB 03 19 03 37 03 5D 03 71 03 8D
			    0xEB, 0x03, 0x19, 0x03, 0x37, 0x03,
			    0x5D, 0x03, 0x71, 0x03, 0x8D};
		mdfld_dsi_send_gen_long_lp(sender, buf, 17,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB3, 0x03, 0xBA, 0x03, 0xD0}; //RB3 03 BA 03 D0
		mdfld_dsi_send_gen_long_lp(sender, buf, 5,
				MDFLD_DSI_SEND_PACKAGE);
	}


	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x03}; //RF0 55 AA 52 08 03
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB0, 0x00, 0x00}; //RB0 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB1, 0x00, 0x00}; //RB1 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB2, 0x05, 0x01, 0x55, 0x00, 0x1A}; //RB2 05 01 55 00 1A
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB3, 0x05, 0x01, 0x55, 0x00, 0x1A}; //RB3 05 01 55 00 1A
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBA, 0x53, 0x01, 0x55, 0x00, 0x3A}; //RBA 53 01 55 00 3A
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xC4, 0x60, 2, //RC4 60
			MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_short_lp(sender, 0xC5, 0x40, 2, //RC5 40
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}; //RF0 55 AA 52 08 04
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB1, 0x03, 0x02, 0x05, 0x07}; //RB1 03 02 05 07
		mdfld_dsi_send_gen_long_lp(sender, buf, 5,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x05}; //RF0 55 AA 52 08 05
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBD, 0x01, 0x03, 0x00, 0x00, 0x03}; //RBD 01 03 00 00 03
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xC0, 0x09, 2, //RC0 09
			MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_short_lp(sender, 0xC1, 0x07, 2, //RC1 07
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xC8, 0x07, 0x38}; //RC8 07 38
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xD0, 0x00, 2, //RD0 00
			MDFLD_DSI_SEND_PACKAGE);

	{
		u8 buf[] = {0xD1, 0x00, 0x05, 0x01, 0x00, 0x00}; //RD1 00 05 01 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0xE5, 0x03, 2, //RE5 03
			MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_short_lp(sender, 0xE6, 0x03, 2, //RE6 03
			MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_short_lp(sender, 0xE9, 0x03, 2, //RE9 03
			MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_gen_short_lp(sender, 0xEA, 0x03, 2, //REA 03
			MDFLD_DSI_SEND_PACKAGE);


	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x06}; //RF0 55 AA 52 08 06
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB0, 0x2D, 0x2D}; //RB0 2D 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB1, 0x01, 0x01}; //RB1 01 01
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB2, 0x03, 0x03}; //RB2 03 03
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB3, 0x2E, 0x2E}; //RB3 2E 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB4, 0x2E, 0x2D}; //RB4 2E 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB5, 0x2D, 0x13}; //RB5 2D 13
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB6, 0x13, 0x11}; //RB6 13 11
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB7, 0x11, 0x36}; //RB7 11 36
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB8, 0x36, 0x15}; //RB8 36 15
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xB9, 0x15, 0x2E}; //RB9 15 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBA, 0x2E, 0x14}; //RBA 2E 14
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBB, 0x14, 0x35}; //RBB 14 35
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBC, 0x35, 0x10}; //RBC 35 10
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBD, 0x10, 0x12}; //RBD 10 12
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBE, 0x12, 0x2D}; //RBE 12 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xBF, 0x2D, 0x2E}; //RBF 2D 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC0, 0x2E, 0x2E}; //RC0 2E 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC1, 0x02, 0x02}; //RC1 02 02
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC2, 0x00, 0x00}; //RC2 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC3, 0x2D, 0x2D}; //RC3 2D 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xE5, 0x34, 0x34}; //RE5 34 34
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC4, 0x2E, 0x2E}; //RC4 2E 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC5, 0x02, 0x02}; //RC5 02 02
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC6, 0x00, 0x00}; //RC6 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC7, 0x2D, 0x2D}; //RC7 2D 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC8, 0x2E, 0x2D}; //RC8 2E 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xC9, 0x2D, 0x14}; //RC9 2D 14
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCA, 0x14, 0x35}; //RCA 14 35
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCB, 0x35, 0x10}; //RCB 35 10
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCC, 0x10, 0x12}; //RCC 10 12
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCD, 0x12, 0x2E}; //RCD 12 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCE, 0x2E, 0x13}; //RCE 2E 13
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xCF, 0x13, 0x11}; //RCF 13 11
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD0, 0x11, 0x36}; //RD0 11 36
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD1, 0x36, 0x15}; //RD1 36 15
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD2, 0x15, 0x2D}; //RD2 15 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD3, 0x2D, 0x2E}; //RD3 2D 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD4, 0x2D, 0x2D}; //RD4 2D 2D
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD5, 0x01, 0x01}; //RD5 01 01
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD6, 0x03, 0x03}; //RD6 03 03
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD7, 0x2E, 0x2E}; //RD7 2E 2E
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xE6, 0x34, 0x34}; //RE6 34 34
		mdfld_dsi_send_gen_long_lp(sender, buf, 3,
				MDFLD_DSI_SEND_PACKAGE);
	}
	{
		u8 buf[] = {0xD8, 0x00, 0x00, 0x00, 0x00, 0x00}; //RD8 00 00 00 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xD9, 0x00, 0x00, 0x00, 0x00, 0x00}; //RD9 00 00 00 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x00, 0x00}; //RF0 55 AA 52 00 00
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	}

	mdfld_dsi_send_gen_short_lp(sender, 0x35, 0x00, 2, //R35 00
			MDFLD_DSI_SEND_PACKAGE);


	//bist +
	if(bist_enable) {
		{
			u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00};
			mdfld_dsi_send_gen_long_lp(sender, buf, 6,
					MDFLD_DSI_SEND_PACKAGE);
		}

		{
			u8 buf[] = {0xEF, 0x00, 0x01};
			mdfld_dsi_send_gen_long_lp(sender, buf, 3,
					MDFLD_DSI_SEND_PACKAGE);
		}

		{
			u8 buf[] = {0xEE, 0x87, 0x78, 0x02, 0x40};
			mdfld_dsi_send_gen_long_lp(sender, buf, 5,
					MDFLD_DSI_SEND_PACKAGE);
		}
	}
	//bist -

	pr_info("%s -\n", __func__);
	return 0;

}

static int boe_tv101wxmnu1_vid_power_on(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct drm_device *dev = dsi_config->dev;
	int offset = 0;

	pr_info("%s +\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_gen_short_lp(sender, MIPI_DCS_EXIT_SLEEP_MODE, 0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	usleep_range(150000, 155000); //150ms
	mdfld_dsi_send_gen_short_lp(sender, MIPI_DCS_SET_DISPLAY_ON, 0, 1,
			MDFLD_DSI_SEND_PACKAGE);

	//mdfld_dsi_send_gen_short_lp(sender, 0x35, 0x00, 2,
			//MDFLD_DSI_SEND_PACKAGE); //te

	{
		u8 buf[] = {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x01};
		mdfld_dsi_send_gen_long_lp(sender, buf, 6,
				MDFLD_DSI_SEND_PACKAGE);
	} //nt35521s_lock

	//for backlight +
	if(!cadiz_exist) {
		//mdfld_dsi_send_gen_short_lp(sender, 0x51, 0xFF, 2,
		//		MDFLD_DSI_SEND_PACKAGE);

		mdfld_dsi_send_gen_short_lp(sender, 0x53, 0x2C, 2,
				MDFLD_DSI_SEND_PACKAGE);

		mdfld_dsi_send_gen_short_lp(sender, 0x55, 0x01, 2,
				MDFLD_DSI_SEND_PACKAGE);
	}
	//for backlight -
	cadiz_wait_trans_complete();
	cadiz_setup_boot1();
	cadiz_setup_ipc_ibc();
	cadiz_setup_boot2();

	REG_WRITE(regs->eot_disable_reg + offset,
			(REG_READ(regs->eot_disable_reg) & ~DSI_EOT_DISABLE_MASK) |
			(0x1 & DSI_EOT_DISABLE_MASK));

	usleep_range(500, 550); //0.5ms
	pr_info("%s -\n", __func__);
	return 0;
}

static int boe_tv101wxmnu1_vid_set_panel_mode(struct mdfld_dsi_config *dsi_config)
{
	pr_info("%s +\n", __func__);
	usleep_range(2000, 2100); //2ms: ensure boot3 transftered after mipi hs mode
	cadiz_setup_boot3();
	cadiz_wait_tx_init_done();
	cadiz_setup_boot4();
	cadiz_set_accessible(1);
	pr_info("%s -\n", __func__);
}

static int boe_tv101wxmnu1_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	pr_info("%s +\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_gen_short_lp(sender, MIPI_DCS_SET_DISPLAY_OFF, 0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	usleep_range(200000, 205000); //200ms TODO:can trim off?
	mdfld_dsi_send_gen_short_lp(sender, MIPI_DCS_ENTER_SLEEP_MODE, 0, 1,
			MDFLD_DSI_SEND_PACKAGE);

	cadiz_set_accessible(0);
	usleep_range(130000, 135000); //130ms
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(100, 110); //100us
	cadiz_reset(0);
	usleep_range(1000, 1100); //1ms
	gpio_set_value_cansleep(panel_en_gpio, 0);


	pr_info("%s -\n", __func__);
	return 0;
}

static int
boe_tv101wxmnu1_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
                                      int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

#if !USE_CADIZ_PWM
	union pwmctrl_reg pwmctrl;
	u32 reg_level;
#endif
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if(level && !boe_lp8557i_enable) {
		if (!lp8557i_enable_backlight()) {
			boe_lp8557i_enable = 1;
			pr_info("Resume (Display On) took\n");
		}
	}

#if USE_CADIZ_PWM
	u8 adj_level;
	static u8 last_adj_level = 0;
	int tmp;
	struct reg_val cadiz_reg;

	//rated current of led backlight is 21mA and is reached
	//when brightness vlaue is 231, so it needs to map from
	//0 ~ 255 to 0 ~ 231.
	level = level * 231 / 255;

	if(cadiz_exist) {
		//cadiz brightness range is 0 ~ 127, and
		//keep max value to 115(0x73)
		adj_level = (u8)((double)level * 127 / 255);

		if(last_adj_level == adj_level)
			return 0;
		last_adj_level = adj_level;

		pr_info("panel bv=%d, cv=0x%x\n", level, adj_level);
		cadiz_reg.reg = 0x0C28;
		cadiz_reg.val = adj_level;
		cadiz_setup_regs(&cadiz_reg, 1);
	} else {
		adj_level = 0xFF & level;
		pr_info("panel bv=%d\n", level);
		mdfld_dsi_send_gen_short_lp(sender, 0x51, adj_level, 2,
				MDFLD_DSI_SEND_PACKAGE);
	}
#else
	reg_level = ~level & 0xFF;

	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = 0x1000;
	pwmctrl.part.pwmtd = reg_level;
	pr_info("brightness level=%d, reg_level=0x%x\n", level, reg_level);
	if (!pwmctrl_mmio)
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, 4);

	if (pwmctrl_mmio) {
		if (level) {
			lnw_gpio_set_alt(bl_pwm_gpio, 1);
			//pr_info("PWM enable\n");

			pwmctrl.part.pwmenable = 0x1;
			writel(pwmctrl.full, pwmctrl_mmio);
		} else {
			pwmctrl.part.pwmenable = 0;
			writel(pwmctrl.full, pwmctrl_mmio);
			lnw_gpio_set_alt(bl_pwm_gpio, 0);
			//pr_info("PWM disable\n");
		}
	} else {
		DRM_ERROR("Cannot map pwmctrl\n");
	}
#endif

	if(!level && boe_lp8557i_enable) {
		if(!lp8557i_disable_backlight())
			boe_lp8557i_enable = 0;
	}

	return 0;
}

static void
boe_tv101wxmnu1_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = TV101WXMNU1_WIDTH;
	pi->height_mm = TV101WXMNU1_HEIGHT;
}


static struct drm_display_mode *boe_tv101wxmnu1_vid_get_config_mode(void)
{
	struct drm_display_mode *mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;
	mode->hsync_start = mode->hdisplay + 16; //HFP 16
	mode->hsync_end = mode->hsync_start + 16; //HSYNC 16
	mode->htotal = mode->hsync_end + 48; //HBP 48

	mode->vdisplay = 1280;
	mode->vsync_start = mode->vdisplay + 16; //VFP 16
	mode->vsync_end = mode->vsync_start + 4; //VSYNC 4
	mode->vtotal = mode->vsync_end + 12; //VBP 12

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	return mode;
}

static int boe_tv101wxmnu1_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int reinit;

	pr_info("%s +\n", __func__);

	reinit = cadiz_need_reinit();

	if (!panel_en_gpio) {
		panel_en_gpio = TV101WXMNU1_AVDD_GPIO;
		if (gpio_request(panel_en_gpio, "panel_en")) {
			DRM_ERROR("Faild to request panel enable gpio\n");
			return -EINVAL;
		}
		if(reinit)
			gpio_direction_output(panel_en_gpio, 0);
		else
			gpio_direction_output(panel_en_gpio, 1);
	}

	if (!panel_reset_gpio) {
		panel_reset_gpio = get_gpio_by_name("disp0_rst");
		if (panel_reset_gpio < 0) {
			DRM_ERROR("Faild to get panel reset gpio\n");
			return -EINVAL;
		}

		if (gpio_request(panel_reset_gpio, "panel_reset")) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return -EINVAL;
		}
		if(reinit)
			gpio_direction_output(panel_reset_gpio, 0);
		else
			gpio_direction_output(panel_reset_gpio, 1);
	}
#if !USE_CADIZ_PWM
	if (!bl_pwm_gpio) {
		bl_pwm_gpio = TV101WXMNU1_BL_PWM_GPIO;
		if (gpio_request(bl_pwm_gpio, "backlight_pwm"))
			DRM_ERROR("Faild to request backlight PWM gpio\n");

		lnw_gpio_set_alt(bl_pwm_gpio, 1);
	}
	pmu_set_pwm(PCI_D0);
#endif

	cadiz_exist  = asustek_get_cardiz_id();

	cadiz_pass_tables(boot1, sizeof(boot1)/sizeof(boot1[0]),
			  boot2, sizeof(boot2)/sizeof(boot2[0]),
			  boot3, sizeof(boot3)/sizeof(boot3[0]),
			  boot4, sizeof(boot4)/sizeof(boot4[0]),
			  ipc_ibc, sizeof(ipc_ibc)/sizeof(ipc_ibc[0]));
	if (!reinit)
		dsi_config->dsi_hw_context.panel_on = true;

	pr_info("%s -\n", __func__);
	return MDFLD_DSI_PANEL_CONNECTED;
}

void boe_tv101wxmnu1_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	panel_en_gpio = 0;
	panel_reset_gpio = 0;
#if !USE_CADIZ_PWM
	bl_pwm_gpio = 0;
#endif
	bist_enable = 0;
	boe_lp8557i_enable = 0;

	p_funcs->reset = boe_tv101wxmnu1_vid_panel_reset;
	p_funcs->power_on = boe_tv101wxmnu1_vid_power_on;
	p_funcs->power_off = boe_tv101wxmnu1_vid_power_off;
	p_funcs->drv_ic_init = boe_tv101wxmnu1_vid_drv_ic_init;
	p_funcs->get_config_mode = boe_tv101wxmnu1_vid_get_config_mode;
	p_funcs->get_panel_info = boe_tv101wxmnu1_vid_get_panel_info;
	p_funcs->dsi_controller_init = boe_tv101wxmnu1_vid_controller_init;
	p_funcs->set_brightness = boe_tv101wxmnu1_vid_set_brightness;
	p_funcs->detect = boe_tv101wxmnu1_vid_detect;
	p_funcs->drv_set_panel_mode = boe_tv101wxmnu1_vid_set_panel_mode;

	struct dentry *dent = debugfs_create_dir("display_bist", NULL);
	debugfs_create_u32("on", 0777, dent, &bist_enable);
}
