/*
 *  linux/arch/arm/mach-mmp/aspenite.c
 *
 *  Support for the Marvell PXA168-based Aspenite and Zylonite2
 *  Development Platform.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/smc91x.h>
#include <linux/i2c/pca953x.h>
#include <linux/card.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/mfd/stmpe.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <mach/addr-map.h>
#include <mach/mfp-pxa168.h>
#include <mach/pxa168.h>
#include <mach/gpio.h>
#include <mach/max8660.h>
#include <mach/pxa3xx_nand.h>
#include <mach/camera.h>
#include <mach/pxa168_eth.h>
#include <mach/pxa168_pcie.h>
#include <mach/cputype.h>

#include <plat/part_table.h>
#include <plat/generic.h>
#include <plat/pxa3xx_pmic.h>
#include <plat/pxa_u2o.h>
#include <plat/pxa3xx_otg.h>

#if defined(CONFIG_SPI_PXA2XX)
#include <linux/spi/spi.h>
#include <plat/pxa2xx_spi.h>
#endif

#include "common.h"
#include <linux/mmc/sdhci.h>
#include <plat/pfn_cfg.h>

/*used by expander max7312, 16 pins gpio expander */
#define GPIO_EXT0(x)		(NR_BUILTIN_GPIO + (x))
#define GPIO_EXT1(x)		(NR_BUILTIN_GPIO + 16 + (x))
#define GPIO_EXT2(x)		(NR_BUILTIN_GPIO + 16 + 16 + (x))

#define CARD_EN GPIO_EXT1(0)
#define CAM_PWDN GPIO_EXT1(1)
#define TW9907_PWDN GPIO_EXT1(4)
#define TW9907_RST_N GPIO_EXT1(2)

#define USB_WIFI_GPIO 101

static unsigned long kovan_pin_config[] __initdata = {
	// Note that the range of GPIO0-GPIO09 conflicts with MMC3
	// See below for definition of MMC3
	#if !defined(CONFIG_CHUMBY_SILVERMOON_SDBOOT) && !defined(CONFIG_MMC3)
	/* Data Flash Interface */
	GPIO0_DFI_D15,
	GPIO1_DFI_D14,
	GPIO2_DFI_D13,
	GPIO3_DFI_D12,
	GPIO4_DFI_D11,
	GPIO5_DFI_D10,
	GPIO6_DFI_D9,
	GPIO7_DFI_D8,
	GPIO8_DFI_D7,
	GPIO9_DFI_D6,
	GPIO10_DFI_D5,
	GPIO11_DFI_D4,
	GPIO12_DFI_D3,
	GPIO13_DFI_D2,
	GPIO14_DFI_D1,
	GPIO15_DFI_D0,
	#endif

	#if defined(CONFIG_MMC3)
	GPIO0_MMC3_DAT7,
	GPIO1_MMC3_DAT6,
	GPIO2_MMC3_DAT5,
	GPIO3_MMC3_DAT4,
	GPIO4_MMC3_DAT3,
	GPIO5_MMC3_DAT2,
	GPIO6_MMC3_DAT1,
	GPIO7_MMC3_DAT0,
	GPIO8_MMC3_CLK,
	GPIO9_MMC3_CMD,
	GPIO16_SMC_nCS0_DIS,
	#endif

	/* Static Memory Controller */
	#ifndef CONFIG_PXA168_CF
	GPIO18_SMC_nCS0,
	GPIO34_SMC_nCS1,
	GPIO23_SMC_nLUA,
	GPIO25_SMC_nLLA,
	GPIO28_SMC_RDY,
	GPIO29_SMC_SCLK,
	GPIO35_SMC_BE1,
	GPIO36_SMC_BE2,
	GPIO27_GPIO,    /* Ethernet IRQ */
	#endif


	/* LCD */
	MFP_CFG_DRV_PULL(GPIO56, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO57, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO58, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO59, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO60, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO61, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO62, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO63, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO64, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO65, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO66, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO67, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO68, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO69, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO70, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO71, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO72, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO73, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO74, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO75, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO76, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO77, AF1, VERY_SLOW, LOW),
	MFP_CFG_DRV_PULL(GPIO78, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO79, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO80, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO81, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO82, AF1, FAST, LOW),
	MFP_CFG_DRV_PULL(GPIO83, AF1, FAST, LOW),


	//CSM_GPIO85_XotgDRV_VBUS,


	/* i2c bus */
	GPIO105_CI2C_SDA,
	GPIO106_CI2C_SCL,

	/* Added to ensure UART1 input works */
	MFP_CFG(GPIO109, AF0),

	/* SSP0 */
	GPIO113_I2S_MCLK, // and this is in fact the I2S audio output to the FPGA
	GPIO114_I2S_FRM,
	GPIO115_I2S_BCLK,
	GPIO116_I2S_RXD,

	MFP_CFG(GPIO117, AF0),

	/* Configure FPGA pins */
	MFP_CFG(GPIO91, AF0), /* HPD report */
	MFP_CFG(GPIO92, AF0), /* Key ready */
	MFP_CFG(GPIO93, AF0), /* Low-voltage alarm */


	/* Turn on power to LCD */
	//CSM_GPIO84_LCD_PWM,

	#if defined(CONFIG_LEDS_NETV) || defined(CONFIG_LEDS_NETV_MODULE)
	/* Set GPIO96 as PWM2 */
	MFP_CFG(GPIO96, AF1),
	#endif

	//CSM_GPIO118_TS_SCLK,
	MFP_CFG(GPIO119, AF0), // fpga_reset_n (output)
	MFP_CFG(GPIO120, AF0), // fpga_init_n (input, mostly)
	//CSM_GPIO121_TS_MOSI, // fpga_din (output)
	MFP_CFG(GPIO97, AF0), // fpga_done (input)

	/* Recovery button */
	MFP_CFG(GPIO89, AF0),

	/* vsync input */
	MFP_CFG(GPIO49, AF0),

	/* LED outputs */
	MFP_CFG(GPIO45, AF0),
	MFP_CFG(GPIO46, AF0),

	/* Touchscreen */
	MFP_CFG(GPIO52, AF0),
};

struct platform_device kovan_device_battery = {
	.name		= "kovan-battery",
	.id		= -1,
};

static inline void kovan_add_battery(void)
{
	int ret;
        ret = platform_device_register(&kovan_device_battery);
	if (ret)
		dev_err(&kovan_device_battery.dev,
			"unable to register device: %d\n", ret);
}

/*
 * STMPE610
 */
static struct stmpe_ts_platform_data stmpe610_ts_data = {
	.sample_time = 4,	/* 80 clocks */
	.mod_12b = 0,		/* 10-bit mode */
	.ref_sel = 0,		/* Internal reference */
	.touch_det_delay = 3,	/* 500 uS */
	.settling = 2,		/* 500 uS settling time */
	.fraction_z = 7,	/* Length of the fractional part */
	.i_drive = 0,		/* 20 mA drive */
};

static struct stmpe_platform_data stmpe610_data = {
	.id		= 1,
	.blocks		= STMPE_BLOCK_TOUCHSCREEN,
	.irq_trigger	= IRQF_TRIGGER_FALLING,
	.irq_gpio	= 52,
	.irq_over_gpio	= true,
	.ts		= &stmpe610_ts_data,
};



#if defined(CONFIG_PCI)



/* #define ASPENITE_REV5 */
#if !defined(ASPENITE_REV5)

#define PCIE_MINICARD_CD_N      GPIO_EXT2(0)
#define PCIE_1P5V_SHDN_N        GPIO_EXT2(1)
#define PCIE_3P3V_SHDN_N        GPIO_EXT2(2)
#define PCIE_MINICARD_PERST_N   GPIO_EXT2(3)
#define PCIE_MINICARD_WAKE_N    GPIO_EXT2(4)
#define PCIE_MINICARD_CLKREQ_N  GPIO_EXT2(5)
#define PCIE_REFCLK_OE          GPIO_EXT2(6)

#define PCIE_CARD_DECTECT       GPIO_EXT2(0)


static unsigned int pcie_card_inserted(void)
{
	int ret = 0;
	int res;

	res = gpio_get_value_cansleep(PCIE_CARD_DECTECT);
	if (!(res < 0)) {

		/*
		 * Check input reg (0x00) bit 0
		 *    0 = card is inserted
		 *    1 = card is not inserted
		 */
		ret = !res;
	}

	return ret;
}

int pxa168_gpio_pcie_init(void)
{
	/* Card inserted? */
	if (!pcie_card_inserted()) {
		printk(KERN_ERR "pcie: No card detected.\n");
		return -EIO;
	}

	if (gpio_request(PCIE_1P5V_SHDN_N, "PCIE_1P5V_SHDN_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_1P5V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_3P3V_SHDN_N, "PCIE_3P3V_SHDN_N")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_3P3V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_REFCLK_OE, "PCIE_REFCLK_OE")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		gpio_free(PCIE_3P3V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_REFCLK_OE);
		return -EIO;
	}

	if (gpio_request(PCIE_MINICARD_PERST_N, "PCIE_MINICARD_PERST_N")) {
		gpio_free(PCIE_1P5V_SHDN_N);
		gpio_free(PCIE_3P3V_SHDN_N);
		gpio_free(PCIE_REFCLK_OE);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_MINICARD_PERST_N);
		return -EIO;
	}

	if (gpio_direction_output(PCIE_MINICARD_PERST_N, 0))
		return -EIO;
	if (gpio_direction_output(PCIE_1P5V_SHDN_N, 1))
		return -EIO;
	if (gpio_direction_output(PCIE_1P5V_SHDN_N, 1))
		return -EIO;
	if (gpio_direction_output(PCIE_3P3V_SHDN_N, 1))
		return -EIO;
	/* wait for power supply to stabilize */
	mdelay(2);
	if (gpio_direction_output(PCIE_REFCLK_OE, 1))
		return -EIO;
	/* 4ms: PCIClock output to stabilize +
	 * 96ms: Tpvperl pr PCISIG Base1.0a design checklist
	 */
	mdelay(100);
	if (gpio_direction_output(PCIE_MINICARD_PERST_N, 1))
		return -EIO;

	gpio_free(PCIE_1P5V_SHDN_N);
	gpio_free(PCIE_3P3V_SHDN_N);
	gpio_free(PCIE_REFCLK_OE);
	gpio_free(PCIE_MINICARD_PERST_N);

	return 0;
}

#else /* Rev 5 */

#define PCIE_3P3V_SHDN_N   GPIO_EXT2(2)
#define PCIE_PRSNT2_N      GPIO_EXT2(3)
#define PCIE_WAKE_N        GPIO_EXT2(4)
#define PCIE_PWRGD         GPIO_EXT2(5)
#define PCIE_REFCLK_OE     GPIO_EXT2(6)

int pxa168_gpio_pcie_init(void)
{

	if (gpio_request(PCIE_3P3V_SHDN_N, "PCIE_3P3V_SHDN_N")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_3P3V_SHDN_N);
		return -EIO;
	}

	if (gpio_request(PCIE_PWRGD, "PCIE_PWRGD")) {
		gpio_free(PCIE_3P3V_SHDN_N);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", PCIE_PWRGD);
		return -EIO;
	}

	if (gpio_request(PCIE_REFCLK_OE, "PCIE_REFCLK_OE")) {
		gpio_free(PCIE_3P3V_SHDN_N);
		gpio_free(PCIE_PWRGD);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", PCIE_REFCLK_OE);
		return -EIO;
	}

	gpio_direction_output(PCIE_3P3V_SHDN_N, 1);
	mdelay(2);
	gpio_direction_output(PCIE_PWRGD, 1);
	mdelay(2);
	gpio_direction_output(PCIE_REFCLK_OE, 1);
	mdelay(100);

	gpio_free(PCIE_3P3V_SHDN_N);
	gpio_free(PCIE_PWRGD);
	gpio_free(PCIE_REFCLK_OE);

	return 0;
}
#endif

static struct pxa168_pcie_platform_data pxa168_pcie_data = {
	.init		= pxa168_gpio_pcie_init,
};
#endif

#if defined(CONFIG_PXA168_CF) && defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
static struct resource pxa168_cf_resources[] = {
	[0] = {
		.start  = 0xD4285000,
		.end    = 0xD4285800,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = IRQ_PXA168_CF,
		.end    = IRQ_PXA168_CF,
		.flags  = IORESOURCE_IRQ,
	},
	[2] = {
		.start  = IRQ_GPIO(32),
		.end    = IRQ_GPIO(32),
		.flags  = IORESOURCE_IRQ,
	}
};

static struct platform_device pxa168_cf_device = {
	.name		= "pxa168-cf",
	.id		= -1,
	.resource	= pxa168_cf_resources,
	.num_resources	= ARRAY_SIZE(pxa168_cf_resources),
};

static void __init pxa168_cf_init(void)
{
	platform_device_register(&pxa168_cf_device);
}
#endif

/*
 * mfp is shared in card, cam and tw9907, only one is effective
 */
typedef enum{
	SW_CARD    = 0x01,
	SW_CAM_ON  = 0x02,
	SW_CAM_OFF = 0x03,
	SW_TW9907  = 0x04,
} SW_TYPE_T;

static int kovan_pinmux_switch(SW_TYPE_T type)
{
	int ret = 0;

	if (gpio_request(CARD_EN, "CARD_EN")) {
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d \n", CARD_EN);
		return -EIO;
	}

	if (gpio_request(CAM_PWDN, "CAM_PWDN")) {
		gpio_free(CARD_EN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", CAM_PWDN);
		return -EIO;
	}

	if (gpio_request(TW9907_PWDN, "TW9907_PWDN")) {
		gpio_free(CARD_EN);
		gpio_free(CAM_PWDN);
		printk(KERN_ERR "Request GPIO failed,"
				"gpio: %d\n", TW9907_PWDN);
		return -EIO;
	}

	switch (type) {
	case SW_CARD:
		gpio_direction_output(CARD_EN, 1);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_CAM_ON:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 0);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_CAM_OFF:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 1);
		break;
	case SW_TW9907:
		gpio_direction_output(CARD_EN, 0);
		gpio_direction_output(CAM_PWDN, 1);
		gpio_direction_output(TW9907_PWDN, 0);
		break;
	default:
		ret = -EIO;
		break;
	}

	gpio_free(CARD_EN);
	gpio_free(CAM_PWDN);
	gpio_free(TW9907_PWDN);

	return ret;
}

#if defined(CONFIG_PXA168_MSP)
/* msp platform data */
static mfp_cfg_t mfp_cfg_msp[]  = {
	GPIO40_MSP_DAT1,
	GPIO41_MSP_DAT0,
	GPIO43_MSP_DAT2,
	GPIO44_MSP_DAT3,
	GPIO42_MSP_BS,
	GPIO50_MSP_SCLK,
};

static int mspro_mfp_config(void)
{
	int ret = 0;

	ret = kovan_pinmux_switch(SW_CARD);
	if (0 == ret)
		mfp_config(ARRAY_AND_SIZE(mfp_cfg_msp));

	return ret;
}

static struct card_platform_data msp_ops = {
	/* GPIO84 used as mspro detect pin */
	.pin_detect		= MFP_PIN_GPIO84,
	.mfp_config		= mspro_mfp_config,
};
#endif

#if defined(CONFIG_PXA168_CAMERA)
static mfp_cfg_t kovan_cam_pins[] = {
	GPIO37_CAM_DAT7,
	GPIO38_CAM_DAT6,
	GPIO39_CAM_DAT5,
	GPIO40_CAM_DAT4,
	GPIO41_CAM_DAT3,
	GPIO42_CAM_DAT2,
	GPIO43_CAM_DAT1,
	GPIO44_CAM_DAT0,
	GPIO46_CAM_VSYNC,
	GPIO48_CAM_HSYNC,
	GPIO54_CAM_MCLK,
	GPIO55_CAM_PCLK,
};

/* sensor init */
static int sensor_power_onoff(int on, int id)
{
	/*
	 * on, 1, power on
	 * on, 0, power off
	 */
	int ret = 0;
	if(on){
		ret = kovan_pinmux_switch(SW_CAM_ON);
	        if (0 == ret)
			mfp_config(ARRAY_AND_SIZE(kovan_cam_pins));
	}else{
		ret = kovan_pinmux_switch(SW_CAM_OFF);
	}
	return ret;
}

static struct sensor_platform_data ov7670_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_onoff,
};

/* sensor init over */
#endif


static struct i2c_pxa_platform_data pwri2c_info __initdata = {
	.use_pio		= 1,
};


static struct fb_videomode video_modes_aspen[] = {
	/* lpj032l001b HVGA mode info */
        [0] = {
                .pixclock       = 16129,
                .refresh        = 60,
                .xres           = 1280,
                .yres           = 720,
                .hsync_len      = 40,
                .left_margin    = 220,
                .right_margin   = 110,
                .vsync_len      = 5,
                .upper_margin   = 20,
                .lower_margin   = 5,
                .sync           = 0, 
        },

	[1] = {
		.pixclock       = 30120,
		.refresh        = 60,
		.xres           = 800,
		.yres           = 480,
		.hsync_len      = 1,
		.left_margin    = 215,
		.right_margin   = 40,
		.vsync_len      = 1,
		.upper_margin   = 34,
		.lower_margin   = 10,
		.sync           = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
	},

        [2] = {
                .pixclock       = 16129,
                .refresh        = 60,
                .xres           = 1024,
                .yres           = 768,
                .hsync_len      = 136,
                .left_margin    = 160,
                .right_margin   = 24,
                .vsync_len      = 6,
                .upper_margin   = 29,
                .lower_margin   = 3,
                .sync           = 0, 
        },

        [3] = {
                .pixclock       = 25641,
                .refresh        = 60,
                .xres           = 800,
                .yres           = 600,
                .hsync_len      = 128,
                .left_margin    = 88,
                .right_margin   = 40,
                .vsync_len      = 4,
                .upper_margin   = 23,
                .lower_margin   = 1,
                .sync           = 0, 
        },

        [4] = {
                .pixclock       = 111111,
                .refresh        = 60,
                .xres           = 480,
                .yres           = 272,
                .hsync_len      = 1,
                .left_margin    = 43,
                .right_margin   = 2,
                .vsync_len      = 1,
                .upper_margin   = 12,
                .lower_margin   = 2,
                .sync           = 0, 
        },

        [5] = {
                .pixclock       = 39722,
                .refresh        = 60,
                .xres           = 640,
                .yres           = 480,
                .hsync_len      = 96,
                .left_margin    = 40,
                .right_margin   = 8,
                .vsync_len      = 2,
                .upper_margin   = 25,
                .lower_margin   = 2,
                .sync           = 0, 
        },

};

/* SPI Control Register. */
#define     CFG_SCLKCNT(div)                    (div<<24)  /* 0xFF~0x2 */
#define     CFG_RXBITS(rx)                      ((rx - 1)<<16)   /* 0x1F~0x1 */
#define     CFG_TXBITS(tx)                      ((tx - 1)<<8)    /* 0x1F~0x1, 0x1: 2bits ... 0x1F: 32bits */
#define     CFG_SPI_ENA(spi)                    (spi<<3)
#define     CFG_SPI_SEL(spi)                    (spi<<2)   /* 1: port1; 0: port0 */
#define     CFG_SPI_3W4WB(wire)                 (wire<<1)  /* 1: 3-wire; 0: 4-wire */

struct pxa168fb_mach_info kovan_lcd_info __initdata = {
	.id                     = "Base-aspen",
	.modes                  = video_modes_aspen,
	.num_modes              = ARRAY_SIZE(video_modes_aspen),
	.pix_fmt                = PIX_FMT_RGB565,
	.io_pin_allocation_mode = PIN_MODE_DUMB_24,
	.dumb_mode              = DUMB_MODE_RGB888,
	.active                 = 1,
	.spi_ctrl		= CFG_SCLKCNT(2) | CFG_TXBITS(16) | CFG_SPI_SEL(1) | CFG_SPI_3W4WB(1) | CFG_SPI_ENA(1),
	.spi_gpio_cs		= GPIO_EXT1(14),
	.spi_gpio_reset         = -1,
	.panel_rbswap		= 1,
	.invert_pixclock	= 1,
	.max_fb_size		= 1024 * 768 * 4 * 2,
};

struct pxa168fb_mach_info kovan_lcd_ovly_info __initdata = {
        .id                     = "Ovly-kovan",
        .modes                  = video_modes_aspen,
        .num_modes              = ARRAY_SIZE(video_modes_aspen),
        .pix_fmt                = PIX_FMT_RGB565,
        .io_pin_allocation_mode = PIN_MODE_DUMB_18_GPIO,
	.dumb_mode              = DUMB_MODE_RGB666,
	.active                 = 1,
	.panel_rbswap		= 1,
	.invert_pixclock        = 1,
	.invert_vsync           = 1,
	.invert_hsync           = 1,
	.panel_rgb_reverse_lanes= 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena     = 0,
	.panel_rbswap           = 0,
	.enable_lcd             = 1,
	.spi_gpio_cs            = 0,
	.spi_gpio_reset         = 0,
	.max_fb_size		= 1920 * 1080 * 2 * 2,
};


static struct i2c_board_info kovan_i2c_board_info[] = {

#if defined(CONFIG_PCI)
	{
		.type           = "max7312",
		.addr           = 0x28,  		/* 0x50/0x51 */
		.platform_data  = &max7312_data[2],
	},
#endif
#if defined(CONFIG_PXA168_CAMERA)
	{
		.type		= "ov7670",
		.addr           = 0x21,
		.platform_data  = &ov7670_sensor_data,
	},
#endif
#if defined(CONFIG_RTC_DRV_ISL1208)
	{
		.type		= "isl1208",
		.addr           = 0x6f,
	},
#endif
};

static struct i2c_board_info pwr_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("stmpe610", 0x44),
		.platform_data = &stmpe610_data,
	},
};


static unsigned int kovan_matrix_key_map[] = {
	KEY(0, 7, KEY_LEFT),
	KEY(4, 7, KEY_RIGHT),
	KEY(0, 6, KEY_HOME),
	KEY(4, 6, KEY_END),
	KEY(1, 7, KEY_ENTER),	/* keypad action */
	KEY(1, 6, KEY_SEND),
};

static unsigned int kovan_android_matrix_key_map[] = {
	KEY(0, 6, KEY_UP),	/* SW 4 */
	KEY(0, 7, KEY_DOWN),	/* SW 5 */
	KEY(1, 6, KEY_LEFT),	/* SW 6 */
	KEY(1, 7, KEY_RIGHT),	/* SW 7 */
	KEY(4, 6, KEY_MENU),	/* SW 8 */
	KEY(4, 7, KEY_BACK),	/* SW 9 */
};

static struct pxa27x_keypad_platform_data kovan_keypad_info __initdata = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= kovan_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(kovan_matrix_key_map),
	.debounce_interval	= 30,
};

static struct pxa27x_keypad_platform_data kovan_android_keypad_info __initdata = {
	.matrix_key_rows	= 8,
	.matrix_key_cols	= 8,
	.matrix_key_map		= kovan_android_matrix_key_map,
	.matrix_key_map_size	= ARRAY_SIZE(kovan_android_matrix_key_map),
	.debounce_interval	= 30,
};

#if (defined(CONFIG_SPI_PXA2XX) || defined(CONFIG_SPI_PXA2XX_MODULE)) \
	&& defined(CONFIG_MTD_M25P80)

static struct pxa2xx_spi_master pxa_ssp_master_info = {
	.num_chipselect	= 1,
	.enable_dma = 1,
};

static struct pxa2xx_spi_chip m25pxx_spi_info = {
	.tx_threshold = 1,
	.rx_threshold = 1,
	.timeout = 1000,
	.gpio_cs = 110
};

static struct spi_board_info __initdata spi_board_info[] = {
	{
		.modalias = "m25p80",
		.mode = SPI_MODE_0,
		.max_speed_hz = 260000,
		.bus_num = 2,
		.chip_select = 0,
		.platform_data = NULL,
		.controller_data = &m25pxx_spi_info,
		.irq = -1,
	},
};

static void __init kovan_init_spi(void)
{
	//pxa168_add_ssp(1);
	pxa168_add_spi(2, &pxa_ssp_master_info);
	spi_register_board_info(spi_board_info, ARRAY_SIZE(spi_board_info));
}
#else
static inline void kovan_init_spi(void) {}
#endif



#if defined(CONFIG_MMC_PXA_SDH)
static struct pfn_cfg mmc3_pfn_cfg[] = {
	PFN_CFG(PIN_MMC_DAT7, GPIO0_MMC3_DAT7, GPIO0_GPIO),
	PFN_CFG(PIN_MMC_DAT6, GPIO1_MMC3_DAT6, GPIO1_GPIO),
	PFN_CFG(PIN_MMC_DAT5, GPIO2_MMC3_DAT5, GPIO2_GPIO),
	PFN_CFG(PIN_MMC_DAT4, GPIO3_MMC3_DAT4, GPIO3_GPIO),
	PFN_CFG(PIN_MMC_DAT3, GPIO4_MMC3_DAT3, GPIO4_GPIO),
	PFN_CFG(PIN_MMC_DAT2, GPIO5_MMC3_DAT2, GPIO5_GPIO),
	PFN_CFG(PIN_MMC_DAT1, GPIO6_MMC3_DAT1, GPIO6_GPIO),
	PFN_CFG(PIN_MMC_DAT0, GPIO7_MMC3_DAT0, GPIO7_GPIO),
	PFN_CFG(PIN_MMC_CLK, GPIO8_MMC3_CLK, GPIO8_GPIO),
	PFN_CFG(PIN_MMC_CMD, GPIO9_MMC3_CMD, GPIO9_GPIO),
	PFN_CFG(PIN_MMC_CD, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_WP, PFN_UNDEF, PFN_UNDEF),
	PFN_CFG(PIN_MMC_END, PFN_TERM, PFN_TERM),
};
#endif


#if defined(CONFIG_MMC3)
static struct pxasdh_platform_data kovan_sdh_platform_data_mmc3 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 8,
	.quirks 	= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.pfn_table	= mmc3_pfn_cfg,
};
#endif

#ifdef CONFIG_USB_GADGET_PXA_U2O

static int kovan_u2o_vbus_status(unsigned base)
{
	return 0;
}

static int kovan_u2o_vbus_set(int vbus_type)
{
	return 0;
}
static int kovan_otg_init(void)
{
	return 0;
}

static int kovan_u2o_vbus_set_ic(int function)
{
	return 0;
}

static struct otg_pmic_ops kovan_otg_ops = {
	.otg_vbus_init          = kovan_otg_init,
	.otg_set_vbus           = kovan_u2o_vbus_set,
	.otg_set_vbus_ic        = kovan_u2o_vbus_set_ic,
	.otg_get_vbus_state     = kovan_u2o_vbus_status,
};

struct otg_pmic_ops *init_kovan_otg_ops(void)
{
	return &kovan_otg_ops;
}

static struct pxa_usb_plat_info kovan_u2o_info = {
	.phy_init	= pxa168_usb_phy_init,
	.phy_deinit	= pxa168_usb_phy_deinit,
	.vbus_set	= kovan_u2o_vbus_set,
	.vbus_status	= kovan_u2o_vbus_status,
	.init_pmic_ops	= (void *)init_kovan_otg_ops,
#ifdef CONFIG_USB_OTG
	.is_otg		= 1,
#else
	.clk_gating	= 1,
#endif
};
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
/* USB 2.0 Host Controller */
static int kovan_u2h_vbus_set (int enable)
{
	gpio_request(USB_WIFI_GPIO, "Wifi Enable");
	gpio_direction_output(USB_WIFI_GPIO, 1);
	gpio_set_value(USB_WIFI_GPIO, 1);
	return 0;
}

static struct pxa_usb_plat_info kovan_u2h_info = {
	.phy_init	= pxa168_usb_phy_init,
	.vbus_set	= kovan_u2h_vbus_set,
};
#endif



static void __init kovan_init(void)
{
	mfp_config(ARRAY_AND_SIZE(kovan_pin_config));
        pxa168_set_vdd_iox(VDD_IO0, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO1, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO2, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO3, VDD_IO_3P3V);
	pxa168_set_vdd_iox(VDD_IO4, VDD_IO_3P3V);
	pxa168_mfp_set_fastio_drive(MFP_DS02X);

	/* on-chip devices */
	pxa168_add_uart(1);

	pxa168_add_freq();

	pxa168_add_ssp(0);
	pxa168_add_twsi(0, &pwri2c_info, ARRAY_AND_SIZE(kovan_i2c_board_info));
	pxa168_add_twsi(1, &pwri2c_info, ARRAY_AND_SIZE(pwr_i2c_board_info));
	if (is_android())
		pxa168_add_keypad(&kovan_android_keypad_info);
	else
		pxa168_add_keypad(&kovan_keypad_info);

#ifdef CONFIG_USB_GADGET_PXA_U2O
 	pxa168_add_u2o(&kovan_u2o_info);
#endif

#ifdef CONFIG_USB_OTG
	pxa168_add_u2ootg(&kovan_u2o_info);
	pxa168_add_u2oehci(&kovan_u2o_info);
#endif

#ifdef CONFIG_USB_EHCI_PXA_U2H
 	pxa168_add_u2h(&kovan_u2h_info);
#endif
#ifdef CONFIG_PCI
	pxa168_add_pcie(&pxa168_pcie_data);
#endif
#if defined(CONFIG_MMC_PXA_SDH)
#if defined(CONFIG_MMC3)
	pxa168_add_sdh(2, &kovan_sdh_platform_data_mmc3);
#endif
#endif
#if defined(CONFIG_CIR)
	pxa168_cir_init(); /*init the gpio */
#endif
#if defined(CONFIG_PXA168_MSP)
	pxa168_add_msp(&msp_ops);
#endif
#if defined(CONFIG_PXA168_CF)
#if defined(CONFIG_PXA168_CF_USE_GPIO_CARDDETECT)
	pxa168_cf_init();
#else
	pxa168_add_cf();	
#endif
#endif

	//pxa168_add_fb(&kovan_lcd_info);
	pxa168_add_fb_ovly(&kovan_lcd_ovly_info);

	kovan_init_spi();
#if defined(CONFIG_PXA168_CAMERA)
	pxa168_add_cam();
#endif
#if defined(CONFIG_PXA_ICR)
	pxa168_add_icr();
#endif

#if defined(CONFIG_BATTERY_ASPENITE)
	kovan_add_battery();
#endif
	pxa168_add_rtc(&pxa910_device_rtc);

}

MACHINE_START(KOVAN, "PXA168-based Kovan Platform")
	.phys_io        = APB_PHYS_BASE,
	.boot_params    = 0x00000100,
	.io_pg_offst    = (APB_VIRT_BASE >> 18) & 0xfffc,
	.map_io		= pxa_map_io,
	.init_irq       = pxa168_init_irq,
	.timer          = &pxa168_timer,
	.init_machine   = kovan_init,
MACHINE_END
