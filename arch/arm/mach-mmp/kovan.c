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

};

static struct i2c_board_info pwr_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("stmpe610", 0x44),
		.platform_data = &stmpe610_data,
	},
};


static struct i2c_pxa_platform_data i2c_info __initdata = {
	.use_pio		= 1,
};




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


static struct pxasdh_platform_data kovan_sdh_platform_data_mmc3 = {
	.detect_delay	= 20,
	.ocr_mask	= MMC_VDD_29_30 | MMC_VDD_30_31,
	.bus_width	= 8,
	.quirks 	= SDHCI_QUIRK_BROKEN_CARD_DETECTION,
	.pfn_table	= mmc3_pfn_cfg,
};



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
	.is_otg		= 1,
};


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
	pxa168_add_twsi(0, &i2c_info, ARRAY_AND_SIZE(kovan_i2c_board_info));
	pxa168_add_twsi(1, &i2c_info, ARRAY_AND_SIZE(pwr_i2c_board_info));

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
	pxa168_add_sdh(2, &kovan_sdh_platform_data_mmc3);
#if defined(CONFIG_CIR)
	pxa168_cir_init();
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
