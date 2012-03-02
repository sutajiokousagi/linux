/*
 * kovan.c  --  SoC audio for kovan
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  TTC FPGA audio amplifier code taken from arch/arm/mach-pxa/mainstone.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Oct 2005   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <asm/uaccess.h>

#include <asm/mach-types.h>
#include <asm/io.h>


#include <plat/regs-ssp.h>
#include <plat/ssp.h>

#include <mach/addr-map.h>

#include "../codecs/es8328.h"
#include "pxa3xx-pcm.h"
#include "pxa3xx-ssp.h"
#include "../codecs/autopower.h"

#define KOVAN_ES8328_DEBUG

#ifdef KOVAN_ES8328_DEBUG
#define dbg(format, arg...) \
	printk(KERN_INFO format "\n" , ## arg)
#else
	#define dbg(format, arg...) do {} while (0)
#endif


static struct snd_soc_card kovan;

struct _ssp_conf {
	unsigned int main_clk;
	unsigned int sys_num;
	unsigned int sys_den;
	unsigned int bit_clk;
	unsigned int ssp_num;
	unsigned int ssp_den;
	unsigned int freq_out;
};

static const struct _ssp_conf ssp_conf[] = {
	/*main_clk, sys_num, sys_den, bit_clk, ssp_num, ssp_den, freq_out*/
	{12288000,  0x659,    0x40,   3072000, 0x100,   0x40,   48000},
	{11289600, 0x1fa1,   0x125,   2822000, 0x100,   0x40,   44100},
	{12288000,  0x659,    0x40,   2048000, 0x180,   0x40,   32000},
	{12288000,  0x659,    0x40,   1536000, 0x200,   0x40,   24000},
	{11289600,  0x6E9,    0x40,   1411000, 0x200,   0x40,   22050},
	{12288000,  0x659,    0x40,   1024000, 0x300,   0x40,   16000},
	{11289600,  0x6E9,    0x40,    706500, 0x400,   0x40,   11025},
	{12288000,  0x659,    0x40,    512000, 0x600,   0x40,   8000},
	{12288000,  0x659,    0x40,   6144000, 0x100,   0x80,   96000},
};

static int ssp_conf_lookup(unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ssp_conf); i++)
		if (ssp_conf[i].freq_out == rate)
			return i;
	return -EINVAL;
}

#define ASYSDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51050)))
#define ASSPDR   (* ((volatile unsigned long *) (APB_VIRT_BASE + 0x51054)))
static void ssp_set_clock(unsigned int ceoff)
{
	int asysdr, asspdr;
	asysdr = ssp_conf[ceoff].sys_num << 16 | ssp_conf[ceoff].sys_den;
	ASYSDR = asysdr;
	asspdr = ssp_conf[ceoff].ssp_num << 16 | ssp_conf[ceoff].ssp_den;
	ASSPDR = asspdr;
}

/*
static const struct snd_soc_dapm_widget kovan_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Bias", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};
*/

/*
static const struct snd_soc_dapm_route audio_map[] = {
	{"Headphone Jack", NULL, "LOUT2"},
	{"Headphone Jack", NULL, "ROUT2"}, 

	{ "LINPUT2", NULL, "Mic Bias" }, 
	{ "Mic Bias", NULL, "Mic Jack" },

	{"LINPUT1", NULL, "Line In Jack"},
	{"RINPUT1", NULL, "Line In Jack"},
};
*/

static int kovan_es8328_init(struct snd_soc_codec *codec)
{
	/* init ssp clock should be done
	 * before powering up codec.
	 */
	printk(KERN_NOTICE "***** INIT *****\n");
	ssp_set_clock(0);

#if 0
	snd_soc_dapm_new_controls(codec, kovan_dapm_widgets,
				ARRAY_SIZE(kovan_dapm_widgets));

	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));

	/* setting for HP autodetection kovan specific */
/*
	snd_soc_write(codec, WM8960_ADDCTL2 , 0x40);
	snd_soc_write(codec, WM8960_ADDCTL4 , 0x8);
	snd_soc_write(codec, WM8960_ADDCTL1 , 0x3);
*/
	snd_soc_dapm_enable_pin(codec, "Mic Bias");
	snd_soc_dapm_enable_pin(codec, "Headphone Jack");
	snd_soc_dapm_enable_pin(codec, "Line In Jack");
	snd_soc_dapm_sync(codec);
#endif

	return 0;
}

static int kovan_es8328_hifi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_soc_dai *codec_dai = machine->codec_dai;
	unsigned int format;

       cpu_dai->playback.channels_min = 2;
       cpu_dai->playback.channels_max = 2;
       cpu_dai->capture.channels_min = 0;
       cpu_dai->capture.channels_max = 0;

	/* Configure SSCR0 as:
		- MOD 1: Network mode
		- FPCKE 1: FIFO Packing enabled
		- FRDC 1: Divider of 2 time slots for network mode
		- TIM 1: TX FIFO Underrun Interrupt mask on
		- RIM 1: RX FIFO Underrun Interrupt mask on
		- SSE 0: SSP disabled
		- FRF 3: Frame Format of Programmable Serial Protocol
		- DSS F: 16-bit data
	*/
	__raw_writel(SSCR0_MOD | SSCR0_FPCKE | SSCR0_SlotsPerFrm(2)
		| SSCR0_TIM | SSCR0_RIM | SSCR0_FRF
		| SSCR0_EDSS | SSCR0_DataSize(16),
		ssp->mmio_base + SSCR0);

	/* Configure SSCR1 as:
		- SCFR 1: Slave clock only active during data transfers
		- SCLKDIR 1: Set clock to Slave mode
		- SFRMDIR 1: Set L/R selection to Slave mode
		- TRAIL 1: Trailing byte handled by DMA burst
		- TSRE 1: TX DMA service request enabled
		- RSRE 1: RX DMA service request enabled
		- RFT 4: RX FIFO trigger threshold
		- TFT 3: TX FIFO trigger threshold
	*/
	__raw_writel(SSCR1_SCFR | SSCR1_SCLKDIR | SSCR1_SFRMDIR
		| SSCR1_TRAIL | SSCR1_TSRE | SSCR1_RSRE
		| SSCR1_TxTresh(3) | SSCR1_RxTresh(4),
		ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);

	/* Configure SS PSP as:
		- EDMYSTOP 4: Extended Dummy Stop of 5 significant bits
		- SFRMWDTH 20: Frame width of 32 clock cycles
		- SFRMP 1: Frame polarity of high
		- FSRT 1: Frame sync timing of 1 clock cycle
		- SCMODE 0: Data driven on falling edge and sampled on rising
	*/
	__raw_writel(SSPSP_SFRMWDTH(16) | SSPSP_FSRT, ssp->mmio_base + SSPSP);

	cpu_dai->playback.formats = SNDRV_PCM_FMTBIT_S16_LE;
	cpu_dai->capture.formats = SNDRV_PCM_FMTBIT_S16_LE;

	format = SND_SOC_DAIFMT_LEFT_J | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF;
	codec_dai->ops->set_fmt(codec_dai, format);

	return 0;
}

static int kovan_es8328_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *machine = rtd->dai;
	struct snd_soc_dai *cpu_dai = machine->cpu_dai;
	struct ssp_device *ssp = cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long rate = runtime->rate;
	int ceoff;

	__raw_writel(SSCR0_MOD | SSCR0_FPCKE | SSCR0_SlotsPerFrm(2)
		| SSCR0_TIM | SSCR0_RIM | SSCR0_FRF
		| SSCR0_EDSS | SSCR0_DataSize(16),
		ssp->mmio_base + SSCR0);
	__raw_writel(SSCR1_SCFR | SSCR1_SCLKDIR | SSCR1_SFRMDIR
		| SSCR1_TRAIL | SSCR1_TSRE | SSCR1_RSRE
		| SSCR1_TxTresh(3) | SSCR1_RxTresh(4),
		ssp->mmio_base + SSCR1);
	__raw_writel(0x3, ssp->mmio_base + SSTSA);
	__raw_writel(0x3, ssp->mmio_base + SSRSA);
	__raw_writel(SSPSP_SFRMWDTH(16) | SSPSP_FSRT, ssp->mmio_base + SSPSP);

	ceoff = ssp_conf_lookup(rate);

	if (ceoff >= 0)
		ssp_set_clock(ceoff);
	else
		printk(KERN_ERR "Wrong audio sample rate\n");

	return 0;
}

static void kovan_es8328_hifi_shutdown(struct snd_pcm_substream *substream)
{
}

static int kovan_es8328_hifi_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *hw_params)
{
	return 0;
}

static int kovan_es8328_voice_startup(struct snd_pcm_substream *substream)
{
	return 0;
}


static int kovan_es8328_voice_prepare(struct snd_pcm_substream *substream)
{
	return 0;
}

static void kovan_es8328_voice_shutdown(struct snd_pcm_substream *substream)
{
}

/* machine stream operations */
static struct snd_soc_ops kovan_es8328_machine_ops[] = {
{
	.startup = kovan_es8328_hifi_startup,
	.prepare = kovan_es8328_hifi_prepare,
	.shutdown = kovan_es8328_hifi_shutdown,
	.hw_params = kovan_es8328_hifi_hw_params,
},
{
	.startup = kovan_es8328_voice_startup,
	.prepare = kovan_es8328_voice_prepare,
	.shutdown = kovan_es8328_voice_shutdown,
},
};

static struct snd_soc_dai_link kovan_dai[] = {
{
	.name = "ES8328",
	.stream_name = "ES8328 HiFi",
	.cpu_dai = &pxa3xx_ssp_dai[0],
	.codec_dai = &es8328_dai,
	.ops = &kovan_es8328_machine_ops[0],
	.init = kovan_es8328_init,
},
};

static struct snd_soc_card kovan = {
	.name = "KOVAN ES8328",
	.platform = &pxa3xx_soc_platform,
	.dai_link = kovan_dai,
	.num_links = ARRAY_SIZE(kovan_dai),
};

static struct es8328_setup_data es8328_setup = {
	.i2c_address = 0x11,
	.i2c_bus = 1,
};

static struct snd_soc_device kovan_snd_devdata = {
	.card		= &kovan,
	.codec_dev	= &soc_codec_dev_es8328,
	.codec_data	= &es8328_setup,
};

static struct platform_device *kovan_snd_device;


/* shut down codec chip */
#if 0
static void kovan_audio_shutdown(struct device *dev)
{
	printk(KERN_ERR "es8328 power down\n");
	es8328_powerdown(1);
}
#endif

static int __init kovan_init(void)
{
	int ret;

	/* ssp_set_clock(0); */
	kovan_snd_device = platform_device_alloc("soc-audio", -1);

	if (!kovan_snd_device)
		return -ENOMEM;

	kovan_snd_devdata.dev = &kovan_snd_device->dev;

	platform_set_drvdata(kovan_snd_device, &kovan_snd_devdata);
	ret = platform_device_add(kovan_snd_device);
	if (ret)
		platform_device_put(kovan_snd_device);

	return ret;
}


static void __exit kovan_exit(void)
{
	platform_device_unregister(kovan_snd_device);
}


module_init(kovan_init);
module_exit(kovan_exit);

/* Module information */
