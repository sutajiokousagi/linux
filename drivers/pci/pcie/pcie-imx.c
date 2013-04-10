/*
 * arch/arm/mach-mx6/pcie.c
 *
 * PCIe host controller driver for IMX6 SOCs
 *
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * Bits taken from arch/arm/mach-dove/pcie.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/clk-provider.h>
#include <linux/regmap.h>
#include <linux/rfkill.h>

#include <asm/sizes.h>
#include <asm/io.h>


/* IOMUXC */
#define IOMUXC_GPR0                     (0x00)
#define IOMUXC_GPR1                     (0x04)
#define IOMUXC_GPR2                     (0x08)
#define IOMUXC_GPR3                     (0x0C)
#define IOMUXC_GPR4                     (0x10)
#define IOMUXC_GPR5                     (0x14)
#define IOMUXC_GPR6                     (0x18)
#define IOMUXC_GPR7                     (0x1C)
#define IOMUXC_GPR8                     (0x20)
#define IOMUXC_GPR9                     (0x24)
#define IOMUXC_GPR10                    (0x28)
#define IOMUXC_GPR11                    (0x2C)
#define IOMUXC_GPR12                    (0x30)
#define IOMUXC_GPR13                    (0x34)


/* Register Definitions */
#define PRT_LOG_R_BaseAddress 0x700

/* Register DB_R0 */
/* Debug Register 0 */
#define DB_R0 (PRT_LOG_R_BaseAddress + 0x28)
#define DB_R0_RegisterSize 32
#define DB_R0_RegisterResetValue 0x0
#define DB_R0_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R0 */

/* Register DB_R1 */
/* Debug Register 1 */
#define DB_R1 (PRT_LOG_R_BaseAddress + 0x2c)
#define DB_R1_RegisterSize 32
#define DB_R1_RegisterResetValue 0x0
#define DB_R1_RegisterResetMask 0xFFFFFFFF
/* End of Register Definition for DB_R1 */

#define ATU_R_BaseAddress 0x900
#define PCIE_PL_iATUVR (ATU_R_BaseAddress + 0x0)
#define PCIE_PL_iATURC1 (ATU_R_BaseAddress + 0x4)
#define PCIE_PL_iATURC2 (ATU_R_BaseAddress + 0x8)
#define PCIE_PL_iATURLBA (ATU_R_BaseAddress + 0xC)
#define PCIE_PL_iATURUBA (ATU_R_BaseAddress + 0x10)
#define PCIE_PL_iATURLA (ATU_R_BaseAddress + 0x14)
#define PCIE_PL_iATURLTA (ATU_R_BaseAddress + 0x18)
#define PCIE_PL_iATURUTA (ATU_R_BaseAddress + 0x1C)

/* GPR1: iomuxc_gpr1_pcie_ref_clk_en(iomuxc_gpr1[16]) */
#define iomuxc_gpr1_pcie_ref_clk_en		(1 << 16)
/* GPR1: iomuxc_gpr1_test_powerdown(iomuxc_gpr1_18) */
#define iomuxc_gpr1_test_powerdown		(1 << 18)

/* GPR12: iomuxc_gpr12_los_level(iomuxc_gpr12[8:4]) */
#define iomuxc_gpr12_los_level			(0x1F << 4)
/* GPR12: iomuxc_gpr12_app_ltssm_enable(iomuxc_gpr12[10]) */
#define iomuxc_gpr12_app_ltssm_enable		(1 << 10)
/* GPR12: iomuxc_gpr12_device_type(iomuxc_gpr12[15:12]) */
#define iomuxc_gpr12_device_type		(0xF << 12)

/* GPR8: iomuxc_gpr8_tx_deemph_gen1(iomuxc_gpr8[5:0]) */
#define iomuxc_gpr8_tx_deemph_gen1		(0x3F << 0)
/* GPR8: iomuxc_gpr8_tx_deemph_gen2_3p5db(iomuxc_gpr8[11:6]) */
#define iomuxc_gpr8_tx_deemph_gen2_3p5db	(0x3F << 6)
/* GPR8: iomuxc_gpr8_tx_deemph_gen2_6db(iomuxc_gpr8[17:12]) */
#define iomuxc_gpr8_tx_deemph_gen2_6db		(0x3F << 12)
/* GPR8: iomuxc_gpr8_tx_swing_full(iomuxc_gpr8[24:18]) */
#define iomuxc_gpr8_tx_swing_full		(0x7F << 18)
/* GPR8: iomuxc_gpr8_tx_swing_low(iomuxc_gpr8[31:25]) */
#define iomuxc_gpr8_tx_swing_low		(0x7F << 25)

/* Registers of PHY */
/* Register PHY_STS_R */
/* PHY Status Register */
#define PHY_STS_R (PRT_LOG_R_BaseAddress + 0x110)

/* Register PHY_CTRL_R */
/* PHY Control Register */
#define PHY_CTRL_R (PRT_LOG_R_BaseAddress + 0x114)

#define SSP_CR_SUP_DIG_MPLL_OVRD_IN_LO 0x0011
/* FIELD: RES_ACK_IN_OVRD [15:15]
// FIELD: RES_ACK_IN [14:14]
// FIELD: RES_REQ_IN_OVRD [13:13]
// FIELD: RES_REQ_IN [12:12]
// FIELD: RTUNE_REQ_OVRD [11:11]
// FIELD: RTUNE_REQ [10:10]
// FIELD: MPLL_MULTIPLIER_OVRD [9:9]
// FIELD: MPLL_MULTIPLIER [8:2]
// FIELD: MPLL_EN_OVRD [1:1]
// FIELD: MPLL_EN [0:0]
*/

#define SSP_CR_SUP_DIG_ATEOVRD 0x0010
/* FIELD: ateovrd_en [2:2]
// FIELD: ref_usb2_en [1:1]
// FIELD: ref_clkdiv2 [0:0]
*/

#define SSP_CR_LANE0_DIG_RX_OVRD_IN_LO 0x1005
/* FIELD: RX_LOS_EN_OVRD [13:13]
// FIELD: RX_LOS_EN [12:12]
// FIELD: RX_TERM_EN_OVRD [11:11]
// FIELD: RX_TERM_EN [10:10]
// FIELD: RX_BIT_SHIFT_OVRD [9:9]
// FIELD: RX_BIT_SHIFT [8:8]
// FIELD: RX_ALIGN_EN_OVRD [7:7]
// FIELD: RX_ALIGN_EN [6:6]
// FIELD: RX_DATA_EN_OVRD [5:5]
// FIELD: RX_DATA_EN [4:4]
// FIELD: RX_PLL_EN_OVRD [3:3]
// FIELD: RX_PLL_EN [2:2]
// FIELD: RX_INVERT_OVRD [1:1]
// FIELD: RX_INVERT [0:0]
*/

#define SSP_CR_LANE0_DIG_RX_ASIC_OUT 0x100D
/* FIELD: LOS [2:2]
// FIELD: PLL_STATE [1:1]
// FIELD: VALID [0:0]
*/

/* control bus bit definition */
#define PCIE_CR_CTL_DATA_LOC 0
#define PCIE_CR_CTL_CAP_ADR_LOC 16
#define PCIE_CR_CTL_CAP_DAT_LOC 17
#define PCIE_CR_CTL_WR_LOC 18
#define PCIE_CR_CTL_RD_LOC 19
#define PCIE_CR_STAT_DATA_LOC 0
#define PCIE_CR_STAT_ACK_LOC 16

/* End of Register Definitions */

#define  PCIE_CONF_BUS(b)		(((b) & 0xFF) << 16)
#define  PCIE_CONF_DEV(d)		(((d) & 0x1F) << 11)
#define  PCIE_CONF_FUNC(f)		(((f) & 0x7) << 8)
#define  PCIE_CONF_REG(r)		((r) & ~0x3)


/* Taken from PCI specs */
enum {
	MemRdWr = 0,
	MemRdLk = 1,
	IORdWr = 2,
	CfgRdWr0 = 4,
	CfgRdWr1 = 5
};


struct imx_pcie_port {
	struct device		*dev;
	u8			index;
	u8			root_bus_nr;
	int			interrupt;

	struct resource		*dbi;
	struct resource		*io;
	struct resource 	*mem;
	struct resource 	*root;

	struct regmap		*iomuxc_gpr;

	void __iomem		*root_base;
	void __iomem		*dbi_base;
	void __iomem		*io_base;
	void __iomem		*mem_base;
	spinlock_t		conf_lock;

	char			io_space_name[16];
	char			mem_space_name[16];

	struct list_head	next;

	struct clk		*lvds1_sel;
	struct clk		*lvds1;
	struct clk		*pcie_ref_125m;
	struct clk		*pcie_axi;
	struct clk		*sata_ref;

        unsigned int		pcie_pwr_en;
        unsigned int		pcie_rst;
        unsigned int		pcie_wake_up;
        unsigned int		pcie_dis;

	struct rfkill		*rfkill;
};

static const struct of_device_id pcie_of_match[] = {
	{
		.compatible	= "fsl,imx6q-pcie",
		.data		= NULL,
	},
	{},
};
MODULE_DEVICE_TABLE(of, pcie_of_match);

static struct list_head pcie_port_list;
static struct hw_pci imx_pcie;

static int pcie_phy_cr_read(void __iomem *dbi_base, int addr, int *data);
static int pcie_phy_cr_write(void __iomem *dbi_base, int addr, int data);
static void change_field(int *in, int start, int end, int val);


#if defined(CONFIG_RFKILL) || defined(CONFIG_RFKILL_MODULE)
static int imx_pcie_set_power(void *data, bool blocked)
{
	struct imx_pcie_port *pp = data;

	if (gpio_is_valid(pp->pcie_dis))
		gpio_set_value(pp->pcie_dis, !blocked);
	return 0;
}


static const struct rfkill_ops imx_pcie_rfkill_ops = {
	.set_block = imx_pcie_set_power,
};
#endif /* RFKILL || RFKILL_MODULE */


/* IMX PCIE GPR configure routines */
static void imx_pcie_clrset(struct imx_pcie_port *pp,
			    u32 mask, u32 val, u32 reg)
{
	u32 tmp;
	regmap_read(pp->iomuxc_gpr, reg, &tmp);
	tmp &= ~mask;
	tmp |= (val & mask);
	regmap_write(pp->iomuxc_gpr, reg, tmp);
}


static struct imx_pcie_port *controller_to_port(int index)
{
	struct imx_pcie_port *pp;

	if (index >= imx_pcie.nr_controllers) {
		pr_err("%d exceeded number of controllers %d\n",
			index, imx_pcie.nr_controllers);
		return NULL;
	}

	list_for_each_entry(pp, &pcie_port_list, next) {
		if (pp->index == index)
			return pp;
	}
	return NULL;
}

static struct imx_pcie_port *bus_to_port(int bus)
{
	int i;
	int rbus;
	struct imx_pcie_port *pp;

	for (i = imx_pcie.nr_controllers - 1 ; i >= 0; i--) {
		pp = controller_to_port(i);
		rbus = pp->root_bus_nr;
		if (rbus != -1 && rbus <= bus)
			break;
	}

	return i >= 0 ? pp : NULL;
}

static int __init imx_pcie_setup(int nr, struct pci_sys_data *sys)
{
	struct imx_pcie_port *pp;
	int ret;

	pp = controller_to_port(nr);
	if (!pp) {
		pr_err("unable to find port %d\n", nr);
		return 0;
	}

	pp->root_bus_nr = sys->busnr;
	dev_dbg(pp->dev, "setting up pcie port %d\n", nr);

	/*
	 * IORESOURCE_MEM
	 */
	snprintf(pp->mem_space_name, sizeof(pp->mem_space_name),
			"PCIe %d MEM", pp->index);

	pp->mem_space_name[sizeof(pp->mem_space_name) - 1] = 0;
	pp->mem->name = pp->mem_space_name;
	pp->mem->flags = IORESOURCE_MEM;
	ret = request_resource(&iomem_resource, pp->mem);
	if (ret)
		printk("Request PCIe Memory resource failed: %d\n", ret);
	pci_add_resource_offset(&sys->resources, pp->mem, sys->mem_offset);


	snprintf(pp->io_space_name, sizeof(pp->io_space_name),
		 "PCIe %d I/O", pp->index);
	pp->io_space_name[sizeof(pp->io_space_name) - 1] = 0;
	pp->io->name = pp->io_space_name;
	pp->io->flags = IORESOURCE_IO;

	ret = request_resource(&iomem_resource, pp->io);
	if (ret)
		printk("Request PCIe IO resource failed: %d\n", ret);
	pci_add_resource_offset(&sys->resources, pp->io, sys->io_offset);

	/*
	 * IORESOURCE_IO
	 */
	ret = pci_ioremap_io(PCIBIOS_MIN_IO, pp->io->start);
	if (ret)
		printk("Request PCIe IO resource failed: %d\n", ret);

	return 1;
}

static int imx_pcie_link_up(struct platform_device *pdev)
{
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);
	int iterations = 200;
	u32 rc, ltssm, rx_valid, temp;

	rc = 0;
	for (iterations = 200; iterations > 0 && !rc; iterations--) {
		/* link is debug bit 36, debug register 1 starts at bit 32 */
		rc = readl(pp->dbi_base + DB_R1) & (0x1 << (36 - 32)) ;
		usleep_range(2000, 3000);

		/* From L0, initiate MAC entry to gen2 if EP/RC supports gen2.
		 * Wait 2ms (LTSSM timeout is 24ms, PHY lock is ~5us in gen2).
		 * If (MAC/LTSSM.state == Recovery.RcvrLock)
		 * && (PHY/rx_valid==0) then pulse PHY/rx_reset. Transition
		 * to gen2 is stuck
		 */
		pcie_phy_cr_read(pp->dbi_base, SSP_CR_LANE0_DIG_RX_ASIC_OUT, &rx_valid);
		ltssm = readl(pp->dbi_base + DB_R0) & 0x3F;
		if ((ltssm == 0x0D) && ((rx_valid & 0x01) == 0)) {
			dev_info(&pdev->dev,
				"transition to gen2 is stuck, reset PHY!\n");
			pcie_phy_cr_read(pp->dbi_base, SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &temp);
			change_field(&temp, 3, 3, 0x1);
			change_field(&temp, 5, 5, 0x1);
			pcie_phy_cr_write(pp->dbi_base, SSP_CR_LANE0_DIG_RX_OVRD_IN_LO,
					0x0028);
			usleep_range(2000, 3000);
			pcie_phy_cr_read(pp->dbi_base, SSP_CR_LANE0_DIG_RX_OVRD_IN_LO, &temp);
			change_field(&temp, 3, 3, 0x0);
			change_field(&temp, 5, 5, 0x0);
			pcie_phy_cr_write(pp->dbi_base, SSP_CR_LANE0_DIG_RX_OVRD_IN_LO,
					0x0000);
		}

		if ((iterations <= 0))
			dev_info(&pdev->dev,
				"link up failed, DB_R0:0x%08x, DB_R1:0x%08x!\n",
				readl(pp->dbi_base + DB_R0),
				readl(pp->dbi_base + DB_R1));
	}

	if (!rc)
		return 0;
	return 1;
}

static int imx_pcie_regions_setup(struct platform_device *pdev,
					struct imx_pcie_port *pp)
{
	void __iomem *dbi_base = pp->dbi_base;
	/*
	 * i.MX6 defines 16MB in the AXI address map for PCIe.
	 *
	 * That address space excepted the pcie registers is
	 * split and defined into different regions by iATU,
	 * with sizes and offsets as follows:
	 *
	 * 0x0100_0000 --- 0x010F_FFFF 1MB IORESOURCE_IO
	 * 0x0110_0000 --- 0x01EF_FFFF 14MB IORESOURCE_MEM
	 * 0x01F0_0000 --- 0x01FF_FFFF 1MB Cfg + Registers
	 */

	/* CMD reg:I/O space, MEM space, and Bus Master Enable */
	writel(readl(dbi_base + PCI_COMMAND)
			| PCI_COMMAND_IO
			| PCI_COMMAND_MEMORY
			| PCI_COMMAND_MASTER,
			dbi_base + PCI_COMMAND);

	/* Set the CLASS_REV of RC CFG header to PCI_CLASS_BRIDGE_PCI */
	writel(readl(dbi_base + PCI_CLASS_REVISION)
			| (PCI_CLASS_BRIDGE_PCI << 16),
			dbi_base + PCI_CLASS_REVISION);

	/*
	 * region0 outbound used to access target cfg
	 */
	writel(0, dbi_base + PCIE_PL_iATUVR);
	writel(pp->root->start, dbi_base + PCIE_PL_iATURLBA);
	writel(pp->dbi->end, dbi_base + PCIE_PL_iATURLA);
	writel(0, dbi_base + PCIE_PL_iATURUBA);

	writel(0, dbi_base + PCIE_PL_iATURLTA);
	writel(0, dbi_base + PCIE_PL_iATURUTA);
	writel(CfgRdWr0, dbi_base + PCIE_PL_iATURC1);
	writel((1<<31), dbi_base + PCIE_PL_iATURC2);


	return 0;
}


static int imx_pcie_valid_config(struct imx_pcie_port *pp,
				struct pci_bus *bus, int devfn)
{
	if (bus->number >= 2)
		return 0;

	if (devfn != 0)
		return 0;

	return 1;
}


static u32 get_bus_address(struct imx_pcie_port *pp,
			   struct pci_bus *bus, u32 devfn, int where)
{
	u32 va_address;
	if (bus->number == 0) {
		va_address = (u32)pp->dbi_base + (where & ~0x3);
	}
	else {
		va_address = (u32)pp->root_base +
					(PCIE_CONF_BUS(bus->number - 1) +
					PCIE_CONF_DEV(PCI_SLOT(devfn)) +
					PCIE_CONF_FUNC(PCI_FUNC(devfn)) +
					PCIE_CONF_REG(where));
	}
	return va_address;
}

static int imx_pcie_read_config(struct pci_bus *bus, u32 devfn, int where,
			int size, u32 *val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	u32 va_address;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	if (imx_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0) {
		*val = 0xffffffff;
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	va_address = get_bus_address(pp, bus, devfn, where);

	*val = readl((u32 *)va_address);

	if (size == 1)
		*val = (*val >> (8 * (where & 3))) & 0xFF;
	else if (size == 2)
		*val = (*val >> (8 * (where & 3))) & 0xFFFF;

	return PCIBIOS_SUCCESSFUL;
}

static int imx_pcie_write_config(struct pci_bus *bus, u32 devfn,
			int where, int size, u32 val)
{
	struct imx_pcie_port *pp = bus_to_port(bus->number);
	u32 va_address = 0, mask = 0, tmp = 0;
	int ret = PCIBIOS_SUCCESSFUL;

	if (!pp) {
		BUG();
		return -EINVAL;
	}

	if (imx_pcie_valid_config(pp, bus, PCI_SLOT(devfn)) == 0)
		return PCIBIOS_DEVICE_NOT_FOUND;

	va_address = get_bus_address(pp, bus, devfn, where);

	if (size == 4) {
		writel(val, (u32 *)va_address);
		goto exit;
	}

	if (size == 2)
		mask = ~(0xFFFF << ((where & 0x3) * 8));
	else if (size == 1)
		mask = ~(0xFF << ((where & 0x3) * 8));
	else
		ret = PCIBIOS_BAD_REGISTER_NUMBER;

	tmp = readl((u32 *)va_address) & mask;
	tmp |= val << ((where & 0x3) * 8);
	writel(tmp, (u32 *)va_address);
exit:

	return ret;
}



static struct pci_ops imx_pcie_ops = {
	.read = imx_pcie_read_config,
	.write = imx_pcie_write_config,
};

static struct pci_bus __init *
imx_pcie_scan_bus(int nr, struct pci_sys_data *sys)
{
	struct imx_pcie_port *pp = controller_to_port(nr);
	if (nr > 1)
		return NULL;
        pp->root_bus_nr = sys->busnr;

        return pci_scan_root_bus(NULL, sys->busnr, &imx_pcie_ops, sys,
                                 &sys->resources);
}

static int __init imx_pcie_map_irq(const struct pci_dev *dev, u8 slot, u8 pin)
{
	struct imx_pcie_port *pp = controller_to_port(0);
	return pp->interrupt;
}

static struct hw_pci imx_pci __initdata = {
	.nr_controllers	= 1,
	.setup		= imx_pcie_setup,
	.scan		= imx_pcie_scan_bus,
	.map_irq	= imx_pcie_map_irq,
};

/* PHY CR bus acess routines */
static int pcie_phy_cr_ack_polling(void __iomem *dbi_base, int max_iterations, int exp_val)
{
	u32 temp_rd_data, wait_counter = 0;

	do {
		temp_rd_data = readl(dbi_base + PHY_STS_R);
		temp_rd_data = (temp_rd_data >> PCIE_CR_STAT_ACK_LOC) & 0x1;
		wait_counter++;
	} while ((wait_counter < max_iterations) && (temp_rd_data != exp_val));

	if (temp_rd_data != exp_val)
		return 0 ;
	return 1 ;
}

static int pcie_phy_cr_cap_addr(void __iomem *dbi_base, int addr)
{
	u32 temp_wr_data;

	/* write addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC ;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture addr */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_ADR_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* deassert cap addr */
	temp_wr_data = addr << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0 ;

	return 1 ;
}

static int pcie_phy_cr_read(void __iomem *dbi_base, int addr , int *data)
{
	u32 temp_rd_data, temp_wr_data;

	/*  write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(dbi_base, addr))
		return 0;

	/* assert rd signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_RD_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* after got ack return data */
	temp_rd_data = readl(dbi_base + PHY_STS_R);
	*data = (temp_rd_data & (0xffff << PCIE_CR_STAT_DATA_LOC)) ;

	/* deassert rd signal */
	temp_wr_data = 0x0;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0 ;

	return 1 ;

}

static int pcie_phy_cr_write(void __iomem *dbi_base, int addr, int data)
{
	u32 temp_wr_data;

	/* write addr */
	/* cap addr */
	if (!pcie_phy_cr_cap_addr(dbi_base, addr))
		return 0 ;

	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* capture data */
	temp_wr_data |= (0x1 << PCIE_CR_CTL_CAP_DAT_LOC);
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0 ;

	/* deassert cap data */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	/* assert wr signal */
	temp_wr_data = 0x1 << PCIE_CR_CTL_WR_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 1))
		return 0;

	/* deassert wr signal */
	temp_wr_data = data << PCIE_CR_CTL_DATA_LOC;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	/* wait for ack de-assetion */
	if (!pcie_phy_cr_ack_polling(dbi_base, 100, 0))
		return 0;

	temp_wr_data = 0x0 ;
	writel(temp_wr_data, dbi_base + PHY_CTRL_R);

	return 1 ;
}

static void change_field(int *in, int start, int end, int val)
{
	int mask;
	mask = ((0xFFFFFFFF << start) ^ (0xFFFFFFFF << (end + 1))) & 0xFFFFFFFF;
	*in = (*in & ~mask) | (val << start);
}

static int imx_pcie_enable_controller(struct platform_device *pdev)
{
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);
	int ret;

	/* Enable PCIE power */
	gpio_set_value(pp->pcie_pwr_en, 1);

	imx_pcie_clrset(pp, iomuxc_gpr1_test_powerdown, 0 << 18, IOMUXC_GPR1);

	imx_pcie_clrset(pp, iomuxc_gpr1_pcie_ref_clk_en, 1 << 16, IOMUXC_GPR1);


	/* Enable clocks */
	ret = clk_set_parent(pp->lvds1_sel, pp->sata_ref);
	if (ret) {
		dev_err(&pdev->dev, "unable to set lvds1 parent: %d\n", ret);
		return -EINVAL;
	}

	ret = clk_prepare_enable(pp->lvds1);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable lvds1: %d\n", ret);
		return -EINVAL;
	}

	ret = clk_prepare_enable(pp->pcie_ref_125m);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable pcie_ref_125m: %d\n", ret);
		return -EINVAL;
	}

	ret = clk_prepare_enable(pp->pcie_axi);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable pcie_axi: %d\n", ret);
		return -EINVAL;
	}


	return 0;
}

static void card_reset(struct platform_device *pdev)
{
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);

	/* activate PERST_B */
	gpio_set_value(pp->pcie_rst, 0);

	/* Add one reset to the pcie external device */
	msleep(100);

	/* deactive PERST_B */
	gpio_set_value(pp->pcie_rst, 1);
}

static void __init add_pcie_port(struct platform_device *pdev)
{
	struct clk *pcie_clk;
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);

	if (imx_pcie_link_up(pdev)) {
		pr_info("IMX PCIe port: link up.\n");
		pp->index = 0;
		pp->root_bus_nr = -1;
		spin_lock_init(&pp->conf_lock);
	} else {
		struct device *dev = &pdev->dev;
		pr_info("IMX PCIe port: link down!\n");
		/* Release the clocks, and disable the power */

		pcie_clk = of_clk_get(dev->of_node, 0);
		if (IS_ERR(pcie_clk))
			pr_err("no pcie clock.\n");

		clk_disable(pcie_clk);
		clk_put(pcie_clk);

		imx_pcie_clrset(pp, iomuxc_gpr1_pcie_ref_clk_en, 0 << 16,
				IOMUXC_GPR1);

		/* Disable PCIE power */
		gpio_set_value(pp->pcie_pwr_en, 0);

		imx_pcie_clrset(pp, iomuxc_gpr1_test_powerdown, 1 << 18,
				IOMUXC_GPR1);
	}
}


static int set_pcie_clock_tunings(struct platform_device *pdev)
{
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);
	/* FIXME the field name should be aligned to RM */
	imx_pcie_clrset(pp, iomuxc_gpr12_app_ltssm_enable, 0 << 10, IOMUXC_GPR12);

	/* configure constant input signal to the pcie ctrl and phy */
	imx_pcie_clrset(pp, iomuxc_gpr12_device_type, PCI_EXP_TYPE_ROOT_PORT << 12,
			IOMUXC_GPR12);
	imx_pcie_clrset(pp, iomuxc_gpr12_los_level, 9 << 4, IOMUXC_GPR12);

	imx_pcie_clrset(pp, iomuxc_gpr8_tx_deemph_gen1, 0 << 0, IOMUXC_GPR8);
	imx_pcie_clrset(pp, iomuxc_gpr8_tx_deemph_gen2_3p5db, 0 << 6, IOMUXC_GPR8);
	imx_pcie_clrset(pp, iomuxc_gpr8_tx_deemph_gen2_6db, 20 << 12, IOMUXC_GPR8);
	imx_pcie_clrset(pp, iomuxc_gpr8_tx_swing_full, 127 << 18, IOMUXC_GPR8);
	imx_pcie_clrset(pp, iomuxc_gpr8_tx_swing_low, 127 << 25, IOMUXC_GPR8);
	return 0;
}


static int __init imx_pcie_pltfm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imx_pcie_port *pp = devm_kzalloc(dev, sizeof(*pp), GFP_KERNEL);
	int ret;

	platform_set_drvdata(pdev, pp);
	pp->dev = &pdev->dev;

        pp->pcie_pwr_en = of_get_named_gpio(pdev->dev.of_node,
                                "power-enable", 0);
        if (gpio_is_valid(pp->pcie_pwr_en))
                devm_gpio_request_one(dev, pp->pcie_pwr_en,
                                    GPIOF_OUT_INIT_LOW,
                                    "PCIe power enable");

        pp->pcie_rst = of_get_named_gpio(pdev->dev.of_node,
                                "pcie-reset", 0);
        if (gpio_is_valid(pp->pcie_rst))
                devm_gpio_request_one(dev, pp->pcie_rst,
                                    GPIOF_OUT_INIT_LOW,
                                    "PCIe reset");

        pp->pcie_wake_up = of_get_named_gpio(pdev->dev.of_node,
                                "wake-up", 0);
        if (gpio_is_valid(pp->pcie_wake_up))
                devm_gpio_request_one(dev, pp->pcie_wake_up,
                                    GPIOF_OUT_INIT_LOW,
                                    "PCIe wake up");

        pp->pcie_dis = of_get_named_gpio(pdev->dev.of_node,
                                "disable-endpoint", 0);
        if (gpio_is_valid(pp->pcie_dis))
                devm_gpio_request_one(dev, pp->pcie_dis,
                                    GPIOF_OUT_INIT_LOW,
                                    "PCIe disable endpoint");


	pp->dbi = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!pp->dbi) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}

	pp->dbi_base = devm_request_and_ioremap(&pdev->dev, pp->dbi);
	if (!pp->dbi_base) {
		pr_err("unable to remap dbi\n");
		return -ENOMEM;
	}


	dev_dbg(dev, "Getting IO memory...\n");
	pp->io = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!pp->io) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}

	dev_dbg(dev, "Getting memmap memory...\n");
	pp->mem = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!pp->mem) {
		dev_err(dev, "no mmio space\n");
		return -EINVAL;
	}

	pp->root = platform_get_resource(pdev, IORESOURCE_MEM, 3);
	if (!pp->root) {
		dev_err(dev, "no root memory space\n");
		return -EINVAL;
	}

	pp->root_base = devm_request_and_ioremap(&pdev->dev, pp->root);
	if (!pp->root_base) {
		dev_err(&pdev->dev, "unable to remap root mem\n");
		return -ENOMEM;
	}


	pp->interrupt = platform_get_irq(pdev, 0);


        /* Setup clocks */
	pp->lvds1_sel = clk_get(dev, "lvds1_sel");
	if (IS_ERR(pp->lvds1_sel)) {
		dev_err(dev,
			"lvds1_sel clock missing or invalid\n");
		return -EINVAL;
	}

	pp->lvds1 = clk_get(dev, "lvds1");
	if (IS_ERR(pp->lvds1)) {
		dev_err(dev,
			"lvds1 clock select missing or invalid\n");
		return -EINVAL;
	}

	pp->pcie_ref_125m = clk_get(dev, "pcie_ref_125m");
	if (IS_ERR(pp->pcie_ref_125m)) {
		dev_err(dev,
			"pcie_ref_125m clock source missing or invalid\n");
		return -EINVAL;
	}

	pp->pcie_axi = clk_get(dev, "pcie_axi");
	if (IS_ERR(pp->pcie_axi)) {
		dev_err(dev, "pcie_axi clock source missing or invalid\n");
		return -EINVAL;
	}

	pp->sata_ref = clk_get(dev, "sata_ref");
	if (IS_ERR(pp->sata_ref)) {
		dev_err(dev, "sata_ref clock source missing or invalid\n");
		return -EINVAL;
	}

	pp->iomuxc_gpr = syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(pp->iomuxc_gpr)) {
		dev_err(dev, "unable to find iomuxc registers\n");
		return -EINVAL;
	}



	/* Enable the pwr, clks and so on */
	set_pcie_clock_tunings(pdev);
	ret = imx_pcie_enable_controller(pdev);
	if (ret)
		goto err_out;

	/* togle the external card's reset */
	card_reset(pdev) ;

	usleep_range(3000, 4000);
	imx_pcie_regions_setup(pdev, pp);
	usleep_range(3000, 4000);

	/* start link up */
	imx_pcie_clrset(pp, iomuxc_gpr12_app_ltssm_enable, 1 << 10, IOMUXC_GPR12);

	/* add the pcie port */
	add_pcie_port(pdev);

	pp->index = imx_pcie.nr_controllers;
	imx_pcie.nr_controllers++;
	list_add_tail(&pp->next, &pcie_port_list);

	pci_common_init(&imx_pci);

#if defined(CONFIG_RFKILL) || defined(CONFIG_RFKILL_MODULE)
/**
 * rfkill_alloc - allocate rfkill structure
 * @name: name of the struct -- the string is not copied internally
 * @parent: device that has rf switch on it
 * @type: type of the switch (RFKILL_TYPE_*)
 * @ops: rfkill methods
 * @ops_data: data passed to each method
 *
 * This function should be called by the transmitter driver to allocate an
 * rfkill structure. Returns %NULL on failure.
 */
        if (gpio_is_valid(pp->pcie_dis)) {
		pp->rfkill = rfkill_alloc("PCIe rfkill", dev, RFKILL_TYPE_WLAN, 
					&imx_pcie_rfkill_ops, pp);
		if (!pp->rfkill) {
			dev_err(dev, "unable to allocate rfkill driver\n");
		}
		ret = rfkill_register(pp->rfkill);
		if (ret) {
			dev_err(dev, "unable to register rfkill driver\n");
			rfkill_destroy(pp->rfkill);
			pp->rfkill = NULL;
		}
	}
#endif /* RFKILL || RFKILL_MODULE */


	return 0;

err_out:
	return ret;
}

static int __exit imx_pcie_pltfm_remove(struct platform_device *pdev)
{
	struct clk *pcie_clk;
	struct device *dev = &pdev->dev;
	struct imx_pcie_port *pp = platform_get_drvdata(pdev);

	if (pp->rfkill) {
		rfkill_unregister(pp->rfkill);
		rfkill_destroy(pp->rfkill);
		pp->rfkill = NULL;
	}

	/* Release clocks, and disable power  */
	pcie_clk = of_clk_get(dev->of_node, 0);
	if (IS_ERR(pcie_clk))
		pr_err("no pcie clock.\n");

	if (pcie_clk) {
		clk_disable(pcie_clk);
		clk_put(pcie_clk);
	}

	imx_pcie_clrset(pp, iomuxc_gpr1_pcie_ref_clk_en, 0 << 16, IOMUXC_GPR1);

	imx_pcie_clrset(pp, iomuxc_gpr1_test_powerdown, 1 << 18, IOMUXC_GPR1);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver imx_pcie_pltfm_driver = {
	.driver = {
		.name		= "imx-pcie",
		.owner		= THIS_MODULE,
		.of_match_table = pcie_of_match,
	},
	.probe		= imx_pcie_pltfm_probe,
	.remove		= __exit_p(imx_pcie_pltfm_remove),
};

/*****************************************************************************\
 *                                                                           *
 * Driver init/exit                                                          *
 *                                                                           *
\*****************************************************************************/

static int __init imx_pcie_drv_init(void)
{
	INIT_LIST_HEAD(&pcie_port_list);
	return platform_driver_register(&imx_pcie_pltfm_driver);
}

static void __exit imx_pcie_drv_exit(void)
{
	platform_driver_unregister(&imx_pcie_pltfm_driver);
}

module_init(imx_pcie_drv_init);
module_exit(imx_pcie_drv_exit);

MODULE_DESCRIPTION("i.MX PCIE platform driver");
MODULE_LICENSE("GPL v2");
