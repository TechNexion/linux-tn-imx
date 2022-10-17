// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller driver for Freescale i.MX6 SoCs
 *
 * Copyright (C) 2013 Kosagi
 *		https://www.kosagi.com
 *
 * Author: Sean Cross <xobs@kosagi.com>
 */

#include <dt-bindings/soc/imx8_hsio.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/phy/phy.h>
#include <linux/reset.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include "../../pci.h"

#include "pcie-designware.h"

#define IMX8MQ_PCIE_LINK_CAP_REG_OFFSET		0x7c
#define IMX8MQ_PCIE_LINK_CAP_L1EL_64US		GENMASK(18, 17)
#define IMX8MQ_PCIE_L1SUB_CTRL1_REG_EN_MASK	0xf
#define IMX8MQ_GPR_PCIE_REF_USE_PAD		BIT(9)
#define IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN	BIT(10)
#define IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE	BIT(11)
#define IMX8MQ_GPR_PCIE_VREG_BYPASS		BIT(12)
#define IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE	GENMASK(11, 8)
#define IMX8MQ_PCIE2_BASE_ADDR			0x33c00000
#define IMX8_HSIO_PCIEB_BASE_ADDR		0x5f010000
#define IMX8MP_GPR_REG0				0x0
#define IMX8MP_GPR_REG0_CLK_MOD_EN		BIT(0)
#define IMX8MP_GPR_REG0_PHY_APB_RST		BIT(4)
#define IMX8MP_GPR_REG0_PHY_INIT_RST		BIT(5)
#define IMX8MP_GPR_REG1				0x4
#define IMX8MP_GPR_REG1_PM_EN_CORE_CLK		BIT(0)
#define IMX8MP_GPR_REG1_PLL_LOCK		BIT(13)
#define IMX8MP_GPR_REG2				0x8
#define IMX8MP_GPR_REG2_P_PLL_MASK		GENMASK(5, 0)
#define IMX8MP_GPR_REG2_M_PLL_MASK		GENMASK(15, 6)
#define IMX8MP_GPR_REG2_S_PLL_MASK		GENMASK(18, 16)
#define IMX8MP_GPR_REG2_P_PLL			(0xc << 0)
#define IMX8MP_GPR_REG2_M_PLL			(0x320 << 6)
#define IMX8MP_GPR_REG2_S_PLL			(0x4 << 16)
#define IMX8MP_GPR_REG3				0xc
#define IMX8MP_GPR_REG3_PLL_CKE			BIT(17)
#define IMX8MP_GPR_REG3_PLL_RST			BIT(31)
#define IMX8MP_GPR_PCIE_SSC_EN			BIT(16)
#define IMX8MP_GPR_PCIE_PWR_OFF			BIT(17)
#define IMX8MP_GPR_PCIE_CMN_RSTN		BIT(18)
#define IMX8MP_GPR_PCIE_AUX_EN			BIT(19)
#define IMX8MP_GPR_PCIE_REF_SEL_MASK		GENMASK(25, 24)
#define IMX8MP_GPR_PCIE_REF_PLL_SYS		GENMASK(25, 24)
#define IMX8MP_GPR_PCIE_REF_EXT_OSC		BIT(25)

#define to_imx6_pcie(x)	dev_get_drvdata((x)->dev)

enum imx6_pcie_variants {
	IMX6Q,
	IMX6SX,
	IMX6QP,
	IMX7D,
	IMX8MQ,
	IMX8MM,
	IMX8QM,
	IMX8QXP,
	IMX8MP,
	IMX8QXP_EP,
	IMX8QM_EP,
	IMX8MQ_EP,
	IMX8MM_EP,
	IMX8MP_EP,
	IMX6SX_EP,
	IMX7D_EP,
	IMX6Q_EP,
	IMX6QP_EP,
};

#define IMX6_PCIE_FLAG_IMX6_PHY			BIT(0)
#define IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE	BIT(1)
#define IMX6_PCIE_FLAG_SUPPORTS_SUSPEND		BIT(2)
#define IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP	BIT(3)
#define IMX6_PCIE_FLAG_SUPPORTS_L1SS		BIT(4)

struct imx6_pcie_drvdata {
	enum imx6_pcie_variants variant;
	enum dw_pcie_device_mode mode;
	u32 flags;
	int dbi_length;
};

struct imx6_pcie {
	struct dw_pcie		*pci;
	int			clkreq_gpio;
	int			dis_gpio;
	int			reset_gpio;
	bool			gpio_active_high;
	struct clk		*pcie_bus;
	struct clk		*pcie_phy;
	struct clk		*pcie_phy_pclk;
	struct clk		*pcie_per;
	struct clk		*pciex2_per;
	struct clk		*pcie_inbound_axi;
	struct clk		*pcie;
	struct clk		*pcie_aux;
	struct clk		*phy_per;
	struct clk		*misc_per;
	struct regmap		*iomuxc_gpr;
	u32			controller_id;
	struct reset_control	*pciephy_reset;
	struct reset_control	*pciephy_perst;
	struct reset_control	*apps_reset;
	struct reset_control	*turnoff_reset;
	struct reset_control	*clkreq_reset;
	u32			tx_deemph_gen1;
	u32			tx_deemph_gen2_3p5db;
	u32			tx_deemph_gen2_6db;
	u32			tx_swing_full;
	u32			tx_swing_low;
	u32			hsio_cfg;
	u32			ext_osc;
	u32			local_addr;
	u32			l1ss_clkreq;
	int			link_gen;
	struct regulator	*vpcie;
	struct regulator	*vph;
	void __iomem		*phy_base;
	void __iomem		*hsmix_base;

	/* power domain for pcie */
	struct device		*pd_pcie;
	/* power domain for pcie csr access */
	struct device		*pd_pcie_per;
	/* power domain for pcie phy */
	struct device		*pd_pcie_phy;
	/* power domain for hsio gpio used by pcie */
	struct device		*pd_hsio_gpio;
	struct device_link	*pd_link;
	struct device_link	*pd_per_link;
	struct device_link	*pd_phy_link;
	struct device_link	*pd_hsio_link;

	const struct imx6_pcie_drvdata *drvdata;
	struct regulator	*epdev_on;
	struct phy		*phy;
};

/* Parameters for the waiting for PCIe PHY PLL to lock on i.MX7 */
#define PHY_PLL_LOCK_WAIT_USLEEP_MAX	200
#define PHY_PLL_LOCK_WAIT_TIMEOUT	(2000 * PHY_PLL_LOCK_WAIT_USLEEP_MAX)

/* PCIe Port Logic registers (memory-mapped) */
#define PL_OFFSET 0x700

#define PCIE_PHY_CTRL (PL_OFFSET + 0x114)
#define PCIE_PHY_CTRL_DATA(x)		FIELD_PREP(GENMASK(15, 0), (x))
#define PCIE_PHY_CTRL_CAP_ADR		BIT(16)
#define PCIE_PHY_CTRL_CAP_DAT		BIT(17)
#define PCIE_PHY_CTRL_WR		BIT(18)
#define PCIE_PHY_CTRL_RD		BIT(19)

#define PCIE_PHY_STAT (PL_OFFSET + 0x110)
#define PCIE_PHY_STAT_ACK		BIT(16)

#define PCIE_LINK_WIDTH_SPEED_CONTROL	0x80C

/* PHY registers (not memory-mapped) */
#define PCIE_PHY_ATEOVRD			0x10
#define  PCIE_PHY_ATEOVRD_EN			BIT(2)
#define  PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT	0
#define  PCIE_PHY_ATEOVRD_REF_CLKDIV_MASK	0x1

#define PCIE_PHY_MPLL_OVRD_IN_LO		0x11
#define  PCIE_PHY_MPLL_MULTIPLIER_SHIFT		2
#define  PCIE_PHY_MPLL_MULTIPLIER_MASK		0x7f
#define  PCIE_PHY_MPLL_MULTIPLIER_OVRD		BIT(9)

/* iMX7 PCIe PHY registers */
#define PCIE_PHY_CMN_REG4		0x14
/* These are probably the bits that *aren't* DCC_FB_EN */
#define PCIE_PHY_CMN_REG4_DCC_FB_EN	0x29

#define PCIE_PHY_CMN_REG24		0x90
#define PCIE_PHY_CMN_REG24_RX_EQ	BIT(6)
#define PCIE_PHY_CMN_REG24_RX_EQ_SEL	BIT(3)

#define PCIE_PHY_CMN_REG26		0x98
#define PCIE_PHY_CMN_REG26_ATT_MODE	0xBC

#define PCIE_PHY_CMN_REG62			0x188
#define PCIE_PHY_CMN_REG62_PLL_CLK_OUT		0x08
#define PCIE_PHY_CMN_REG64			0x190
#define PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM	0x8C
#define PCIE_PHY_CMN_REG75			0x1D4
#define PCIE_PHY_CMN_REG75_PLL_DONE		0x3
#define PCIE_PHY_TRSV_REG5			0x414
#define PCIE_PHY_TRSV_REG5_GEN1_DEEMP		0x2D
#define PCIE_PHY_TRSV_REG6			0x418
#define PCIE_PHY_TRSV_REG6_GEN2_DEEMP		0xF

#define PHY_RX_OVRD_IN_LO 0x1005
#define PHY_RX_OVRD_IN_LO_RX_DATA_EN		BIT(5)
#define PHY_RX_OVRD_IN_LO_RX_PLL_EN		BIT(3)

/* iMX8 HSIO registers */
#define IMX8QM_PHYX2_LPCG_OFFSET		0x00000
#define	IMX8QM_PHYX2_LPCG_PCLK0_MASK		GENMASK(17, 16)
#define	IMX8QM_PHYX2_LPCG_PCLK1_MASK		GENMASK(21, 20)
#define IMX8QM_CSR_PHYX2_OFFSET			0x90000
#define IMX8QM_CSR_PHYX1_OFFSET			0xA0000
#define IMX8QM_CSR_PHYX_STTS0_OFFSET		0x4
#define IMX8QM_CSR_PCIEA_OFFSET			0xB0000
#define IMX8QM_CSR_PCIEB_OFFSET			0xC0000
#define IMX8QM_CSR_PCIE_CTRL1_OFFSET		0x4
#define IMX8QM_CSR_PCIE_CTRL2_OFFSET		0x8
#define IMX8QM_CSR_PCIE_STTS0_OFFSET		0xC
#define IMX8QM_CSR_MISC_OFFSET			0xE0000

#define IMX8QM_CTRL_LTSSM_ENABLE		BIT(4)
#define IMX8QM_CTRL_READY_ENTR_L23		BIT(5)
#define IMX8QM_CTRL_PM_XMT_TURNOFF		BIT(9)
#define IMX8QM_CTRL_BUTTON_RST_N		BIT(21)
#define IMX8QM_CTRL_PERST_N			BIT(22)
#define IMX8QM_CTRL_POWER_UP_RST_N		BIT(23)

#define IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2	BIT(13)
#define IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST	BIT(19)
#define IMX8QM_STTS0_LANE0_TX_PLL_LOCK		BIT(4)
#define IMX8QM_STTS0_LANE1_TX_PLL_LOCK		BIT(12)

#define IMX8QM_PCIE_TYPE_MASK			GENMASK(27, 24)

#define IMX8QM_PHYX2_CTRL0_APB_MASK		0x3
#define IMX8QM_PHY_APB_RSTN_0			BIT(0)
#define IMX8QM_PHY_APB_RSTN_1			BIT(1)

#define IMX8QM_MISC_IOB_RXENA			BIT(0)
#define IMX8QM_MISC_IOB_TXENA			BIT(1)
#define IMX8QM_CSR_MISC_IOB_A_0_TXOE		BIT(2)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK	(0x3 << 3)
#define IMX8QM_CSR_MISC_IOB_A_0_M1M0_2		BIT(4)
#define IMX8QM_MISC_PHYX1_EPCS_SEL		BIT(12)
#define IMX8QM_MISC_PCIE_AB_SELECT		BIT(13)
#define IMX8QM_MISC_CLKREQ_1			BIT(22)
#define IMX8QM_MISC_CLKREQ_0			BIT(23)
#define IMX8QM_MISC_CLKREQ_OVERRIDE_EN_1	BIT(24)
#define IMX8QM_MISC_CLKREQ_OVERRIDE_EN_0	BIT(25)

#define IMX8MM_GPR_PCIE_REF_CLK_SEL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_PLL		(0x3 << 24)
#define IMX8MM_GPR_PCIE_REF_CLK_EXT		(0x2 << 24)
#define IMX8MM_GPR_PCIE_AUX_EN			BIT(19)
#define IMX8MM_GPR_PCIE_CMN_RST			BIT(18)
#define IMX8MM_GPR_PCIE_POWER_OFF		BIT(17)
#define IMX8MM_GPR_PCIE_SSC_EN			BIT(16)

static int imx6_pcie_cz_enabled;
static void imx6_pcie_ltssm_disable(struct device *dev);

static bool imx6_pcie_readable_reg(struct device *dev, unsigned int reg)
{
	enum imx6_pcie_variants variant;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	variant = imx6_pcie->drvdata->variant;
	if (variant == IMX8QXP || variant == IMX8QXP_EP) {
		switch (reg) {
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PHYX1_OFFSET + IMX8QM_CSR_PHYX_STTS0_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_STTS0_OFFSET:
			return true;

		default:
			return false;
		}
	} else {
		switch (reg) {
		case IMX8QM_PHYX2_LPCG_OFFSET:
		case IMX8QM_CSR_PHYX2_OFFSET:
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PHYX2_OFFSET + IMX8QM_CSR_PHYX_STTS0_OFFSET:
		case IMX8QM_CSR_PHYX1_OFFSET + IMX8QM_CSR_PHYX_STTS0_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_STTS0_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_STTS0_OFFSET:
			return true;
		default:
			return false;
		}
	}
}

static bool imx6_pcie_writeable_reg(struct device *dev, unsigned int reg)
{
	enum imx6_pcie_variants variant;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	variant = imx6_pcie->drvdata->variant;
	if (variant == IMX8QXP || variant == IMX8QXP_EP) {
		switch (reg) {
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
			return true;

		default:
			return false;
		}
	} else {
		switch (reg) {
		case IMX8QM_PHYX2_LPCG_OFFSET:
		case IMX8QM_CSR_PHYX2_OFFSET:
		case IMX8QM_CSR_PHYX1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET:
		case IMX8QM_CSR_MISC_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEA_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL1_OFFSET:
		case IMX8QM_CSR_PCIEB_OFFSET + IMX8QM_CSR_PCIE_CTRL2_OFFSET:
			return true;
		default:
			return false;
		}
	}
}

static const struct regmap_config imx6_pcie_regconfig = {
	.max_register = IMX8QM_CSR_MISC_OFFSET,
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.val_format_endian = REGMAP_ENDIAN_NATIVE,
	.num_reg_defaults_raw =  IMX8QM_CSR_MISC_OFFSET / sizeof(uint32_t) + 1,
	.readable_reg = imx6_pcie_readable_reg,
	.writeable_reg = imx6_pcie_writeable_reg,
	.cache_type = REGCACHE_NONE,
};

static int pcie_phy_poll_ack(struct imx6_pcie *imx6_pcie, bool exp_val)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	bool val;
	u32 max_iterations = 10;
	u32 wait_counter = 0;

	do {
		val = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT) &
			PCIE_PHY_STAT_ACK;
		wait_counter++;

		if (val == exp_val)
			return 0;

		udelay(1);
	} while (wait_counter < max_iterations);

	return -ETIMEDOUT;
}

static int pcie_phy_wait_ack(struct imx6_pcie *imx6_pcie, int addr)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 val;
	int ret;

	val = PCIE_PHY_CTRL_DATA(addr);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	val |= PCIE_PHY_CTRL_CAP_ADR;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	val = PCIE_PHY_CTRL_DATA(addr);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, val);

	return pcie_phy_poll_ack(imx6_pcie, false);
}

/* Read from the 16-bit PCIe PHY control registers (not memory-mapped) */
static int pcie_phy_read(struct imx6_pcie *imx6_pcie, int addr, u16 *data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 phy_ctl;
	int ret;

	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	/* assert Read signal */
	phy_ctl = PCIE_PHY_CTRL_RD;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, phy_ctl);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	*data = dw_pcie_readl_dbi(pci, PCIE_PHY_STAT);

	/* deassert Read signal */
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x00);

	return pcie_phy_poll_ack(imx6_pcie, false);
}

static int pcie_phy_write(struct imx6_pcie *imx6_pcie, int addr, u16 data)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	u32 var;
	int ret;

	/* write addr */
	/* cap addr */
	ret = pcie_phy_wait_ack(imx6_pcie, addr);
	if (ret)
		return ret;

	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* capture data */
	var |= PCIE_PHY_CTRL_CAP_DAT;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	/* deassert cap data */
	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, false);
	if (ret)
		return ret;

	/* assert wr signal */
	var = PCIE_PHY_CTRL_WR;
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack */
	ret = pcie_phy_poll_ack(imx6_pcie, true);
	if (ret)
		return ret;

	/* deassert wr signal */
	var = PCIE_PHY_CTRL_DATA(data);
	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, var);

	/* wait for ack de-assertion */
	ret = pcie_phy_poll_ack(imx6_pcie, false);
	if (ret)
		return ret;

	dw_pcie_writel_dbi(pci, PCIE_PHY_CTRL, 0x0);

	return 0;
}

static void imx6_pcie_reset_phy(struct imx6_pcie *imx6_pcie)
{
	u16 tmp;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_PHY))
		return;

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp |= (PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);

	usleep_range(2000, 3000);

	pcie_phy_read(imx6_pcie, PHY_RX_OVRD_IN_LO, &tmp);
	tmp &= ~(PHY_RX_OVRD_IN_LO_RX_DATA_EN |
		  PHY_RX_OVRD_IN_LO_RX_PLL_EN);
	pcie_phy_write(imx6_pcie, PHY_RX_OVRD_IN_LO, tmp);
}

#ifdef CONFIG_ARM
/*  Added for PCI abort handling */
static int imx6q_pcie_abort_handler(unsigned long addr,
		unsigned int fsr, struct pt_regs *regs)
{
	unsigned long pc = instruction_pointer(regs);
	unsigned long instr;
	int reg;

	/* if the abort from user-space, just return and report it */
	if (user_mode(regs))
		return 1;

	instr = *(unsigned long *)pc;
	reg = (instr >> 12) & 15;

	/*
	 * If the instruction being executed was a read,
	 * make it look like it read all-ones.
	 */
	if ((instr & 0x0c100000) == 0x04100000) {
		unsigned long val;

		if (instr & 0x00400000)
			val = 255;
		else
			val = -1;

		regs->uregs[reg] = val;
		regs->ARM_pc += 4;
		return 0;
	}

	if ((instr & 0x0e100090) == 0x00100090) {
		regs->uregs[reg] = -1;
		regs->ARM_pc += 4;
		return 0;
	}

	return 1;
}
#endif

static void imx6_pcie_detach_pd(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	if (imx6_pcie->pd_hsio_link && !IS_ERR(imx6_pcie->pd_hsio_link))
		device_link_del(imx6_pcie->pd_hsio_link);
	if (imx6_pcie->pd_hsio_gpio && !IS_ERR(imx6_pcie->pd_hsio_gpio))
		dev_pm_domain_detach(imx6_pcie->pd_hsio_gpio, true);
	if (imx6_pcie->pd_phy_link && !IS_ERR(imx6_pcie->pd_phy_link))
		device_link_del(imx6_pcie->pd_phy_link);
	if (imx6_pcie->pd_pcie_phy && !IS_ERR(imx6_pcie->pd_pcie_phy))
		dev_pm_domain_detach(imx6_pcie->pd_pcie_phy, true);
	if (imx6_pcie->pd_per_link && !IS_ERR(imx6_pcie->pd_per_link))
		device_link_del(imx6_pcie->pd_per_link);
	if (imx6_pcie->pd_pcie_per && !IS_ERR(imx6_pcie->pd_pcie_per))
		dev_pm_domain_detach(imx6_pcie->pd_pcie_per, true);
	if (imx6_pcie->pd_link && !IS_ERR(imx6_pcie->pd_link))
		device_link_del(imx6_pcie->pd_link);
	if (imx6_pcie->pd_pcie && !IS_ERR(imx6_pcie->pd_pcie))
		dev_pm_domain_detach(imx6_pcie->pd_pcie, true);

	imx6_pcie->pd_hsio_gpio = NULL;
	imx6_pcie->pd_hsio_link = NULL;
	imx6_pcie->pd_pcie_phy = NULL;
	imx6_pcie->pd_phy_link = NULL;
	imx6_pcie->pd_pcie_per = NULL;
	imx6_pcie->pd_per_link = NULL;
	imx6_pcie->pd_pcie = NULL;
	imx6_pcie->pd_link = NULL;
}

static int imx6_pcie_attach_pd(struct device *dev)
{
	int ret = 0;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct device_link *link;
	struct device *pd_dev;

	/* Do nothing when in a single power domain */
	if (dev->pm_domain)
		return 0;

	imx6_pcie->pd_pcie = dev_pm_domain_attach_by_name(dev, "pcie");
	if (IS_ERR(imx6_pcie->pd_pcie))
		return PTR_ERR(imx6_pcie->pd_pcie);
	/* Do nothing when power domain missing */
	if (!imx6_pcie->pd_pcie)
		return 0;
	link = device_link_add(dev, imx6_pcie->pd_pcie,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
	if (!link) {
		dev_err(dev, "Failed to add device_link to pcie pd.\n");
		return -EINVAL;
	} else {
		imx6_pcie->pd_link = link;
	}

	imx6_pcie->pd_pcie_phy = dev_pm_domain_attach_by_name(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pd_pcie_phy)) {
		ret = PTR_ERR(imx6_pcie->pd_pcie_phy);
		goto err_ret;
	}

	link = device_link_add(dev, imx6_pcie->pd_pcie_phy,
			DL_FLAG_STATELESS |
			DL_FLAG_PM_RUNTIME |
			DL_FLAG_RPM_ACTIVE);
	if (!link) {
		dev_err(dev, "Failed to add device_link to pcie_phy pd.\n");
		ret = -EINVAL;
		goto err_ret;
	} else {
		imx6_pcie->pd_phy_link = link;
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QM:
	case IMX8QM_EP:
		/*
		 * PCIA CSR would be touched during the initialization of the
		 * PCIEB of 8QM.
		 * Enable the PCIEA PD for this case here.
		 */
		if (imx6_pcie->controller_id) {
			pd_dev = dev_pm_domain_attach_by_name(dev, "pcie_per");
			if (IS_ERR(pd_dev)) {
				ret = PTR_ERR(pd_dev);
				goto err_ret;
			} else {
				imx6_pcie->pd_pcie_per = pd_dev;
			}
			link = device_link_add(dev, imx6_pcie->pd_pcie_per,
					DL_FLAG_STATELESS |
					DL_FLAG_PM_RUNTIME |
					DL_FLAG_RPM_ACTIVE);
			if (!link) {
				dev_err(dev, "Failed to link pcie_per pd\n");
				ret = -EINVAL;
				goto err_ret;
			} else {
				imx6_pcie->pd_per_link = link;
			}
		}
		fallthrough;
	case IMX8QXP:
	case IMX8QXP_EP:
		pd_dev = dev_pm_domain_attach_by_name(dev, "hsio_gpio");
		if (IS_ERR(pd_dev)) {
			ret = PTR_ERR(pd_dev);
			goto err_ret;
		} else {
			imx6_pcie->pd_hsio_gpio = pd_dev;
		}

		link = device_link_add(dev, imx6_pcie->pd_hsio_gpio,
				DL_FLAG_STATELESS |
				DL_FLAG_PM_RUNTIME |
				DL_FLAG_RPM_ACTIVE);
		if (!link) {
			dev_err(dev, "Failed to add device_link to hsio_gpio pd.\n");
			ret = -EINVAL;
			goto err_ret;
		} else {
			imx6_pcie->pd_hsio_link = link;
		}

		break;
	default:
		break;
	}

	return 0;

err_ret:
	imx6_pcie_detach_pd(dev);
	return ret;
}

static unsigned int imx6_pcie_grp_offset(const struct imx6_pcie *imx6_pcie)
{
	return imx6_pcie->controller_id == 1 ? IOMUXC_GPR16 : IOMUXC_GPR14;
}

static int imx6_pcie_enable_ref_clk(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	unsigned int offset;
	int ret = 0;

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
	case IMX6SX_EP:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			break;
		}

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN, 0);
		break;
	case IMX6QP:
	case IMX6QP_EP:
	case IMX6Q:
	case IMX6Q_EP:
		/* power up core phy and enable ref clock */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 0 << 18);
		/*
		 * the async reset input need ref clock to sync internally,
		 * when the ref clock comes after reset, internal synced
		 * reset time is too short, cannot meet the requirement.
		 * add one ~10us delay here.
		 */
		usleep_range(10, 100);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 1 << 16);
		break;
	case IMX7D:
	case IMX7D_EP:
		break;
	case IMX8MQ:
	case IMX8MM:
	case IMX8MP:
	case IMX8MQ_EP:
	case IMX8MM_EP:
	case IMX8MP_EP:
		ret = clk_prepare_enable(imx6_pcie->pcie_aux);
		if (ret) {
			dev_err(dev, "unable to enable pcie_aux clock\n");
			break;
		}

		offset = imx6_pcie_grp_offset(imx6_pcie);
		/*
		 * Set the over ride low and enabled
		 * make sure that REF_CLK is turned on.
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE,
				   0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
				   IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN);
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		ret = clk_prepare_enable(imx6_pcie->pcie_inbound_axi);
		if (ret) {
			dev_err(dev, "unable to enable pcie_axi clock\n");
			return ret;
		}
		ret = clk_prepare_enable(imx6_pcie->pcie_per);
		if (ret) {
			dev_err(dev, "unable to enable pcie_per clock\n");
			goto err_pcie_per;
		}

		ret = clk_prepare_enable(imx6_pcie->phy_per);
		if (unlikely(ret)) {
			dev_err(dev, "unable to enable phy per clock\n");
			goto err_phy_per;
		}
		ret = clk_prepare_enable(imx6_pcie->misc_per);
		if (unlikely(ret)) {
			dev_err(dev, "unable to enable misc per clock\n");
			goto err_misc_per;
		}
		/*
		 * PCIA CSR would be touched during the initialization of the
		 * PCIEB of 8QM.
		 * Enable the PCIEA peripheral clock for this case here.
		 */
		if (imx6_pcie->drvdata->variant == IMX8QM
				&& imx6_pcie->controller_id == 1) {
			ret = clk_prepare_enable(imx6_pcie->pcie_phy_pclk);
			if (unlikely(ret)) {
				dev_err(dev, "can't enable pciephyp clock\n");
				goto err_pcie_phy_pclk;
			}
			ret = clk_prepare_enable(imx6_pcie->pciex2_per);
			if (unlikely(ret)) {
				dev_err(dev, "can't enable pciex2 per clock\n");
				goto err_pciex2_per;
			}
		}
		break;
	default:
		break;
	}

	return ret;
err_pciex2_per:
	clk_disable_unprepare(imx6_pcie->pcie_phy_pclk);
err_pcie_phy_pclk:
	clk_disable_unprepare(imx6_pcie->misc_per);
err_misc_per:
	clk_disable_unprepare(imx6_pcie->phy_per);
err_phy_per:
	clk_disable_unprepare(imx6_pcie->pcie_per);
err_pcie_per:
	clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
	return ret;
}

static void imx7d_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	struct device *dev = imx6_pcie->pci->dev;

	if (regmap_read_poll_timeout(imx6_pcie->iomuxc_gpr,
				     IOMUXC_GPR22, val,
				     val & IMX7D_GPR22_PCIE_PHY_PLL_LOCKED,
				     PHY_PLL_LOCK_WAIT_USLEEP_MAX,
				     PHY_PLL_LOCK_WAIT_TIMEOUT))
		dev_err(dev, "PCIe PLL lock timeout\n");
}

static void imx8_pcie_wait_for_phy_pll_lock(struct imx6_pcie *imx6_pcie)
{
	u32 val, offset = 0, cond = 0;
	int ret = 0;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8MP:
	case IMX8MP_EP:
		if (phy_init(imx6_pcie->phy) != 0)
			dev_err(dev, "Waiting for PHY PLL ready timeout!\n");
		/* wait for core_clk enabled */
		for (cond = 0; cond < 2000; cond++) {
			val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG1);
			if (val & IMX8MP_GPR_REG1_PM_EN_CORE_CLK)
				break;
			udelay(10);
		}
		if (cond >= 2000)
			ret = -ETIMEDOUT;
		break;
	case IMX8MM:
	case IMX8MM_EP:
		for (cond = 0; cond < 2000; cond++) {
			val = readl(imx6_pcie->phy_base + PCIE_PHY_CMN_REG75);
			if (val == PCIE_PHY_CMN_REG75_PLL_DONE)
				break;
			udelay(10);
		}
		if (cond >= 2000)
			ret = -ETIMEDOUT;
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		if (imx6_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
			if (imx6_pcie->controller_id == 0) /* pciea 1 lanes */
				cond = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			else /* pcieb 1 lanes */
				cond = IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
			offset = IMX8QM_CSR_PHYX2_OFFSET + 0x4;
		} else if (imx6_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
			offset = IMX8QM_CSR_PHYX2_OFFSET
				+ imx6_pcie->controller_id * SZ_64K
				+ IMX8QM_CSR_PHYX_STTS0_OFFSET;
			cond = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			if (imx6_pcie->controller_id == 0) /* pciea 2 lanes */
				cond |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
		} else if (imx6_pcie->hsio_cfg == PCIEAX2SATA) {
			offset = IMX8QM_CSR_PHYX2_OFFSET + 0x4;
			cond = IMX8QM_STTS0_LANE0_TX_PLL_LOCK;
			cond |= IMX8QM_STTS0_LANE1_TX_PLL_LOCK;
		}

		ret = regmap_read_poll_timeout(imx6_pcie->iomuxc_gpr, offset,
					       val, val & cond,
					       PHY_PLL_LOCK_WAIT_USLEEP_MAX,
					       PHY_PLL_LOCK_WAIT_TIMEOUT);
		break;
	default:
		break;
	}
	if (ret)
		dev_err(dev, "PCIe PLL lock timeout\n");
	else
		dev_info(dev, "PCIe PLL is locked.\n");
}

static void imx6_pcie_clk_enable(struct imx6_pcie *imx6_pcie)
{
	int ret;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	ret = clk_prepare_enable(imx6_pcie->pcie_phy);
	if (ret)
		dev_err(dev, "unable to enable pcie_phy clock\n");

	ret = clk_prepare_enable(imx6_pcie->pcie_bus);
	if (ret)
		dev_err(dev, "unable to enable pcie_bus clock\n");

	ret = clk_prepare_enable(imx6_pcie->pcie);
	if (ret)
		dev_err(dev, "unable to enable pcie clock\n");

	ret = imx6_pcie_enable_ref_clk(imx6_pcie);
	if (ret)
		dev_err(dev, "unable to enable pcie ref clock\n");

	/* allow the clocks to stabilize */
	usleep_range(200, 500);
}

static void imx6_pcie_clk_disable(struct imx6_pcie *imx6_pcie)
{
	clk_disable_unprepare(imx6_pcie->pcie);
	clk_disable_unprepare(imx6_pcie->pcie_phy);
	clk_disable_unprepare(imx6_pcie->pcie_bus);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6Q:
	case IMX6Q_EP:
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_REF_CLK_EN, 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD,
				IMX6Q_GPR1_PCIE_TEST_PD);
		break;
	case IMX6SX:
	case IMX6SX_EP:
		clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		break;
	case IMX7D:
	case IMX7D_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL);
		break;
	case IMX8MP:
	case IMX8MP_EP:
		phy_exit(imx6_pcie->phy);
		phy_power_off(imx6_pcie->phy);
		fallthrough;
	case IMX8MQ:
	case IMX8MM:
	case IMX8MQ_EP:
	case IMX8MM_EP:
		clk_disable_unprepare(imx6_pcie->pcie_aux);
		break;
	case IMX8QM:
	case IMX8QM_EP:
		if (imx6_pcie->controller_id == 1) {
			clk_disable_unprepare(imx6_pcie->pciex2_per);
			clk_disable_unprepare(imx6_pcie->pcie_phy_pclk);
		}
		fallthrough;
	case IMX8QXP:
	case IMX8QXP_EP:
		clk_disable_unprepare(imx6_pcie->pcie_per);
		clk_disable_unprepare(imx6_pcie->pcie_inbound_axi);
		clk_disable_unprepare(imx6_pcie->phy_per);
		clk_disable_unprepare(imx6_pcie->misc_per);
		break;
	default:
		break;
	}
}

static void imx6_pcie_assert_core_reset(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	int i;
	struct device *dev = imx6_pcie->pci->dev;

	switch (imx6_pcie->drvdata->variant) {
	case IMX7D:
	case IMX7D_EP:
	case IMX8MQ:
	case IMX8MM:
	case IMX8MQ_EP:
	case IMX8MM_EP:
		reset_control_assert(imx6_pcie->pciephy_reset);
		fallthrough;
	case IMX8MP:
	case IMX8MP_EP:
		imx6_pcie_ltssm_disable(dev);
		reset_control_assert(imx6_pcie->pciephy_reset);
		reset_control_assert(imx6_pcie->pciephy_perst);
		break;
	case IMX6SX:
	case IMX6SX_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN,
				   IMX6SX_GPR12_PCIE_TEST_POWERDOWN);
		/* Force PCIe PHY reset */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET);
		break;
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST,
				   IMX6Q_GPR1_PCIE_SW_RST);
		break;
	case IMX6Q:
	case IMX6Q_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD, 1 << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_REF_CLK_EN, 0 << 16);
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
		imx6_pcie_clk_enable(imx6_pcie);
		/*
		 * Set the over ride low and enabled
		 * make sure that REF_CLK is turned on.
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
			IMX8QM_CSR_MISC_OFFSET,
			IMX8QM_MISC_CLKREQ_1,
			0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
			IMX8QM_CSR_MISC_OFFSET,
			IMX8QM_MISC_CLKREQ_OVERRIDE_EN_1,
			IMX8QM_MISC_CLKREQ_OVERRIDE_EN_1);
		val = IMX8QM_CSR_PCIEB_OFFSET;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_BUTTON_RST_N,
				IMX8QM_CTRL_BUTTON_RST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PERST_N,
				IMX8QM_CTRL_PERST_N);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_POWER_UP_RST_N,
				IMX8QM_CTRL_POWER_UP_RST_N);
		break;
	case IMX8QM:
	case IMX8QM_EP:
		imx6_pcie_clk_enable(imx6_pcie);
		/*
		 * Set the over ride low and enabled
		 * make sure that REF_CLK is turned on.
		 */
		if (imx6_pcie->controller_id) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_CLKREQ_1,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_CLKREQ_OVERRIDE_EN_1,
				IMX8QM_MISC_CLKREQ_OVERRIDE_EN_1);
		} else {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_CLKREQ_0,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_CLKREQ_OVERRIDE_EN_0,
				IMX8QM_MISC_CLKREQ_OVERRIDE_EN_0);
		}
		for (i = 0; i <= imx6_pcie->controller_id; i++) {
			val = IMX8QM_CSR_PCIEA_OFFSET + i * SZ_64K;
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_BUTTON_RST_N,
					IMX8QM_CTRL_BUTTON_RST_N);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_PERST_N,
					IMX8QM_CTRL_PERST_N);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
					IMX8QM_CTRL_POWER_UP_RST_N,
					IMX8QM_CTRL_POWER_UP_RST_N);
		}
		break;
	}

	if (imx6_pcie->vpcie && regulator_is_enabled(imx6_pcie->vpcie) > 0) {
		int ret = regulator_disable(imx6_pcie->vpcie);

		if (ret)
			dev_err(dev, "failed to disable vpcie regulator: %d\n",
				ret);
	}
}

static void imx6_pcie_set_l1_latency(struct imx6_pcie *imx6_pcie)
{
	u32 val;
	struct dw_pcie *pci = imx6_pcie->pci;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8MQ:
	case IMX8MM:
	case IMX8MP:
		/*
		 * Configure the L1 latency of rc to less than 64us
		 * Otherwise, the L1/L1SUB wouldn't be enable by ASPM.
		 */
		dw_pcie_dbi_ro_wr_en(pci);
		val = readl(pci->dbi_base + SZ_1M +
				IMX8MQ_PCIE_LINK_CAP_REG_OFFSET);
		val &= ~PCI_EXP_LNKCAP_L1EL;
		val |= IMX8MQ_PCIE_LINK_CAP_L1EL_64US;
		writel(val, pci->dbi_base + SZ_1M +
				IMX8MQ_PCIE_LINK_CAP_REG_OFFSET);
		dw_pcie_dbi_ro_wr_dis(pci);
		break;
	default:
		break;
	}
}

static void imx6_pcie_deassert_core_reset(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	int ret, i;
	u32 val, tmp;

	if (imx6_pcie->vpcie && !regulator_is_enabled(imx6_pcie->vpcie)) {
		ret = regulator_enable(imx6_pcie->vpcie);
		if (ret) {
			dev_err(dev, "failed to enable vpcie regulator: %d\n",
				ret);
			return;
		}
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
	case IMX8MP:
	case IMX8MP_EP:
		/* ClKs had been enabled */
		break;
	default:
		imx6_pcie_clk_enable(imx6_pcie);
		break;
	}

	/* Some boards don't have PCIe reset GPIO. */
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					imx6_pcie->gpio_active_high);
		msleep(20);
		gpio_set_value_cansleep(imx6_pcie->reset_gpio,
					!imx6_pcie->gpio_active_high);
	}

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QM:
	case IMX8QM_EP:
		if (imx6_pcie->controller_id)
			/* Set the APB clock masks */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_PHYX2_LPCG_OFFSET,
				IMX8QM_PHYX2_LPCG_PCLK0_MASK |
				IMX8QM_PHYX2_LPCG_PCLK1_MASK,
				IMX8QM_PHYX2_LPCG_PCLK0_MASK |
				IMX8QM_PHYX2_LPCG_PCLK1_MASK);
		fallthrough;
	case IMX8QXP:
	case IMX8QXP_EP:
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		/* bit19 PM_REQ_CORE_RST of pciex#_stts0 should be cleared. */
		for (i = 0; i < 2000; i++) {
			regmap_read(imx6_pcie->iomuxc_gpr,
					val + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&tmp);
			if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) == 0)
				break;
			udelay(10);
		}

		if ((tmp & IMX8QM_CTRL_STTS0_PM_REQ_CORE_RST) != 0)
			dev_err(dev, "ERROR PM_REQ_CORE_RST is still set.\n");

		/* wait for phy pll lock firstly. */
		imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	case IMX8MQ:
	case IMX8MM:
	case IMX8MQ_EP:
	case IMX8MM_EP:
		reset_control_deassert(imx6_pcie->pciephy_reset);

		imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);

		if (imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_L1SS)
			/*
			 * Configure the CLK_REQ# high, let the L1SS
			 * automatically controlled by HW later.
			 */
			reset_control_deassert(imx6_pcie->clkreq_reset);
		imx6_pcie_set_l1_latency(imx6_pcie);
		break;
	case IMX8MP:
	case IMX8MP_EP:
		msleep(20);
		reset_control_deassert(imx6_pcie->pciephy_reset);
		reset_control_deassert(imx6_pcie->pciephy_perst);

		/* release pcie_phy_apb_reset and pcie_phy_init_resetn */
		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG0);
		val |= IMX8MP_GPR_REG0_PHY_APB_RST;
		val |= IMX8MP_GPR_REG0_PHY_INIT_RST;
		writel(val, imx6_pcie->hsmix_base + IMX8MP_GPR_REG0);

		val = imx6_pcie_grp_offset(imx6_pcie);
		if (imx6_pcie->ext_osc) {
			/*TODO Configure the external OSC as REF clock */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_REF_SEL_MASK,
					   IMX8MP_GPR_PCIE_REF_SEL_MASK);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_AUX_EN,
					   IMX8MP_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_PWR_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_CMN_RSTN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_REF_SEL_MASK,
					   IMX8MP_GPR_PCIE_REF_EXT_OSC);
		} else {
			/* Configure the internal PLL as REF clock */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_REF_SEL_MASK,
					   IMX8MP_GPR_PCIE_REF_PLL_SYS);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_AUX_EN,
					   IMX8MP_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_PWR_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
					   IMX8MP_GPR_PCIE_CMN_RSTN, 0);
		}

		phy_calibrate(imx6_pcie->phy);
		/*
		 * GPR_PCIE_PHY_CTRL_BUS[3:0]
		 * 0:i_ssc_en 1:i_power_off
		 * 2:i_cmn_rstn 3:aux_en_glue.ctrl_bus
		 */
		val = imx6_pcie_grp_offset(imx6_pcie);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, val,
				   IMX8MP_GPR_PCIE_CMN_RSTN,
				   IMX8MP_GPR_PCIE_CMN_RSTN);

		imx8_pcie_wait_for_phy_pll_lock(imx6_pcie);

		if (imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_L1SS)
			/*
			 * Configure the CLK_REQ# high, let the L1SS
			 * automatically controlled by HW later.
			 */
			reset_control_deassert(imx6_pcie->clkreq_reset);
		imx6_pcie_set_l1_latency(imx6_pcie);
		break;
	case IMX7D:
	case IMX7D_EP:
		reset_control_deassert(imx6_pcie->pciephy_reset);

		/* Workaround for ERR010728, failure of PCI-e PLL VCO to
		 * oscillate, especially when cold.  This turns off "Duty-cycle
		 * Corrector" and other mysterious undocumented things.
		 */
		if (likely(imx6_pcie->phy_base)) {
			/* De-assert DCC_FB_EN */
			writel(PCIE_PHY_CMN_REG4_DCC_FB_EN,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG4);
			/* Assert RX_EQS and RX_EQS_SEL */
			writel(PCIE_PHY_CMN_REG24_RX_EQ_SEL
				| PCIE_PHY_CMN_REG24_RX_EQ,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG24);
			/* Assert ATT_MODE */
			writel(PCIE_PHY_CMN_REG26_ATT_MODE,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG26);
		} else {
			dev_warn(dev, "Unable to apply ERR010728 workaround. DT missing fsl,imx7d-pcie-phy phandle ?\n");
		}

		imx7d_pcie_wait_for_phy_pll_lock(imx6_pcie);
		break;
	case IMX6SX:
	case IMX6SX_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR5,
				   IMX6SX_GPR5_PCIE_BTNRST_RESET, 0);
		break;
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_SW_RST, 0);

		usleep_range(200, 500);
		break;
	case IMX6Q:		/* Nothing to do */
	case IMX6Q_EP:
		break;
	}

	return;
}

static void imx6_pcie_configure_type(struct imx6_pcie *imx6_pcie)
{
	unsigned int addr, mask, val, mode;
	unsigned int variant = imx6_pcie->drvdata->variant;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	mode = imx6_pcie->drvdata->mode;
	switch (mode) {
	case DW_PCIE_RC_TYPE:
		mode = PCI_EXP_TYPE_ROOT_PORT;
		break;
	case DW_PCIE_EP_TYPE:
		mode = PCI_EXP_TYPE_ENDPOINT;
		break;
	default:
		dev_err(dev, "INVALID device type %d\n", mode);
	}

	switch (variant) {
	case IMX8QM:
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM_EP:
		if (imx6_pcie->controller_id)
			addr = IMX8QM_CSR_PCIEB_OFFSET;
		else
			addr = IMX8QM_CSR_PCIEA_OFFSET;
		mask = IMX8QM_PCIE_TYPE_MASK;
		val = FIELD_PREP(IMX8QM_PCIE_TYPE_MASK, mode);
		break;
	case IMX8MQ:
	case IMX8MQ_EP:
		if (imx6_pcie->controller_id == 1) {
			addr = IOMUXC_GPR12;
			mask = IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE;
			val = FIELD_PREP(IMX8MQ_GPR12_PCIE2_CTRL_DEVICE_TYPE, mode);
		} else {
			addr = IOMUXC_GPR12;
			mask = IMX6Q_GPR12_DEVICE_TYPE;
			val = FIELD_PREP(IMX6Q_GPR12_DEVICE_TYPE, mode);
		}
		break;
	default:
		addr = IOMUXC_GPR12;
		mask = IMX6Q_GPR12_DEVICE_TYPE;
		val = FIELD_PREP(IMX6Q_GPR12_DEVICE_TYPE, mode);
		break;
	}
	regmap_update_bits(imx6_pcie->iomuxc_gpr, addr, mask, val);
}

static void imx6_pcie_init_phy(struct imx6_pcie *imx6_pcie)
{
	int i;
	unsigned int offset, val;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		if (imx6_pcie->hsio_cfg == PCIEAX2SATA) {
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 1.
			 * bit13 phy_ab_select 0.
			 */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				0);
		} else if (imx6_pcie->hsio_cfg == PCIEAX1PCIEBX1SATA) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_PHYX2_OFFSET,
				IMX8QM_PHYX2_CTRL0_APB_MASK,
				IMX8QM_PHY_APB_RSTN_0
				| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				IMX8QM_MISC_PHYX1_EPCS_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
		} else if (imx6_pcie->hsio_cfg == PCIEAX2PCIEBX1) {
			/*
			 * bit 0 rx ena 1.
			 * bit12 PHY_X1_EPCS_SEL 0.
			 * bit13 phy_ab_select 1.
			 */
			if (imx6_pcie->controller_id)
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX1_OFFSET,
					IMX8QM_PHY_APB_RSTN_0,
					IMX8QM_PHY_APB_RSTN_0);
			else
				regmap_update_bits(imx6_pcie->iomuxc_gpr,
					IMX8QM_CSR_PHYX2_OFFSET,
					IMX8QM_PHYX2_CTRL0_APB_MASK,
					IMX8QM_PHY_APB_RSTN_0
					| IMX8QM_PHY_APB_RSTN_1);

			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PHYX1_EPCS_SEL,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_PCIE_AB_SELECT,
				IMX8QM_MISC_PCIE_AB_SELECT);
		}

		if (imx6_pcie->ext_osc) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				IMX8QM_MISC_IOB_RXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				0);
		} else {
			/* Try to used the internal pll as ref clk */
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_RXENA,
				0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_MISC_IOB_TXENA,
				IMX8QM_MISC_IOB_TXENA);
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
				IMX8QM_CSR_MISC_OFFSET,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_MASK,
				IMX8QM_CSR_MISC_IOB_A_0_TXOE
				| IMX8QM_CSR_MISC_IOB_A_0_M1M0_2);
		}

		break;
	case IMX8MM:
	case IMX8MM_EP:
		offset = imx6_pcie_grp_offset(imx6_pcie);

		dev_info(imx6_pcie->pci->dev, "%s REF_CLK is used!.\n",
			 imx6_pcie->ext_osc ? "EXT" : "PLL");
		if (imx6_pcie->ext_osc) {
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_AUX_EN,
					   IMX8MM_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_POWER_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_EXT);
			udelay(100);
			/* Do the PHY common block reset */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_CMN_RST,
					   IMX8MM_GPR_PCIE_CMN_RST);
			udelay(200);
		} else {
			/* Configure the internal PLL as REF clock */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MQ_GPR_PCIE_REF_USE_PAD, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_AUX_EN,
					   IMX8MM_GPR_PCIE_AUX_EN);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_POWER_OFF, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_SSC_EN, 0);
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_REF_CLK_SEL,
					   IMX8MM_GPR_PCIE_REF_CLK_PLL);
			udelay(100);
			/* Configure the PHY */
			writel(PCIE_PHY_CMN_REG62_PLL_CLK_OUT,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG62);
			writel(PCIE_PHY_CMN_REG64_AUX_RX_TX_TERM,
			       imx6_pcie->phy_base + PCIE_PHY_CMN_REG64);
			/* Do the PHY common block reset */
			regmap_update_bits(imx6_pcie->iomuxc_gpr, offset,
					   IMX8MM_GPR_PCIE_CMN_RST,
					   IMX8MM_GPR_PCIE_CMN_RST);
			udelay(200);
		}

		/*
		 * In order to pass the compliance tests.
		 * Configure the TRSV regiser of iMX8MM PCIe PHY.
		 */
		writel(PCIE_PHY_TRSV_REG5_GEN1_DEEMP,
		       imx6_pcie->phy_base + PCIE_PHY_TRSV_REG5);
		writel(PCIE_PHY_TRSV_REG6_GEN2_DEEMP,
		       imx6_pcie->phy_base + PCIE_PHY_TRSV_REG6);

		break;
	case IMX8MQ:
	case IMX8MQ_EP:
		/*
		 * TODO: Currently this code assumes external
		 * oscillator is being used
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				   imx6_pcie_grp_offset(imx6_pcie),
				   IMX8MQ_GPR_PCIE_REF_USE_PAD,
				   IMX8MQ_GPR_PCIE_REF_USE_PAD);
		/*
		 * Regarding the datasheet, the PCIE_VPH is suggested
		 * to be 1.8V. If the PCIE_VPH is supplied by 3.3V, the
		 * VREG_BYPASS should be cleared to zero.
		 */
		if (imx6_pcie->vph &&
		    regulator_get_voltage(imx6_pcie->vph) > 3000000)
			regmap_update_bits(imx6_pcie->iomuxc_gpr,
					   imx6_pcie_grp_offset(imx6_pcie),
					   IMX8MQ_GPR_PCIE_VREG_BYPASS,
					   0);
		break;
	case IMX8MP:
	case IMX8MP_EP:
		phy_power_on(imx6_pcie->phy);
		dev_info(imx6_pcie->pci->dev, "%s REF_CLK is used!.\n",
			 imx6_pcie->ext_osc ? "EXT" : "PLL");
		imx6_pcie_clk_enable(imx6_pcie);

		/* Set P=12,M=800,S=4 and must set ICP=2'b01. */
		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG2);
		val &= ~IMX8MP_GPR_REG2_P_PLL_MASK;
		val |= IMX8MP_GPR_REG2_P_PLL;
		val &= ~IMX8MP_GPR_REG2_M_PLL_MASK;
		val |= IMX8MP_GPR_REG2_M_PLL;
		val &= ~IMX8MP_GPR_REG2_S_PLL_MASK;
		val |= IMX8MP_GPR_REG2_S_PLL;
		writel(val, imx6_pcie->hsmix_base + IMX8MP_GPR_REG2);
		/* wait greater than 1/F_FREF =1/2MHZ=0.5us */
		udelay(1);

		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG3);
		val |= IMX8MP_GPR_REG3_PLL_RST;
		writel(val, imx6_pcie->hsmix_base + IMX8MP_GPR_REG3);
		udelay(10);

		/* Set 1 to pll_cke of GPR_REG3 */
		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG3);
		val |= IMX8MP_GPR_REG3_PLL_CKE;
		writel(val, imx6_pcie->hsmix_base + IMX8MP_GPR_REG3);

		/* Lock time should be greater than 300cycle=300*0.5us=150us */
		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG1);
		for (i = 0; i < 100; i++) {
			val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG1);
			if (val & IMX8MP_GPR_REG1_PLL_LOCK)
				break;
			udelay(10);
		}
		if (i >= 100)
			dev_err(imx6_pcie->pci->dev,
				"PCIe PHY PLL clock is not locked.\n");
		else
			dev_info(imx6_pcie->pci->dev,
				"PCIe PHY PLL clock is locked.\n");

		/* pcie_clock_module_en */
		val = readl(imx6_pcie->hsmix_base + IMX8MP_GPR_REG0);
		val |= IMX8MP_GPR_REG0_CLK_MOD_EN;
		writel(val, imx6_pcie->hsmix_base + IMX8MP_GPR_REG0);
		break;
	case IMX7D:
	case IMX7D_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX7D_GPR12_PCIE_PHY_REFCLK_SEL, imx6_pcie->ext_osc ? 0 : BIT(5));
		break;
	case IMX6SX:
	case IMX6SX_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_RX_EQ_MASK,
				   IMX6SX_GPR12_PCIE_RX_EQ_2);
		fallthrough;
	default:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0 << 10);

		/* configure constant input signal to the pcie ctrl and phy */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_LOS_LEVEL, 9 << 4);

		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN1,
				   imx6_pcie->tx_deemph_gen1 << 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_3P5DB,
				   imx6_pcie->tx_deemph_gen2_3p5db << 6);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_DEEMPH_GEN2_6DB,
				   imx6_pcie->tx_deemph_gen2_6db << 12);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_FULL,
				   imx6_pcie->tx_swing_full << 18);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR8,
				   IMX6Q_GPR8_TX_SWING_LOW,
				   imx6_pcie->tx_swing_low << 25);
		break;
	}

	imx6_pcie_configure_type(imx6_pcie);
}

static int imx6_setup_phy_mpll(struct imx6_pcie *imx6_pcie)
{
	unsigned long phy_rate = clk_get_rate(imx6_pcie->pcie_phy);
	int mult, div;
	u16 val;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_PHY))
		return 0;

	switch (phy_rate) {
	case 125000000:
		/*
		 * The default settings of the MPLL are for a 125MHz input
		 * clock, so no need to reconfigure anything in that case.
		 */
		return 0;
	case 100000000:
		mult = 25;
		div = 0;
		break;
	case 200000000:
		mult = 25;
		div = 1;
		break;
	default:
		dev_err(imx6_pcie->pci->dev,
			"Unsupported PHY reference clock rate %lu\n", phy_rate);
		return -EINVAL;
	}

	pcie_phy_read(imx6_pcie, PCIE_PHY_MPLL_OVRD_IN_LO, &val);
	val &= ~(PCIE_PHY_MPLL_MULTIPLIER_MASK <<
		 PCIE_PHY_MPLL_MULTIPLIER_SHIFT);
	val |= mult << PCIE_PHY_MPLL_MULTIPLIER_SHIFT;
	val |= PCIE_PHY_MPLL_MULTIPLIER_OVRD;
	pcie_phy_write(imx6_pcie, PCIE_PHY_MPLL_OVRD_IN_LO, val);

	pcie_phy_read(imx6_pcie, PCIE_PHY_ATEOVRD, &val);
	val &= ~(PCIE_PHY_ATEOVRD_REF_CLKDIV_MASK <<
		 PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT);
	val |= div << PCIE_PHY_ATEOVRD_REF_CLKDIV_SHIFT;
	val |= PCIE_PHY_ATEOVRD_EN;
	pcie_phy_write(imx6_pcie, PCIE_PHY_ATEOVRD, val);

	return 0;
}

static int imx6_pcie_wait_for_speed_change(struct imx6_pcie *imx6_pcie)
{
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;
	u32 tmp;
	unsigned int retries;

	for (retries = 0; retries < 200; retries++) {
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		/* Test if the speed change finished. */
		if (!(tmp & PORT_LOGIC_SPEED_CHANGE))
			return 0;
		usleep_range(100, 1000);
	}

	dev_err(dev, "Speed change timeout\n");
	return -ETIMEDOUT;
}

static void imx6_pcie_ltssm_enable(struct device *dev)
{
	u32 val;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6Q:
	case IMX6Q_EP:
	case IMX6SX:
	case IMX6SX_EP:
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2,
				   IMX6Q_GPR12_PCIE_CTL_2);
		break;
	case IMX7D:
	case IMX7D_EP:
	case IMX8MQ:
	case IMX8MM:
	case IMX8MP:
	case IMX8MQ_EP:
	case IMX8MM_EP:
	case IMX8MP_EP:
		reset_control_deassert(imx6_pcie->apps_reset);
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE,
				IMX8QM_CTRL_LTSSM_ENABLE);
		break;
	}
}

static int imx6_pcie_start_link(struct dw_pcie *pci)
{
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);
	struct device *dev = pci->dev;
	u8 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_EXP);
	u32 tmp;
	int ret;

	if (dw_pcie_link_up(pci)) {
		dev_dbg(dev, "link is already up\n");
		return 0;
	}

	dw_pcie_dbi_ro_wr_en(pci);
	/*
	 * Force Gen1 operation when starting the link.  In case the link is
	 * started in Gen2 mode, there is a possibility the devices on the
	 * bus will not be detected at all.  This happens with PCIe switches.
	 */
	if (!imx6_pcie_cz_enabled) {
		tmp = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
		tmp &= ~PCI_EXP_LNKCAP_SLS;
		tmp |= PCI_EXP_LNKCAP_SLS_2_5GB;
		dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, tmp);
	}

	/* Start LTSSM. */
	imx6_pcie_ltssm_enable(dev);

	ret = dw_pcie_wait_for_link(pci);
	if (ret)
		goto err_reset_phy;

	if (pci->link_gen >= 2) {
		/* Fill up target link speed before speed change. */
		tmp = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCTL2);
		tmp &= ~PCI_EXP_LNKCAP_SLS;
		tmp |= pci->link_gen;
		dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCTL2, tmp);

		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp &= ~PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		/* Allow Gen2 mode after the link is up. */
		tmp = dw_pcie_readl_dbi(pci, offset + PCI_EXP_LNKCAP);
		tmp &= ~PCI_EXP_LNKCAP_SLS;
		tmp |= pci->link_gen;
		dw_pcie_writel_dbi(pci, offset + PCI_EXP_LNKCAP, tmp);

		/*
		 * Start Directed Speed Change so the best possible
		 * speed both link partners support can be negotiated.
		 */
		tmp = dw_pcie_readl_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL);
		tmp |= PORT_LOGIC_SPEED_CHANGE;
		dw_pcie_writel_dbi(pci, PCIE_LINK_WIDTH_SPEED_CONTROL, tmp);

		if (imx6_pcie->drvdata->flags &
		    IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE) {
			/*
			 * On i.MX7, DIRECT_SPEED_CHANGE behaves differently
			 * from i.MX6 family when no link speed transition
			 * occurs and we go Gen1 -> yep, Gen1. The difference
			 * is that, in such case, it will not be cleared by HW
			 * which will cause the following code to report false
			 * failure.
			 */

			ret = imx6_pcie_wait_for_speed_change(imx6_pcie);
			if (ret) {
				dev_err(dev, "Failed to bring link up!\n");
				goto err_reset_phy;
			}
		}

		/* Make sure link training is finished as well! */
		ret = dw_pcie_wait_for_link(pci);
		if (ret) {
			dev_err(dev, "Failed to bring link up!\n");
			goto err_reset_phy;
		}
	} else {
		dev_info(dev, "Link: Gen2 disabled\n");
	}

	dw_pcie_dbi_ro_wr_dis(pci);
	tmp = dw_pcie_readw_dbi(pci, offset + PCI_EXP_LNKSTA);
	dev_info(dev, "Link up, Gen%i\n", tmp & PCI_EXP_LNKSTA_CLS);
	msleep(100);
	return 0;

err_reset_phy:
	dw_pcie_dbi_ro_wr_dis(pci);
	dev_dbg(dev, "PHY DEBUG_R0=0x%08x DEBUG_R1=0x%08x\n",
		dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG0),
		dw_pcie_readl_dbi(pci, PCIE_PORT_DEBUG1));
	imx6_pcie_reset_phy(imx6_pcie);
	if (!imx6_pcie_cz_enabled) {
		imx6_pcie_clk_disable(imx6_pcie);
		if (imx6_pcie->vpcie != NULL)
			regulator_disable(imx6_pcie->vpcie);
		if (imx6_pcie->epdev_on != NULL)
			regulator_disable(imx6_pcie->epdev_on);
		if (gpio_is_valid(imx6_pcie->dis_gpio))
			gpio_set_value_cansleep(imx6_pcie->dis_gpio, 0);
	}

	return ret;
}

static void pci_imx_set_msi_en(struct pcie_port *pp)
{
	u16 val;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);

	if (pci_msi_enabled()) {
		u8 offset = dw_pcie_find_capability(pci, PCI_CAP_ID_MSI);
		dw_pcie_dbi_ro_wr_en(pci);
		val = dw_pcie_readw_dbi(pci, offset + PCI_MSI_FLAGS);
		val |= PCI_MSI_FLAGS_ENABLE;
		val &= ~PCI_MSI_FLAGS_64BIT;
		dw_pcie_writew_dbi(pci, offset + PCI_MSI_FLAGS, val);
		dw_pcie_dbi_ro_wr_dis(pci);
	}
}

static void imx6_pcie_stop_link(struct dw_pcie *pci)
{
	struct device *dev = pci->dev;

	/* turn off pcie ltssm */
	imx6_pcie_ltssm_disable(dev);
}

static u64 imx6_pcie_cpu_addr_fixup(struct dw_pcie *pcie, u64 cpu_addr)
{
	unsigned int offset;
	struct dw_pcie_ep *ep = &pcie->ep;
	struct pcie_port *pp = &pcie->pp;
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pcie);
	struct resource_entry *entry;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP))
		return cpu_addr;

	if (imx6_pcie->drvdata->mode == DW_PCIE_RC_TYPE) {
		entry = resource_list_first_type(&pp->bridge->windows,
						 IORESOURCE_MEM);
		offset = entry->res->start;
	} else {
		offset = ep->phys_base;
	}

	return (cpu_addr + imx6_pcie->local_addr - offset);
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = imx6_pcie_start_link,
	.stop_link = imx6_pcie_stop_link,
	.cpu_addr_fixup = imx6_pcie_cpu_addr_fixup,
};

static void imx_pcie_ep_init(struct dw_pcie_ep *ep)
{
	enum pci_barno bar;
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	for (bar = BAR_0; bar <= BAR_5; bar++)
		dw_pcie_ep_reset_bar(pci, bar);
}

static int imx_pcie_ep_raise_irq(struct dw_pcie_ep *ep, u8 func_no,
				   enum pci_epc_irq_type type,
				   u16 interrupt_num)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);

	switch (type) {
	case PCI_EPC_IRQ_LEGACY:
		return dw_pcie_ep_raise_legacy_irq(ep, func_no);
	case PCI_EPC_IRQ_MSI:
		return dw_pcie_ep_raise_msi_irq(ep, func_no, interrupt_num);
	case PCI_EPC_IRQ_MSIX:
		return dw_pcie_ep_raise_msix_irq(ep, func_no, interrupt_num);
	default:
		dev_err(pci->dev, "UNKNOWN IRQ type\n");
		return -EINVAL;
	}

	return 0;
}

/*
 * iMX8QM/iMXQXP: Bar1/3/5 are reserved.
 */
static const struct pci_epc_features imx8q_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3 | 1 << BAR_5,
};

static const struct pci_epc_features imx8m_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	.reserved_bar = 1 << BAR_1 | 1 << BAR_3,
	.align = SZ_64K,
};

static const struct pci_epc_features imx6q_pcie_epc_features = {
	.linkup_notifier = false,
	.msi_capable = true,
	.msix_capable = false,
	.reserved_bar = 1 << BAR_0 | 1 << BAR_1 | 1 << BAR_2,
	.align = SZ_64K,
};

static const struct pci_epc_features*
imx_pcie_ep_get_features(struct dw_pcie_ep *ep)
{
	struct dw_pcie *pci = to_dw_pcie_from_ep(ep);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	switch (imx6_pcie->drvdata->variant) {
	case IMX8QM_EP:
	case IMX8QXP_EP:
		return &imx8q_pcie_epc_features;
	case IMX8MQ_EP:
	case IMX8MM_EP:
	case IMX8MP_EP:
	case IMX7D_EP:
	case IMX6SX_EP:
		return &imx8m_pcie_epc_features;
	default:
		return &imx6q_pcie_epc_features;
	}
}

static const struct dw_pcie_ep_ops pcie_ep_ops = {
	.ep_init = imx_pcie_ep_init,
	.raise_irq = imx_pcie_ep_raise_irq,
	.get_features = imx_pcie_ep_get_features,
};

static int imx_add_pcie_ep(struct imx6_pcie *imx6_pcie,
					struct platform_device *pdev)
{
	int ret;
	unsigned int pcie_dbi2_offset;
	struct dw_pcie_ep *ep;
	struct resource *res;
	struct dw_pcie *pci = imx6_pcie->pci;
	struct device *dev = pci->dev;

	ep = &pci->ep;
	ep->ops = &pcie_ep_ops;

	switch (imx6_pcie->drvdata->variant) {
	case IMX8MQ_EP:
	case IMX8MM_EP:
	case IMX8MP_EP:
		pcie_dbi2_offset = SZ_1M;
		break;
	default:
		pcie_dbi2_offset = SZ_4K;
		break;
	}
	pci->dbi_base2 = pci->dbi_base + pcie_dbi2_offset;
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "addr_space");
	if (!res)
		return -EINVAL;

	ep->phys_base = res->start;
	ep->addr_size = resource_size(res);
	ep->page_size = SZ_64K;

	ret = dw_pcie_ep_init(ep);
	if (ret) {
		dev_err(dev, "failed to initialize endpoint\n");
		return ret;
	}
	/* Start LTSSM. */
	imx6_pcie_ltssm_enable(dev);

	return 0;
}

static void imx6_pcie_ltssm_disable(struct device *dev)
{
	u32 val;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
	case IMX6SX_EP:
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6Q_GPR12_PCIE_CTL_2, 0);
		break;
	case IMX7D:
	case IMX7D_EP:
	case IMX8MQ:
	case IMX8MM:
	case IMX8MP:
	case IMX8MQ_EP:
	case IMX8MM_EP:
	case IMX8MP_EP:
		reset_control_assert(imx6_pcie->apps_reset);
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		/* Bit4 of the CTRL2 */
		val = IMX8QM_CSR_PCIEA_OFFSET
			+ imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_LTSSM_ENABLE, 0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				val + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23, 0);
		break;
	default:
		dev_err(dev, "ltssm_disable not supported\n");
	}
}

static ssize_t bus_freq_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	u32 bus_freq;

	ret = sscanf(buf, "%x\n", &bus_freq);
	if (ret != 1)
		return -EINVAL;
	if (bus_freq) {
		dev_info(dev, "pcie request bus freq high.\n");
		request_bus_freq(BUS_FREQ_HIGH);
	} else {
		dev_info(dev, "pcie release bus freq high.\n");
		release_bus_freq(BUS_FREQ_HIGH);
	}

	return count;
}
static DEVICE_ATTR_WO(bus_freq);

static struct attribute *imx_pcie_rc_attrs[] = {
	&dev_attr_bus_freq.attr,
	NULL
};

static struct attribute_group imx_pcie_attrgroup = {
	.attrs	= imx_pcie_rc_attrs,
};

static void imx6_pcie_clkreq_enable(struct imx6_pcie *imx6_pcie)
{
	/*
	 * If the L1SS is supported, disable the over ride after link up.
	 * Let the the CLK_REQ# controlled by HW L1SS automatically.
	 */
	switch (imx6_pcie->drvdata->variant) {
	case IMX8MQ:
	case IMX8MM:
	case IMX8MP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
			imx6_pcie_grp_offset(imx6_pcie),
			IMX8MQ_GPR_PCIE_CLK_REQ_OVERRIDE_EN,
			0);
		break;
	default:
		break;
	};
}

#ifdef CONFIG_PM_SLEEP
static void imx6_pcie_pm_turnoff(struct imx6_pcie *imx6_pcie)
{
	int i;
	u32 dst, val;
	struct device *dev = imx6_pcie->pci->dev;

	/* Some variants have a turnoff reset in DT */
	if (imx6_pcie->turnoff_reset) {
		reset_control_assert(imx6_pcie->turnoff_reset);
		reset_control_deassert(imx6_pcie->turnoff_reset);
		goto pm_turnoff_sleep;
	}

	/* Others poke directly at IOMUXC registers */
	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
	case IMX6SX_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX6QP:
	case IMX6QP_EP:
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR12,
				   IMX6SX_GPR12_PCIE_PM_TURN_OFF, 0);
		break;
	case IMX8QXP:
	case IMX8QXP_EP:
	case IMX8QM:
	case IMX8QM_EP:
		dst = IMX8QM_CSR_PCIEA_OFFSET + imx6_pcie->controller_id * SZ_64K;
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				IMX8QM_CTRL_PM_XMT_TURNOFF);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_PM_XMT_TURNOFF,
				0);
		regmap_update_bits(imx6_pcie->iomuxc_gpr,
				dst + IMX8QM_CSR_PCIE_CTRL2_OFFSET,
				IMX8QM_CTRL_READY_ENTR_L23,
				IMX8QM_CTRL_READY_ENTR_L23);
		/* check the L2 is entered or not. */
		for (i = 0; i < 10000; i++) {
			regmap_read(imx6_pcie->iomuxc_gpr,
					dst + IMX8QM_CSR_PCIE_STTS0_OFFSET,
					&val);
			if (val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2)
				break;
			udelay(10);
		}
		if ((val & IMX8QM_CTRL_STTS0_PM_LINKST_IN_L2) == 0)
			dev_err(dev, "PCIE%d can't enter into L2.\n",
					imx6_pcie->controller_id);
		break;
	default:
		dev_err(dev, "PME_Turn_Off not implemented\n");
		return;
	}

	/*
	 * Components with an upstream port must respond to
	 * PME_Turn_Off with PME_TO_Ack but we can't check.
	 *
	 * The standard recommends a 1-10ms timeout after which to
	 * proceed anyway as if acks were received.
	 */
pm_turnoff_sleep:
	usleep_range(1000, 10000);
}

static int imx6_pcie_suspend_noirq(struct device *dev)
{
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_SUSPEND))
		return 0;
	if (unlikely(imx6_pcie->drvdata->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * ERR005723 "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				   IMX6Q_GPR1_PCIE_TEST_PD,
				   IMX6Q_GPR1_PCIE_TEST_PD);
	} else {
		imx6_pcie_pm_turnoff(imx6_pcie);
		imx6_pcie_ltssm_disable(dev);
		imx6_pcie_clk_disable(imx6_pcie);
	}

	return 0;
}

static int imx6_pcie_resume_noirq(struct device *dev)
{
	int ret;
	struct imx6_pcie *imx6_pcie = dev_get_drvdata(dev);
	struct pcie_port *pp = &imx6_pcie->pci->pp;

	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_SUSPEND))
		return 0;
	if (unlikely(imx6_pcie->drvdata->variant == IMX6Q)) {
		/*
		 * L2 can exit by 'reset' or Inband beacon (from remote EP)
		 * toggling phy_powerdown has same effect as 'inband beacon'
		 * So, toggle bit18 of GPR1, used as a workaround of errata
		 * ERR005723 "PCIe PCIe does not support L2 Power Down"
		 */
		regmap_update_bits(imx6_pcie->iomuxc_gpr, IOMUXC_GPR1,
				IMX6Q_GPR1_PCIE_TEST_PD, 0);
	} else {
		imx6_pcie_assert_core_reset(imx6_pcie);
		imx6_pcie_init_phy(imx6_pcie);
		imx6_pcie_deassert_core_reset(imx6_pcie);
		dw_pcie_setup_rc(pp);
		pci_imx_set_msi_en(pp);

		ret = imx6_pcie_start_link(imx6_pcie->pci);
		if (ret < 0)
			dev_info(dev, "pcie link is down after resume.\n");
		if (imx6_pcie->l1ss_clkreq)
			imx6_pcie_clkreq_enable(imx6_pcie);
	}

	return 0;
}
#endif

static const struct dev_pm_ops imx6_pcie_pm_ops = {
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(imx6_pcie_suspend_noirq,
				      imx6_pcie_resume_noirq)
};

static const struct dw_pcie_host_ops imx6_pcie_host_ops = {
};

static int __init imx6_pcie_compliance_test_enable(char *str)
{
	if (!strcmp(str, "yes")) {
		pr_info("Enable the i.MX PCIe compliance tests mode.\n");
		imx6_pcie_cz_enabled = 1;
	}
	return 1;
}

__setup("pcie_cz_enabled=", imx6_pcie_compliance_test_enable);

static int imx6_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dw_pcie *pci;
	struct imx6_pcie *imx6_pcie;
	struct device_node *np;
	struct resource *dbi_base, *hsio_res;
	struct device_node *node = dev->of_node;
	void __iomem *iomem;
	struct regmap_config regconfig = imx6_pcie_regconfig;
	int ret;

	imx6_pcie = devm_kzalloc(dev, sizeof(*imx6_pcie), GFP_KERNEL);
	if (!imx6_pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;
	pci->pp.ops = &imx6_pcie_host_ops;

	imx6_pcie->pci = pci;
	imx6_pcie->drvdata = of_device_get_match_data(dev);

	/* Find the PHY if one is defined, only imx7d uses it */
	np = of_parse_phandle(node, "fsl,imx7d-pcie-phy", 0);
	if (np) {
		struct resource res;

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			dev_err(dev, "Unable to map PCIe PHY\n");
			return ret;
		}
		imx6_pcie->phy_base = devm_ioremap_resource(dev, &res);
		if (IS_ERR(imx6_pcie->phy_base))
			return PTR_ERR(imx6_pcie->phy_base);
	}

	imx6_pcie->phy = devm_phy_get(dev, "pcie-phy");
	if (IS_ERR(imx6_pcie->phy)) {
		if (PTR_ERR(imx6_pcie->phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		/* Set NULL if there is no pcie-phy */
		imx6_pcie->phy = NULL;
	}

	/* Find the HSIO MIX if one is defined, only imx8mp uses it */
	np = of_parse_phandle(node, "fsl,imx8mp-hsio-mix", 0);
	if (np) {
		struct resource res;

		ret = of_address_to_resource(np, 0, &res);
		if (ret) {
			dev_err(dev, "Unable to find HSIO MIX res\n");
			return ret;
		}
		imx6_pcie->hsmix_base = devm_ioremap_resource(dev, &res);
		if (IS_ERR(imx6_pcie->hsmix_base)) {
			dev_err(dev, "Unable to map HSIO MIX res\n");
			return PTR_ERR(imx6_pcie->hsmix_base);
		}
	}

	dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pci->dbi_base = devm_ioremap_resource(dev, dbi_base);
	if (IS_ERR(pci->dbi_base))
		return PTR_ERR(pci->dbi_base);

	if (of_property_read_u32(node, "hsio-cfg", &imx6_pcie->hsio_cfg))
		imx6_pcie->hsio_cfg = 0;
	if (of_property_read_u32(node, "ext_osc", &imx6_pcie->ext_osc) < 0)
		imx6_pcie->ext_osc = 0;
	if (of_property_read_u32(node, "local-addr", &imx6_pcie->local_addr))
		imx6_pcie->local_addr = 0;
	if (of_property_read_bool(node, "l1ss-disabled"))
		imx6_pcie->l1ss_clkreq = 0;
	else
		imx6_pcie->l1ss_clkreq = 1;

	/* Fetch GPIOs */
	imx6_pcie->clkreq_gpio = of_get_named_gpio(node, "clkreq-gpio", 0);
	if (gpio_is_valid(imx6_pcie->clkreq_gpio)) {
		devm_gpio_request_one(&pdev->dev, imx6_pcie->clkreq_gpio,
				      GPIOF_OUT_INIT_LOW, "PCIe CLKREQ");
	} else if (imx6_pcie->clkreq_gpio == -EPROBE_DEFER) {
		return imx6_pcie->clkreq_gpio;
	}

	imx6_pcie->dis_gpio = of_get_named_gpio(node, "disable-gpio", 0);
	if (gpio_is_valid(imx6_pcie->dis_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, imx6_pcie->dis_gpio,
					    GPIOF_OUT_INIT_LOW, "PCIe DIS");
		if (ret) {
			dev_err(&pdev->dev, "unable to get disable gpio\n");
			return ret;
		}
	} else if (imx6_pcie->dis_gpio == -EPROBE_DEFER) {
		return imx6_pcie->dis_gpio;
	}
	imx6_pcie->epdev_on = devm_regulator_get(&pdev->dev, "epdev_on");
	if (IS_ERR(imx6_pcie->epdev_on))
		return -EPROBE_DEFER;
	imx6_pcie->reset_gpio = of_get_named_gpio(node, "reset-gpio", 0);
	imx6_pcie->gpio_active_high = of_property_read_bool(node,
						"reset-gpio-active-high");
	if (gpio_is_valid(imx6_pcie->reset_gpio)) {
		ret = devm_gpio_request_one(dev, imx6_pcie->reset_gpio,
				imx6_pcie->gpio_active_high ?
					GPIOF_OUT_INIT_HIGH :
					GPIOF_OUT_INIT_LOW,
				"PCIe reset");
		if (ret) {
			dev_err(dev, "unable to get reset gpio\n");
			return ret;
		}
	} else if (imx6_pcie->reset_gpio == -EPROBE_DEFER) {
		return imx6_pcie->reset_gpio;
	}

	/* Fetch clocks */
	imx6_pcie->pcie_phy = devm_clk_get(dev, "pcie_phy");
	if (IS_ERR(imx6_pcie->pcie_phy))
		return dev_err_probe(dev, PTR_ERR(imx6_pcie->pcie_phy),
				     "pcie_phy clock source missing or invalid\n");

	imx6_pcie->pcie_bus = devm_clk_get(dev, "pcie_bus");
	if (IS_ERR(imx6_pcie->pcie_bus))
		return dev_err_probe(dev, PTR_ERR(imx6_pcie->pcie_bus),
				     "pcie_bus clock source missing or invalid\n");

	imx6_pcie->pcie = devm_clk_get(dev, "pcie");
	if (IS_ERR(imx6_pcie->pcie))
		return dev_err_probe(dev, PTR_ERR(imx6_pcie->pcie),
				     "pcie clock source missing or invalid\n");

	switch (imx6_pcie->drvdata->variant) {
	case IMX6SX:
	case IMX6SX_EP:
		imx6_pcie->pcie_inbound_axi = devm_clk_get(dev,
							   "pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi))
			return dev_err_probe(dev, PTR_ERR(imx6_pcie->pcie_inbound_axi),
					     "pcie_inbound_axi clock missing or invalid\n");
		break;
	case IMX8MP:
	case IMX8MP_EP:
		imx6_pcie->pciephy_perst = devm_reset_control_get_exclusive(dev,
									    "pciephy_perst");
		if (IS_ERR(imx6_pcie->pciephy_perst)) {
			dev_err(dev, "Failed to get PCIEPHY perst control\n");
			return PTR_ERR(imx6_pcie->pciephy_perst);
		}
		fallthrough;
	case IMX8MQ:
	case IMX8MM:
	case IMX8MQ_EP:
	case IMX8MM_EP:
		imx6_pcie->pcie_aux = devm_clk_get(dev, "pcie_aux");
		if (IS_ERR(imx6_pcie->pcie_aux))
			return dev_err_probe(dev, PTR_ERR(imx6_pcie->pcie_aux),
					     "pcie_aux clock source missing or invalid\n");
		fallthrough;
	case IMX7D:
	case IMX7D_EP:
		if (dbi_base->start == IMX8MQ_PCIE2_BASE_ADDR)
			imx6_pcie->controller_id = 1;

		imx6_pcie->pciephy_reset = devm_reset_control_get_exclusive(dev,
									    "pciephy");
		if (IS_ERR(imx6_pcie->pciephy_reset)) {
			dev_err(dev, "Failed to get PCIEPHY reset control\n");
			return PTR_ERR(imx6_pcie->pciephy_reset);
		}

		imx6_pcie->apps_reset = devm_reset_control_get_exclusive(dev,
									 "apps");
		if (IS_ERR(imx6_pcie->apps_reset)) {
			dev_err(dev, "Failed to get PCIE APPS reset control\n");
			return PTR_ERR(imx6_pcie->apps_reset);
		}
		break;
	case IMX8QM:
	case IMX8QM_EP:
	case IMX8QXP:
	case IMX8QXP_EP:
		if (dbi_base->start == IMX8_HSIO_PCIEB_BASE_ADDR)
			imx6_pcie->controller_id = 1;

		imx6_pcie->pcie_per = devm_clk_get(dev, "pcie_per");
		if (IS_ERR(imx6_pcie->pcie_per)) {
			dev_err(dev, "pcie_per clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_per);
		}

		imx6_pcie->pcie_inbound_axi = devm_clk_get(&pdev->dev,
				"pcie_inbound_axi");
		if (IS_ERR(imx6_pcie->pcie_inbound_axi)) {
			dev_err(&pdev->dev,
				"pcie clock source missing or invalid\n");
			return PTR_ERR(imx6_pcie->pcie_inbound_axi);
		}

		imx6_pcie->phy_per = devm_clk_get(dev, "phy_per");
		if (IS_ERR(imx6_pcie->phy_per)) {
			dev_err(dev, "failed to get phy per clock.\n");
			return PTR_ERR(imx6_pcie->phy_per);
		}

		imx6_pcie->misc_per = devm_clk_get(dev, "misc_per");
		if (IS_ERR(imx6_pcie->misc_per)) {
			dev_err(dev, "failed to get misc per clock.\n");
			return PTR_ERR(imx6_pcie->misc_per);
		}
		if (imx6_pcie->drvdata->variant == IMX8QM
				&& imx6_pcie->controller_id == 1) {
			imx6_pcie->pcie_phy_pclk = devm_clk_get(dev,
					"pcie_phy_pclk");
			if (IS_ERR(imx6_pcie->pcie_phy_pclk)) {
				dev_err(dev, "no pcie_phy_pclk clock\n");
				return PTR_ERR(imx6_pcie->pcie_phy_pclk);
			}

			imx6_pcie->pciex2_per = devm_clk_get(dev, "pciex2_per");
			if (IS_ERR(imx6_pcie->pciex2_per)) {
				dev_err(dev, "can't get pciex2_per.\n");
				return PTR_ERR(imx6_pcie->pciex2_per);
			}
		}

		hsio_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"hsio");
		if (hsio_res) {
			iomem = devm_ioremap(dev, hsio_res->start,
					     resource_size(hsio_res));
			if (IS_ERR(iomem))
				return PTR_ERR(iomem);
			imx6_pcie->iomuxc_gpr =
				devm_regmap_init_mmio(dev, iomem, &regconfig);
			if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
				dev_err(dev, "failed to init register map\n");
				return PTR_ERR(imx6_pcie->iomuxc_gpr);
			}
		} else {
			dev_err(dev, "missing *hsio* reg space\n");
		}
		break;
	default:
		break;
	}

	/* Grab turnoff reset */
	imx6_pcie->turnoff_reset = devm_reset_control_get_optional_exclusive(dev, "turnoff");
	if (IS_ERR(imx6_pcie->turnoff_reset)) {
		dev_err(dev, "Failed to get TURNOFF reset control\n");
		return PTR_ERR(imx6_pcie->turnoff_reset);
	}

	imx6_pcie->clkreq_reset = devm_reset_control_get_optional_exclusive(dev, "clkreq");
	if (IS_ERR(imx6_pcie->clkreq_reset)) {
		dev_err(dev, "Failed to get CLKREQ reset control\n");
		return PTR_ERR(imx6_pcie->clkreq_reset);
	}

	/* Grab GPR config register range */
	if (imx6_pcie->iomuxc_gpr == NULL) {
		imx6_pcie->iomuxc_gpr =
			syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
		if (IS_ERR(imx6_pcie->iomuxc_gpr)) {
			dev_err(dev, "unable to find iomuxc registers\n");
			return PTR_ERR(imx6_pcie->iomuxc_gpr);
		}
	}

	/* Grab PCIe PHY Tx Settings */
	if (of_property_read_u32(node, "fsl,tx-deemph-gen1",
				 &imx6_pcie->tx_deemph_gen1))
		imx6_pcie->tx_deemph_gen1 = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-3p5db",
				 &imx6_pcie->tx_deemph_gen2_3p5db))
		imx6_pcie->tx_deemph_gen2_3p5db = 0;

	if (of_property_read_u32(node, "fsl,tx-deemph-gen2-6db",
				 &imx6_pcie->tx_deemph_gen2_6db))
		imx6_pcie->tx_deemph_gen2_6db = 20;

	if (of_property_read_u32(node, "fsl,tx-swing-full",
				 &imx6_pcie->tx_swing_full))
		imx6_pcie->tx_swing_full = 127;

	if (of_property_read_u32(node, "fsl,tx-swing-low",
				 &imx6_pcie->tx_swing_low))
		imx6_pcie->tx_swing_low = 127;

	/* Limit link speed */
	pci->link_gen = 1;
	ret = of_property_read_u32(node, "fsl,max-link-speed", &pci->link_gen);

	imx6_pcie->vpcie = devm_regulator_get_optional(&pdev->dev, "vpcie");
	if (IS_ERR(imx6_pcie->vpcie)) {
		if (PTR_ERR(imx6_pcie->vpcie) != -ENODEV)
			return PTR_ERR(imx6_pcie->vpcie);
		imx6_pcie->vpcie = NULL;
	}

	imx6_pcie->vph = devm_regulator_get_optional(&pdev->dev, "vph");
	if (IS_ERR(imx6_pcie->vph)) {
		if (PTR_ERR(imx6_pcie->vph) != -ENODEV)
			return PTR_ERR(imx6_pcie->vph);
		imx6_pcie->vph = NULL;
	}

	platform_set_drvdata(pdev, imx6_pcie);

	ret = imx6_pcie_attach_pd(dev);
	if (ret)
		return ret;

	ret = regulator_enable(imx6_pcie->epdev_on);
	if (ret) {
		dev_err(dev, "failed to enable the epdev_on regulator\n");
		goto err_ret;
	}

	if (gpio_is_valid(imx6_pcie->dis_gpio))
		gpio_set_value_cansleep(imx6_pcie->dis_gpio, 1);
	imx6_pcie_assert_core_reset(imx6_pcie);
	imx6_pcie_init_phy(imx6_pcie);
	imx6_pcie_deassert_core_reset(imx6_pcie);
	imx6_setup_phy_mpll(imx6_pcie);

	switch (imx6_pcie->drvdata->mode) {
	case DW_PCIE_RC_TYPE:
		/* add attributes for bus freq */
		ret = sysfs_create_group(&pdev->dev.kobj, &imx_pcie_attrgroup);
		if (ret)
			goto err_ret;

		ret = dw_pcie_host_init(&pci->pp);
		if (ret < 0) {
			if (imx6_pcie_cz_enabled) {
				/* The PCIE clocks wouldn't be turned off */
				dev_info(dev, "To do the compliance tests.\n");
				ret = 0;
			} else {
				imx6_pcie_detach_pd(dev);
				return ret;
			}
		}

		pci_imx_set_msi_en(&imx6_pcie->pci->pp);
		break;
	case DW_PCIE_EP_TYPE:
		if (!IS_ENABLED(CONFIG_PCI_IMX6_EP)) {
			ret = -ENODEV;
			goto err_ret;
		}

		ret = imx_add_pcie_ep(imx6_pcie, pdev);
		if (ret < 0)
			goto err_ret;
		break;
	default:
		dev_err(dev, "INVALID device type.\n");
		goto err_ret;
	}

	return 0;

err_ret:
	imx6_pcie_detach_pd(dev);
	return ret;
}

static void imx6_pcie_shutdown(struct platform_device *pdev)
{
	struct imx6_pcie *imx6_pcie = platform_get_drvdata(pdev);

	/* bring down link, so bootloader gets clean state in case of reboot */
	imx6_pcie_assert_core_reset(imx6_pcie);
}

static const struct imx6_pcie_drvdata drvdata[] = {
	[IMX6Q] = {
		.variant = IMX6Q,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE,
		.dbi_length = 0x200,
	},
	[IMX6SX] = {
		.variant = IMX6SX,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
	},
	[IMX6QP] = {
		.variant = IMX6QP,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY |
			 IMX6_PCIE_FLAG_IMX6_SPEED_CHANGE |
			 IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
		.dbi_length = 0x200,
	},
	[IMX7D] = {
		.variant = IMX7D,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND,
	},
	[IMX8MQ] = {
		.variant = IMX8MQ,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_SUPPORTS_L1SS,
	},
	[IMX8MM] = {
		.variant = IMX8MM,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_SUPPORTS_L1SS,
	},
	[IMX8QM] = {
		.variant = IMX8QM,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
	[IMX8QXP] = {
		.variant = IMX8QXP,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
	[IMX8MP] = {
		.variant = IMX8MP,
		.mode = DW_PCIE_RC_TYPE,
		.flags = IMX6_PCIE_FLAG_SUPPORTS_SUSPEND |
			 IMX6_PCIE_FLAG_SUPPORTS_L1SS,
	},
	[IMX8QXP_EP] = {
		.variant = IMX8QXP_EP,
		.mode = DW_PCIE_EP_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
	[IMX8QM_EP] = {
		.variant = IMX8QM_EP,
		.mode = DW_PCIE_EP_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_CPU_ADDR_FIXUP,
	},
	[IMX8MQ_EP] = {
		.variant = IMX8MQ_EP,
		.mode = DW_PCIE_EP_TYPE,
	},
	[IMX8MM_EP] = {
		.variant = IMX8MM_EP,
		.mode = DW_PCIE_EP_TYPE,
	},
	[IMX8MP_EP] = {
		.variant = IMX8MP_EP,
		.mode = DW_PCIE_EP_TYPE,
	},
	[IMX6SX_EP] = {
		.variant = IMX6SX_EP,
		.mode = DW_PCIE_EP_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY,
	},
	[IMX7D_EP] = {
		.variant = IMX7D_EP,
		.mode = DW_PCIE_EP_TYPE,
	},
	[IMX6Q_EP] = {
		.variant = IMX6Q_EP,
		.mode = DW_PCIE_EP_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY,
	},
	[IMX6QP_EP] = {
		.variant = IMX6QP_EP,
		.mode = DW_PCIE_EP_TYPE,
		.flags = IMX6_PCIE_FLAG_IMX6_PHY,
	},
};

static const struct of_device_id imx6_pcie_of_match[] = {
	{ .compatible = "fsl,imx6q-pcie",  .data = &drvdata[IMX6Q],  },
	{ .compatible = "fsl,imx6sx-pcie", .data = &drvdata[IMX6SX], },
	{ .compatible = "fsl,imx6qp-pcie", .data = &drvdata[IMX6QP], },
	{ .compatible = "fsl,imx7d-pcie",  .data = &drvdata[IMX7D],  },
	{ .compatible = "fsl,imx8mq-pcie", .data = &drvdata[IMX8MQ], },
	{ .compatible = "fsl,imx8mm-pcie", .data = &drvdata[IMX8MM], },
	{ .compatible = "fsl,imx8qm-pcie", .data = &drvdata[IMX8QM], },
	{ .compatible = "fsl,imx8qxp-pcie", .data = &drvdata[IMX8QXP], },
	{ .compatible = "fsl,imx8mp-pcie", .data = &drvdata[IMX8MP], },
	{ .compatible = "fsl,imx8qxp-pcie-ep", .data = &drvdata[IMX8QXP_EP], },
	{ .compatible = "fsl,imx8qm-pcie-ep", .data = &drvdata[IMX8QM_EP], },
	{ .compatible = "fsl,imx8mq-pcie-ep", .data = &drvdata[IMX8MQ_EP], },
	{ .compatible = "fsl,imx8mm-pcie-ep", .data = &drvdata[IMX8MM_EP], },
	{ .compatible = "fsl,imx8mp-pcie-ep", .data = &drvdata[IMX8MP_EP], },
	{ .compatible = "fsl,imx6sx-pcie-ep", .data = &drvdata[IMX6SX_EP], },
	{ .compatible = "fsl,imx7d-pcie-ep", .data = &drvdata[IMX7D_EP], },
	{ .compatible = "fsl,imx6q-pcie-ep", .data = &drvdata[IMX6Q_EP], },
	{ .compatible = "fsl,imx6qp-pcie-ep", .data = &drvdata[IMX6QP_EP], },
	{},
};

static struct platform_driver imx6_pcie_driver = {
	.driver = {
		.name	= "imx6q-pcie",
		.of_match_table = imx6_pcie_of_match,
		.suppress_bind_attrs = true,
		.pm = &imx6_pcie_pm_ops,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe    = imx6_pcie_probe,
	.shutdown = imx6_pcie_shutdown,
};

static void imx6_pcie_quirk(struct pci_dev *dev)
{
	struct pci_bus *bus = dev->bus;
	struct pcie_port *pp = bus->sysdata;

	/* Bus parent is the PCI bridge, its parent is this platform driver */
	if (!bus->dev.parent || !bus->dev.parent->parent)
		return;

	/* Make sure we only quirk devices associated with this driver */
	if (bus->dev.parent->parent->driver != &imx6_pcie_driver.driver)
		return;

	if (pci_is_root_bus(bus)) {
		struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
		struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

		/*
		 * Limit config length to avoid the kernel reading beyond
		 * the register set and causing an abort on i.MX 6Quad
		 */
		if (imx6_pcie->drvdata->dbi_length) {
			dev->cfg_size = imx6_pcie->drvdata->dbi_length;
			dev_info(&dev->dev, "Limiting cfg_size to %d\n",
					dev->cfg_size);
		}
	}
}
DECLARE_PCI_FIXUP_CLASS_HEADER(PCI_VENDOR_ID_SYNOPSYS, 0xabcd,
			PCI_CLASS_BRIDGE_PCI, 8, imx6_pcie_quirk);

static void imx6_pcie_l1ss_quirk(struct pci_dev *dev)
{
	u32 reg, rc_l1sub, ep_l1sub, header;
	int ttl, ret;
	int pos = PCI_CFG_SPACE_SIZE;
	struct pci_bus *bus = dev->bus;
	struct pcie_port *pp = bus->sysdata;
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct imx6_pcie *imx6_pcie = to_imx6_pcie(pci);

	/* Return directly, if the L1SS is not supported by RC */
	if (!(imx6_pcie->drvdata->flags & IMX6_PCIE_FLAG_SUPPORTS_L1SS))
		return;

	/* Make sure the L1SS is not force disabled. */
	if (imx6_pcie->l1ss_clkreq == 0)
		return;

	reg = dw_pcie_find_ext_capability(pci, PCI_EXT_CAP_ID_L1SS);
	rc_l1sub = dw_pcie_readl_dbi(pci, reg + PCI_L1SS_CAP);

	/* minimum 8 bytes per capability */
	ttl = (PCI_CFG_SPACE_EXP_SIZE - PCI_CFG_SPACE_SIZE) / 8;
	ret = dw_pcie_read(pp->va_cfg0_base + pos, 4, &header);
	/*
	 * If we have no capabilities, this is indicated by cap ID,
	 * cap version and next pointer all being 0.
	 */
	if (header == 0)
		return;

	while (ttl-- > 0) {
		if (PCI_EXT_CAP_ID(header) == PCI_EXT_CAP_ID_L1SS && pos != 0)
			break;

		pos = PCI_EXT_CAP_NEXT(header);
		if (pos < PCI_CFG_SPACE_SIZE)
			break;

		ret = dw_pcie_read(pp->va_cfg0_base + pos, 4, &header);
	}
	ret = dw_pcie_read(pp->va_cfg0_base + pos + PCI_L1SS_CAP, 4, &ep_l1sub);

	if (rc_l1sub & ep_l1sub & PCI_L1SS_CAP_L1_PM_SS) {
		imx6_pcie->l1ss_clkreq = 1;
		imx6_pcie_clkreq_enable(imx6_pcie);
	} else {
		imx6_pcie->l1ss_clkreq = 0;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_SYNOPSYS, 0xabcd, imx6_pcie_l1ss_quirk);

static int __init imx6_pcie_init(void)
{
#ifdef CONFIG_ARM
	/*
	 * Since probe() can be deferred we need to make sure that
	 * hook_fault_code is not called after __init memory is freed
	 * by kernel and since imx6q_pcie_abort_handler() is a no-op,
	 * we can install the handler here without risking it
	 * accessing some uninitialized driver state.
	 */
	hook_fault_code(8, imx6q_pcie_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
#endif

	return platform_driver_register(&imx6_pcie_driver);
}
device_initcall(imx6_pcie_init);
MODULE_LICENSE("GPL v2");
