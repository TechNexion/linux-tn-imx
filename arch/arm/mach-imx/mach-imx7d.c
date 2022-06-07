/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx7-iomuxc-gpr.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/pm_opp.h>
#include <linux/phy.h>
#include <linux/regmap.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

static struct property device_disabled = {
	.name = "status",
	.length = sizeof("disabled"),
	.value = "disabled",
};

static int ar8031_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Set RGMII IO voltage to 1.8V */
	phy_write(dev, 0x1d, 0x1f);
	phy_write(dev, 0x1e, 0x8);

	/* disable phy AR8031 SmartEEE function. */
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);
	val = phy_read(dev, 0xe);
	val &= ~(0x1 << 8);
	phy_write(dev, 0xe, val);

	return 0;
}

static int bcm54220_phy_fixup(struct phy_device *dev)
{
	/* enable RXC skew select RGMII copper mode */
	phy_write(dev, 0x1e, 0x21);
	phy_write(dev, 0x1f, 0x7ea8);
	phy_write(dev, 0x1e, 0x2f);
	phy_write(dev, 0x1f, 0x71b7);

	return 0;
}

static int ar8035_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Ar803x phy SmartEEE feature cause link status generates glitch,
	* which cause ethernet link down/up issue, so disable SmartEEE
	*/
	phy_write(dev, 0xd, 0x3);
	phy_write(dev, 0xe, 0x805d);
	phy_write(dev, 0xd, 0x4003);

	val = phy_read(dev, 0xe);
	phy_write(dev, 0xe, val & ~(1 << 8));

	/*
	* Enable 125MHz clock from CLK_25M on the AR8031.  This
	* is fed in to the IMX6 on the ENET_REF_CLK (V22) pad.
	* Also, introduce a tx clock delay.
	*
	* This is the same as is the AR8031 fixup.
	*/

	ar8031_phy_fixup(dev);

	/*check phy power*/
	val = phy_read(dev, 0x0);
	if (val & BMCR_PDOWN)
		phy_write(dev, 0x0, val & ~BMCR_PDOWN);

	 return 0;
}

static int rtl8211f_phy_fixup(struct phy_device *dev)
{
	u16 val;

	/* Energy Efficient Ethernet(EEE) feature causes bidirectional
	 * transmission issue, so disable EEE.
	 */
	phy_write(dev, 0x1f, 0x0a43);
	val = phy_read(dev, 0x19);
	phy_write(dev, 0x19, val & ~(1 << 5));

	return 0;
}

#define PHY_ID_RTL8211F 0x001cc916
#define PHY_ID_AR8035   0x004dd072
#define PHY_ID_AR8031	0x004dd074
#define PHY_ID_BCM54220	0x600d8589

static void __init imx7d_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		phy_register_fixup_for_uid(PHY_ID_AR8035, 0xffffffef,
					   ar8035_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_AR8031, 0xffffffff,
					   ar8031_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_BCM54220, 0xffffffff,
					   bcm54220_phy_fixup);
		phy_register_fixup_for_uid(PHY_ID_RTL8211F, 0xffffffff,
					   rtl8211f_phy_fixup);
	}
}

static void __init imx7d_enet_mdio_fixup(void)
{
	struct regmap *gpr;

	/* The management data input/output (MDIO) bus where often high-speed,
	 * open-drain operation is required. i.MX7D TO1.0 ENET MDIO pin has no
	 * open drain as IC ticket number: TKT252980, i.MX7D TO1.1 fix the issue.
	 * GPR1[8:7] are reserved bits at TO1.0, there no need to add version check.
	 */
	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR0, IMX7D_GPR0_ENET_MDIO_OPEN_DRAIN_MASK,
				   IMX7D_GPR0_ENET_MDIO_OPEN_DRAIN_MASK);
	else
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
}

static void __init imx7d_enet_clk_sel(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx7d-iomuxc-gpr");
	if (!IS_ERR(gpr)) {
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_TX_CLK_SEL_MASK, 0);
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX7D_GPR1_ENET_CLK_DIR_MASK, 0);
	} else {
		pr_err("failed to find fsl,imx7d-iomux-gpr regmap\n");
	}
}

/*
 * OCOTP_TESTER3[9:8] (see Fusemap Description Table offset 0x440)
 * defines a 2-bit SPEED_GRADING
 */
#define OCOTP_CFG3			0x440
#define OCOTP_TESTER3_SPEED_SHIFT	8
#define OCOTP_TESTER3_SPEED_800MHZ	0
#define OCOTP_TESTER3_SPEED_850MHZ	1
#define OCOTP_TESTER3_SPEED_1GHZ	2
#define OCOTP_TESTER3_SPEED_1P2GHZ	3

static void __init imx7d_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np = NULL;
	void __iomem *base;
	u32 val;

	if (cpu_is_imx7d())
		np = of_find_compatible_node(NULL, NULL, "fsl,imx7d-ocotp");

	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_TESTER3_SPEED_SHIFT;
	val &= 0x3;
	if (cpu_is_imx7d()) {
		if (val < OCOTP_TESTER3_SPEED_1GHZ) {
			if (dev_pm_opp_disable(cpu_dev, 996000000))
				pr_warn("Failed to disable 996MHz OPP\n");
		}

		if (val < OCOTP_TESTER3_SPEED_1P2GHZ) {
			if (dev_pm_opp_disable(cpu_dev, 1200000000))
				pr_warn("Failed to disable 1200MHz OPP\n");
		}
	}
	iounmap(base);

put_node:
	of_node_put(np);
}

static void __init imx7d_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (dev_pm_opp_of_add_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx7d_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static inline void imx7d_enet_init(void)
{
	imx6_enet_mac_init("fsl,imx7d-fec", "fsl,imx7d-ocotp");
	imx7d_enet_mdio_fixup();
	imx7d_enet_phy_init();
	imx7d_enet_clk_sel();
}

static inline void imx7d_disable_arm_arch_timer(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "arm,armv7-timer");
	if (node) {
		pr_info("disable arm arch timer for nosmp!\n");
		of_add_property(node, &device_disabled);
	}
}

static void __init imx7d_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_default_populate(NULL, NULL, parent);
	imx7d_pm_init();
	imx_anatop_init();
	imx7d_enet_init();
}

static void __init imx7d_init_irq(void)
{
	imx_gpcv2_check_dt();
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
#ifndef CONFIG_SMP
	imx7d_disable_arm_arch_timer();
#endif
}

static void __init imx7d_init_late(void)
{
	if (IS_ENABLED(CONFIG_ARM_IMX7D_CPUFREQ)) {
		imx7d_opp_init();
		platform_device_register_simple("imx7d-cpufreq", -1, NULL, 0);
	}
	imx7d_cpuidle_init();
}

static void __init imx7d_map_io(void)
{
	debug_ll_io_init();
	imx7_pm_map_io();
	imx_busfreq_map_io();
}

static const char *const imx7d_dt_compat[] __initconst = {
	"fsl,imx7d",
	"fsl,imx7s",
	NULL,
};

DT_MACHINE_START(IMX7D, "Freescale i.MX7 Dual (Device Tree)")
	.map_io         = imx7d_map_io,
	.smp            = smp_ops(imx_smp_ops),
	.init_irq	= imx7d_init_irq,
	.init_machine	= imx7d_init_machine,
	.init_late	= imx7d_init_late,
	.dt_compat	= imx7d_dt_compat,
MACHINE_END
