/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_AXONFABRIC_H
#define _LINUX_AXONFABRIC_H

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>

#define AXONF_ADDR_MAGIC   0x0000
#define AXONF_ADDR_VERSION 0x0004
#define AXONF_ADDR_CONFIG  0x0008
#define AXONF_ADDR_CTRLREG 0x0010
#define AXONF_ADDR_RGBLED  0x0020
#define AXONF_ADDR_IOBLOCK 0x0100

#define AXONF_SIZE_MAGIC   4
#define AXONF_SIZE_VERSION 3
#define AXONF_SIZE_CONFIG  1
#define AXONF_SIZE_CTRLREG 2
#define AXONF_SIZE_RGBLED  3
#define AXONF_SIZE_IOBLOCK 256

/* For Axon Fabric Firmware version 0.2.0 and later */

#define AXONF_IOB_OFFSET_EN			0
#define AXONF_IOB_OFFSET_PP			1
#define AXONF_IOB_OFFSET_DIR		2
#define AXONF_IOB_OFFSET_SEL		3
#define AXONF_IOB_OFFSET_ODR		5
#define AXONF_IOB_OFFSET_IDR		6
#define AXONF_IOB_OFFSET_INT_IDR	7

#define AXONF_REGMAP_NAME	"axonfabric_regmap"

#define AXONF_IOB_MASK_ENABLE		(0x01 << AXONF_IOB_OFFSET_EN)
#define AXONF_IOB_MASK_PP			(0x01 << AXONF_IOB_OFFSET_PP)
#define AXONF_IOB_MASK_DIR			(0x01 << AXONF_IOB_OFFSET_DIR)
#define AXONF_IOB_MASK_SEL			(0x03 << AXONF_IOB_OFFSET_SEL)
#define AXONF_IOB_MASK_ODR			(0x01 << AXONF_IOB_OFFSET_ODR)
#define AXONF_IOB_MASK_IDR			(0x01 << AXONF_IOB_OFFSET_IDR)
#define AXONF_IOB_MASK_INT_IDR		(0x01 << AXONF_IOB_OFFSET_INT_IDR)

#define AXONF_IOB_REG_CTRLSTATUS	0x00
#define AXONF_IOB_REG_IRQ			0x08


/* IO Block definitions */
#define AXONF_IOB_SEL_PASSTHROUGH   0x0
#define AXONF_IOB_SEL_GPIO          0x3
#define AXONF_IOB_SEL_ALT0          0x1
#define AXONF_IOB_SEL_ALT1          0x2

#define AXONF_CTRLREG_GIO_ENABLE				0
#define AXONF_CTRLREG_SHARED_SYSCONFIG_DISABLE	1
#define AXONF_CTRLREG_LB_TEST_MODE_ENABLE		15

#define AXONF_NGPIOS					104
#define AXONF_NBANKS					13

#define MAX_BANK 		13
#define BANK_SZ 		8

/*
 * Driver data and types
 */
#define AXONF_GPIO_MASK					0x000FF
#define AXONF_INT_MASK					0x00100
#define AXONF_TYPE_MASK					0xFF000
#define AXONF_SOC_MASK 					0x0F000
#define AXONF_FLVL_MASK					0xF0000
#define AXONF_FLVL_1					0x10000
#define AXONF_FLVL_2					0x20000
#define AXONF_FLVL_3					0x30000

#define AXONF_INT						0x00100
#define AXONF_IMX6						0x01000
#define AXONF_IMX8MM					0x02000

#define AXONF_CHIP_TYPE(x)	((x) & AXONF_TYPE_MASK)
#define AXONF_SOC_TYPE(x)	((x) & AXONF_SOC_MASK)

#define EMIT_DEBUG_INFO 0  // Set to 1 to turn on helpful debug messages
#define AXONF_DBG_INFO(dev, fmt, ...) \
	do { if (EMIT_DEBUG_INFO) dev_info(dev, "%s: " fmt, __func__, __VA_ARGS__); } while (0)

// Enable DEBUGFS
#define CONFIG_AXONF_DEBUGFS

/* platform data for the Axon Fabric driver */

struct axonf_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	u32		invert;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

struct axonf_chip {
	u8 bank_mask[MAX_BANK];
	u8 allocated[MAX_BANK];
	u8 fw_version[AXONF_SIZE_VERSION];
	struct regmap* regmap;

#if 0 // #ifdef CONFIG_GPIO_PCA953X_IRQ
	struct mutex irq_lock;
	u8 irq_mask[MAX_BANK];
	u8 irq_stat[MAX_BANK];
	u8 irq_trig_raise[MAX_BANK];
	u8 irq_trig_fall[MAX_BANK];
#endif

#ifdef CONFIG_AXONF_DEBUGFS
	struct dentry *debugfs_top_dir;
	u16 a; // address in fabric to read/write from
#endif

	struct i2c_client *client;
	unsigned long driver_data;
	struct regulator *regulator;
	struct gpio_desc *reset_gpio;
	u8 dir_lock;
	struct mutex io_lock;

	u32 statusled_rgb_color;
	u16 ctrlreg;
	u8 cfgdata;

};

#endif /* _LINUX_AXONFABRIC_H */
