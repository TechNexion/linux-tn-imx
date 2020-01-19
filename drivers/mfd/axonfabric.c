/* SPDX-License-Identifier: GPL-2.0 */

/*
 *  Driver for TechNexion AXON FABRIC
 *  Implements:
 *  - Configuration of IO banks
 *  - Status LED configuration and control
 *  - GPIO expansion
 *
 *  Utilizes i2c communications interface between the SOC
 *  and the fabric for control and data.
 *
 *  Copyright (C) 2019 TechNexion Ltd.
 *
 *  Derived from drivers/gpio/gpio-pca953x.c
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *
 */

#include <linux/acpi.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/mfd/axonfabric.h>
#include <asm/byteorder.h>
#include <linux/debugfs.h>
#include <linux/regmap.h>

#include <asm/unaligned.h>

/* For Axon Fabric Firmware version 0.2.0 and later */

#define AXONF_IOB_OFFSET_EN			0
#define AXONF_IOB_OFFSET_PP			1
#define AXONF_IOB_OFFSET_DIR		2
#define AXONF_IOB_OFFSET_SEL		3
#define AXONF_IOB_OFFSET_ODR		5
#define AXONF_IOB_OFFSET_IDR		6
#define AXONF_IOB_OFFSET_INT_IDR	7

#define AXONF_REGMAP_NAME	"apds9960_regmap"

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

#define AXONF_NGPIOS					104

#define AXONF_CHIP_TYPE(x)	((x) & AXONF_TYPE_MASK)
#define AXONF_SOC_TYPE(x)	((x) & AXONF_SOC_MASK)

#define AXONF_NBANKS					13

#define CONFIG_AXONF_DEBUGFS

#define AXONF_USE_REGMAP



/*
 * Bank Masks: Tell driver which I/O banks can be used as I2C GPIO
 */
const u8 axonf_imx6_bank_mask[] = {
	0xFF, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0x1F, 0xFC,
	0xFF, 0x7F, 0x7F, 0xFF, 0x7F
	};

const u8 axonf_imx8mm_bank_mask[] = {
	0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0xFF, 0x1F, 0xFC,
	0xFF, 0x7F, 0x1F, 0xFF, 0x7F
	};

/*
 * Magic data
 */
const u8 axonf_magic[] = "AXON\0";

/*
 * ID table. First member is name, second member is data private to the driver.
 * The device data is stored as an unsigned long, but the maximum we will use is
 * 32-bits (unsigned int).
 */

static const struct i2c_device_id axonf_id[] = {
	{ "axon-imx6-f01",   AXONF_NGPIOS | AXONF_IMX6   | AXONF_FLVL_1, },
	{ "axon-imx8mm-f01", AXONF_NGPIOS | AXONF_IMX8MM | AXONF_FLVL_1, },
	{ "axon-imx6-f03",   AXONF_NGPIOS | AXONF_IMX6   | AXONF_FLVL_3 | AXONF_INT, },
	{ "axon-imx8mm-f03", AXONF_NGPIOS | AXONF_IMX8MM | AXONF_FLVL_3 | AXONF_INT, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, axonf_id);

#define MAX_BANK 		13
#define BANK_SZ 		8

#define NBANK(chip) DIV_ROUND_UP(chip->gpio_chip.ngpio, BANK_SZ)

struct axonf_reg_config {
	int ctrl_status;
	int irq;
};

static const struct axonf_reg_config axonf_regs = {
	.ctrl_status =       AXONF_IOB_REG_CTRLSTATUS,
	.irq =     			 AXONF_IOB_REG_IRQ
};

struct axonf_chip {
	unsigned gpio_start;
	u8 reg_ctrl_status[MAX_BANK*8];
	u8 reg_irq[MAX_BANK*8];
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
#endif

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
	unsigned long driver_data;
	struct regulator *regulator;
	u8 dir_lock;

	const struct axonf_reg_config *regs;

	u32 statusled_rgb_color;
	u16 ctrlreg;
	u8 cfgdata;

	int (*write_regs)(struct axonf_chip *, int, u8 *);
	int (*read_regs)(struct axonf_chip *, int, u8 *);
};

/*
 * HACK - need to figure out how to push the pointer to this struct through
 * the inode private data.
 */
#ifdef CONFIG_AXONF_DEBUGFS

struct axonf_chip* g_chip;
#endif // CONFIG_AXONF_DEBUGFS

/*
 * Reads a block of data from a designated address
 */

#define MAX_AXON_I2C_XFER_SIZE 256

/**
 * axonf_reg_read: Read a single axon fabric register.
 *
 * @chip: Device to read from.
 * @reg: Register to read.
 */
int axonf_reg_read(struct axonf_chip *chip, unsigned short reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(chip->regmap, reg, &val);

	if (ret < 0)
		return ret;
	else
		return val;
}
EXPORT_SYMBOL_GPL(axonf_reg_read);

/**
 * axonf_bulk_read: Read multiple axon fabric registers
 *
 * @chip: Device to read from
 * @reg: First register
 * @count: Number of registers
 * @buf: Buffer to fill.
 */
int axonf_bulk_read(struct axonf_chip *chip, unsigned short reg,
		     int count, u8 *buf)
{
	return regmap_bulk_read(chip->regmap, reg, buf, count);
}
EXPORT_SYMBOL_GPL(axonf_bulk_read);

/**
 * axonf_reg_write: Write a single axon fabric register.
 *
 * @chip: Device to write to.
 * @reg: Register to write to.
 * @val: Value to write.
 */
int axonf_reg_write(struct axonf_chip *chip, unsigned short reg,
		    u8 val) {

	return regmap_write(chip->regmap, reg, val);

}
EXPORT_SYMBOL_GPL(axonf_reg_write);

/**
 * axonf_bulk_write: Write a sequence of registers
 *
 * @chip: Device to write to.
 * @reg: Register to start from
 * @count: Number of bytes to write
 * @buf: Buffer to write
 */
int axonf_bulk_write(struct axonf_chip *chip, unsigned short reg, int count,
		    u8 *buf) {

	return regmap_bulk_write(chip->regmap, reg, buf, count);

}
EXPORT_SYMBOL_GPL(axonf_bulk_write);


/**
 * axonf_iob_read_single: Returns the value of the requested ioblock register (reg)
 * containing the data for the signal at the requested offset (off)
 * @chip: This device
 * @reg: The ioblock register
 * @val: Pointer to place the read value
 * @off: IO offset
 */
static int axonf_iob_read_single(struct axonf_chip *chip, int reg, u8 *val,
				int off)
{

	int ret;
	int bank = off / BANK_SZ;

	u16 address = (u16)AXONF_ADDR_IOBLOCK + ((u16)bank << 4) + (u16)reg;

	ret = axonf_reg_read(chip, address);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = ret;

	return 0;
}

/*
 * Writes a byte (val) to the register (reg) of the IOB specified by offset (off)
 */
static int axonf_iob_write_single(struct axonf_chip *chip, int reg, u8 val,
				int off)
{

	int ret;
	u16 bank = off / BANK_SZ;

	u16 address = (u16)AXONF_ADDR_IOBLOCK + (bank << 4) + (u16)reg;

	ret = axonf_reg_write(chip, address, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to write data\n");
		return ret;
	}

	return 0;
}

/*
 * Returns the value of the requested register (reg) and the requested
 * bank number (bank)
 */
static int axonf_iob_read_reg(struct axonf_chip *chip, int reg, u8 *val,
				int bank)
{

	int ret;

	u16 address = (u16)AXONF_ADDR_IOBLOCK + ((u16)bank << 4) + (u16)reg;

	ret = axonf_reg_read(chip, address);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register\n");
		return ret;
	}

	*val = ret;

	return 0;
}

/*
 * Writes a byte (val) to the register (reg) of the IOB specified by the bank number
 */
static int axonf_iob_write_reg(struct axonf_chip *chip, int reg, u8 val,
				int bank)
{

	int ret;
	u16 address = (u16)AXONF_ADDR_IOBLOCK + (bank << 4) + (u16)reg;

	ret = axonf_reg_write(chip, address, val);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to write data. reg: %d bank: %d\n", reg, bank);
		return ret;
	}

	return 0;
}

/*
 * reads all of the data from one bank
 * bank: bank address (start address)
 * val: Pointer to array of values to store the data (must be at least BANK_SZ size)
 */
static int axonf_iob_read_regs(struct axonf_chip *chip, int bank, u8 *val)
{
	int ret = 0;
	int i;

	for (i = 0; i < BANK_SZ; i++) {

		// Read the register at address. Address is reg + i (reg is the starting address)
		// Address to store the data is "val[i]". We also provide the bank, which is calculated
		// from the reg.
		ret = axonf_iob_read_reg(chip, i, val+i, bank);
		if (ret < 0) {
			dev_err(&chip->client->dev, "failed reading register\n");
			return ret;
		}
	}

	return 0;
}

/*
 * Writes all of the data for the specific bank
 * reg: bank address
 * val: Pointer to array of values to be written (must be at least BANK_SZ size)
 */
static int axonf_iob_write_regs(struct axonf_chip *chip, int bank, u8 *val)
{
	int ret = 0;

	int i;

	for (i = 0; i < BANK_SZ; i++) {

		// Weite the register at address. Address is reg + i (reg is the starting address)
		// Address to store the data is "val[i]". We also provide the bank, which is calculated
		// from the reg.
		ret = axonf_iob_write_reg(chip, i, val[i], bank);
		if (ret < 0) {
			dev_err(&chip->client->dev, "failed writing register\n");
			return ret;
		}
	}
	return 0;
}

/**
 * axonf_gpio_enable: Enable or disable a gpio
 * @gc: This GPIO chip
 * @off: GPIO offset
 * @val: Nonzero to enable, zero to disable
 */
static int axonf_gpio_enable(struct gpio_chip *gc, unsigned off, unsigned val)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;
	int bank = off / BANK_SZ;
	int offset = off % BANK_SZ;

	if(val) /* enable */
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] | AXONF_IOB_MASK_ENABLE;
	else
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] & ~AXONF_IOB_MASK_ENABLE;

	ret = axonf_iob_write_reg(chip, chip->regs->ctrl_status + offset, reg_val, bank);
	if (ret)
		goto exit;

	chip->reg_ctrl_status[bank * BANK_SZ + offset] = reg_val;
exit:
	return ret;
}

/**
 * axonf_gpio_set_select: Set the selection for an ioblock (gpio)
 * Normally used to set the ioblock as a gpio.
 * @gc: This gpiochip
 * @off: GPIO offset for this chip
 * @val: The selection for this gpio
 */
static int axonf_gpio_set_select(struct gpio_chip *gc, unsigned off, unsigned val)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret, bank, offset;

	// Bank is calculated using the offset (off) within the gpiochip
	bank = off / BANK_SZ;

	// Offset, below, is the gpio index within the axon iobank
	offset = off % BANK_SZ;

	// The new selection value is in 'val' but needs to be shifted up
	// and masked.
	val = (val << AXONF_IOB_OFFSET_SEL) & AXONF_IOB_MASK_SEL;

	// Set the register value (bitwise OR)
	reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] | val;

	ret = axonf_iob_write_reg(chip, chip->regs->ctrl_status + offset, reg_val, bank);
	if (ret)
		goto exit;

	chip->reg_ctrl_status[bank * BANK_SZ + offset] = reg_val;

exit:

	return ret;
}

/**
 * axonf_gpio_direction_input: Set the direction of the GPIO as input
 * @gc: This gpiochip
 * @off: GPIO offset
 */
static int axonf_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;
	int bank, offset;

	// Do the normal bank/offset calculations (see earlier functions for comments)
	bank = off / BANK_SZ;
	offset = off % BANK_SZ;

	if(chip->dir_lock)
		return 0;

	// Setting the axon iobank control_status register DIR bit makes the
	// IOBLOCK an INPUT.
	reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] | AXONF_IOB_MASK_DIR;

	ret = axonf_iob_write_single(chip, chip->regs->ctrl_status + offset, reg_val, off);
	if (ret) {
		dev_err(&chip->client->dev, "failed to set direction input for line %d: err %d\n",off,ret);
		goto exit;
	}

	chip->reg_ctrl_status[bank * BANK_SZ + offset] = reg_val;

exit:

	return ret;
}

/**
 * axonf_gpio_direction_output: Set the output value of the gpio, then sets
 * the direction to output.
 * @gc: This gpiochip
 * @off: GPIO offset
 * @val: nonzero sets output high, zero sets output low
 */

static int axonf_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret = 0;
	int bank, offset;

	// Do the normal bank/offset calculations (see earlier functions for comments)
	bank = off / BANK_SZ;
	offset = off % BANK_SZ;

	if(val) /* enable */
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] | AXONF_IOB_MASK_ODR;
	else
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] & ~AXONF_IOB_MASK_ODR;

	// The direction lock flag is for local loopback test purposes.
	// When we want to check the IO connectivity between the SOC and the FPGA, it is
	// very useful to be able to keep the IOblock as an input (when the default)
	// behavior would be to have it as an output if the IO is in GPIO mode.
	if(!chip->dir_lock) {
		// Set the direction to OUTPUT (set the DIR flag to be ZERO/LOW)
		reg_val &= ~AXONF_IOB_MASK_DIR;
	}

	ret = axonf_iob_write_single(chip, chip->regs->ctrl_status + offset, reg_val, off);

	if (ret) {
		dev_err(&chip->client->dev, "failed to set output value for line %d: err %d\n",off,ret);
		goto exit;
	}

	chip->reg_ctrl_status[bank * BANK_SZ + offset] = reg_val;

exit:

	return ret;
}

/**
 * axonf_gpio_get_value: Returns the value of the requested GPIO
 * @gc: The gpiochip
 * @off: The GPIO offset
 */
static int axonf_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;
	int offset;

	// Do the normal bank/offset calculations (see earlier functions for comments)
	offset = off % BANK_SZ;

	ret = axonf_iob_read_single(chip, chip->regs->ctrl_status + offset, &reg_val, off);

	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		dev_err(&chip->client->dev, "failed to get value for line %d: err %d\n", off, ret);
		return 0;
	}

	return (reg_val & AXONF_IOB_MASK_IDR) ? 1 : 0;
}

/**
 * axonf_gpio_set_value: Sets the value of the requested GPIO
 * @gc: This gpiochip
 * @off: The offset of the GPIO
 * @val: Nonzero to set the GPIO high, zero to set it low
 */
static void axonf_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret, offset, bank;

	// Calculate the bank
	bank = off / BANK_SZ;

	// Calculate the offset within the bank
	offset = off % BANK_SZ;

	if(val)
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] | AXONF_IOB_MASK_ODR;
	else
		reg_val = chip->reg_ctrl_status[bank * BANK_SZ + offset] & ~AXONF_IOB_MASK_ODR;

	ret = axonf_iob_write_single(chip, chip->regs->ctrl_status + offset, reg_val, off);

	if (ret)
		goto exit;

	chip->reg_ctrl_status[bank * BANK_SZ + offset] = reg_val;
exit:

	return;
}

/**
 * axonf_gpio_get_direction: gets the direction of the GPIO
 * @gc: this GPIO chip
 * @off: the offset of the GPIO
 */
static int axonf_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret, offset;

	offset = off % BANK_SZ;

	ret = axonf_iob_read_single(chip, chip->regs->ctrl_status + offset, &reg_val, off);

	if (ret < 0)
		return ret;

	return (reg_val & AXONF_IOB_MASK_DIR) ? 1 : 0;

}


static int axonf_gpio_request(struct gpio_chip *gc, unsigned off)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 bank_mask;
	int offset, allocated, bank;
	unsigned int sel;
	int ret = 0;

	/* Check to see if this gpio is available */
	offset = off % BANK_SZ;
	bank = off / BANK_SZ;

	bank_mask = (chip->bank_mask[bank] >> offset) & 0x1;
	allocated = (chip->allocated[bank] >> offset) & 0x1;

	sel = (chip->reg_ctrl_status[bank * BANK_SZ + offset] & AXONF_IOB_MASK_SEL) >> AXONF_IOB_OFFSET_SEL;

	/* check if this pin is available */
	if (!bank_mask) {
		dev_info(&chip->client->dev,
			"pin %u cannot be allocated (check mask)\n", off);
		ret = -EINVAL;
		goto exit;
	}

	/* Make sure that this iob can be configured as GPIO.
	   IOBs that are not configured as GPIO during initialization cannot
	   be allocated as GPIO at runtime. This to to prevent conflicts IOB
	   that might be in passthrough mode, or in alternate modes. */
	if (sel != AXONF_IOB_SEL_GPIO)  {
		/* Let's send a helpful message to the developer */
		switch (sel) {
			case AXONF_IOB_SEL_PASSTHROUGH:
				dev_info(&chip->client->dev,
					"pin %u cannot be allocated as GPIO. Configured in passthrough mode. sel=%x\n", off, sel);
				break;
			default:
				dev_info(&chip->client->dev,
					"pin %u cannot be allocated as GPIO. Configured in one of the ALT modes. sel=%x\n", off, sel);
				break;
		}
		ret = -EBUSY;
		goto exit;
	}

	/* check if this pin is already allocated */
	if (allocated & (1 << offset)) {
		dev_info(&chip->client->dev,
			"pin %u already in use\n", off);
		ret = -EBUSY;
		goto exit;
	}

	chip->allocated[bank] |= (1 << offset);

	/* Change the sel mux to select the ODR */
	ret = axonf_gpio_set_select(gc, off, AXONF_IOB_SEL_GPIO);
	if(ret)
		goto exit;

	/* make sure the pin is enabled */
	ret = axonf_gpio_enable(gc, off, 1);
	if(ret)
		goto exit;

exit:
	return ret;
}

static void axonf_gpio_free(struct gpio_chip *gc, unsigned off)
{
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 bank_mask;
	int offset, allocated, bank;

	// Bank calculation
	bank = off / BANK_SZ;

	/* Check to see if this gpio is available */
	bank_mask = chip->bank_mask[bank];
	allocated = chip->allocated[bank];
	offset = off % BANK_SZ;

	/* check if this pin is available */
	if ((bank_mask & (1 << offset)) == 0) {
		dev_info(&chip->client->dev,
			"pin %u cannot be freed (check mask)\n", offset);
		return;
	}

	chip->allocated[off / BANK_SZ] &= ~(1u << offset);

}


static void axonf_setup_gpio(struct axonf_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = axonf_gpio_direction_input;
	gc->direction_output = axonf_gpio_direction_output;
	gc->get = axonf_gpio_get_value;
	gc->set = axonf_gpio_set_value;
	gc->get_direction = axonf_gpio_get_direction;
	gc->request = axonf_gpio_request;
	gc->free = axonf_gpio_free;
	gc->can_sleep = false;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}


/**
 * axonf_read_magic: Read magic data of fabric. Store it into the
 * the chip device data.
 * @chip: This chip
 */
static int axonf_read_magic(struct axonf_chip *chip)
{
	u8 magic[AXONF_SIZE_MAGIC+1];
	int ret;

	memset(magic, 0, AXONF_SIZE_MAGIC+1);

	ret = axonf_bulk_read(chip, AXONF_ADDR_MAGIC, AXONF_SIZE_MAGIC, magic);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed reading magic %d\n",ret);
		goto exit;
	}

	if(strncmp(magic, axonf_magic, AXONF_SIZE_MAGIC)) {
		dev_err(&chip->client->dev,
				"Bad magic. Read 0x%02X 0x%02X 0x%02X 0x%02X (%s)  Expected: 0x%02X 0x%02X 0x%02X 0x%02X (%s) \n",
				magic[0], magic[1], magic[2], magic[3], magic,
				axonf_magic[0], axonf_magic[1], axonf_magic[2], axonf_magic[3], axonf_magic);

		ret = -ENODEV;
		goto exit;
	}

	ret = 0;

exit:

	return ret;
}

/**
 * axonf_read_cfgdata: Read configuration data
 * Configuration data tells us SOM and fabric-specific details regarding
 * the firmware loaded into the fabric. Currently it reads the fabric
 * level and the SOM type, but should be extended to read fabric capabilities
 * and OEM-specific data for customizations.
 * @chip: Ths device data
 */
static int axonf_read_cfgdata(struct axonf_chip *chip)
{
	int ret;

	ret = axonf_reg_read(chip, AXONF_ADDR_CONFIG);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed reading config data %d\n",ret);
		goto exit;
	}

	chip->cfgdata = (u8) ret;

	ret = 0;

exit:

	return ret;
}

/**
 * axonf_read_version: Read version of fabric and store it into the device data.
 * @chip: This device data
 */
static int axonf_read_version(struct axonf_chip *chip)
{
	u8 version[3];
	int ret;

	ret = axonf_bulk_read(chip, AXONF_ADDR_VERSION, AXONF_SIZE_VERSION, version);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed reading version %d\n",ret);
		goto exit;
	}

	memcpy(chip->fw_version, version, AXONF_SIZE_VERSION);
	ret = 0;

exit:

	return ret;
}

/**
 * axonf_read_status_led: Read status LED value
 * @chip: This device data
 */
static int axonf_read_statusled(struct axonf_chip *chip)
{
	u8 led_val[AXONF_SIZE_RGBLED+1];
	int ret;
	u32 led_rgb = 0;

	ret = axonf_bulk_read(chip, AXONF_ADDR_RGBLED, AXONF_SIZE_RGBLED, led_val);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to read status led %d\n",ret);
		goto exit;
	}

	led_rgb = 	(( led_val[0] << 16) & 0xFF0000 ) |  // R
				(( led_val[1] << 8)  & 0x00FF00 ) |  // G
				(  led_val[2]        & 0x0000FF );   // B

	chip->statusled_rgb_color = led_rgb;

	ret = 0;

exit:

	return ret;
}

/**
 * axonf_write_status_led: Writes new value to status LED
 * @chip: This device data
 * @led_rgb: A 32-bit value containing a 24-bit RGB color to set the LED to.
 */
static int axonf_write_statusled(struct axonf_chip *chip, u32 led_rgb)
{
	u8 led_val[AXONF_SIZE_RGBLED+1];
	int ret;

	led_val[0] = led_rgb >> 16 & 0xFF;  // R
	led_val[1] = led_rgb >> 8  & 0xFF;  // G
	led_val[2] = led_rgb & 0xFF;        // B

	ret = axonf_bulk_write(chip, AXONF_ADDR_RGBLED, AXONF_SIZE_RGBLED, led_val);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to write status led %d\n",ret);
		goto exit;
	}

	chip->statusled_rgb_color = led_rgb & 0x00FFFFFF;

	ret = 0;

exit:

	return ret;
}

/**
 * axonf_read_ctrlreg: Read control register
 * @chip: This chip
 */
static int axonf_read_ctrlreg(struct axonf_chip *chip)
{
	u8 reg[AXONF_SIZE_CTRLREG+1];
	int ret;
	u16 ctrlreg = 0;

	ret = axonf_bulk_read(chip, AXONF_ADDR_CTRLREG, AXONF_SIZE_CTRLREG, reg);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to read control register %d\n",ret);
		goto exit;
	}

	ctrlreg =  (reg[0] << 8) | reg[1] ;

	dev_info(&chip->client->dev, "Read control register: %04X\n",ctrlreg);

	chip->ctrlreg = ctrlreg;

	ret = 0;

exit:

	return ret;
}

/*
 * axonf_write_ctrlreg: Writes new value to control register
 */

static int axonf_write_ctrlreg(struct axonf_chip *chip, u16 ctrlreg)
{
	u8 val[AXONF_SIZE_CTRLREG+1];
	int ret;

	val[1] = ctrlreg & 0xFF;
	val[0] = (ctrlreg >> 8) & 0xFF;  // Upper byte will go first

	ret = axonf_bulk_write(chip, AXONF_ADDR_CTRLREG, AXONF_SIZE_CTRLREG, val);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to write control register %d\n",ret);
		goto exit;
	}

	dev_info(&chip->client->dev, "Wrote control register: %04X\n",ctrlreg);

	chip->ctrlreg = ctrlreg;

	ret = 0;

exit:

	return ret;
}

/*
 * Sysfs show/store functions
 */

static ssize_t axonf_statusled_rgbhex_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct axonf_chip *chip = dev_get_drvdata(dev);
	int len, ret;

	ret = axonf_read_statusled(chip);

	if(ret) {
		dev_err(&chip->client->dev, "failed to read status led %d\n",ret);
		return ret;
	}

	len = sprintf(buf,"%06X\n",chip->statusled_rgb_color);

	return len;
}

static ssize_t axonf_statusled_rgbhex_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct axonf_chip *chip = dev_get_drvdata(dev);
	int ret;
	u32 led_rgb;

	sscanf(buf,"%06X", &led_rgb);

	ret = axonf_write_statusled(chip, led_rgb);

	if(ret) {
		dev_err(&chip->client->dev, "failed to write status led %d\n",ret);
		return ret;
	}

	return count;
}

static DEVICE_ATTR(
		statusled_rgbhex,
		S_IRUGO | S_IWUSR,
		axonf_statusled_rgbhex_show,
        axonf_statusled_rgbhex_store);

/*
	Set the control register bit to val.
		bit = index of ctrl register bit
		val = value of the bit to set
*/
static int axonf_ctrlreg_write_bit( struct device *dev, char val, int bit) {

	struct axonf_chip *chip = dev_get_drvdata(dev);
	int mask, ret;

	// Test to make sure val is a either 0 or 1
	if( val && (val != 1))
		return -EINVAL;

	mask = 1 << bit;

	if(val)
		chip->ctrlreg |= mask;
	else
		chip->ctrlreg &= ~mask;

	ret = axonf_write_ctrlreg( chip, chip->ctrlreg );

	if(ret) {
		dev_err(&chip->client->dev, "failed to write ctrlreg %d\n",ret);
		return ret;
	}

	return 0;
}


/*
	Read the value of the control register bit
		bit = index of ctrl register bit
		val = value of the bit to set
		returns 0 or 1 upon success, depending on the value of the bit
		returns a negative value upon failure
*/
static int axonf_ctrlreg_read_bit( struct device *dev, int bit) {

	struct axonf_chip *chip = dev_get_drvdata(dev);
	int ret;
	int val;

	ret = axonf_read_ctrlreg(chip);

	if(ret) {
		dev_err(&chip->client->dev, "failed to read control register %d\n",ret);
		return ret;
	}

	val = (chip->ctrlreg & (0x1 << bit)) ? 1 : 0;

	return val;
}

static ssize_t axonf_shared_sc_disable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char val;
	int ret, len;

	ret = axonf_ctrlreg_read_bit(dev, AXONF_CTRLREG_SHARED_SYSCONFIG_DISABLE);

	if(ret < 0) return ret;

	val = ret ? '1' : '0';

	len = sprintf(buf,"%c\n", val);

	return len;
}

static ssize_t axonf_shared_sc_disable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	int val;

	sscanf(buf,"%d", &val);

	ret = axonf_ctrlreg_write_bit(dev, val, AXONF_CTRLREG_SHARED_SYSCONFIG_DISABLE);

	if(ret)
		return ret;

	return count;
}

static DEVICE_ATTR(
		shared_sysconfig_disable,
		S_IRUGO | S_IWUSR,
		axonf_shared_sc_disable_show,
        axonf_shared_sc_disable_store);

static ssize_t axonf_shared_gio_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	char val;
	int ret, len;

	ret = axonf_ctrlreg_read_bit(dev, AXONF_CTRLREG_GIO_ENABLE);

	if(ret < 0) return ret;

	val = ret ? '1' : '0';

	len = sprintf(buf,"%c\n", val);

	return len;
}

static ssize_t axonf_shared_gio_enable_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int ret;
	int val;

	sscanf(buf,"%d", &val);

	ret = axonf_ctrlreg_write_bit(dev, val, AXONF_CTRLREG_GIO_ENABLE);

	if(ret)
		return ret;

	return count;
}

static DEVICE_ATTR(
		gio_enable,
		S_IRUGO | S_IWUSR,
		axonf_shared_gio_enable_show,
        axonf_shared_gio_enable_store);

static struct attribute *axonf_attrs[] = {
    &dev_attr_statusled_rgbhex.attr,
	&dev_attr_shared_sysconfig_disable.attr,
	&dev_attr_gio_enable.attr,
    NULL
};

static const struct attribute_group axonf_group = {
	.name = NULL,
	.attrs = axonf_attrs,
};

static const struct attribute_group *axonf_groups[] = {
	&axonf_group,
	NULL,
};

//ATTRIBUTE_GROUPS(axonf);

#if 0 //#ifdef CONFIG_GPIO_PCA953X_IRQ
static void axonf_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct axonf_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[d->hwirq / BANK_SZ] &= ~(1 << (d->hwirq % BANK_SZ));
}

static void axonf_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct axonf_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[d->hwirq / BANK_SZ] |= 1 << (d->hwirq % BANK_SZ);
}

static void axonf_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct axonf_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void axonf_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct axonf_chip *chip = gpiochip_get_data(gc);
	u8 new_irqs;
	int level, i;
	u8 invert_irq_mask[MAX_BANK];

	if (chip->driver_data & PCA_PCAL) {
		/* Enable latch on interrupt-enabled inputs */
		axonf_write_regs(chip, PCAL953X_IN_LATCH, chip->irq_mask);

		for (i = 0; i < NBANK(chip); i++)
			invert_irq_mask[i] = ~chip->irq_mask[i];

		/* Unmask enabled interrupts */
		axonf_write_regs(chip, PCAL953X_INT_MASK, invert_irq_mask);
	}

	/* Look for any newly setup interrupt */
	for (i = 0; i < NBANK(chip); i++) {
		new_irqs = chip->irq_trig_fall[i] | chip->irq_trig_raise[i];
		new_irqs &= ~chip->reg_direction[i];

		while (new_irqs) {
			level = __ffs(new_irqs);
			axonf_gpio_direction_input(&chip->gpio_chip,
							level + (BANK_SZ * i));
			new_irqs &= ~(1 << level);
		}
	}

	mutex_unlock(&chip->irq_lock);
}

static int axonf_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct axonf_chip *chip = gpiochip_get_data(gc);
	int bank_nb = d->hwirq / BANK_SZ;
	u8 mask = 1 << (d->hwirq % BANK_SZ);

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall[bank_nb] |= mask;
	else
		chip->irq_trig_fall[bank_nb] &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise[bank_nb] |= mask;
	else
		chip->irq_trig_raise[bank_nb] &= ~mask;

	return 0;
}

static struct irq_chip axonf_irq_chip = {
	.name			= "axonfabric",
	.irq_mask		= axonf_irq_mask,
	.irq_unmask		= axonf_irq_unmask,
	.irq_bus_lock		= axonf_irq_bus_lock,
	.irq_bus_sync_unlock	= axonf_irq_bus_sync_unlock,
	.irq_set_type		= axonf_irq_set_type,
};

static bool axonf_irq_pending(struct axonf_chip *chip, u8 *pending)
{
	u8 cur_stat[MAX_BANK];
	u8 old_stat[MAX_BANK];
	bool pending_seen = false;
	bool trigger_seen = false;
	u8 trigger[MAX_BANK];
	int ret, i;

	if (chip->driver_data & PCA_PCAL) {
		/* Read the current interrupt status from the device */
		ret = axonf_read_regs(chip, PCAL953X_INT_STAT, trigger);
		if (ret)
			return false;

		/* Check latched inputs and clear interrupt status */
		ret = axonf_read_regs(chip, PCA953X_INPUT, cur_stat);
		if (ret)
			return false;

		for (i = 0; i < NBANK(chip); i++) {
			/* Apply filter for rising/falling edge selection */
			pending[i] = (~cur_stat[i] & chip->irq_trig_fall[i]) |
				(cur_stat[i] & chip->irq_trig_raise[i]);
			pending[i] &= trigger[i];
			if (pending[i])
				pending_seen = true;
		}

		return pending_seen;
	}

	ret = axonf_read_regs(chip, chip->regs->input, cur_stat);
	if (ret)
		return false;

	/* Remove output pins from the equation */
	for (i = 0; i < NBANK(chip); i++)
		cur_stat[i] &= chip->reg_direction[i];

	memcpy(old_stat, chip->irq_stat, NBANK(chip));

	for (i = 0; i < NBANK(chip); i++) {
		trigger[i] = (cur_stat[i] ^ old_stat[i]) & chip->irq_mask[i];
		if (trigger[i])
			trigger_seen = true;
	}

	if (!trigger_seen)
		return false;

	memcpy(chip->irq_stat, cur_stat, NBANK(chip));

	for (i = 0; i < NBANK(chip); i++) {
		pending[i] = (old_stat[i] & chip->irq_trig_fall[i]) |
			(cur_stat[i] & chip->irq_trig_raise[i]);
		pending[i] &= trigger[i];
		if (pending[i])
			pending_seen = true;
	}

	return pending_seen;
}

static irqreturn_t axonf_irq_handler(int irq, void *devid)
{
	struct axonf_chip *chip = devid;
	u8 pending[MAX_BANK];
	u8 level;
	unsigned nhandled = 0;
	int i;

	if (!axonf_irq_pending(chip, pending))
		return IRQ_NONE;

	for (i = 0; i < NBANK(chip); i++) {
		while (pending[i]) {
			level = __ffs(pending[i]);
			handle_nested_irq(irq_find_mapping(chip->gpio_chip.irqdomain,
							level + (BANK_SZ * i)));
			pending[i] &= ~(1 << level);
			nhandled++;
		}
	}

	return (nhandled > 0) ? IRQ_HANDLED : IRQ_NONE;
}

static int axonf_irq_setup(struct axonf_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret, i;

	if (client->irq && irq_base != -1
			&& (chip->driver_data & PCA_INT)) {
		ret = axonf_read_regs(chip,
					chip->regs->input, chip->irq_stat);
		if (ret)
			return ret;

		/*
		 * There is no way to know which GPIO line generated the
		 * interrupt.  We have to rely on the previous read for
		 * this purpose.
		 */
		for (i = 0; i < NBANK(chip); i++)
			chip->irq_stat[i] &= chip->reg_direction[i];
		mutex_init(&chip->irq_lock);

		ret = devm_request_threaded_irq(&client->dev,
					client->irq,
					   NULL,
					   axonf_irq_handler,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT |
						   IRQF_SHARED,
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			return ret;
		}

		ret =  gpiochip_irqchip_add_nested(&chip->gpio_chip,
						   &axonf_irq_chip,
						   irq_base,
						   handle_simple_irq,
						   IRQ_TYPE_NONE);
		if (ret) {
			dev_err(&client->dev,
				"could not connect irqchip to gpiochip\n");
			return ret;
		}

		gpiochip_set_nested_irqchip(&chip->gpio_chip,
					    &axonf_irq_chip,
					    client->irq);
	}

	return 0;
}

//#else /* CONFIG_GPIO_PCA953X_IRQ */
static int axonf_irq_setup(struct axonf_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;

	if (irq_base != -1 && (chip->driver_data & PCA_INT))
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}
#endif

static void axonf_print_iob_regs(struct axonf_chip *chip)
{
	u8 msg [ 200 ];
	int i,j;

	// Print out the contents of the IOB registers
	for (j=0; j<AXONF_NBANKS; j++) {
		sprintf(msg, "%02x ", chip->reg_ctrl_status[j*BANK_SZ]); \
		for(i = 1; i<BANK_SZ; i++) { \
			sprintf(msg, "%s%02x ", msg, chip->reg_ctrl_status[j*BANK_SZ+i]); \
		}
		dev_info(&chip->client->dev, "io bank %02d ctrl_status=  %s\n", j, msg);
	}
}

static int device_axonf_init(struct axonf_chip *chip, u32 invert)
{
	int ret,i;
	char propname[20];

	chip->regs = &axonf_regs;

	ret = axonf_read_magic(chip);

	if (ret)
		goto out;

	ret = axonf_read_version(chip);

	if (ret)
		goto out;

	ret = axonf_read_cfgdata(chip);

#if 0 // Temporarily disable return on failure here
	if (ret)
		goto out;
#endif

	dev_info(&chip->client->dev,"cfgdata value: 0x%02X\n", chip->cfgdata);

	/* Read the status led */
	ret = axonf_read_statusled(chip);

	if(ret)
		goto out;

	dev_info(&chip->client->dev,"Status LED value: 0x%08X\n", chip->statusled_rgb_color);

	ret = axonf_read_ctrlreg(chip);

	if (ret) {
		dev_err(&chip->client->dev,"error reading control register\n");
		goto out;
	}

	dev_info(&chip->client->dev,"ctrlreg value: 0x%04X\n", chip->ctrlreg);

	for( i = 0; i<MAX_BANK; i++) {

		// Property name is ioblockN-ctrl where N is the bank number.
		sprintf(propname, "ioblock%d-ctrl", i);

		ret = of_property_read_u8_array(
			chip->client->dev.of_node,
			propname,
			&chip->reg_ctrl_status[BANK_SZ*i],
			BANK_SZ);

		if(ret){
			dev_err(&chip->client->dev,"error reading property %s in dev tree\n",propname);
			goto out;
		}

		// Write the values for all of the registers in the bank
		ret = axonf_iob_write_regs(chip, i, chip->reg_ctrl_status + BANK_SZ*i);

		if (ret)
			goto out;
	}

	memset(chip->allocated, 0, sizeof(chip->allocated));

	/* Assign the bank mask based on the driver data */
	switch(AXONF_SOC_TYPE(chip->driver_data)) {
		case AXONF_IMX6:
			memcpy(chip->bank_mask, axonf_imx6_bank_mask, MAX_BANK);
			break;
		case AXONF_IMX8MM:
			memcpy(chip->bank_mask, axonf_imx8mm_bank_mask, MAX_BANK);
			break;
		default:
			dev_err(&chip->client->dev,
					"Cannot set bank mask. Unknown SOC type. %08lX\n",
					chip->driver_data);
			break;
	}

	axonf_print_iob_regs(chip);

out:
	return ret;
}

//
// debugfs enablement
//

static int axonf_iobregs_debug_show(struct seq_file *m, void *unused)
{
	struct axonf_chip *chip = g_chip;
	u8 regs[BANK_SZ];

	u8 msg [ 200 ];
	int i,j;

	for (j=0; j<AXONF_NBANKS; j++) {

		// Read the registers. The address to read is the second argument. The destination
		// memory is the third argument (regs)
		if(axonf_iob_read_regs(chip, j, regs)) goto out;

		// Form the message
		sprintf(msg, "%02x ", regs[0]);
		for(i = 1; i<BANK_SZ; i++) {
			sprintf(msg, "%s%02x ", msg, regs[i]);
		}

		seq_printf(m,"iobank%d ctrl_status=       %s\n",j,msg);
	}

out:
	return 0;
}


static int axonf_iobregs_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, axonf_iobregs_debug_show,
					&inode->i_private);
}

static const struct file_operations axonf_iobregs_debug_fops = {
	.open		= axonf_iobregs_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int axonf_init_debugfs(struct axonf_chip *chip) {
	struct dentry* ret;
	long err;

	chip->debugfs_top_dir = NULL;

	dev_info(&chip->client->dev,
			"Initializing debugfs, chip address: %p\n", chip);

	ret = debugfs_create_dir("axonfabric", NULL);

	if(IS_ERR(ret)) goto exit_err;

	chip->debugfs_top_dir = ret;

	ret = debugfs_create_file("iob_regs", S_IRUGO,
					chip->debugfs_top_dir, (void*) chip,
					&axonf_iobregs_debug_fops);

	if(IS_ERR(ret)) goto exit_err;

	return 0;

exit_err:
	err = PTR_ERR(ret);
	switch (err) {
		case -ENODEV:
			dev_err(&chip->client->dev,
				"Cannot create debugfs entry. Debugfs not compiled in kernel.\n");
			break;
		default:
			dev_err(&chip->client->dev,
				"Cannot create debugfs entry. Error:%ld\n", err);
			break;
	}

	// Clean up
	if(chip->debugfs_top_dir)
		debugfs_remove_recursive(chip->debugfs_top_dir);

	return -1;
}

static void axonf_free_debugfs(struct axonf_chip *chip) {

	// If we have a debugfs, then we need to clean up
	if(chip->debugfs_top_dir)
		debugfs_remove_recursive(chip->debugfs_top_dir);
}

static const struct of_device_id axonf_dt_ids[] = {
	{ .compatible = "technexion,axon-imx6-f01",   .data = (void*)(AXONF_NGPIOS | AXONF_IMX6   | AXONF_FLVL_1), },
	{ .compatible = "technexion,axon-imx8mm-f01", .data = (void*)(AXONF_NGPIOS | AXONF_IMX8MM | AXONF_FLVL_1), },
	{ .compatible = "technexion,axon-imx6-f03",   .data = (void*)(AXONF_NGPIOS | AXONF_IMX6   | AXONF_INT | AXONF_FLVL_3), },
	{ .compatible = "technexion,axon-imx8mm-f03", .data = (void*)(AXONF_NGPIOS | AXONF_IMX8MM | AXONF_INT | AXONF_FLVL_3), },
	{ }
};

static const struct regmap_range axonf_readable_ranges[] = {
	regmap_reg_range(AXONF_ADDR_MAGIC, AXONF_ADDR_CONFIG),
	regmap_reg_range(AXONF_ADDR_CTRLREG,
				AXONF_ADDR_CTRLREG + AXONF_SIZE_CTRLREG - 1 ),
	regmap_reg_range(AXONF_ADDR_RGBLED,
				AXONF_ADDR_RGBLED + AXONF_SIZE_RGBLED - 1),
	regmap_reg_range(AXONF_ADDR_IOBLOCK,
				AXONF_ADDR_IOBLOCK + AXONF_SIZE_IOBLOCK - 1),
};

static const struct regmap_access_table axonf_readable_table = {
	.yes_ranges	= axonf_readable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(axonf_readable_ranges),
};

static const struct regmap_range axonf_writeable_ranges[] = {
	regmap_reg_range(AXONF_ADDR_CTRLREG,
				AXONF_ADDR_CTRLREG + AXONF_SIZE_CTRLREG - 1 ),
	regmap_reg_range(AXONF_ADDR_RGBLED,
				AXONF_ADDR_RGBLED + AXONF_SIZE_RGBLED - 1),
	regmap_reg_range(AXONF_ADDR_IOBLOCK,
				AXONF_ADDR_IOBLOCK + AXONF_SIZE_IOBLOCK - 1),
};

static const struct regmap_access_table axonf_writeable_table = {
	.yes_ranges	= axonf_writeable_ranges,
	.n_yes_ranges	= ARRAY_SIZE(axonf_writeable_ranges),
};

struct regmap_config axonf_regmap_config = {
	.name = AXONF_REGMAP_NAME,
	.reg_bits = 16,
	.val_bits = 8,

	.rd_table = &axonf_readable_table,
	.wr_table = &axonf_writeable_table
};
EXPORT_SYMBOL_GPL(axonf_regmap_config);

static int axonf_probe(struct i2c_client *client,
				   const struct i2c_device_id *i2c_id)
{
	struct axonf_platform_data *pdata;
	struct axonf_chip *chip;
	int irq_base = 0;
	int ret;
	u32 invert = 0;
	struct regulator *reg;

	chip = devm_kzalloc(&client->dev,
			sizeof(struct axonf_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

#ifdef CONFIG_AXONF_DEBUGFS
	g_chip = chip; // HACK
#endif // CONFIG_AXONF_DEBUGFS

	chip->regmap = devm_regmap_init_i2c(client,&axonf_regmap_config);
	if (IS_ERR(chip->regmap)) {
		ret = PTR_ERR(chip->regmap);
		dev_err(&client->dev,"Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
		chip->names = pdata->names;
	} else {
		struct gpio_desc *reset_gpio;

		chip->gpio_start = -1;
		irq_base = 0;

		/*
		 * See if we need to de-assert a reset pin.
		 *
		 * There is no known ACPI-enabled platforms that are
		 * using "reset" GPIO. Otherwise any of those platform
		 * must use _DSD method with corresponding property.
		 */
		reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
						     GPIOD_OUT_LOW);

		if (IS_ERR(reset_gpio)) {
			dev_err(&client->dev, "failed to get reset gpio. error %ld\n", PTR_ERR(reset_gpio));
			return PTR_ERR(reset_gpio);
		}
	}

	chip->client = client;
	chip->dir_lock = 0;

	reg = devm_regulator_get_optional(&client->dev, "vcc");
	if (IS_ERR(reg)) {
		reg = NULL;
	}
	if(reg) {
		ret = regulator_enable(reg);
		if (ret) {
			dev_err(&client->dev, "reg en err: %d\n", ret);
			return ret;
		}
	}

	chip->regulator = reg;

	if (i2c_id) {
		chip->driver_data = i2c_id->driver_data;
	} else {
		const struct of_device_id *match;

		match = of_match_device(axonf_dt_ids, &client->dev);
		if (match) {
			chip->driver_data = (int)(uintptr_t)match->data;
		}
	}

	/*
	 * In case we have an i2c-mux controlled by a GPIO provided by an
	 * expander using the same driver higher on the device tree, read the
	 * i2c adapter nesting depth and use the retrieved value as lockdep
	 * subclass for chip->i2c_lock.
	 *
	 * REVISIT: This solution is not complete. It protects us from lockdep
	 * false positives when the expander controlling the i2c-mux is on
	 * a different level on the device tree, but not when it's on the same
	 * level on a different branch (in which case the subclass number
	 * would be the same).
	 *
	 * TODO: Once a correct solution is developed, a similar fix should be
	 * applied to all other i2c-controlled GPIO expanders (and potentially
	 * regmap-i2c).
	 */
	lockdep_set_subclass(&chip->i2c_lock,
			     i2c_adapter_depth(client->adapter));

	ret = device_reset(&client->dev);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if(of_get_property(chip->client->dev.of_node,"lock-direction",NULL)) {
		chip->dir_lock = 1;
		dev_warn(&client->dev, "'lock-direction' set. Kernel gpio calls will not change direction of IOs. For QC testing only.\n");
	}

	/*
	 * initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	axonf_setup_gpio(chip, chip->driver_data & AXONF_GPIO_MASK);

	chip->write_regs = axonf_iob_write_regs;
	chip->read_regs = axonf_iob_read_regs;

	ret = device_axonf_init(chip, invert);

	if (ret)
		goto err_exit;

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		goto err_exit;

//	ret = axonf_irq_setup(chip, irq_base);
//	if (ret)
//		goto err_exit;

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);

	/* Report FW version */
	dev_info(&client->dev, "Fabric programmed with version: %d.%d.%d\n",
			chip->fw_version[0],
			chip->fw_version[1],
			chip->fw_version[2]);

	/* Create sysfs group for other attributes */
	ret = sysfs_create_groups(&client->dev.kobj,axonf_groups);
	if(ret) {
		dev_err(&client->dev, "sysfs creation failed.\n");
		goto err_exit;
	}

#ifdef CONFIG_AXONF_DEBUGFS
	/* Create debugfs */
	axonf_init_debugfs(chip);
#endif // CONFIG_AXONF_DEBUGFS

	return 0;

err_exit:
	if(chip->regulator)
		regulator_disable(chip->regulator);

	return ret;
}

static int axonf_remove(struct i2c_client *client)
{
	struct axonf_platform_data *pdata = dev_get_platdata(&client->dev);
	struct axonf_chip *chip = i2c_get_clientdata(client);
	int ret;

#ifdef CONFIG_AXONF_DEBUGFS
	axonf_free_debugfs(chip);
#endif // CONFIG_AXONF_DEBUGFS

	sysfs_remove_groups(&client->dev.kobj, axonf_groups);

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
	} else {
		ret = 0;
	}

	if(chip->regulator)
		regulator_disable(chip->regulator);

	return ret;
}



MODULE_DEVICE_TABLE(of, axonf_dt_ids);

static struct i2c_driver axonf_driver = {
	.driver = {
		.name	= "axonfabric",
		.of_match_table = axonf_dt_ids,
	},
	.probe		= axonf_probe,
	.remove		= axonf_remove,
	.id_table	= axonf_id,
};

static int __init axonf_init(void)
{
	return i2c_add_driver(&axonf_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(axonf_init);

static void __exit axonf_exit(void)
{
	i2c_del_driver(&axonf_driver);
}
module_exit(axonf_exit);

MODULE_AUTHOR("john weber <john.weber@technexion.com>");
MODULE_DESCRIPTION("Multifunction driver for Axon Fabric");
MODULE_LICENSE("GPL");
