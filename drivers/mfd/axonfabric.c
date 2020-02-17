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
#include <linux/mfd/core.h>
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
#include <linux/uaccess.h>
#include <linux/regmap.h>

#include <asm/unaligned.h>

#define BANK_TO_IOBANK_ADDR(bank) ((u16)AXONF_ADDR_IOBLOCK + ((u16)bank << 4))

//#define USE_SINGLE_REG_TRANS

static const struct mfd_cell axonfabric_cells[] = {
	{
		.name = "axonfabric-pinctrl",
		.of_compatible = "technexion,axonfabric-pinctrl",
	},
	{
		.name = "axonfabric-gpio",
		.of_compatible = "technexion,axonfabric-gpio",
	},
};

/*
 * Magic data
 */
const u8 axonf_magic[] = "AXON\0";

/*
 * Bank Masks: Tell driver which I/O blocks can be used as GPIO
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
 * HACK - need to figure out how to push the pointer to this struct through
 * the inode private data.
 */
#ifdef CONFIG_AXONF_DEBUGFS

struct axonf_chip* g_chip;
#endif // CONFIG_AXONF_DEBUGFS

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

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X\n", reg);

	mutex_lock(&chip->io_lock);
	ret = regmap_read(chip->regmap, reg, &val);
	mutex_unlock(&chip->io_lock);

	if (ret < 0)
		return ret;
	else {
		AXONF_DBG_INFO(&chip->client->dev, " Read value: %02X", val);
		return val;
	}
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
#ifndef USE_SINGLE_REG_TRANS
int axonf_bulk_read(struct axonf_chip *chip, unsigned short reg,
		     int count, u8 *buf)
{
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, count: %d\n", reg, count);

	mutex_lock(&chip->io_lock);
	ret = regmap_bulk_read(chip->regmap, reg, buf, count);
	mutex_unlock(&chip->io_lock);

	return ret;
}
#else
int axonf_bulk_read(struct axonf_chip *chip, unsigned short reg,
		     int count, u8 *buf)
{
	int ret = 0;
	unsigned int val;
	u16 i;

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, count: %d, using single transactions.\n", reg, count);

	mutex_lock(&chip->io_lock);
	for(i=0; i < count; i++) {
		ret = regmap_read(chip->regmap, reg+i, &val);
		if(ret < 0){
			dev_err(&chip->client->dev, "%s: failed. addr: %04X, error: %d\n", __func__, reg + i, ret);
			return ret;
		}
		buf[i] =  (u8)val;
	}
	mutex_unlock(&chip->io_lock);

	return ret;
}
#endif
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
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, val: %02X\n", reg, val);

	mutex_lock(&chip->io_lock);
	ret = regmap_write(chip->regmap, reg, val);
	mutex_unlock(&chip->io_lock);

	return ret;

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
#ifndef USE_SINGLE_REG_TRANS
int axonf_bulk_write(struct axonf_chip *chip, unsigned short reg, int count,
		    u8 *buf) {

	int ret;
	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, count: %d\n", reg, count);

	mutex_lock(&chip->io_lock);
	ret = regmap_bulk_write(chip->regmap, reg, buf, count);
	mutex_unlock(&chip->io_lock);

	return ret;

}
#else
int axonf_bulk_write(struct axonf_chip *chip, unsigned short reg, int count,
		    u8 *buf) {

	int ret = 0;
	u16 i;

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, count: %d, using single transactions.", reg, count);

	mutex_lock(&chip->io_lock);
	for(i=0; i< count; i++) {
		ret = regmap_write(chip->regmap, reg+i, buf[i]);
		if(ret < 0) {
			dev_err(&chip->client->dev, "%s: failed. addr: %04X, error: %d\n", __func__, reg + i, ret);
			return ret;
		}
	}
	mutex_unlock(&chip->io_lock);

	return ret;

}
#endif
EXPORT_SYMBOL_GPL(axonf_bulk_write);

/**
 * axonf_reg_update_bits: Update register bits
 *
 * @chip: Device to write to.
 * @reg: Register to update.
 * @mask: Bitmask, with bits to be updated set.
 * @val: Value of bits to update.
 */
int axonf_reg_update_bits(struct axonf_chip *chip, unsigned short reg, u8 mask,
		    u8 val) {
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called. Addr:%04X, mask: %02X, val: %02X\n", reg, mask, val);

	mutex_lock(&chip->io_lock);
	ret = regmap_update_bits(chip->regmap, reg, mask, val);
	mutex_unlock(&chip->io_lock);

	if(ret < 0)
		dev_err(&chip->client->dev, "%s: failed. error: %d\n",__func__, ret);

	return ret;
}
EXPORT_SYMBOL_GPL(axonf_reg_update_bits);

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

static ssize_t axonf_lock_direction_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int len, val;
	struct axonf_chip *chip = dev_get_drvdata(dev);

	val = chip->dir_lock ? '1':'0';

	len = sprintf(buf,"%c\n", val);

	return len;
}

static ssize_t axonf_lock_direction_store(struct device *dev, struct device_attribute *attr,
					const char *buf, size_t count)
{
	int val,ret;
	struct axonf_chip *chip = dev_get_drvdata(dev);

	ret = sscanf(buf,"%d", &val);

	if(ret <= 0)
		return ret;

	chip->dir_lock = val ? 1:0;

	return count;
}

static DEVICE_ATTR(
		lock_direction,
		S_IRUGO | S_IWUSR,
		axonf_lock_direction_show,
        axonf_lock_direction_store);

static struct attribute *axonf_attrs[] = {
    &dev_attr_statusled_rgbhex.attr,
	&dev_attr_shared_sysconfig_disable.attr,
	&dev_attr_gio_enable.attr,
	&dev_attr_lock_direction.attr,
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

static int device_axonf_init(struct axonf_chip *chip, u32 invert)
{
	int ret;

	dev_info(&chip->client->dev,"Beginning init.\n");

	ret = axonf_read_magic(chip);

	if (ret)
		goto out;

	ret = axonf_read_version(chip);

	if (ret)
		goto out;

	ret = axonf_read_cfgdata(chip);

	if (ret)
		goto out;

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

	if(chip->dir_lock)
		ret = axonf_write_ctrlreg(chip,
			(1 << AXONF_CTRLREG_GIO_ENABLE) |
			(1 << AXONF_CTRLREG_LB_TEST_MODE_ENABLE));  // Put in loopback test mode, and enable global IO
	else
		ret = axonf_write_ctrlreg(chip,(1 << AXONF_CTRLREG_GIO_ENABLE));  // Enable global IO

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
		if(axonf_bulk_read(chip, BANK_TO_IOBANK_ADDR(j), BANK_SZ, regs)) goto out;

		// Form the message
		sprintf(msg, "%02x ", regs[0]);
		for(i = 1; i<BANK_SZ; i++) {
			sprintf(msg, "%s%02x ", msg, regs[i]);
		}

		seq_printf(m,"io bank %02d ctrl_status=  %s\n",j,msg);
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

static ssize_t axonf_d_debug_read(struct file * file,
		char __user *user_buf, size_t count, loff_t *ppos) {

	char buf[32];
	ssize_t len;
	int val;
	struct axonf_chip * chip = g_chip;

	val = axonf_reg_read(chip, chip->a);
	if(val < 0)
		return val;

	len = sprintf(buf, "0x%02x\n", val);

	return simple_read_from_buffer(user_buf, count, ppos, buf, len);
}

static ssize_t axonf_d_debug_write(struct file * file,
		const char __user *user_buf, size_t count, loff_t *ppos) {

	char buf[32];
	ssize_t len;
	unsigned int val;
	struct axonf_chip *chip = g_chip;

	len = min(count, sizeof(buf) -1);
	if (copy_from_user(buf, user_buf, len))
		return -EFAULT;

	buf[len] = '\0';
	if (kstrtouint(buf, 0, &val))
		return -EINVAL;

	// Write value to address
	axonf_reg_write(chip, chip->a, (u8)val);

	return count;
}


static const struct file_operations axonf_d_debug_fops = {
	.read		= axonf_d_debug_read,
	.write      = axonf_d_debug_write,
	.llseek		= default_llseek,
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

	ret = debugfs_create_x16("a", S_IRUGO | S_IWUGO,
					chip->debugfs_top_dir, &(chip->a));

	ret = debugfs_create_file("d", S_IRUGO | S_IWUGO,
					chip->debugfs_top_dir, (void*) chip,
					&axonf_d_debug_fops);

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

static const struct regmap_range axonf_volatile_ranges[] = {
	regmap_reg_range(AXONF_ADDR_IOBLOCK,
				AXONF_ADDR_IOBLOCK + AXONF_SIZE_IOBLOCK - 1),
};

static const struct regmap_access_table axonf_volatile_table = {
	.yes_ranges	= axonf_volatile_ranges,
	.n_yes_ranges	= ARRAY_SIZE(axonf_volatile_ranges),
};

static const struct regmap_config axonf_regmap_config = {
	.name = AXONF_REGMAP_NAME,
	.reg_bits = 16,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,

	.rd_table = &axonf_readable_table,
	.wr_table = &axonf_writeable_table,
	.volatile_table = &axonf_volatile_table,
};
EXPORT_SYMBOL_GPL(axonf_regmap_config);

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

static const struct of_device_id axonf_dt_ids[] = {
	{ .compatible = "technexion,axon-imx6-f01",   .data = (void*)(AXONF_NGPIOS | AXONF_IMX6   | AXONF_FLVL_1), },
	{ .compatible = "technexion,axon-imx8mm-f01", .data = (void*)(AXONF_NGPIOS | AXONF_IMX8MM | AXONF_FLVL_1), },
	{ .compatible = "technexion,axon-imx6-f03",   .data = (void*)(AXONF_NGPIOS | AXONF_IMX6   | AXONF_INT | AXONF_FLVL_3), },
	{ .compatible = "technexion,axon-imx8mm-f03", .data = (void*)(AXONF_NGPIOS | AXONF_IMX8MM | AXONF_INT | AXONF_FLVL_3), },
	{ }
};
MODULE_DEVICE_TABLE(of, axonf_dt_ids);

static int axonf_probe(struct i2c_client *client,
				   const struct i2c_device_id *i2c_id)
{
	struct axonf_chip *chip;
	int irq_base = 0;
	int ret;
	u32 invert = 0;
	struct regulator *reg;
	struct gpio_desc *reset_gpio;

	chip = devm_kzalloc(&client->dev,
			sizeof(struct axonf_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	mutex_init(&chip->io_lock);
	i2c_set_clientdata(client, chip);

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
	lockdep_set_subclass(&chip->i2c_lock, i2c_adapter_depth(client->adapter));

	ret = device_reset(&client->dev);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if(of_get_property(chip->client->dev.of_node,"lock-direction",NULL)) {
		chip->dir_lock = 1;
		dev_warn(&client->dev, "'lock-direction' set. Kernel gpio calls will not change direction of IOs. For QC testing only.\n");
	}

	ret = device_axonf_init(chip, invert);

	if (ret)
		goto err_exit;

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

	ret = mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO, axonfabric_cells,
			      ARRAY_SIZE(axonfabric_cells), NULL, 0, NULL);
			      //regmap_irq_get_domain(tps->irq_data));
	if (ret < 0)
		goto err_exit;

	dev_info(&client->dev, "Probed.\n");

	return 0;

err_exit:
	if(chip->regulator)
		regulator_disable(chip->regulator);

	return ret;
}

static int axonf_remove(struct i2c_client *client)
{
	struct axonf_chip *chip = i2c_get_clientdata(client);
	int ret = 0;

#ifdef CONFIG_AXONF_DEBUGFS
	axonf_free_debugfs(chip);
#endif // CONFIG_AXONF_DEBUGFS

	mfd_remove_devices(&client->dev);

	sysfs_remove_groups(&client->dev.kobj, axonf_groups);

	if(chip->regulator)
		regulator_disable(chip->regulator);

	return ret;
}


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
subsys_initcall_sync(axonf_init);

static void __exit axonf_exit(void)
{
	i2c_del_driver(&axonf_driver);
}
module_exit(axonf_exit);

MODULE_AUTHOR("john weber <john.weber@technexion.com>");
MODULE_DESCRIPTION("Multifunction driver for Axon Fabric");
MODULE_LICENSE("GPL");
