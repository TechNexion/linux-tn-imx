/*
 *  Axon Fabric GPIO driver
 *
 * 	Copyright (C) John Weber (john.weber@technexion.com)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/mfd/axonfabric-core.h>
#include <linux/mfd/axonfabric.h>

#define NBANK(chip) DIV_ROUND_UP(chip->gpio_chip.ngpio, BANK_SZ)
#define GPIO_TO_IOBANK_ADDR(off) ((u16)AXONF_ADDR_IOBLOCK + (((u16)off / BANK_SZ) << 4) + ((u16)off % BANK_SZ))

struct axonfabric_gpio {
	struct axonf_chip *axonf_chip;
	struct gpio_chip gpio_chip;
	struct device *dev;
};

/**
 * axonf_gpio_enable: Enable or disable a gpio
 * @gc: This GPIO chip
 * @off: GPIO offset
 * @val: Nonzero to enable, zero to disable
 */
static int axonf_gpio_enable(struct gpio_chip *gc, unsigned off, unsigned val)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called for line %d\n", off);

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_ENABLE, val ? 0xFF : 0);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to enable line %d: err %d\n",off,ret);
	}

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
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip =axonfabric_gpio->axonf_chip;
	int ret = 0;

	AXONF_DBG_INFO(&chip->client->dev, "called for line %d\n", off);

	// The new selection value is in 'val' but needs to be shifted up
	// and masked.
	val = (val << AXONF_IOB_OFFSET_SEL) & AXONF_IOB_MASK_SEL;

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_SEL, val);

	return ret;
}

/**
 * axonf_gpio_direction_input: Set the direction of the GPIO as input
 * @gc: This gpiochip
 * @off: GPIO offset
 */
static int axonf_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip =axonfabric_gpio->axonf_chip;

	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called for line %d\n", off);

	if(chip->dir_lock)
		return 0;

	// Disable the I/O while we update the direction
	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_ENABLE, 0);
	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to disable I/O for line %d: err %d\n",off,ret);
		return ret;
	}

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_DIR, AXONF_IOB_MASK_DIR);

	if (ret < 0 ) {
		dev_err(&chip->client->dev, "failed to set direction line %d: err %d\n",off,ret);
		return ret;
	}

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_ENABLE, 0xFF);

	if(ret < 0) {
		dev_err(&chip->client->dev, "failed to enable I/O for line %d: err %d\n",off,ret);
	}

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
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip =axonfabric_gpio->axonf_chip;

	int ret = 0;

	AXONF_DBG_INFO(&chip->client->dev, "called for line %d, val: %d\n", off, val);

	if(chip->dir_lock)
		return ret;

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_DIR | AXONF_IOB_MASK_ODR, val ? AXONF_IOB_MASK_ODR : 0);

	if (ret < 0 ) {
		dev_err(&chip->client->dev, "failed to set output value for line %d: err %d\n",off,ret);
		return ret;
	}

	return ret;
}

/**
 * axonf_gpio_get_value: Returns the value of the requested GPIO
 * @gc: The gpiochip
 * @off: The GPIO offset
 */
static int axonf_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip =axonfabric_gpio->axonf_chip;

	int ret;
	u16 address;

	address = GPIO_TO_IOBANK_ADDR(off);

	AXONF_DBG_INFO(&chip->client->dev, "Called for line %d, address is %x\n", off, address);

	ret = axonf_reg_read(chip, address);

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed to get value for line %d: err %d\n", off, ret);
		return 0;
	}

	if(chip->dir_lock){
		// If the dir_lock is set, this is used for loopback test mode
		// Reading the internal IDR is necessary instead of the external IDR
		return ((u8)ret & AXONF_IOB_MASK_INT_IDR) ? 1 : 0;
	}

	return ((u8)ret & AXONF_IOB_MASK_IDR) ? 1 : 0;
}

/**
 * axonf_gpio_set_value: Sets the value of the requested GPIO
 * @gc: This gpiochip
 * @off: The offset of the GPIO
 * @val: Nonzero to set the GPIO high, zero to set it low
 */
static void axonf_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, " called for line %d\n", off);

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(off),
				AXONF_IOB_MASK_ODR, val ? AXONF_IOB_MASK_ODR : 0);

	if (ret < 0)
		dev_err(&chip->client->dev, "failed to set value for line %d: err %d\n",off,ret);

}

/**
 * axonf_gpio_get_direction: gets the direction of the GPIO
 * @gc: this GPIO chip
 * @off: the offset of the GPIO
 */
static int axonf_gpio_get_direction(struct gpio_chip *gc, unsigned off)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;

	int ret;
	u16 address;

	address = GPIO_TO_IOBANK_ADDR(off);

	AXONF_DBG_INFO(&chip->client->dev, " called for line %d, address is  0x%04X\n", off, address);

	ret = axonf_reg_read(chip, address);

	if(ret >= 0)
		return ((u8)ret & AXONF_IOB_MASK_DIR) ? 1 : 0;

	dev_err(&chip->client->dev, "failed to get direction for line %d: err %d\n",off,ret);
	return ret;

}

static int axonf_gpio_request(struct gpio_chip *gc, unsigned off)
{
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;

	u8 bank_mask;
	int offset, allocated, bank;
	int sel;
	int ret = 0;

	AXONF_DBG_INFO(&chip->client->dev, " called for line %d\n", off);

	/* Check to see if this gpio is available */
	offset = off % BANK_SZ;
	bank = off / BANK_SZ;

	bank_mask = (chip->bank_mask[bank] >> offset) & 0x1;
	allocated = (chip->allocated[bank] >> offset) & 0x1;

	sel = axonf_reg_read(chip, GPIO_TO_IOBANK_ADDR(off));

	if(sel < 0) {
		dev_err(&chip->client->dev, "%s: select could not be read.\n",__func__);
	}

	sel = (sel & AXONF_IOB_MASK_SEL) >> AXONF_IOB_OFFSET_SEL;

	/* check if this pin is available */
	if (!bank_mask) {
		dev_info(&chip->client->dev,
			"pin %u cannot be allocated (check bank mask)\n", off);
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
	struct axonfabric_gpio *axonfabric_gpio = gpiochip_get_data(gc);
	struct axonf_chip *chip =axonfabric_gpio->axonf_chip;

	u8 bank_mask;
	int offset, allocated, bank;

	AXONF_DBG_INFO(&chip->client->dev, "called for line %d\n", off);

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

static const struct gpio_chip template_chip = {
	.label				= "gpio-axonfabric",
	.owner				= THIS_MODULE,
	.request			= axonf_gpio_request,
	.free				= axonf_gpio_free,
	.direction_output	= axonf_gpio_direction_output,
	.direction_input	= axonf_gpio_direction_input,
	.get_direction		= axonf_gpio_get_direction,
	.get				= axonf_gpio_get_value,
	.set				= axonf_gpio_set_value,
//	.set_config			= axonf_gpio_set_config, // Not implemented yet, but could be useful
	.can_sleep			= false,
	.ngpio				= AXONF_NGPIOS,
	.base				= -1
};

static int axonfabric_gpio_probe(struct platform_device *pdev)
{
	struct axonf_chip *chip = dev_get_drvdata(pdev->dev.parent);
	struct axonfabric_gpio *axonfabric_gpio;
	int ret;

	axonfabric_gpio = devm_kzalloc(&pdev->dev, sizeof(*axonfabric_gpio),
				     GFP_KERNEL);
	if (!axonfabric_gpio)
		return -ENOMEM;

	axonfabric_gpio->dev = &pdev->dev;
	axonfabric_gpio->dev->of_node = pdev->dev.parent->of_node;
	axonfabric_gpio->axonf_chip = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, axonfabric_gpio);

	memcpy(&axonfabric_gpio->gpio_chip, &template_chip, sizeof(axonfabric_gpio->gpio_chip));

	axonfabric_gpio->gpio_chip.parent = &pdev->dev;

#ifdef CONFIG_OF_GPIO
	axonfabric_gpio->gpio_chip.of_node = chip->client->dev.of_node;
#endif

	ret = devm_gpiochip_add_data(&pdev->dev, &axonfabric_gpio->gpio_chip,
				     axonfabric_gpio);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register gpiochip, %d\n", ret);
		return ret;
	}

	platform_set_drvdata(pdev, axonfabric_gpio);

	dev_info(&pdev->dev, "Probed.\n");

	return 0;
}

static int axonfabric_gpio_remove(struct platform_device *pdev)
{
	struct axonfabric_gpio *axonfabric_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&axonfabric_gpio->gpio_chip);

	return 0;
}

static const struct of_device_id axonfabric_gpio_dt_match[] = {
	{ .compatible = "technexion,axonfabric-gpio" },
	{  }
};
MODULE_DEVICE_TABLE(of, axonfabric_gpio_dt_match);

static const struct platform_device_id axonfabric_gpio_id_table[] = {
	{ "axonfabric-gpio", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, axonfabric_gpio_id_table);

static struct platform_driver axonfabric_gpio_driver = {
	.driver = {
		.name = "axonfabric-gpio",
		.of_match_table = of_match_ptr(axonfabric_gpio_dt_match)
	},
	.probe = axonfabric_gpio_probe,
	.remove = axonfabric_gpio_remove,
	.id_table = axonfabric_gpio_id_table,
};

module_platform_driver(axonfabric_gpio_driver);

MODULE_AUTHOR("john weber <john.weber@technexion.com>");
MODULE_DESCRIPTION("GPIO driver for Axon Fabric");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:axonfabric-gpio");
