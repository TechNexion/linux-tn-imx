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
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/regmap.h>
#include <linux/mfd/axonfabric-core.h>
#include <linux/mfd/axonfabric.h>
#include <linux/pinctrl/axonfabric-pins.h>

#include "pinctrl-utils.h"

#define NBANK(chip) DIV_ROUND_UP(chip->gpio_chip.ngpio, BANK_SZ)
#define GPIO_TO_IOBANK_ADDR(off) ((u16)AXONF_ADDR_IOBLOCK + (((u16)off / BANK_SZ) << 4) + ((u16)off % BANK_SZ))

#define AXONF_PINCONF_MODE_PUSH_PULL 	BIT(AXONF_IOB_OFFSET_PP)
#define AXONF_PINCONF_MODE_OPEN_DRAIN 	0



struct axonf_pin_function {
	const char *name;
	const char * const *groups;
	unsigned ngroups;
	int mux_option;
};

struct axonfabric_gpio {
	struct axonf_chip *axonf_chip;
	struct gpio_chip gpio_chip;
	struct device *dev;
	struct pinctrl_dev *pctl;
	const struct axonf_pin_function *functions;
	unsigned num_functions;
	const struct axonf_pingroup *pin_groups;
	int num_pin_groups;
	const struct pinctrl_pin_desc *pins;
	unsigned num_pins;
};

enum axonf_pinmux_option {
	AXONF_PINMUX_PASSTHRU_OUTPUT = 0,
	AXONF_PINMUX_PASSTHRU_INPUT = 1,
	AXONF_PINMUX_ALT0_OUTPUT = 2,
	AXONF_PINMUX_ALT1_OUTPUT = 3,
	AXONF_PINMUX_GPIO = 4,
};


#define AXONF_FUNCTION_GROUP(fname, mux)			\
	{						\
		.name = #fname,				\
		.groups = gpio_groups,			\
		.ngroups = ARRAY_SIZE(gpio_groups),	\
		.mux_option = AXONF_PINMUX_##mux,	\
	}

static const struct axonf_pin_function axonf_pin_function[] = {
	AXONF_FUNCTION_GROUP(passthru_out, PASSTHRU_OUTPUT),
	AXONF_FUNCTION_GROUP(passthru_in, PASSTHRU_INPUT),
	AXONF_FUNCTION_GROUP(alt0_out, ALT0_OUTPUT),
	AXONF_FUNCTION_GROUP(alt1_out, ALT1_OUTPUT),
	AXONF_FUNCTION_GROUP(gpio, GPIO),
};

static int axonf_pinctrl_get_groups_count(struct pinctrl_dev *pctldev) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_gpio->num_pin_groups;
}

static const char *axonf_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned group) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_gpio->pin_groups[group].name;
}

static int axonf_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned group, const unsigned **pins, unsigned *num_pins) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	*pins = axonfabric_gpio->pin_groups[group].pins;
	*num_pins = axonfabric_gpio->pin_groups[group].npins;
	return 0;
}

static const struct pinctrl_ops axonf_pinctrl_ops = {
	.get_groups_count = axonf_pinctrl_get_groups_count,
	.get_group_name = axonf_pinctrl_get_group_name,
	.get_group_pins = axonf_pinctrl_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int axonf_pinctrl_get_funcs_count(struct pinctrl_dev *pctldev) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_gpio->num_functions;
}

static const char *axonf_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
			unsigned function) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_gpio->functions[function].name;
}

static int axonf_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
		unsigned function, const char * const **groups,
		unsigned * const num_groups) {

	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);

	*groups = axonfabric_gpio->functions[function].groups;
	*num_groups = axonfabric_gpio->functions[function].ngroups;
	return 0;
}

/**
 * axonf_pinctrl_set: set function for pinctrl group
 * @pctrldev: this device
 * @function: unsigned integer function number
 * @group: unsigned integer pinctrl group
 * returns 0 on success, less than zero on failure
 */

static int axonf_pinctrl_set(struct pinctrl_dev *pctldev, unsigned function,
		unsigned group) {
	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;
	u8 seldir;
	unsigned pin;
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, "called for group %u\n", group);


	switch(axonfabric_gpio->functions[function].mux_option) {
		case AXONF_PINMUX_PASSTHRU_OUTPUT:
			seldir = (AXONF_IOB_SEL_PASSTHROUGH << AXONF_IOB_OFFSET_SEL) | AXONF_IOB_MASK_ENABLE;
			break;
		case AXONF_PINMUX_PASSTHRU_INPUT:
			seldir = (AXONF_IOB_SEL_PASSTHROUGH << AXONF_IOB_OFFSET_SEL) |
				AXONF_IOB_MASK_DIR | AXONF_IOB_MASK_ENABLE;
			break;
		case AXONF_PINMUX_ALT0_OUTPUT:
			seldir = (AXONF_IOB_SEL_ALT0 << AXONF_IOB_OFFSET_SEL) | AXONF_IOB_MASK_ENABLE;
			break;
		case AXONF_PINMUX_ALT1_OUTPUT:
			seldir = (AXONF_IOB_SEL_ALT1 << AXONF_IOB_OFFSET_SEL) | AXONF_IOB_MASK_ENABLE;
			break;
		case AXONF_PINMUX_GPIO:
			seldir = (AXONF_IOB_SEL_GPIO << AXONF_IOB_OFFSET_SEL) ;
			break;
		default:
			seldir = (AXONF_IOB_SEL_GPIO << AXONF_IOB_OFFSET_SEL);
	}

	pin = axonfabric_gpio->pin_groups[group].pins[0];

	dev_dbg(&chip->client->dev, "%s(): Grpup %u (pin %u) to function %u and val %02X\n",
		__func__, group, pin, function, seldir);

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(pin),
			AXONF_IOB_MASK_SEL | AXONF_IOB_MASK_DIR | AXONF_IOB_MASK_ENABLE, seldir);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Group %d update failed %d\n",
			group, ret);
		return ret;
	}
	//axonfabric_gpio->gpio_control[group].io_function = function;

	return ret;
}

static const struct pinmux_ops axonf_pinmux_ops = {
	.get_functions_count	= axonf_pinctrl_get_funcs_count,
	.get_function_name	= axonf_pinctrl_get_func_name,
	.get_function_groups	= axonf_pinctrl_get_func_groups,
	.set_mux		= axonf_pinctrl_set,
};

static int axonf_pinconf_get(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *config) {
	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;
	enum pin_config_param param = pinconf_to_config_param(*config);
	int arg = 0;
	int ret = 0;
	u8 prop;

	AXONF_DBG_INFO(&chip->client->dev, "called for pin %u\n", pin);

	switch (param) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		prop = AXONF_PINCONF_MODE_OPEN_DRAIN;
		break;

	case PIN_CONFIG_DRIVE_PUSH_PULL:
		prop = AXONF_PINCONF_MODE_PUSH_PULL;
		break;

	default:
		dev_err(&chip->client->dev, "Properties not supported\n");
		return -ENOTSUPP;
	}

	// Read the configuration
	ret = axonf_reg_read(chip, GPIO_TO_IOBANK_ADDR(pin));
	if(ret < 0) {
		dev_err(&chip->client->dev, "Could not get configuration.\n");
		return ret;
	}

	if((u8)(ret & 0xFF) & prop)
		arg = 1;

	*config = pinconf_to_config_packed(param, (u16)arg);

	return 0;
}

static int axonf_pinconf_set(struct pinctrl_dev *pctldev,
			unsigned pin, unsigned long *configs,
			unsigned num_configs)
{
	struct axonfabric_gpio *axonfabric_gpio = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_gpio->axonf_chip;
	enum pin_config_param param;
	u8 mode_prop = 0;
	int i;
	int ret = 0;

	AXONF_DBG_INFO(&chip->client->dev, "called for pin %u\n", pin);

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);

		switch (param) {
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			mode_prop |= AXONF_PINCONF_MODE_OPEN_DRAIN;
			break;
		case PIN_CONFIG_DRIVE_PUSH_PULL:
			mode_prop |= AXONF_PINCONF_MODE_PUSH_PULL;
			break;
		default:
			dev_err(&chip->client->dev, "Properties not supported\n");
			return -ENOTSUPP;
		}
	}

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(pin),
				AXONF_IOB_MASK_PP, mode_prop);

	if(ret<0)
		dev_err(&chip->client->dev, "Could not set configuration.\n");

	return ret;
}

static const struct pinconf_ops axonf_pinconf_ops = {
	.pin_config_get = axonf_pinconf_get,
	.pin_config_set = axonf_pinconf_set,
};

static struct pinctrl_desc axonf_pinctrl_desc = {
	.pctlops = &axonf_pinctrl_ops,
	.pmxops = &axonf_pinmux_ops,
	.confops = &axonf_pinconf_ops,
	.owner = THIS_MODULE,
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

	axonfabric_gpio->pins = axonf_pins_desc;
	axonfabric_gpio->num_pins = ARRAY_SIZE(axonf_pins_desc);
	axonfabric_gpio->functions = axonf_pin_function;
	axonfabric_gpio->num_functions = ARRAY_SIZE(axonf_pin_function);
	axonfabric_gpio->pin_groups = axonf_pingroups;
	axonfabric_gpio->num_pin_groups = ARRAY_SIZE(axonf_pingroups);
	axonf_pinctrl_desc.name = dev_name(&pdev->dev);
	axonf_pinctrl_desc.pins = axonf_pins_desc;
	axonf_pinctrl_desc.npins = ARRAY_SIZE(axonf_pins_desc);
	axonfabric_gpio->pctl = devm_pinctrl_register(&pdev->dev, &axonf_pinctrl_desc,
					     axonfabric_gpio);
	if (IS_ERR(axonfabric_gpio->pctl)) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		return PTR_ERR(axonfabric_gpio->pctl);
	}

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
