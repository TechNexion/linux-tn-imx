/*
 *  Axon Fabric Pinctrl driver
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

struct axonfabric_pinctrl {
	struct axonf_chip *axonf_chip;
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

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_pinctrl->num_pin_groups;
}

static const char *axonf_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
		unsigned group) {

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_pinctrl->pin_groups[group].name;
}

static int axonf_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
		unsigned group, const unsigned **pins, unsigned *num_pins) {

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	*pins = axonfabric_pinctrl->pin_groups[group].pins;
	*num_pins = axonfabric_pinctrl->pin_groups[group].npins;
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

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_pinctrl->num_functions;
}

static const char *axonf_pinctrl_get_func_name(struct pinctrl_dev *pctldev,
			unsigned function) {

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	return axonfabric_pinctrl->functions[function].name;
}

static int axonf_pinctrl_get_func_groups(struct pinctrl_dev *pctldev,
		unsigned function, const char * const **groups,
		unsigned * const num_groups) {

	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);

	*groups = axonfabric_pinctrl->functions[function].groups;
	*num_groups = axonfabric_pinctrl->functions[function].ngroups;
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
	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_pinctrl->axonf_chip;
	u8 seldir;
	unsigned pin;
	int ret;

	AXONF_DBG_INFO(&chip->client->dev, "called for group %u\n", group);


	switch(axonfabric_pinctrl->functions[function].mux_option) {
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

	pin = axonfabric_pinctrl->pin_groups[group].pins[0];

	dev_dbg(&chip->client->dev, "%s(): Grpup %u (pin %u) to function %u and val %02X\n",
		__func__, group, pin, function, seldir);

	ret = axonf_reg_update_bits(chip, GPIO_TO_IOBANK_ADDR(pin),
			AXONF_IOB_MASK_SEL | AXONF_IOB_MASK_DIR | AXONF_IOB_MASK_ENABLE, seldir);
	if (ret < 0) {
		dev_err(&chip->client->dev, "Group %d update failed %d\n",
			group, ret);
		return ret;
	}
	//axonfabric_pinctrl->gpio_control[group].io_function = function;

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
	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_pinctrl->axonf_chip;
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
	struct axonfabric_pinctrl *axonfabric_pinctrl = pinctrl_dev_get_drvdata(pctldev);
	struct axonf_chip *chip = axonfabric_pinctrl->axonf_chip;
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

static int axonfabric_pinctrl_probe(struct platform_device *pdev)
{
	struct axonfabric_pinctrl *axonfabric_pinctrl;

	axonfabric_pinctrl = devm_kzalloc(&pdev->dev, sizeof(*axonfabric_pinctrl),
				     GFP_KERNEL);
	if (!axonfabric_pinctrl)
		return -ENOMEM;

	axonfabric_pinctrl->dev = &pdev->dev;
	axonfabric_pinctrl->dev->of_node = pdev->dev.parent->of_node;
	axonfabric_pinctrl->axonf_chip = dev_get_drvdata(pdev->dev.parent);
	platform_set_drvdata(pdev, axonfabric_pinctrl);

	axonfabric_pinctrl->pins = axonf_pins_desc;
	axonfabric_pinctrl->num_pins = ARRAY_SIZE(axonf_pins_desc);
	axonfabric_pinctrl->functions = axonf_pin_function;
	axonfabric_pinctrl->num_functions = ARRAY_SIZE(axonf_pin_function);
	axonfabric_pinctrl->pin_groups = axonf_pingroups;
	axonfabric_pinctrl->num_pin_groups = ARRAY_SIZE(axonf_pingroups);
	axonf_pinctrl_desc.name = dev_name(&pdev->dev);
	axonf_pinctrl_desc.pins = axonf_pins_desc;
	axonf_pinctrl_desc.npins = ARRAY_SIZE(axonf_pins_desc);
	axonfabric_pinctrl->pctl = devm_pinctrl_register(&pdev->dev, &axonf_pinctrl_desc,
					     axonfabric_pinctrl);
	if (IS_ERR(axonfabric_pinctrl->pctl)) {
		dev_err(&pdev->dev, "Couldn't register pinctrl driver\n");
		return PTR_ERR(axonfabric_pinctrl->pctl);
	}

	platform_set_drvdata(pdev, axonfabric_pinctrl);

	dev_info(&pdev->dev, "Probed.\n");

	return 0;
}

static int axonfabric_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id axonfabric_pinctrl_dt_match[] = {
	{ .compatible = "technexion,axonfabric-pinctrl" },
	{  }
};
MODULE_DEVICE_TABLE(of, axonfabric_pinctrl_dt_match);

static const struct platform_device_id axonfabric_pinctrl_id_table[] = {
	{ "axonfabric-pinctrl", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(platform, axonfabric_pinctrl_id_table);

static struct platform_driver axonfabric_pinctrl_driver = {
	.driver = {
		.name = "axonfabric-pinctrl",
		.of_match_table = of_match_ptr(axonfabric_pinctrl_dt_match)
	},
	.probe = axonfabric_pinctrl_probe,
	.remove = axonfabric_pinctrl_remove,
	.id_table = axonfabric_pinctrl_id_table,
};

module_platform_driver(axonfabric_pinctrl_driver);

MODULE_AUTHOR("john weber <john.weber@technexion.com>");
MODULE_DESCRIPTION("Pinctrl driver for Axon Fabric");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:axonfabric-pinctrl");
