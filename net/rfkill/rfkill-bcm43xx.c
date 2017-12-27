/*
 *
 * Copyright (C) 2016 Richard Hu <richard.hu@technexion.com>
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>

struct bcm43xx_rfkill_data {
	const char		*name;
	enum rfkill_type	type;
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*shutdown_gpio;

	struct rfkill		*rfkill_dev;
	char			*reset_name;
	char			*shutdown_name;
};

struct bcm43xx_rfkill_platform_data {
	char			*name;
	enum rfkill_type	type;
};

static int bcm43xx_rfkill_set_power(void *data, bool blocked)
{
	struct bcm43xx_rfkill_data *rfkill = data;


	if (blocked) {
		gpiod_set_value_cansleep(rfkill->shutdown_gpio, 0);
		gpiod_set_value_cansleep(rfkill->reset_gpio, 0);
	} else {
		gpiod_set_value_cansleep(rfkill->shutdown_gpio, 0);
		gpiod_set_value_cansleep(rfkill->reset_gpio, 0);
		msleep(15);
		gpiod_set_value_cansleep(rfkill->reset_gpio, 1);
	}

	return 0;
}

static const struct rfkill_ops bcm43xx_rfkill_ops = {
	.set_block = bcm43xx_rfkill_set_power,
};

static int bcm43xx_rfkill_get_pdata_from_of(struct device *dev,
		struct bcm43xx_rfkill_data *rfkill)
{
	struct device_node *np = dev->of_node;

	if (!np) {
		np = of_find_matching_node(NULL, dev->driver->of_match_table);
		if (!np) {
			dev_notice(dev, "device tree node not available\n");
			return -ENODEV;
		}
	}
	of_property_read_u32(np, "type", &rfkill->type);
	of_property_read_string(np, "name", &rfkill->name);
	return 0;
}

static int bcm43xx_rfkill_probe(struct platform_device *pdev)
{
	struct bcm43xx_rfkill_platform_data *pdata = pdev->dev.platform_data;
	struct bcm43xx_rfkill_data *rfkill;
	struct gpio_desc *gpio;
	int ret;
	int len;

	rfkill = devm_kzalloc(&pdev->dev, sizeof(*rfkill), GFP_KERNEL);
	if (!rfkill)
		return -ENOMEM;

	if (pdata) {
		rfkill->name = pdata->name;
		rfkill->type = pdata->type;
	} else {
		ret = bcm43xx_rfkill_get_pdata_from_of(&pdev->dev, rfkill);
		if (ret) {
			dev_err(&pdev->dev, "no platform data\n");
			return ret;
		}
	}

	len = strlen(rfkill->name);
	rfkill->reset_name = devm_kzalloc(&pdev->dev, len + 7, GFP_KERNEL);
	if (!rfkill->reset_name)
		return -ENOMEM;

	rfkill->shutdown_name = devm_kzalloc(&pdev->dev, len + 10, GFP_KERNEL);
	if (!rfkill->shutdown_name)
		return -ENOMEM;

	snprintf(rfkill->reset_name, len + 7 , "%s_reset", rfkill->name);
	snprintf(rfkill->shutdown_name, len + 10, "%s_shutdown", rfkill->name);

	gpio = devm_gpiod_get_index(&pdev->dev, rfkill->reset_name, 0, GPIOD_OUT_LOW);
	if (!IS_ERR(gpio)) {
		ret = gpiod_direction_output(gpio, 0);
		if (ret)
			return ret;
		rfkill->reset_gpio = gpio;
	}

	gpio = devm_gpiod_get_index(&pdev->dev, rfkill->shutdown_name, 1, GPIOD_OUT_LOW);
	if (!IS_ERR(gpio)) {
		ret = gpiod_direction_output(gpio, 0);
		if (ret)
			return ret;
		rfkill->shutdown_gpio = gpio;
	}

	/* Make sure at-least one of the GPIO is defined and that
	 * a name is specified for this instance
	 */
	if ((!rfkill->reset_gpio && !rfkill->shutdown_gpio) || !rfkill->name) {
		dev_err(&pdev->dev, "invalid platform data\n");
		return -EINVAL;
	}

	rfkill->rfkill_dev = rfkill_alloc(rfkill->name, &pdev->dev,
					  rfkill->type, &bcm43xx_rfkill_ops,
					  rfkill);
	if (!rfkill->rfkill_dev)
		return -ENOMEM;

	ret = rfkill_register(rfkill->rfkill_dev);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, rfkill);

	dev_info(&pdev->dev, "%s device registered.\n", rfkill->name);

	return 0;
}

static int bcm43xx_rfkill_remove(struct platform_device *pdev)
{
	struct bcm43xx_rfkill_data *rfkill = platform_get_drvdata(pdev);

	rfkill_unregister(rfkill->rfkill_dev);
	rfkill_destroy(rfkill->rfkill_dev);

	return 0;
}

static const struct of_device_id bcm43xx_rfkill_of_match_table[] = {
	{ .compatible = "net,rfkill-bcm43xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm43xx_rfkill_of_match_table);

static struct platform_driver bcm43xx_rfkill_driver = {
	.probe = bcm43xx_rfkill_probe,
	.remove = bcm43xx_rfkill_remove,
	.driver = {
		.name = "rfkill-bcm43xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(bcm43xx_rfkill_of_match_table),
	},
};

module_platform_driver(bcm43xx_rfkill_driver);

MODULE_AUTHOR("Richard Hu <richard.hu@technexion.com>");
MODULE_DESCRIPTION("bcm43xx rfkill driver");
MODULE_LICENSE("GPL v2");
