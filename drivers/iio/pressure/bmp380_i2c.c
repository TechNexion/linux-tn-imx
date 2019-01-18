#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "bmp380.h"

static int bmp380_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct regmap *regmap;
	const char *name = NULL;
	unsigned int val;
	int ret;

	regmap = devm_regmap_init_i2c(client, &bmp380_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap %d\n",
				(int)PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	ret = regmap_write(regmap, BMP3_CMD_ADDR, BMP380_CMD_SOFTRESET);
	if (ret < 0)
		return ret;

	ret = regmap_read(regmap, BMP3_CHIP_ID_ADDR, &val);
	if (ret < 0) {
		dev_err(&client->dev, "Error reading I2C chip ID\n");
		return ret;
	}

	if (val != BMP3_CHIP_ID) {
		dev_err(&client->dev, "Wrong chip ID, got %x expected %x\n",
				val, BMP3_CHIP_ID);
		return -ENODEV;
	}

	if (id)
		name = id->name;

	return bmp380_probe(&client->dev, regmap, id->driver_data, name, client->irq);
}

static const struct acpi_device_id bmp380_acpi_i2c_match[] = {
	{"bmp380", BMP3_CHIP_ID_ADDR },
	{ },
};
MODULE_DEVICE_TABLE(acpi, bmp380_acpi_i2c_match);

static const struct i2c_device_id bmp380_i2c_id[] = {
	{"bmp380", BMP3_CHIP_ID_ADDR },
	{ },
};
MODULE_DEVICE_TABLE(i2c, bmp380_i2c_id);

static struct i2c_driver bmp380_i2c_driver = {
	.driver = {
		.name	= "bmp380",
		.acpi_match_table = ACPI_PTR(bmp380_acpi_i2c_match),
	},
	.probe		= bmp380_i2c_probe,
	.id_table	= bmp380_i2c_id,
};
module_i2c_driver(bmp380_i2c_driver);

MODULE_AUTHOR("Alvin Chen <alvin.chen@technexion.com>");
MODULE_DESCRIPTION("Driver for Bosch Sensortec BMP380 pressure and temperature sensor");
MODULE_LICENSE("GPL v2");
