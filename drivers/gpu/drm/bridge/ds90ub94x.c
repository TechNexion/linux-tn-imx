#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/regmap.h>

#define DS90UB941_I2C_ADDR 0x0c
#define DS90UB948_I2C_ADDR 0x2c

static const struct regmap_config config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

struct ds90ub94x {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_config *probe_info;
	int i2c_config_size;
};

struct i2c_config{
	unsigned char i2c_reg;
	unsigned char i2c_val;
};

struct i2c_config ds90ub941_probe_config[] = {
	{0x01, 0x0F}, //Reset DSI/DIGITLE

	{0x03, 0xBA}, //Enable FPD-Link I2C pass through
	{0x1E, 0x01}, //Select FPD-Link III Port 0
	{0x5B, 0x21}, //FPD3_TX_MODE=single, Reset PLL
	{0x4F, 0x8C}, //DSI Continuous Clock Mode,DSI 4 lanes

	{0x40, 0x04},
	{0x41, 0x05},
	{0x42, 0x14},
	{0x01, 0x00}, //Release DSI/DIGITLE reset

	{0x07, 0x54}, //SlaveID_0: touch panel
	{0x08, 0x54}, //SlaveAlias_0: touch panel

	{0x30, 0x01}, //REM_INTB_CTRL: port 0 remote interrupt
};

struct i2c_config ds90ub948_probe_config[] = {
	{0x01, 0x01}, //ds90ub948 reset
	{0x49, 0x62}, //Set FPD_TX_MODE, MAPSEL=1(SPWG), Single OLDI output
	{0x34, 0x02}, //Select FPD-Link III Port 0, GPIOx instead of D_GPIOx
	{0x1D, 0x19}, //GPIO0, MIPI_BL_EN
	{0x1E, 0x99}, //GPIO1, MIPI_VDDEN; GPIO2, MIPI_BL_PWM

	{0x34, 0x02}, //Reset touch interrupt
	{0x1F, 0x09},

	{0x08, 0x54}, //SlaveID_0: touch panel
	{0x10, 0x54}, //SlaveAlias_0: touch panel

	{0xC6, 0x21}, //INTB: enable INTB_IN on remote DES
};

static int ds90ub94x_init(struct ds90ub94x *ds90ub94x)
{
	int i;
	for ( i = 0 ; i < ds90ub94x->i2c_config_size ; i++)
	{
		unsigned char reg = ds90ub94x->probe_info[i].i2c_reg;
		unsigned char val = ds90ub94x->probe_info[i].i2c_val;
		dev_dbg(ds90ub94x->dev, "ds90ub94x %s() write num %d: reg 0x%x, value 0x%x\n",
				__func__, i, reg, val);

		if (regmap_write(ds90ub94x->regmap, reg, val) < 0){
			dev_err(ds90ub94x->dev, " regmap_write fail reg 0x%x, value 0x%x\n", reg, val);
			return -EIO;
		}
		msleep(10);
	}
	return 0;
}

static int ds90ub94x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ds90ub94x *ds90ub941, *ds90ub948;
	struct i2c_client *dummy_client;
	int ret;

	ds90ub941 = devm_kzalloc(&client->dev, sizeof(struct ds90ub94x), GFP_KERNEL);
	ds90ub948 = devm_kzalloc(&client->dev, sizeof(struct ds90ub94x), GFP_KERNEL);

	if ( !ds90ub941 || !ds90ub948 )
		return -ENOMEM;

	i2c_set_clientdata(client, ds90ub941);
	//add new i2c_device for UB948
	dummy_client = i2c_new_dummy_device(client->adapter, DS90UB948_I2C_ADDR);

	ds90ub941->dev = &client->dev;
	ds90ub941->regmap = devm_regmap_init_i2c(client, &config);
	ds90ub941->probe_info = ds90ub941_probe_config;
	ds90ub941->i2c_config_size = ARRAY_SIZE(ds90ub941_probe_config);

	ds90ub948->dev = &dummy_client->dev;
	ds90ub948->regmap = devm_regmap_init_i2c(dummy_client, &config);
	ds90ub948->probe_info = ds90ub948_probe_config;
	ds90ub948->i2c_config_size = ARRAY_SIZE(ds90ub948_probe_config);

	if (IS_ERR(ds90ub941->regmap))
		return PTR_ERR(ds90ub941->regmap);
	else if (IS_ERR(ds90ub948->regmap))
		return PTR_ERR(ds90ub948->regmap);

	ret = ds90ub94x_init(ds90ub941);
	if ( ret < 0 )
		return ret;

	ret = ds90ub94x_init(ds90ub948);
	if ( ret < 0 )
		return ret;

	//read attribute to set dual lvds channel
        if (ds90ub941->dev->of_node) {
                struct device_node *dev_node = ds90ub941->dev->of_node;
                if (of_property_read_bool(dev_node, "vizionpanel-dual-lvds-channel")) {
			dev_info(ds90ub941->dev, "Using Dual channel lvds\n");
			regmap_update_bits(ds90ub941->regmap, 0x5B, BIT(3), 1);
			regmap_update_bits(ds90ub948->regmap, 0x49, BIT(1), 0);
                } else
			dev_info(ds90ub941->dev, "Using Single channel lvds\n");
        }

	dev_info(ds90ub941->dev, "ds90ub94x probe success\n");

	return 0;
}

void ds90ub94x_remove(struct i2c_client *client)
{

}

static const struct of_device_id ds90ub94x_of_match[] = {
	{
		.compatible = "ti,ds90ub94x"
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ds90ub94x_of_match);

static struct i2c_driver ds90ub94x_driver = {
	.driver = {
		.name = "ds90ub94x",
		.of_match_table = ds90ub94x_of_match,
	},
	.probe = ds90ub94x_probe,
	.remove = ds90ub94x_remove,
};
module_i2c_driver(ds90ub94x_driver);

MODULE_DESCRIPTION("TI. DS90UB941/DS90UB948 SerDer bridge");
MODULE_LICENSE("GPL");
