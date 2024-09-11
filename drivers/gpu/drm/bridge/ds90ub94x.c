#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/fwnode.h>
#include <linux/delay.h>
#include <linux/regmap.h>
#include <linux/i2c-atr.h>

#define DS90UB941_I2C_ADDR 0x0c
#define DS90UB948_I2C_ADDR 0x2c

#define DETECT_SER_REGMAP ds90ub941->regmap
#define DETECT_DES_REGMAP ds90ub941->regmap
#define GENERAL_STS	0x0c
#define I2C_DEVICE_ID	0x00
#define DUAL_STS_DUAL_STS_P1	0x5a
#define LINK_DETECT	BIT(0)
#define FPD3_LINK_RDY	BIT(7)
#define FPD3_TX_STS	BIT(6)
#define DSI_CLK_DET	BIT(3)
#define FREQ_STABLE	BIT(0)
#define DSI_PANEL_RDY	(FPD3_LINK_RDY | FPD3_TX_STS | DSI_CLK_DET | FREQ_STABLE)

#define XNOR(x, y)	~(x ^ y)

#define RESET_MDELAY 50
#define I2C_RW_MDELAY 50

static const struct regmap_config config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

#define UB941_MAX_PORT_ALIASES 8

struct ds90ub94x {
	struct device *dev;
	struct regmap *regmap;
	struct i2c_config *reset;
	struct i2c_config *probe_info;
	struct i2c_config *dual_channel;
	int reset_size;
	int probe_info_size;
	int dual_channel_size;
	struct i2c_client *client;
	struct i2c_atr *atr;

	const struct i2c_client *aliased_clients[UB941_MAX_PORT_ALIASES];
};

struct i2c_config{
	unsigned char i2c_reg;
	unsigned char i2c_val;
	unsigned char check_mask;
};

struct i2c_config ds90ub941_reset[] = {
	{0x01, 0x0E, 0x00},
	{0x04, 0x20, 0x00}, //clear CRC_ERROR count
};

struct i2c_config ds90ub948_reset[] = {
	{0x01, 0x03, 0x00},
};

struct i2c_config ds90ub941_dual_channel[] = {
	{0x5B, 0x29, 0x08},
};

struct i2c_config ds90ub948_dual_channel[] = {
	{0x49, 0x60, 0x02},
};

struct i2c_config ds90ub941_probe_config[] = {
	{0x04, 0x00, 0x33}, //start CRC_ERROR count
	{0x03, 0x9A, 0xFA}, //Enable FPD-Link I2C pass through, DSI clock auto switch

	{0x5B, 0x20, 0xFF}, //FPD3_TX_MODE=auto_detect, Reset PLL
	{0x1E, 0x02, 0x07}, //Select FPD-Link III Port 0
	{0x0F, 0x09, 0x0F}, //PWR_SW_CTL_EN: local high to enable power of remote side

	{0x1E, 0x01, 0x07}, //Select FPD-Link III Port 0
	{0x5B, 0x21, 0xFF}, //FPD3_TX_MODE=single, Reset PLL
	{0x4F, 0x8C, 0xFE}, //DSI Continuous Clock Mode,DSI 4 lanes
	{0x0D, 0x03, 0x0F}, //GPIO0, MIPI_BL_EN
	{0x0E, 0x33, 0xFF}, //GPIO1, MIPI_VDDEN; GPIO2, MIPI_BL_PWM
	{0x0F, 0x03, 0x0F}, //Reset touch interrupt

	{0x01, 0x00, 0x0F}, //Release DSI/DIGITLE reset
	{0xC6, 0x21, 0xFF}, //REM_INTB the same as INTB_IN on UB948, Global Interrupt Enable 
};

struct i2c_config ds90ub948_probe_config[] = {
	{0x03, 0xF8, 0xFE}, //enable CRC, i2c pass through
	{0x49, 0x62, 0xE3}, //Set FPD_TX_MODE, MAPSEL=1(SPWG), Single OLDI output
	{0x34, 0x01, 0xFB}, //Select FPD-Link III Port 0
	{0x26, 0x19, 0xFF}, //SCL_HIGH_TIME: 1.5 us (50 ns * 0x19)
	{0x27, 0x19, 0xFF}, //SCL_LOW_TIME: 1.5 us (50 ns * 0x19)
	{0x1D, 0x15, 0x0F}, //GPIO0, MIPI_BL_EN
	{0x1E, 0x55, 0xFF}, //GPIO1, MIPI_VDDEN; GPIO2, MIPI_BL_PWM
	{0x1F, 0x05, 0x0F}, //Reset touch interrupt
};

#define ADAPTER_NUM 2
#define UB941_RR_SLAVE_ID(n)			n == 0? 0x07: (0x70 + (n - 1))
#define UB941_RR_SLAVE_ALIAS(n)			n == 0? 0x08: (0x77 + (n - 1))

static int ds90ub94x_atr_attach_client(struct i2c_atr *atr, u32 chan_id,
				   const struct i2c_client *client, u16 alias)
{
	struct ds90ub94x *priv = i2c_atr_get_driver_data(atr);
	struct device *dev = &priv->client->dev;
	struct i2c_client *dummy_client;
	unsigned int reg_idx;

	for (reg_idx = 0; reg_idx < ARRAY_SIZE(priv->aliased_clients); reg_idx++) {
		if (!priv->aliased_clients[reg_idx])
			break;
	}

	if (reg_idx == ARRAY_SIZE(priv->aliased_clients)) {
		dev_err(priv->dev, "alias pool exhausted\n");
		return -EADDRNOTAVAIL;
	}

	priv->aliased_clients[reg_idx] = client;

	regmap_write(priv->regmap, UB941_RR_SLAVE_ID(reg_idx), client->addr << 1);
	regmap_write(priv->regmap, UB941_RR_SLAVE_ALIAS(reg_idx), alias << 1);

	//Occupied the alias address
	dummy_client = devm_i2c_new_dummy_device(dev, priv->client->adapter, alias);
	dev_info(dev, "client 0x%02x assigned alias 0x%02x\n",
		client->addr, alias);

	return 0;
}

static void ds90ub94x_atr_detach_client(struct i2c_atr *atr, u32 chan_id,
				    const struct i2c_client *client)
{
	struct ds90ub94x *priv = i2c_atr_get_driver_data(atr);
	struct device *dev = &priv->client->dev;
	unsigned int reg_idx;

	dev_info(dev, "client 0x%02x\n",client->addr);

	for (reg_idx = 0; reg_idx < ARRAY_SIZE(priv->aliased_clients); reg_idx++) {
		if (priv->aliased_clients[reg_idx] == client)
			break;
	}

	if (reg_idx == ARRAY_SIZE(priv->aliased_clients)) {
		dev_err(priv->dev, "client 0x%02x is not mapped!\n", client->addr);
		return;
	}

	priv->aliased_clients[reg_idx] = NULL;

	regmap_write(priv->regmap, UB941_RR_SLAVE_ALIAS(reg_idx), 0);

	dev_dbg(dev, "client 0x%02x released at slot %u\n",
		client->addr, reg_idx);
}

static const struct i2c_atr_ops ds90ub94x_atr_ops = {
	.attach_client = ds90ub94x_atr_attach_client,
	.detach_client = ds90ub94x_atr_detach_client,
};

static int ds90ub94x_init_atr(struct ds90ub94x *priv)
{
	struct device *dev = &priv->client->dev;
	struct i2c_adapter *parent_adap = priv->client->adapter;

	priv->atr = i2c_atr_new(parent_adap, dev, &ds90ub94x_atr_ops,
                                ADAPTER_NUM);
	if (IS_ERR(priv->atr))
		return PTR_ERR(priv->atr);

	i2c_atr_set_driver_data(priv->atr, priv);

	return 0;
}

static int ds90ub94x_add_i2c_adapter(struct ds90ub94x *priv)
{
	struct device *dev = &priv->client->dev;
	struct fwnode_handle *i2c_handle;
	int ret;

	i2c_handle = device_get_named_child_node(dev, "i2c");
	if (!i2c_handle)
		return 0;

	ret = i2c_atr_add_adapter(priv->atr, 0,
				  dev, i2c_handle);

	fwnode_handle_put(i2c_handle);

	if (ret)
		return ret;

	return 0;
}

static int regmap_i2c_rw_check_retry(struct ds90ub94x *ds90ub94x, struct i2c_config *i2c_config, int i2c_config_size)
{
	int i;
	unsigned int reg, val;

	//continue write
	for ( i = 0; i < i2c_config_size ; i++ )
	{
		reg = i2c_config[i].i2c_reg;
		val = i2c_config[i].i2c_val;

		if (regmap_write(ds90ub94x->regmap, reg, val) < 0)
			return -EIO;
	}

	return 0;
}

static int ds90ub94x_init(struct ds90ub94x *ds90ub94x)
{
	int i;
	unsigned char reg, val;

	// reset reg can't read back the same val
	for ( i = 0 ; i < ds90ub94x->reset_size ; i++ )
	{
		reg = ds90ub94x->reset[i].i2c_reg;
		val = ds90ub94x->reset[i].i2c_val;

		if (regmap_write(ds90ub94x->regmap, reg, val) < 0)
			return -EIO;
	}

	msleep(RESET_MDELAY);
	if ( regmap_i2c_rw_check_retry(ds90ub94x, ds90ub94x->probe_info, ds90ub94x->probe_info_size) != 0 ) {
		dev_err(ds90ub94x->dev, "ds90ub94x init fail\n");
		return -EIO;
	}

	return 0;
}

static int ds90ub94x_probe(struct i2c_client *client)
{
	struct ds90ub94x *ds90ub941, *ds90ub948;
	struct i2c_client *dummy_client;
	struct gpio_desc *reset_gpio;
	int ret_ser, ret_des, i, crc_error_1, crc_error_2, val_read;
	int ret = 0;

	ds90ub941 = devm_kzalloc(&client->dev, sizeof(struct ds90ub94x), GFP_KERNEL);
	ds90ub948 = devm_kzalloc(&client->dev, sizeof(struct ds90ub94x), GFP_KERNEL);

	if ( !ds90ub941 || !ds90ub948 ) {
		ret = -ENOMEM;
		goto req_failed;
	}

	i2c_set_clientdata(client, ds90ub941);
	ds90ub941->client = client;
	//add new i2c_device for UB948
	dummy_client = devm_i2c_new_dummy_device(&client->dev, client->adapter, DS90UB948_I2C_ADDR);

	ds90ub941->dev = &client->dev;
	ds90ub941->regmap = devm_regmap_init_i2c(client, &config);
	ds90ub941->reset = ds90ub941_reset;
	ds90ub941->probe_info = ds90ub941_probe_config;
	ds90ub941->dual_channel = ds90ub941_dual_channel;
	ds90ub941->reset_size = ARRAY_SIZE(ds90ub941_reset);
	ds90ub941->probe_info_size = ARRAY_SIZE(ds90ub941_probe_config);
	ds90ub941->dual_channel_size = ARRAY_SIZE(ds90ub941_dual_channel);

	ds90ub948->dev = &dummy_client->dev;
	ds90ub948->regmap = devm_regmap_init_i2c(dummy_client, &config);
	ds90ub948->reset = ds90ub948_reset;
	ds90ub948->probe_info = ds90ub948_probe_config;
	ds90ub948->dual_channel = ds90ub948_dual_channel;
	ds90ub948->reset_size = ARRAY_SIZE(ds90ub948_reset);
	ds90ub948->probe_info_size = ARRAY_SIZE(ds90ub948_probe_config);
	ds90ub948->dual_channel_size = ARRAY_SIZE(ds90ub948_dual_channel);

	if (IS_ERR(ds90ub941->regmap)) {
		ret = PTR_ERR(ds90ub941->regmap);
		goto req_failed;
	}
	else if (IS_ERR(ds90ub948->regmap)) {
		ret = PTR_ERR(ds90ub948->regmap);
		goto req_failed;
	}

	reset_gpio = devm_gpiod_get_optional(&client->dev, "reset",
					     GPIOD_OUT_HIGH);

	if (! reset_gpio) {
		dev_info(ds90ub941->dev,"reset-gpio get failed\n");
		dev_info(ds90ub941->dev,"not using HW-PDB reset");
	} else {
		gpiod_direction_output(reset_gpio, 0);
		msleep(RESET_MDELAY);
		gpiod_direction_output(reset_gpio, 1);
		msleep(RESET_MDELAY);
		gpiod_direction_output(reset_gpio, 0);
		msleep(RESET_MDELAY);
	}

	//Start to probe
	ret = regmap_read(DETECT_SER_REGMAP, I2C_DEVICE_ID, &val_read);
	if (!ret)
		ret_ser = ds90ub94x_init(ds90ub941);
	else {
		dev_err(ds90ub941->dev, "failed to find ds90ub941(serializer) ...");
		ret= -ENXIO;
		goto connection_failed;
	}

	msleep(RESET_MDELAY);

	ret = regmap_read(DETECT_DES_REGMAP, GENERAL_STS, &val_read);
	if (( (val_read & LINK_DETECT ) == LINK_DETECT )) {
		// before we init the DS90UB948, check the DSI and pixel-clock is ready or not.
		// If not, defer probe.
		ret = regmap_read(DETECT_SER_REGMAP, DUAL_STS_DUAL_STS_P1, &val_read);
		val_read &= DSI_PANEL_RDY;

		if ( val_read == DSI_PANEL_RDY )
			ret_des = ds90ub94x_init(ds90ub948);
		else {
			dev_err(ds90ub941->dev, "wait for DSI and pixel clock ready, deferring...");
			goto err_out;
		}
	} else {
		dev_err(ds90ub948->dev, "failed to find ds90ub948(deserializer) ...");
		ret= -ENXIO;
		goto connection_failed;
	}

	if ( ret_ser || ret_des ){
		if ( ret_ser )
			dev_err(ds90ub941->dev,"=====> init failed, full_retry = %d times\n", i);
		if ( ret_des )
			dev_err(ds90ub948->dev,"=====> init failed, full_retry = %d times\n", i);
		goto err_out;
	}

	//read attribute to set dual lvds channel
        if (ds90ub941->dev->of_node) {
                struct device_node *dev_node = ds90ub941->dev->of_node;
                if (of_property_read_bool(dev_node, "vizionpanel-dual-lvds-channel")) {
			dev_info(ds90ub941->dev, "Using Dual channel lvds\n");

			if ( regmap_i2c_rw_check_retry(ds90ub941, ds90ub941->dual_channel, ds90ub941->dual_channel_size) < 0 ) {
				ret = -EIO;
				goto err_out;
			}
			msleep(I2C_RW_MDELAY);
			if ( regmap_i2c_rw_check_retry(ds90ub948, ds90ub948->dual_channel, ds90ub948->dual_channel_size) < 0 ) {
				ret = -EIO;
				goto err_out;
			}
                } else
			dev_info(ds90ub941->dev, "Using Single channel lvds\n");
        }

	ret = ds90ub94x_init_atr(ds90ub941);
	ret = ds90ub94x_add_i2c_adapter(ds90ub941);

	regmap_read(ds90ub941->regmap, 0x0A, &crc_error_1);
	regmap_read(ds90ub941->regmap, 0x0B, &crc_error_2);
	dev_info(ds90ub941->dev, "ds90ub94x probe success with CRC_ERROR_COUNT = 0x%02x%02x\n", crc_error_2, crc_error_1);
	return ret;

err_out:
	regmap_read(ds90ub941->regmap, 0x0A, &crc_error_1);
	regmap_read(ds90ub941->regmap, 0x0B, &crc_error_2);
	dev_err(ds90ub941->dev, "ds90ub94x probe failed with CRC_ERROR_COUNT = 0x%02x%02x\n", crc_error_2, crc_error_1);
	return -EPROBE_DEFER;

req_failed:
	dev_err(ds90ub941->dev, "request memery/regmap failed\n");
	return ret;

connection_failed:
	dev_err(ds90ub941->dev, "ds90ub94x failed connect to each other\n");
	return ret;
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

static int __init ds90ub94x_link_init(void)
{
	int err;

	err = i2c_add_driver(&ds90ub94x_driver);
	if (err < 0)
		return err;

	return 0;
}
module_init(ds90ub94x_link_init);

static void __exit ds90ub94x_link_remove(void)
{
	i2c_del_driver(&ds90ub94x_driver);
}

module_exit(ds90ub94x_link_remove);

MODULE_DESCRIPTION("TI. DS90UB941/DS90UB948 SerDer bridge");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(I2C_ATR);
