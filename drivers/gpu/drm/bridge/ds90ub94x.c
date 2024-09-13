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
#define WAIT_DSI_MDELAY 1000
#define MAX_WAIT_DSI_MDELAY 10000

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
	struct i2c_config *remote_gpio;
	struct i2c_config *dual_channel;
	struct i2c_client *client;
	struct i2c_atr *atr;
	int dsi_delay;

	const struct i2c_client *aliased_clients[UB941_MAX_PORT_ALIASES];

	struct workqueue_struct *check_and_enable_dsi_wq;
	struct delayed_work check_and_enable_dsi;
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

struct i2c_config ds90ub941_remote_gpio[] = {
	{0x0D, 0x03, 0x0F}, //GPIO0, MIPI_BL_EN
	{0x0E, 0x33, 0xFF}, //GPIO1, MIPI_VDDEN; GPIO2, MIPI_BL_PWM
	{0x0F, 0x03, 0x0F}, //Reset touch interrupt
};

struct i2c_config ds90ub948_remote_gpio[] = {
	{0x1D, 0x15, 0x0F}, //GPIO0, MIPI_BL_EN
	{0x1E, 0x55, 0xFF}, //GPIO1, MIPI_VDDEN; GPIO2, MIPI_BL_PWM
	{0x1F, 0x05, 0x0F}, //Reset touch interrupt
};

struct i2c_config ds90ub941_probe_config[] = {
	{0x01, 0x08, 0x0F}, //Disable DSI
	{0x4F, 0x0C, 0xFE}, //DSI Discontinuous Clock Mode

	{0x04, 0x00, 0x33}, //start CRC_ERROR count
	{0x03, 0x9A, 0xFA}, //Enable FPD-Link I2C pass through, DSI clock auto switch

	{0x5B, 0x20, 0xFF}, //FPD3_TX_MODE=auto_detect, Reset PLL
	{0x1E, 0x02, 0x07}, //Select FPD-Link III Port 0
	{0x0F, 0x09, 0x0F}, //PWR_SW_CTL_EN: local high to enable power of remote side

	{0x1E, 0x01, 0x07}, //Select FPD-Link III Port 0
	{0x5B, 0x21, 0xFF}, //FPD3_TX_MODE=single, Reset PLL

	{0xC6, 0x21, 0xFF}, //REM_INTB the same as INTB_IN on UB948, Global Interrupt Enable 
};

struct i2c_config ds90ub948_probe_config[] = {
	{0x03, 0xF8, 0xFE}, //enable CRC, i2c pass through
	{0x49, 0x62, 0xE3}, //Set FPD_TX_MODE, MAPSEL=1(SPWG), Single OLDI output
	{0x34, 0x01, 0xFB}, //Select FPD-Link III Port 0
	{0x26, 0x19, 0xFF}, //SCL_HIGH_TIME: 1.5 us (50 ns * 0x19)
	{0x27, 0x19, 0xFF}, //SCL_LOW_TIME: 1.5 us (50 ns * 0x19)
};

struct i2c_config ds90ub926_probe_config[] = {
	{0x03, 0xF8, 0xFE}, //enable CRC, i2c pass through
	{0x26, 0x19, 0xFF}, //SCL_HIGH_TIME: 1.5 us (50 ns * 0x19)
	{0x27, 0x19, 0xFF}, //SCL_LOW_TIME: 1.5 us (50 ns * 0x19)
};

struct i2c_config ds90ub941_lock_dsi[] = {
	{0x01, 0x08, 0x0F}, //Disable DSI
	{0x4F, 0x0C, 0xFE}, //DSI Discontinuous Clock Mode
};

struct i2c_config ds90ub941_unlock_dsi[] = {
	{0x4F, 0x8C, 0xFE}, //DSI Continuous Clock Mode,DSI 4 lanes
	{0x01, 0x00, 0x0F}, //Enable DSI
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

static int regmap_i2c_rw_check(struct ds90ub94x *ds90ub94x, struct i2c_config *i2c_config, int i2c_config_size)
{
	int i;
	unsigned int reg, val, mask, val_read;

	//continue write
	for ( i = 0; i < i2c_config_size ; i++ )
	{
		reg = i2c_config[i].i2c_reg;
		val = i2c_config[i].i2c_val;
		mask = i2c_config[i].check_mask;

		if (regmap_write(ds90ub94x->regmap, reg, val) < 0)
			return -EIO;
		if (regmap_read(ds90ub94x->regmap, reg, &val_read) < 0)
			return -EIO;
		if (mask != ((XNOR(val, val_read)) & mask)){
			dev_err(ds90ub94x->dev, "read check failed reg=0x%02x, val=0x%02x, val_read=0x%02x, mask=0x%02x\n",
									reg, val, val_read, mask);
			return -EIO;
		}
		msleep(I2C_RW_MDELAY);
	}

	return 0;
}

static void check_and_enable_dsi_workfn(struct work_struct *work)
{
	struct ds90ub94x *ds90ub941 = container_of(work, struct ds90ub94x, check_and_enable_dsi.work);
	unsigned int reg_val;
	int ret;

	dev_info(ds90ub941->dev, "check_and_enable_dsi_workfn start!!\n");
	/* Enable DSI, check if the signal is stable.
	 * If not, disable it and defer the check.	*/
	if ( regmap_i2c_rw_check(ds90ub941, ds90ub941_unlock_dsi, ARRAY_SIZE(ds90ub941_unlock_dsi)) < 0 ) {
		dev_err(ds90ub941->dev, "Failed to unlock DSI signal: %d\n", ret);
		goto lock_dsi_func;
	}

	ret = regmap_read(ds90ub941->regmap, DUAL_STS_DUAL_STS_P1, &reg_val);
	if (ret) {
		dev_err(ds90ub941->dev, "Failed to read DUAL_STS_DUAL_STS_P1 register\n");
		goto lock_dsi_func;
	}

	// Check if the register matches the expected condition
	if ((reg_val & DSI_PANEL_RDY) == DSI_PANEL_RDY) {
		dev_info(ds90ub941->dev, "Stable DSI signal, start display!!\n");
		return;
	} else {
		// Reschedule the work to run again
		queue_delayed_work(ds90ub941->check_and_enable_dsi_wq, &ds90ub941->check_and_enable_dsi, msecs_to_jiffies(ds90ub941->dsi_delay));
	}

lock_dsi_func:
	//Disable DSI
	if ( regmap_i2c_rw_check(ds90ub941, ds90ub941_lock_dsi, ARRAY_SIZE(ds90ub941_lock_dsi)) < 0 ) {
		dev_err(ds90ub941->dev, "Failed to lock DSI signal: %d\n", ret);
		return;
	}
}

static int ds90ub94x_probe(struct i2c_client *client)
{
	struct ds90ub94x *ds90ub941, *ds90ub948;
	struct i2c_client *dummy_client;
	struct gpio_desc *reset_gpio;
	struct device_node *dev_node;
	int ret_ser, ret_des, crc_error_1, crc_error_2, val_read, des_probe_config_size, dsi_delay_val;
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
	ds90ub941->remote_gpio = ds90ub941_remote_gpio;

	ds90ub948->dev = &dummy_client->dev;
	ds90ub948->regmap = devm_regmap_init_i2c(dummy_client, &config);
	ds90ub948->reset = ds90ub948_reset;
	ds90ub948->probe_info = ds90ub948_probe_config;
	ds90ub948->dual_channel = ds90ub948_dual_channel;
	ds90ub948->remote_gpio = ds90ub948_remote_gpio;
	des_probe_config_size = ARRAY_SIZE(ds90ub948_probe_config);

	if (IS_ERR(ds90ub941->regmap)) {
		ret = PTR_ERR(ds90ub941->regmap);
		goto req_failed;
	}
	else if (IS_ERR(ds90ub948->regmap)) {
		ret = PTR_ERR(ds90ub948->regmap);
		goto req_failed;
	}

	//read attribute to set dual lvds channel and/or remote gpio
	if (! ds90ub941->dev->of_node) {
		ret = PTR_ERR(ds90ub941->dev->of_node);
		goto req_failed;
	}
	dev_node = ds90ub941->dev->of_node;

	/* If not using remote-gpio, it means it's the vizionpanel-ttl.
	 * We need to change the probe_config to avoid non-exist register on DS90U926 */ 
	if (! of_property_read_bool(dev_node, "vizionpanel-remote-gpio")) {
		des_probe_config_size = ARRAY_SIZE(ds90ub926_probe_config);
		ds90ub948->probe_info = ds90ub926_probe_config;
	}

	// Read the 'dsi-delay' property
	ret = of_property_read_u32(dev_node, "dsi-delay", &dsi_delay_val);
	if (ret || dsi_delay_val > MAX_WAIT_DSI_MDELAY) {
		dev_err(ds90ub941->dev, "Failed to read dsi-delay property or it's larger than 10000 ms!!\n");
		dev_info(ds90ub941->dev, "Using default DSI delay time = %u ms\n", WAIT_DSI_MDELAY);
		ds90ub941->dsi_delay = WAIT_DSI_MDELAY;
	} else {
		dev_info(ds90ub941->dev, "DSI delay value: %u ms\n", dsi_delay_val);
		ds90ub941->dsi_delay = dsi_delay_val;
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

	/* check connection of DS90UB941 */
	ret = regmap_read(DETECT_SER_REGMAP, I2C_DEVICE_ID, &val_read);
	if (ret) {
		dev_err(ds90ub941->dev, "failed to find ds90ub941(serializer) ...");
		ret= -ENXIO;
		goto connection_failed;
	}

	/* Start to probe DS90UB941 */
	regmap_i2c_rw_check(ds90ub941, ds90ub941->reset, ARRAY_SIZE(ds90ub941_reset));
	msleep(RESET_MDELAY);
	ret_ser = regmap_i2c_rw_check(ds90ub941, ds90ub941->probe_info, ARRAY_SIZE(ds90ub941_probe_config));
	msleep(RESET_MDELAY);

	/* check connection of DS90UB948 */
	ret = regmap_read(DETECT_DES_REGMAP, GENERAL_STS, &val_read);
	if (( (val_read & LINK_DETECT ) != LINK_DETECT )) {
		dev_err(ds90ub948->dev, "failed to find ds90ub948(deserializer) ...");
		ret= -ENXIO;
		goto connection_failed;
	}

	/* Start to probe DS90UB948 */
	regmap_i2c_rw_check(ds90ub948, ds90ub948->reset, ARRAY_SIZE(ds90ub948_reset));
	msleep(RESET_MDELAY);
	ret_des = regmap_i2c_rw_check(ds90ub948, ds90ub948->probe_info, des_probe_config_size); //there are 2 sku of probe_info

	if ( ret_ser || ret_des ){
		if ( ret_ser )
			dev_err(ds90ub941->dev,"=====> init failed\n");
		if ( ret_des )
			dev_err(ds90ub948->dev,"=====> init failed\n");
		goto err_out;
	}

	if (of_property_read_bool(dev_node, "vizionpanel-dual-lvds-channel")) {
		dev_info(ds90ub941->dev, "Using Dual channel lvds\n");

		if ( regmap_i2c_rw_check(ds90ub941, ds90ub941->dual_channel, ARRAY_SIZE(ds90ub941_dual_channel)) < 0 ) {
			ret = -EIO;
			goto err_out;
		}
		msleep(I2C_RW_MDELAY);
		if ( regmap_i2c_rw_check(ds90ub948, ds90ub948->dual_channel, ARRAY_SIZE(ds90ub948_dual_channel)) < 0 ) {
			ret = -EIO;
			goto err_out;
		}
	} else
		dev_info(ds90ub941->dev, "Using Single channel lvds\n");

	if (of_property_read_bool(dev_node, "vizionpanel-remote-gpio")) {
		dev_info(ds90ub941->dev, "Using remote gpio for DSI control\n");

		if ( regmap_i2c_rw_check(ds90ub941, ds90ub941->remote_gpio, ARRAY_SIZE(ds90ub941_remote_gpio)) < 0 ) {
			ret = -EIO;
			goto err_out;
		}
		msleep(I2C_RW_MDELAY);
		if ( regmap_i2c_rw_check(ds90ub948, ds90ub948->remote_gpio, ARRAY_SIZE(ds90ub948_remote_gpio)) < 0 ) {
			ret = -EIO;
			goto err_out;
		}
	} else
		dev_info(ds90ub941->dev, "Using other gpio for DSI control(ex: io-expander)\n");

	ret = ds90ub94x_init_atr(ds90ub941);
	ret = ds90ub94x_add_i2c_adapter(ds90ub941);

	// Create workqueue
	ds90ub941->check_and_enable_dsi_wq = create_singlethread_workqueue("check_and_enable_dsi_wq");
	if (!ds90ub941->check_and_enable_dsi_wq) {
		dev_err(ds90ub941->dev, "Failed to create workqueue\n");
		return -ENOMEM;
	}

	// Initialize delayed work
	INIT_DELAYED_WORK(&ds90ub941->check_and_enable_dsi, check_and_enable_dsi_workfn);

	// Schedule the work to run immediately
	queue_delayed_work(ds90ub941->check_and_enable_dsi_wq, &ds90ub941->check_and_enable_dsi, msecs_to_jiffies(ds90ub941->dsi_delay));
	dev_info(ds90ub941->dev, "Probe complete, register check work scheduled\n");

	regmap_read(ds90ub941->regmap, 0x0A, &crc_error_1);
	regmap_read(ds90ub941->regmap, 0x0B, &crc_error_2);
	dev_info(ds90ub941->dev, "ds90ub94x probe success with CRC_ERROR_COUNT = 0x%02x%02x\n", crc_error_2, crc_error_1);
	return ret;

err_out:
	regmap_read(ds90ub941->regmap, 0x0A, &crc_error_1);
	regmap_read(ds90ub941->regmap, 0x0B, &crc_error_2);
	dev_err(ds90ub941->dev, "ds90ub94x probe failed with CRC_ERROR_COUNT = 0x%02x%02x\n", crc_error_2, crc_error_1);
	return ret;

req_failed:
	dev_err(ds90ub941->dev, "request memery/regmap failed\n");
	return ret;

connection_failed:
	dev_err(ds90ub941->dev, "ds90ub94x failed connect to each other\n");
	return ret;
}

void ds90ub94x_remove(struct i2c_client *client)
{
    struct ds90ub94x *ds90ub94x = i2c_get_clientdata(client);

    // Cancel the delayed work and destroy the workqueue
    cancel_delayed_work_sync(&ds90ub94x->check_and_enable_dsi);
    destroy_workqueue(ds90ub94x->check_and_enable_dsi_wq);
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
