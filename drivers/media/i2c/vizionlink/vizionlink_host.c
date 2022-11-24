#include <linux/of.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>

#include "vizionlink_cam.h"

struct vh_st
{
	struct i2c_client *i2c_client;
	struct gpio_desc *pdb_gpio;
	u8 ser_alias_addr;
	u8 connect_port;
};

struct __reg8b_v {
	u8 reg;
	u8 value;
};

static struct __reg8b_v __9096x_i2c_setting [] = {
	{.reg = 0x02, .value = 0x1E}, //GENERAL_CFG
	{.reg = 0x07, .value = 0xFE}, //BCC Watchdog Control
	{.reg = 0x08, .value = 0x1C}, //I2C Control 1
	{.reg = 0x09, .value = 0x10}, //I2C Control 2
	{.reg = 0x0A, .value = 0x7A}, //SCL High Time
	{.reg = 0x0B, .value = 0x7A}, //SCL Low Time
	{.reg = 0x0D, .value = 0xB9}, //IO_CTL

	{.reg = 0x32, .value = 0x01}, //CSI_PORT_SEL
	{.reg = 0x33, .value = 0x20}, //CSI_CTL
	{.reg = 0x1F, .value = 0x00}, //CSI_PLL_CTL

	//Port config/setting
	{.reg = 0x4C, .value = 0x01}, //FPD3_PORT_SEL,Port 0
	{.reg = 0x58, .value = 0x5E}, //BCC_CONFIG
	{.reg = 0x6D, .value = 0x7C}, //PORT_CONFIG

	{.reg = 0x4C, .value = 0x12}, //FPD3_PORT_SEL,Port 1
	{.reg = 0x58, .value = 0x5E}, //BCC_CONFIG
	{.reg = 0x6D, .value = 0x7C}, //PORT_CONFIG

	{.reg = 0x4C, .value = 0x24}, //FPD3_PORT_SEL,Port 2
	{.reg = 0x58, .value = 0x5E}, //BCC_CONFIG
	{.reg = 0x6D, .value = 0x7C}, //PORT_CONFIG

	{.reg = 0x4C, .value = 0x38}, //FPD3_PORT_SEL,Port 3
	{.reg = 0x58, .value = 0x5E}, //BCC_CONFIG
	{.reg = 0x6D, .value = 0x7C}, //PORT_CONFIG
	{.reg = 0x7C, .value = 0x20}, //
	{.reg = 0xB0, .value = 0x1C}, //IND_ACC_CTL
};

static int __i2c_read(struct i2c_client *client, u8 reg, u8 *val, u8 size)
{
	dev_dbg(&client->dev, "read reg 0x%x, size %d\n", reg, size);

	if (1 != i2c_master_send(client, &reg, 1))
		goto __except;
	if (size != i2c_master_recv(client, val, size))
		goto __except;

	return 0;

__except:
	dev_err(&client->dev, "failed to i2c read. reg 0x%x\n", reg);
	return -EIO;
}

static int __i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	u8 sb[2];

	dev_dbg(&client->dev, "write reg 0x%x, val 0x%x\n", reg, val);

	sb[0] = reg;
	sb[1] = val;

	if (2 != i2c_master_send(client, sb, 2))
		goto __except;

	return 0;

__except:
	dev_err(&client->dev,
		"failed to i2c write. reg 0x%x, val 0x%x\n", reg, val);
	return -EIO;
}

static int vh_is_exist(struct vh_st *this)
{
	u8 rev_mask_id = 0;
	struct device *dev = &this->i2c_client->dev;

	if ( ! IS_ERR(this->pdb_gpio) ) {
		dev_dbg(dev, "trigger pdb pin\n");

		if (gpiod_cansleep(this->pdb_gpio)) {
			gpiod_set_value_cansleep(this->pdb_gpio, 0);
			msleep(10);
			gpiod_set_value_cansleep(this->pdb_gpio, 1);
			msleep(50);
		} else {
			gpiod_set_value(this->pdb_gpio, 0);
			msleep(10);
			gpiod_set_value(this->pdb_gpio, 1);
			msleep(50);
		}
	}

	if (0 != __i2c_read(this->i2c_client, 3, &rev_mask_id, 1))
		return -EINVAL;

	dev_dbg(dev, "rev mask id of deserializer chip is 0x%x\n", rev_mask_id);
	return 0;
}

static int vh_init(struct vh_st *this)
{
	struct device *dev = &this->i2c_client->dev;
	int i;
	int err;

	//9096x Digital Reset
	err = __i2c_write(this->i2c_client, 1, 3);
	if (err != 0)
		return -EIO;
	msleep(20);

	for (i = 0 ; i < ARRAY_SIZE(__9096x_i2c_setting) ; i++) {
		err = __i2c_write(this->i2c_client,
				  __9096x_i2c_setting[i].reg,
				  __9096x_i2c_setting[i].value);
		if (err != 0)
			return -EIO;
	}

	dev_dbg(dev, "write reg count [%d]\n", i);
	return 0;
}

static int vh_check_port_lock_pass(struct vh_st *this)
{
	struct device *dev = &this->i2c_client->dev;
	int i;
	u8 v = 0;
	int ret = -1;

	for (i = 0 ; i < 4 ; i++) {
		__i2c_write(this->i2c_client, 0x4c, (i << 4) | (1 << i));
		__i2c_read(this->i2c_client, 0x4d, &v, 1);
		if((v & 0xC0) >> 6 != i){
			return ret;
		}
		if ((v & 0x7) == 3) {
			__i2c_read(this->i2c_client, 0x5b, &v, 1);
			__i2c_write(this->i2c_client, 0x5c,
				    this->ser_alias_addr << 1);
			this->connect_port = i;
			dev_info(dev,
				 "Port %d pass and lock.Serializer addr 0x%x, alias to 0x%x\n",
				 this->connect_port,
				 v >> 1,
				 this->ser_alias_addr);
			ret = 0;
		} else {
			dev_dbg(dev, "Port %d disconnect\n", i);
		}
	}

	return ret;
}

static int vh_polling_connect(struct vh_st *this)
{
	int state_flag = 0;
	u8 v = 0;
	int retry_time = 100;

	while (state_flag != -1 && retry_time != 0) {
		msleep(50);

		switch (state_flag) {
		case 0:
			__i2c_read(this->i2c_client, 4, &v, 1);
			if ((v & (1 << 6)) == (1 << 6) &&
			    (v & (1 << 4)) == (1 << 4)) {
				state_flag = 1;
			}
			retry_time --;
			break;
		case 1:
			if (vh_check_port_lock_pass(this) == 0) {
				__i2c_write(this->i2c_client,
					    0x4c,
					    (this->connect_port << 4) |
					    (1 << this->connect_port));
				state_flag = -1;
			} else {
				retry_time --;
			}
			break;
		}
	}

	if (retry_time == 0) {
		return -EINVAL;
	} else {
		return 0;
	}
}

static int vh_register_i2c_alias_addr(struct vh_st *this, u8 index,
				      u8 client_addr, u8 alias_addr)
{
	if ( index > 8)
		return -EINVAL;

	__i2c_write(this->i2c_client, 0x5D + index, client_addr << 1);
	__i2c_write(this->i2c_client, 0x65 + index, alias_addr << 1);

	return 0;
}

static int vh_setting_i2c_alias_map(struct vh_st *this)
{
	struct device *dev = &this->i2c_client->dev;
	struct property *local;
	struct property *remote;
	int ret;
	int index;
	u32 local_addr;
	u32 remote_addr;

	local = of_find_property(dev->of_node,
				 "i2c_addr_alias_map_local",
				 &ret);
	remote = of_find_property(dev->of_node,
				  "i2c_addr_alias_map_remote",
				  &ret);

	if (local == NULL || remote == NULL) {
		dev_err(dev, "find not property of alias map\n");
		return -EINVAL;
	}

	index = 0;
	ret = 0;
	while(ret == 0 && index < 8) {
		ret = of_property_read_u32_index(dev->of_node,
						 "i2c_addr_alias_map_local",
						 index,
						 &local_addr);
		if (ret != 0 || local_addr > 0x7f)
			break;

		ret = of_property_read_u32_index(dev->of_node,
						 "i2c_addr_alias_map_remote",
						 index,
						 &remote_addr);
		if (ret != 0 || remote_addr > 0x7f)
			break;

		ret = vh_register_i2c_alias_addr(this,
						 index,
						 (u8)(remote_addr & 0x7f),
						 (u8)(local_addr & 0x7f));
		if (ret != 0)
			break;

		dev_dbg(dev, "i2c address alias "
			"index: %d local: 0x%x remote: 0x%x\n",
			index, local_addr, remote_addr);
		index ++;
	}

	return 0;
}

static int vh_append_i2c_device(struct vh_st *this)
{
	struct device *dev = &this->i2c_client->dev;
	struct device_node *i2c_dev_list;
	struct device_node *i2c_dev;
	struct i2c_client *client;
	struct i2c_board_info info;

	i2c_dev_list = of_get_child_by_name(dev->of_node, "i2c_dev_list");
	if(!i2c_dev_list) {
		dev_dbg(dev, "find not i2c device list in device tree\n");
		return 0;
	}

	for_each_available_child_of_node(i2c_dev_list, i2c_dev) {
		if (of_i2c_get_board_info(dev, i2c_dev, &info) != 0) {
			dev_err(dev, "Failed to get board info %pOF\n", i2c_dev);
			continue;
		}

		client = i2c_new_client_device(this->i2c_client->adapter, &info);
		if (IS_ERR(client)) {
			dev_err(dev, "Failed to registering %pOF\n", i2c_dev);
		}
	}

	return 0;
}

static int vh_configure_des_csi(struct vh_st *this)
{
	struct device *dev = &this->i2c_client->dev;
	u32 temp1;
	u8 temp2;
	u8 des_csi_lanes;
	u8 des_csi_continuous_clock;
	int ret;

	temp1 = 0xff;
	ret = of_property_read_u32(dev->of_node, "des_csi_lanes", &temp1);
	if (ret == 0) {
		if (temp1 > 0 && temp1 < 5) {
			dev_dbg(dev, "deserializer csi lanes: %d\n", temp1);
			des_csi_lanes = (u8)(4 - temp1);
		} else {
			dev_err(dev,
				"value of 'des_csi_lanes' is invaild.\n");
			return -EINVAL;
		}
	} else {
		dev_dbg(dev,
			 "Failed to get property of device node 'des_csi_lanes'."
			 "use default 4 lanes\n");
		des_csi_lanes = 0;
	}

	temp1 = 2;
	ret = of_property_read_u32(dev->of_node,
				   "des_csi_continuous_clock",
				   &temp1);
	if (ret == 0) {
		if ((temp1 == 0 || temp1 == 1)) {
			dev_dbg(dev,
				"csi continuous clock of deserializer: %d\n",
				temp1);
			des_csi_continuous_clock = temp1;
		} else {
			dev_err(dev,
				"value of 'des_csi_continuous_clock' is invaild.");
			return -EINVAL;
		}
	} else {
		dev_dbg(dev,
			"Failed to get property of device node 'des_csi_continuous_clock'."
			"csi continuous clock of deserializer: 0\n");
		des_csi_continuous_clock = 0;
	}

	//0x33 //Set CSI-2 Transmit enable (and Continuous clock if desired) in CSI_CTL register
	__i2c_read(this->i2c_client, 0x33, &temp2, 1);
	temp2 &= ~(0x3 << 4);
	temp2 |= (des_csi_lanes << 4);
	temp2 &= ~(0x1 << 1);
	temp2 |= (des_csi_continuous_clock << 1);
	temp2 |= 1;
	__i2c_write(this->i2c_client, 0x33, temp2);
	//0x34 //Enable CSI-2 Periodic Calibration (if desired) in the CSI_CTL2 register
	//0x20 //Enable Forwarding for assigned ports in the FWD_CTL1 register
	__i2c_write(this->i2c_client, 0x20, (0xf0 & ~(1 << (this->connect_port + 4))));

	return 0;
}

static int vh_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct vh_st *obj = NULL;
	u32 temp;
	int error;
	int timeout;

	dev_info(&client->dev, "%s\n", client->dev.of_node->full_name);

	obj = devm_kzalloc(&client->dev, sizeof(struct vh_st), GFP_KERNEL);
	if (obj == NULL) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}
	obj->i2c_client = client;

	if (of_property_read_u32(client->dev.of_node,
				 "ser_alias_id",
				 &temp) != 0) {
		dev_err(&client->dev,
			"Failed to get property of device node 'ser_alias_id'\n");
		return -EINVAL;
	}
	obj->ser_alias_addr = temp & 0xff;
	dev_dbg(&client->dev,
		"Serializer alias id is 0x%x\n", obj->ser_alias_addr);

	obj->pdb_gpio = devm_gpiod_get(&client->dev, "pdb", GPIOD_OUT_LOW);
	if (IS_ERR(obj->pdb_gpio)) {
		error = PTR_ERR(obj->pdb_gpio);
		if (error != EPROBE_DEFER)
			dev_err(&client->dev,
			"Failed to get gpio pin setting 'pdb'\n");

		return error;
	}

	if (vh_is_exist(obj) != 0) {
		dev_err(&client->dev,
			"Failed to find vizionlink-host board\n");
		return -EINVAL;
	}

	if (vh_init(obj) != 0) {
		dev_err(&client->dev,
			"Failed to initial vizionlink-host board\n");
		return -EINVAL;
	}

	if (vh_polling_connect(obj) != 0) {
		dev_err(&client->dev,
			"Failed to detect that vizionlink-cam board is connected on any port\n");
		return -EINVAL;
	}
	for (timeout = 0 ; timeout < 500 ; timeout ++) {
		usleep_range(9000, 10000);
		if (vc_init(client, obj->ser_alias_addr) == 0) {
			break;
		}
	}
	if (timeout >= 500 ) {
		dev_err(&client->dev, "Failed to initial vizionlink-cam board\n");
		return -EINVAL;
	}

	if (vh_setting_i2c_alias_map(obj) != 0) {
		dev_err(&client->dev, "Failed to setting i2c alias map\n");
		return -EINVAL;
	}

	if (vh_append_i2c_device(obj) != 0) {
		dev_err(&client->dev, "Failed to append i2c device\n");
		return -EINVAL;
	}

	for (timeout = 0 ; timeout < 500 ; timeout ++) {
		usleep_range(9000, 10000);
		if (vc_configure_ser_csi(client, obj->ser_alias_addr) == 0) {
			break;
		}
	}
	if (timeout >= 500 ) {
		dev_err(&client->dev, "Failed to configure serializer csi\n");
		return -EINVAL;
	}

	if (vh_configure_des_csi(obj) != 0) {
		dev_err(&client->dev, "Failed to configure deserializer csi\n");
		return -EINVAL;
	}

	return 0;
}

void vh_remove(struct i2c_client *client)
{

}

static const struct i2c_device_id vh_id[] = {
	{ "vizionlink", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, vh_id);

static const struct of_device_id vh_of[] = {
	{ .compatible = "tn,vizionlink" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, vh_of);

static struct i2c_driver vh_driver = {
	.driver = {
		.of_match_table = of_match_ptr(vh_of),
		.name  = "vizionlink",
	},
	.probe = vh_probe,
	.remove = vh_remove,
	.id_table = vh_id,
};

module_i2c_driver(vh_driver);

MODULE_AUTHOR("TechNexion Inc.");
MODULE_DESCRIPTION("TechNexion vizionlink driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("SERDES");

