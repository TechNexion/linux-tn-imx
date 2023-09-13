#include "tevs_i2c.h"

int tevs_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u8 size)
{
	struct i2c_msg msg[2];
	u8 buf[2];

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	msg[0].addr = client->addr;
	msg[0].flags = client->flags;
	msg[0].buf = buf;
	msg[0].len = sizeof(buf);

	msg[1].addr = client->addr;
	msg[1].flags = client->flags | I2C_M_RD;
	msg[1].buf = val;
	msg[1].len = size;

	return i2c_transfer(client->adapter, msg, 2);
}

int tevs_i2c_read_16b(struct i2c_client *client, u16 reg, u16 *value)
{
	u8 v[2] = { 0 };
	int ret;

	ret = tevs_i2c_read(client, reg, v, 2);

	if (unlikely(ret < 0)) {
		dev_err(&client->dev, "i2c transfer error.\n");
		return ret;
	}

	*value = (v[0] << 8) | v[1];
	dev_dbg(&client->dev, "%s() read reg 0x%x, value 0x%x\n", __func__, reg,
		*value);

	return 0;
}

int tevs_i2c_write_16b(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int retry_tmp = 0;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	while ((i2c_transfer(client->adapter, &msg, 1)) < 0) {
		retry_tmp++;
		dev_err(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write 16b reg:%x val:%x.\n", reg, val);

		if (retry_tmp > 50) {
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}

int tevs_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len)
{
	struct i2c_msg msg;
	int retry_tmp = 0;

	if (len == 0) {
		return 0;
	}

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = len;

	while ((i2c_transfer(client->adapter, &msg, 1)) < 0) {
		retry_tmp++;
		dev_err(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write bust buf:%x.\n", client->addr);

		if (retry_tmp > 50) {
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}