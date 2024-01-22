#ifndef __TEVS_I2C_H__
#define __TEVS_I2C_H__

#include <linux/i2c.h>

int tevs_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u8 size);
int tevs_i2c_read_16b(struct i2c_client *client, u16 reg, u16 *value);
int tevs_i2c_write_16b(struct i2c_client *client, u16 reg, u16 val);
int tevs_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len);

#endif
