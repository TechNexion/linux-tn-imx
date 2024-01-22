#ifndef __SENSOR_I2C_H__
#define __SENSOR_I2C_H__

#include <linux/i2c.h>

int sensor_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u8 size);
int sensor_i2c_read_16b(struct i2c_client *client, u16 reg, u16 *value);
int sensor_i2c_write_16b(struct i2c_client *client, u16 reg, u16 val);
int sensor_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len);

#endif
