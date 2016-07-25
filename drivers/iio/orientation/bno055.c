/*
 * Copyright (c) 2016 Intel Corporation
 *
 * Driver for Bosch Sensortec BNO055 digital motion sensor.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define pr_fmt(fmt) "bno055: " fmt

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>


#define BNO055_CHIP_ID		0xA0
#define REG_MAG_RADIUS_MSB	0x6A

/* BNO055 configuration registers */
#define REG_PWR_MODE		0x3E
#define REG_OPR_MODE		0x3D
#define REG_UNIT_SEL		0x3B
#define REG_AXIS_MAP_SIGN	0x42
#define REG_AXIS_MAP_CONFIG	0x41
#define REG_UNIT_SEL		0x3B
#define REG_SYS_TRIGGER		0x3F
#define BNO055_REG_PAGE_ID	0x07
#define BNO055_REG_ID		0x00

/* BNO055 status registers */
#define SYS_STATUS		0x39
#define INT_STATUS		0x37
#define CALIB_STATUS	0x35

/* BNO055 data registers */

#define GRV_DATA_Z_MSB 0x33
#define GRV_DATA_Z_LSB 0x32
#define GRV_DATA_Y_MSB 0x31
#define GRV_DATA_Y_LSB 0x30
#define GRV_DATA_X_MSB 0x2F
#define GRV_DATA_X_LSB 0x2E

#define LIA_DATA_Z_MSB 0x2D
#define LIA_DATA_Z_LSB 0x2C
#define LIA_DATA_Y_MSB 0x2B
#define LIA_DATA_Y_LSB 0x2A
#define LIA_DATA_X_MSB 0x29
#define LIA_DATA_X_LSB 0x28

#define QUA_DATA_Z_MSB 0x27
#define QUA_DATA_Z_LSB 0x26
#define QUA_DATA_Y_MSB 0x25
#define QUA_DATA_Y_LSB 0x24
#define QUA_DATA_X_MSB 0x23
#define QUA_DATA_X_LSB 0x22
#define QUA_DATA_W_MSB 0x21
#define QUA_DATA_W_LSB 0x20

#define EUL_PITCH_MSB 0x1F
#define EUL_PITCH_LSB 0x1E
#define EUL_ROLL_MSB 0x1D
#define EUL_ROLL_LSB 0x1C
#define EUL_HEADING_MSB 0x1B
#define EUL_HEADING_LSB 0x1A

#define GYR_DATA_Z_MSB 0x19
#define GYR_DATA_Z_LSB 0x18
#define GYR_DATA_Y_MSB 0x17
#define GYR_DATA_Y_LSB 0x16
#define GYR_DATA_X_MSB 0x15
#define GYR_DATA_X_LSB 0x14

#define MAG_DATA_Z_MSB 0x13
#define MAG_DATA_Z_LSB 0x12
#define MAG_DATA_Y_MSB 0x11
#define MAG_DATA_Y_LSB 0x10
#define MAG_DATA_X_MSB 0x0F
#define MAG_DATA_X_LSB 0x0E

#define ACC_DATA_Z_MSB 0x0D
#define ACC_DATA_Z_LSB 0x0C
#define ACC_DATA_Y_MSB 0x0B
#define ACC_DATA_Y_LSB 0x0A
#define ACC_DATA_X_MSB 0x09
#define ACC_DATA_X_LSB 0x08

/* operation modes */
#define FUSION_NDOF_MODE 0x0C

/* power modes */
#define NORMAL_MODE 0x00
#define LOW_POWER_MODE BIT(0)
#define SUSPEND_MODE BIT(1)

#define PROPER_CALIBRATION 0xFF

#define BASE_REG  0x08
#define SENSOR_AXIS_TO_REG(sensor,axis)	(BASE_REG + (sensor * 6) + (axis * 2))

#define BNO055_CHANNEL(_type, _axis) {					\
	.type = IIO_##_type,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.scan_index = AXIS_##_axis,					\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,					\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
}

struct bno055_data {
	struct i2c_client *client;
	struct mutex lock;
	struct regmap *regmap;
};

enum bno055_axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
};

enum {ACC=0, MAG=1, GYR=2, EUL=3, QUA=4};

static const struct iio_chan_spec bno055_channels[] = {
	BNO055_CHANNEL(ANGL_VEL, X),
	BNO055_CHANNEL(ANGL_VEL, Y),
	BNO055_CHANNEL(ANGL_VEL, Z),
	BNO055_CHANNEL(MAGN, X),
	BNO055_CHANNEL(MAGN, Y),
	BNO055_CHANNEL(MAGN, Z),
	BNO055_CHANNEL(ACCEL, X),
	BNO055_CHANNEL(ACCEL, Y),
	BNO055_CHANNEL(ACCEL, Z),
	{
		.type = IIO_ROT,
		.modified = 1,
		.channel2 = IIO_MOD_QUATERNION,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static bool bno055_is_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_PWR_MODE:
	case REG_OPR_MODE:
	case REG_UNIT_SEL:
	case REG_AXIS_MAP_SIGN:
	case REG_AXIS_MAP_CONFIG:
	case REG_SYS_TRIGGER:
		return true;
	default:
		return false;
	};
}

static bool bno055_is_volatile_reg(struct device *dev, unsigned int reg)
{
	if ((reg >= ACC_DATA_X_LSB) && (reg <= QUA_DATA_Z_MSB))
		return true;
	else
		return false;
}

static const struct regmap_config bno055_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = REG_MAG_RADIUS_MSB,
	.cache_type = REGCACHE_RBTREE,

	.writeable_reg = bno055_is_writeable_reg,
	.volatile_reg = bno055_is_volatile_reg,
};


static int read_bno055(struct bno055_data *data, int sensor, int axis, int *val)
{
	int ret;
	__le16 raw_val;
	int reg;

	reg = SENSOR_AXIS_TO_REG(sensor, axis);

	ret = regmap_bulk_read(data->regmap, reg, &raw_val,
			       sizeof(raw_val));
	if (ret < 0) {
		dev_err(&data->client->dev, "Error reading axis %d\n", axis);
		return ret;
	}

	*val = sign_extend32(le16_to_cpu(raw_val), 15);
	return IIO_VAL_INT;
}

/* Read raw quaternion values W,X,Y and Z.
 * Raw values needs to be divied by the 16384 to get the exact quaternion values.
 */

static int read_rot(struct bno055_data *data, int *vals, int *val_len)
{
	int ret, i;
	__le16 raw_val[4];

	ret = regmap_bulk_read(data->regmap, QUA_DATA_W_LSB, &raw_val,
			       sizeof(raw_val));
	if (ret < 0) {
		dev_err(&data->client->dev, "Error reading Orientation \n");
		return ret;
	}

	for (i = 0; i < 4; ++i)
		vals[i] = sign_extend32(le16_to_cpu(raw_val[i]), 15);

	*val_len =  4;
	return IIO_VAL_INT_MULTIPLE;
}

static int bno055_read_raw_multi(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int size,
			   int *val, int *val2, long mask)
{
	int ret = -EINVAL;
	struct bno055_data *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_ROT:
			ret = read_rot(data, val, val2);
			break;
		case IIO_ANGL_VEL:
			/* Gyroscope unit degrees per second */
			/* Raw value needs to be divied by the 16 to get the exact value.*/
			ret = read_bno055(data, GYR, chan->scan_index, val);
			break;
		case IIO_ACCEL:
			/* Accelerometer Unit m/s2 */
			/* Raw value needs to be divied by the 100 to get the exact value.*/
			ret = read_bno055(data, ACC, chan->scan_index, val);
			break;
		case IIO_MAGN:
			/* Magnetometer Unit microTesla */
			/* Raw value needs to be divied by the 16 to get the exact value. */
			ret = read_bno055(data, MAG, chan->scan_index, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;

	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&data->lock);

	return ret;
}

static const struct iio_info bno055_info = {
	.driver_module = THIS_MODULE,
	.read_raw_multi = &bno055_read_raw_multi,
};


static int bno055_chip_init(struct bno055_data *data)
{
	int ret;

	ret = regmap_write(data->regmap, REG_PWR_MODE,
				 NORMAL_MODE);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"failed to write power mode register\n");
		return ret;
	}

	ret = regmap_write(data->regmap, REG_OPR_MODE,
				 FUSION_NDOF_MODE);
	if (ret < 0)
		dev_err(&data->client->dev,
			"failed to write operation mode register\n");

	return ret;
}

static ssize_t show_calibration_status(struct device *dev, struct device_attribute *attr,
                char *buf){
	struct bno055_data *data;
	int ret;
	unsigned int calib_stat;

	data = dev->driver_data;

	ret = regmap_read(data->regmap, CALIB_STATUS, &calib_stat);

	if (ret){
		dev_err(dev, "Failed to read calibration status.\n");
		return ret;
	}

	if (calib_stat != PROPER_CALIBRATION)
		dev_info(dev, "bad calibration.  expected %x got %x\n",
			PROPER_CALIBRATION, calib_stat);

	return scnprintf(buf, PAGE_SIZE, "0x%x\n", calib_stat);
}

static DEVICE_ATTR(calib_status, 0444, show_calibration_status, NULL);

static int bno055_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *indio_dev;
	struct bno055_data *data;
	unsigned int chip_id;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	mutex_init(&data->lock);
	data->client = client;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->channels = bno055_channels;
	indio_dev->num_channels = ARRAY_SIZE(bno055_channels);
	indio_dev->info = &bno055_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	data->regmap = devm_regmap_init_i2c(client, &bno055_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(&client->dev, "Failed to allocate register map.\n");
		return PTR_ERR(data->regmap);
	}

	indio_dev->dev.driver_data = data;
	i2c_set_clientdata(client, indio_dev);

	ret = regmap_read(data->regmap, BNO055_REG_ID, &chip_id);
	if (ret < 0)
		return ret;
	if (chip_id != BNO055_CHIP_ID) {
		dev_err(&client->dev, "bad chip id.  expected %x got %x\n",
			BNO055_CHIP_ID, chip_id);
		return -EINVAL;
	}

	ret = bno055_chip_init(data);
	if (ret < 0)
		return ret;

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret){
		dev_err(&client->dev, "failed to register IIO device.\n");
		return ret;
	}

	ret = device_create_file(&indio_dev->dev, &dev_attr_calib_status);
	if (ret){
		dev_err(&client->dev, "failed to create sysfs entry.\n");
		devm_iio_device_unregister(&client->dev, indio_dev);
		return ret;
	}

   return ret;
}

static int bno055_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);

	device_remove_file(&indio_dev->dev, &dev_attr_calib_status);
	devm_iio_device_unregister(&client->dev, indio_dev);
	return 0;
}

static const struct acpi_device_id bno055_acpi_match[] = {
	{"bno055", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, bno055_acpi_match);

static const struct i2c_device_id bno055_id[] = {
	{"bno055", 0},
	{ },
};
MODULE_DEVICE_TABLE(i2c, bno055_id);

static struct i2c_driver bno055_driver = {
	.driver = {
		.name	= "bno055",
		.acpi_match_table = ACPI_PTR(bno055_acpi_match),
	},
	.probe		= bno055_probe,
	.remove		= bno055_remove,
	.id_table	= bno055_id,
};
module_i2c_driver(bno055_driver);

MODULE_AUTHOR("navin patidar <navin.patidar@gmail.com>");
MODULE_DESCRIPTION("Driver for Bosch Sensortec BNO055 orientation sensor");
MODULE_LICENSE("GPL v2");
