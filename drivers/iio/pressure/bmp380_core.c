#include <linux/acpi.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/log2.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/delay.h>
#include "bmp380.h"

struct bmp3_reg_calib_data {
	uint16_t	par_t1;
	uint16_t	par_t2;
	int8_t		unused;	// fill the alignment hole
	int8_t		par_t3;
	int16_t		par_p1;
	int16_t		par_p2;
	int8_t		par_p3;
	int8_t		par_p4;
	uint16_t	par_p5;
	uint16_t	par_p6;
	int8_t		par_p7;
	int8_t		par_p8;
	int16_t		par_p9;
	int8_t		par_p10;
	int8_t		par_p11;
	int16_t		t_lin;
};

struct bmp380_data {
	struct regmap *regmap;
	struct bmp3_reg_calib_data reg_calib_data;
	u8 op_mode;
	u8 press_en;
	u8 temp_en;
	u8 oversampling_temp;
	u8 oversampling_press;
	s32 t_fine;
};

static const char *bmp380_match_acpi_device(struct device *dev)
{
	const struct acpi_device_id *id;

	id = acpi_match_device(dev->driver->acpi_match_table, dev);
	if (!id)
		return NULL;

	return dev_name(dev);
}

const struct regmap_config bmp380_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

EXPORT_SYMBOL(bmp380_regmap_config);

int bmp3_set_op_mode(struct regmap *regmap, struct bmp380_data *data)
{
	int rslt;

	rslt = regmap_write(regmap, BMP3_PWR_CTRL_ADDR, BMP3_CMD_PARSE);

	if (rslt < 0)
		return -ENOMEM;

	return rslt;
}

static int bmp380_read_press(struct bmp380_data *data, int *val, int *val2)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret = 0;
	uint8_t reg_data[3] = {0};
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	uint64_t comp_press;
	uint32_t uncomp_data;

	int64_t partial_out1;
	int64_t partial_out2;

	ret = bmp3_set_op_mode(data->regmap, data);
	if (ret < 0)
		return ret;

	usleep_range(5500, 6000);

	ret = regmap_bulk_read(data->regmap, BMP3_PRESS_ADDR, reg_data, 3);
	if (ret < 0) {
		dev_err(dev, "failed to read pressure\n");
		return ret;
	}

	uncomp_data = ((uint32_t)reg_data[0]) | (((uint32_t)reg_data[1]) << 8) | (((uint32_t)reg_data[2]) << 16);

//	dev_info(dev, "Praw: B2 - B0, combined: %u, %u, %u %u\n", reg_data[2], reg_data[1], reg_data[0], uncomp_data);

	partial_data1 = data->reg_calib_data.t_lin * data->reg_calib_data.t_lin;
	partial_data3 = partial_data1 * data->reg_calib_data.t_lin;

/*
	dev_info(dev, "Praw, T-lin: %u, %i PS Param: T: %u, %u, %i\n", uncomp_data, data->reg_calib_data.t_lin, data->reg_calib_data.par_t1, 
				data->reg_calib_data.par_t2, data->reg_calib_data.par_t3);
	dev_info(dev, "PS Param: P01-P06: %i, %i, %i, %i, %u, %u\n", data->reg_calib_data.par_p1, data->reg_calib_data.par_p2, 
				data->reg_calib_data.par_p3, data->reg_calib_data.par_p4, data->reg_calib_data.par_p5, data->reg_calib_data.par_p6);
	dev_info(dev, "PS Param: P07-P11: %i, %i, %i, %i, %i, %i\n", data->reg_calib_data.par_p7, data->reg_calib_data.par_p8, 
				data->reg_calib_data.par_p9, data->reg_calib_data.par_p10, data->reg_calib_data.par_p11, data->reg_calib_data.t_lin);
*/

	partial_data6 = (((int64_t)data->reg_calib_data.par_p6) * ((int64_t)data->reg_calib_data.t_lin)) << 9;
	partial_data5 = (((int64_t)data->reg_calib_data.par_p7) * partial_data1) << 7;
	partial_data4 = ((int64_t)data->reg_calib_data.par_p8) * partial_data3;
	partial_out1 = (((int64_t)(data->reg_calib_data.par_p5)) << 18);
	partial_out1 = partial_out1 + partial_data4 + partial_data5 + partial_data6;
/*
	dev_info(dev, "Intermediate Results 1: pd1, pd2, pd3, Partial_out1: %lld, %lld, %lld, %lld\n", partial_data4, partial_data5, partial_data6, partial_out1);
*/
	partial_data5 = (((int64_t)data->reg_calib_data.par_p2 - 16384) * ((int64_t)data->reg_calib_data.t_lin)) << 8;
	partial_data4 = (((int64_t)data->reg_calib_data.par_p3) * partial_data1) << 5;
	partial_data2 = ((int64_t)data->reg_calib_data.par_p4) * partial_data3;
	partial_out2 = ((int64_t)(data->reg_calib_data.par_p1 - 16384)) << 17;
	partial_out2 = partial_out2 + partial_data2 + partial_data4 + partial_data5;
	partial_out2 = partial_out2 * ((int64_t)uncomp_data);
/*
	dev_info(dev, "Intermediate Results 2: pd1, pd2, pd3, Partial_out2: %lld, %lld, %lld, %lld\n", partial_data5, partial_data4, partial_data2, partial_out2);
*/

	partial_data2 = ((int64_t)data->reg_calib_data.par_p10) * ((int64_t)data->reg_calib_data.t_lin);
	partial_data3 = partial_data2 + ((int64_t)data->reg_calib_data.par_p9);
	partial_data4 = partial_data3 * uncomp_data;
	partial_data5 = partial_data4 * uncomp_data;
	partial_data1 = ((int64_t)uncomp_data) * ((int64_t)uncomp_data);
	partial_data1 = partial_data1 >> 9;
	partial_data1 = partial_data1 * ((int64_t)uncomp_data);
	partial_data2 = partial_data1 >> 8;
	partial_data3 = (((int64_t)(data->reg_calib_data.par_p11)) * partial_data2);
	partial_data4 = partial_data5 + partial_data3;
	partial_data4 = partial_data4 >> 11;

//	dev_info(dev, "Intermediate Results 3: pd5, pd1, pd2, pd3: %lld, %lld, %lld, %lld\n", partial_data5, partial_data1, partial_data2, partial_data3);

	/* Intermediate Result */
//	dev_info(dev, "Intermediate Results: Partial_out1, Partial_out2, Partial_data4: %lld, %lld, %lld\n", partial_out1, partial_out2, partial_data4);
	partial_data4 = partial_data4 + partial_out2;
	partial_data4 = partial_data4 >> 22;
	partial_data4 = partial_data4 + partial_out1;
	comp_press = (((uint64_t)partial_data4 * 25) >> 13);

	if (val && val2) {
		*val = comp_press;
		*val2 = 100;
		return IIO_VAL_FRACTIONAL;
	}

	return ret;
}

static int bmp380_read_temp(struct bmp380_data *data, int *val, int *val2)
{
	struct device *dev = regmap_get_device(data->regmap);
	int32_t uncomp_data;
	int ret = 0;
	uint8_t reg_data[3] = {0};
	int64_t partial_data1;
	int64_t partial_data2;
	int64_t partial_data3;
	int64_t partial_data4;
	int64_t partial_data5;
	int64_t partial_data6;
	int64_t comp_temp;

	ret = bmp3_set_op_mode(data->regmap, data);
	if (ret < 0)
		return ret;

	usleep_range(5500, 6000);

	ret = regmap_bulk_read(data->regmap, BMP3_TEMPE_ADDR, reg_data, 3);
	if (ret < 0) {
		dev_err(dev, "failed to read temperature\n");
		return ret;
	}

	uncomp_data = (int32_t)((((uint32_t)reg_data[0]) | (((uint32_t)reg_data[1]) << 8)) | (uint32_t)((int32_t)reg_data[2] << 16));

	partial_data1 = uncomp_data - (((int32_t)data->reg_calib_data.par_t1) << 8);
	partial_data2 = (data->reg_calib_data.par_t2) * partial_data1;
	partial_data3 = partial_data1 * partial_data1;
	partial_data4 = partial_data3 * data->reg_calib_data.par_t3;
	partial_data5 = ((partial_data2 << 18) + partial_data4);
	partial_data6 = partial_data5 >> 32;
//	dev_info(dev, "Intermediate Results: T-Lin: %i, %lld, %lld, %lld, %lld\n", uncomp_data, partial_data2, partial_data4, partial_data5, partial_data6);
	data->reg_calib_data.t_lin = (int16_t)(partial_data6 >> 16);
	comp_temp = (partial_data6 * 25) >> 14;

	if (val && val2) {
		*val = comp_temp;
		*val2 = 100;
		return IIO_VAL_FRACTIONAL;
	}

	return ret;
}

static const int bmp380_oversampling_avail[] = { 1, 2, 4, 8, 16 };

static int bmp380_write_oversampling_ratio_temp(struct bmp380_data *data,
									int val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmp380_oversampling_avail); i++) {
		if (bmp380_oversampling_avail[i] == val) {
			data->oversampling_temp = ilog2(val);

			return 0;
		}
	}

	return -EINVAL;
}

static int bmp380_write_oversampling_ratio_press(struct bmp380_data *data,
									int val)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmp380_oversampling_avail); i++) {
		if (bmp380_oversampling_avail[i] == val) {
			data->oversampling_press = ilog2(val);

			return 0;
		}
	}

	return -EINVAL;
}

static int bmp380_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int val, int val2, long mask)
{
	struct bmp380_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (chan->type) {
		case IIO_TEMP:
			return bmp380_write_oversampling_ratio_temp(data, val);
		case IIO_PRESSURE:
			return bmp380_write_oversampling_ratio_press(data, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static int bmp380_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan, int *val, int *val2, long mask)
{
	struct bmp380_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (chan->type) {
		case IIO_TEMP:
			return bmp380_read_temp(data, val, val2);
		case IIO_PRESSURE:
			return bmp380_read_press(data, val, val2);
		default:
			return -EINVAL;
		}
	}

	return -EINVAL;
}

static const struct iio_chan_spec bmp380_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
	},
	{
		.type = IIO_PRESSURE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
				      BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),
	},
};

static const struct iio_info bmp380_info = {
	.read_raw = &bmp380_read_raw,
	.write_raw = &bmp380_write_raw,
};

static void bmp380_core_remove(void *arg)
{
	iio_device_unregister(arg);
}

int8_t bmp3_soft_reset(struct regmap *regmap)
{
	int  rslt;
	unsigned int cmd_rdy_status;
	unsigned int cmd_err_status;

	regmap_read(regmap, BMP3_SENS_STATUS_REG_ADDR, &cmd_rdy_status);

		if (cmd_rdy_status & BMP3_CMD_RDY) {
			rslt = regmap_write(regmap, BMP3_CMD_ADDR, BMP380_CMD_SOFTRESET);
			if (rslt > 0) {
				msleep(2);
				rslt = regmap_read(regmap, BMP3_ERR_REG_ADDR, &cmd_err_status);
				if ((cmd_err_status & BMP3_CMD_ERR) || (rslt < 0)) {
					rslt = BMP3_E_CMD_EXEC_FAILED;
				}
			}
		} else {
			rslt = BMP3_E_CMD_EXEC_FAILED;
		}

	return rslt;
}

static void parse_calib_data(const uint8_t *reg_data, struct bmp380_data *data)
{
	struct bmp3_reg_calib_data *reg_calib_data = &data->reg_calib_data;

	reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
	reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
	reg_calib_data->par_t3 = (int8_t)reg_data[4];
	reg_calib_data->par_p1 = (int16_t)BMP3_CONCAT_BYTES(reg_data[6], reg_data[5]);
	reg_calib_data->par_p2 = (int16_t)BMP3_CONCAT_BYTES(reg_data[8], reg_data[7]);
	reg_calib_data->par_p3 = (int8_t)reg_data[9];
	reg_calib_data->par_p4 = (int8_t)reg_data[10];
	reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
	reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14],  reg_data[13]);
	reg_calib_data->par_p7 = (int8_t)reg_data[15];
	reg_calib_data->par_p8 = (int8_t)reg_data[16];
	reg_calib_data->par_p9 = (int16_t)BMP3_CONCAT_BYTES(reg_data[18], reg_data[17]);
	reg_calib_data->par_p10 = (int8_t)reg_data[19];
	reg_calib_data->par_p11 = (int8_t)reg_data[20];
}

static int8_t get_calib_data(struct regmap *regmap, struct bmp380_data *data)
{
	int rslt;
	uint8_t calib_data[BMP3_CALIB_DATA_LEN] = {0};

	rslt = regmap_raw_read(regmap, BMP3_CALIB_DATA_ADDR, calib_data, BMP3_CALIB_DATA_LEN);

	if (rslt < 0) {
		return rslt;
	}
	parse_calib_data(calib_data, data);

	return rslt;
}

int8_t bmp3_init(struct regmap *regmap, struct bmp380_data *data)
{
	int rslt;

	rslt = bmp3_soft_reset(regmap);

		if (!rslt){ 
			rslt = get_calib_data(regmap, data);
		}
		else{
			rslt = BMP3_E_DEV_NOT_FOUND;
		}

	return rslt;
}

static int set_pwr_ctrl_settings(struct regmap *regmap, uint32_t desired_settings, const struct bmp380_data *data)
{
	int rslt;
	unsigned int reg_data;

	rslt = regmap_read(regmap, BMP3_PWR_CTRL_ADDR, &reg_data);

	if (rslt < 0)
		return -ENOMEM;

	if (desired_settings & BMP3_PRESS_EN_SEL)
		reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN, data->press_en);

	if (desired_settings & BMP3_TEMP_EN_SEL)
		reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, data->temp_en);

	rslt = regmap_write(regmap, BMP3_PWR_CTRL_ADDR, reg_data);

	if (rslt < 0)
		return -ENOMEM;

	return rslt;
}

int bmp380_probe(struct device *dev, struct regmap *regmap, unsigned int chip, const char *name, int irq)
{
	struct iio_dev *indio_dev;
	struct bmp380_data *data;
	uint16_t settings_sel;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	ret = devm_add_action(dev, bmp380_core_remove, indio_dev);
	if (ret < 0) {
		dev_err(dev, "failed to register remove action\n");
		return ret;
	}

	if (!name && ACPI_HANDLE(dev))
		name = bmp380_match_acpi_device(dev);

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;
	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->channels = bmp380_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmp380_channels);
	indio_dev->info = &bmp380_info;

	bmp3_init(regmap, data);
	data->press_en = BMP3_ENABLE;
	data->temp_en = BMP3_ENABLE;
	settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL;
	set_pwr_ctrl_settings(regmap, settings_sel, data);

	data->op_mode = BMP3_FORCED_MODE;
	ret = bmp3_set_op_mode(regmap, data);

	if (ret < 0) {
		return ret;
	}

	return iio_device_register(indio_dev);
}
EXPORT_SYMBOL(bmp380_probe);

MODULE_AUTHOR("Alvin Chen <alvin.chen@technexion.com>");
MODULE_DESCRIPTION("Driver for Bosch Sensortec BMP380 pressure and temperature sensor");
MODULE_LICENSE("GPL v2");
