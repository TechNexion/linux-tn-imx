#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "sensor_tbls.h"
#include "otp_flash.h"

struct sensor {
	struct v4l2_subdev v4l2_subdev;
	struct media_pad pad;
	struct v4l2_mbus_framefmt fmt;
	struct i2c_client *i2c_client;
	struct otp_flash *otp_flash_instance;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *host_power_gpio;
	struct gpio_desc *device_power_gpio;
	struct gpio_desc *standby_gpio;
	u8 selected_mode;
	u8 selected_sensor;
	bool supports_over_4k_res;
	char *sensor_name;
};

static int sensor_standby(struct i2c_client *client, int enable);

static int sensor_i2c_read(struct i2c_client *client, u16 reg, u8 *val, u8 size)
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

static int sensor_i2c_read_16b(struct i2c_client *client, u16 reg, u16 *value)
{
	u8 v[2] = {0,0};
	int ret;

	ret = sensor_i2c_read(client, reg, v, 2);

	if (unlikely(ret < 0)) {
		dev_err(&client->dev, "i2c transfer error.\n");
		return ret;
	}

	*value = (v[0] << 8) | v[1];
	dev_dbg(&client->dev, "%s() read reg 0x%x, value 0x%x\n",
		 __func__, reg, *value);

	return 0;
}

static int sensor_i2c_write_16b(struct i2c_client *client, u16 reg, u16 val)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
	{
		dev_err(&client->dev, "i2c transfer error.\n");
		return -EIO;
	}

	return 0;
}

static int sensor_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len)
{
	struct i2c_msg msg;
	int ret;

	if (len == 0) {
		return 0;
	}

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = len;

	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
	{
		dev_err(&client->dev, "i2c transfer error.\n");
		return -EIO;
	}

	return 0;
}

static int ops_power(struct v4l2_subdev *sub_dev, int on)
{
	//struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, on);
	return 0;
}

static int ops_init(struct v4l2_subdev *sub_dev, u32 val)
{
	//struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, val);
	return 0;
}

static int ops_load_fw(struct v4l2_subdev *sub_dev)
{
	//struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s()\n", __func__);
	return 0;
}

static int ops_reset(struct v4l2_subdev *sub_dev, u32 val)
{
	//struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s() [%d]\n", __func__, val);
	return 0;
}

static int ops_get_frame_interval(struct v4l2_subdev *sub_dev,
				  struct v4l2_subdev_frame_interval *fi)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int ops_set_frame_interval(struct v4l2_subdev *sub_dev,
				  struct v4l2_subdev_frame_interval *fi)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (fi->pad != 0)
		return -EINVAL;

	fi->interval.numerator = 1;
	fi->interval.denominator = 30;

	return 0;
}

static int ops_set_stream(struct v4l2_subdev *sub_dev, int enable)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int ret = 0;

	dev_dbg(sub_dev->dev, "%s() enable [%x]\n", __func__, enable);

	if (instance->selected_mode >= ap1302_sensor_table[instance->selected_sensor].res_list_size)
		return -EINVAL;

	if (enable == 0) {
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
		//VIDEO_WIDTH
		sensor_i2c_write_16b(instance->i2c_client, 0x2000, 1920);
		//VIDEO_HEIGHT
		sensor_i2c_write_16b(instance->i2c_client, 0x2002, 1080);
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
		ret = sensor_standby(instance->i2c_client, 1);
	} else {
		ret = sensor_standby(instance->i2c_client, 0);
		if (ret == 0) {
			int fps = ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].framerates;
			dev_dbg(sub_dev->dev, "%s() width=%d, height=%d\n", __func__, 
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width, 
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
			//VIDEO_WIDTH
			sensor_i2c_write_16b(instance->i2c_client, 0x2000,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width);
			//VIDEO_HEIGHT
			sensor_i2c_write_16b(instance->i2c_client, 0x2002,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			sensor_i2c_write_16b(instance->i2c_client, 0x2020, fps << 8); //VIDEO_MAX_FPS
			sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
		}
	}

	return ret;
}

static int ops_enum_mbus_code(struct v4l2_subdev *sub_dev,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_mbus_code_enum *code)
{
	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_2X8;

	return 0;
}

static int ops_get_fmt(struct v4l2_subdev *sub_dev,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sub_dev,
						 sd_state,
						 format->pad);
	else
		fmt = &instance->fmt;

	memmove(mbus_fmt, fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int ops_set_fmt(struct v4l2_subdev *sub_dev,
		       struct v4l2_subdev_state *sd_state,
		       struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *fmt;
	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int i;

	dev_dbg(sub_dev->dev, "%s()\n", __func__);

	if (format->pad != 0)
		return -EINVAL;

	for(i = 0 ; i < ap1302_sensor_table[instance->selected_sensor].res_list_size ; i++)
	{
		if (mbus_fmt->width == ap1302_sensor_table[instance->selected_sensor].res_list[i].width &&
				mbus_fmt->height == ap1302_sensor_table[instance->selected_sensor].res_list[i].height)
			break;
	}

	if (i >= ap1302_sensor_table[instance->selected_sensor].res_list_size)
	{
		return -EINVAL;
	}
	instance->selected_mode = i;
	dev_dbg(sub_dev->dev, "%s() selected mode index [%d]\n", __func__,
		instance->selected_mode);

	mbus_fmt->width = ap1302_sensor_table[instance->selected_sensor].res_list[i].width;
	mbus_fmt->height = ap1302_sensor_table[instance->selected_sensor].res_list[i].height;
	mbus_fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	mbus_fmt->colorspace = V4L2_COLORSPACE_SRGB;
	mbus_fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(mbus_fmt->colorspace);
	mbus_fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	mbus_fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(mbus_fmt->colorspace);
	memset(mbus_fmt->reserved, 0, sizeof(mbus_fmt->reserved));

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		fmt = v4l2_subdev_get_try_format(sub_dev, sd_state, 0);
	else
		fmt = &instance->fmt;

	memmove(fmt, mbus_fmt, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int ops_enum_frame_size(struct v4l2_subdev *sub_dev,
			       struct v4l2_subdev_state *sd_state,
			       struct v4l2_subdev_frame_size_enum *fse)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	dev_dbg(sub_dev->dev, "%s() %x %x %x\n", __func__,
		fse->pad, fse->code, fse->index);

	if ((fse->pad != 0) ||
	    (fse->index >= ap1302_sensor_table[instance->selected_sensor].res_list_size))
		return -EINVAL;

	if(!instance->supports_over_4k_res &&
	    ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].width > 4096)
		return -EINVAL;

	fse->min_width = fse->max_width = ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].width;
	fse->min_height = fse->max_height = ap1302_sensor_table[instance->selected_sensor].res_list[fse->index].height;

	return 0;
}

static int ops_enum_frame_interval(struct v4l2_subdev *sub_dev,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_frame_interval_enum *fie)
{
	struct sensor *instance = container_of(sub_dev, struct sensor, v4l2_subdev);
	int i;
	dev_dbg(sub_dev->dev, "%s() %x %x %x\n", __func__,
				fie->pad, fie->code, fie->index);

	if ((fie->pad != 0) ||
	    (fie->index != 0))
		return -EINVAL;

	fie->interval.numerator = 1;

	for(i = 0 ; i < ap1302_sensor_table[instance->selected_sensor].res_list_size ; i++) {
		if(fie->width == ap1302_sensor_table[instance->selected_sensor].res_list[i].width &&
			fie->height == ap1302_sensor_table[instance->selected_sensor].res_list[i].height) {
				fie->interval.denominator = ap1302_sensor_table[instance->selected_sensor].res_list[i].framerates;
				break;
			}
	}

	return 0;
}

static int ops_media_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_subdev_core_ops sensor_v4l2_subdev_core_ops = {
	.s_power = ops_power,
	.init = ops_init,
	.load_fw = ops_load_fw,
	.reset = ops_reset,
};
static const struct v4l2_subdev_video_ops sensor_v4l2_subdev_video_ops = {
	.g_frame_interval = ops_get_frame_interval,
	.s_frame_interval = ops_set_frame_interval,
	.s_stream = ops_set_stream,
};
static const struct v4l2_subdev_pad_ops sensor_v4l2_subdev_pad_ops = {
	.enum_mbus_code = ops_enum_mbus_code,
	.get_fmt = ops_get_fmt,
	.set_fmt = ops_set_fmt,
	.enum_frame_size = ops_enum_frame_size,
	.enum_frame_interval = ops_enum_frame_interval,
};

static const struct v4l2_subdev_ops sensor_subdev_ops = {
	.core = &sensor_v4l2_subdev_core_ops,
	.video = &sensor_v4l2_subdev_video_ops,
	.pad = &sensor_v4l2_subdev_pad_ops,
};

static const struct media_entity_operations sensor_media_entity_ops = {
	.link_setup = ops_media_link_setup,
};

static int check_sensor_chip_id(struct i2c_client *client, u16* chip_id)
{
	int timeout;

	for (timeout = 0 ; timeout < 100 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x60AC, chip_id);
		if ((*chip_id & 0x7) == 0)
			break;
	}
	if (timeout >= 100) {
		dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, *chip_id);
		return -EINVAL;
	}
	sensor_i2c_write_16b(client, 0x60AA, 0x0002); // DMA_SIZE
	sensor_i2c_write_16b(client, 0x60A0, 0x0320); // DMA_SRC_0
	sensor_i2c_write_16b(client, 0x60A2, 0x3000); // DMA_SRC_1
	sensor_i2c_write_16b(client, 0x60A4, 0x0000); // DMA_DST_0
	sensor_i2c_write_16b(client, 0x60A6, 0x60A4); // DMA_DST_1
	sensor_i2c_write_16b(client, 0x60AC, 0x0032); // DMA_CTRL
	for (timeout = 0 ; timeout < 100 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x60AC, chip_id);
		if ((*chip_id & 0x7) == 0)
			break;
	}
	if (timeout >= 100) {
		dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, *chip_id);
		return -EINVAL;
	}
	sensor_i2c_read_16b(client, 0x60A4, chip_id);

	return 0;
}

static int set_standby_mode_rel419(struct i2c_client *client, int enable)
{
	u16 v = 0;
	int timeout;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	if (enable == 1) {
		sensor_i2c_write_16b(client, 0xf056, 0x0000);
		sensor_i2c_write_16b(client, 0x601a, 0x8140);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if (timeout < 500) {
			if(check_sensor_chip_id(client, &v) == 0) {
				if (v == 0x356) {
					dev_dbg(&client->dev, "sensor check: v=0x%x\nbypass standby and set fw stall gate frames.\n", v);
					return 0;
				}
			}
			// Reset ADV_GPIO in Advanced Registers
			sensor_i2c_write_16b(client, 0xF038, 0x002A);
			sensor_i2c_write_16b(client, 0xF03A, 0x0000);
			sensor_i2c_write_16b(client, 0xE002, 0x0490);
			sensor_i2c_write_16b(client, 0xFFFE, 1);
			msleep(100);
		} else {
			dev_err(&client->dev, "timeout: line[%d]\n", __LINE__);
			return -EINVAL;
		}
	} else {
		sensor_i2c_write_16b(client, 0xFFFE, 0);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0, &v);
		  	if (v != 0)
		 	  break;
		}
		if (timeout >= 500) {
		 dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
		 return -EINVAL;
		}

		if(check_sensor_chip_id(client, &v) == 0) {
			if (v == 0x356) {
				dev_dbg(&client->dev, "sensor check: v=0x%x\nrecover status from fw stall gate frames.\n", v);
				sensor_i2c_write_16b(client, 0x601a, 0x8340);
				msleep(10);
			}
		}

		sensor_i2c_read_16b(client, 0x601a, &v);
		if ((v & 0x200) != 0x200){
			dev_dbg(&client->dev, "stop waking up: camera is working.\n");
			return 0;
		}

		sensor_i2c_write_16b(client, 0x601a, 0x241);
		usleep_range(1000, 2000);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 1) == 0)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			return -EINVAL;
		}

		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x8000) == 0x8000)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			return -EINVAL;
		}

		sensor_i2c_write_16b(client, 0x601a, 0x8250);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x8040) == 0x8040)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			return -EINVAL;
		}
		sensor_i2c_write_16b(client, 0xF056, 0x0000);

		dev_dbg(&client->dev, "sensor wake up\n");
	}

	return 0;
}

static int sensor_standby(struct i2c_client *client, int enable)
{
	u16 v = 0;
	int timeout;
	u16 checksum = 0;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	sensor_i2c_read_16b(client, 0x6134, &checksum);

	if(checksum != 0xFFFF){
		return set_standby_mode_rel419(client, enable); // standby for rel419
	}

	if (enable == 1) {
		sensor_i2c_write_16b(client, 0x601a, 0x0180);
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if (timeout < 500) {
			msleep(100);
		} else {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			return -EINVAL;
		}
	} else {
		sensor_i2c_write_16b(client, 0x601a, 0x0380);
		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0)
				break;
		}
		if (timeout >= 100) {
			dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
			return -EINVAL;
		}
		dev_dbg(&client->dev, "sensor wake up\n");
	}

	return 0;
}

static int sensor_power_on(struct sensor *instance)
{
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);
	gpiod_set_value_cansleep(instance->host_power_gpio, 1);
	gpiod_set_value_cansleep(instance->device_power_gpio, 1);
	usleep_range(500, 5000);
	gpiod_set_value_cansleep(instance->reset_gpio, 1);
	msleep(10);

	return 0;
}

static int sensor_power_off(struct sensor *instance)
{
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);
	gpiod_set_value_cansleep(instance->standby_gpio, 0);
	gpiod_set_value_cansleep(instance->reset_gpio, 0);
	usleep_range(50, 500);
	gpiod_set_value_cansleep(instance->device_power_gpio, 0);
	gpiod_set_value_cansleep(instance->host_power_gpio, 0);
	msleep(10);

	return 0;
}

static int sensor_try_on(struct sensor *instance)
{
	u16 v;
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);

	sensor_power_off(instance);

	sensor_power_on(instance);

	if (sensor_i2c_read_16b(instance->i2c_client, 0, &v) != 0) {
		dev_err(&instance->i2c_client->dev, "%s() try on failed\n",
			__func__);
		sensor_power_off(instance);
		return -EINVAL;
	}

	return 0;
}

static int sensor_load_bootdata(struct sensor *instance)
{
	struct device *dev = &instance->i2c_client->dev;
	int index = 0;
	size_t len = BOOT_DATA_WRITE_LEN;
	u16 otp_data;
	u16 *bootdata_temp_area;
	u16 checksum;

	bootdata_temp_area = devm_kzalloc(dev,
					  BOOT_DATA_WRITE_LEN + 2,
					  GFP_KERNEL);
	if (bootdata_temp_area == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -EINVAL;
	}

	checksum = ap1302_otp_flash_get_checksum(instance->otp_flash_instance);

	while(!(len < BOOT_DATA_WRITE_LEN)) {
		bootdata_temp_area[0] = cpu_to_be16(BOOT_DATA_START_REG);
		len = ap1302_otp_flash_read(instance->otp_flash_instance,
					    (u8 *)(&bootdata_temp_area[1]),
					    index, BOOT_DATA_WRITE_LEN);
		dev_dbg(dev, "index: 0x%04x, len [%zu]\n", index, len);
		sensor_i2c_write_bust(instance->i2c_client,
				      (u8 *)bootdata_temp_area,
				      len + 2);
		index += len;
	}

	sensor_i2c_write_16b(instance->i2c_client, 0x6002, 0xffff);
	devm_kfree(dev, bootdata_temp_area);

	index = 0;
	otp_data = 0;
	while(otp_data != checksum && index < 20) {
		msleep(10);
		sensor_i2c_read_16b(instance->i2c_client, 0x6134, &otp_data);
		index ++;
	}
	if (unlikely(index == 20)) {
		if (likely(otp_data == 0))
			dev_err(dev, "failed try to read checksum\n");
		else
			dev_err(dev, "bootdata checksum missmatch\n");

		return -EINVAL;
	}

	return 0;
}

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sensor *instance = NULL;
	struct device *dev = &client->dev;
	struct v4l2_mbus_framefmt *fmt;
	struct header_ver2 *header;
	int data_lanes;
	int continuous_clock;
	int i;
	int ret;

	dev_info(&client->dev, "%s() device node: %s\n",
		       __func__, client->dev.of_node->full_name);

	instance = devm_kzalloc(dev, sizeof(struct sensor), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -EINVAL;
	}
	instance->i2c_client = client;

	instance->host_power_gpio = devm_gpiod_get(dev, "host-power",
						   GPIOD_OUT_LOW);
	instance->device_power_gpio = devm_gpiod_get(dev, "device-power",
						     GPIOD_OUT_LOW);
	instance->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	instance->standby_gpio = devm_gpiod_get(dev, "standby", GPIOD_OUT_LOW);

	if (IS_ERR(instance->reset_gpio) ||
	    IS_ERR(instance->host_power_gpio) ||
	    IS_ERR(instance->device_power_gpio) ||
	    IS_ERR(instance->standby_gpio) ) {
		dev_err(dev, "get gpio object failed\n");
		return -EPROBE_DEFER;
	}

	data_lanes = 4;
	if (of_property_read_u32(dev->of_node, "data-lanes", &data_lanes) == 0) {
		if ((data_lanes < 1) || (data_lanes > 4)) {
			dev_err(dev, "value of 'data-lanes' property is invaild\n");
			data_lanes = 4;
		}
	}

	continuous_clock = 0;
	if (of_property_read_u32(dev->of_node, "continuous-clock",
				 &continuous_clock) == 0) {
		if (continuous_clock > 1) {
			dev_err(dev, "value of 'continuous-clock' property is invaild\n");
			continuous_clock = 0;
		}
	}

	instance->supports_over_4k_res = of_property_read_bool(dev->of_node, "supports-over-4k-res");

	dev_dbg(dev, "data-lanes [%d] ,continuous-clock [%d], supports-over-4k-res [%d]\n",
		data_lanes, continuous_clock, instance->supports_over_4k_res);

	if (sensor_try_on(instance) != 0) {
		return -EINVAL;
	}

	instance->otp_flash_instance = ap1302_otp_flash_init(dev);
	if(IS_ERR(instance->otp_flash_instance)) {
		dev_err(dev, "otp flash init failed\n");
		return -EINVAL;
	}

	header = instance->otp_flash_instance->header_data;
	for(i = 0 ; i < ARRAY_SIZE(ap1302_sensor_table); i++)
	{
		if (strcmp((const char*)header->product_name, ap1302_sensor_table[i].sensor_name) == 0)
			break;
	}
	instance->selected_sensor = i;
	dev_dbg(dev, "selected_sensor:%d, sensor_name:%s\n", i, header->product_name);

	if(sensor_load_bootdata(instance) != 0) {
		dev_err(dev, "load bootdata failed\n");
		return -EINVAL;
	}

	fmt = &instance->fmt;
	fmt->width = ap1302_sensor_table[instance->selected_sensor].res_list[0].width;
	fmt->height = ap1302_sensor_table[instance->selected_sensor].res_list[0].height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;
	fmt->colorspace =  V4L2_COLORSPACE_SRGB;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	memset(fmt->reserved, 0, sizeof(fmt->reserved));

	v4l2_i2c_subdev_init(&instance->v4l2_subdev,
			     instance->i2c_client, &sensor_subdev_ops);
	//instance->v4l2_subdev.flags |= V4L2_SUBDEV_FL_HAS_EVENTS;
	instance->pad.flags = MEDIA_PAD_FL_SOURCE;
	instance->v4l2_subdev.entity.ops = &sensor_media_entity_ops;
	instance->v4l2_subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&instance->v4l2_subdev.entity, 1, &instance->pad);
	ret += v4l2_async_register_subdev(&instance->v4l2_subdev);
	if (ret != 0) {
		dev_err(&instance->i2c_client->dev, "v4l2 register failed\n");
		return -EINVAL;
	}

	//set something reference from DevX tool register log
	//cntx select 'Video'
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
	sensor_i2c_write_16b(instance->i2c_client, 0x1000, 0); //CTRL
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
	msleep(1);
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
	//Video output
	sensor_i2c_write_16b(instance->i2c_client, 0x2000, ap1302_sensor_table[instance->selected_sensor].res_list[0].width); //VIDEO_WIDTH
	sensor_i2c_write_16b(instance->i2c_client, 0x2002, ap1302_sensor_table[instance->selected_sensor].res_list[0].height); //VIDEO_HEIGHT
	sensor_i2c_write_16b(instance->i2c_client, 0x2012, 0x50); //VIDEO_OUT_FMT
	//continuous clock, data-lanes
	sensor_i2c_write_16b(instance->i2c_client, 0x2030,
			     0x10 | (continuous_clock << 5) | (data_lanes)); //VIDEO_HINF_CTRL
	sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC

	//let ap1302 go to standby mode
	ret = sensor_standby(instance->i2c_client, 1);
	if (ret == 0)
		dev_info(&client->dev, "probe success\n");
	else
		dev_err(&client->dev, "probe failed\n");

	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ "tevi-ap1302", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static const struct of_device_id sensor_of[] = {
	{ .compatible = "tn,tevi-ap1302" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sensor_of);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(sensor_of),
		.name  = "tevi-ap1302",
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

module_i2c_driver(sensor_i2c_driver);

MODULE_AUTHOR("TECHNEXION Inc.");
MODULE_DESCRIPTION("TechNexion driver for TEVI-AR Series");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("Camera");
