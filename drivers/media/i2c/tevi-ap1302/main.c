#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <media/media-entity.h>
#include "sensor_tbls.h"
#include "otp_flash.h"

#define DRIVER_NAME "tevi-ap1302"

#define AP1302_BRIGHTNESS						(0x7000)
#define AP1302_BRIGHTNESS_MASK					(0xFFFF)
#define AP1302_CONTRAST							(0x7002)
#define AP1302_CONTRAST_MASK					(0xFFFF)
#define AP1302_SATURATION						(0x7006)
#define AP1302_SATURATION_MASK					(0xFFFF)
#define AP1302_AWB_CTRL_MODE					(0x5100)
#define AP1302_AWB_CTRL_MODE_MASK				(0x00FF)
#define AP1302_AWB_CTRL_MODE_MANUAL_TEMP 		(7U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO				(15U << 0)
#define AP1302_AWB_CTRL_MODE_MANUAL_TEMP_IDX 	(0U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO_IDX			(1U << 0)
#define AP1302_GAMMA							(0x700A)
#define AP1302_GAMMA_MASK						(0xFFFF)
#define AP1302_AE_MANUAL_EXP_TIME				(0x500C)
#define AP1302_AE_MANUAL_EXP_TIME_MASK			(0xFFFFFFFF)
#define AP1302_AE_MANUAL_GAIN					(0x5006)
#define AP1302_AE_MANUAL_GAIN_MASK				(0xFFFF)
#define AP1302_ORIENTATION						(0x100C)
#define AP1302_ORIENTATION_HFLIP				(1U << 0)
#define AP1302_ORIENTATION_VFLIP				(1U << 1)
#define AP1302_AWB_MANUAL_TEMP					(0x510A)
#define AP1302_AWB_MANUAL_TEMP_MASK				(0xFFFF)
#define AP1302_SHARPEN							(0x7010)
#define AP1302_SHARPEN_MASK						(0xFFFF)
#define AP1302_BACKLIGHT_COMPENSATION 			(0x501A)
#define AP1302_BACKLIGHT_COMPENSATION_MASK		(0xFFFF)
#define AP1302_DZ_TGT_FCT						(0x1010)
#define AP1302_DZ_TGT_FCT_MASK					(0xFFFF)
#define AP1302_SFX_MODE							(0x1016)
#define AP1302_SFX_MODE_SFX_MASK				(0x00FF)
#define AP1302_SFX_MODE_SFX_NORMAL				(0U << 0)
#define AP1302_SFX_MODE_SFX_BW					(3U << 0)
#define AP1302_SFX_MODE_SFX_GRAYSCALE			(6U << 0)
#define AP1302_SFX_MODE_SFX_NEGATIVE			(7U << 0)
#define AP1302_SFX_MODE_SFX_SKETCH				(15U << 0)
#define AP1302_SFX_MODE_SFX_NORMAL_IDX			(0U << 0)
#define AP1302_SFX_MODE_SFX_BW_IDX				(1U << 0)
#define AP1302_SFX_MODE_SFX_GRAYSCALE_IDX		(2U << 0)
#define AP1302_SFX_MODE_SFX_NEGATIVE_IDX		(3U << 0)
#define AP1302_SFX_MODE_SFX_SKETCH_IDX			(4U << 0)
#define AP1302_AE_CTRL_MODE						(0x5002)
#define AP1302_AE_CTRL_MODE_MASK				(0x00FF)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN		(0U << 0)
#define AP1302_AE_CTRL_FULL_AUTO				(12U << 0)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN_IDX	(0U << 0)
#define AP1302_AE_CTRL_FULL_AUTO_IDX			(1U << 0)
#define AP1302_DZ_CT_X							(0x118C)
#define AP1302_DZ_CT_X_MASK						(0xFFFF)
#define AP1302_DZ_CT_Y							(0x118E)
#define AP1302_DZ_CT_Y_MASK						(0xFFFF)
#define AP1302_FLICK_CTRL                       (0x5440)
#define AP1302_FLICK_CTRL_FREQ(n)				((n) << 8)
#define AP1302_FLICK_CTRL_ETC_IHDR_UP			BIT(6)
#define AP1302_FLICK_CTRL_ETC_DIS				BIT(5)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define AP1302_FLICK_CTRL_FRC_EN				BIT(2)
#define AP1302_FLICK_CTRL_MODE_MASK				(0x03)
#define AP1302_FLICK_CTRL_ETC_IHDR_UP			BIT(6)
#define AP1302_FLICK_CTRL_ETC_DIS				BIT(5)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
#define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
#define AP1302_FLICK_CTRL_FRC_EN				BIT(2)
#define AP1302_FLICK_CTRL_MODE_DISABLED         (0U << 0)
#define AP1302_FLICK_CTRL_MODE_MANUAL           (1U << 0)
#define AP1302_FLICK_CTRL_MODE_AUTO             (2U << 0)
#define AP1302_FLICK_CTRL_FREQ_MASK			    (0xFF00)
#define AP1302_FLICK_CTRL_MODE_50HZ             (AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL)
#define AP1302_FLICK_CTRL_MODE_60HZ             (AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL)
#define AP1302_FLICK_MODE_DISABLED_IDX			(0U << 0)
#define AP1302_FLICK_MODE_ENABLED_IDX			(3U << 0)

#define V4L2_CID_SENSOR_FLASH_ID            (V4L2_CID_USER_BASE + 44)
// TODO This should go in v4l2-controls.h after V4L2_CID_USER_CCS_BASE
/* The base for the AP1302 driver controls.
 * We reserve 32 controls for this driver. */
// #ifndef V4L2_CID_USER_CCS_BASE
// #define V4L2_CID_USER_CCS_BASE 					(V4L2_CID_USER_BASE + 0x10f0)
// #endif
// #define V4L2_CID_USER_AP1302_BASE 				(V4L2_CID_USER_CCS_BASE + 128)

// #define V4L2_CID_AP1302_STEREO_ORDER 			(V4L2_CID_USER_AP1302_BASE + 0)

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
	u8 flash_id;
	bool supports_over_4k_res;
	char *sensor_name;

	struct mutex lock;	/* Protects formats */
	/* V4L2 Controls */
	struct v4l2_ctrl_handler ctrls;

	// bool stereo_order;
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
	int retry_tmp = 0;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;
	buf[2] = val >> 8;
	buf[3] = val & 0xff;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.buf = buf;
	msg.len = sizeof(buf);


	while((i2c_transfer(client->adapter, &msg, 1)) < 0)
	{
		retry_tmp++;
		dev_dbg(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write 16b reg:%x val:%x.\n", reg, val);

		if (retry_tmp > 50)
		{
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}

static int sensor_i2c_write_bust(struct i2c_client *client, u8 *buf, size_t len)
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

	while((i2c_transfer(client->adapter, &msg, 1)) < 0)
	{
		retry_tmp++;
		dev_dbg(&client->dev, "i2c transfer retry:%d.\n", retry_tmp);
		dev_dbg(&client->dev, "write bust buf:%x.\n", client->addr);

		if (retry_tmp > 50)
		{
			dev_err(&client->dev, "i2c transfer error.\n");
			return -EIO;
		}
	}

	return 0;
}

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
		/* 
		 * This is a workaround for ISP bug. 
		 * Before ISP standby, setting a default width and height for next streaming on.
		 * Otherwise, some resolutions of sensor will not streaming successful.
		 */ 
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
		//VIDEO_WIDTH
		sensor_i2c_write_16b(instance->i2c_client, 0x2000, 1280);
		//VIDEO_HEIGHT
		sensor_i2c_write_16b(instance->i2c_client, 0x2002, 720);
		sensor_i2c_write_16b(instance->i2c_client, 0x1184, 0xb); //ATOMIC
		ret = sensor_standby(instance->i2c_client, 1);
	} else {
		ret = sensor_standby(instance->i2c_client, 0);
		if (ret == 0) {
			int fps = ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].framerates;
			dev_dbg(sub_dev->dev, "%s() width=%d, height=%d, mode=%d\n", __func__,
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width,
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height,
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].mode);
			sensor_i2c_write_16b(instance->i2c_client, 0x1184, 1); //ATOMIC
			//PREVIEW_SENSOR_MODE
			sensor_i2c_write_16b(instance->i2c_client, 0x2014,
						ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].mode);
			//PREVIEW_WIDTH
			sensor_i2c_write_16b(instance->i2c_client, 0x2000,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width);
			//PREVIEW_HEIGHT
			sensor_i2c_write_16b(instance->i2c_client, 0x2002,
					     ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			//PREVIEW_MAX_FPS
			sensor_i2c_write_16b(instance->i2c_client, 0x2020, fps << 8);
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

/* -----------------------------------------------------------------------------
 * V4L2 Controls
 */

static int ops_set_brightness(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_BRIGHTNESS, value & AP1302_BRIGHTNESS_MASK);
}

static int ops_get_brightness(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BRIGHTNESS, &val);
	if (ret)
		return ret;

	*value = val & AP1302_BRIGHTNESS_MASK;
	return 0;
}

static int ops_set_contrast(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_CONTRAST, value & AP1302_CONTRAST_MASK);
}

static int ops_get_contrast(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_CONTRAST, &val);
	if (ret)
		return ret;

	*value = val & AP1302_CONTRAST_MASK;
	return 0;
}

static int ops_set_saturation(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_SATURATION, value & AP1302_SATURATION_MASK);
}

static int ops_get_saturation(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SATURATION, &val);
	if (ret)
		return ret;

	*value = val & AP1302_SATURATION_MASK;
	return 0;
}

static const char * const awb_mode_strings[] = {
	"Manual Temp Mode", // AP1302_AWB_CTRL_MODE_MANUAL_TEMP
	"Auto Mode", // AP1302_AWB_CTRL_MODE_AUTO
	NULL,
};

static int ops_set_awb_mode(struct sensor *instance, s32 mode)
{
	u16 val = mode & AP1302_AWB_CTRL_MODE_MASK;

	switch(val)
	{
	case 0:
		val = AP1302_AWB_CTRL_MODE_MANUAL_TEMP;
		break;
	case 1:
		val = AP1302_AWB_CTRL_MODE_AUTO;
		break;
	default:
		val = AP1302_AWB_CTRL_MODE_AUTO;
		break;
	}

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_AWB_CTRL_MODE, val);
}

static int ops_get_awb_mode(struct sensor *instance, s32 *mode)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AWB_CTRL_MODE, &val);
	if (ret)
		return ret;

	switch (val & AP1302_AWB_CTRL_MODE_MASK)
	{
	case AP1302_AWB_CTRL_MODE_MANUAL_TEMP:
		*mode = 0;
		break;
	case AP1302_AWB_CTRL_MODE_AUTO:
		*mode = 1;
		break;
	default:
		*mode = 1;
		break;
	}

	return 0;
}

static int ops_set_gamma(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_GAMMA, value & AP1302_GAMMA_MASK);
}

static int ops_get_gamma(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_GAMMA, &val);
	if (ret)
		return ret;

	*value = val & AP1302_GAMMA_MASK;
	return 0;
}

static int ops_set_exposure(struct sensor *instance, s32 value)
{
	int ret;

	ret = sensor_i2c_write_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME, (value >> 16) & 0xFFFF);
	if (ret)
		return ret;
	usleep_range(9000, 10000);
	ret = sensor_i2c_write_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME + 2, value & 0xFFFF);
	if (ret)
		return ret;

	return 0;
}

static int ops_get_exposure(struct sensor *instance, s32 *value)
{
	u16 val_msb, val_lsb;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME, &val_msb);
	if (ret)
		return ret;
	usleep_range(9000, 10000);
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME + 2, &val_lsb);
	if (ret)
		return ret;

	*value = ((u32)(val_msb) << 16) + val_lsb;
	return 0;
}

static int ops_set_gain(struct sensor *instance, s32 value)
{
	// Format is u8
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_AE_MANUAL_GAIN, value & AP1302_AE_MANUAL_GAIN_MASK);
}

static int ops_get_gain(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_GAIN, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AE_MANUAL_GAIN_MASK;
	return 0;
}

static int ops_set_hflip(struct sensor *instance, s32 flip)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	val &= ~AP1302_ORIENTATION_HFLIP;
	val |= flip ? AP1302_ORIENTATION_HFLIP : 0;

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_ORIENTATION, val);
}

static int ops_get_hflip(struct sensor *instance, s32 *flip)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	*flip = !!(val & AP1302_ORIENTATION_HFLIP);
	return 0;
}

static int ops_set_vflip(struct sensor *instance, s32 flip)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	val &= ~AP1302_ORIENTATION_VFLIP;
	val |= flip ? AP1302_ORIENTATION_VFLIP : 0;

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_ORIENTATION, val);
}

static int ops_get_vflip(struct sensor *instance, s32 *flip)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_ORIENTATION, &val);
	if (ret)
		return ret;

	*flip = !!(val & AP1302_ORIENTATION_VFLIP);
	return 0;
}

static int ops_set_awb_temp(struct sensor *instance, s32 value)
{
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_AWB_MANUAL_TEMP, value & AP1302_AWB_MANUAL_TEMP_MASK);
}

static int ops_get_awb_temp(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AWB_MANUAL_TEMP, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AWB_MANUAL_TEMP_MASK;
	return 0;
}

static int ops_set_sharpen(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_SHARPEN, value & AP1302_SHARPEN_MASK);
}

static int ops_get_sharpen(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SHARPEN, &val);
	if (ret)
		return ret;

	*value = val & AP1302_SHARPEN_MASK;
	return 0;
}

static int ops_set_backlight_compensation(struct sensor *instance, s32 value)
{
	// Format is u3.12
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_BACKLIGHT_COMPENSATION, value & AP1302_BACKLIGHT_COMPENSATION_MASK);
}

static int ops_get_backlight_compensation(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BACKLIGHT_COMPENSATION, &val);
	if (ret)
		return ret;

	*value = val & AP1302_BACKLIGHT_COMPENSATION_MASK;
	return 0;
}

static int ops_set_zoom_target(struct sensor *instance, s32 value)
{
	// Format u7.8
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_DZ_TGT_FCT, value & AP1302_DZ_TGT_FCT_MASK);
}

static int ops_get_zoom_target(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_TGT_FCT, &val);
	if (ret)
		return ret;

	*value = val & AP1302_DZ_TGT_FCT_MASK;
	return 0;
}

static const char * const sfx_mode_strings[] = {
	"Normal Mode", // AP1302_SFX_MODE_SFX_NORMAL
	"Black and White Mode", // AP1302_SFX_MODE_SFX_BW
	"Grayscale Mode", // AP1302_SFX_MODE_SFX_GRAYSCALE
	"Negative Mode", // AP1302_SFX_MODE_SFX_NEGATIVE
	"Sketch Mode", // AP1302_SFX_MODE_SFX_SKETCH
	NULL,
};

static int ops_set_special_effect(struct sensor *instance, s32 mode)
{
	u16 val = mode & AP1302_SFX_MODE_SFX_MASK;

	switch(val)
	{
	case 0:
		val = AP1302_SFX_MODE_SFX_NORMAL;
		break;
	case 1:
		val = AP1302_SFX_MODE_SFX_BW;
		break;
	case 2:
		val = AP1302_SFX_MODE_SFX_GRAYSCALE;
		break;
	case 3:
		val = AP1302_SFX_MODE_SFX_NEGATIVE;
		break;
	case 4:
		val = AP1302_SFX_MODE_SFX_SKETCH;
		break;
	default:
		val = AP1302_SFX_MODE_SFX_NORMAL;
		break;
	}

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_SFX_MODE, val);
}

static int ops_get_special_effect(struct sensor *instance, s32 *mode)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SFX_MODE, &val);
	if (ret)
		return ret;

	switch (val & AP1302_SFX_MODE_SFX_MASK)
	{
	case AP1302_SFX_MODE_SFX_NORMAL:
		*mode = 0;
		break;
	case AP1302_SFX_MODE_SFX_BW:
		*mode = 1;
		break;
	case AP1302_SFX_MODE_SFX_GRAYSCALE:
		*mode = 2;
		break;
	case AP1302_SFX_MODE_SFX_NEGATIVE:
		*mode = 3;
		break;
	case AP1302_SFX_MODE_SFX_SKETCH:
		*mode = 4;
		break;
	default:
		*mode = 0;
		break;
	}

	return 0;
}

static const char * const ae_mode_strings[] = {
	"Manual Mode", // AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN
	"Auto Mode", // AP1302_AE_CTRL_FULL_AUTO
	NULL,
};

static int ops_set_ae_mode(struct sensor *instance, s32 mode)
{
	u16 val = mode & AP1302_AE_CTRL_MODE_MASK;

	switch(val)
	{
	case 0:
		val |= AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN;
		break;
	case 1:
		val |= AP1302_AE_CTRL_FULL_AUTO;
		break;
	default:
		val |= AP1302_AE_CTRL_FULL_AUTO;
		break;
	}

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_AE_CTRL_MODE, val);
}

static int ops_get_ae_mode(struct sensor *instance, s32 *mode)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_CTRL_MODE, &val);
	if (ret)
		return ret;

	switch (val & AP1302_AE_CTRL_MODE_MASK)
	{
	case AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN:
		*mode = 0;
		break;
	case AP1302_AE_CTRL_FULL_AUTO:
		*mode = 1;
		break;
	default:
		*mode = 1;
		break;
	}
	return 0;
}

static const char * const flick_mode_strings[] = {
	"Disabled",
	"50 Hz",
	"60 Hz",
	"Auto",
	NULL,
};

static int ops_set_flick_mode(struct sensor *instance, s32 mode)
{
	u16 val = 0;
	switch(mode)
	{
	case 0:
		val = AP1302_FLICK_CTRL_MODE_DISABLED;
		break;
	case 1:
		val = AP1302_FLICK_CTRL_MODE_50HZ;
		break;
	case 2:
		val = AP1302_FLICK_CTRL_MODE_60HZ;
		break;
	case 3:
		val = AP1302_FLICK_CTRL_MODE_AUTO |
				AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET |
				AP1302_FLICK_CTRL_FRC_EN;
		break;
	default:
		val = AP1302_FLICK_CTRL_MODE_DISABLED;
		break;
	}

	return sensor_i2c_write_16b(instance->i2c_client, AP1302_FLICK_CTRL, val);
}

static int ops_get_flick_mode(struct sensor *instance, s32 *mode)
{
	u16 val;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_FLICK_CTRL, &val);
	if (ret)
		return ret;

	switch (val & AP1302_FLICK_CTRL_MODE_MASK)
	{
	case AP1302_FLICK_CTRL_MODE_DISABLED:
		*mode = 0;
		break;
	case AP1302_FLICK_CTRL_MODE_MANUAL:
		if((val & AP1302_FLICK_CTRL_FREQ_MASK) == AP1302_FLICK_CTRL_FREQ(50))
			*mode = 1;
		else if((val & AP1302_FLICK_CTRL_FREQ_MASK)  == AP1302_FLICK_CTRL_FREQ(50))
			*mode = 2;
		break;
	case AP1302_FLICK_CTRL_MODE_AUTO:
		*mode = 3;
		break;
	default:
		*mode = 0;
		break;
	}
	return 0;
}

static int ops_set_pan_target(struct sensor *instance, s32 value)
{
	// Format u7.8
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_DZ_CT_X, value & AP1302_DZ_CT_X_MASK);
}

static int ops_get_pan_target(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_CT_X, &val);
	if (ret)
		return ret;

	*value = val & AP1302_DZ_CT_X_MASK;
	return 0;
}

static int ops_set_tilt_target(struct sensor *instance, s32 value)
{
	// Format u7.8
	return sensor_i2c_write_16b(instance->i2c_client, AP1302_DZ_CT_Y, value & AP1302_DZ_CT_Y_MASK);
}

static int ops_get_tilt_target(struct sensor *instance, s32 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_CT_Y, &val);
	if (ret)
		return ret;

	*value = val & AP1302_DZ_CT_Y_MASK;
	return 0;
}

static int ops_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor *instance = container_of(ctrl->handler, struct sensor, ctrls);

	switch (ctrl->id)
	{
	case V4L2_CID_BRIGHTNESS:
		return ops_set_brightness(instance, ctrl->val);

	case V4L2_CID_CONTRAST:
		return ops_set_contrast(instance, ctrl->val);

	case V4L2_CID_SATURATION:
		return ops_set_saturation(instance, ctrl->val);

	case V4L2_CID_AUTO_WHITE_BALANCE:
		return ops_set_awb_mode(instance, ctrl->val);

	case V4L2_CID_GAMMA:
		return ops_set_gamma(instance, ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ops_set_exposure(instance, ctrl->val);

	case V4L2_CID_GAIN:
		return ops_set_gain(instance, ctrl->val);

	case V4L2_CID_HFLIP:
		return ops_set_hflip(instance, ctrl->val);

	case V4L2_CID_VFLIP:
		return ops_set_vflip(instance, ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ops_set_flick_mode(instance, ctrl->val);

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return ops_set_awb_temp(instance, ctrl->val);

	case V4L2_CID_SHARPNESS:
		return ops_set_sharpen(instance, ctrl->val);

	case V4L2_CID_BACKLIGHT_COMPENSATION:
		return ops_set_backlight_compensation(instance, ctrl->val);

	case V4L2_CID_COLORFX:
		return ops_set_special_effect(instance, ctrl->val);

	case V4L2_CID_EXPOSURE_AUTO:
		return ops_set_ae_mode(instance, ctrl->val);

	case V4L2_CID_PAN_ABSOLUTE:
		return ops_set_pan_target(instance, ctrl->val);

	case V4L2_CID_TILT_ABSOLUTE:
		return ops_set_tilt_target(instance, ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ops_set_zoom_target(instance, ctrl->val);
	case V4L2_CID_SENSOR_FLASH_ID:
		return 0;

	default:
		dev_dbg(&instance->i2c_client->dev, "Unknown control 0x%x\n",ctrl->id);
		return -EINVAL;
	}
}

static int ops_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct sensor *instance = container_of(ctrl->handler, struct sensor, ctrls);

	switch (ctrl->id)
	{
	case V4L2_CID_BRIGHTNESS:
		return ops_get_brightness(instance, &ctrl->val);

	case V4L2_CID_CONTRAST:
		return ops_get_contrast(instance, &ctrl->val);

	case V4L2_CID_SATURATION:
		return ops_get_saturation(instance, &ctrl->val);

	case V4L2_CID_AUTO_WHITE_BALANCE:
		return ops_get_awb_mode(instance, &ctrl->val);

	case V4L2_CID_GAMMA:
		return ops_get_gamma(instance, &ctrl->val);

	case V4L2_CID_EXPOSURE:
		return ops_get_exposure(instance, &ctrl->val);

	case V4L2_CID_GAIN:
		return ops_get_gain(instance, &ctrl->val);

	case V4L2_CID_HFLIP:
		return ops_get_hflip(instance, &ctrl->val);

	case V4L2_CID_VFLIP:
		return ops_get_vflip(instance, &ctrl->val);

	case V4L2_CID_POWER_LINE_FREQUENCY:
		return ops_get_flick_mode(instance, &ctrl->val);

	case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
		return ops_get_awb_temp(instance, &ctrl->val);

	case V4L2_CID_SHARPNESS:
		return ops_get_sharpen(instance, &ctrl->val);

	case V4L2_CID_BACKLIGHT_COMPENSATION:
		return ops_get_backlight_compensation(instance, &ctrl->val);

	case V4L2_CID_COLORFX:
		return ops_get_special_effect(instance, &ctrl->val);

	case V4L2_CID_EXPOSURE_AUTO:
		return ops_get_ae_mode(instance, &ctrl->val);

	case V4L2_CID_PAN_ABSOLUTE:
		return ops_get_pan_target(instance, &ctrl->val);

	case V4L2_CID_TILT_ABSOLUTE:
		return ops_get_tilt_target(instance, &ctrl->val);

	case V4L2_CID_ZOOM_ABSOLUTE:
		return ops_get_zoom_target(instance, &ctrl->val);
	case V4L2_CID_SENSOR_FLASH_ID:
		ctrl->val = instance->otp_flash_instance->flash_id;
		return 0;

	default:
		dev_dbg(&instance->i2c_client->dev, "Unknown control 0x%x\n",ctrl->id);
		return -EINVAL;
	}
}

static int ops_media_link_setup(struct media_entity *entity,
				const struct media_pad *local,
				const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct v4l2_ctrl_ops sensor_ctrl_ops = {
	.s_ctrl = ops_s_ctrl,
};

static const struct v4l2_ctrl_config ops_ctrls[] = {
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.name = "Brightness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x1000,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.name = "White_Balance_Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.max = AP1302_AWB_CTRL_MODE_AUTO_IDX,
		.def = AP1302_AWB_CTRL_MODE_AUTO_IDX,
		.qmenu = awb_mode_strings,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.name = "Gamma",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x0, // 2.2
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xF4240,
		.step = 1,
		.def = 0x8235, // 33333 us
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_HFLIP,
		.name = "HFlip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_VFLIP,
		.name = "VFlip",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.name = "Power_Line_Frequency",
		.type = V4L2_CTRL_TYPE_MENU,
		.max = AP1302_FLICK_MODE_ENABLED_IDX,
		.def = AP1302_FLICK_MODE_DISABLED_IDX,
		.qmenu = flick_mode_strings,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.name = "White_Balance_Temperature",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x8FC,
		.max = 0x3A98,
		.step = 0x1,
		.def = 0x1388,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_SHARPNESS,
		.name = "Sharpness",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_BACKLIGHT_COMPENSATION,
		.name = "Backlight_Compensation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x100,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.name = "Special_Effect",
		.type = V4L2_CTRL_TYPE_MENU,
		.max = AP1302_SFX_MODE_SFX_SKETCH_IDX,
		.def = AP1302_SFX_MODE_SFX_NORMAL_IDX,
		.qmenu = sfx_mode_strings,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_AUTO,
		.name = "Exposure_Mode",
		.type = V4L2_CTRL_TYPE_MENU,
		.max = AP1302_AE_CTRL_FULL_AUTO_IDX,
		.def = AP1302_AE_CTRL_FULL_AUTO_IDX,
		.qmenu = ae_mode_strings,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_PAN_ABSOLUTE,
		.name = "Pan_Target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x100,
		.step = 0x1,
		.def = 0x80,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_TILT_ABSOLUTE,
		.name = "Tilt_Target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x100,
		.step = 0x1,
		.def = 0x80,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.name = "Zoom_Target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x100,
		.max = 0x800,
		.step = 0x1,
		.def = 0x100,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_SENSOR_FLASH_ID,
		.name = "Sensor_Flash_ID",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0x7F,
		.step = 0x1,
		.def = 0x54,
	},
};

static int ops_ctrls_init(struct sensor *instance)
{
	unsigned int i;
	int ret;

	ret = v4l2_ctrl_handler_init(&instance->ctrls, ARRAY_SIZE(ops_ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ops_ctrls); i++)
	{
		struct v4l2_ctrl * ctrl = v4l2_ctrl_new_custom(&instance->ctrls,
								&ops_ctrls[i], NULL);
		ret = ops_g_ctrl(ctrl);
		if (!ret && ctrl->default_value != ctrl->val) {
			// Updating default value based on firmware values
			dev_dbg(&instance->i2c_client->dev,"Ctrl '%s' default value updated from %lld to %d\n",
					ctrl->name, ctrl->default_value, ctrl->val);
			ctrl->default_value = ctrl->val;
			ctrl->cur.val = ctrl->val;
		}
	}

	if (instance->ctrls.error) {
		dev_err(&instance->i2c_client->dev, "ctrls error\n");
		ret = instance->ctrls.error;
		v4l2_ctrl_handler_free(&instance->ctrls);
		return ret;
	}

	/* Use same lock for controls as for everything else. */
	instance->ctrls.lock = &instance->lock;
	instance->v4l2_subdev.ctrl_handler = &instance->ctrls;

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
		for (timeout = 0 ; timeout < 500 ; timeout ++) {
			if(check_sensor_chip_id(client, &v) == 0) {
				if (v == 0x356) {
					dev_dbg(&client->dev, "sensor check: v=0x%x\nrecover status from fw stall gate frames.\n", v);
					sensor_i2c_write_16b(client, 0x601a, 0x8340);
					msleep(10);
					break;
				}
			}
		}

		for (timeout = 0 ; timeout < 100 ; timeout ++) {
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, 0x601a, &v);
			if ((v & 0x200) == 0x200)
				break;
		}
		if ( (v & 0x200) != 0x200 ) {
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

	for (timeout = 0 ; timeout < 50 ; timeout ++) {
		usleep_range(9000, 10000);
		sensor_i2c_read_16b(client, 0x6134, &checksum);
		if (checksum == 0xFFFF)
			break;
	}
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
	usleep_range(500, 5000);
	gpiod_set_value_cansleep(instance->reset_gpio, 1);
	msleep(10);

	return 0;
}

static int sensor_power_off(struct sensor *instance)
{
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);
	gpiod_set_value_cansleep(instance->reset_gpio, 0);
	usleep_range(50, 500);
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
	// struct device_node *flash_node;
	struct v4l2_mbus_framefmt *fmt;
	int data_lanes;
	int continuous_clock;
	// int flash_id;
	int i;
	int ret;
	int retry_f;

	dev_info(&client->dev, "%s() device node: %s\n",
		       __func__, client->dev.of_node->full_name);

	instance = devm_kzalloc(dev, sizeof(struct sensor), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return -EINVAL;
	}
	instance->i2c_client = client;

	instance->host_power_gpio = devm_gpiod_get_optional(dev, "host-power", GPIOD_OUT_LOW);
	if (IS_ERR(instance->host_power_gpio)) {
		ret = PTR_ERR(instance->host_power_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get host-power GPIO (%d)", ret);
		return ret;
	}

	instance->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(instance->reset_gpio)) {
		ret = PTR_ERR(instance->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get reset GPIO (%d)", ret);
		return ret;
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

	retry_f = 0x01;
	/*
	bit 0: start bit
	bit 1: sensor_try_on fail
	bit 2: ap1302_otp_flash_init fail
	bit 3: sensor_load_bootdata fail
	bit 4-7: retry count
	*/
	while(retry_f) {
		retry_f &= ~0x01;

		if (sensor_try_on(instance) != 0) {
			retry_f |= 0x02 ;
		}

		instance->otp_flash_instance = ap1302_otp_flash_init(dev);
		if(IS_ERR(instance->otp_flash_instance)) {
			dev_err(dev, "otp flash init failed\n");
			// retry_f |= 0x04 ;
			return -EINVAL;
		}

		for(i = 0 ; i < ARRAY_SIZE(ap1302_sensor_table); i++)
		{
			if (strcmp((const char*)instance->otp_flash_instance->product_name, ap1302_sensor_table[i].sensor_name) == 0)
				break;
		}

		if(i >= ARRAY_SIZE(ap1302_sensor_table)) {
			dev_err(dev, "cannot not support the product: %s\n",
				(const char*)instance->otp_flash_instance->product_name);
			return  -EINVAL;
		}

		instance->selected_sensor = i;
		dev_dbg(dev, "selected_sensor:%d, sensor_name:%s\n", i, instance->otp_flash_instance->product_name);

		if(sensor_load_bootdata(instance) != 0) {
			dev_err(dev, "load bootdata failed\n");
			retry_f |= 0x08 ;
		}

		if ((retry_f & 0x0F) != 0x00) {
			if (((retry_f & 0x30) >> 4 ) < 3 ) {
				retry_f += 1 << 4;
				retry_f &= ~0x0F;
				dev_err(dev, "Probe retry:%d.\n", ((retry_f & 0x30) >> 4 ));
			}
			else {
				retry_f &= 0x00;
				dev_dbg(dev, "Probe retry failed\n");
				return  -EINVAL;
			}
		}
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
	instance->v4l2_subdev.flags |= (V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE);

	ret = ops_ctrls_init(instance);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_probe;
	}

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

error_probe:
	mutex_destroy(&instance->lock);

	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ DRIVER_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, sensor_id);

static const struct of_device_id sensor_of[] = {
	{ .compatible = "tn,"DRIVER_NAME },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sensor_of);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.of_match_table = of_match_ptr(sensor_of),
		.name  = DRIVER_NAME,
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
