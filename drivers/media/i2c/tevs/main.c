#define DEBUG

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
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include "sensor_i2c.h"
#include "sensor_tbls.h"
#include "otp_flash.h"

#define DRIVER_NAME "tevs"

#define AP1302_BRIGHTNESS						HOST_COMMAND_ISP_CTRL_BRIGHTNESS // ap1302 reg 0x7000
#define AP1302_BRIGHTNESS_MAX					HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MAX
#define AP1302_BRIGHTNESS_MIN					HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MIN
#define AP1302_BRIGHTNESS_MASK					(0xFFFF)
#define AP1302_CONTRAST							HOST_COMMAND_ISP_CTRL_CONTRAST // ap1302 reg 0x7002
#define AP1302_CONTRAST_MAX						HOST_COMMAND_ISP_CTRL_CONTRAST_MAX
#define AP1302_CONTRAST_MIN						HOST_COMMAND_ISP_CTRL_CONTRAST_MIN
#define AP1302_CONTRAST_MASK					(0xFFFF)
#define AP1302_SATURATION						HOST_COMMAND_ISP_CTRL_SATURATION // ap1302 reg 0x7006
#define AP1302_SATURATION_MAX					HOST_COMMAND_ISP_CTRL_SATURATION_MAX
#define AP1302_SATURATION_MIN					HOST_COMMAND_ISP_CTRL_SATURATION_MIN
#define AP1302_SATURATION_MASK					(0xFFFF)
#define AP1302_AWB_CTRL_MODE					HOST_COMMAND_ISP_CTRL_AWB_MODE // ap1302 reg 0x5100
#define AP1302_AWB_CTRL_MODE_MASK				(0x00FF)
#define AP1302_AWB_CTRL_MODE_MANUAL_TEMP 		(7U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO				(15U << 0)
#define AP1302_AWB_CTRL_MODE_MANUAL_TEMP_IDX 	(0U << 0)
#define AP1302_AWB_CTRL_MODE_AUTO_IDX			(1U << 0)
#define AP1302_GAMMA							HOST_COMMAND_ISP_CTRL_GAMMA // ap1302 reg 0x700A
#define AP1302_GAMMA_MAX						HOST_COMMAND_ISP_CTRL_GAMMA_MAX
#define AP1302_GAMMA_MIN						HOST_COMMAND_ISP_CTRL_GAMMA_MIN
#define AP1302_GAMMA_MASK						(0xFFFF)
#define AP1302_AE_MANUAL_EXP_TIME				HOST_COMMAND_ISP_CTRL_EXP_TIME_MSB // ap1302 reg 0x500C
#define AP1302_AE_MANUAL_EXP_TIME_MAX			HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_MSB
#define AP1302_AE_MANUAL_EXP_TIME_MIN			HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_MSB
#define AP1302_AE_MANUAL_EXP_TIME_MASK			(0xFFFFFFFF)
#define AP1302_AE_MANUAL_GAIN					HOST_COMMAND_ISP_CTRL_EXP_GAIN // ap1302 reg 0x5006
#define AP1302_AE_MANUAL_GAIN_MAX				HOST_COMMAND_ISP_CTRL_EXP_GAIN_MAX
#define AP1302_AE_MANUAL_GAIN_MIN				HOST_COMMAND_ISP_CTRL_EXP_GAIN_MIN
#define AP1302_AE_MANUAL_GAIN_MASK				(0x00FF)
#define AP1302_ORIENTATION						HOST_COMMAND_ISP_CTRL_FLIP // ap1302 reg 0x100C
#define AP1302_ORIENTATION_HFLIP				(1U << 0)
#define AP1302_ORIENTATION_VFLIP				(1U << 1)
// #define AP1302_FLICK_CTRL						(0xFFFF) // AP1302_REG_16BIT(0x5440)
// #define AP1302_FLICK_CTRL_FREQ(n)				((n) << 8)
// #define AP1302_FLICK_CTRL_ETC_IHDR_UP			BIT(6)
// #define AP1302_FLICK_CTRL_ETC_DIS				BIT(5)
// #define AP1302_FLICK_CTRL_FRC_OVERRIDE_MAX_ET	BIT(4)
// #define AP1302_FLICK_CTRL_FRC_OVERRIDE_UPPER_ET	BIT(3)
// #define AP1302_FLICK_CTRL_FRC_EN				BIT(2)
// #define AP1302_FLICK_CTRL_MODE_MASK				(3U << 0)
// #define AP1302_FLICK_CTRL_MODE_DISABLED			(0U << 0)
// #define AP1302_FLICK_CTRL_MODE_MANUAL			(1U << 0)
// #define AP1302_FLICK_CTRL_MODE_AUTO				(2U << 0)
#define AP1302_AWB_MANUAL_TEMP					HOST_COMMAND_ISP_CTRL_AWB_TEMP // ap1302 reg 0x510A
#define AP1302_AWB_MANUAL_TEMP_MAX				HOST_COMMAND_ISP_CTRL_AWB_TEMP_MAX
#define AP1302_AWB_MANUAL_TEMP_MIN				HOST_COMMAND_ISP_CTRL_AWB_TEMP_MIN
#define AP1302_AWB_MANUAL_TEMP_MASK				(0xFFFF)
#define AP1302_SHARPEN							HOST_COMMAND_ISP_CTRL_SHARPEN // ap1302 reg 0x7010
#define AP1302_SHARPEN_MAX						HOST_COMMAND_ISP_CTRL_SHARPEN_MAX
#define AP1302_SHARPEN_MIN						HOST_COMMAND_ISP_CTRL_SHARPEN_MIN
#define AP1302_SHARPEN_MASK						(0xFFFF)
#define AP1302_BACKLIGHT_COMPENSATION 			HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION // ap1302 reg 0x501A
#define AP1302_BACKLIGHT_COMPENSATION_MAX 		HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MAX
#define AP1302_BACKLIGHT_COMPENSATION_MIN 		HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MIN
#define AP1302_BACKLIGHT_COMPENSATION_MASK		(0xFFFF)
#define AP1302_DZ_TGT_FCT						HOST_COMMAND_ISP_CTRL_ZOOM_TIMES // ap1302 reg 0x1010
#define AP1302_DZ_TGT_FCT_MAX					HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MAX
#define AP1302_DZ_TGT_FCT_MIN					HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MIN
#define AP1302_DZ_TGT_FCT_MASK					(0xFFFF)
#define AP1302_SFX_MODE							HOST_COMMAND_ISP_CTRL_EFFECT // ap1302 reg 0x1016
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
#define AP1302_AE_CTRL_MODE						HOST_COMMAND_ISP_CTRL_AE_MODE // ap1302 reg 0x5002
#define AP1302_AE_CTRL_MODE_MASK				(0x00FF)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN		(0U << 0)
#define AP1302_AE_CTRL_FULL_AUTO				(12U << 0)
#define AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN_IDX	(0U << 0)
#define AP1302_AE_CTRL_FULL_AUTO_IDX			(1U << 0)
#define AP1302_DZ_CT_X							HOST_COMMAND_ISP_CTRL_CT_X // ap1302 reg 0x118C
#define AP1302_DZ_CT_X_MASK						(0xFFFF)
#define AP1302_DZ_CT_Y							HOST_COMMAND_ISP_CTRL_CT_Y // ap1302 reg 0x118E
#define AP1302_DZ_CT_Y_MASK						(0xFFFF)
#define AP1302_DZ_CT_MAX						HOST_COMMAND_ISP_CTRL_CT_MAX
#define AP1302_DZ_CT_MIN						HOST_COMMAND_ISP_CTRL_CT_MIN

// TODO This should go in v4l2-controls.h after V4L2_CID_USER_CCS_BASE
/* The base for the AP1302 driver controls.
 * We reserve 32 controls for this driver. */
#ifndef V4L2_CID_USER_CCS_BASE
#define V4L2_CID_USER_CCS_BASE 					(V4L2_CID_USER_BASE + 0x10f0)
#endif
#define V4L2_CID_USER_AP1302_BASE 				(V4L2_CID_USER_CCS_BASE + 128)

#define V4L2_CID_AP1302_STEREO_ORDER 			(V4L2_CID_USER_AP1302_BASE + 0)

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

	struct mutex lock;	/* Protects formats */
	/* V4L2 Controls */
	struct v4l2_ctrl_handler ctrls;

	bool stereo_order;
};

static int sensor_standby(struct i2c_client *client, int enable);

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
		//VIDEO_WIDTH
		sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH, 1920);
		usleep_range(9000, 10000);
		//VIDEO_HEIGHT
		sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT, 1080);
		usleep_range(9000, 10000);
		ret = sensor_standby(instance->i2c_client, 1);
	} else {
		ret = sensor_standby(instance->i2c_client, 0);
		if (ret == 0) {
			int fps = ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].framerates;
			dev_dbg(sub_dev->dev, "%s() width=%d, height=%d\n", __func__,
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width,
				ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			usleep_range(9000, 10000);
			//VIDEO_WIDTH
			sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH,
						ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].width);
			usleep_range(9000, 10000);
			//VIDEO_HEIGHT
			sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT,
						ap1302_sensor_table[instance->selected_sensor].res_list[instance->selected_mode].height);
			usleep_range(9000, 10000);
			//VIDEO_MAX_FPS
			sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS, fps);
			usleep_range(9000, 10000);
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

static int ops_get_brightness_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BRIGHTNESS_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_BRIGHTNESS_MASK;
	return 0;
}

static int ops_get_brightness_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BRIGHTNESS_MIN, &val);
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

static int ops_get_contrast_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_CONTRAST_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_CONTRAST_MASK;
	return 0;
}

static int ops_get_contrast_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_CONTRAST_MIN, &val);
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

static int ops_get_saturation_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SATURATION_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_SATURATION_MASK;
	return 0;
}

static int ops_get_saturation_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SATURATION_MIN, &val);
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

static int ops_get_gamma_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_GAMMA_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_GAMMA_MASK;
	return 0;
}

static int ops_get_gamma_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_GAMMA_MIN, &val);
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

static int ops_get_exposure_max(struct sensor *instance, s64 *value)
{
	u16 val_msb, val_lsb;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME_MAX, &val_msb);
	if (ret)
		return ret;
	usleep_range(9000, 10000);
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME_MAX + 2, &val_lsb);
	if (ret)
		return ret;

	*value = ((u32)(val_msb) << 16) + val_lsb;
	return 0;
}

static int ops_get_exposure_min(struct sensor *instance, s64 *value)
{
	u16 val_msb, val_lsb;
	int ret;

	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME_MIN, &val_msb);
	if (ret)
		return ret;
	usleep_range(9000, 10000);
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_EXP_TIME_MIN + 2, &val_lsb);
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

static int ops_get_gain_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_GAIN_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AE_MANUAL_GAIN_MASK;
	return 0;
}

static int ops_get_gain_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AE_MANUAL_GAIN_MIN, &val);
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

// static const u16 ap1302_flicker_values[] = {
// 	AP1302_FLICK_CTRL_MODE_DISABLED,
// 	AP1302_FLICK_CTRL_FREQ(50) | AP1302_FLICK_CTRL_MODE_MANUAL,
// 	AP1302_FLICK_CTRL_FREQ(60) | AP1302_FLICK_CTRL_MODE_MANUAL,
// 	AP1302_FLICK_CTRL_MODE_AUTO,
// };

// static int ops_set_flicker_freq(struct sensor *instance, s32 val)
// {
// 	return sensor_i2c_write_16b(instance->i2c_client, AP1302_FLICK_CTRL,
// 			    ap1302_flicker_values[val]);
// }

// static int ops_get_flicker_freq(struct sensor *instance, s32 *value)
// {
// 	u16 val;
// 	int ret;

// 	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_FLICK_CTRL, &val);
// 	if (ret)
// 		return ret;

// 	*value = V4L2_CID_POWER_LINE_FREQUENCY_AUTO; // Default


// 	if ((val & AP1302_FLICK_CTRL_MODE_MASK) ==
// 			AP1302_FLICK_CTRL_MODE_DISABLED) {
// 		*value = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
// 	}
// 	else if ((val & AP1302_FLICK_CTRL_MODE_MASK) ==
// 			AP1302_FLICK_CTRL_MODE_MANUAL) {
// 		if ((val >> 8) == 50)
// 			*value = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
// 		if ((val >> 8) == 60)
// 			*value = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;

// 	}
// 	else if((val & AP1302_FLICK_CTRL_MODE_MASK) ==
// 			AP1302_FLICK_CTRL_MODE_AUTO) {
// 		*value = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
// 	}

// 	return 0;
// }

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

static int ops_get_awb_temp_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AWB_MANUAL_TEMP_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_AWB_MANUAL_TEMP_MASK;
	return 0;
}

static int ops_get_awb_temp_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_AWB_MANUAL_TEMP_MIN, &val);
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

static int ops_get_sharpen_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SHARPEN_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_SHARPEN_MASK;
	return 0;
}

static int ops_get_sharpen_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_SHARPEN_MIN, &val);
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

static int ops_get_backlight_compensation_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BACKLIGHT_COMPENSATION_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_BACKLIGHT_COMPENSATION_MASK;
	return 0;
}

static int ops_get_backlight_compensation_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_BACKLIGHT_COMPENSATION_MIN, &val);
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

static int ops_get_zoom_target_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_TGT_FCT_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_DZ_TGT_FCT_MASK;
	return 0;
}

static int ops_get_zoom_target_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_TGT_FCT_MIN, &val);
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
	u16 val = mode & AP1302_SFX_MODE_SFX_MASK;

	switch(val)
	{
	case 0:
		val = AP1302_AE_CTRL_MANUAL_EXP_TIME_GAIN;
		break;
	case 1:
		val = AP1302_AE_CTRL_FULL_AUTO;
		break;
	default:
		val = AP1302_AE_CTRL_FULL_AUTO;
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

static int ops_get_pan_tilt_target_max(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_CT_MAX, &val);
	if (ret)
		return ret;

	*value = val & AP1302_DZ_CT_Y_MASK;
	return 0;
}

static int ops_get_pan_tilt_target_min(struct sensor *instance, s64 *value)
{
	u16 val;
	int ret;
	ret = sensor_i2c_read_16b(instance->i2c_client, AP1302_DZ_CT_MIN, &val);
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

	// case V4L2_CID_POWER_LINE_FREQUENCY:
	// 	return ops_set_flicker_freq(instance, ctrl->val);

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

	// case V4L2_CID_POWER_LINE_FREQUENCY:
	// 	return ops_get_flicker_freq(instance, &ctrl->val);

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
		.step = 0x1,
		.def = 0x0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.name = "Contrast",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.name = "Saturation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
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
		.step = 0x1,
		.def = 0x2333, // 2.2
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
		.min = 0x1,
		.max = 0x40,
		.step = 0x1,
		.def = 0x1,
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
	// {
	// 	.ops = &sensor_ctrl_ops,
	// 	.id = V4L2_CID_POWER_LINE_FREQUENCY,
	// 	.min = 0,
	// 	.max = 3,
	// 	.def = 3,
	// },
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
		.step = 0x1,
		.def = 0x0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_BACKLIGHT_COMPENSATION,
		.name = "Backlight_Compensation",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
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
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_TILT_ABSOLUTE,
		.name = "Tilt_Target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
	},
	{
		.ops = &sensor_ctrl_ops,
		.id = V4L2_CID_ZOOM_ABSOLUTE,
		.name = "Zoom_Target",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0x0,
		.max = 0xFFFF,
		.step = 0x1,
		.def = 0x0,
	},
};

static int ops_ctrls_init(struct sensor *instance)
{
	unsigned int i;
	int ret;

	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);

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
			dev_info(&instance->i2c_client->dev,"Ctrl '%s' default value updated from %lld to %d\n",
					ctrl->name, ctrl->default_value, ctrl->val);
			ctrl->default_value = ctrl->val;
			ctrl->cur.val = ctrl->val;
		}
		// Updating maximum and minimum value
		switch (ctrl->id)
		{
		case V4L2_CID_BRIGHTNESS:
			ops_get_brightness_max(instance, &ctrl->maximum);
			ops_get_brightness_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_CONTRAST:
			ops_get_contrast_max(instance, &ctrl->maximum);
			ops_get_contrast_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_SATURATION:
			ops_get_saturation_max(instance, &ctrl->maximum);
			ops_get_saturation_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_GAMMA:
			ops_get_gamma_max(instance, &ctrl->maximum);
			ops_get_gamma_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_EXPOSURE:
			ops_get_exposure_max(instance, &ctrl->maximum);
			ops_get_exposure_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_GAIN:
			ops_get_gain_max(instance, &ctrl->maximum);
			ops_get_gain_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
			ops_get_awb_temp_max(instance, &ctrl->maximum);
			ops_get_awb_temp_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_SHARPNESS:
			ops_get_sharpen_max(instance, &ctrl->maximum);
			ops_get_sharpen_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_BACKLIGHT_COMPENSATION:
			ops_get_backlight_compensation_max(instance, &ctrl->maximum);
			ops_get_backlight_compensation_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_PAN_ABSOLUTE:
		case V4L2_CID_TILT_ABSOLUTE:
			ops_get_pan_tilt_target_max(instance, &ctrl->maximum);
			ops_get_pan_tilt_target_min(instance, &ctrl->minimum);
			break;

		case V4L2_CID_ZOOM_ABSOLUTE:
			ops_get_zoom_target_max(instance, &ctrl->maximum);
			ops_get_zoom_target_min(instance, &ctrl->minimum);
		default:
			break;
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

// static void ops_ctrls_cleanup(struct sensor *instance)
// {
// 	v4l2_ctrl_handler_free(&instance->ctrls);
// }

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

static int sensor_standby(struct i2c_client *client, int enable)
{
	u16 v = 0;
	int timeout = 0;
	dev_dbg(&client->dev, "%s():enable=%d\n", __func__, enable);

	if (enable == 1) {
		sensor_i2c_write_16b(client, HOST_COMMAND_ISP_CTRL_SYSTEM_START, 0x0000);
		while (timeout < 100) {
			if (++timeout >= 100) {
				dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
				return -EINVAL;
			}
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, HOST_COMMAND_ISP_CTRL_SYSTEM_START, &v);
			if ((v & 0x100) == 0)
				break;
		}
		dev_dbg(&client->dev, "sensor standby\n");
	} else {
		sensor_i2c_write_16b(client, HOST_COMMAND_ISP_CTRL_SYSTEM_START, 0x0001);
		while (timeout < 100) {
			if (++timeout >= 100) {
				dev_err(&client->dev, "timeout: line[%d]v=%x\n", __LINE__, v);
				return -EINVAL;
			}
			usleep_range(9000, 10000);
			sensor_i2c_read_16b(client, HOST_COMMAND_ISP_CTRL_SYSTEM_START, &v);
			if ((v & 0x100) == 0x100)
				break;
		}
		dev_dbg(&client->dev, "sensor wakeup\n");
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
	// u16 v;
	dev_dbg(&instance->i2c_client->dev, "%s()\n", __func__);

	sensor_power_off(instance);

	sensor_power_on(instance);

	// if (sensor_i2c_read_16b(instance->i2c_client, 0, &v) != 0) {
	// 	dev_err(&instance->i2c_client->dev, "%s() try on failed\n",
	// 		__func__);
	// 	sensor_power_off(instance);
	// 	return -EINVAL;
	// }

	return 0;
}

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct sensor *instance = NULL;
	struct device *dev = &client->dev;
	struct v4l2_mbus_framefmt *fmt;
	int data_lanes;
	int continuous_clock;
	int i = ARRAY_SIZE(ap1302_sensor_table);
	int ret;
	int timeout = 0;
	uint8_t isp_state = 0;
	// int retry_f;

	dev_info(dev, "%s() device node: %s\n",
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
	instance->device_power_gpio = devm_gpiod_get_optional(dev, "device-power", GPIOD_OUT_LOW);
	if (IS_ERR(instance->device_power_gpio)) {
		ret = PTR_ERR(instance->device_power_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get device-power GPIO (%d)", ret);
		return ret;
	}
	instance->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(instance->reset_gpio)) {
		ret = PTR_ERR(instance->reset_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get reset GPIO (%d)", ret);
		return ret;
	}
	instance->standby_gpio = devm_gpiod_get_optional(dev, "standby", GPIOD_OUT_LOW);
	if (IS_ERR(instance->standby_gpio)) {
		ret = PTR_ERR(instance->standby_gpio);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Cannot get standby GPIO (%d)", ret);
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

	dev_info(dev, "[AP1302 with TI MCU Testing] delay for waiting isp bootup ... nian\n");
	if (sensor_try_on(instance) != 0) {
		dev_err(dev, "cannot find ap1302\n");
		return -EINVAL;
	}

	msleep(500);
	while (timeout < 20) {
		if (++timeout >= 20) {
			dev_err(dev, "isp bootup timeout\n");
			ret = -EINVAL;
			goto error_probe;
		}
		sensor_i2c_read(instance->i2c_client, HOST_COMMAND_MCU_BOOT_STATE, &isp_state, 1);
		dev_dbg(dev, "isp bootup state: %d\n", isp_state);
		if (isp_state == 0x08)
			break;
		msleep(50);
	}

	instance->otp_flash_instance = ap1302_load_bootdata(instance->i2c_client);
	if(IS_ERR(instance->otp_flash_instance)) {
		dev_err(dev, "otp flash init failed\n");
		// return -EINVAL;
		i = (ARRAY_SIZE(ap1302_sensor_table) - 1);
		dev_warn(dev, "seletct unknown camera list: %d\n", i);
		return -EINVAL;
		// goto bypass_otp;
	}
	else{
		for (i = 0; i < ARRAY_SIZE(ap1302_sensor_table); i++) {
			if (strcmp((const char *)instance->otp_flash_instance->product_name,
				   ap1302_sensor_table[i].sensor_name) == 0)
				break;
		}
	}
// bypass_otp:
	if(i >= ARRAY_SIZE(ap1302_sensor_table)) {
		dev_err(dev, "cannot not support the product: %s\n",
			(const char*)instance->otp_flash_instance->product_name);
		return -EINVAL;
		i = ARRAY_SIZE(ap1302_sensor_table) - 1;
	}

	instance->selected_sensor = i;
	dev_dbg(dev, "selected_sensor:%d, sensor_name:%s\n", i, instance->otp_flash_instance->product_name);

	// if(sensor_load_bootdata(instance) != 0) {
	// 	dev_err(dev, "load bootdata failed\n");
	// 	return  -EINVAL;
	// }

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

	v4l2_i2c_subdev_init(&instance->v4l2_subdev, instance->i2c_client, &sensor_subdev_ops);

	instance->v4l2_subdev.flags |= (V4L2_SUBDEV_FL_HAS_EVENTS | V4L2_SUBDEV_FL_HAS_DEVNODE);
	instance->pad.flags = MEDIA_PAD_FL_SOURCE;
	instance->v4l2_subdev.entity.ops = &sensor_media_entity_ops;
	instance->v4l2_subdev.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&instance->v4l2_subdev.entity, 1, &instance->pad);
	ret += v4l2_async_register_subdev(&instance->v4l2_subdev);
	if (ret != 0) {
		dev_err(&instance->i2c_client->dev, "v4l2 register failed\n");
		return -EINVAL;
	}

	ret = ops_ctrls_init(instance);
	if (ret) {
		dev_err(&client->dev, "failed to init controls: %d", ret);
		goto error_probe;
	}

	dev_info(dev, "[AP1302 with TI MCU Testing] probe ... nian\n");
	sensor_standby(instance->i2c_client, 0);
	usleep_range(9000, 10000);
	sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT, 0x50); //VIDEO_OUT_FMT
	usleep_range(9000, 10000);
	// continuous clock, data-lanes
	sensor_i2c_write_16b(instance->i2c_client, HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL, 0x10 | (continuous_clock << 5) | (data_lanes));
	usleep_range(9000, 10000);

	//let ap1302 go to standby mode
	ret = sensor_standby(instance->i2c_client, 1);
	if (ret == 0)
		dev_info(dev, "probe success\n");
	else
		dev_err(dev, "probe failed\n");

	// release reset and standby control
	devm_gpiod_put(dev, instance->reset_gpio);
	devm_gpiod_put(dev, instance->standby_gpio);

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
MODULE_DESCRIPTION("TechNexion driver for TEVS");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("Camera");
