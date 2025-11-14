#ifndef __SENSOR_TABLES_H__
#define __SENSOR_TABLES_H__
#include <linux/kernel.h>
#include <media/v4l2-subdev.h>

#define SENSOR_CHIP_ID_NONE                 0x0000
#define SENSOR_CHIP_ID_ONSEMI_AR0144        0x0356
#define SENSOR_CHIP_ID_ONSEMI_AR0145        0x1750
#define SENSOR_CHIP_ID_ONSEMI_AR0234        0x0A56
#define SENSOR_CHIP_ID_ONSEMI_AR0521        0x0457
#define SENSOR_CHIP_ID_ONSEMI_AR0522        0x1457
#define SENSOR_CHIP_ID_ONSEMI_AR0821        0x2557
#define SENSOR_CHIP_ID_ONSEMI_AR0822        0x0F56
#define SENSOR_CHIP_ID_ONSEMI_AR1335        0x0153

struct resolution {
	u16 width;
	u16 height;
	u16 framerates;
	u16 mode;
};

/* AR0144 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0144_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 60, .mode = 0 },
	{ .width = 1280, .height = 720, .framerates = 60, .mode = 0 },
	{ .width = 1280, .height = 800, .framerates = 60, .mode = 0 },
};

static u32 ar0144_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0145 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0145_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 115, .mode = 0 },
	{ .width = 1280, .height = 720, .framerates = 115, .mode = 0 },
	{ .width = 1280, .height = 800, .framerates = 115, .mode = 0 },
};

static u32 ar0145_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0234 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0234_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 120, .mode = 1 },
	{ .width = 1280, .height = 720, .framerates = 120, .mode = 0 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 0 },
	{ .width = 1920, .height = 1200, .framerates = 60, .mode = 0 },
};

static u32 ar0234_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0521 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0521_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 120, .mode = 3 },
	{ .width = 1280, .height = 720, .framerates = 60, .mode = 3 },
	{ .width = 1280, .height = 960, .framerates = 60, .mode = 3 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 1 },
	{ .width = 2560, .height = 1440, .framerates = 32, .mode = 1 },
	{ .width = 2592, .height = 1944, .framerates = 24, .mode = 1 },
};

static u32 ar0521_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0522 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0522_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 120, .mode = 3 },
	{ .width = 1280, .height = 720, .framerates = 60, .mode = 3 },
	{ .width = 1280, .height = 960, .framerates = 60, .mode = 3 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 1 },
	{ .width = 2560, .height = 1440, .framerates = 32, .mode = 1 },
	{ .width = 2592, .height = 1944, .framerates = 24, .mode = 1 },
};

static u32 ar0522_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0821 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0821_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 60, .mode = 2 },
	{ .width = 1280, .height = 720, .framerates = 60, .mode = 2 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 2 },
	{ .width = 2560, .height = 1440, .framerates = 30, .mode = 0 },
	{ .width = 3840, .height = 2160, .framerates = 15, .mode = 0 },
};

static u32 ar0821_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR0822 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar0822_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 60, .mode = 1 },
	{ .width = 1280, .height = 720, .framerates = 60, .mode = 1 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 1 },
	{ .width = 2560, .height = 1440, .framerates = 30, .mode = 0 },
	{ .width = 3840, .height = 2160, .framerates = 15, .mode = 0 },
};

static u32 ar0822_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

/* AR1335 default setting for 4 data lanes and data frequency 800 MHz */
static struct resolution ar1335_res_list[] = {
	{ .width = 640, .height = 480, .framerates = 60, .mode = 4 },
	{ .width = 1280, .height = 720, .framerates = 120, .mode = 4 },
	{ .width = 1920, .height = 1080, .framerates = 60, .mode = 3 },
	{ .width = 2560, .height = 1440, .framerates = 30, .mode = 1 },
	{ .width = 3840, .height = 2160, .framerates = 15, .mode = 0 },
	{ .width = 4208, .height = 3120, .framerates = 10, .mode = 0 },
};

static u32 ar1335_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

struct sensor_info {
    const u16 chip_id;
	const char *sensor_name;
	struct resolution *res_list;
	u32 res_list_size;
	u32 *code_list;
	u32 code_list_size;
};

static struct sensor_info tevs_sensor_table[] = {
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0144,
      .sensor_name = "TEVS-AR0144",
	  .res_list = ar0144_res_list,
	  .res_list_size = ARRAY_SIZE(ar0144_res_list),
	  .code_list = ar0144_code_list,
	  .code_list_size = ARRAY_SIZE(ar0144_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0145,
      .sensor_name = "TEVS-AR0145",
	  .res_list = ar0145_res_list,
	  .res_list_size = ARRAY_SIZE(ar0145_res_list),
	  .code_list = ar0145_code_list,
	  .code_list_size = ARRAY_SIZE(ar0145_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0234,
      .sensor_name = "TEVS-AR0234",
	  .res_list = ar0234_res_list,
	  .res_list_size = ARRAY_SIZE(ar0234_res_list),
	  .code_list = ar0234_code_list,
	  .code_list_size = ARRAY_SIZE(ar0234_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0521,
      .sensor_name = "TEVS-AR0521",
	  .res_list = ar0521_res_list,
	  .res_list_size = ARRAY_SIZE(ar0521_res_list),
	  .code_list = ar0521_code_list,
	  .code_list_size = ARRAY_SIZE(ar0521_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0522,
      .sensor_name = "TEVS-AR0522",
	  .res_list = ar0522_res_list,
	  .res_list_size = ARRAY_SIZE(ar0522_res_list),
	  .code_list = ar0522_code_list,
	  .code_list_size = ARRAY_SIZE(ar0522_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0821,
      .sensor_name = "TEVS-AR0821",
	  .res_list = ar0821_res_list,
	  .res_list_size = ARRAY_SIZE(ar0821_res_list),
	  .code_list = ar0821_code_list,
	  .code_list_size = ARRAY_SIZE(ar0821_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0822,
      .sensor_name = "TEVS-AR0822",
	  .res_list = ar0822_res_list,
	  .res_list_size = ARRAY_SIZE(ar0822_res_list),
	  .code_list = ar0822_code_list,
	  .code_list_size = ARRAY_SIZE(ar0822_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR1335,
      .sensor_name = "TEVS-AR1335",
	  .res_list = ar1335_res_list,
	  .res_list_size = ARRAY_SIZE(ar1335_res_list),
	  .code_list = ar1335_code_list,
	  .code_list_size = ARRAY_SIZE(ar1335_code_list) },
};

#endif //__SENSOR_TABLES_H__