#ifndef __SENSOR_TABLES_H__
#define __SENSOR_TABLES_H__
#include <linux/kernel.h>
#include <media/v4l2-subdev.h>

#define SENSOR_CHIP_ID_NONE					0x0000
#define SENSOR_CHIP_ID_ONSEMI_AR0144		0x0356
#define SENSOR_CHIP_ID_ONSEMI_AR0145		0x1750
#define SENSOR_CHIP_ID_ONSEMI_AR0234		0x0A56
#define SENSOR_CHIP_ID_ONSEMI_AR0235		0x1850
#define SENSOR_CHIP_ID_ONSEMI_AR0246		0x1F56
#define SENSOR_CHIP_ID_ONSEMI_AR0521		0x0457
#define SENSOR_CHIP_ID_ONSEMI_AR0522		0x1457
#define SENSOR_CHIP_ID_ONSEMI_AR0544		0x0453
#define SENSOR_CHIP_ID_ONSEMI_AR0821		0x2557
#define SENSOR_CHIP_ID_ONSEMI_AR0822		0x0F56
#define SENSOR_CHIP_ID_ONSEMI_AR0830		0x0553
#define SENSOR_CHIP_ID_ONSEMI_AR1335		0x0153
#define SENSOR_CHIP_ID_ONSEMI_AR2020		0x0653

struct resolution {
	u16 width;
	u16 height;
	u16 *framerates;
	u16 framerates_size;
	u16 mode;
};

/* AR0144 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0144_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0144_framerates_640x480[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0144_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0144_framerates_1280x800[] = { 60, 30, 20, 15, 10, 5 };

static struct resolution ar0144_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0144_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0144_framerates_640x480),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0144_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0144_framerates_1280x720),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 800,
	  .framerates = ar0144_framerates_1280x800,
	  .framerates_size = ARRAY_SIZE(ar0144_framerates_1280x800),
	  .mode = 0 },
};

/* AR0145 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0145_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0145_framerates_640x480[] = { 115, 60, 30, 20, 15, 10, 5 };
static u16 ar0145_framerates_1280x720[] = { 115, 60, 30, 20, 15, 10, 5 };
static u16 ar0145_framerates_1280x800[] = { 115, 60, 30, 20, 15, 10, 5 };

static struct resolution ar0145_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0145_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0145_framerates_640x480),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0145_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0145_framerates_1280x720),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 800,
	  .framerates = ar0145_framerates_1280x800,
	  .framerates_size = ARRAY_SIZE(ar0145_framerates_1280x800),
	  .mode = 0 },
};

/* AR0234 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0234_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0234_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0234_framerates_1280x720[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0234_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0234_framerates_1920x1200[] = { 60, 30, 20, 15, 10, 5 };

static struct resolution ar0234_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0234_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0234_framerates_640x480),
	  .mode = 1 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0234_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0234_framerates_1280x720),
	  .mode = 0 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0234_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0234_framerates_1920x1080),
	  .mode = 0 },
	{ .width = 1920,
	  .height = 1200,
	  .framerates = ar0234_framerates_1920x1200,
	  .framerates_size = ARRAY_SIZE(ar0234_framerates_1920x1200),
	  .mode = 0 },
};

/* AR0235 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0235_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0235_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0235_framerates_1280x720[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0235_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0235_framerates_1920x1200[] = { 60, 30, 20, 15, 10, 5 };

static struct resolution ar0235_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0235_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0235_framerates_640x480),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0235_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0235_framerates_1280x720),
	  .mode = 0 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0235_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0235_framerates_1920x1080),
	  .mode = 0 },
	{ .width = 1920,
	  .height = 1200,
	  .framerates = ar0235_framerates_1920x1200,
	  .framerates_size = ARRAY_SIZE(ar0235_framerates_1920x1200),
	  .mode = 0 },
};

/* AR0246 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0246_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0246_framerates_640x480[] = { 30, 20, 15, 10, 5 };
static u16 ar0246_framerates_1280x720[] = { 30, 20, 15, 10, 5 };
static u16 ar0246_framerates_1920x1080[] = { 30, 20, 15, 10, 5 };

static struct resolution ar0246_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0246_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0246_framerates_640x480),
	  .mode = 0 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0246_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0246_framerates_1280x720),
	  .mode = 0 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0246_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0246_framerates_1920x1080),
	  .mode = 0 },
};

/* AR0521 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0521_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0521_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0521_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0521_framerates_1280x960[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0521_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0521_framerates_2560x1440[] = { 32, 30, 20, 15, 10, 5 };
static u16 ar0521_framerates_2592x1944[] = { 24, 20, 15, 10, 5 };

static struct resolution ar0521_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0521_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_640x480),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0521_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_1280x720),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 960,
	  .framerates = ar0521_framerates_1280x960,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_1280x960),
	  .mode = 3 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0521_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_1920x1080),
	  .mode = 1 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0521_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_2560x1440),
	  .mode = 1 },
	{ .width = 2592,
	  .height = 1944,
	  .framerates = ar0521_framerates_2592x1944,
	  .framerates_size = ARRAY_SIZE(ar0521_framerates_2592x1944),
	  .mode = 1 },
};

/* AR0522 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0522_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0522_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0522_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0522_framerates_1280x960[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0522_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0522_framerates_2560x1440[] = { 32, 30, 20, 15, 10, 5 };
static u16 ar0522_framerates_2592x1944[] = { 24, 20, 15, 10, 5 };

static struct resolution ar0522_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0522_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_640x480),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0522_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_1280x720),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 960,
	  .framerates = ar0522_framerates_1280x960,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_1280x960),
	  .mode = 3 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0522_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_1920x1080),
	  .mode = 1 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0522_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_2560x1440),
	  .mode = 1 },
	{ .width = 2592,
	  .height = 1944,
	  .framerates = ar0522_framerates_2592x1944,
	  .framerates_size = ARRAY_SIZE(ar0522_framerates_2592x1944),
	  .mode = 1 },
};

/* AR0544 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0544_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0544_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar0544_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0544_framerates_1280x960[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0544_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0544_framerates_2560x1440[] = { 32, 30, 20, 15, 10, 5 };
static u16 ar0544_framerates_2592x1944[] = { 24, 20, 15, 10, 5 };

static struct resolution ar0544_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0544_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_640x480),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0544_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_1280x720),
	  .mode = 2 },
	{ .width = 1280,
	  .height = 960,
	  .framerates = ar0544_framerates_1280x960,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_1280x960),
	  .mode = 2 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0544_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_1920x1080),
	  .mode = 2 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0544_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_2560x1440),
	  .mode = 0 },
	{ .width = 2592,
	  .height = 1944,
	  .framerates = ar0544_framerates_2592x1944,
	  .framerates_size = ARRAY_SIZE(ar0544_framerates_2592x1944),
	  .mode = 0 },
};

/* AR0821 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0821_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0821_framerates_640x480[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0821_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0821_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0821_framerates_2560x1440[] = { 30, 20, 15, 10, 5 };
static u16 ar0821_framerates_3840x2160[] = { 15, 10, 5 };

static struct resolution ar0821_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0821_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0821_framerates_640x480),
	  .mode = 2 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0821_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0821_framerates_1280x720),
	  .mode = 2 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0821_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0821_framerates_1920x1080),
	  .mode = 2 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0821_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0821_framerates_2560x1440),
	  .mode = 0 },
	{ .width = 3840,
	  .height = 2160,
	  .framerates = ar0821_framerates_3840x2160,
	  .framerates_size = ARRAY_SIZE(ar0821_framerates_3840x2160),
	  .mode = 0 },
};

/* AR0822 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0822_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0822_framerates_640x480[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0822_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0822_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0822_framerates_2560x1440[] = { 30, 20, 15, 10, 5 };
static u16 ar0822_framerates_3840x2160[] = { 15, 10, 5 };

static struct resolution ar0822_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0822_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0822_framerates_640x480),
	  .mode = 1 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0822_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0822_framerates_1280x720),
	  .mode = 1 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0822_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0822_framerates_1920x1080),
	  .mode = 1 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0822_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0822_framerates_2560x1440),
	  .mode = 0 },
	{ .width = 3840,
	  .height = 2160,
	  .framerates = ar0822_framerates_3840x2160,
	  .framerates_size = ARRAY_SIZE(ar0822_framerates_3840x2160),
	  .mode = 0 },
};

/* AR0830 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar0830_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar0830_framerates_640x480[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0830_framerates_1280x720[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0830_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar0830_framerates_2560x1440[] = { 30, 20, 15, 10, 5 };
static u16 ar0830_framerates_3840x2160[] = { 15, 10, 5 };

static struct resolution ar0830_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar0830_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar0830_framerates_640x480),
	  .mode = 3 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar0830_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar0830_framerates_1280x720),
	  .mode = 2 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar0830_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar0830_framerates_1920x1080),
	  .mode = 2 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar0830_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar0830_framerates_2560x1440),
	  .mode = 1 },
	{ .width = 3840,
	  .height = 2160,
	  .framerates = ar0830_framerates_3840x2160,
	  .framerates_size = ARRAY_SIZE(ar0830_framerates_3840x2160),
	  .mode = 1 },
};

/* AR1335 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar1335_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

static u16 ar1335_framerates_640x480[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar1335_framerates_1280x720[] = { 120, 60, 30, 20, 15, 10, 5 };
static u16 ar1335_framerates_1920x1080[] = { 60, 30, 20, 15, 10, 5 };
static u16 ar1335_framerates_2560x1440[] = { 30, 20, 15, 10, 5 };
static u16 ar1335_framerates_3840x2160[] = { 15, 10, 5 };
static u16 ar1335_framerates_4208x3120[] = { 10, 5 };

static struct resolution ar1335_res_list[] = {
	{ .width = 640,
	  .height = 480,
	  .framerates = ar1335_framerates_640x480,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_640x480),
	  .mode = 4 },
	{ .width = 1280,
	  .height = 720,
	  .framerates = ar1335_framerates_1280x720,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_1280x720),
	  .mode = 4 },
	{ .width = 1920,
	  .height = 1080,
	  .framerates = ar1335_framerates_1920x1080,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_1920x1080),
	  .mode = 3 },
	{ .width = 2560,
	  .height = 1440,
	  .framerates = ar1335_framerates_2560x1440,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_2560x1440),
	  .mode = 1 },
	{ .width = 3840,
	  .height = 2160,
	  .framerates = ar1335_framerates_3840x2160,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_3840x2160),
	  .mode = 0 },
	{ .width = 4208,
	  .height = 3120,
	  .framerates = ar1335_framerates_4208x3120,
	  .framerates_size = ARRAY_SIZE(ar1335_framerates_4208x3120),
	  .mode = 0 },
};

/* AR2020 default setting for 4 data lanes and data frequency 800 MHz */
static u32 ar2020_code_list[] = {
	MEDIA_BUS_FMT_UYVY8_1X16,
};

// static u16 ar2020_framerates_640x480[] = { 120, 60, 30, 20, 15, 10, 5 };
// static u16 ar2020_framerates_1280x720[] = { 120, 60, 30, 20, 15, 10, 5 };
// static u16 ar2020_framerates_1280x960[] = { 100, 60, 30, 20, 15, 10, 5 };
static u16 ar2020_framerates_1920x1440[] = { 40, 30, 20, 15, 10, 5 };
static u16 ar2020_framerates_2560x1920[] = { 25, 20, 15, 10, 5 };
static u16 ar2020_framerates_4208x3156[] = { 10, 5 };

static struct resolution ar2020_res_list[] = {
	// { .width = 640,
	//   .height = 480,
	//   .framerates = ar2020_framerates_640x480,
	//   .framerates_size = ARRAY_SIZE(ar2020_framerates_640x480),
	//   .mode = 3 },
	// { .width = 1280,
	//   .height = 720,
	//   .framerates = ar2020_framerates_1280x720,
	//   .framerates_size = ARRAY_SIZE(ar2020_framerates_1280x720),
	//   .mode = 3 },
	// { .width = 1280,
	//   .height = 960,
	//   .framerates = ar2020_framerates_1280x960,
	//   .framerates_size = ARRAY_SIZE(ar2020_framerates_1280x960),
	//   .mode = 3 },
	{ .width = 1920,
	  .height = 1440,
	  .framerates = ar2020_framerates_1920x1440,
	  .framerates_size = ARRAY_SIZE(ar2020_framerates_1920x1440),
	  .mode = 2 },
	{ .width = 2560,
	  .height = 1920,
	  .framerates = ar2020_framerates_2560x1920,
	  .framerates_size = ARRAY_SIZE(ar2020_framerates_2560x1920),
	  .mode = 2 },
	{ .width = 4208,
	  .height = 3156,
	  .framerates = ar2020_framerates_4208x3156,
	  .framerates_size = ARRAY_SIZE(ar2020_framerates_4208x3156),
	  .mode = 0 },
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
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0235,
	  .sensor_name = "TEVS-AR0235",
	  .res_list = ar0235_res_list,
	  .res_list_size = ARRAY_SIZE(ar0235_res_list),
	  .code_list = ar0235_code_list,
	  .code_list_size = ARRAY_SIZE(ar0235_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0246,
	  .sensor_name = "TEVS-AR0246",
	  .res_list = ar0246_res_list,
	  .res_list_size = ARRAY_SIZE(ar0246_res_list),
	  .code_list = ar0246_code_list,
	  .code_list_size = ARRAY_SIZE(ar0246_code_list) },
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
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0544,
	  .sensor_name = "TEVS-AR0544",
	  .res_list = ar0544_res_list,
	  .res_list_size = ARRAY_SIZE(ar0544_res_list),
	  .code_list = ar0544_code_list,
	  .code_list_size = ARRAY_SIZE(ar0544_code_list) },
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
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR0830,
	  .sensor_name = "TEVS-AR0830",
	  .res_list = ar0830_res_list,
	  .res_list_size = ARRAY_SIZE(ar0830_res_list),
	  .code_list = ar0830_code_list,
	  .code_list_size = ARRAY_SIZE(ar0830_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR1335,
	  .sensor_name = "TEVS-AR1335",
	  .res_list = ar1335_res_list,
	  .res_list_size = ARRAY_SIZE(ar1335_res_list),
	  .code_list = ar1335_code_list,
	  .code_list_size = ARRAY_SIZE(ar1335_code_list) },
	{ .chip_id = SENSOR_CHIP_ID_ONSEMI_AR2020,
	  .sensor_name = "TEVS-AR2020",
	  .res_list = ar2020_res_list,
	  .res_list_size = ARRAY_SIZE(ar2020_res_list),
	  .code_list = ar2020_code_list,
	  .code_list_size = ARRAY_SIZE(ar2020_code_list) },
};

#endif //__SENSOR_TABLES_H__