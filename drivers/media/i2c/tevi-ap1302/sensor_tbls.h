#ifndef __AP1302_SENSOR_TABLES_H__
#define __AP1302_SENSOR_TABLES_H__

struct resolution {
	u16 width;
	u16 height;
	u16 framerates;
};

static struct resolution ar0144_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 60},
	{.width = 1280, .height = 800, .framerates = 60},
};

static struct resolution ar0234_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 120},
	{.width = 1920, .height = 1080, .framerates = 96},
	{.width = 1920, .height = 1200, .framerates = 87},
};

static struct resolution ar0521_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 60},
	{.width = 1920, .height = 1080, .framerates = 60},
	{.width = 2560, .height = 1440, .framerates = 40},
	{.width = 2592, .height = 1944, .framerates = 40},
};

static struct resolution ar0522_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 60},
	{.width = 1920, .height = 1080, .framerates = 60},
	{.width = 2560, .height = 1440, .framerates = 40},
	{.width = 2592, .height = 1944, .framerates = 40},
};

static struct resolution ar0821_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 30},
	{.width = 1920, .height = 1080, .framerates = 30},
	{.width = 2560, .height = 1440, .framerates = 30},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 24},
};

static struct resolution ar1335_res_list[] = {
	// {.width = 640, .height = 480, .framerates = 120},    //HD  480p
	{.width = 1280, .height = 720, .framerates = 30},    //HD  720p
	{.width = 1920, .height = 1080, .framerates = 30},   //FHD 1080p
	// {.width = 2560, .height = 1440, .framerates = 30},   //2K  1440p
	// {.width = 3840, .height = 2160, .framerates = 24},   //4K  2160p
	// {.width = 4192, .height = 3120, .framerates = 24},
	// {.width = 4208, .height = 3120, .framerates = 15},
};

static struct resolution ar1820_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 30},    //HD  720p
	{.width = 1920, .height = 1080, .framerates = 30},   //FHD 1080p
	{.width = 2560, .height = 1440, .framerates = 30},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 30},   //4K  2160p
	{.width = 4136, .height = 3102, .framerates = 20},
};

struct sensor_info {
	const char* sensor_name;
	const struct resolution *res_list;
	u32 res_list_size;
};

static struct sensor_info ap1302_sensor_table[] = {
	{
		.sensor_name = "TEVI-AR0144",
		.res_list = ar0144_res_list,
		.res_list_size = ARRAY_SIZE(ar0144_res_list)
	},
	{
		.sensor_name = "TEVI-AR0234",
		.res_list = ar0234_res_list,
		.res_list_size = ARRAY_SIZE(ar0234_res_list)
	},
	{
		.sensor_name = "TEVI-AR0521",
		.res_list = ar0521_res_list,
		.res_list_size = ARRAY_SIZE(ar0521_res_list)
	},
	{
		.sensor_name = "TEVI-AR0522",
		.res_list = ar0522_res_list,
		.res_list_size = ARRAY_SIZE(ar0522_res_list)
	},
	{
		.sensor_name = "TEVI-AR0821",
		.res_list = ar0821_res_list,
		.res_list_size = ARRAY_SIZE(ar0821_res_list)
	},
	{
		.sensor_name = "TEVI-AR0822",
		.res_list = ar0821_res_list,
		.res_list_size = ARRAY_SIZE(ar0821_res_list)
	},
	{
		.sensor_name = "TEVI-AR1335",
		.res_list = ar1335_res_list,
		.res_list_size = ARRAY_SIZE(ar1335_res_list)
	},
	{
		.sensor_name = "TEVI-AR1820",
		.res_list = ar1820_res_list,
		.res_list_size = ARRAY_SIZE(ar1820_res_list)
	},
};

#endif //__AP1302_SENSOR_TABLES_H__