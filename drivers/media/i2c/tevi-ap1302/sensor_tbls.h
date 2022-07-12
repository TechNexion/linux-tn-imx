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
	{.width = 1280, .height = 720, .framerates = 60},
	{.width = 1920, .height = 1080, .framerates = 60},
	{.width = 1920, .height = 1200, .framerates = 60},
};

static struct resolution ar0521_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 30},
	{.width = 1920, .height = 1080, .framerates = 30},
	{.width = 2592, .height = 1944, .framerates = 25},
};

static struct resolution ar0821_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 30},
	{.width = 1920, .height = 1080, .framerates = 30},
	{.width = 2560, .height = 1440, .framerates = 25},
	{.width = 3840, .height = 2160, .framerates = 20},
};

static struct resolution ar1335_res_list[] = {
	{.width = 1280, .height = 720, .framerates = 30},    //HD  720p
	{.width = 1920, .height = 1080, .framerates = 30},   //FHD 1080p
	{.width = 2560, .height = 1440, .framerates = 25},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 20},   //4K  2160p
	{.width = 4192, .height = 3120, .framerates = 20},
	{.width = 4208, .height = 3120, .framerates = 20},
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
		.res_list = ar0521_res_list,
		.res_list_size = ARRAY_SIZE(ar0521_res_list)
	},
	{
		.sensor_name = "TEVI-AR0821",
		.res_list = ar0821_res_list,
		.res_list_size = ARRAY_SIZE(ar0821_res_list)
	},
	{
		.sensor_name = "TEVI-AR1335",
		.res_list = ar1335_res_list,
		.res_list_size = ARRAY_SIZE(ar1335_res_list)
	},
};

#endif //__AP1302_SENSOR_TABLES_H__