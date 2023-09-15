#ifndef __AP1302_SENSOR_TABLES_H__
#define __AP1302_SENSOR_TABLES_H__

struct sensor_params {
	u16 width;
	u16 height;
	u16 framerates;
	u16 mode;
};

static struct sensor_params ar0144_res_list[] = {
	{.width = 640, .height = 480, .framerates = 60, .mode = 0},
	{.width = 1280, .height = 720, .framerates = 60, .mode = 0},
	{.width = 1280, .height = 800, .framerates = 60, .mode = 0},
};

static struct sensor_params ar0234_res_list[] = {
	{.width = 640, .height = 480, .framerates = 120, .mode = 1},
	{.width = 1280, .height = 720, .framerates = 120, .mode = 0},
	{.width = 1920, .height = 1080, .framerates = 85, .mode = 0},
	{.width = 1920, .height = 1200, .framerates = 60, .mode = 0},
};

static struct sensor_params ar0521_res_list[] = {
	{.width = 640, .height = 480, .framerates = 60, .mode = 3},
	{.width = 1280, .height = 720, .framerates = 60, .mode = 3},
	{.width = 1280, .height = 960, .framerates = 60, .mode = 3},
	{.width = 1920, .height = 1080, .framerates = 60, .mode = 1},
	{.width = 2560, .height = 1440, .framerates = 30, .mode = 1},
	{.width = 2592, .height = 1944, .framerates = 30, .mode = 1},
};

static struct sensor_params ar0522_res_list[] = {
	{.width = 640, .height = 480, .framerates = 60, .mode = 3},
	{.width = 1280, .height = 720, .framerates = 60, .mode = 3},
	{.width = 1920, .height = 1080, .framerates = 60, .mode = 1},
	{.width = 2560, .height = 1440, .framerates = 40, .mode = 1},
	{.width = 2592, .height = 1944, .framerates = 40, .mode = 1},
};

static struct sensor_params ar0821_res_list[] = {
	{.width = 640, .height = 480, .framerates = 30, .mode = 2},
	{.width = 1280, .height = 720, .framerates = 30, .mode = 2},
	{.width = 1920, .height = 1080, .framerates = 30, .mode = 2},
	{.width = 2560, .height = 1440, .framerates = 30, .mode = 0},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 20, .mode = 0},
};

static struct sensor_params ar0822_res_list[] = {
	{.width = 640, .height = 480, .framerates = 30, .mode = 1},
	{.width = 1280, .height = 720, .framerates = 30, .mode = 1},
	{.width = 1920, .height = 1080, .framerates = 30, .mode = 1},
	{.width = 2560, .height = 1440, .framerates = 30, .mode = 0},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 20, .mode = 0},
};

static struct sensor_params ar1335_res_list[] = {
	{.width = 640, .height = 480, .framerates = 60, .mode = 4},     //VGA  480p
	{.width = 1280, .height = 720, .framerates = 60, .mode = 4},    //HD  720p
	{.width = 1920, .height = 1080, .framerates = 60, .mode = 3},   //FHD 1080p
	{.width = 2560, .height = 1440, .framerates = 30, .mode = 1},   //2K  1440p
	{.width = 3840, .height = 2160, .framerates = 20, .mode = 0},   //4K  2160p
	{.width = 4208, .height = 3120, .framerates = 15, .mode = 0},   //13M
};

struct sensor_info {
	const char* sensor_name;
	const struct sensor_params *res_list;
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
		.res_list = ar0822_res_list,
		.res_list_size = ARRAY_SIZE(ar0822_res_list)
	},
	{
		.sensor_name = "TEVI-AR1335",
		.res_list = ar1335_res_list,
		.res_list_size = ARRAY_SIZE(ar1335_res_list)
	},
};

#endif //__AP1302_SENSOR_TABLES_H__