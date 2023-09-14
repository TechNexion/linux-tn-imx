#ifndef __OTP_FLASH_H__
#define __OTP_FLASH_H__

#include "tevs_i2c.h"

/* Define host command register of MCU information page */
#define HOST_COMMAND_MCU_INFO_VERSION_MSB                       (0x0000)
#define HOST_COMMAND_MCU_INFO_VERSION_LSB                       (0x0002)
#define HOST_COMMAND_MCU_BOOT_STATE                             (0x0004)

/* Define host command register of ISP bootdata page */
#define HOST_COMMAND_ISP_BOOTDATA_1                             (0x1000)
#define HOST_COMMAND_ISP_BOOTDATA_2                             (0x1002)
#define HOST_COMMAND_ISP_BOOTDATA_3                             (0x1004)
#define HOST_COMMAND_ISP_BOOTDATA_4                             (0x1006)
#define HOST_COMMAND_ISP_BOOTDATA_5                             (0x1008)
#define HOST_COMMAND_ISP_BOOTDATA_6                             (0x100A)
#define HOST_COMMAND_ISP_BOOTDATA_7                             (0x100C)
#define HOST_COMMAND_ISP_BOOTDATA_8                             (0x100E)
#define HOST_COMMAND_ISP_BOOTDATA_9                             (0x1010)
#define HOST_COMMAND_ISP_BOOTDATA_10                            (0x1012)
#define HOST_COMMAND_ISP_BOOTDATA_11                            (0x1014)
#define HOST_COMMAND_ISP_BOOTDATA_12                            (0x1016)
#define HOST_COMMAND_ISP_BOOTDATA_13                            (0x1018)
#define HOST_COMMAND_ISP_BOOTDATA_14                            (0x101A)
#define HOST_COMMAND_ISP_BOOTDATA_15                            (0x101C)
#define HOST_COMMAND_ISP_BOOTDATA_16                            (0x101E)
#define HOST_COMMAND_ISP_BOOTDATA_17                            (0x1020)
#define HOST_COMMAND_ISP_BOOTDATA_18                            (0x1022)
#define HOST_COMMAND_ISP_BOOTDATA_19                            (0x1024)
#define HOST_COMMAND_ISP_BOOTDATA_20                            (0x1026)
#define HOST_COMMAND_ISP_BOOTDATA_21                            (0x1028)
#define HOST_COMMAND_ISP_BOOTDATA_22                            (0x102A)
#define HOST_COMMAND_ISP_BOOTDATA_23                            (0x102C)
#define HOST_COMMAND_ISP_BOOTDATA_24                            (0x102E)
#define HOST_COMMAND_ISP_BOOTDATA_25                            (0x1030)
#define HOST_COMMAND_ISP_BOOTDATA_26                            (0x1032)
#define HOST_COMMAND_ISP_BOOTDATA_27                            (0x1034)
#define HOST_COMMAND_ISP_BOOTDATA_28                            (0x1036)
#define HOST_COMMAND_ISP_BOOTDATA_29                            (0x1038)
#define HOST_COMMAND_ISP_BOOTDATA_30                            (0x103A)
#define HOST_COMMAND_ISP_BOOTDATA_31                            (0x103C)
#define HOST_COMMAND_ISP_BOOTDATA_32                            (0x103E)
#define HOST_COMMAND_ISP_BOOTDATA_33                            (0x1040)
#define HOST_COMMAND_ISP_BOOTDATA_34                            (0x1042)
#define HOST_COMMAND_ISP_BOOTDATA_35                            (0x1044)
#define HOST_COMMAND_ISP_BOOTDATA_36                            (0x1046)
#define HOST_COMMAND_ISP_BOOTDATA_37                            (0x1048)
#define HOST_COMMAND_ISP_BOOTDATA_38                            (0x104A)
#define HOST_COMMAND_ISP_BOOTDATA_39                            (0x104C)
#define HOST_COMMAND_ISP_BOOTDATA_40                            (0x104E)
#define HOST_COMMAND_ISP_BOOTDATA_41                            (0x1050)
#define HOST_COMMAND_ISP_BOOTDATA_42                            (0x1052)
#define HOST_COMMAND_ISP_BOOTDATA_43                            (0x1054)
#define HOST_COMMAND_ISP_BOOTDATA_44                            (0x1056)
#define HOST_COMMAND_ISP_BOOTDATA_45                            (0x1058)
#define HOST_COMMAND_ISP_BOOTDATA_46                            (0x105A)
#define HOST_COMMAND_ISP_BOOTDATA_47                            (0x105C)
#define HOST_COMMAND_ISP_BOOTDATA_48                            (0x105E)
#define HOST_COMMAND_ISP_BOOTDATA_49                            (0x1060)
#define HOST_COMMAND_ISP_BOOTDATA_50                            (0x1062)
#define HOST_COMMAND_ISP_BOOTDATA_51                            (0x1064)
#define HOST_COMMAND_ISP_BOOTDATA_52                            (0x1066)
#define HOST_COMMAND_ISP_BOOTDATA_53                            (0x1068)
#define HOST_COMMAND_ISP_BOOTDATA_54                            (0x106A)
#define HOST_COMMAND_ISP_BOOTDATA_55                            (0x106C)
#define HOST_COMMAND_ISP_BOOTDATA_56                            (0x106E)
#define HOST_COMMAND_ISP_BOOTDATA_57                            (0x1070)
#define HOST_COMMAND_ISP_BOOTDATA_58                            (0x1072)
#define HOST_COMMAND_ISP_BOOTDATA_59                            (0x1074)
#define HOST_COMMAND_ISP_BOOTDATA_60                            (0x1076)
#define HOST_COMMAND_ISP_BOOTDATA_61                            (0x1078)
#define HOST_COMMAND_ISP_BOOTDATA_62                            (0x107A)
#define HOST_COMMAND_ISP_BOOTDATA_63                            (0x107C)

/* Define host command register of ISP control page */
#define HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH                     (0x2000)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT                    (0x2002)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT                    (0x2004)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_SENSOR_MODE               (0x2006)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_THROUGHPUT                (0x2008)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS                   (0x200A)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_MSB        (0x200C)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_LSB        (0x200E)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_MSB          (0x2010)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_LSB          (0x2012)
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL                 (0x2014)
#define HOST_COMMAND_ISP_CTRL_AE_MODE                           (0x2016)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MSB                      (0x2018)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_LSB                      (0x201A)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_MSB                  (0x201C)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_LSB                  (0x201E)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_MSB                  (0x2020)
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_LSB                  (0x2022)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN                          (0x2024)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MAX                      (0x2026)
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MIN                      (0x2028)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_MSB              (0x202A)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_LSB              (0x202C)
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_GAIN                  (0x202E)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION            (0x2030)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MAX        (0x2032)
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MIN        (0x2034)
#define HOST_COMMAND_ISP_CTRL_AWB_MODE                          (0x2036)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP                          (0x2038)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MAX                      (0x203A)
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MIN                      (0x203C)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS                        (0x203E)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MAX                    (0x2040)
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MIN                    (0x2042)
#define HOST_COMMAND_ISP_CTRL_CONTRAST                          (0x2044)
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MAX                      (0x2046)
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MIN                      (0x2048)
#define HOST_COMMAND_ISP_CTRL_SATURATION                        (0x204A)
#define HOST_COMMAND_ISP_CTRL_SATURATION_MAX                    (0x204C)
#define HOST_COMMAND_ISP_CTRL_SATURATION_MIN                    (0x204E)
#define HOST_COMMAND_ISP_CTRL_GAMMA                             (0x2050)
#define HOST_COMMAND_ISP_CTRL_GAMMA_MAX                         (0x2052)
#define HOST_COMMAND_ISP_CTRL_GAMMA_MIN                         (0x2054)
#define HOST_COMMAND_ISP_CTRL_DENOISE                           (0x2056)
#define HOST_COMMAND_ISP_CTRL_DENOISE_MAX                       (0x2058)
#define HOST_COMMAND_ISP_CTRL_DENOISE_MIN                       (0x205A)
#define HOST_COMMAND_ISP_CTRL_SHARPEN                         	(0x205C)
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MAX                     	(0x205E)
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MIN                     	(0x2060)
#define HOST_COMMAND_ISP_CTRL_FLIP                              (0x2062)
#define HOST_COMMAND_ISP_CTRL_EFFECT                            (0x2064)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TYPE                         (0x2066)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES                        (0x2068)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MAX                    (0x206A)
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MIN                    (0x206C)
#define HOST_COMMAND_ISP_CTRL_CT_X                              (0x206E)
#define HOST_COMMAND_ISP_CTRL_CT_Y                              (0x2070)
#define HOST_COMMAND_ISP_CTRL_CT_MAX                            (0x2072)
#define HOST_COMMAND_ISP_CTRL_CT_MIN                            (0x2074)
#define HOST_COMMAND_ISP_CTRL_SYSTEM_START                      (0x2076)

#define DEFAULT_HEADER_VERSION 3

struct otp_flash {
	struct device *dev;
	struct nvmem_device *nvmem;
	char *product_name;
	void *header_data;
};

struct header_info {
	u8 header_version;
	u16 content_offset;
	u16 sensor_type;
	u8 sensor_fuseid[16];
	u8 product_name[64];
	u8 lens_id[16];
	u16 fix_checksum;
	u8 tn_fw_version[2];
	u16 vendor_fw_version;
	u16 custom_number;
	u8 build_year;
	u8 build_month;
	u8 build_day;
	u8 build_hour;
	u8 build_minute;
	u8 build_second;
	u16 mipi_datarate;
	u32 content_len;
	u16 content_checksum;
	u16 total_checksum;
} __attribute__((packed));

struct otp_flash *tevs_load_bootdata(struct i2c_client *client);

#endif
