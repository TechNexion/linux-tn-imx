#ifndef __OTP_FLASH_H__
#define __OTP_FLASH_H__

#include <linux/i2c.h>

#define BOOT_DATA_START_REG (0x8000)
#define BOOT_DATA_END_REG (0x9FFF)
#define BOOT_DATA_WRITE_LEN (BOOT_DATA_END_REG - BOOT_DATA_START_REG + 1)

struct otp_flash {
	struct device *dev;
	struct nvmem_device *nvmem;
	char* product_name;
	void *header_data;
};

/*
header Version 2 : {
	uint8 header_version
	uint16 content_offset //content_offset may not be equal to the size of header
	uint8 product_name[64]
	uint8 product_version
	uint8 lens_name[64]
	uint8 lens_version
	uint8 content_version
	uint32 content_checksum
	uint32 content_len
	uint16 pll_bootdata_len
}
*/
struct header_ver2 {
	u8 header_version;
	u16 content_offset;
	u8 product_name[64];
	u8 product_version;
	u8 lens_name[64];
	u8 lens_version;
	u8 content_version;
	u32 content_checksum;
	u32 content_len;
	u16 pll_bootdata_len;
} __attribute__((packed));

// struct VzHeaderV3 {   // Header Version 3
// 	// Fixed Area
	// uint8_t header_version;
	// uint16_t content_offset;
	// uint16_t sensor_type;
	// uint8_t sensor_fuseid[16];
	// uint8_t product_name[64];
	// uint8_t lens_id[16];
	// uint16_t fix_checksum;
	// uint8_t tn_fw_version[2];
	// uint16_t vendor_fw_version;
	// uint16_t custom_number;
	// uint8_t build_year;
	// uint8_t build_month;
	// uint8_t build_day;
	// uint8_t build_hour;
	// uint8_t build_minute;
	// uint8_t build_second;
	// uint16_t mipi_datarate;
	// uint32_t content_len;
	// uint16_t content_checksum;
	// uint16_t total_checksum;
// };

struct header_ver3 {
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

struct otp_flash;
struct otp_flash *ap1302_otp_flash_init(struct device *dev);
u16 ap1302_otp_flash_get_checksum(struct otp_flash *instance);
size_t ap1302_otp_flash_read(struct otp_flash *instance, u8 *data, int addr, size_t len);
size_t ap1302_otp_flash_get_pll_length(struct otp_flash *instance);
size_t ap1302_otp_flash_get_pll_section(struct otp_flash*instance, u8 *data);

#endif
