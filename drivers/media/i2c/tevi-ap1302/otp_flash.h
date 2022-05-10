#ifndef __OTP_FLASH_H__
#define __OTP_FLASH_H__

#include <linux/i2c.h>

#define BOOT_DATA_START_REG (0x8000)
#define BOOT_DATA_END_REG (0x9FFF)
#define BOOT_DATA_WRITE_LEN (BOOT_DATA_END_REG - BOOT_DATA_START_REG + 1)

struct otp_flash {
	struct device *dev;
	struct nvmem_device *nvmem;
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

struct otp_flash;
struct otp_flash *ap1302_otp_flash_init(struct device *dev);
u16 ap1302_otp_flash_get_checksum(struct otp_flash *instance);
size_t ap1302_otp_flash_read(struct otp_flash *instance, u8 *data, int addr, size_t len);
size_t ap1302_otp_flash_get_pll_length(struct otp_flash *instance);
size_t ap1302_otp_flash_get_pll_section(struct otp_flash*instance, u8 *data);

#endif
