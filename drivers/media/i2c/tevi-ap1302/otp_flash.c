#include <linux/i2c.h>
#include "otp_flash.h"

#ifdef __FAKE__
#include "bootdata.h"

struct otp_flash *ap1302_otp_flash_init(struct device *dev)
{
	return NULL;
}

u16 ap1302_otp_flash_get_checksum(struct otp_flash *instance)
{
	return BOOTDATA_CHECKSUM;
}

size_t ap1302_otp_flash_read(struct otp_flash *instance, u8 *data, int addr, size_t len)
{
	size_t l;

	l = len > BOOT_DATA_WRITE_LEN ? BOOT_DATA_WRITE_LEN : len;

	l = (BOOTDATA_TOTAL_SIZE - addr) < BOOT_DATA_WRITE_LEN ?
	       BOOTDATA_TOTAL_SIZE - addr : l;

	memmove(data, &((u8*)__bootdata__)[addr], l);
	return l;
}

size_t ap1302_otp_flash_get_pll_length(struct otp_flash *instance)
{
	return BOOTDATA_PLL_INIT_SIZE;
}

size_t ap1302_otp_flash_get_pll_section(struct otp_flash *instance, u8 *data)
{
	ap1302_otp_flash_read(instance, data, 0, BOOTDATA_PLL_INIT_SIZE);
	return BOOTDATA_PLL_INIT_SIZE;
}
#else
#include <linux/nvmem-consumer.h>

int get_flash_id(struct otp_flash *instance)
{
	struct device_node *flash_node;
	struct device *dev = instance->dev;
	int flash_id;
	int ret = 0;
	flash_node = of_parse_phandle(dev->of_node, "nvmem", 0);
	if (flash_node == NULL) {
		dev_err(dev, "missing nvmem handle\n");
		return -EINVAL;
	}

	ret = of_property_read_u32(flash_node, "reg", &flash_id);
	if (ret) {
		dev_err(dev, "invalid flash id on %pOF\n", flash_node);
		return ret;
	}
	dev_dbg(dev, "flash id: 0x%02x\n", flash_id);
	instance->flash_id = flash_id;

	return ret;
}

struct otp_flash *ap1302_otp_flash_init(struct device *dev)
{
	struct otp_flash *instance;
	u8 __header_ver;
	struct header_ver2 *header;
	struct header_ver3 *headerv3;
	
	instance = devm_kzalloc(dev, sizeof(struct otp_flash), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		return ERR_PTR(-EINVAL);
	}
	instance->dev = dev;

	get_flash_id(instance);

	instance->nvmem = devm_nvmem_device_get(dev, "calib-data");
	if (IS_ERR(instance->nvmem)) {
		dev_err(dev, "find not otp flash setting\n");
		goto fail1;
	}

	nvmem_device_read(instance->nvmem, 0, 1, &__header_ver);

	if (__header_ver == 2) {
		instance->header_data =
			devm_kzalloc(dev, sizeof(struct header_ver2),
				     GFP_KERNEL);

		nvmem_device_read(instance->nvmem,
				  0,
				  sizeof(struct header_ver2),
				  instance->header_data);

		header = instance->header_data;
		instance->product_name = header->product_name;
		dev_info(dev, "Product:%s, Version:%d. Lens:%s, Version:%d\n",
			 header->product_name,
			 header->product_version,
			 header->lens_name,
			 header->lens_version);

		dev_dbg(dev, "content ver: %d, content checksum: %x\n",
			header->content_version, header->content_checksum);
		dev_dbg(dev, "content length: %d, pll bootdata length: %d\n",
			header->content_len, header->pll_bootdata_len);

		return instance;
	} 
	else if(__header_ver == 3) {
		instance->header_data =
			devm_kzalloc(dev, sizeof(struct header_ver3),
				     GFP_KERNEL);

		nvmem_device_read(instance->nvmem,
				  0,
				  sizeof(struct header_ver3),
				  instance->header_data);

		headerv3 = instance->header_data;
		instance->product_name = headerv3->product_name;
		dev_info(dev, "Product:%s, HeaderVer:%d, Version:%d.%d.%d.%d, MIPI_Rate:%d\n",
			 headerv3->product_name,
			 headerv3->header_version,
			 headerv3->tn_fw_version[0],
			 headerv3->tn_fw_version[1],
			 headerv3->vendor_fw_version,
			 headerv3->custom_number,
			 headerv3->mipi_datarate);

		dev_dbg(dev, "content checksum: %x, content length: %d\n",
				headerv3->content_checksum, headerv3->content_len);

		return instance;
	}
	else {
		dev_err(dev, "can't recognize header version number '0x%X'\n",
			__header_ver);
	}

fail1:
	devm_kfree(dev, instance);
	return ERR_PTR(-EINVAL);
}

u16 ap1302_otp_flash_get_checksum(struct otp_flash *instance)
{
	struct header_ver2 *header;
	struct header_ver3 *headerv3;

	if( ((u8*)instance->header_data)[0] == 2 ) {
		header = (struct header_ver2 *)instance->header_data;
		return header->content_checksum;
	}
	else if(((u8*)instance->header_data)[0] == 3 ) {
		headerv3 = (struct header_ver3 *)instance->header_data;
		return headerv3->content_checksum;
	}

	return 0xffff;
}

size_t ap1302_otp_flash_read(struct otp_flash *instance, u8 *data, int addr, size_t len)
{
	u8 *temp;
	struct header_ver2 *header;
	struct header_ver3 *headerv3;
	size_t l;

	temp = (u8*)instance->header_data;
	if(temp[0] == 2) {
		header = (struct header_ver2 *)instance->header_data;
		l = len > BOOT_DATA_WRITE_LEN ? BOOT_DATA_WRITE_LEN : len;

		l = (header->content_len - addr) < BOOT_DATA_WRITE_LEN ?
			header->content_len - addr : l;

		nvmem_device_read(instance->nvmem,
				  addr + header->content_offset,
				  l,
				  data);
		return l;
	}
	else if(temp[0] == 3) {
		headerv3 = (struct header_ver3 *)instance->header_data;
		l = len > BOOT_DATA_WRITE_LEN ? BOOT_DATA_WRITE_LEN : len;

		l = (headerv3->content_len - addr) < BOOT_DATA_WRITE_LEN ?
			headerv3->content_len - addr : l;

		nvmem_device_read(instance->nvmem,
				  addr + headerv3->content_offset,
				  l,
				  data);
		return l;
	}

	return 0;
}

size_t ap1302_otp_flash_get_pll_length(struct otp_flash *instance)
{
	u8 *temp;
	struct header_ver2 *header;

	temp = (u8*)instance->header_data;
	if(temp[0] == 2) {
		header = (struct header_ver2 *)instance->header_data;
		return header->pll_bootdata_len;
	}

	return 0;
}

size_t ap1302_otp_flash_get_pll_section(struct otp_flash *instance, u8 *data)
{
	u8 *temp;
	struct header_ver2 *header;

	temp = (u8*)instance->header_data;
	if(temp[0] == 2) {
		header = (struct header_ver2 *)instance->header_data;
		if (header->pll_bootdata_len != 0) {
			ap1302_otp_flash_read(instance, data, 0,
					      header->pll_bootdata_len);
		}
		return header->pll_bootdata_len;
	}

	return 0;
}
#endif
