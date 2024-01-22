#include <linux/i2c.h>
#include "otp_flash.h"

struct otp_flash *ap1302_load_bootdata(struct i2c_client *client)
{
	struct otp_flash *instance;
	struct device *dev = &client->dev;
	u8 __header_ver;
	struct header_ver2 *header;
	struct header_ver3 *headerv3;

	instance = devm_kzalloc(dev, sizeof(struct otp_flash), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		goto fail1;
	}
	instance->dev = dev;

	sensor_i2c_read(client, HOST_COMMAND_ISP_BOOTDATA_1, &__header_ver, 1);

	if (__header_ver == 2) {
		instance->header_data =
			devm_kzalloc(dev, sizeof(struct header_ver2),
				     GFP_KERNEL);

		sensor_i2c_read(client, HOST_COMMAND_ISP_BOOTDATA_1, instance->header_data, sizeof(struct header_ver2));

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

		sensor_i2c_read(client, HOST_COMMAND_ISP_BOOTDATA_1, instance->header_data, sizeof(struct header_ver3));

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

