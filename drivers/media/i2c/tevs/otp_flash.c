#include "otp_flash.h"

struct otp_flash *tevs_load_bootdata(struct i2c_client *client)
{
	struct otp_flash *instance;
	struct device *dev = &client->dev;
	u8 header_ver;
	struct header_info *header;

	instance = devm_kzalloc(dev, sizeof(struct otp_flash), GFP_KERNEL);
	if (instance == NULL) {
		dev_err(dev, "allocate memory failed\n");
		goto fail1;
	}
	instance->dev = dev;

	tevs_i2c_read(client, HOST_COMMAND_ISP_BOOTDATA_1, &header_ver, 1);

	if (header_ver == DEFAULT_HEADER_VERSION) {
		instance->header_data = devm_kzalloc(
			dev, sizeof(struct header_info), GFP_KERNEL);

		tevs_i2c_read(client, HOST_COMMAND_ISP_BOOTDATA_1,
				instance->header_data,
				sizeof(struct header_info));

		header = instance->header_data;
		instance->product_name = header->product_name;
		dev_info(
			dev,
			"Product:%s, HeaderVer:%d, Version:%d.%d.%d.%d, MIPI_Rate:%d\n",
			header->product_name, header->header_version,
			header->tn_fw_version[0], header->tn_fw_version[1],
			header->vendor_fw_version, header->custom_number,
			header->mipi_datarate);

		dev_dbg(dev, "content checksum: %x, content length: %d\n",
			header->content_checksum, header->content_len);

		return instance;
	} else {
		dev_err(dev, "can't recognize header version number '0x%X'\n",
			header_ver);
	}

fail1:
	devm_kfree(dev, instance);
	return ERR_PTR(-EINVAL);
}
