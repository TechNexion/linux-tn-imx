#ifndef __VIZIONLINK_CAM_H__
#define __VIZIONLINK_CAM_H__

#include <linux/i2c.h>

extern int vc_init(struct i2c_client *client, u8 ser_alias_addr);
extern int vc_configure_ser_csi(struct i2c_client *client, u8 ser_alias_addr);

#endif
