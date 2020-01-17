/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_AXONFABRIC_H
#define _LINUX_AXONFABRIC_H

#include <linux/types.h>
#include <linux/i2c.h>

/* platform data for the Axon Fabric driver */

struct axonf_platform_data {
	/* number of the first GPIO */
	unsigned	gpio_base;

	/* initial polarity inversion setting */
	u32		invert;

	/* interrupt base */
	int		irq_base;

	void		*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	int		(*teardown)(struct i2c_client *client,
				unsigned gpio, unsigned ngpio,
				void *context);
	const char	*const *names;
};

#define AXONF_ADDR_MAGIC   0x0000
#define AXONF_ADDR_VERSION 0x0004
#define AXONF_ADDR_CONFIG  0x0008
#define AXONF_ADDR_CTRLREG 0x0010
#define AXONF_ADDR_RGBLED  0x0020
#define AXONF_ADDR_IOBLOCK 0x0100

#endif /* _LINUX_AXONFABRIC_H */
