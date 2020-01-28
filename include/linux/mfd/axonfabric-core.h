/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_AXONFABRIC_CORE_H
#define _LINUX_AXONFABRIC_CORE_H

#include <linux/types.h>

struct axonf_chip;

extern int axonf_reg_read(struct axonf_chip *chip, unsigned short reg);
extern int axonf_bulk_read(struct axonf_chip *chip, unsigned short reg,
		     int count, u8 *buf);
extern int axonf_reg_write(struct axonf_chip *chip, unsigned short reg,
		    u8 val);
extern int axonf_bulk_write(struct axonf_chip *chip, unsigned short reg,
			int count, u8 *buf);
extern int axonf_reg_update_bits(struct axonf_chip *chip, unsigned short reg,
			u8 mask, u8 val);

#endif /* _LINUX_AXONFABRIC_CORE_H */
