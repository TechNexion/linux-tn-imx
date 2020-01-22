/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_AXONFABRIC_CORE_H
#define _LINUX_AXONFABRIC_CORE_H

#include <linux/types.h>

struct axonf_chip;

extern int axonf_iob_read_single(struct axonf_chip *chip, int reg, u8 *val,
				int off);
extern int axonf_iob_write_single(struct axonf_chip *chip, int reg, u8 val,
				int off);
extern int axonf_iob_read_reg(struct axonf_chip *chip, int reg, u8 *val,
				int bank);
extern int axonf_iob_write_reg(struct axonf_chip *chip, int reg, u8 val, int bank);

#endif /* _LINUX_AXONFABRIC_CORE_H */
