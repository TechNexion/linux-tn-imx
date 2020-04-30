/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Utility functions for CAAM exported
 *
 * Copyright 2018 NXP
 */

#ifndef _CAAM_UTIL_H_
#define _CAAM_UTIL_H_

#include <linux/device.h>

#include "caam.h"
#include "regs.h"
#include "caam_desc.h"

extern int prepare_write_data(struct device *jrdev, const u8 *data, size_t size,
			      caam_dma_addr_t *dma_addr, u8 **allocated_data);

extern void unprepare_write_data(struct device *jrdev,
				 caam_dma_addr_t dma_addr,
				 u8 *allocated_data, size_t size);

extern int prepare_read_data(struct device *jrdev, size_t size,
			     caam_dma_addr_t *dma_addr, u8 **allocated_data);

extern void read_data_prepared(struct device *jrdev, u8 *data,
			       caam_dma_addr_t dma_addr,
			       u8 *allocated_data, size_t size);

extern void unprepare_read_data(struct device *jrdev, caam_dma_addr_t dma_addr,
				u8 *allocated_data, size_t size);

extern caam_dma_addr_t get_caam_dma_addr(const void *address);

extern int caam_black_key(struct device *jrdev,
			  const void *key, size_t key_length, u8 key_memtype,
			  void *black_key, size_t *black_key_length,
			  u8 black_key_memtype, u8 keyauth, u8 trusted_key);

extern int caam_random_black_key(struct device *jrdev,
				 size_t key_length,
				 void *black_key, size_t *black_key_length,
				 u8 black_key_memtype, u8 keyauth,
				 u8 trusted_key);

extern int caam_blob_encap(struct device *jrdev,
			   const void  *secret, size_t secret_length,
			   u8 secret_memtype,
			   u8 keycolor, size_t key_length_in_secret,
			   u8 auth, u8 trusted_key,
			   const void *keymod, size_t *keymod_length,
			   u8 keymod_memtype,
			   void *blob, size_t *blob_length,
			   u8 blob_memtype, u8 blobcolor);

extern int caam_blob_decap(struct device *jrdev,
			   const void *blob, size_t blob_length,
			   u8 blob_memtype,
			   u8 blobcolor,
			   const void *keymod, size_t *keymod_length,
			   u8 keymod_memtype,
			   void *secret, size_t *secret_length,
			   u8 secret_memtype,
			   u8 keycolor, size_t *key_length_in_secret,
			   u8 auth, u8 trusted_key);

/**
 * @brief      Determines if key color is valid
 *
 * @param[in]  keycolor  The keycolor
 *
 * @return     True if valid, False otherwise.
 */
static inline bool is_key_color(u8 keycolor)
{
	return (keycolor == BLACK_KEY) || (keycolor == RED_KEY);
}

/**
 * @brief      Determines if the memory type is valid
 *
 * @param[in]  memtype  The memory type
 *
 * @return     True if valid, False otherwise.
 */
static inline bool is_memory_type(u8 memtype)
{
	return (memtype == DATA_GENMEM) ||
		(memtype == DATA_DMAMEM) ||
		(memtype == DATA_SECMEM);
}

/**
 * @brief      Determines if the blob color is valid
 *
 * @param[in]  blobcolor  The blob color
 *
 * @return     True if valid, False otherwise.
 */
static inline bool is_blob_color(u8 blobcolor)
{
	return (blobcolor == RED_BLOB) || (blobcolor == BLACK_BLOB);
}

/**
 * @brief      Determines if the auth is valid
 *
 * @param[in]  auth  The authentication
 *
 * @return     True if valid, False otherwise.
 */
static inline bool is_auth(u8 auth)
{
	return (auth == KEY_COVER_ECB) || (auth == KEY_COVER_CCM);
}

/**
 * @brief      Determines if trusted key value is valid
 *
 * @param[in]  trusted_key  The trusted key value
 *
 * @return     True if valid, False otherwise.
 */
static inline bool is_trusted_key(u8 trusted_key)
{
	return (trusted_key == UNTRUSTED_KEY) || (trusted_key == TRUSTED_KEY);
}

#endif /* _CAAM_UTIL_H_ */
