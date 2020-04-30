/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Shared descriptors for caam black key and blob
 *
 * Copyright 2016 NXP
 */

#ifndef CAAM_DESC_H
#define CAAM_DESC_H

#include <linux/types.h>

#include "compat.h"
#include "regs.h"

/* Define treatment of secure memory vs. general memory blobs */
#define DATA_GENMEM 0
#define DATA_DMAMEM 1
#define DATA_SECMEM 2

/* Define treatment of red/black keys */
#define RED_KEY 0 /* Clear key */
#define BLACK_KEY 1 /* Encrypted key */

/* Definition of the different type of blob */
#define RED_BLOB 0
#define BLACK_BLOB 1

/* Define key encryption/covering options */
#define KEY_COVER_ECB 0	/* cover key in AES-ECB */
#define KEY_COVER_CCM 1 /* cover key with AES-CCM */

/* Define the trust in the key, to select either JDKEK or TDKEK */
#define UNTRUSTED_KEY 0
#define TRUSTED_KEY 1

/* Define space required for BKEK + MAC tag storage in any blob */
#define BLOB_OVERHEAD (32 + 16)

#define PAD_16_BYTE(_key_size) (roundup(_key_size, 16))

/* An ECB black key is a multiple of 16 byte, at least the size of the key */
#define ECB_BLACK_KEY_SIZE(_key_size) (PAD_16_BYTE(_key_size))

#define PAD_8_BYTE(_key_size) (roundup(_key_size, 8))

/* A CCM Black key is a multiple of 8 byte, at least the size of the key
 * plus 6 byte for the nonce and 6 byte for the IV
 */
#define NONCE_SIZE 6
#define IV_SIZE 6
#define CCM_OVERHEAD (NONCE_SIZE + IV_SIZE)
#define CCM_BLACK_KEY_SIZE(_key_size) (PAD_8_BYTE(_key_size) \
							+ CCM_OVERHEAD)

static inline int secret_size_in_ccm_black_key(int key_size)
{
	return ((key_size >= CCM_OVERHEAD) ? key_size - CCM_OVERHEAD : 0);
}

#define SECRET_SIZE_IN_CCM_BLACK_KEY(_key_size) \
	secret_size_in_ccm_black_key(_key_size)

/* A red key is unencrypted so its size is the same */
#define RED_KEY_SIZE(_key_size) (_key_size)

/* Keymod size is 8 byte for blob created in secure memory, else 16 */
#define KEYMOD_SIZE_SM 8
#define KEYMOD_SIZE_GM 16

/* Create job descriptor to cover key */
extern int cnstr_black_key_jobdesc(u32 **desc, caam_dma_addr_t key,
				   size_t key_length,
				   caam_dma_addr_t cover_key,
				   size_t cover_key_length,
				   u8 auth, u8 trusted_key);

/* Create job descriptor to generate a random key and cover it */
extern int cnstr_random_black_key_jobdesc(u32 **desc, size_t key_length,
					  caam_dma_addr_t cover_key,
					  size_t cover_key_length,
					  u8 auth, u8 trusted_key);

/* Encapsulate data in a blob */
extern int cnstr_blob_encap_jobdesc(u32 **desc,
				    caam_dma_addr_t secret,
				    size_t secret_length,
				    u8 keycolor, u8 auth,  u8 trusted_key,
				    u8 memtype,
				    caam_dma_addr_t keymod,
				    size_t keymod_length,
				    caam_dma_addr_t blob, size_t blob_length,
				    u8 blobcolor);

/* Decapsulate data from a blob */
extern int cnstr_blob_decap_jobdesc(u32 **desc,
				    caam_dma_addr_t blob, size_t blob_length,
				    u8 blobcolor,
				    caam_dma_addr_t keymod,
				    size_t keymod_length,
				    caam_dma_addr_t secret,
				    size_t secret_length,
				    u8 keycolor, u8 auth, u8 trusted_key,
				    u8 memtype);

extern int cnstr_key_jobdesc(u32 **desc,
			     caam_dma_addr_t key, size_t key_size,
			     size_t key_length,
			     u32 key_class, u32 enc,
			     u32 nwb, u32 ekt, u32 kdest);

extern int cnstr_key_imm_jobdesc(u32 **desc,
				 void *key, size_t key_size, size_t key_length,
				 u32 key_class, u32 enc,
				 u32 nwb, u32 ekt, u32 kdest);

#endif /* CAAM_DESC_H */
