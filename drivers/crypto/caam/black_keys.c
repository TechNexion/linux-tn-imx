// SPDX-License-Identifier: GPL-2.0
/*
 * Key covering function for CAAM
 *
 * Copyright 2018 NXP
 */

#include "caam.h"
#include "desc.h"
#include "caam_desc.h"
#include "caam_util.h"
#include "desc_constr.h"

/**
 * @brief         Cover a key and create a black key
 *
 * @details       Details:
 * - If @b memtype is setted to @b DATA_SECMEM, the key to cover is read
 *   directly by the CAAM from Secure Memeory without intermediate copy. The
 *   value of @b key must be a physical address in secure memory.
 * - The parameter @b black_key_length is used to check the output memory can
 *   store the covered key and inform the user of the size of the covered key.
 *
 * @attention     Limited to class 1 keys at the present time
 *
 * @attention     The input and output data are copied to temporary arrays
 *                except for @b key if @b memtype is @b DATA_SECMEM.
 *
 * @param[in]     jrdev              The jrdev
 * @param[in]     key                The key
 * @param[in]     key_length         The key length
 * @param[in]     key_memtype        The memtype
 * @param[out]    black_key          The black key
 * @param[in,out] black_key_length   The black key length
 * @param[in]     black_key_memtype  The black key memtype
 * @param[in]     keyauth            The keyauth
 * @param[in]     trusted_key        The trusted key
 *
 * @return        0 if no error, otherwise error code
 */
int caam_black_key(struct device *jrdev,
		   const void *key, size_t key_length, u8 key_memtype,
		   void *black_key, size_t *black_key_length,
		   u8 black_key_memtype, u8 keyauth, u8 trusted_key)
{
	int retval = 0;
	u32 dsize, jstat;
	u32 __iomem *coverdesc = NULL;
	size_t black_key_length_req = 0;

	u8 *temp_key = NULL;
	caam_dma_addr_t key_dma;
	u8 *temp_black_key = NULL;
	caam_dma_addr_t black_key_dma;

	/* Check input */
	if (!jrdev)
		return -EINVAL;

#ifdef DEBUG
	dev_info(jrdev, "%s input: [key: %p(%zu) black_key: %p(%zu), auth: %x]",
		 __func__, key, key_length,
		 black_key, *black_key_length, keyauth);
#endif

	/* Trusted key not supported */
	if (trusted_key == TRUSTED_KEY)
		return -EINVAL;

	if (!key || !black_key)
		return -EINVAL;

	if (!is_memory_type(key_memtype) ||
	    !is_memory_type(black_key_memtype) ||
	    !is_auth(keyauth) ||
	    !is_trusted_key(trusted_key))
		return -EINVAL;

	/* Advanced tests */
	if (keyauth == KEY_COVER_ECB) {
		black_key_length_req = ECB_BLACK_KEY_SIZE(key_length);
		if (*black_key_length < black_key_length_req) {
			*black_key_length = black_key_length_req;
			return -EINVAL;
		}
	} else if (keyauth == KEY_COVER_CCM) {
		black_key_length_req = CCM_BLACK_KEY_SIZE(key_length);
		if (*black_key_length < black_key_length_req) {
			*black_key_length = black_key_length_req;
			return -EINVAL;
		}
	}
	*black_key_length = key_length;

	if (black_key_memtype == DATA_SECMEM) {
		black_key_dma = get_caam_dma_addr(black_key);
		if (black_key_dma == 0)
			return -EINVAL;
	}

#ifdef DEBUG
	dev_info(jrdev, "%s processing: [key: %p(%zu) black_key: %p(%zu)",
		 __func__, key, key_length, black_key, *black_key_length);
	dev_info(jrdev, "req:%zu, auth: 0x%x]", black_key_length_req, keyauth);
#endif

	if (key_memtype == DATA_GENMEM) {
		if (prepare_write_data(jrdev, key, key_length,
				       &key_dma, &temp_key)) {
			dev_err(jrdev, "unable to prepare key: %p\n", key);
			retval = -ENOMEM;
			goto exit;
		}
	} else {
		key_dma = get_caam_dma_addr(key);
		if (key_dma == 0)
			return -ENOMEM;
	}

	if (black_key_memtype == DATA_GENMEM) {
		if (prepare_read_data(jrdev, black_key_length_req,
				      &black_key_dma, &temp_black_key)) {
			dev_err(jrdev, "unable to prepare cover key\n");
			retval = -ENOMEM;
			goto unprepare_key;
		}
	} else {
		black_key_dma = get_caam_dma_addr(black_key);
		if (black_key_dma == 0)
			return -ENOMEM;
	}

	dsize = cnstr_black_key_jobdesc(&coverdesc, key_dma, key_length,
					black_key_dma, *black_key_length,
					keyauth, trusted_key);
	if (!dsize) {
		dev_err(jrdev, "failed to construct the cover descriptor:\n");
		retval = -ENOMEM;
		goto unprepare_black_key;
	}

	jstat = jr_run_job_and_wait_completion(jrdev, coverdesc);
	if (jstat) {
		dev_err(jrdev, "Covering job failed\n");
		retval = -EIO;
		goto free_desc;
	}

	if (black_key_memtype == DATA_GENMEM) {
		read_data_prepared(jrdev, black_key, black_key_dma,
				   temp_black_key, black_key_length_req);
	}

	/* Update with correct size */
	*black_key_length = black_key_length_req;

free_desc:
	kfree(coverdesc);

unprepare_black_key:
	if (black_key_memtype == DATA_GENMEM)
		unprepare_read_data(jrdev, black_key_dma, temp_black_key,
				    black_key_length_req);

unprepare_key:
	if (key_memtype == DATA_GENMEM)
		unprepare_write_data(jrdev, key_dma, temp_key, key_length);

exit:
	return retval;
}
EXPORT_SYMBOL(caam_black_key);

int caam_random_black_key(struct device *jrdev,
			  size_t key_length,
			  void *black_key, size_t *black_key_length,
			  u8 black_key_memtype, u8 keyauth, u8 trusted_key)
{
	int retval = 0;
	u32 jstat;
	u32 __iomem *coverdesc = NULL;
	size_t black_key_length_req = 0;

	u8 *temp_black_key = NULL;
	caam_dma_addr_t black_key_dma;

	/* Check job ring */
	if (!jrdev)
		return -EINVAL;

#ifdef DEBUG
	dev_info(jrdev, "%s input: [key: (%zu) black_key: %p(%zu), auth: %x]",
		 __func__, key_length,
		 black_key, *black_key_length, keyauth);
#endif

	/* Advanced tests */
	if (keyauth == KEY_COVER_ECB)
		black_key_length_req = ECB_BLACK_KEY_SIZE(key_length);
	else if (keyauth == KEY_COVER_CCM)
		black_key_length_req = CCM_BLACK_KEY_SIZE(key_length);

	if (*black_key_length < black_key_length_req) {
		*black_key_length = black_key_length_req;
		return -EINVAL;
	}
	*black_key_length = key_length;

#ifdef DEBUG
	dev_info(jrdev, "%s processing: [key: (%zu) black_key: %p(%zu)",
		 __func__, key_length, black_key, *black_key_length);
	dev_info(jrdev, "req:%zu, auth: 0x%x]", black_key_length_req, keyauth);
#endif

	if (prepare_read_data(jrdev, black_key_length_req,
			      &black_key_dma, &temp_black_key)) {
		dev_err(jrdev, "unable to prepare covered random key\n");
		retval = -ENOMEM;
		goto exit;
	}

	retval = cnstr_random_black_key_jobdesc(&coverdesc, key_length,
						black_key_dma,
						*black_key_length,
						keyauth, trusted_key);
	if (retval) {
		dev_err(jrdev,
			"failed to construct the cover random descriptor\n");
		goto unprepare_black_key;
	}

	jstat = jr_run_job_and_wait_completion(jrdev, coverdesc);
	if (jstat) {
		dev_err(jrdev, "Covering random job failed\n");
		retval = -EIO;
		goto free_desc;
	}

	read_data_prepared(jrdev, black_key, black_key_dma,
			   temp_black_key, black_key_length_req);

	*black_key_length = black_key_length_req;

	retval = 0;

free_desc:
	kfree(coverdesc);

unprepare_black_key:
	unprepare_read_data(jrdev, black_key_dma, temp_black_key,
			    black_key_length_req);

exit:
	return retval;
}
EXPORT_SYMBOL(caam_random_black_key);

/* Test of black key generation depending on secret size */

#include <linux/module.h>

#define MAX_INPUT_SIZE 64

static char input[MAX_INPUT_SIZE];
static char output[MAX_INPUT_SIZE + CCM_OVERHEAD];

int create_black_key_size_n(struct device *jrdev, size_t n, u8 auth)
{
	int ret;
	size_t output_size = sizeof(output);

	ret = caam_black_key(jrdev,
			     input, n, DATA_GENMEM,
			     output, &output_size,
			     DATA_GENMEM, auth, UNTRUSTED_KEY);
	if (ret)
		pr_err("Creation of black key size: %zd, auth: %d -> %d",
		       n, auth, ret);

	return (ret) ? 1 : 0;
}

int bk_test(void)
{
	int nb_errors = 0;
	int i;
	struct device *jrdev;

	jrdev = caam_jr_alloc();
	if (!jrdev)
		return -ENOMEM;

	for (i = 1; i <= MAX_INPUT_SIZE; i++) {
		pr_info("Size %d\n", i);
		nb_errors += create_black_key_size_n(jrdev, i, KEY_COVER_ECB);
		nb_errors += create_black_key_size_n(jrdev, i, KEY_COVER_CCM);
	}

	pr_info("Nb errors: %d\n", nb_errors);

	return nb_errors;
}

int bk_init(void)
{
#ifdef DEBUG
	return bk_test();
#endif
	return 0;
}

void bk_exit(void)
{
}

module_init(bk_init)
module_exit(bk_exit)
