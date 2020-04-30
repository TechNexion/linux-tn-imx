// SPDX-License-Identifier: GPL-2.0
/*
 * Blob encapsulation/decapsulation functions for CAAM
 *
 * Copyright 2018 NXP
 */

#include "caam.h"
#include "desc.h"
#include "caam_desc.h"
#include "caam_util.h"

/**
 * @brief         Perform the encapsulation of a secret in a blob
 *
 * @details       Details:
 * - If @b memtype is setted to @b DATA_SECMEM, the secret to encapsulate is
 *   read directly by the CAAM from Secure Memeory without intermediate copy.
 *   The value of @b secret must be a physical address in secure memory.
 *
 * - The parameter @b keymod_length is used to check the size of the key
 *   modifier is long enough, else the function will fail with the appropriate
 *   size set.
 *
 * - The parameter @b blob_length is used to check the size of the output for
 *   the blob is long enough, else the function will fail with the appropriate
 *   size set.
 *
 * - The parameter @b key_length_in_secret is used to indicate the real size of
 *   the key to encapsulate in the case it is a black key.
 *
 * @attention     The input and output data are copied to temporary arrays
 *                except for @b secret if @b memtype is @b DATA_SECMEM.
 *
 * @param[in]     jrdev                 The jrdev
 * @param[in]     secret                The secret
 * @param[in]     secret_length         The secret length
 * @param[in]     secret_memtype        The secret memtype
 * @param[in]     keycolor              The keycolor
 * @param[in]     key_length_in_secret  The key length in secret, if the secret
 *                                      to encapsulate is a covered key (black
 *                                      key), the original size of the key must
 *                                      be provided.
 * @param[in]     auth                  The auth
 * @param[in]     trusted_key           The trusted key
 * @param[in]     keymod                The keymod
 * @param[in,out] keymod_length         The keymod length
 * @param[in]     keymod_memtype        The keymod memtype
 * @param[out]    blob                  The blob
 * @param[in,out] blob_length           The blob length
 * @param[in]     blob_memtype          The memtype
 * @param[in]     blobcolor             The blobcolor
 *
 * @return        0 if no error, otherwise error code
 */
int caam_blob_encap(struct device *jrdev,
		    const void  *secret, size_t secret_length,
		    u8 secret_memtype,
		    u8 keycolor, size_t key_length_in_secret,
		    u8 auth, u8 trusted_key,
		    const void *keymod, size_t *keymod_length,
		    u8 keymod_memtype,
		    void *blob, size_t *blob_length,
		    u8 blob_memtype, u8 blobcolor)
{
	int retval = 0;
	u32 dsize, jstat;
	u32 __iomem *blob_encap = NULL;
	size_t secret_length_for_cnstr = 0;

	u8 *temp_secret = NULL;
	caam_dma_addr_t secret_dma;
	u8 *temp_keymod = NULL;
	caam_dma_addr_t keymod_dma;
	u8 *temp_blob = NULL;
	caam_dma_addr_t blob_dma;

	/*
	 * basic checks
	 */

	/* Check job ring */
	if (!jrdev)
		return -EINVAL;

#ifdef DEBUG
	dev_info(jrdev, "%s input:[secret: %p(%zu) color:%x auth:%x, memtype:%x",
		 __func__, secret, secret_length, keycolor, auth,
		 secret_memtype);
	dev_info(jrdev, ", keymod: %p(%zu), memtype:%x",
		 keymod, *keymod_length, keymod_memtype);
	dev_info(jrdev, "blob: %p(%zu), memtype:%x color: %x]",
		 blob, *blob_length, blob_memtype, blobcolor);
#endif

	/* Check input buffers */
	if (!secret || !keymod || !blob)
		return -EINVAL;

	/* Check values */
	if (!is_memory_type(secret_memtype) ||
	    !is_key_color(keycolor) ||
	    !is_memory_type(keymod_memtype) ||
	    !is_memory_type(blob_memtype) ||
	    !is_blob_color(blobcolor))
		return -EINVAL;

	if (keycolor == BLACK_KEY) {
		if (!is_auth(auth) || !is_trusted_key(trusted_key))
			return -EINVAL;

		/* Trusted key not supported */
		if (trusted_key == TRUSTED_KEY)
			return -EINVAL;
	} else {
		auth = KEY_COVER_ECB; // No value for auth for red keys
	}

	/*
	 * Advanced checks
	 */

	/* Check the keymod is long enough */
	if (secret_memtype == DATA_SECMEM) {
		if (*keymod_length < KEYMOD_SIZE_SM) {
			*keymod_length = KEYMOD_SIZE_SM;
			return -EINVAL;
		}
		*keymod_length = KEYMOD_SIZE_SM;
	} else {
		if (*keymod_length < KEYMOD_SIZE_GM) {
			*keymod_length = KEYMOD_SIZE_GM;
			return -EINVAL;
		}
		*keymod_length = KEYMOD_SIZE_GM;
	}

	/* A black key cannot be stored in a red blob */
	if (keycolor == BLACK_KEY && blobcolor == RED_BLOB)
		return -EINVAL;

	/* A red key can only be put in a black blob if it comes from secure
	 * memory
	 */
	if (keycolor == RED_KEY && blobcolor == BLACK_BLOB)
		if (secret_memtype != DATA_SECMEM)
			return -EINVAL;

	/* adapt the size of the black key */
	secret_length_for_cnstr = secret_length;
	if (keycolor == BLACK_KEY)
		secret_length_for_cnstr = key_length_in_secret;

	/* Check the blob can be stored */
	if ((secret_length_for_cnstr + BLOB_OVERHEAD) > *blob_length) {
		*blob_length = (secret_length_for_cnstr + BLOB_OVERHEAD);
		return -EINVAL;
	}
	*blob_length = secret_length_for_cnstr + BLOB_OVERHEAD;

#ifdef DEBUG
	dev_info(jrdev, "%s processing: [secret: %p(%zu) cnstr:%zu",
		 __func__,
		 secret, secret_length, secret_length_for_cnstr);
	dev_info(jrdev, " color:%x auth:%x, memtype:%x,",
		 keycolor, auth, secret_memtype);
	dev_info(jrdev, ", keymod: %p(%zu), memtype:%x",
		 keymod, *keymod_length, keymod_memtype);
	dev_info(jrdev, "blob: %p(%zu), memtype:%x color: %x]",
		 blob, *blob_length, blob_memtype, blobcolor);
#endif

	if (secret_memtype == DATA_GENMEM) {
		if (prepare_write_data(jrdev, secret, secret_length,
				       &secret_dma, &temp_secret)) {
			dev_err(jrdev, "unable to prepare secret: %p\n",
				secret);
			retval = -ENOMEM;
			goto exit;
		}
	} else {
		secret_dma = get_caam_dma_addr(secret);
		if (secret_dma == 0)
			return -ENOMEM;
	}

	if (keymod_memtype == DATA_GENMEM) {
		if (prepare_write_data(jrdev, keymod, *keymod_length,
				       &keymod_dma, &temp_keymod)) {
			dev_err(jrdev, "unable to prepare keymod: %p\n",
				keymod);
			retval = -ENOMEM;
			goto unprepare_secret;
		}
	} else {
		keymod_dma = get_caam_dma_addr(secret);
		if (secret_dma == 0)
			return -ENOMEM;
	}

	if (blob_memtype == DATA_GENMEM) {
		if (prepare_read_data(jrdev, *blob_length, &blob_dma,
				      &temp_blob)) {
			dev_err(jrdev, "unable to prepare blob\n");
			retval = -ENOMEM;
			goto unprepare_keymod;
		}
	} else {
		blob_dma = get_caam_dma_addr(secret);
		if (secret_dma == 0)
			return -ENOMEM;
	}

	dsize = cnstr_blob_encap_jobdesc(&blob_encap,
					 secret_dma, secret_length_for_cnstr,
					 keycolor, auth, trusted_key,
					 secret_memtype,
					 keymod_dma, *keymod_length,
					 blob_dma, *blob_length, blobcolor);
	if (!dsize) {
		dev_err(jrdev, "failed to construct the encap descriptor:\n");
		retval = -ENOMEM;
		goto unprepare_blob;
	}

	jstat = jr_run_job_and_wait_completion(jrdev, blob_encap);
	if (jstat) {
		dev_err(jrdev, "Encapsulation job failed\n");
		retval = -EIO;
		goto free_desc;
	}

	if (blob_memtype == DATA_GENMEM)
		read_data_prepared(jrdev, blob, blob_dma, temp_blob,
				   *blob_length);

free_desc:
	kfree(blob_encap);

unprepare_blob:
	if (blob_memtype == DATA_GENMEM)
		unprepare_read_data(jrdev, blob_dma, temp_blob, *blob_length);

unprepare_keymod:
	if (keymod_memtype == DATA_GENMEM)
		unprepare_write_data(jrdev, keymod_dma, temp_keymod,
				     *keymod_length);

unprepare_secret:
	if (secret_memtype == DATA_GENMEM)
		unprepare_write_data(jrdev, secret_dma, temp_secret,
				     secret_length);

exit:
	return retval;
}
EXPORT_SYMBOL(caam_blob_encap);

/**
 * @brief         Perform the decapsulation of a secret from a blob
 *
 * @details       Details:
 * - If @b memtype is setted to @b DATA_SECMEM, the secret to decapsulate is
 *   written directly by the CAAM to Secure Memeory without intermediate copy.
 *   The value of @b secret must be a physical address in secure memory.
 *
 * - If @b memtype is setted to @b DATA_SECMEM, it is possible to retrieve data
 *   or key from a black blob without the output to be covered.
 *
 * - The parameter @b keymod_length is used to check the size of the key
 *   modifier is long enough, else the function will fail with the appropriate
 *   size set.
 *
 * - The parameter @b secret_length is used to check the size of the output
 *   memory is long enough, else the function will fail with the appropriate
 *   size set.
 *
 * - The parameter @b key_length_in_secret is used to indicate the user of the
 *   real size of the data encapsulated in the blob which is useful in case of
 *   decapsulation of black keys.
 *
 * @attention     The input and output data are copied to temporary arrays
 *                except for @b secret if @b memtype is @b DATA_SECMEM.
 *
 * @param[in]     jrdev                 The jrdev
 * @param[in]     blob                  The blob
 * @param[in]     blob_length           The blob length
 * @param[in]     blob_memtype          The memtype
 * @param[in]     blobcolor             The blobcolor
 * @param[in]     keymod                The keymod
 * @param[in,out] keymod_length         The keymod length
 * @param[in]     keymod_memtype        The keymod memtype
 * @param[in]     secret                The secret
 * @param[in,out] secret_length         The secret length
 * @param[in]     secret_memtype        The secret memtype
 * @param[in]     keycolor              The keycolor
 * @param[out]    key_length_in_secret  The key length in secret
 * @param[in]     auth                  The auth
 * @param[in]     trusted_key           The trusted key
 *
 * @return        0 if no error, otherwise error code
 */
int caam_blob_decap(struct device *jrdev,
		    const void *blob, size_t blob_length, u8 blob_memtype,
		    u8 blobcolor,
		    const void *keymod, size_t *keymod_length,
		    u8 keymod_memtype,
		    void *secret, size_t *secret_length, u8 secret_memtype,
		    u8 keycolor, size_t *key_length_in_secret,
		    u8 auth, u8 trusted_key)
{
	int retval = 0;
	u32 dsize, jstat;
	u32 __iomem *blob_decap = NULL;
	size_t secret_length_for_cnstr;
	size_t secret_length_req;

	u8 *temp_secret = NULL;
	caam_dma_addr_t secret_dma;
	u8 *temp_keymod = NULL;
	caam_dma_addr_t keymod_dma;
	u8 *temp_blob = NULL;
	caam_dma_addr_t blob_dma;

	/*
	 * basic checks
	 */

	/* Check job ring */
	if (!jrdev)
		return -EINVAL;

#ifdef DEBUG
	dev_info(jrdev, "%s input: [blob: %p(%zu), memtype:%x ,blob color: %x",
		 __func__,
		 blob, blob_length, blob_memtype, blobcolor);
	dev_info(jrdev, " keymod: %p(%zu), memtype:%x",
		 keymod, *keymod_length, keymod_memtype);
	dev_info(jrdev, " secret: %p(%zu) color:%x auth:%x, memtype:%x]",
		 secret, *secret_length, keycolor, auth, secret_memtype);
#endif

	/* Check input buffers */
	if (!secret || !keymod || !blob)
		return -EINVAL;

	/* Check values */
	if (!is_key_color(keycolor) ||
	    !is_memory_type(blob_memtype) ||
	    !is_blob_color(blobcolor) ||
	    !is_memory_type(keymod_memtype) ||
	    !is_memory_type(secret_memtype))
		return -EINVAL;

	if (keycolor == BLACK_KEY) {
		if (!is_auth(auth) || !is_trusted_key(trusted_key))
			return -EINVAL;

		/* Trusted key not supported */
		if (trusted_key == TRUSTED_KEY)
			return -EINVAL;
	} else {
		auth = KEY_COVER_ECB; // No value for auth for red keys
	}

	/*
	 * Advanced checks
	 */

	/* Check keymod size depending on memory type */
	if (secret_memtype == DATA_SECMEM) {
		if (*keymod_length < KEYMOD_SIZE_SM) {
			*keymod_length = KEYMOD_SIZE_SM;
			return -EINVAL;
		}
		*keymod_length = KEYMOD_SIZE_SM;
	} else {
		if (*keymod_length < KEYMOD_SIZE_GM) {
			*keymod_length = KEYMOD_SIZE_GM;
			return -EINVAL;
		}
		*keymod_length = KEYMOD_SIZE_GM;
	}

	/* Check the user want a black key if it is a black blob */
	if (blobcolor != BLACK_BLOB && keycolor == BLACK_KEY)
		return -EINVAL;

	/* Check the blob is valid */
	if (blob_length <= BLOB_OVERHEAD)
		return -EINVAL;

	*key_length_in_secret = blob_length - BLOB_OVERHEAD;
	secret_length_req = *key_length_in_secret;
	secret_length_for_cnstr = *key_length_in_secret;

	/* Compute the size of of the output array required to stock the
	 * secret
	 */
	if (keycolor == BLACK_KEY) {
		if (auth == KEY_COVER_CCM)
			secret_length_req =
					CCM_BLACK_KEY_SIZE(secret_length_req);
		else
			secret_length_req =
					ECB_BLACK_KEY_SIZE(secret_length_req);
	}

	/* Chech the secret can be stored */
	if (*secret_length < secret_length_req) {
		*secret_length = secret_length_req;
		return -EINVAL;
	}
	*secret_length = secret_length_req;

#ifdef DEBUG
	dev_info(jrdev, "%s processing: [blob: %p(%zu), memtype:%x ,blob color: %x,",
		 __func__,
		 blob, blob_length, blob_memtype, blobcolor);
	dev_info(jrdev, " keymod: %p(%zu), memtype:%x, secret: %p(%zu) cnstr:%zu",
		 keymod, *keymod_length, keymod_memtype,
		 secret, *secret_length, secret_length_for_cnstr);
	dev_info(jrdev, " color:%x auth:%x, memtype:%x]",
		 keycolor, auth, secret_memtype);
#endif

	if (blob_memtype == DATA_GENMEM) {
		if (prepare_write_data(jrdev, blob, blob_length,
				       &blob_dma, &temp_blob)) {
			dev_err(jrdev, "unable to prepare blob: %p\n", blob);
			retval = -ENOMEM;
			goto exit;
		}
	} else {
		blob_dma = get_caam_dma_addr(blob);
		if (blob_dma == 0)
			return -ENOMEM;
	}

	if (keymod_memtype == DATA_GENMEM) {
		if (prepare_write_data(jrdev, keymod, *keymod_length,
				       &keymod_dma, &temp_keymod)) {
			dev_err(jrdev, "unable to prepare keymod: %p\n",
				keymod);
			retval = -ENOMEM;
			goto unprepare_blob;
		}
	} else {
		keymod_dma = get_caam_dma_addr(keymod);
		if (keymod_dma == 0)
			return -ENOMEM;
	}

	if (secret_memtype == DATA_GENMEM) {
		if (prepare_read_data(jrdev, *secret_length,
				      &secret_dma, &temp_secret)) {
			dev_err(jrdev, "unable to prepare secret\n");
			retval = -ENOMEM;
			goto unprepare_keymod;
		}
	} else {
		secret_dma = get_caam_dma_addr(secret);
		if (secret_dma == 0)
			return -ENOMEM;
	}

	dsize = cnstr_blob_decap_jobdesc(&blob_decap,
					 blob_dma, blob_length, blobcolor,
					 keymod_dma, *keymod_length,
					 secret_dma, secret_length_for_cnstr,
					 keycolor, auth, trusted_key,
					 secret_memtype);
	if (!dsize) {
		dev_err(jrdev, "failed to construct the encap descriptor:\n");
		retval = -ENOMEM;
		goto unprepare_secret;
	}

	jstat = jr_run_job_and_wait_completion(jrdev, blob_decap);
	if (jstat) {
		dev_err(jrdev, "Decapsulation job failed\n");
		retval = -EIO;
		goto free_desc;
	}

	if (secret_memtype == DATA_GENMEM)
		read_data_prepared(jrdev, secret, secret_dma, temp_secret,
				   *secret_length);

free_desc:
	kfree(blob_decap);

unprepare_secret:
	if (secret_memtype == DATA_GENMEM)
		unprepare_read_data(jrdev, secret_dma, temp_secret,
				    *secret_length);

unprepare_keymod:
	if (keymod_memtype == DATA_GENMEM)
		unprepare_write_data(jrdev, keymod_dma, temp_keymod,
				     *keymod_length);

unprepare_blob:
	if (blob_memtype == DATA_GENMEM)
		unprepare_write_data(jrdev, blob_dma, temp_blob, blob_length);

exit:
	return retval;
}
EXPORT_SYMBOL(caam_blob_decap);
