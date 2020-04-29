// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2018 NXP
 *
 * kb.c: Creates a char device /dev/kb that generates black blob keys from
 * randomly generated plain.
 * This was developed as part of another project which consumes black blob keys
 * via sk-xxx(aes) Crypto API implementation.
 * The device generates black key blobs from user supplied plain key or
 * randomly.
 * When inserting the module, a sysfs class directory is created and populated
 * with 2 default attributes at /sys/class/kb/kb/ which are:
 *  - key_size
 *  - cover_mode.
 * You can customize the blob generation using those attributes.
 *
 *  To change the key size (for random key generation):
 *  echo [key-size]> /sys/class/kb/kb/key_size
 *  Possible values:
 *  16: 128 bits key size
 *  24: 192 bits key size
 *  32: 256 bits key size
 *
 *  To change key cover mode:
 *  echo [cover-mode] > /sys/class/kb/kb/cover_mode
 *  Possible values:
 *  0: AES-ECB
 *  1: AES-CCM
 *
 *  # Create black key
 *  echo 1 > /sys/class/kb/kb/command
 *  echo -n "0123456789abcdef" > /dev/kb
 *  sync /dev/kb
 *  cat /dev/kb > blackkey
 *
 *  # Export black key
 *  echo 3 > /sys/class/kb/kb/command
 *  cat blackkey > /dev/kb
 *  sync /dev/kb
 *  cat /dev/kb > black_blob
 *
 *  # Import black key
 *  echo 2 > /sys/class/kb/kb/command
 *  cat black_blob > /dev/kb
 *  sync /dev/kb
 *  cat /dev/kb > blackkey
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "sm.h"
#include "desc.h"
#include "jr.h"
#include "desc_constr.h"
#include "error.h"
#include "pdb.h"

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
#define NOT_TRUSTED_KEY 0
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

#define SECRET_SIZE_IN_CCM_BLACK_KEY(_key_size) \
	((_key_size >= CCM_OVERHEAD) ? _key_size - CCM_OVERHEAD : 0)

/* A red key is unencrypted so its size is the same */
#define RED_KEY_SIZE(_key_size) (_key_size)

/* Keymod size is 8 byte for blob created in secure memory, else 16 */
#define KEYMOD_SIZE_SM 8
#define KEYMOD_SIZE_GM 16

#define OP_PCL_BLOB_BLACK		0x0004
#define FIFOST_TYPE_KEY_CCM_JKEK	(0x14 << FIFOST_TYPE_SHIFT)
#define OP_PCL_BLOB_PTXT_SECMEM		0x0008
#define OP_PCL_BLOB_EKT			0x0100
#define OP_PCL_BLOB_BLACK		0x0004
#define OP_PCL_BLOB_PTXT_SECMEM		0x0008


/* Create job descriptor to cover key */
int cnstr_black_key_jobdesc(u32 **desc, caam_dma_addr_t key, size_t key_length,
			caam_dma_addr_t cover_key, size_t cover_key_length,
			u8 auth, u8 trusted_key);

/* Encapsulate data in a blob */
int cnstr_blob_encap_jobdesc(u32 **desc,
				caam_dma_addr_t secret, size_t secret_length,
				u8 keycolor, u8 auth,  u8 trusted_key,
				u8 memtype,
				caam_dma_addr_t keymod, size_t keymod_length,
				caam_dma_addr_t blob, size_t blob_length,
				u8 blobcolor);

/* Decapsulate data from a blob */
int cnstr_blob_decap_jobdesc(u32 **desc,
				caam_dma_addr_t blob, size_t blob_length,
				u8 blobcolor,
				caam_dma_addr_t keymod, size_t keymod_length,
				caam_dma_addr_t secret, size_t secret_length,
				u8 keycolor, u8 auth, u8 trusted_key,
				u8 memtype);

int cnstr_key_jobdesc(u32 **desc,
			caam_dma_addr_t key, size_t key_size, size_t key_length,
			u32 key_class, u32 enc,
			u32 nwb, u32 ekt, u32 kdest);

int cnstr_key_imm_jobdesc(u32 **desc,
			void *key, size_t key_size, size_t key_length,
			u32 key_class, u32 enc,
			u32 nwb, u32 ekt, u32 kdest);



int prepare_write_data(struct device *jrdev, u8 *data, size_t size,
			caam_dma_addr_t *dma_addr, u8 **allocated_data);

void unprepare_write_data(struct device *jrdev,
				caam_dma_addr_t dma_addr,
				u8 *allocated_data, size_t size);

int prepare_read_data(struct device *jrdev, size_t size,
			caam_dma_addr_t *dma_addr, u8 **allocated_data);

void read_data_prepared(struct device *jrdev, u8 *data,
			caam_dma_addr_t dma_addr,
			u8 *allocated_data, size_t size);

void unprepare_read_data(struct device *jrdev, caam_dma_addr_t dma_addr,
				u8 *allocated_data, size_t size);

caam_dma_addr_t get_caam_dma_addr(void *address);


int caam_black_key(struct device *jrdev,
	void *key, size_t key_length, u8 key_memtype,
	void *black_key, size_t *black_key_length,
	u8 black_key_memtype, u8 keyauth, u8 trusted_key);


int test_key_loading(struct device *jrdev,
		u8 *key, size_t key_size, size_t key_length,
		u32 key_class, u32 imm, u32 enc,
		u32 nwb, u32 ekt, u32 kdest);

int caam_blob_encap(struct device *jrdev,
			void  *secret, size_t secret_length, u8 secret_memtype,
			u8 keycolor, size_t key_length_in_secret,
			u8 auth, u8 trusted_key,
			void *keymod, size_t *keymod_length, u8 keymod_memtype,
			void *blob, size_t *blob_length,
			u8 blob_memtype, u8 blobcolor);

int caam_blob_decap(struct device *jrdev,
			void *blob, size_t blob_length, u8 blob_memtype,
			u8 blobcolor,
			void *keymod, size_t *keymod_length, u8 keymod_memtype,
			void *secret, size_t *secret_length, u8 secret_memtype,
			u8 keycolor, size_t *key_length_in_secret,
			u8 auth, u8 trusted_key);

/* Structure to wait for the job to complete */
struct jr_job_result {
	int error;
	struct completion completion;
};

/* Callback when a job has been completed
 * It expects the context to be a pointer on jr_job_result
 */
static void jr_job_done_cb(struct device *jrdev, u32 *desc, u32 err,
				void *context)
{
	struct jr_job_result *res = context;

	dev_dbg(jrdev, "jobs %p done: %x", desc, err);
	if (err)
		caam_jr_strstatus(jrdev, err); /* Print error details */

	res->error = err; /* save off the error for postprocessing */

	complete(&res->completion);	/* mark us complete */
}

/* Run a job and wait for its completion */
int jr_run_job_and_wait_completion(struct device *jrdev, u32 *jobdesc)
{
	struct jr_job_result jobres = {0};
	int rtn = 0;

	init_completion(&jobres.completion);

	dev_dbg(jrdev, "Enqueing job %p", jobdesc);
	rtn = caam_jr_enqueue(jrdev, jobdesc, jr_job_done_cb, &jobres);
	if (rtn)
		goto exit;

	wait_for_completion_interruptible(&jobres.completion);
	rtn = jobres.error;

exit:
	return rtn;
}
EXPORT_SYMBOL(jr_run_job_and_wait_completion);

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
	return (memtype == DATA_GENMEM)
		|| (memtype == DATA_DMAMEM)
		|| (memtype == DATA_SECMEM);
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
	return (trusted_key == NOT_TRUSTED_KEY) || (trusted_key == TRUSTED_KEY);
}


/**
 * @brief      Prepare data to be written to CAAM
 *
 * @details    The function performs:
 *  - Allocation of memory compatible with DMA
 *  - Retrieve the DMA address
 *  - Copy @b data into @b allocated_data
 *  - Synchronise the DMA
 *
 * @note       The function unprepare_write_data() must be called from cleanup
 *
 * @param[in]  jrdev           The jrdev
 * @param[in]  data            The data
 * @param[in]  size            The size
 * @param[out] dma_addr        The dma address
 * @param[out] allocated_data  The allocated data
 *
 * @return     0 on success else error code
 */
int prepare_write_data(struct device *jrdev, u8 *data, size_t size,
			caam_dma_addr_t *dma_addr, u8 **allocated_data)
{
	int err = 0;

	/* Allocate memory for data compatible with DMA */
	*allocated_data = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (!*allocated_data) {
		err = (-ENOMEM);
		goto exit;
	}

	/* Get DMA address */
	*dma_addr = dma_map_single(jrdev, *allocated_data, size, DMA_TO_DEVICE);
	if (dma_mapping_error(jrdev, *dma_addr)) {
		dev_err(jrdev, "unable to map data: %p\n", data);
		err = (-ENOMEM);
		goto free_alloc;
	}

	/* Copy the data and synchronize the DMA */
	memcpy(*allocated_data, data, size);
	dma_sync_single_for_device(jrdev, *dma_addr, size, DMA_TO_DEVICE);

	goto exit;

free_alloc:
	kfree(*allocated_data);

exit:
	return err;
}
EXPORT_SYMBOL(prepare_write_data);

/**
 * @brief      Unprepare the data written
 *
 * @details    The function performs:
 * - Clear the temporary memory
 * - Unmap the DMA address
 * - Free @b allocated_data
 *
 * @attention  Must be called on data prepared with prepare_write_data()
 *
 * @param[in]  jrdev           The jrdev
 * @param[in]  dma_addr        The dma address
 * @param[in]  allocated_data  The allocated data
 * @param[in]  size            The size
 */
void unprepare_write_data(struct device *jrdev,
				caam_dma_addr_t dma_addr,
				u8 *allocated_data, size_t size)
{
	/* Clear the data */
	memset(allocated_data, 0, size);
	dma_sync_single_for_device(jrdev, dma_addr, size, DMA_TO_DEVICE);

	/* Free the resources */
	dma_unmap_single(jrdev, dma_addr, size, DMA_TO_DEVICE);
	kfree(allocated_data);
}
EXPORT_SYMBOL(unprepare_write_data);

/**
 * @brief      Prepare data to be read from CAAM
 *
 * @details    The function performs:
 *  - Allocation of memory compatible with DMA
 *  - Retrieve the DMA address
 *
 * @note       The function read_data_prepared() must be called prior to read
 *             data. The function unprepare_read_data() must be called from
 *             cleanup
 *
 * @param[in]  jrdev           The jrdev
 * @param[in]  size            The size
 * @param[out] dma_addr        The dma address
 * @param[out] allocated_data  The allocated data
 *
 * @return     0 on success else error code
 */
int prepare_read_data(struct device *jrdev, size_t size,
			caam_dma_addr_t *dma_addr, u8 **allocated_data)
{
	int err = 0;

	/* Allocate memory for data compatible with DMA */
	*allocated_data = kmalloc(size, GFP_KERNEL | GFP_DMA);
	if (!*allocated_data) {
		err = (-ENOMEM);
		goto exit;
	}

	/* Get DMA address */
	*dma_addr = dma_map_single(jrdev, *allocated_data, size,
		DMA_FROM_DEVICE);
	if (dma_mapping_error(jrdev, *dma_addr)) {
		dev_err(jrdev, "unable to map data\n");
		err = (-ENOMEM);
		goto free_alloc;
	}

	goto exit;


free_alloc:
	kfree(*allocated_data);

exit:
	return err;
}
EXPORT_SYMBOL(prepare_read_data);

/**
 * @brief         Reads prepared data
 *
 * @details       The function performs:
 *  - Synchronization of the data
 *  - copy @b allocated_data to @b data
 *
 * @attention     Must be called on data prepared with prepare_read_data()
 *
 * @param[in]     jrdev           The jrdev
 * @param[in,out] data            The data
 * @param[in]     dma_addr        The dma address
 * @param[in]     allocated_data  The allocated data
 * @param[in]     size            The size
 */
void read_data_prepared(struct device *jrdev, u8 *data,
			caam_dma_addr_t dma_addr,
			u8 *allocated_data, size_t size)
{
	/* Synchronize the DMA and copy the data */
	dma_sync_single_for_device(jrdev, dma_addr, size, DMA_FROM_DEVICE);
	memcpy(data, allocated_data, size);
}
EXPORT_SYMBOL(read_data_prepared);

/**
 * @brief      Unprepare the data read
 *
 * @details    The function performs:
 * - Clear the temporary memory
 * - DMA unmapping
 * - Free @b allocated_data
 *
 * @attention  Must be called on data prepared with prepare_read_data()
 *
 * @param      jrdev           The jrdev
 * @param[in]  dma_addr        The dma address
 * @param      allocated_data  The allocated data
 * @param[in]  size            The size
 */
void unprepare_read_data(struct device *jrdev, caam_dma_addr_t dma_addr,
				u8 *allocated_data, size_t size)
{
	/* Clear the data */
	memset(allocated_data, 0, size);
	dma_sync_single_for_device(jrdev, dma_addr, size, DMA_FROM_DEVICE);

	/* Free the resources */
	dma_unmap_single(jrdev, dma_addr, size,	DMA_FROM_DEVICE);
	kfree(allocated_data);
}
EXPORT_SYMBOL(unprepare_read_data);

/**
 * @brief      Gets the caam dma address of a physical address.
 *
 * @param      phy_address  The physical address
 *
 * @return     The caam dma address.
 */
caam_dma_addr_t get_caam_dma_addr(void *phy_address)
{
	uintptr_t ptr_conv;
	caam_dma_addr_t caam_dma_address = 0;

	/* Check conversion is possible */
	if (sizeof(caam_dma_address) < sizeof(phy_address)) {
		/* We have to check that all bits sets in the phy_address
		 * can be stored in caam_dma_address
		 */

		/* We generate a mask of the representable bits */
		u64 mask = GENMASK_ULL(sizeof(caam_dma_address) * 8, 0);

		/* We check that the bits not reprensentable of
		 * phy_address are not set
		 */
		if ((uintptr_t)phy_address & (~mask))
			goto exit;
	}

	/* We convert address to caam_dma_address */
	ptr_conv = (uintptr_t)phy_address;
	caam_dma_address = (caam_dma_addr_t)ptr_conv;

exit:
	return caam_dma_address;
}
EXPORT_SYMBOL(get_caam_dma_addr);


#define INITIAL_DESCSZ 16	/* size of tmp buffer for descriptor const. */

/*
 * Construct a black key conversion job descriptor
 *
 * This function constructs a job descriptor capable of performing
 * a key blackening operation on a plaintext secure memory resident object.
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - key	physical pointer to the plaintext, which will also hold
 *		the result. Since encryption occurs in place, caller must
 *              ensure that the space is large enough to accommodate the
 *              blackened key
 * - keysz	size of the plaintext
 * - auth	if a CCM-covered key is required, use KEY_COVER_CCM, else
 *		use KEY_COVER_ECB.
 *
 * KEY to key1 from @key_addr LENGTH 16 BYTES;
 * FIFO STORE from key1[ecb] TO @key_addr LENGTH 16 BYTES;
 *
 * Note that this variant uses the JDKEK only; it does not accommodate the
 * trusted key encryption key at this time.
 *
 * Cleanup: The descriptor must be freed
 */
int cnstr_black_key_jobdesc(u32 **desc, caam_dma_addr_t key, size_t key_length,
			caam_dma_addr_t cover_key, size_t cover_key_length,
			u8 auth, u8 trusted_key)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	/* Trusted key not supported */
	if (trusted_key != NOT_TRUSTED_KEY)
		return 0;

	memset(tmpdesc, 0, ARRAY_SIZE(tmpdesc) * sizeof(u32));
	idx = 1;

	/* Load key to class 1 key register */
	tmpdesc[idx++] = CMD_KEY | CLASS_1 | (key_length & KEY_LENGTH_MASK);
	tmpdesc[idx++] = (uintptr_t)key;

	/* ...and write back out via FIFO store*/
	tmpdesc[idx] = CMD_FIFO_STORE | CLASS_1
			| (cover_key_length & KEY_LENGTH_MASK);

	/* plus account for ECB/CCM option in FIFO_STORE */
	if (auth == KEY_COVER_ECB)
		tmpdesc[idx] |= FIFOST_TYPE_KEY_KEK;
	else
		tmpdesc[idx] |= FIFOST_TYPE_KEY_CCM_JKEK;

	idx++;
	tmpdesc[idx++] = (uintptr_t)cover_key;

	/* finish off the job header */
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	/* now allocate execution buffer and coat it with executable */
	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "cover desc:",
			DUMP_PREFIX_OFFSET, 16, 4, *desc, desc_bytes(*desc), 1);
#endif

	return dsize;
}
EXPORT_SYMBOL(cnstr_black_key_jobdesc);

/*
 * Construct a blob encapsulation job descriptor
 *
 * This function dynamically constructs a blob encapsulation job descriptor
 * from the following arguments:
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - keymod	Physical pointer to a key modifier, which must reside in a
 *		contiguous piece of memory. Modifier will be assumed to be
 *		8 bytes long for a blob of type DATA_SECMEM, or 16 bytes long
 *		for a blob of type DATA_GENMEM (see blobcolor argument).
 * - secretbuf	Physical pointer to a secret, normally a black or red key,
 *		possibly residing within an accessible secure memory page,
 *		of the secret to be encapsulated to an output blob.
 * - outbuf	Physical pointer to the destination buffer to receive the
 *		encapsulated output. This buffer will need to be 48 bytes
 *		larger than the input because of the added encapsulation data.
 *		The generated descriptor will account for the increase in size,
 *		but the caller must also account for this increase in the
 *		buffer allocator.
 * - secretsz	Size of input secret, in bytes. This is limited to 65536
 *		less the size of blob overhead, since the length embeds into
 *		DECO pointer in/out instructions.
 * - keycolor   Determines if the source data is covered (black key) or
 *		plaintext (red key). RED_KEY or BLACK_KEY are defined in
 *		for this purpose.
 * - blobcolor	Determine if encapsulated blob should be a secure memory
 *		blob (DATA_SECMEM), with partition data embedded with key
 *		material, or a general memory blob (DATA_GENMEM).
 * - auth	If BLACK_KEY source is covered via AES-CCM, specify
 *		KEY_COVER_CCM, else uses AES-ECB (KEY_COVER_ECB).
 *
 * Upon completion, desc points to a buffer containing a CAAM job
 * descriptor which encapsulates data into an externally-storable blob
 * suitable for use across power cycles.
 *
 * This is an example of a black key encapsulation job into a general memory
 * blob. Notice the 16-byte key modifier in the LOAD instruction. Also note
 * the output 48 bytes longer than the input:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400010           ld: ccb2-key len=16 offs=0
 * [02] 08144891               ptr->@0x08144891
 * [03] F800003A    seqoutptr: len=58
 * [04] 01000000               out_ptr->@0x01000000
 * [05] F000000A     seqinptr: len=10
 * [06] 09745090               in_ptr->@0x09745090
 * [07] 870D0004    operation: encap blob  reg=memory, black, format=normal
 *
 * This is an example of a red key encapsulation job for storing a red key
 * into a secure memory blob. Note the 8 byte modifier on the 12 byte offset
 * in the LOAD instruction; this accounts for blob permission storage:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400C08           ld: ccb2-key len=8 offs=12
 * [02] 087D0784               ptr->@0x087d0784
 * [03] F8000050    seqoutptr: len=80
 * [04] 09251BB2               out_ptr->@0x09251bb2
 * [05] F0000020     seqinptr: len=32
 * [06] 40000F31               in_ptr->@0x40000f31
 * [07] 870D0008    operation: encap blob  reg=memory, red, sec_mem,
 *                             format=normal
 *
 * Note: this function only generates 32-bit pointers at present, and should
 * be refactored using a scheme that allows both 32 and 64 bit addressing
 */
int cnstr_blob_encap_jobdesc(u32 **desc,
				caam_dma_addr_t secret, size_t secret_length,
				u8 keycolor, u8 auth, u8 trusted_key,
				u8 memtype,
				caam_dma_addr_t keymod, size_t keymod_length,
				caam_dma_addr_t blob, size_t blob_length,
				u8 blobcolor)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	/* Trusted key not supported */
	if (trusted_key != NOT_TRUSTED_KEY)
		return 0;

	memset(tmpdesc, 0, ARRAY_SIZE(tmpdesc) * sizeof(u32));
	idx = 1;

	/*
	 * Key modifier works differently for secure/general memory blobs
	 * This accounts for the permission/protection data encapsulated
	 * within the blob if a secure memory blob is requested
	 */
	if (memtype == DATA_SECMEM)
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY |
				 ((12 << LDST_OFFSET_SHIFT) & LDST_OFFSET_MASK)
				 | (keymod_length & LDST_LEN_MASK);
	else /* is general memory blob */
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB
				 | LDST_SRCDST_BYTE_KEY
				 | (keymod_length & LDST_LEN_MASK);

	tmpdesc[idx++] = (u32)keymod;

	/*
	 * Encapsulation output must include space for blob key encryption
	 * key and MAC tag
	 */
	tmpdesc[idx++] = CMD_SEQ_OUT_PTR | (secret_length + BLOB_OVERHEAD);
	tmpdesc[idx++] = (u32)blob;

	/* Input data, should be somewhere in secure memory */
	tmpdesc[idx++] = CMD_SEQ_IN_PTR | secret_length;
	tmpdesc[idx++] = (uintptr_t)secret;

	/* Set blob encap, then color */
	tmpdesc[idx] = CMD_OPERATION | OP_TYPE_ENCAP_PROTOCOL | OP_PCLID_BLOB;

	if (memtype == DATA_SECMEM)
		tmpdesc[idx] |= OP_PCL_BLOB_PTXT_SECMEM;

	if (auth == KEY_COVER_CCM)
		tmpdesc[idx] |= OP_PCL_BLOB_EKT;

	/* An input black key cannot be stored in a red blob */
	if (keycolor == BLACK_KEY)
		tmpdesc[idx] |= OP_PCL_BLOB_BLACK;

	idx++;
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "encap desc:",
			DUMP_PREFIX_OFFSET, 16, 4, *desc, desc_bytes(*desc), 1);
#endif

	return dsize;
}
EXPORT_SYMBOL(cnstr_blob_encap_jobdesc);

/*
 * Construct a blob decapsulation job descriptor
 *
 * This function dynamically constructs a blob decapsulation job descriptor
 * from the following arguments:
 *
 * - desc	pointer to a pointer to the descriptor generated by this
 *		function. Caller will be responsible to kfree() this
 *		descriptor after execution.
 * - keymod	Physical pointer to a key modifier, which must reside in a
 *		contiguous piece of memory. Modifier will be assumed to be
 *		8 bytes long for a blob of type DATA_SECMEM, or 16 bytes long
 *		for a blob of type DATA_GENMEM (see blobcolor argument).
 * - blobbuf	Physical pointer (into external memory) of the blob to
 *		be decapsulated. Blob must reside in a contiguous memory
 *		segment.
 * - outbuf	Physical pointer of the decapsulated output, possibly into
 *		a location within a secure memory page. Must be contiguous.
 * - secretsz	Size of encapsulated secret in bytes (not the size of the
 *		input blob).
 * - keycolor   Determines if decapsulated content is encrypted (BLACK_KEY)
 *		or left as plaintext (RED_KEY).
 * - blobcolor	Determine if encapsulated blob should be a secure memory
 *		blob (DATA_SECMEM), with partition data embedded with key
 *		material, or a general memory blob (DATA_GENMEM).
 * - auth	If decapsulation path is specified by BLACK_KEY, then if
 *		AES-CCM is requested for key covering use KEY_COVER_CCM, else
 *		use AES-ECB (KEY_COVER_ECB).
 *
 * Upon completion, desc points to a buffer containing a CAAM job descriptor
 * that decapsulates a key blob from external memory into a black (encrypted)
 * key or red (plaintext) content.
 *
 * This is an example of a black key decapsulation job from a general memory
 * blob. Notice the 16-byte key modifier in the LOAD instruction.
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400010           ld: ccb2-key len=16 offs=0
 * [02] 08A63B7F               ptr->@0x08a63b7f
 * [03] F8000010    seqoutptr: len=16
 * [04] 01000000               out_ptr->@0x01000000
 * [05] F000003A     seqinptr: len=58
 * [06] 01000010               in_ptr->@0x01000010
 * [07] 860D0004    operation: decap blob  reg=memory, black, format=normal
 *
 * This is an example of a red key decapsulation job for restoring a red key
 * from a secure memory blob. Note the 8 byte modifier on the 12 byte offset
 * in the LOAD instruction:
 *
 * [00] B0800008       jobhdr: stidx=0 len=8
 * [01] 14400C08           ld: ccb2-key len=8 offs=12
 * [02] 01000000               ptr->@0x01000000
 * [03] F8000020    seqoutptr: len=32
 * [04] 400000E6               out_ptr->@0x400000e6
 * [05] F0000050     seqinptr: len=80
 * [06] 08F0C0EA               in_ptr->@0x08f0c0ea
 * [07] 860D0008    operation: decap blob  reg=memory, red, sec_mem,
 *			       format=normal
 *
 * Note: this function only generates 32-bit pointers at present, and should
 * be refactored using a scheme that allows both 32 and 64 bit addressing
 */
int cnstr_blob_decap_jobdesc(u32 **desc,
				caam_dma_addr_t blob, size_t blob_length,
				u8 blobcolor,
				caam_dma_addr_t keymod, size_t keymod_length,
				caam_dma_addr_t secret, size_t secret_length,
				u8 keycolor, u8 auth,  u8 trusted_key,
				u8 memtype)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u16 dsize, idx;

	/* Trusted key not supported */
	if (trusted_key != NOT_TRUSTED_KEY)
		return 0;

	memset(tmpdesc, 0, ARRAY_SIZE(tmpdesc) * sizeof(u32));
	idx = 1;

	/* Load key modifier */
	if (memtype == DATA_SECMEM)
		tmpdesc[idx++] = CMD_LOAD | LDST_CLASS_2_CCB |
				 LDST_SRCDST_BYTE_KEY |
				 ((12 << LDST_OFFSET_SHIFT) & LDST_OFFSET_MASK)
				 | (keymod_length & LDST_LEN_MASK);
	else /* is general memory blob */
		tmpdesc[idx++] = CMD_LOAD
				| LDST_CLASS_2_CCB
				| LDST_SRCDST_BYTE_KEY
				| (keymod_length & LDST_LEN_MASK);

	tmpdesc[idx++] = (u32)keymod;

	/* Compensate BKEK + MAC tag over size of encapsulated secret */
	tmpdesc[idx++] = CMD_SEQ_IN_PTR | blob_length;
	tmpdesc[idx++] = (u32)blob;
	tmpdesc[idx++] = CMD_SEQ_OUT_PTR | secret_length;
	tmpdesc[idx++] = (uintptr_t)secret;

	/* Decapsulate from secure memory partition to black blob */
	tmpdesc[idx] = CMD_OPERATION | OP_TYPE_DECAP_PROTOCOL | OP_PCLID_BLOB;

	if (memtype == DATA_SECMEM)
		tmpdesc[idx] |= OP_PCL_BLOB_PTXT_SECMEM;

	if (auth == KEY_COVER_CCM)
		tmpdesc[idx] |= OP_PCL_BLOB_EKT;

	if (keycolor == BLACK_KEY)
		tmpdesc[idx] |= OP_PCL_BLOB_BLACK;

	idx++;
	tmpdesc[0] = CMD_DESC_HDR | HDR_ONE | (idx & HDR_DESCLEN_MASK);
	dsize = idx * sizeof(u32);

	tdesc = kmalloc(dsize, GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, dsize);
	*desc = tdesc;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "decap desc:",
		DUMP_PREFIX_OFFSET, 16, 4, *desc, desc_bytes(*desc), 1);
#endif

	return dsize;
}
EXPORT_SYMBOL(cnstr_blob_decap_jobdesc);


/*
 * @brief      Create a job descriptor to load a key by address
 *
 * @param[out] desc        The description
 * @param[in]  key         The key
 * @param[in]  key_size    The key size
 * @param[in]  key_length  The key length
 * @param[in]  key_class   The key class
 * @param[in]  enc         The encode
 * @param[in]  nwb         The nwb
 * @param[in]  ekt         The ekt
 * @param[in]  kdest       The kdest
 *
 * @return     Return the number of bytes of the descriptor
 */
int cnstr_key_jobdesc(u32 **desc,
			caam_dma_addr_t key, size_t key_size, size_t key_length,
			u32 key_class, u32 enc,
			u32 nwb, u32 ekt, u32 kdest)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u32 key_option;

	init_desc(tmpdesc, CMD_DESC_HDR);

	key_option = key_class | enc | nwb | ekt | kdest;


	if (enc)
		append_key(tmpdesc, key, key_length, key_option);
	else
		append_key(tmpdesc, key, key_size, key_option);


	tdesc = kmalloc(desc_bytes(tmpdesc), GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, desc_bytes(tmpdesc));
	*desc = tdesc;

#ifdef DEBUG
	print_hex_dump(KERN_ERR, "key desc:",
		DUMP_PREFIX_OFFSET, 16, 4, *desc, desc_bytes(*desc), 1);
#endif

	return desc_bytes(tmpdesc);
}
EXPORT_SYMBOL(cnstr_key_jobdesc);

/**
 * @brief      Create a job descriptor to load an immediate key
 *
 * @param[out] desc        The description
 * @param[in]  key         The key
 * @param[in]  key_size    The key size
 * @param[in]  key_length  The key length
 * @param[in]  key_class   The key class
 * @param[in]  enc         The encode
 * @param[in]  nwb         The nwb
 * @param[in]  ekt         The ekt
 * @param[in]  kdest       The kdest
 *
 * @return     Return the number of bytes of the descriptor
 */
int cnstr_key_imm_jobdesc(u32 **desc,
			void *key, size_t key_size, size_t key_length,
			u32 key_class, u32 enc,
			u32 nwb, u32 ekt, u32 kdest)
{
	u32 *tdesc, tmpdesc[INITIAL_DESCSZ];
	u32 key_option;

	init_desc(tmpdesc, CMD_DESC_HDR);

	key_option = key_class | enc | nwb | ekt | kdest;

	append_key_as_imm(tmpdesc, (void *)key, key_size, key_length,
		key_option);

	tdesc = kmalloc(desc_bytes(tmpdesc), GFP_KERNEL | GFP_DMA);
	if (tdesc == NULL)
		return 0;

	memcpy(tdesc, tmpdesc, desc_bytes(tmpdesc));
	*desc = tdesc;

	return desc_bytes(tmpdesc);
}
EXPORT_SYMBOL(cnstr_key_imm_jobdesc);


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
			void  *secret, size_t secret_length, u8 secret_memtype,
			u8 keycolor, size_t key_length_in_secret,
			u8 auth, u8 trusted_key,
			void *keymod, size_t *keymod_length, u8 keymod_memtype,
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

	dev_dbg(jrdev, "%s input:[secret: %p(%zu) color:%x auth:%x, memtype:%x",
		__func__, secret, secret_length, keycolor, auth,
		secret_memtype);
	dev_dbg(jrdev, ", keymod: %p(%zu), blob: %p(%zu), color: %x]",
		keymod, *keymod_length,
		blob, *blob_length, blobcolor);

	/* Check input buffers */
	if (!secret || !keymod || !blob)
		return -EINVAL;

	/* Check values */
	if (!is_memory_type(secret_memtype)
		|| !is_key_color(keycolor)
		|| !is_memory_type(keymod_memtype)
		|| !is_memory_type(blob_memtype)
		|| !is_blob_color(blobcolor))
		return -EINVAL;

	if (keycolor == BLACK_KEY) {
		if (!is_auth(auth) || !is_trusted_key(trusted_key))
			return -EINVAL;

		/* Trusted key not supported */
		if (trusted_key != NOT_TRUSTED_KEY)
			return -EINVAL;
	} else
		auth = KEY_COVER_ECB; // No value for auth for red keys

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
	if ((keycolor == BLACK_KEY) && (blobcolor == RED_BLOB))
		return -EINVAL;

	/* A red key can only be put in a black blob if it comes from secure
	 * memory
	 */
	if ((keycolor == RED_KEY) && (blobcolor == BLACK_BLOB))
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

	dev_dbg(jrdev, "%s processing: [secret: %p(%zu) cnstr:%zu",
		__func__,
		secret, secret_length, secret_length_for_cnstr);
	dev_dbg(jrdev, " color:%x auth:%x, memtype:%x,",
		keycolor, auth, secret_memtype);
	dev_dbg(jrdev, "keymod: %p(%zu), blob: %p(%zu), blob color: %x]",
		keymod, *keymod_length,
		blob, *blob_length, blobcolor);

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
		keycolor, auth, trusted_key, secret_memtype,
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
			void *blob, size_t blob_length, u8 blob_memtype,
			u8 blobcolor,
			void *keymod, size_t *keymod_length, u8 keymod_memtype,
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

	dev_dbg(jrdev, "%s input: [blob: %p(%zu),blob color: %x,",
		__func__,
		blob, blob_length, blobcolor);
	dev_dbg(jrdev, " keymod: %p(%zu),",
		keymod, *keymod_length);
	dev_dbg(jrdev, " secret: %p(%zu) color:%x auth:%x, memtype:%x]",
		secret, *secret_length, keycolor, auth, secret_memtype);

	/* Check input buffers */
	if (!secret || !keymod || !blob)
		return -EINVAL;

	/* Check values */
	if (!is_key_color(keycolor)
		|| !is_memory_type(blob_memtype)
		|| !is_blob_color(blobcolor)
		|| !is_memory_type(keymod_memtype)
		|| !is_memory_type(secret_memtype))
		return -EINVAL;

	if (keycolor == BLACK_KEY) {
		if (!is_auth(auth) || !is_trusted_key(trusted_key))
			return -EINVAL;

		/* Trusted key not supported */
		if (trusted_key != NOT_TRUSTED_KEY)
			return -EINVAL;
	} else
		auth = KEY_COVER_ECB; // No value for auth for red keys

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
	if ((blobcolor != BLACK_BLOB) && (keycolor == BLACK_KEY))
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

	dev_dbg(jrdev, "%s processing: [blob: %p(%zu),blob color: %x,",
		__func__,
		blob, blob_length, blobcolor);
	dev_dbg(jrdev, " keymod: %p(%zu), secret: %p(%zu) cnstr:%zu",
		keymod, *keymod_length,
		secret, *secret_length, secret_length_for_cnstr);
	dev_dbg(jrdev, " color:%x auth:%x, memtype:%x]",
		keycolor, auth, secret_memtype);

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
		keycolor, auth, trusted_key, secret_memtype);
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
	void *key, size_t key_length, u8 key_memtype,
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

	dev_dbg(jrdev, "%s input: [key: %p(%zu) black_key: %p(%zu), auth: %x]",
		__func__, key, key_length,
		black_key, *black_key_length, keyauth);

	/* Trusted key not supported */
	if (trusted_key != NOT_TRUSTED_KEY)
		return -EINVAL;

	if (!key || !black_key)
		return -EINVAL;

	if (!is_memory_type(key_memtype)
		|| !is_memory_type(black_key_memtype)
		|| !is_auth(keyauth)
		|| !is_trusted_key(trusted_key))
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

	dev_dbg(jrdev, "%s processing: [key: %p(%zu) black_key: %p(%zu)",
		__func__, key, key_length, black_key, *black_key_length);
	dev_dbg(jrdev, "req:%zu, auth: 0x%x]", black_key_length_req, keyauth);

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
		black_key_dma, *black_key_length, keyauth, trusted_key);
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

/**
 * @brief      Test the loading of a key using a "KEY command"
 *
 * @details    The parameters are the field of a KEY command. The parameters are
 *             ORED together to fill the KEY command descriptor. See Secure
 *             Reference manual for more details, section "KEY command".
 *
 * @attention  The value are ORED together so the shift of the bit must already
 *             be set so the use of the value defined in desc.h are recommended.
 *
 * @param[in]  jrdev       The jrdev
 * @param[in]  key         The key
 * @param[in]  key_size    The key size
 * @param[in]  key_length  The key length
 * @param[in]  ctype       The ctype
 * @param[in]  key_class   The key class
 * @param[in]  imm         The imm
 * @param[in]  enc         The encode
 * @param[in]  nwb         The nwb
 * @param[in]  ekt         The ekt
 * @param[in]  kdest       The kdest
 *
 * @return     0 if key can be loaded, otherwise error code
 */
int test_key_loading(struct device *jrdev,
		u8 *key, size_t key_size, size_t key_length,
		u32 key_class, u32 imm, u32 enc,
		u32 nwb, u32 ekt, u32 kdest)
{
	int retval = -EINVAL;
	u32 dsize, jstat;
	u32 __iomem *key_desc = NULL;

	u8 *temp_key = NULL;
	caam_dma_addr_t key_dma;

	if (!jrdev)
		goto exit;

	dev_dbg(jrdev, "%s input key: %p(%zu)-(%zu)",
		__func__, key, key_size, key_length);
	dev_dbg(jrdev, "[class:%x imm:%x enc:%x nwb:%x ekt:%x kdest:%x]",
		key_class, imm, enc, nwb, ekt, kdest);

	/* Check input */
	if (!key)
		goto exit;

	if (prepare_write_data(jrdev, key, key_size, &key_dma, &temp_key)) {
		dev_err(jrdev, "unable to prepare key: %p\n", key);
		retval = -ENOMEM;
		goto exit;
	}

	if (imm)
		dsize = cnstr_key_jobdesc(&key_desc,
			key_dma, key_size, key_length,
			key_class, enc, nwb, ekt, kdest);
	else
		dsize = cnstr_key_imm_jobdesc(&key_desc,
			temp_key, key_size, key_length,
			key_class, enc, nwb, ekt, kdest);

	if (!dsize) {
		dev_err(jrdev, "failed to construct the cover descriptor:\n");
		retval = -ENOMEM;
		goto unprepare_key;
	}

	jstat = jr_run_job_and_wait_completion(jrdev, key_desc);
	if (jstat) {
		dev_err(jrdev, "Key loading job failed\n");
		retval = -EIO;
		goto free_desc;
	}

	/* Set output result */
	retval = 0;

free_desc:
	kfree(key_desc);

unprepare_key:
	unprepare_write_data(jrdev, key_dma, temp_key, key_length);

exit:
	return retval;
}
EXPORT_SYMBOL(test_key_loading);

#define KB_DEVICE_NODE_NAME  "kb"
#define KB_DEVICE_FILE_NAME  "kb"
#define KB_DEVICE_PROC_NAME  "kb"
#define KB_DEVICE_CLASS_NAME "kb"

#define KEY_CCM_OVERHEAD 12

#define KEY_COLOR_RED	0x0
#define KEY_COLOR_BLACK	0x1

#define AES_KEY_SIZE_128 16
#define AES_KEY_SIZE_192 24
#define AES_KEY_SIZE_256 32
#define AES_MAX_BLOB_SIZE 128
#define AES_MAX_SK_KEY_SIZE 64

//Set of command the module can receive
#define CMD_NONE        0 // print last command status
#define CMD_BLACK_KEY   1 // Blacken key upon write
#define CMD_IMPORT_BLOB 2 // Decapsulate the blob written
#define CMD_EXPORT_BLOB 3 // Encapsulate the key written

#define KB_DATA_SIZE_MAX AES_MAX_BLOB_SIZE

struct kb_op_t {
	/* Key details */
	u8 *key;
	size_t key_len;
	size_t key_len_max;

	/* Blob details */
	u8 *blob;
	size_t blob_len;
	size_t blob_len_max;

	/* Configuration */
	u8 cfg_key_color;
	u8 cfg_key_len;
	u8 cfg_cover_mode;
};

void kb_op_to_string(struct kb_op_t *kb_op)
{
	if (kb_op)
		pr_info("kb_op (%p) [key: %p(%lu/%lu) blob: %p(%lu/%lu), color: %x, len: %d, cover:%d]",
			kb_op, kb_op->key, kb_op->key_len, kb_op->key_len_max,
			kb_op->blob, kb_op->blob_len, kb_op->blob_len_max,
			kb_op->cfg_key_color, kb_op->cfg_key_len, kb_op->cfg_cover_mode);
}

struct kb_dev_t {
	struct kb_op_t kb_op;
	struct semaphore sem;
	struct cdev chardev;
	struct device *device;

	/* read/write data */
	u8 *kb_data;
	size_t kb_data_len;
	size_t kb_data_len_max;

	/*SM KS*/
	struct device *ksdev;
	struct caam_drv_private_sm *kspriv;
	u32 nb_units;
	u32 unit;
	u32 keyslot;

	/*JR DEV*/
	struct device *jrdev;

	u8 command; /* command to perform on write and read */
};

void kb_dev_to_string(struct kb_dev_t *kb_dev)
{
	if (!kb_dev) {
		pr_err("%s kb_dev_t is NULL", __func__);
		return;
	}

	pr_info("kb_dev (%p): [data: %p(%lu/%lu)]", kb_dev,
		kb_dev->kb_data, kb_dev->kb_data_len, kb_dev->kb_data_len_max);
	kb_op_to_string(&kb_dev->kb_op);
}

static struct class *gs_kb_class;
static struct kb_dev_t *gs_kb_dev;

static dev_t gs_kb_dev_number;

static u8 gs_keymod[] = { 0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08, 0x07,
		0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00 };

/*
 * Forward declarations
 */
static int kb_open(struct inode *inode, struct file *filp);
static int kb_release(struct inode *inode, struct file *filp);
static ssize_t kb_read(struct file *filp, char __user *buf, size_t count,
						loff_t *f_pos);
static ssize_t kb_write(struct file *filp, const char __user *buf, size_t count,
						loff_t *f_pos);

static int kb_fsync(struct file *filp, loff_t start, loff_t end, int datasync);

/* File operation for the device */
const static struct file_operations kb_fops = {
	.owner = THIS_MODULE,
	.open = kb_open,
	.release = kb_release,
	.read = kb_read,
	.write = kb_write,
	.fsync = kb_fsync,
};

bool is_valid_command(uint32_t cmd)
{
	return ((cmd == CMD_BLACK_KEY)
			|| (cmd == CMD_IMPORT_BLOB)
			|| (cmd == CMD_EXPORT_BLOB));
}

static int kb_open(struct inode *inode, struct file *filp)
{
	struct kb_dev_t *kbdev = container_of(inode->i_cdev, struct kb_dev_t,
								chardev);
	dev_info(kbdev->device, "%s %s", __func__, "start");

	filp->private_data = kbdev;

	return 0;

}

static int kb_release(struct inode *inode, struct file *filp)
{
	struct kb_dev_t *kbdev = filp->private_data;

	dev_info(kbdev->device, "%s %s", __func__, "start");

	return 0;
}

static ssize_t kb_read(struct file *filp, char __user *buffer, size_t length,
						loff_t *offset)
{
	struct kb_dev_t *kbdev = filp->private_data;
	size_t bytes_to_copy;
	int remaindersize;
	int err = 0;

	/* Compute the part to return to user and if out buffer is big enough */
	dev_info(kbdev->device, "length: %lu, off: %llu", length, *offset);
	if (*offset >= kbdev->kb_data_len) {
		dev_err(kbdev->device, "Read over the data");
		goto out;
	}
	remaindersize = kbdev->kb_data_len - *offset;
	bytes_to_copy = (remaindersize <= length) ? remaindersize : length;
	dev_info(kbdev->device, "rem: %d, nb_bytes: %lu", remaindersize, bytes_to_copy);
	err = copy_to_user(buffer, kbdev->kb_data + *offset, bytes_to_copy);
	if (err) {
		err = -EPERM;
		dev_err(kbdev->device, "Copy failed");
		goto out;
	}

	*offset += bytes_to_copy;
	err = bytes_to_copy;

	dev_info(kbdev->device, "Copied:%lu", bytes_to_copy);

out:
	up(&(kbdev->sem));

	return err;
}

static ssize_t kb_write(struct file *filp, const char __user *buf, size_t count,
			loff_t *f_pos)
{
	struct kb_dev_t *kbdev = filp->private_data;
	ssize_t err = 0;

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	if (!is_valid_command(kbdev->command)) {
		err = -EINVAL;
		dev_err(kbdev->device, "Invalid command");
		goto out;
	}

	memset(kbdev->kb_data, 0, kbdev->kb_data_len_max);
	kbdev->kb_data_len = 0;

	if ((kbdev->command == CMD_BLACK_KEY)
		|| (kbdev->command == CMD_EXPORT_BLOB))
	{
		/* The input is a key */
		if (count > AES_MAX_SK_KEY_SIZE) {
			err = -EINVAL;
			dev_err(kbdev->device, "Input too large");
			goto out;
		}

		if (copy_from_user(kbdev->kb_data, buf, count)) {
			err = -EFAULT;
			dev_err(kbdev->device, "Copy failed");
			goto out;
		}

		kbdev->kb_data_len = count;
		err = count;

	} else if (kbdev->command == CMD_IMPORT_BLOB) {
		/* The input is a blob */
		if (count > AES_MAX_BLOB_SIZE) {
			err = -EINVAL;
			dev_err(kbdev->device, "Input too large");
			goto out;
		}

		if (copy_from_user(kbdev->kb_data, buf, count)) {
			err = -EFAULT;
			dev_err(kbdev->device, "Copy failed");
			goto out;
		}
		kbdev->kb_data_len = count;
		err = count;
	}

	kb_dev_to_string(kbdev);

	dev_info(kbdev->device, "readed %lu", count);

out:
	up(&kbdev->sem);

	return err;
}

/* Safely copy data from a source to a destination:
 * - Check the size to copy
 * - Erase the destination
 * - Copy the data
 * - Erase the source
 */
static int copy_data(u8 *dst, size_t *p_dst_len, size_t dst_len_max,
						u8 *src, size_t *p_src_len, size_t src_len_max)
{
	if (!dst || !p_dst_len || !src || !p_src_len)
		return -EINVAL;

	if (*p_src_len > dst_len_max)
		return -ENOMEM;

	memset(dst, 0, dst_len_max);
	memcpy(dst, src, *p_src_len);
	*p_dst_len = *p_src_len;
	memset(src, 0, src_len_max);
	*p_src_len = 0;

	return 0;
}

static int kb_fsync(struct file *filp, loff_t start, loff_t end, int datasync)
{
	struct kb_dev_t *kbdev = filp->private_data;
	int ret = 0;
	ssize_t err = 0;
	u8 *buf_in = NULL;
	size_t *p_buf_in_len = 0;
	size_t buf_in_len_max = 0;
	u8 *buf_out = NULL;
	size_t *p_buf_out_len = 0;
	size_t buf_out_len_max = 0;

dev_err(kbdev->device, "%s %s", __func__, "start");

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	if (!is_valid_command(kbdev->command)) {
		err = -EINVAL;
		dev_err(kbdev->device, "Invalid command");
		goto out;
	}

	kbdev->jrdev = caam_jr_alloc();
	if (!kbdev->jrdev) {
		err = -EINVAL;
		dev_err(kbdev->device, "Cannot allocate a job ring");
		goto out;
	}

	kb_dev_to_string(kbdev);

dev_err(kbdev->device, "%s %s", __func__, "setting bufin");

	/* Prepare copy of the input */
	if ((kbdev->command == CMD_BLACK_KEY)
		|| (kbdev->command == CMD_EXPORT_BLOB))
	{
		buf_in = kbdev->kb_op.key;
		p_buf_in_len = &kbdev->kb_op.key_len;
		buf_in_len_max = kbdev->kb_op.key_len_max;
	} else if (kbdev->command == CMD_IMPORT_BLOB) {
		buf_in = kbdev->kb_op.blob;
		p_buf_in_len = &kbdev->kb_op.blob_len;
		buf_in_len_max = kbdev->kb_op.blob_len_max;
	}

dev_err(kbdev->device, "%s %s", __func__, "copy input");

	/* Copy input data */
	err = copy_data(buf_in, p_buf_in_len, buf_in_len_max,
			kbdev->kb_data, &kbdev->kb_data_len, kbdev->kb_data_len_max);
	if (err) {
		dev_err(kbdev->device, "Failed to copy data to input");
		goto free_jr;
	}

	dev_info(kbdev->device, "In:%p(%lu)", buf_in, *p_buf_in_len);
	print_hex_dump(KERN_ERR, "In dump:",
			DUMP_PREFIX_OFFSET, 16, 4, buf_in, *p_buf_in_len, 1);

dev_err(kbdev->device, "%s %s", __func__, "processing");

	/* Different processing depending on command */
	if (kbdev->command == CMD_BLACK_KEY) {
		/* Need to save size as we are doing inplace */
		size_t output_size = kbdev->kb_op.key_len_max;

		dev_info(kbdev->device, "Doing black key");

		if (kbdev->kb_op.key_len != kbdev->kb_op.cfg_key_len) {
			ret = -EINVAL;
			dev_err(kbdev->device, "The input key size does not correspond to the size configured");
			goto free_jr;
		}

		ret = caam_black_key(kbdev->jrdev,
			kbdev->kb_op.key, kbdev->kb_op.key_len, DATA_GENMEM,
			kbdev->kb_op.key, &output_size, DATA_GENMEM,
			(kbdev->kb_op.cfg_cover_mode? KEY_COVER_CCM : KEY_COVER_ECB), NOT_TRUSTED_KEY);

		/* Update size */
		kbdev->kb_op.key_len = output_size;

		/* set buf_out */
		buf_out = kbdev->kb_op.key;
		p_buf_out_len = &kbdev->kb_op.key_len;
		buf_out_len_max = kbdev->kb_op.key_len_max;

		if (ret) {
			/* Clear the input which can be in plaintext */
			memset(kbdev->kb_op.key, 0, kbdev->kb_op.key_len_max);
			kbdev->kb_op.key_len = 0;
		} else {
			ret = test_key_loading(kbdev->jrdev, kbdev->kb_op.key, kbdev->kb_op.key_len, kbdev->kb_op.cfg_key_len,
				 CLASS_2, CMD_KEY, KEY_ENC, KEY_NWB,
				(kbdev->kb_op.cfg_cover_mode)? KEY_EKT : 0,
				KEY_DEST_CLASS_REG);
		}

	} else if (kbdev->command == CMD_IMPORT_BLOB) {
		/* Default parameter */
		size_t keymod_size = ARRAY_SIZE(gs_keymod);
		size_t key_length_in_secret = 0;
		u8 keycolor = BLACK_KEY;
		u8 blobcolor = BLACK_BLOB;
		u8 memtype = DATA_GENMEM;

		kbdev->kb_op.key_len = kbdev->kb_op.key_len_max;

		dev_info(kbdev->device, "Doing import blob");
		ret =  caam_blob_decap(kbdev->jrdev,
					kbdev->kb_op.blob, kbdev->kb_op.blob_len, DATA_GENMEM,
					blobcolor,
					gs_keymod, &keymod_size, DATA_GENMEM,
					kbdev->kb_op.key, &kbdev->kb_op.key_len, memtype,
					keycolor, &key_length_in_secret,
					(kbdev->kb_op.cfg_cover_mode)? KEY_COVER_CCM : KEY_COVER_ECB,
					NOT_TRUSTED_KEY);

		if (key_length_in_secret != kbdev->kb_op.cfg_key_len) {
			dev_err(kbdev->device, "Decapsulated size does not correspond to the size configured");
			ret = -EINVAL;
			goto free_jr;
		}

		/* set buf_out */
		buf_out = kbdev->kb_op.key;
		p_buf_out_len = &kbdev->kb_op.key_len;
		buf_out_len_max = kbdev->kb_op.key_len_max;

	} else if (kbdev->command == CMD_EXPORT_BLOB) {
		/* Default parameter */
		size_t keymod_size = ARRAY_SIZE(gs_keymod);
		size_t key_length_in_secret = kbdev->kb_op.cfg_key_len;
		u8 keycolor = BLACK_KEY;
		u8 blobcolor = BLACK_BLOB;
		u8 memtype = DATA_GENMEM;

		kbdev->kb_op.blob_len = kbdev->kb_op.blob_len_max;

		if (kbdev->kb_op.cfg_cover_mode == KEY_COVER_ECB) {
			if (kbdev->kb_op.key_len != ECB_BLACK_KEY_SIZE(kbdev->kb_op.cfg_key_len)) {
				ret = -EINVAL;
				dev_err(kbdev->device, "The input ECB black key does not have a size corresponding to the one configured");
				goto free_jr;
			}
		} else {
			if (kbdev->kb_op.key_len != CCM_BLACK_KEY_SIZE(kbdev->kb_op.cfg_key_len)) {
				ret = -EINVAL;
				dev_err(kbdev->device, "The input CCM black key does not have a size corresponding to the one configured");
				goto free_jr;
			}
		}

		dev_info(kbdev->device, "Doing export blob");
		ret = caam_blob_encap(kbdev->jrdev,
					kbdev->kb_op.key, kbdev->kb_op.key_len, memtype,
					keycolor, key_length_in_secret,
					(kbdev->kb_op.cfg_cover_mode)? KEY_COVER_CCM : KEY_COVER_ECB,
					NOT_TRUSTED_KEY,
					gs_keymod, &keymod_size, DATA_GENMEM,
					kbdev->kb_op.blob, &kbdev->kb_op.blob_len, DATA_GENMEM,
					blobcolor);

		/* set buf_out */
		buf_out = kbdev->kb_op.blob;
		p_buf_out_len = &kbdev->kb_op.blob_len;
		buf_out_len_max = kbdev->kb_op.blob_len_max;

	} else {
		err = -EPERM;
		dev_err(kbdev->device, "Not implemented");
	}

	/* Check the result of the operations */
	if (ret) {
		dev_err(kbdev->device, "Command failed");
		err = ret;
		goto free_jr;
	}

	dev_info(kbdev->device, "Out: %p(%lu)", buf_out, *p_buf_out_len);
	print_hex_dump(KERN_ERR, "outdump:",
			DUMP_PREFIX_OFFSET, 16, 4, buf_out, *p_buf_out_len, 1);

	/* Write the data to kb_data */
	err = copy_data(kbdev->kb_data, &kbdev->kb_data_len, kbdev->kb_data_len_max,
		buf_out, p_buf_out_len, buf_out_len_max);
	if (err) {
		dev_err(kbdev->device, "Failed to copy data to output");
		goto free_jr;
	}

free_jr:
	caam_jr_free(kbdev->jrdev);

out:
	up(&kbdev->sem);

	return err;
}

/* Definition of the function to modify attributes */

static ssize_t cover_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct kb_dev_t *kbdev = (struct kb_dev_t *) dev_get_drvdata(dev);
	uint32_t cfg_cover_mode = KEY_COVER_ECB;

dev_info(dev, "%s %s", __func__, "start");

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	cfg_cover_mode = kbdev->kb_op.cfg_cover_mode;

	up(&kbdev->sem);

	return snprintf(buf, sizeof(cfg_cover_mode), "%d\n", cfg_cover_mode);
}

static ssize_t cover_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int err;
	struct kb_dev_t *kbdev = (struct kb_dev_t *) dev_get_drvdata(dev);
	int cfg_cover_mode;
	bool cover_mode_valid;

dev_info(dev, "%s %s", __func__, "start");

	if (kstrtoint(buf, 10, &cfg_cover_mode))
		return -EIO;

	cover_mode_valid = (cfg_cover_mode == KEY_COVER_CCM)
				|| (cfg_cover_mode == KEY_COVER_ECB);
	if (!cover_mode_valid)
		return -EINVAL;

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	kbdev->kb_op.cfg_cover_mode = cfg_cover_mode;
	err = sizeof(cfg_cover_mode);

	up(&kbdev->sem);
	return err;
}

/* It uses cover_mode_show and cover_mode_store */
static DEVICE_ATTR_RW(cover_mode);

static ssize_t key_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct kb_dev_t *kbdev = (struct kb_dev_t *) dev_get_drvdata(dev);

	uint32_t keysz = AES_KEY_SIZE_128;

dev_info(dev, "%s %s", __func__, "start");

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	keysz = kbdev->kb_op.cfg_key_len;

	up(&kbdev->sem);

	return snprintf(buf, sizeof(keysz), "%d\n", keysz);
}

static ssize_t key_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct kb_dev_t *kbdev = (struct kb_dev_t *) dev_get_drvdata(dev);
	int keysz, err;
	bool valid_keysz;

dev_info(dev, "%s %s", __func__, "start");

	if (kstrtoint(buf, 10, &keysz))
		return -EIO;

	valid_keysz = (keysz != AES_KEY_SIZE_128)
			|| (keysz != AES_KEY_SIZE_192)
			|| (keysz != AES_KEY_SIZE_256);
	if (!valid_keysz)
		return -EINVAL;

	if (down_interruptible(&kbdev->sem))
		return -ERESTARTSYS;

	kbdev->kb_op.cfg_key_len = keysz;
	err = sizeof(keysz);

	up(&kbdev->sem);

	return err;
}

/* It uses key_size_show and key_size_store */
static DEVICE_ATTR_RW(key_size);

static ssize_t command_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct kb_dev_t *kdev = (struct kb_dev_t *) dev_get_drvdata(dev);

	uint32_t cmd = CMD_NONE;

dev_info(dev, "%s %s", __func__, "start");

	if (down_interruptible(&(kdev->sem)))
		return -ERESTARTSYS;

	cmd = kdev->command;
	up(&(kdev->sem));

	return snprintf(buf, sizeof(cmd), "%d\n", cmd);
}

static ssize_t command_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct kb_dev_t *kdev = (struct kb_dev_t *) dev_get_drvdata(dev);
	uint32_t command, err;

dev_info(dev, "%s %s", __func__, "start");

	if (kstrtoint(buf, 10, &command))
		return -EIO;

	if (!is_valid_command(command))
		return -EINVAL;

	if (down_interruptible(&(kdev->sem)))
		return -ERESTARTSYS;

	kdev->command = command;
	err = sizeof(command);

	up(&(kdev->sem));
	return err;
}

/* It uses command_show and command_store */
static DEVICE_ATTR_RW(command);

/* Functions to init the device */

static int __kb_setup_op(struct kb_op_t *kbop)
{
	int err = 0;

	if (!kbop) {
		err = -EINVAL;
		goto exit;
	}

	/* Allocate memory to store AES key */
	kbop->key = kzalloc(AES_MAX_SK_KEY_SIZE, GFP_KERNEL);
	if (!kbop->key) {
		err = -ENOMEM;
		goto exit;
	}
	kbop->key_len = 0;
	kbop->key_len_max = AES_MAX_SK_KEY_SIZE;

	/* Allocate memory to store blob */
	kbop->blob = kzalloc(AES_MAX_BLOB_SIZE, GFP_KERNEL);
	if (!kbop->blob) {
		err = -ENOMEM;
		goto free_key;
	}
	kbop->blob_len = 0;
	kbop->blob_len_max = AES_MAX_BLOB_SIZE;

	goto exit;

free_key:
	kfree(kbop->key);

exit:
	return err;
}

static void __kb_release_op(struct kb_op_t *kbop)
{
	if (!kbop) {
		pr_err("%s %s: kbop is NULL", KB_DEVICE_NODE_NAME, __func__);
		return;
	}

	kfree(kbop->key);
	kfree(kbop->blob);
}

static int __kb_setup_keystore(struct kb_dev_t *kbdev)
{
	int err = 0;
	struct device_node *dev_node;
	struct device *ctrldev;
	struct platform_device *pdev;
	struct caam_drv_private *ctrlpriv;

dev_info(kbdev->device, "%s %s", __func__, "start");

	if (!kbdev) {
		err = -EINVAL;
		goto exit;
	}

	/* Retrieve instance of keystore device */
	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL,
								"fsl,sec4.0");
		if (!dev_node) {
			dev_err(kbdev->device, "Failed to find CAAM node");
			err = -ENODEV;
			goto exit;
		}
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev) {
		dev_err(kbdev->device, "Failed to find platfrorm dev");
		err = -ENODEV;
		goto exit;
	}

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	kbdev->ksdev = ctrlpriv->smdev;

	/* Store if the keystore has been initialised */
	kbdev->kspriv = dev_get_drvdata(kbdev->ksdev);

exit:
	return err;
}

static void __kb_release_keystore(struct kb_dev_t *kbdev)
{
	if (!kbdev) {
		pr_err("%s %s: kbdev is NULL", KB_DEVICE_NODE_NAME, __func__);
		return;
	}

	(void)kbdev;
	/*
	if (kbdev->ksdev) {
		sm_keystore_slot_dealloc(kbdev->ksdev, kbdev->unit, kbdev->keyslot);
		sm_release_keystore(kbdev->ksdev, kbdev->unit);
	}
	*/
}

static int __kb_setup_kbdev(struct kb_dev_t *kbdev, dev_t dev_number)
{
	int err;

	if (!kbdev) {
		err = -EINVAL;
		goto exit;
	}

	memset(kbdev, 0, sizeof(*kbdev));

	cdev_init(&kbdev->chardev, &kb_fops);

	kbdev->chardev.owner = THIS_MODULE;
	kbdev->chardev.ops = &kb_fops;

	err = cdev_add(&kbdev->chardev, dev_number, 1);
	if (err)
		goto exit;

	sema_init(&kbdev->sem, 1);

	kbdev->kb_data_len_max = KB_DATA_SIZE_MAX;
	kbdev->kb_data = kzalloc(kbdev->kb_data_len_max, GFP_KERNEL);
	if (!kbdev->kb_data) {
		err = -ENOMEM;
		goto remove_chardev;
	}
	kbdev->kb_data_len = 0;

	/*defaults*/
	kbdev->kb_op.cfg_key_len = AES_KEY_SIZE_128;
	kbdev->kb_op.cfg_cover_mode = KEY_COVER_ECB;
	kbdev->kb_op.cfg_key_color = KEY_COLOR_BLACK;
	kbdev->command = CMD_NONE;

	goto exit;

remove_chardev:
	cdev_del(&kbdev->chardev);

exit:
	return err;
}

static void __kb_release_kbdev(struct kb_dev_t *kbdev)
{
	if (!kbdev) {
		pr_err("%s %s: kbdev is NULL", KB_DEVICE_NODE_NAME, __func__);
		return;
	}

	cdev_del(&kbdev->chardev);
	kfree(kbdev->kb_data);
}

/*
 * This function is called when the module is loaded
 */
int kb_init(void)
{
	dev_t dev_number = 0;
	struct kb_dev_t *kb_dev;
	struct class *kb_class;
	int err = -1;

	pr_info("%s: %s", KB_DEVICE_NODE_NAME, __func__);

	/* Alloc a device region */
	err = alloc_chrdev_region(&dev_number, 0, 1, KB_DEVICE_NODE_NAME);
	if (err < 0) {
		pr_alert("Failed to alloc char dev region.\n");
		goto fail;
	}

	kb_dev = kmalloc(sizeof(*kb_dev), GFP_KERNEL);

	if (!kb_dev) {
		err = -ENOMEM;
		pr_err("Failed to alloc kb_dev.\n");
		goto unregister;
	}

	err = __kb_setup_kbdev(kb_dev, dev_number);
	if (err) {
		pr_err("Failed to setup dev: %d.\n", err);
		goto cleanup;
	}

	kb_class = class_create(THIS_MODULE, KB_DEVICE_CLASS_NAME);
	if (IS_ERR(kb_class)) {
		err = PTR_ERR(kb_class);
		pr_err("Failed to create kb class.\n");
		goto release_kbdev;
	}

	kb_dev->device = device_create(kb_class, NULL, dev_number, "%s",
					KB_DEVICE_FILE_NAME);
	if (IS_ERR(kb_dev->device)) {
		err = PTR_ERR(kb_dev->device);
		pr_err("Failed to create kb device.");
		goto destroy_class;
	}

	err = __kb_setup_op(&kb_dev->kb_op);
	if (err) {
		dev_err(kb_dev->device, "Failed to setup op");
		goto destroy_device;
	}

	err = __kb_setup_keystore(kb_dev);
	if (err) {
		dev_err(kb_dev->device, "Failed to setup keystore");
		goto release_op;
	}

	/* Create attribute file in sysfs */
	err = device_create_file(kb_dev->device, &dev_attr_cover_mode);
	if (err < 0) {
		dev_err(kb_dev->device, "Cant create device attribute %s\n",
			dev_attr_cover_mode.attr.name);
		goto release_keystore;
	}

	err = device_create_file(kb_dev->device, &dev_attr_key_size);
	if (err < 0) {
		dev_err(kb_dev->device, "Cant create device attribute %s\n",
			dev_attr_key_size.attr.name);
		goto release_keystore;
	}

	err = device_create_file(kb_dev->device, &dev_attr_command);
	if (err < 0) {
		dev_err(kb_dev->device, "Cant create device attribute %s\n",
			dev_attr_command.attr.name);
		goto release_keystore;
	}

	dev_set_drvdata(kb_dev->device, kb_dev);

	/*kb_create_proc();*/

	/* Save info in static variables */
	gs_kb_dev_number = dev_number;
	gs_kb_dev = kb_dev;
	gs_kb_class = kb_class;

	pr_info("Succedded to initialize kb device.\n");

	return 0;

release_keystore:
	__kb_release_keystore(kb_dev);

release_op:
	__kb_release_op(&kb_dev->kb_op);

destroy_device:
	device_destroy(kb_class, dev_number);

destroy_class:
	class_destroy(kb_class);

release_kbdev:
	__kb_release_kbdev(kb_dev);

cleanup:
	kfree(kb_dev);

unregister:
	unregister_chrdev_region(dev_number, 1);

fail:
	return err;

}

static void kb_exit(void)
{
	pr_info("Destroy kb device.\n");

	if (!gs_kb_dev) {
		pr_err("%s %s: gs_kb_dev is NULL", KB_DEVICE_NODE_NAME, __func__);
		return;
	}

	/* kb_remove_proc(); */

	__kb_release_keystore(gs_kb_dev);

	__kb_release_op(&gs_kb_dev->kb_op);

	device_destroy(gs_kb_class, gs_kb_dev_number);
	class_destroy(gs_kb_class);

	__kb_release_kbdev(gs_kb_dev);

	kfree(gs_kb_dev);

	unregister_chrdev_region(gs_kb_dev_number, 1);
}

MODULE_DESCRIPTION("Black key blob generation device");
MODULE_LICENSE("GPL");

module_init(kb_init);
module_exit(kb_exit);


struct switch_param_t {
	const char *name;
	size_t nb_values;
	u32 *values;
};

void switch_param_to_string(struct switch_param_t *sw_param, int select)
{
	size_t idx = 0;

	if (!sw_param) {
		pr_err("%s switch_param_t is NULL", __func__);
		return;
	}

	pr_info("sw_param(%p) %s, %lu values: {",
			sw_param, sw_param->name, sw_param->nb_values);

	for (idx = 0; idx < sw_param->nb_values; idx++) {
		bool selected = (idx == select - 1);
		pr_cont("%s%x%s,", (selected?"*":""),
			sw_param->values[idx], (selected?"*":""));
	}

	pr_cont("}");
}

#define SW_RESET_VALUE 1

struct switch_list_t {
	size_t nb_sw;
	size_t nb_sw_max;
	struct switch_param_t **sw_params;
	/* Minimal value is 1 */
	size_t *sw_selection;
};

void switch_list_to_string(struct switch_list_t *sw_list, bool all)
{
	size_t idx = 0;

	if (!sw_list) {
		pr_err("%s switch_list_t is NULL", __func__);
		return;
	}

	pr_info("sw_list(%p), %lu/%lu switches: ", sw_list, sw_list->nb_sw,
		sw_list->nb_sw_max);

	if (all)
		for (idx = 0; idx < sw_list->nb_sw; idx++)
			switch_param_to_string(sw_list->sw_params[idx],
				sw_list->sw_selection[idx]);
	else {
		pr_cont("{");
		for (idx = 0; idx < sw_list->nb_sw; idx++)
			pr_cont("%lu,", sw_list->sw_selection[idx]);
		pr_cont("}");
	}
}

int add_to_switch_list(struct switch_list_t * sw_list,
						struct switch_param_t *sw)
{
	if (!sw_list || !sw)
		return -EINVAL;

	if (sw_list->nb_sw >= sw_list->nb_sw_max)
		return -ENOMEM;

	sw_list->sw_params[sw_list->nb_sw] = sw;

	sw_list->nb_sw++;

	return 0;
}

int sw_list_reset(struct switch_list_t * sw_list)
{
	size_t idx;

	if (!sw_list)
		return -EINVAL;

	/* Find the index of the SW */
	for (idx = 0; idx < sw_list->nb_sw; idx++)
		sw_list->sw_selection[idx] = SW_RESET_VALUE;

	return 0;
}

u32 sw_list_get_sw_value(struct switch_list_t * sw_list,
						struct switch_param_t *sw)
{
	size_t idx;
	u32 ret = -EINVAL;

	/* Find the index of the SW */
	for (idx = 0; idx < sw_list->nb_sw; idx++)
		if (sw_list->sw_params[idx] == sw)
			goto found;

	goto exit;

found:
	ret = sw->values[sw_list->sw_selection[idx] - 1];
	pr_info("found -> idx %lu val: %x", idx, ret);

exit:
	return ret;
}

int sw_list_next(struct switch_list_t * sw_list)
{
	size_t idx_sw;
	size_t last_setted = 0;

	if (!sw_list)
		return -EINVAL;

	/* Search for the last setted*/
	for (idx_sw = 0; idx_sw < sw_list->nb_sw; idx_sw++) {
		if (sw_list->sw_selection[idx_sw] != SW_RESET_VALUE)
			last_setted = idx_sw;
	}

	/* Search if we can increment one sw before the last setted */
	for (idx_sw = 0; idx_sw < last_setted; idx_sw++) {
		if (sw_list->sw_selection[idx_sw] <
			sw_list->sw_params[idx_sw]->nb_values)
		{
			size_t idx_sw2;

			/* Switch not maxed out founded so we increment it */
			sw_list->sw_selection[idx_sw]++;

			/* Rest all previous */
			for (idx_sw2 = 0; idx_sw2 < idx_sw; idx_sw2++)
				sw_list->sw_selection[idx_sw2] = SW_RESET_VALUE;

			goto exit;
		}
	}

	/* All previous SW are maxed out */

	/* Check if last SW is maxed out */
	if (sw_list->sw_selection[last_setted] <
			sw_list->sw_params[last_setted]->nb_values) {
		/* Increment it */
		sw_list->sw_selection[last_setted]++;
	} else {
		/* Check if it is the last sw which can be set */
		if ((last_setted + 1) == sw_list->nb_sw) {
			return -EPERM;
		} else {
			/* reset the list and increment the next one if possible */
			sw_list_reset(sw_list);
			sw_list->sw_selection[last_setted + 1]++;
		}
	}

exit:
	return 0;
}

#define FL_MAX_SW 16

#define SW_NAME(_name) _sw_##_name
#define SW_ARR_NAME(_name) _sw_arr_##_name

#define DECLARE_SW(_name, ...) \
		u32 SW_ARR_NAME(_name)[] = __VA_ARGS__;\
		struct switch_param_t SW_NAME(_name) = {\
			.name = #_name,\
			.nb_values = ARRAY_SIZE(SW_ARR_NAME(_name)),\
			.values = SW_ARR_NAME(_name),\
		}

#define ADD_SW(_list, _sw) add_to_switch_list(&_list, &SW_NAME(_sw))

#define SW_SELECT(_list, _sw) sw_list_get_sw_value(&_list, &SW_NAME(_sw))
