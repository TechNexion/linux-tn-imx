/*
 * Copyright 2019 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "compat.h"
#include "intern.h"
#include "sm.h"
#include "key_blob.h"
#include "desc.h"
#include "jr.h"
#include "caam_desc.h"
#include "caam_util.h"

#include "linux/printk.h"

#define KEY_MAX_LENGTH (512 - BLOB_OVERHEAD)
#define KEY_CCM_OVERHEAD 12

#define KEY_COLOR_RED	0x0
#define KEY_COLOR_BLACK	0x1

static struct kobject *kb_kobj;
static struct kobj_attribute *kb_kattr;
static struct attribute_group *kb_attr_group;

static int kb_major;
static struct class *kb_class;

#define DATA_SIZE		16
static u8 data[] = {
	0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
	0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};

static u8 skeymod[] = {
	0x0f, 0x0e, 0x0d, 0x0c, 0x0b, 0x0a, 0x09, 0x08,
	0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00
};

kb_addr_t kb_addr;
kb_addr_t kb_addr_user;

/*
 * Blacken a clear key in a slot. Operates "in place".
 * Limited to class 1 keys at the present time
 */
static int sm_keystore_cover_key(struct device *dev, u32 unit, u32 slot,
				 u16 key_length, u8 keyauth)
{
	int retval = 0;
	struct device *jrdev;
	void *slotphys;
	size_t black_key_length;

	if (!dev)
		return -EINVAL;

	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev))
		return -ENOMEM;

	slotphys = sm_keystore_get_slot_phys_addr(dev, unit, slot);
	if (!slotphys) {
		retval = -ENOMEM;
		goto free_jr;
	}

	black_key_length = sm_keystore_get_slot_size(dev, unit, slot);
	if (!black_key_length) {
		retval = -ENOMEM;
		goto free_jr;
	}

	retval = caam_black_key(jrdev,
				slotphys, key_length, DATA_SECMEM,
				slotphys, &black_key_length, DATA_SECMEM,
				keyauth, UNTRUSTED_KEY);

free_jr:
	caam_jr_free(jrdev);

	return retval;
}

/* Export a black/red key to a blob in external memory */
static int sm_keystore_slot_export(struct device *ksdev, u32 unit, u32 slot,
				   u8 keycolor, u8 keyauth,
				   u8 *outbuf, u16 keylen, u8 *keymod)
{
	struct device *jrdev;
	int retval = 0;
	u8 __iomem *slotphys;
	size_t blob_len;
	size_t keymod_len = KEYMOD_SIZE_SM;
	u8 blob_color = (keycolor == BLACK_KEY) ? BLACK_BLOB : RED_BLOB;

	if (!ksdev || !outbuf || !keymod)
		return -EINVAL;

	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev))
		return -ENOMEM;

	slotphys = sm_keystore_get_slot_phys_addr(ksdev, unit, slot);
	if (!slotphys) {
		retval = -ENOMEM;
		goto free_jr;
	}

	blob_len = sm_keystore_get_slot_size(ksdev, unit, slot);
	if (!blob_len) {
		retval = -ENOMEM;
		goto free_jr;
	}

	retval = caam_blob_encap(jrdev,
				 slotphys, keylen, DATA_SECMEM,
				 keycolor, keylen,
				 keyauth, UNTRUSTED_KEY,
				 keymod, &keymod_len, DATA_GENMEM,
				 outbuf, &blob_len,
				 DATA_GENMEM, blob_color);

free_jr:
	caam_jr_free(jrdev);

	return retval;
}

/* Import a black/red key from a blob residing in external memory */
static int sm_keystore_slot_import(struct device *ksdev, u32 unit, u32 slot,
				   u8 keycolor, u8 keyauth,
				   u8 *inbuf, u16 keylen, u8 *keymod)
{
	struct device *jrdev;
	int retval = 0;
	u8 __iomem *slotphys;
	size_t keymod_len = KEYMOD_SIZE_SM;
	size_t key_len;
	size_t secret_size = 0;
	u8 blob_color = (keycolor == BLACK_KEY) ? BLACK_BLOB : RED_BLOB;

	if (!ksdev || !inbuf || !keymod)
		return -EINVAL;

	jrdev = caam_jr_alloc();
	if (IS_ERR(jrdev))
		return -ENOMEM;

	slotphys = sm_keystore_get_slot_phys_addr(ksdev, unit, slot);
	if (!slotphys) {
		retval = -ENOMEM;
		goto free_jr;
	}

	key_len = sm_keystore_get_slot_size(ksdev, unit, slot);
	if (!key_len) {
		retval = -ENOMEM;
		goto free_jr;
	}

	retval = caam_blob_decap(jrdev,
				 inbuf, keylen + BLOB_OVERHEAD,
				 DATA_GENMEM, blob_color,
				 keymod, &keymod_len, DATA_GENMEM,
				 slotphys, &key_len, DATA_SECMEM,
				 keycolor, &secret_size,
				 keyauth, UNTRUSTED_KEY);

free_jr:
	caam_jr_free(jrdev);

	return retval;
}

static int aes_cipher(struct device *ksdev, u8* key, u16 keysz, u8* indata, u8* outdata, u16 sz, u32 operation, u32 keymode)
{
	struct caam_drv_private_sm *kspriv = dev_get_drvdata(ksdev);
	struct device *jrdev = kspriv->smringdev;
	int ret = 0;
	u32 jstat;

	dma_addr_t key_dma;
	dma_addr_t indata_dma;
	dma_addr_t outdata_dma;

	u32 __iomem *desc;

	desc = (u32*)kzalloc(MAX_CAAM_DESCSIZE * sizeof(u32), GFP_KERNEL | GFP_DMA);
	if(!desc)
		return -ENOMEM;

	key_dma = dma_map_single(jrdev, key, keysz, DMA_TO_DEVICE);
	dma_sync_single_for_device(jrdev, key_dma, keysz, DMA_TO_DEVICE);

	indata_dma = dma_map_single(jrdev, indata, sz, DMA_TO_DEVICE);
	dma_sync_single_for_device(jrdev, indata_dma, sz, DMA_TO_DEVICE);

	outdata_dma = dma_map_single(jrdev, outdata, sz, DMA_FROM_DEVICE);

	desc[1] = CMD_KEY | CLASS_1 | (keysz & KEY_LENGTH_MASK) | keymode;
	desc[2] = (u32)key_dma;
	desc[3] = CMD_OPERATION | OP_TYPE_CLASS1_ALG | OP_ALG_AAI_ECB | OP_ALG_ALGSEL_AES | operation;
	desc[4] = CMD_FIFO_LOAD | FIFOLD_CLASS_CLASS1 | FIFOLD_TYPE_MSG | FIFOLD_TYPE_LAST1 | sz;
	desc[5] = (u32)indata_dma;
	desc[6] = CMD_FIFO_STORE | FIFOST_TYPE_MESSAGE_DATA | sz;
	desc[7] = (u32)outdata_dma;
	desc[0] = CMD_DESC_HDR | HDR_ONE | (8 & HDR_DESCLEN_MASK);

	jstat = jr_run_job_and_wait_completion(jrdev, desc);
	if (jstat) {
		dev_err(jrdev, "Encrypt/Decrypt job failed\n");
		ret = -EIO;
		goto free_desc;
	}

	dma_sync_single_for_cpu(jrdev, outdata_dma, sz, DMA_FROM_DEVICE);

	dma_unmap_single(jrdev, key_dma, keysz, DMA_TO_DEVICE);
	dma_unmap_single(jrdev, indata_dma, sz, DMA_TO_DEVICE);
	dma_unmap_single(jrdev, outdata_dma, sz, DMA_FROM_DEVICE);

free_desc:
	kfree(desc);

	return ret;
}

static ssize_t kb_encap(u32 key_len, u32 key_color, u32 key_cover)
{
	int ret = 0;
	struct device_node *dev_node;
	struct platform_device *pdev;
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	u32 units, unit, keyslot;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return -ENODEV;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	units = sm_detect_keystore_units(ksdev);

	if (units < 2)
		return -ENODEV;

	unit = 1;
	sm_establish_keystore(ksdev, unit);

	if (copy_from_user(kb_addr.key_addr, kb_addr_user.key_addr, key_len))
		return -EFAULT;

	if (key_cover == KEY_COVER_CCM) {
		sm_keystore_slot_alloc(ksdev, unit, key_len + KEY_CCM_OVERHEAD, &keyslot);
		sm_keystore_slot_load(ksdev, unit, keyslot, kb_addr.key_addr, key_len + KEY_CCM_OVERHEAD);
		sm_keystore_slot_export(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, key_len, skeymod);
	} else {
		sm_keystore_slot_alloc(ksdev, unit, key_len, &keyslot);
		sm_keystore_slot_load(ksdev, unit, keyslot, kb_addr.key_addr, key_len);
		sm_keystore_slot_export(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, key_len, skeymod);
	}

	if (copy_to_user(kb_addr_user.blob_addr, kb_addr.blob_addr, key_len + BLOB_OVERHEAD))
		return -EFAULT;

	sm_keystore_slot_dealloc(ksdev, unit, keyslot);
	sm_release_keystore(ksdev, unit);

	return ret;
}

static ssize_t kb_decap(u32 blob_len, u32 key_color, u32 key_cover)
{
	int ret = 0;
	struct device_node *dev_node;
	struct platform_device *pdev;
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	u32 units, unit, keyslot;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return -ENODEV;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	units = sm_detect_keystore_units(ksdev);

	if (units < 2)
		return -ENODEV;

	unit = 1;

	sm_establish_keystore(ksdev, unit);

	if (copy_from_user(kb_addr.blob_addr, kb_addr_user.blob_addr, blob_len))
		return -EFAULT;

	if (key_cover == KEY_COVER_CCM) {
		sm_keystore_slot_alloc(ksdev, unit, blob_len - BLOB_OVERHEAD + KEY_CCM_OVERHEAD, &keyslot);
		sm_keystore_slot_import(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, blob_len - BLOB_OVERHEAD, skeymod);
		sm_keystore_slot_read(ksdev, unit, keyslot, blob_len - BLOB_OVERHEAD + KEY_CCM_OVERHEAD, kb_addr.key_addr);
	} else {
		sm_keystore_slot_alloc(ksdev, unit, blob_len - BLOB_OVERHEAD, &keyslot);
		sm_keystore_slot_import(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, blob_len - BLOB_OVERHEAD, skeymod);
		sm_keystore_slot_read(ksdev, unit, keyslot, blob_len - BLOB_OVERHEAD, kb_addr.key_addr);
	}

	if (copy_to_user(kb_addr_user.key_addr, kb_addr.key_addr, blob_len - BLOB_OVERHEAD))
		return -EFAULT;

	sm_keystore_slot_dealloc(ksdev, unit, keyslot);
	sm_release_keystore(ksdev, unit);

	return ret;
}

static ssize_t kb_encr(kb_operation_t *op)
{
	int ret = 0;
	struct device_node *dev_node;
	struct platform_device *pdev;
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	u32 units, unit, keyslot;
	u32 blob_len, key_color, key_cover;
	u8 __iomem *key, *plaindata, *cipherdata;

	blob_len = op->blob_len;
	key_color = op->key_color;
	key_cover = op->key_cover;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return -ENODEV;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	units = sm_detect_keystore_units(ksdev);

	if (units < 2)
		return -ENODEV;

	unit = 1;

	plaindata = kzalloc(DATA_SIZE, GFP_KERNEL | GFP_DMA);
	cipherdata = kzalloc(DATA_SIZE, GFP_KERNEL | GFP_DMA);

	if (copy_from_user(kb_addr.blob_addr, kb_addr_user.blob_addr, blob_len))
		return -EFAULT;

	sm_establish_keystore(ksdev, unit);
	sm_keystore_slot_alloc(ksdev, unit, blob_len - BLOB_OVERHEAD, &keyslot);
	sm_keystore_slot_import(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, blob_len - BLOB_OVERHEAD, skeymod);

	key = kzalloc(blob_len - BLOB_OVERHEAD, GFP_KERNEL | GFP_DMA);
	memcpy(key, (u8 *)kspriv->slot_get_address(ksdev, unit, keyslot), blob_len - BLOB_OVERHEAD);

	memcpy(plaindata, op->buffer, DATA_SIZE);

	if (key_color == KEY_COLOR_BLACK)
		aes_cipher(ksdev, key, blob_len - BLOB_OVERHEAD, plaindata, cipherdata, DATA_SIZE, OP_ALG_ENCRYPT, KEY_ENC);
	else if (key_color == KEY_COLOR_RED)
		aes_cipher(ksdev, key, blob_len - BLOB_OVERHEAD, plaindata, cipherdata, DATA_SIZE, OP_ALG_ENCRYPT, 0);
	else {
		ret = -1;
		goto out_free;
	}

	memcpy(op->buffer, cipherdata, DATA_SIZE);

out_free:
	kfree(key);
	kfree(plaindata);
	kfree(cipherdata);
	sm_keystore_slot_dealloc(ksdev, unit, keyslot);
	sm_release_keystore(ksdev, unit);

	return ret;
}

static ssize_t kb_decr(kb_operation_t *op)
{
	int ret = 0;
	struct device_node *dev_node;
	struct platform_device *pdev;
	struct device *ctrldev, *ksdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *kspriv;
	u32 units, unit, keyslot;
	u32 blob_len, key_color, key_cover;
	u8 __iomem *key, *plaindata, *cipherdata;

	blob_len = op->blob_len;
	key_color = op->key_color;
	key_cover = op->key_cover;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return -ENODEV;

	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);
	ksdev = ctrlpriv->smdev;
	kspriv = dev_get_drvdata(ksdev);
	if (kspriv == NULL)
		return -ENODEV;

	units = sm_detect_keystore_units(ksdev);

	if (units < 2)
		return -ENODEV;

	unit = 1;

	plaindata = kzalloc(DATA_SIZE, GFP_KERNEL | GFP_DMA);
	cipherdata = kzalloc(DATA_SIZE, GFP_KERNEL | GFP_DMA);

	if (copy_from_user(kb_addr.blob_addr, kb_addr_user.blob_addr, blob_len))
		return -EFAULT;

	sm_establish_keystore(ksdev, unit);
	sm_keystore_slot_alloc(ksdev, unit, blob_len - BLOB_OVERHEAD, &keyslot);
	sm_keystore_slot_import(ksdev, unit, keyslot, key_color, key_cover, kb_addr.blob_addr, blob_len - BLOB_OVERHEAD, skeymod);

	key = kzalloc(blob_len - BLOB_OVERHEAD, GFP_KERNEL | GFP_DMA);
	memcpy(key, (u8 *)kspriv->slot_get_address(ksdev, unit, keyslot), blob_len - BLOB_OVERHEAD);

	memcpy(cipherdata, op->buffer, DATA_SIZE);

	if (key_color == KEY_COLOR_BLACK)
		aes_cipher(ksdev, key, blob_len - BLOB_OVERHEAD, cipherdata, plaindata, DATA_SIZE, OP_ALG_DECRYPT, KEY_ENC);
	else if (key_color == KEY_COLOR_RED)
		aes_cipher(ksdev, key, blob_len - BLOB_OVERHEAD, cipherdata, plaindata, DATA_SIZE, OP_ALG_DECRYPT, 0);
	else{
		ret = -1;
		goto out_free;
	}

	memcpy(op->buffer, plaindata, DATA_SIZE);

out_free:
	kfree(key);
	kfree(plaindata);
	kfree(cipherdata);
	sm_keystore_slot_dealloc(ksdev, unit, keyslot);
	sm_release_keystore(ksdev, unit);

	return ret;
}

static ssize_t kb_encap_mfg(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	struct file *filp;
	int ret = 0;
	u32 key_len;
	unsigned long value;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0)
		return -EINVAL;

	if (value == 1)
		filp = filp_open("/device/key.bin", O_RDONLY, 0);
	else
		return -EINVAL;

	if (IS_ERR(filp)) {
		printk("key file open failed\n");
		return -ENOENT;
	}

	memset(kb_addr.key_addr, 0, KEY_MAX_LENGTH);
	memset(kb_addr.blob_addr, 0, KEY_MAX_LENGTH + BLOB_OVERHEAD);

	key_len = kernel_read(filp, 0, kb_addr.key_addr, KEY_MAX_LENGTH);

	filp_close(filp, NULL);

	if (key_len < 0) {
		printk("key file read failed\n");
		return -EIO;
	}

	kb_encap(key_len, RED_KEY, KEY_COVER_ECB);

	if (value == 1)
		filp = filp_open("/device/key_blob", O_WRONLY | O_CREAT | O_TRUNC, 0644);
	else
		return -EINVAL;

	if (IS_ERR(filp)) {
		printk("blob file open failed\n");
		return -ENOENT;
	}

	if (kernel_write(filp, kb_addr.blob_addr, key_len + BLOB_OVERHEAD, 0) < 0) {
		printk("blob file write failed\n");
		ret = -EIO;
	}

	filp_close(filp, NULL);

	return ret ? 0 : count;
}

static ssize_t kb_decap_mfg(struct kobject *kobj, struct kobj_attribute *attr,
			     const char *buf, size_t count)
{
	struct file *filp;
	int ret = 0;
	u32 blob_len;
	unsigned long value;

	ret = kstrtoul(buf, 16, &value);
	if (ret < 0)
		return -EINVAL;

	if (value == 1)
		filp = filp_open("/device/key_blob", O_RDONLY, 0);
	else
		return -EINVAL;

	if (IS_ERR(filp)) {
		printk("blob file open failed\n");
		return -ENOENT;
	}

	memset(kb_addr.key_addr, 0, KEY_MAX_LENGTH);
	memset(kb_addr.blob_addr, 0, KEY_MAX_LENGTH + BLOB_OVERHEAD);

	blob_len = kernel_read(filp, 0, kb_addr.blob_addr, KEY_MAX_LENGTH + BLOB_OVERHEAD);

	filp_close(filp, NULL);

	if (blob_len < 0) {
		printk("blob file read failed\n");
		return -EIO;
	}

	kb_decap(blob_len, RED_KEY, KEY_COVER_ECB);

	if (value == 1)
		filp = filp_open("/device/key.bin", O_WRONLY | O_CREAT | O_TRUNC, 0644);
	else
		return -EINVAL;

	if (IS_ERR(filp)) {
		printk("key file open failed\n");
		return -ENOENT;
	}

	if (kernel_write(filp, kb_addr.key_addr, blob_len - BLOB_OVERHEAD, 0) < 0) {
		printk("key file write failed\n");
		ret = -EIO;
	}

	filp_close(filp, NULL);

	return ret ? 0 : count;
}

static long kb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int errval = 0;

	switch (cmd) {
		case KB_IOCTL_ENCAP:
		{
			kb_parameter_t kb;

			if (copy_from_user(&kb, (kb_parameter_t *)arg, sizeof(kb_parameter_t)))
				return -EFAULT;
			kb_encap(kb.key_len, kb.key_color, kb.key_cover);
			break;
		}
		case KB_IOCTL_DECAP:
	    {
			kb_parameter_t kb;

			if (copy_from_user(&kb, (kb_parameter_t *)arg, sizeof(kb_parameter_t)))
				return -EFAULT;
			kb_decap(kb.blob_len, kb.key_color, kb.key_cover);
			break;
	    }
		case KB_IOCTL_SEND_VRT_ADDR:
		{
			if (copy_from_user(&kb_addr_user, (kb_addr_t *)arg, sizeof(kb_addr_t)))
				return -EFAULT;
			break;
		}
		case KB_IOCTL_ENCR:
		{
			kb_operation_t op;

			if (copy_from_user(&op, (kb_operation_t *)arg, sizeof(kb_operation_t)))
				return -EFAULT;
			op.returned = kb_encr(&op);

			if (copy_to_user((kb_operation_t *)arg, &op, sizeof(kb_operation_t)))
				return -EFAULT;
			break;
		}
		case KB_IOCTL_DECR:
		{
			kb_operation_t op;

			if (copy_from_user(&op, (kb_operation_t *)arg, sizeof(kb_operation_t)))
				return -EFAULT;
			op.returned = kb_decr(&op);

			if (copy_to_user((kb_operation_t *)arg, &op, sizeof(kb_operation_t)))
				return -EFAULT;
			break;
		}
		default:
		    break;
	}

	return errval;
}

static int kb_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot)) {
		printk(KERN_ERR
				"mmap failed!\n");
		return -ENOBUFS;
	}
	return 0;
}

static int kb_open(struct inode *inode, struct file *file)
{
	int errval = 0;

	return errval;
}

static int kb_release(struct inode *inode, struct file *file)
{
	int errval = 0;

	return errval;
}

static const struct file_operations kb_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = kb_ioctl,
	.mmap = kb_mmap,
	.open = kb_open,
	.release = kb_release,
};

static int __init key_blob_init(void)
{
	struct device_node *dev_node;
	struct platform_device *pdev;

	struct attribute **attrs;
	int ret;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return -ENODEV;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return -ENODEV;

	of_node_get(dev_node);

	kb_addr.key_addr = devm_kzalloc(&pdev->dev, KEY_MAX_LENGTH, GFP_KERNEL | GFP_DMA);
	if (!kb_addr.key_addr)
		return -ENOMEM;
	kb_addr.blob_addr = devm_kzalloc(&pdev->dev, KEY_MAX_LENGTH + BLOB_OVERHEAD, GFP_KERNEL | GFP_DMA);
	if (!kb_addr.blob_addr)
		return -ENOMEM;

	/* The last one is NULL, which is used to detect the end */
	attrs = devm_kzalloc(&pdev->dev, 3 * sizeof(*attrs),
			     GFP_KERNEL);
	kb_kattr = devm_kzalloc(&pdev->dev, 2 * sizeof(*kb_kattr),
				 GFP_KERNEL);
	kb_attr_group = devm_kzalloc(&pdev->dev, sizeof(*kb_attr_group),
				      GFP_KERNEL);
	if (!attrs || !kb_kattr || !kb_attr_group)
		return -ENOMEM;

	sysfs_attr_init(kb_kattr[0].attr);
	kb_kattr[0].attr.name = "encap";
	kb_kattr[0].attr.mode = 0600;
	kb_kattr[0].show = NULL;
	kb_kattr[0].store = kb_encap_mfg;
	attrs[0] = &kb_kattr[0].attr;

	sysfs_attr_init(kb_kattr[1].attr);
	kb_kattr[1].attr.name = "decap";
	kb_kattr[1].attr.mode = 0600;
	kb_kattr[1].show = NULL;
	kb_kattr[1].store = kb_decap_mfg;
	attrs[1] = &kb_kattr[1].attr;

	kb_attr_group->attrs = attrs;

	kb_kobj = kobject_create_and_add("kb", NULL);
	if (!kb_kobj) {
		dev_err(&pdev->dev, "failed to add kobject\n");
		return -ENOMEM;
	}

	ret = sysfs_create_group(kb_kobj, kb_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "failed to create sysfs group: %d\n", ret);
		kobject_put(kb_kobj);
		return ret;
	}

	kb_major = register_chrdev(0, "kb", &kb_fops);
	if (kb_major < 0) {
		printk("KB: Unable to register driver\n");
		return -ENODEV;
	}
	kb_class = class_create(THIS_MODULE, "kb");
	if (IS_ERR(kb_class)) {
		printk("KB: Unable to create class\n");
		unregister_chrdev(kb_major, "kb");
		return PTR_ERR(kb_class);
	}
	device_create(kb_class, NULL, MKDEV(kb_major, 0), NULL, "kb");

	return 0;
}

static void __exit key_blob_exit(void)
{
	struct device_node *dev_node;
	struct platform_device *pdev;

	dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec-v4.0");
	if (!dev_node) {
		dev_node = of_find_compatible_node(NULL, NULL, "fsl,sec4.0");
		if (!dev_node)
			return;
	}

	pdev = of_find_device_by_node(dev_node);
	if (!pdev)
		return;

	of_node_put(dev_node);

	devm_kfree(&pdev->dev, kb_addr.key_addr);
	devm_kfree(&pdev->dev, kb_addr.blob_addr);

	sysfs_remove_group(kb_kobj, kb_attr_group);
	kobject_put(kb_kobj);

	device_destroy(kb_class, MKDEV(kb_major, 0));
	class_destroy(kb_class);
	unregister_chrdev(kb_major, "kb");

	return;
}

module_init(key_blob_init);
module_exit(key_blob_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL CAAM KEY BLOB");
MODULE_AUTHOR("Freescale Semiconductor - MCU");
