/*
 * CAAM Secure Memory Storage Interface
 * Copyright (C) 2008-2015 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 *
 * Loosely based on the SHW Keystore API for SCC/SCC2
 * Experimental implementation and NOT intended for upstream use. Expect
 * this interface to be amended significantly in the future once it becomes
 * integrated into live applications.
 *
 * Known issues:
 *
 * - Executes one instance of an secure memory "driver". This is tied to the
 *   fact that job rings can't run as standalone instances in the present
 *   configuration.
 *
 * - It does not expose a userspace interface. The value of a userspace
 *   interface for access to secrets is a point for further architectural
 *   discussion.
 *
 * - Partition/permission management is not part of this interface. It
 *   depends on some level of "knowledge" agreed upon between bootloader,
 *   provisioning applications, and OS-hosted software (which uses this
 *   driver).
 *
 * - No means of identifying the location or purpose of secrets managed by
 *   this interface exists; "slot location" and format of a given secret
 *   needs to be agreed upon between bootloader, provisioner, and OS-hosted
 *   application.
 */

#include "compat.h"
#include "regs.h"
#include "intern.h"
#include "sm.h"
#include "jr.h"
#include <linux/of_address.h>

#ifdef SM_DEBUG_CONT
void sm_show_page(struct device *dev, struct sm_page_descriptor *pgdesc)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	u32 i, *smdata;

	dev_info(dev, "physical page %d content at 0x%08x\n",
		 pgdesc->phys_pagenum, pgdesc->pg_base);
	smdata = pgdesc->pg_base;
	for (i = 0; i < (smpriv->page_size / sizeof(u32)); i += 4)
		dev_info(dev, "[0x%08x] 0x%08x 0x%08x 0x%08x 0x%08x\n",
			 (u32)&smdata[i], smdata[i], smdata[i+1], smdata[i+2],
			 smdata[i+3]);
}
#endif

static __always_inline u32 sm_send_cmd(struct caam_drv_private_sm *smpriv,
					     struct caam_drv_private_jr *jrpriv,
					     u32 cmd, u32 *status)
{
	void __iomem *write_address;
	void __iomem *read_address;

	if (smpriv->sm_reg_offset == SM_V1_OFFSET) {
		struct caam_secure_mem_v1 *sm_regs_v1;
		sm_regs_v1 = (struct caam_secure_mem_v1 *)
			((void *)jrpriv->rregs + SM_V1_OFFSET);
		write_address = &sm_regs_v1->sm_cmd;
		read_address = &sm_regs_v1->sm_status;

	} else if (smpriv->sm_reg_offset == SM_V2_OFFSET) {
		struct caam_secure_mem_v2 *sm_regs_v2;
		sm_regs_v2 = (struct caam_secure_mem_v2 *)
			((void *)jrpriv->rregs + SM_V2_OFFSET);
		write_address = &sm_regs_v2->sm_cmd;
		read_address = &sm_regs_v2->sm_status;

	} else {
		return -EINVAL;
	}

	wr_reg32(write_address, cmd);

	udelay(10);

	/* Read until the command has terminated and the status is correct */
	do {
		*status = rd_reg32(read_address);
	} while (((*status & SMCS_CMDERR_MASK) >>  SMCS_CMDERR_SHIFT)
				   == SMCS_CMDERR_INCOMP);

	return 0;
}

/*
 * Following section establishes the default methods for keystore access
 * They are NOT intended for use external to this module
 *
 * In the present version, these are the only means for the higher-level
 * interface to deal with the mechanics of accessing the phyiscal keystore
 */


int slot_alloc(struct device *dev, u32 unit, u32 size, u32 *slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;
	u32 i;
#ifdef SM_DEBUG
	dev_info(dev, "slot_alloc(): requesting slot for %d bytes\n", size);
#endif

	if (size > smpriv->slot_size)
		return -EKEYREJECTED;

	for (i = 0; i < ksdata->slot_count; i++) {
		if (ksdata->slot[i].allocated == 0) {
			ksdata->slot[i].allocated = 1;
			(*slot) = i;
#ifdef SM_DEBUG
			dev_info(dev, "slot_alloc(): new slot %d allocated\n",
				 *slot);
#endif
			return 0;
		}
	}

	return -ENOSPC;
}
EXPORT_SYMBOL(slot_alloc);

int slot_dealloc(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;
	u8 __iomem *slotdata;

#ifdef SM_DEBUG
	dev_info(dev, "slot_dealloc(): releasing slot %d\n", slot);
#endif
	if (slot >= ksdata->slot_count)
		return -EINVAL;
	slotdata = ksdata->base_address + slot * smpriv->slot_size;

	if (ksdata->slot[slot].allocated == 1) {
		/* Forcibly overwrite the data from the keystore */
		memset_io(ksdata->base_address + slot * smpriv->slot_size, 0,
		       smpriv->slot_size);

		ksdata->slot[slot].allocated = 0;
#ifdef SM_DEBUG
		dev_info(dev, "slot_dealloc(): slot %d released\n", slot);
#endif
		return 0;
	}

	return -EINVAL;
}
EXPORT_SYMBOL(slot_dealloc);

void *slot_get_address(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return NULL;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_address(): slot %d is 0x%08x\n", slot,
		 (u32)ksdata->base_address + slot * smpriv->slot_size);
#endif

	return ksdata->base_address + slot * smpriv->slot_size;
}

void *slot_get_physical(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return NULL;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_physical(): slot %d is 0x%08x\n", slot,
		 (u32)ksdata->phys_address + slot * smpriv->slot_size);
#endif

	return ksdata->phys_address + slot * smpriv->slot_size;
}

u32 slot_get_base(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	/*
	 * There could potentially be more than one secure partition object
	 * associated with this keystore.  For now, there is just one.
	 */

	(void)slot;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_base(): slot %d = 0x%08x\n",
		slot, (u32)ksdata->base_address);
#endif

	return (uintptr_t)(ksdata->base_address);
}

u32 slot_get_offset(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *ksdata = smpriv->pagedesc[unit].ksdata;

	if (slot >= ksdata->slot_count)
		return -EINVAL;

#ifdef SM_DEBUG
	dev_info(dev, "slot_get_offset(): slot %d = %d\n", slot,
		slot * smpriv->slot_size);
#endif

	return slot * smpriv->slot_size;
}

u32 slot_get_slot_size(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);


#ifdef SM_DEBUG
	dev_info(dev, "slot_get_slot_size(): slot %d = %d\n", slot,
		 smpriv->slot_size);
#endif
	/* All slots are the same size in the default implementation */
	return smpriv->slot_size;
}



int kso_init_data(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *keystore_data = NULL;
	u32 slot_count;
	u32 keystore_data_size;

	/*
	 * Calculate the required size of the keystore data structure, based
	 * on the number of keys that can fit in the partition.
	 */
	slot_count = smpriv->page_size / smpriv->slot_size;
#ifdef SM_DEBUG
	dev_info(dev, "kso_init_data: %d slots initializing\n", slot_count);
#endif

	keystore_data_size = sizeof(struct keystore_data) +
				slot_count *
				sizeof(struct keystore_data_slot_info);

	keystore_data = kzalloc(keystore_data_size, GFP_KERNEL);

	if (!keystore_data)
		return -ENOMEM;

#ifdef SM_DEBUG
	dev_info(dev, "kso_init_data: keystore data size = %d\n",
		 keystore_data_size);
#endif

	/*
	 * Place the slot information structure directly after the keystore data
	 * structure.
	 */
	keystore_data->slot = (struct keystore_data_slot_info *)
			      (keystore_data + 1);
	keystore_data->slot_count = slot_count;

	smpriv->pagedesc[unit].ksdata = keystore_data;
	smpriv->pagedesc[unit].ksdata->base_address =
		smpriv->pagedesc[unit].pg_base;
	smpriv->pagedesc[unit].ksdata->phys_address =
		smpriv->pagedesc[unit].pg_phys;

	return 0;
}

void kso_cleanup_data(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	struct keystore_data *keystore_data = NULL;

	if (smpriv->pagedesc[unit].ksdata != NULL)
		keystore_data = smpriv->pagedesc[unit].ksdata;

	/* Release the allocated keystore management data */
	kfree(smpriv->pagedesc[unit].ksdata);

	return;
}



/*
 * Keystore management section
 */

void sm_init_keystore(struct device *dev)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	smpriv->data_init = kso_init_data;
	smpriv->data_cleanup = kso_cleanup_data;
	smpriv->slot_alloc = slot_alloc;
	smpriv->slot_dealloc = slot_dealloc;
	smpriv->slot_get_address = slot_get_address;
	smpriv->slot_get_physical = slot_get_physical;
	smpriv->slot_get_base = slot_get_base;
	smpriv->slot_get_offset = slot_get_offset;
	smpriv->slot_get_slot_size = slot_get_slot_size;
#ifdef SM_DEBUG
	dev_info(dev, "sm_init_keystore(): handlers installed\n");
#endif
}
EXPORT_SYMBOL(sm_init_keystore);

/* Return available pages/units */
u32 sm_detect_keystore_units(struct device *dev)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	return smpriv->localpages;
}
EXPORT_SYMBOL(sm_detect_keystore_units);

/*
 * Do any keystore specific initializations
 */
int sm_establish_keystore(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

#ifdef SM_DEBUG
	dev_info(dev, "sm_establish_keystore(): unit %d initializing\n", unit);
#endif

	if (smpriv->data_init == NULL)
		return -EINVAL;

	/* Call the data_init function for any user setup */
	return smpriv->data_init(dev, unit);
}
EXPORT_SYMBOL(sm_establish_keystore);

void sm_release_keystore(struct device *dev, u32 unit)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

#ifdef SM_DEBUG
	dev_info(dev, "sm_establish_keystore(): unit %d releasing\n", unit);
#endif
	if ((smpriv != NULL) && (smpriv->data_cleanup != NULL))
		smpriv->data_cleanup(dev, unit);

	return;
}
EXPORT_SYMBOL(sm_release_keystore);

/*
 * Subsequent interfacce (sm_keystore_*) forms the accessor interfacce to
 * the keystore
 */
int sm_keystore_slot_alloc(struct device *dev, u32 unit, u32 size, u32 *slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;

	spin_lock(&smpriv->kslock);

	if ((smpriv->slot_alloc == NULL) ||
	    (smpriv->pagedesc[unit].ksdata == NULL))
		goto out;

	retval =  smpriv->slot_alloc(dev, unit, size, slot);

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_alloc);

int sm_keystore_slot_dealloc(struct device *dev, u32 unit, u32 slot)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;

	spin_lock(&smpriv->kslock);

	if ((smpriv->slot_alloc == NULL) ||
	    (smpriv->pagedesc[unit].ksdata == NULL))
		goto out;

	retval = smpriv->slot_dealloc(dev, unit, slot);
out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_dealloc);

int sm_keystore_slot_load(struct device *dev, u32 unit, u32 slot,
			  const u8 *key_data, u32 key_length)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;
	u32 slot_size;
	u8 __iomem *slot_location;

	spin_lock(&smpriv->kslock);

	slot_size = smpriv->slot_get_slot_size(dev, unit, slot);

	if (key_length > slot_size) {
		retval = -EFBIG;
		goto out;
	}

	slot_location = smpriv->slot_get_address(dev, unit, slot);

	memcpy_toio(slot_location, key_data, key_length);

	retval = 0;

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_load);

int sm_keystore_slot_read(struct device *dev, u32 unit, u32 slot,
			  u32 key_length, u8 *key_data)
{
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);
	int retval = -EINVAL;
	u8 __iomem *slot_addr;
	u32 slot_size;

	spin_lock(&smpriv->kslock);

	slot_addr = smpriv->slot_get_address(dev, unit, slot);
	slot_size = smpriv->slot_get_slot_size(dev, unit, slot);

	if (key_length > slot_size) {
		retval = -EKEYREJECTED;
		goto out;
	}

	memcpy_fromio(key_data, slot_addr, key_length);
	retval = 0;

out:
	spin_unlock(&smpriv->kslock);
	return retval;
}
EXPORT_SYMBOL(sm_keystore_slot_read);

void *sm_keystore_get_slot_phys_addr(struct device *dev, u32 unit, u32 slot)
{
	void *retval = NULL;
	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	spin_lock(&smpriv->kslock);

	if (!smpriv->slot_get_physical)
		goto out;

	retval = smpriv->slot_get_physical(dev, unit, slot);

out:
	spin_unlock(&smpriv->kslock);
 	return retval;
}
EXPORT_SYMBOL(sm_keystore_get_slot_phys_addr);

u32 sm_keystore_get_slot_size(struct device *dev, u32 unit, u32 slot)
{
	int retval = 0;

	struct caam_drv_private_sm *smpriv = dev_get_drvdata(dev);

	spin_lock(&smpriv->kslock);

	if (!smpriv->slot_get_slot_size)
		goto out;

	retval = smpriv->slot_get_slot_size(dev, unit, slot);

out:
	spin_unlock(&smpriv->kslock);
 	return retval;
}
EXPORT_SYMBOL(sm_keystore_get_slot_size);

/*
 * Initialization/shutdown subsystem
 * Assumes statically-invoked startup/shutdown from the controller driver
 * for the present time, to be reworked when a device tree becomes
 * available. This code will not modularize in present form.
 *
 * Also, simply uses ring 0 for execution at the present
 */

int caam_sm_startup(struct platform_device *pdev)
{
	struct device *ctrldev, *smdev;
	struct caam_drv_private *ctrlpriv;
	struct caam_drv_private_sm *smpriv;
	struct caam_drv_private_jr *jrpriv;	/* need this for reg page */
	struct platform_device *sm_pdev;
	struct sm_page_descriptor *lpagedesc;
	u32 page, pgstat, lpagect, detectedpage, smvid, smpart;
	int ret = 0;

	struct device_node *np;
	ctrldev = &pdev->dev;
	ctrlpriv = dev_get_drvdata(ctrldev);

	/*
	 * If ctrlpriv is NULL, it's probably because the caam driver wasn't
	 * properly initialized (e.g. RNG4 init failed). Thus, bail out here.
	 */
	if (!ctrlpriv) {
		ret = -ENODEV;
		goto exit;
	}

	/*
	 * Set up the private block for secure memory
	 * Only one instance is possible
	 */
	smpriv = kzalloc(sizeof(struct caam_drv_private_sm), GFP_KERNEL);
	if (smpriv == NULL) {
		dev_err(ctrldev, "can't alloc private mem for secure memory\n");
		ret = -ENOMEM;
		goto exit;
	}
	smpriv->parentdev = ctrldev; /* copy of parent dev is handy */
	spin_lock_init(&smpriv->kslock);

	/* Create the dev */
	np = of_find_compatible_node(NULL, NULL, "fsl,imx6q-caam-sm");
	if (np)
		of_node_clear_flag(np, OF_POPULATED);
	sm_pdev = of_platform_device_create(np, "caam_sm", ctrldev);

	if (sm_pdev == NULL) {
		ret = -EINVAL;
		goto free_smpriv;
	}

	/* Save a pointer to the platform device for Secure Memory */
	smpriv->sm_pdev = sm_pdev;
	smdev = &sm_pdev->dev;
	dev_set_drvdata(smdev, smpriv);
	ctrlpriv->smdev = smdev;

	/* Set the Secure Memory Register Map Version */
	if (ctrlpriv->has_seco) {
		int i = ctrlpriv->first_jr_index;

		smvid = rd_reg32(&ctrlpriv->jr[i]->perfmon.smvid);
		smpart = rd_reg32(&ctrlpriv->jr[i]->perfmon.smpart);

	} else {
		smvid = rd_reg32(&ctrlpriv->ctrl->perfmon.smvid);
		smpart = rd_reg32(&ctrlpriv->ctrl->perfmon.smpart);

	}

	if (smvid < SMVID_V2)
		smpriv->sm_reg_offset = SM_V1_OFFSET;
	else
		smpriv->sm_reg_offset = SM_V2_OFFSET;

	/*
	 * Collect configuration limit data for reference
	 * This batch comes from the partition data/vid registers in perfmon
	 */
	smpriv->max_pages = ((smpart & SMPART_MAX_NUMPG_MASK) >>
			    SMPART_MAX_NUMPG_SHIFT) + 1;
	smpriv->top_partition = ((smpart & SMPART_MAX_PNUM_MASK) >>
				SMPART_MAX_PNUM_SHIFT) + 1;
	smpriv->top_page =  ((smpart & SMPART_MAX_PG_MASK) >>
				SMPART_MAX_PG_SHIFT) + 1;
	smpriv->page_size = 1024 << ((smvid & SMVID_PG_SIZE_MASK) >>
				SMVID_PG_SIZE_SHIFT);
	smpriv->slot_size = 1 << CONFIG_CRYPTO_DEV_FSL_CAAM_SM_SLOTSIZE;

#ifdef SM_DEBUG
	dev_info(smdev, "max pages = %d, top partition = %d\n",
			smpriv->max_pages, smpriv->top_partition);
	dev_info(smdev, "top page = %d, page size = %d (total = %d)\n",
			smpriv->top_page, smpriv->page_size,
			smpriv->top_page * smpriv->page_size);
	dev_info(smdev, "selected slot size = %d\n", smpriv->slot_size);
#endif

	/*
	 * Now probe for partitions/pages to which we have access. Note that
	 * these have likely been set up by a bootloader or platform
	 * provisioning application, so we have to assume that we "inherit"
	 * a configuration and work within the constraints of what it might be.
	 *
	 * Assume use of the zeroth ring in the present iteration (until
	 * we can divorce the controller and ring drivers, and then assign
	 * an SM instance to any ring instance).
	 */
	smpriv->smringdev = caam_jr_alloc();
	if (IS_ERR(smpriv->smringdev)) {
		dev_err(smdev, "Device for job ring not created\n");
		ret = -ENODEV;
		goto unregister_smpdev;
	}

	jrpriv = dev_get_drvdata(smpriv->smringdev);
	lpagect = 0;
	pgstat = 0;
	lpagedesc = kzalloc(sizeof(struct sm_page_descriptor)
			    * smpriv->max_pages, GFP_KERNEL);
	if (lpagedesc == NULL) {
		ret = -ENOMEM;
		goto free_smringdev;
	}

	for (page = 0; page < smpriv->max_pages; page++) {
		u32 page_ownership;

		if (sm_send_cmd(smpriv, jrpriv,
				((page << SMC_PAGE_SHIFT) & SMC_PAGE_MASK) |
				(SMC_CMD_PAGE_INQUIRY & SMC_CMD_MASK),
				&pgstat)) {
			ret = -EINVAL;
			goto free_lpagedesc;
		}

		page_ownership = (pgstat & SMCS_PGWON_MASK) >> SMCS_PGOWN_SHIFT;
		if ((page_ownership == SMCS_PGOWN_OWNED)
			|| (page_ownership == SMCS_PGOWN_NOOWN)) {
			/* page allocated */
			lpagedesc[page].phys_pagenum =
				(pgstat & SMCS_PAGE_MASK) >> SMCS_PAGE_SHIFT;
			lpagedesc[page].own_part =
				(pgstat & SMCS_PART_SHIFT) >> SMCS_PART_MASK;
			lpagedesc[page].pg_base = (u8 *)ctrlpriv->sm_base +
				(smpriv->page_size * page);
			if (ctrlpriv->has_seco) {
/* FIXME: get different addresses viewed by CPU and CAAM from
 * platform property
 */
				lpagedesc[page].pg_phys = (u8 *)0x20800000 +
					(smpriv->page_size * page);
			} else {
				lpagedesc[page].pg_phys =
					(u8 *) ctrlpriv->sm_phy +
					(smpriv->page_size * page);
			}
			lpagect++;
#ifdef SM_DEBUG
			dev_info(smdev,
				"physical page %d, owning partition = %d\n",
				lpagedesc[page].phys_pagenum,
				lpagedesc[page].own_part);
#endif
		}
	}

	smpriv->pagedesc = kzalloc(sizeof(struct sm_page_descriptor) * lpagect,
				   GFP_KERNEL);
	if (smpriv->pagedesc == NULL) {
		ret = -ENOMEM;
		goto free_lpagedesc;
	}
	smpriv->localpages = lpagect;

	detectedpage = 0;
	for (page = 0; page < smpriv->max_pages; page++) {
		if (lpagedesc[page].pg_base != NULL) {	/* e.g. live entry */
			memcpy(&smpriv->pagedesc[detectedpage],
			       &lpagedesc[page],
			       sizeof(struct sm_page_descriptor));
#ifdef SM_DEBUG_CONT
			sm_show_page(smdev, &smpriv->pagedesc[detectedpage]);
#endif
			detectedpage++;
		}
	}

	kfree(lpagedesc);

	sm_init_keystore(smdev);

	goto exit;

free_lpagedesc:
	kfree(lpagedesc);
free_smringdev:
	caam_jr_free(smpriv->smringdev);
unregister_smpdev:
	of_device_unregister(smpriv->sm_pdev);
free_smpriv:
	kfree(smpriv);

exit:
	return ret;
}

void caam_sm_shutdown(struct platform_device *pdev)
{
	struct device *ctrldev, *smdev;
	struct caam_drv_private *priv;
	struct caam_drv_private_sm *smpriv;

	ctrldev = &pdev->dev;
	priv = dev_get_drvdata(ctrldev);
	smdev = priv->smdev;

	/* Return if resource not initialized by startup */
	if (smdev == NULL)
		return;

	smpriv = dev_get_drvdata(smdev);

	caam_jr_free(smpriv->smringdev);

	/* Remove Secure Memory Platform Device */
	of_device_unregister(smpriv->sm_pdev);

	kfree(smpriv->pagedesc);
	kfree(smpriv);
}
EXPORT_SYMBOL(caam_sm_shutdown);

static void  __exit caam_sm_exit(void)
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

	caam_sm_shutdown(pdev);
}

static int __init caam_sm_init(void)
{
	struct device_node *dev_node;
	struct platform_device *pdev;

	/*
	 * Do of_find_compatible_node() then of_find_device_by_node()
	 * once a functional device tree is available
	 */
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

	caam_sm_startup(pdev);

	return 0;
}

module_init(caam_sm_init);
module_exit(caam_sm_exit);

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("FSL CAAM Secure Memory / Keystore");
MODULE_AUTHOR("Freescale Semiconductor - NMSG/MAD");
