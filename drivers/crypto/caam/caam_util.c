// SPDX-License-Identifier: GPL-2.0
/*
 * Utility functions for CAAM
 *
 * Copyright 2018 NXP
 */

#include <linux/dma-mapping.h>
#include <linux/slab.h>

#include "caam.h"
#include "caam_util.h"

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
int prepare_write_data(struct device *jrdev, const u8 *data, size_t size,
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
caam_dma_addr_t get_caam_dma_addr(const void *phy_address)
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
		if ((uintptr_t)phy_address & ~mask)
			goto exit;
	}

	/* We convert address to caam_dma_address */
	ptr_conv = (uintptr_t)phy_address;
	caam_dma_address = (caam_dma_addr_t)ptr_conv;

exit:
	return caam_dma_address;
}
EXPORT_SYMBOL(get_caam_dma_addr);
