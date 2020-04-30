/*
 * CAAM/SEC 4.x transport/backend driver
 * JobR backend functionality
 *
 * Copyright 2008-2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 */

#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "caam.h"
#include "compat.h"
#include "ctrl.h"
#include "regs.h"
#include "jr.h"
#include "desc.h"
#include "intern.h"
#include "inst_rng.h"

struct jr_driver_data {
	/* List of Physical JobR's with the Driver */
	struct list_head	jr_list;
	spinlock_t		jr_alloc_lock;	/* jr_list lock */
} ____cacheline_aligned;

static struct jr_driver_data driver_data;

static inline void mask_itr(struct device *dev)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);

	clrsetbits_32(&jrp->rregs->rconfig_lo, 0, JRCFG_IMSK);
}

static inline void unmask_itr(struct device *dev)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);

	clrsetbits_32(&jrp->rregs->rconfig_lo, JRCFG_IMSK, 0);
}

/*
 * Put the CAAM in quiesce, ie stop
 *
 * Must be called with itr disabled
 */
static int caam_jr_stop_processing(struct device *dev, u32 jrcr_bits)
{
	unsigned int timeout = 100000;
	unsigned int halt_status;
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);

	if (rd_reg32(&jrp->rregs->jrintstatus) & JRINT_ERR_HALT_MASK) {
		dev_err(dev, "Not ready to quiesce\n");
		return -EINVAL;
	}

	/* initiate quiesce */
	wr_reg32(&jrp->rregs->jrcommand, jrcr_bits);

	/* Wait for the quiesce completion or timeout run out */
	do {
		cpu_relax();
		halt_status = rd_reg32(&jrp->rregs->jrintstatus) &
			      JRINT_ERR_HALT_MASK;
	} while ((halt_status == JRINT_ERR_HALT_INPROGRESS) &&
		 --timeout);

	halt_status = rd_reg32(&jrp->rregs->jrintstatus) & JRINT_ERR_HALT_MASK;

	/* Check that the flush is complete */
	if (halt_status != JRINT_ERR_HALT_COMPLETE) {
		dev_err(dev, "failed to quiesce\n");
		return -EIO;
	}

	return 0;
}

/*
 * Flush the job ring, so the jobs running will be stopped, jobs queued will be
 * invalidated and the CAAM will no longer fetch fron input ring.
 *
 * Must be called with itr disabled
 */
static int caam_jr_flush(struct device *dev)
{
	return caam_jr_stop_processing(dev, JRCR_RESET);
}

#ifdef CONFIG_PM_SLEEP

/* The resume can be used after a park or a flush if CAAM has not been reset */
static int caam_jr_restart_processing(struct device *dev)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	u32 park_status = rd_reg32(&jrp->rregs->jrintstatus) &
			  JRINT_ERR_HALT_MASK;

	/* Check that the flush/park is completed */
	if (park_status != JRINT_ERR_HALT_COMPLETE)
		return -1;

	/* Resume processing of jobs */
	wr_reg32(&jrp->rregs->jrintstatus, JRINT_ERR_HALT_COMPLETE);

	return 0;
}

#endif /* CONFIG_PM_SLEEP */

static int caam_reset_hw_jr(struct device *dev)
{
	int err = 0;
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	unsigned int timeout = 100000;
	unsigned int reset_status;

	/*
	 * mask interrupts since we are going to poll
	 * for reset completion status
	 */
	mask_itr(dev);

	err = caam_jr_flush(dev);
	if (err)
		return err;

	/* initiate reset */
	wr_reg32(&jrp->rregs->jrcommand, JRCR_RESET);
	do {
		cpu_relax();
		reset_status = rd_reg32(&jrp->rregs->jrcommand) & JRCR_RESET;
	} while (reset_status && --timeout);

	reset_status = rd_reg32(&jrp->rregs->jrcommand) & JRCR_RESET;
	if (reset_status != 0) {
		dev_err(dev, "failed to reset job ring %d\n", jrp->ridx);
		return -EIO;
	}

	/* unmask interrupts */
	unmask_itr(dev);

	return 0;
}

/*
 * Shutdown JobR independent of platform property code
 */
static int caam_jr_shutdown(struct device *dev)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	dma_addr_t inpbusaddr, outbusaddr;
	int ret;

	ret = caam_reset_hw_jr(dev);

	tasklet_kill(&jrp->irqtask);

	/* Release interrupt */
	free_irq(jrp->irq, dev);

	/* Free rings */
	inpbusaddr = rd_reg64(&jrp->rregs->inpring_base);
	outbusaddr = rd_reg64(&jrp->rregs->outring_base);
	dma_free_coherent(dev, sizeof(dma_addr_t) * JOBR_DEPTH,
			  jrp->inpring, inpbusaddr);
	dma_free_coherent(dev, sizeof(struct jr_outentry) * JOBR_DEPTH,
			  jrp->outring, outbusaddr);
	kfree(jrp->entinfo);

	return ret;
}

static int caam_jr_remove(struct platform_device *pdev)
{
	int ret;
	struct device *jrdev;
	struct caam_drv_private_jr *jrpriv;

	jrdev = &pdev->dev;
	jrpriv = dev_get_drvdata(jrdev);

	/*
	 * Deinstantiate RNG by first JR
	 */
	if (jrpriv->ridx == 0)
		deinst_rng(pdev);

	/*
	 * Return EBUSY if job ring already allocated.
	 */
	if (atomic_read(&jrpriv->tfm_count)) {
		dev_err(jrdev, "Device is busy\n");
		return -EBUSY;
	}

	/* Remove the node from Physical JobR list maintained by driver */
	spin_lock(&driver_data.jr_alloc_lock);
	list_del(&jrpriv->list_node);
	spin_unlock(&driver_data.jr_alloc_lock);

	/* Release ring */
	ret = caam_jr_shutdown(jrdev);
	if (ret)
		dev_err(jrdev, "Failed to shut down job ring\n");
	irq_dispose_mapping(jrpriv->irq);

	return ret;
}

/* Main per-ring interrupt handler */
static irqreturn_t caam_jr_interrupt(int irq, void *st_dev)
{
	struct device *dev = st_dev;
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	u32 irqstate;

	/*
	 * Check the output ring for ready responses, kick
	 * tasklet if jobs done.
	 */
	irqstate = rd_reg32(&jrp->rregs->jrintstatus);
	if (!irqstate)
		return IRQ_NONE;

	/*
	 * If JobR error, we got more development work to do
	 * Flag a bug now, but we really need to shut down and
	 * restart the queue (and fix code).
	 */
	if (irqstate & JRINT_JR_ERROR) {
		dev_err(dev, "job ring error: irqstate: %08x\n", irqstate);
		BUG();
	}

	/* mask valid interrupts */
	mask_itr(dev);

	/* Have valid interrupt at this point, just ACK and trigger */
	wr_reg32(&jrp->rregs->jrintstatus, irqstate);

	preempt_disable();
	tasklet_schedule(&jrp->irqtask);
	preempt_enable();

	return IRQ_HANDLED;
}

/* Deferred service handler, run as interrupt-fired tasklet */
static void caam_jr_dequeue(unsigned long data)
{
	int hw_idx, sw_idx, i, head, tail;
	struct caam_jr_dequeue_params *params = (void *)data;
	struct device *dev = params->dev;
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	void (*usercall)(struct device *dev, u32 *desc, u32 status, void *arg);
	u32 *userdesc, userstatus;
	void *userarg;

	while (rd_reg32(&jrp->rregs->outring_used)) {

		head = ACCESS_ONCE(jrp->head);

		spin_lock(&jrp->outlock);

		sw_idx = tail = jrp->tail;
		hw_idx = jrp->out_ring_read_index;

		for (i = 0; CIRC_CNT(head, tail + i, JOBR_DEPTH) >= 1; i++) {
			sw_idx = (tail + i) & (JOBR_DEPTH - 1);

			if (jrp->outring[hw_idx].desc ==
			    caam_dma_to_cpu(jrp->entinfo[sw_idx].desc_addr_dma))
				break; /* found */
		}
		/* we should never fail to find a matching descriptor */
		BUG_ON(CIRC_CNT(head, tail + i, JOBR_DEPTH) <= 0);

		/* Unmap just-run descriptor so we can post-process */
		dma_unmap_single(dev,
				 caam_dma_to_cpu(jrp->outring[hw_idx].desc),
				 jrp->entinfo[sw_idx].desc_size,
				 DMA_TO_DEVICE);

		/* mark completed, avoid matching on a recycled desc addr */
		jrp->entinfo[sw_idx].desc_addr_dma = 0;

		/* Stash callback params for use outside of lock */
		usercall = jrp->entinfo[sw_idx].callbk;
		userarg = jrp->entinfo[sw_idx].cbkarg;
		userdesc = jrp->entinfo[sw_idx].desc_addr_virt;
		userstatus = caam32_to_cpu(jrp->outring[hw_idx].jrstatus);

		/*
		 * Make sure all information from the job has been obtained
		 * before telling CAAM that the job has been removed from the
		 * output ring.
		 */
		mb();

		/* set done */
		wr_reg32(&jrp->rregs->outring_rmvd, 1);

		jrp->out_ring_read_index = (jrp->out_ring_read_index + 1) &
					   (JOBR_DEPTH - 1);

		/*
		 * if this job completed out-of-order, do not increment
		 * the tail.  Otherwise, increment tail by 1 plus the
		 * number of subsequent jobs already completed out-of-order
		 */
		if (sw_idx == tail) {
			do {
				tail = (tail + 1) & (JOBR_DEPTH - 1);
			} while (CIRC_CNT(head, tail, JOBR_DEPTH) >= 1 &&
				 jrp->entinfo[tail].desc_addr_dma == 0);

			jrp->tail = tail;
		}

		spin_unlock(&jrp->outlock);

		/* Finally, execute user's callback */
		usercall(dev, userdesc, userstatus, userarg);
	}

	/* reenable / unmask IRQs */
	if (params->enable_itr)
		unmask_itr(dev);
}

/**
 * caam_jr_alloc() - Alloc a job ring for someone to use as needed.
 *
 * returns :  pointer to the newly allocated physical
 *	      JobR dev can be written to if successful.
 **/
struct device *caam_jr_alloc(void)
{
	struct caam_drv_private_jr *jrpriv, *min_jrpriv = NULL;
	struct device *dev = ERR_PTR(-ENODEV);
	int min_tfm_cnt	= INT_MAX;
	int tfm_cnt;

	spin_lock(&driver_data.jr_alloc_lock);

	if (list_empty(&driver_data.jr_list)) {
		spin_unlock(&driver_data.jr_alloc_lock);
		return ERR_PTR(-ENODEV);
	}

	list_for_each_entry(jrpriv, &driver_data.jr_list, list_node) {
		tfm_cnt = atomic_read(&jrpriv->tfm_count);
		if (tfm_cnt < min_tfm_cnt) {
			min_tfm_cnt = tfm_cnt;
			min_jrpriv = jrpriv;
		}
		if (!min_tfm_cnt)
			break;
	}

	if (min_jrpriv) {
		atomic_inc(&min_jrpriv->tfm_count);
		dev = min_jrpriv->dev;
	}
	spin_unlock(&driver_data.jr_alloc_lock);

	return dev;
}
EXPORT_SYMBOL(caam_jr_alloc);

/**
 * caam_jr_free() - Free the Job Ring
 * @rdev     - points to the dev that identifies the Job ring to
 *             be released.
 **/
void caam_jr_free(struct device *rdev)
{
	struct caam_drv_private_jr *jrpriv = dev_get_drvdata(rdev);

	atomic_dec(&jrpriv->tfm_count);
}
EXPORT_SYMBOL(caam_jr_free);

/**
 * caam_jr_enqueue() - Enqueue a job descriptor head. Returns 0 if OK,
 * -EBUSY if the queue is full, -EIO if it cannot map the caller's
 * descriptor.
 * @dev:  device of the job ring to be used.
 * @desc: points to a job descriptor that execute our request. All
 *        descriptors (and all referenced data) must be in a DMAable
 *        region, and all data references must be physical addresses
 *        accessible to CAAM (i.e. within a PAMU window granted
 *        to it).
 * @cbk:  pointer to a callback function to be invoked upon completion
 *        of this request. This has the form:
 *        callback(struct device *dev, u32 *desc, u32 stat, void *arg)
 *        where:
 *        @dev:    contains the job ring device that processed this
 *                 response.
 *        @desc:   descriptor that initiated the request, same as
 *                 "desc" being argued to caam_jr_enqueue().
 *        @status: untranslated status received from CAAM. See the
 *                 reference manual for a detailed description of
 *                 error meaning, or see the JRSTA definitions in the
 *                 register header file
 *        @areq:   optional pointer to an argument passed with the
 *                 original request
 * @areq: optional pointer to a user argument for use at callback
 *        time.
 **/
int caam_jr_enqueue(struct device *dev, u32 *desc,
		    void (*cbk)(struct device *dev, u32 *desc,
				u32 status, void *areq),
		    void *areq)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);
	struct caam_jrentry_info *head_entry;
	int head, tail, desc_size;
	dma_addr_t desc_dma;

	desc_size = (caam32_to_cpu(*desc) & HDR_JD_LENGTH_MASK) * sizeof(u32);
	desc_dma = dma_map_single(dev, desc, desc_size, DMA_TO_DEVICE);
	if (dma_mapping_error(dev, desc_dma)) {
		dev_err(dev, "caam_jr_enqueue(): can't map jobdesc\n");
		return -EIO;
	}

	spin_lock_bh(&jrp->inplock);

	head = jrp->head;
	tail = ACCESS_ONCE(jrp->tail);

	if (!rd_reg32(&jrp->rregs->inpring_avail) ||
	    CIRC_SPACE(head, tail, JOBR_DEPTH) <= 0) {
		spin_unlock_bh(&jrp->inplock);
		dma_unmap_single(dev, desc_dma, desc_size, DMA_TO_DEVICE);
		return -EBUSY;
	}

	head_entry = &jrp->entinfo[head];
	head_entry->desc_addr_virt = desc;
	head_entry->desc_size = desc_size;
	head_entry->callbk = (void *)cbk;
	head_entry->cbkarg = areq;
	head_entry->desc_addr_dma = desc_dma;

	jrp->inpring[jrp->inp_ring_write_index] = cpu_to_caam_dma(desc_dma);
	/*
	 * Guarantee that the descriptor's DMA address has been written to
	 * the next slot in the ring before the write index is updated, since
	 * other cores may update this index independently.
	 */
	smp_wmb();

	jrp->inp_ring_write_index = (jrp->inp_ring_write_index + 1) &
				    (JOBR_DEPTH - 1);
	jrp->head = (head + 1) & (JOBR_DEPTH - 1);

	/*
	 * Ensure that all job information has been written before
	 * notifying CAAM that a new job was added to the input ring.
	 */
	wmb();

	wr_reg32(&jrp->rregs->inpring_jobadd, 1);

	spin_unlock_bh(&jrp->inplock);

	return 0;
}
EXPORT_SYMBOL(caam_jr_enqueue);

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
		caam_jr_strstatus(jrdev, err);

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

static void caam_jr_init_hw(struct device *dev, dma_addr_t inpbusaddr,
			    dma_addr_t outbusaddr)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);

	wr_reg64(&jrp->rregs->inpring_base, inpbusaddr);
	wr_reg64(&jrp->rregs->outring_base, outbusaddr);
	wr_reg32(&jrp->rregs->inpring_size, JOBR_DEPTH);
	wr_reg32(&jrp->rregs->outring_size, JOBR_DEPTH);

	/* Select interrupt coalescing parameters */
	clrsetbits_32(&jrp->rregs->rconfig_lo, 0, JOBR_INTC |
		      (JOBR_INTC_COUNT_THLD << JRCFG_ICDCT_SHIFT) |
		      (JOBR_INTC_TIME_THLD << JRCFG_ICTT_SHIFT));
}

static void caam_jr_reset_index(struct caam_drv_private_jr *jrp)
{
	jrp->inp_ring_write_index = 0;
	jrp->out_ring_read_index = 0;
	jrp->head = 0;
	jrp->tail = 0;
}

/*
 * Init JobR independent of platform property detection
 */
static int caam_jr_init(struct device *dev)
{
	struct caam_drv_private_jr *jrp;
	dma_addr_t inpbusaddr, outbusaddr;
	int i, error;

	jrp = dev_get_drvdata(dev);

	error = caam_reset_hw_jr(dev);
	if (error)
		goto out_kill_deq;

	jrp->tasklet_params.dev = dev;
	jrp->tasklet_params.enable_itr = 1;
	tasklet_init(&jrp->irqtask, caam_jr_dequeue,
		     (unsigned long)&jrp->tasklet_params);

	/* Connect job ring interrupt handler. */
	error = request_irq(jrp->irq, caam_jr_interrupt, IRQF_SHARED,
			    dev_name(dev), dev);
	if (error) {
		dev_err(dev, "can't connect JobR %d interrupt (%d)\n",
			jrp->ridx, jrp->irq);
		goto out_kill_deq;
	}

	error = -ENOMEM;
	jrp->inpring = dma_alloc_coherent(dev, sizeof(*jrp->inpring) *
					  JOBR_DEPTH, &inpbusaddr, GFP_KERNEL);
	if (!jrp->inpring)
		goto out_free_irq;

	jrp->outring = dma_alloc_coherent(dev, sizeof(*jrp->outring) *
					  JOBR_DEPTH, &outbusaddr, GFP_KERNEL);
	if (!jrp->outring)
		goto out_free_inpring;

	jrp->entinfo = kcalloc(JOBR_DEPTH, sizeof(*jrp->entinfo), GFP_KERNEL);
	if (!jrp->entinfo)
		goto out_free_outring;

	for (i = 0; i < JOBR_DEPTH; i++)
		jrp->entinfo[i].desc_addr_dma = !0;

	/* Setup rings */
	caam_jr_reset_index(jrp);

	jrp->ringsize = JOBR_DEPTH;

	caam_jr_init_hw(dev, inpbusaddr, outbusaddr);

	spin_lock_init(&jrp->inplock);
	spin_lock_init(&jrp->outlock);

	return 0;

out_free_outring:
	dma_free_coherent(dev, sizeof(struct jr_outentry) * JOBR_DEPTH,
			  jrp->outring, outbusaddr);
out_free_inpring:
	dma_free_coherent(dev, sizeof(dma_addr_t) * JOBR_DEPTH,
			  jrp->inpring, inpbusaddr);
	dev_err(dev, "can't allocate job rings for %d\n", jrp->ridx);
out_free_irq:
	free_irq(jrp->irq, dev);
out_kill_deq:
	tasklet_kill(&jrp->irqtask);
	return error;
}

static int caam_jr_instantiate_rng(struct device *jrdev)
{
	int error = 0;
	struct caam_drv_private_jr *jrpriv;
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(jrdev->parent);

	if (ctrlpriv->has_seco || ctrlpriv->has_optee) {
		dev_dbg(jrdev, "RNG instantiated by secure component\n");
		goto exit;
	}

	jrpriv = dev_get_drvdata(jrdev);

	/*
	 * If this is the first available JR
	 * then try to instantiate RNG
	 */
	if (jrpriv->ridx == 0)
		error = inst_rng_imx(to_platform_device(jrdev));

exit:
	return error;
}

/*
 * Probe routine for each detected JobR subsystem.
 */
static int caam_jr_probe(struct platform_device *pdev)
{
	struct device *jrdev;
	struct device_node *nprop;
	struct caam_job_ring __iomem *ctrl;
	struct caam_drv_private_jr *jrpriv;
	static int total_jobrs;
	int error;

	jrdev = &pdev->dev;
	jrpriv = devm_kmalloc(jrdev, sizeof(*jrpriv), GFP_KERNEL);
	if (!jrpriv) {
		error = -ENOMEM;
		goto exit;
	}

	dev_set_drvdata(jrdev, jrpriv);
	jrpriv->dev = jrdev;

	/* save ring identity relative to detection */
	jrpriv->ridx = total_jobrs++;

	nprop = pdev->dev.of_node;
	/* Get configuration properties from device tree */
	/* First, get register page */
	ctrl = of_iomap(nprop, 0);
	if (!ctrl) {
		dev_err(jrdev, "of_iomap() failed\n");
		error = -ENOMEM;
		goto exit;
	}

	jrpriv->rregs = (struct caam_job_ring __iomem __force *)ctrl;

	if (of_machine_is_compatible("fsl,imx8mm") ||
		of_machine_is_compatible("fsl,imx8qm") ||
		of_machine_is_compatible("fsl,imx8qxp") ||
		of_machine_is_compatible("fsl,imx8mq")) {
		error = dma_set_mask_and_coherent(jrdev, DMA_BIT_MASK(32));
	} else if (sizeof(dma_addr_t) == sizeof(u64)) {
		if (caam_dpaa2)
			error = dma_set_mask_and_coherent(jrdev,
							  DMA_BIT_MASK(49));
		else if (of_device_is_compatible(nprop,
						 "fsl,sec-v5.0-job-ring"))
			error = dma_set_mask_and_coherent(jrdev,
							  DMA_BIT_MASK(40));
		else
			error = dma_set_mask_and_coherent(jrdev,
							  DMA_BIT_MASK(36));
	} else {
		error = dma_set_mask_and_coherent(jrdev, DMA_BIT_MASK(32));
	}
	if (error) {
		dev_err(jrdev, "dma_set_mask_and_coherent failed (%d)\n",
			error);
		goto unmap_ctrl;
	}

	/* Identify the interrupt */
	jrpriv->irq = irq_of_parse_and_map(nprop, 0);
	if (jrpriv->irq <= 0) {
		kfree(jrpriv);
		return -EINVAL;
	}

	/* Now do the platform independent part */
	error = caam_jr_init(jrdev); /* now turn on hardware */
	if (error)
		goto dispose_irq_mapping;

	spin_lock(&driver_data.jr_alloc_lock);
	list_add_tail(&jrpriv->list_node, &driver_data.jr_list);
	spin_unlock(&driver_data.jr_alloc_lock);

	atomic_set(&jrpriv->tfm_count, 0);

	device_init_wakeup(&pdev->dev, 1);
	device_set_wakeup_enable(&pdev->dev, false);

	error = caam_jr_instantiate_rng(jrdev);
	if (error)
		goto remove_jr_from_list;

	goto exit;

remove_jr_from_list:
	spin_lock(&driver_data.jr_alloc_lock);
	list_del(&jrpriv->list_node);
	spin_unlock(&driver_data.jr_alloc_lock);
dispose_irq_mapping:
	irq_dispose_mapping(jrpriv->irq);
unmap_ctrl:
	iounmap(ctrl);

exit:
	return error;
}

#ifdef CONFIG_PM_SLEEP

static void caam_jr_get_hw_state(struct device *dev)
{
	struct caam_drv_private_jr *jrp = dev_get_drvdata(dev);

	jrp->state.inpbusaddr = rd_reg64(&jrp->rregs->inpring_base);
	jrp->state.outbusaddr = rd_reg64(&jrp->rregs->outring_base);
}

static int caam_jr_suspend(struct device *dev)
{
	int err = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct caam_drv_private_jr *jrpriv = platform_get_drvdata(pdev);
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(dev->parent);
	struct caam_jr_dequeue_params suspend_params = {
		.dev = dev,
		.enable_itr = 0,
	};

	if (ctrlpriv->caam_off_during_pm) {
		tasklet_disable(&jrpriv->irqtask);

		/* mask itr to call flush */
		mask_itr(dev);

		/* Invalid job in process */
		err = caam_jr_flush(dev);
		if (err) {
			dev_err(dev, "Failed to flush\n");
			goto exit;
		}

		/* Dequeing jobs flushed */
		caam_jr_dequeue((unsigned long)&suspend_params);

		/* Save state */
		caam_jr_get_hw_state(dev);

	} else if (device_may_wakeup(&pdev->dev)) {
		enable_irq_wake(jrpriv->irq);
	}

exit:
	return err;
}

static int caam_jr_resume(struct device *dev)
{
	int err = 0;
	struct platform_device *pdev = to_platform_device(dev);
	struct caam_drv_private_jr *jrpriv = platform_get_drvdata(pdev);
	struct caam_drv_private *ctrlpriv = dev_get_drvdata(dev->parent);

	if (ctrlpriv->caam_off_during_pm) {
		u64 inp_addr;

		/*
		 * Check if the CAAM has been resetted checking the address of
		 * the input ring
		 */
		inp_addr = rd_reg64(&jrpriv->rregs->inpring_base);
		if (inp_addr != 0) {
			/* JR still has some configuration */
			if (inp_addr == jrpriv->state.inpbusaddr) {
				/* JR has not been resetted */
				err = caam_jr_restart_processing(dev);
				if (err)
					goto exit;

				tasklet_enable(&jrpriv->irqtask);

				unmask_itr(dev);

				goto exit;
			} else if (ctrlpriv->has_optee) {
				/* JR has been used by OPTEE, reset it */
				err = caam_reset_hw_jr(dev);
				if (err) {
					dev_err(dev, "Failed to reset JR\n");
					goto exit;
				}
			} else {
				/* No explanation, return error */
				err = -EIO;
				goto exit;
			}
		}

		caam_jr_reset_index(jrpriv);

		caam_jr_init_hw(dev, jrpriv->state.inpbusaddr,
				jrpriv->state.outbusaddr);

		tasklet_enable(&jrpriv->irqtask);

		err = caam_jr_instantiate_rng(dev);
		if (err) {
			dev_err(dev, "Failed to instantiate RNG\n");
			goto exit;
		}

	} else if (device_may_wakeup(&pdev->dev)) {
		disable_irq_wake(jrpriv->irq);
	}

exit:
	return err;
}

SIMPLE_DEV_PM_OPS(caam_jr_pm_ops, caam_jr_suspend, caam_jr_resume);

#endif /* CONFIG_PM_SLEEP */

static const struct of_device_id caam_jr_match[] = {
	{
		.compatible = "fsl,sec-v4.0-job-ring",
	},
	{
		.compatible = "fsl,sec4.0-job-ring",
	},
	{},
};
MODULE_DEVICE_TABLE(of, caam_jr_match);

static struct platform_driver caam_jr_driver = {
	.driver = {
		.name = "caam_jr",
		.of_match_table = caam_jr_match,
#ifdef CONFIG_PM_SLEEP
		.pm = &caam_jr_pm_ops,
#endif
	},
	.probe       = caam_jr_probe,
	.remove      = caam_jr_remove,
};

static int __init jr_driver_init(void)
{
	spin_lock_init(&driver_data.jr_alloc_lock);
	INIT_LIST_HEAD(&driver_data.jr_list);
	return platform_driver_register(&caam_jr_driver);
}

static void __exit jr_driver_exit(void)
{
	platform_driver_unregister(&caam_jr_driver);
}

module_init(jr_driver_init);
module_exit(jr_driver_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FSL CAAM JR request backend");
MODULE_AUTHOR("Freescale Semiconductor - NMG/STC");
