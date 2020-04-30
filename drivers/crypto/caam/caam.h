/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2018 NXP
 */

#ifndef _CAAM_H_
#define _CAAM_H_

#include <linux/types.h>

/* Retrieve a reference on a CAAM JR */
struct device *caam_jr_alloc(void);

/* Release a reference on a CAAM JR */
void caam_jr_free(struct device *jrdev);

/* Enqueue a job to be processed
 *
 * The function cbk will be called when the job is completed with:
 *  - jrdev the device used for enqueing
 *  - desc the descriptor processed
 *  - status the status of the processing: 0 if completed with no error,
 *    another value if there was an error. The error can be decoded with
 *    caam_jr_strstatus
 *  - areq pointer on user specific data
 */
int caam_jr_enqueue(struct device *jrdev, u32 *desc,
		    void (*cbk)(struct device *jrdev, u32 *desc,
				u32 status, void *areq),
		    void *areq);

/* Print a description of status
 * status is the error code passed to the callback when enqueing a job with
 * caam_jr_enqueue
 */
void caam_jr_strstatus(struct device *jrdev, u32 status);

/* Enqueue a job and wait for its completion */
int jr_run_job_and_wait_completion(struct device *jrdev, u32 *jobdesc);

#endif /* _CAAM_H_ */
