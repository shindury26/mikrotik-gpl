/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef _DRIVERS_CRYPTO_MSM_QCE50_H_
#define _DRIVERS_CRYPTO_MSM_QCE50_H_

#include <linux/msm-sps.h>
#include "qcryptohw_50.h"

/* MAX Data xfer block size between BAM and CE */
#define MAX_CE_BAM_BURST_SIZE   0x40

#define CRYPTO_REG_SIZE 4
#define NUM_OF_CRYPTO_AUTH_IV_REG 16
#define NUM_OF_CRYPTO_CNTR_IV_REG 4
#define NUM_OF_CRYPTO_AUTH_BYTE_COUNT_REG 4
#define CRYPTO_TOTAL_REGISTERS_DUMPED   26
#define CRYPTO_RESULT_DUMP_SIZE   \
	ALIGN((CRYPTO_TOTAL_REGISTERS_DUMPED * CRYPTO_REG_SIZE), \
	MAX_CE_BAM_BURST_SIZE)

/* State of consumer/producer Pipe */
enum qce_pipe_st_enum {
	QCE_PIPE_STATE_IDLE = 0,
	QCE_PIPE_STATE_IN_PROG = 1,
	QCE_PIPE_STATE_COMP = 2,
	QCE_PIPE_STATE_LAST
};

struct qce_sps_ep_conn_data {
	struct sps_pipe			*pipe;
	struct sps_connect		connect;
	struct sps_register_event	event;
};

/* CE Result DUMP format*/
struct ce_result_dump_format {
	uint32_t auth_iv[NUM_OF_CRYPTO_AUTH_IV_REG];
	uint32_t auth_byte_count[NUM_OF_CRYPTO_AUTH_BYTE_COUNT_REG];
	uint32_t encr_cntr_iv[NUM_OF_CRYPTO_CNTR_IV_REG];
	uint32_t status;
	uint32_t status2;
} __attribute__((aligned(CRYPTO_RESULT_DUMP_SIZE)));

struct qce_ce_cfg_reg_setting {

	uint32_t encr_cfg_aes_cbc_128;
	uint32_t encr_cfg_aes_cbc_256;

	uint32_t encr_cfg_aes_ecb_128;
	uint32_t encr_cfg_aes_ecb_256;

	uint32_t encr_cfg_aes_xts_128;
	uint32_t encr_cfg_aes_xts_256;

	uint32_t encr_cfg_aes_ctr_128;
	uint32_t encr_cfg_aes_ctr_256;

	uint32_t encr_cfg_aes_ccm_128;
	uint32_t encr_cfg_aes_ccm_256;

	uint32_t encr_cfg_des_cbc;
	uint32_t encr_cfg_des_ecb;

	uint32_t encr_cfg_3des_cbc;
	uint32_t encr_cfg_3des_ecb;

	uint32_t auth_cfg_cmac_128;
	uint32_t auth_cfg_cmac_256;

	uint32_t auth_cfg_sha1;
	uint32_t auth_cfg_sha256;

	uint32_t auth_cfg_hmac_sha1;
	uint32_t auth_cfg_hmac_sha256;

	uint32_t auth_cfg_aes_ccm_128;
	uint32_t auth_cfg_aes_ccm_256;
	uint32_t auth_cfg_aead_sha1_hmac;
	uint32_t auth_cfg_aead_sha256_hmac;
};

/* DM data structure with buffers, commandlists & commmand pointer lists */
struct ce_sps_data {
	uint32_t			bam_irq;
	uint32_t			bam_mem;
	void __iomem			*bam_iobase;

	struct qce_sps_ep_conn_data	producer;
	struct qce_sps_ep_conn_data	consumer;
	uint32_t			ce_device;
	unsigned int			pipe_pair_index;
	unsigned int			src_pipe_index;
	unsigned int			dest_pipe_index;
	unsigned long			bam_handle;
};

struct bam_registration_info {
	struct list_head qlist;
	unsigned long handle;
	uint32_t cnt;
	uint32_t bam_mem;
	void __iomem *bam_iobase;
};
/*
 * CE HW device structure.
 * Each engine has an instance of the structure.
 * Each engine can only handle one crypto operation at one time. It is up to
 * the sw above to ensure single threading of operation on an engine.
 */

// 2 descriptors used per request or more
#define RING_SIZE (QCE_MAX_NUM_DSCR / 2)

struct qcrypto_resp_ctx;

struct qce_device {
        unsigned cpu;
        atomic_t desc_in_count;
        atomic_t bh_wait;

        struct qcrypto_resp_ctx *ring[RING_SIZE];
        u32 ring_head;
        u32 ring_tail;

        u64 total_req;
        u64 busy_req;
        u64 err_req;
        u32 bounce_req;
        u32 done_seq;
        u32 auth_fails;
        u32 same_context;

        struct qcrypto_cipher_ctx *last_context;

        int is_dakota;
	struct device *pdev;        /* Handle to platform_device structure */
	struct bam_registration_info *pbam;

	//unsigned char *coh_vmem;    /* Allocated coherent virtual memory */
	//dma_addr_t coh_pmem;	    /* Allocated coherent physical memory */

	int memsize;				/* Memory allocated */
	uint32_t bam_mem;		/* bam physical address, from DT */
	uint32_t bam_mem_size;		/* bam io size, from DT */

	void __iomem *iobase;	    /* Virtual io base of CE HW  */
	unsigned int phy_iobase;    /* Physical io base of CE HW    */

	struct clk *ce_core_clk;	/* Handle to CE clk */
	struct clk *ce_clk;		/* Handle to CE clk */
	struct clk *ce_bus_clk;	/* Handle to CE AXI clk*/

	uint32_t crypto_cfg_be;
	struct ce_sps_data ce_sps;
	uint32_t engines_avail;
        int init_once;
};

#endif /* _DRIVERS_CRYPTO_MSM_QCE50_H */
