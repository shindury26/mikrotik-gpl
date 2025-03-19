/* Qualcomm Crypto Engine driver.
 *
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
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
#define pr_fmt(fmt) "QCE50: %s: " fmt, __func__

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/device.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/crypto.h>
#include <linux/bitops.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/interrupt.h>
#include <crypto/hash.h>
#include <crypto/sha.h>
#include <crypto/internal/aead.h>

#include "qcrypto.h"
#include "qce.h"
#include "qce50.h"

#define CRYPTO_CONFIG_RESET 0xE001F
#define CE_CLK_100MHZ	100000000
#define CE_CLK_DIV	1000000

// akronite
#define MSM_CLK_CTL_BASE        IOMEM(0xFA010000)
#define REG(off)	        (MSM_CLK_CTL_BASE + (off))
#define CRYPTO_RESET_ENG(n)	REG(0x3E00 + (n * 0x4))
#define CRYPTO_RESET_AHB	REG(0x3E10)

static DEFINE_MUTEX(bam_register_lock);
static LIST_HEAD(qce50_bam_list);
static struct qce_ce_cfg_reg_setting reg;

struct qce_device *engine[4];
unsigned engines = 0;

void _qce_aead_complete(struct sps_event_notify *notify);

/* Standard initialization vector for SHA-1, source: FIPS 180-2 */
static uint32_t  _std_init_vector_sha1[] =   {
	0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0
};

/* Standard initialization vector for SHA-256, source: FIPS 180-2 */
static uint32_t _std_init_vector_sha256[] = {
	0x6A09E667, 0xBB67AE85, 0x3C6EF372, 0xA54FF53A,
	0x510E527F, 0x9B05688C,	0x1F83D9AB, 0x5BE0CD19
};

static int _probe_ce_engine(struct qce_device *pce_dev)
{
	unsigned int rev;
	unsigned int maj_rev, min_rev, step_rev;

	rev = readl_relaxed(pce_dev->iobase + CRYPTO_VERSION_REG);
	mb();
	maj_rev = (rev & CRYPTO_CORE_MAJOR_REV_MASK) >> CRYPTO_CORE_MAJOR_REV;
	min_rev = (rev & CRYPTO_CORE_MINOR_REV_MASK) >> CRYPTO_CORE_MINOR_REV;
	step_rev = (rev & CRYPTO_CORE_STEP_REV_MASK) >> CRYPTO_CORE_STEP_REV;

	if (maj_rev != 0x05) {
		pr_err("Unknown Qualcomm crypto device at 0x%x, rev %d.%d.%d\n",
			pce_dev->phy_iobase, maj_rev, min_rev, step_rev);
		return -EIO;
	};
	pce_dev->engines_avail = readl_relaxed(pce_dev->iobase +
					CRYPTO_ENGINES_AVAIL);
	dev_info(pce_dev->pdev, "Qualcomm Crypto %d.%d.%d device found @0x%x\n",
			maj_rev, min_rev, step_rev, pce_dev->phy_iobase);

	dev_info(pce_dev->pdev,
			"CE device = %d, "
			"IO base, CE = 0x%p, "
			"Consumer (IN) PIPE %d,    "
			"Producer (OUT) PIPE %d\n"
			"IO base BAM = 0x%p, "
			"BAM IRQ %d, "
			"Engines Availability = 0x%x\n",
                        engines,
			pce_dev->iobase,
			pce_dev->ce_sps.dest_pipe_index,
			pce_dev->ce_sps.src_pipe_index,
			pce_dev->ce_sps.bam_iobase,
			pce_dev->ce_sps.bam_irq,
			pce_dev->engines_avail);
	return 0;
};

/**
 * Allocate and Connect a CE peripheral's SPS endpoint
 *
 * This function allocates endpoint context and
 * connect it with memory endpoint by calling
 * appropriate SPS driver APIs.
 *
 * Also registers a SPS callback function with
 * SPS driver
 *
 * This function should only be called once typically
 * during driver probe.
 *
 * @pce_dev - Pointer to qce_device structure
 * @ep   - Pointer to sps endpoint data structure
 * @is_produce - 1 means Producer endpoint
 *		 0 means Consumer endpoint
 *
 * @return - 0 if successful else negative value.
 *
 */
static int qce_sps_init_ep_conn(struct qce_device *pce_dev,
				struct qce_sps_ep_conn_data *ep,
				bool is_producer)
{
	int rc = 0;
	struct sps_pipe *sps_pipe_info;
	struct sps_connect *sps_connect_info = &ep->connect;
	struct sps_register_event *sps_event = &ep->event;

	/* Allocate endpoint context */
	sps_pipe_info = sps_alloc_endpoint();
	if (!sps_pipe_info) {
		pr_err("sps_alloc_endpoint() failed!!! is_producer=%d",
			   is_producer);
		rc = -ENOMEM;
		goto out;
	}
	/* Now save the sps pipe handle */
	ep->pipe = sps_pipe_info;

	/* Get default connection configuration for an endpoint */
	rc = sps_get_config(sps_pipe_info, sps_connect_info);
	if (rc) {
		pr_err("sps_get_config() fail pipe_handle=0x%lx, rc = %d\n",
				(uintptr_t)sps_pipe_info, rc);
		goto get_config_err;
	}

	/* Modify the default connection configuration */
	if (is_producer) {
		/*
		* For CE producer transfer, source should be
		* CE peripheral where as destination should
		* be system memory.
		*/
		sps_connect_info->source = pce_dev->ce_sps.bam_handle;
		sps_connect_info->destination = SPS_DEV_HANDLE_MEM;
		/* Producer pipe will handle this connection */
		sps_connect_info->mode = SPS_MODE_SRC;
		sps_connect_info->options =
			SPS_O_AUTO_ENABLE | SPS_O_DESC_DONE;
	} else {
		/* For CE consumer transfer, source should be
		 * system memory where as destination should
		 * CE peripheral
		 */
		sps_connect_info->source = SPS_DEV_HANDLE_MEM;
		sps_connect_info->destination = pce_dev->ce_sps.bam_handle;
		sps_connect_info->mode = SPS_MODE_DEST;
		sps_connect_info->options =
			SPS_O_AUTO_ENABLE | SPS_O_NO_Q;
	}

	/* Producer pipe index */
	sps_connect_info->src_pipe_index = pce_dev->ce_sps.src_pipe_index;
	/* Consumer pipe index */
	sps_connect_info->dest_pipe_index = pce_dev->ce_sps.dest_pipe_index;
	/* Set pipe group */
	//sps_connect_info->lock_group = pce_dev->ce_sps.pipe_pair_index;
	//sps_connect_info->event_thresh = 0x10;
	/*
	 * Max. no of scatter/gather buffers that can
	 * be passed by block layer = 32 (NR_SG).
	 * Each BAM descritor needs 64 bits (8 bytes).
	 * One BAM descriptor is required per buffer transfer.
	 * So we would require total 256 (32 * 8) bytes of descriptor FIFO.
	 * But due to HW limitation we need to allocate atleast one extra
	 * descriptor memory (256 bytes + 8 bytes). But in order to be
	 * in power of 2, we are allocating 512 bytes of memory.
	 */
	sps_connect_info->desc.size = QCE_MAX_NUM_DSCR *
					sizeof(struct sps_iovec);
	sps_connect_info->desc.base = dma_alloc_coherent(pce_dev->pdev,
					sps_connect_info->desc.size,
					&sps_connect_info->desc.phys_base,
					GFP_KERNEL);
	if (sps_connect_info->desc.base == NULL) {
		rc = -ENOMEM;
		pr_err("Can not allocate coherent memory for sps data\n");
		goto get_config_err;
	}

	memset(sps_connect_info->desc.base, 0x00, sps_connect_info->desc.size);

	/* Establish connection between peripheral and memory endpoint */
	rc = sps_connect(sps_pipe_info, sps_connect_info);
	if (rc) {
		pr_err("sps_connect() fail pipe_handle=0x%lx, rc = %d\n",
				(uintptr_t)sps_pipe_info, rc);
		goto sps_connect_err;
	}

	sps_event->mode = SPS_TRIGGER_CALLBACK;
	if (is_producer)
		sps_event->options = SPS_O_EOT | SPS_O_DESC_DONE;
	else
		sps_event->options = 0;
	sps_event->xfer_done = NULL;
	sps_event->user = NULL;

	pr_debug("success, %s : pipe_handle=0x%lx, desc fifo base (phy) = 0x%p\n",
		is_producer ? "PRODUCER(RX/OUT)" : "CONSUMER(TX/IN)",
		(uintptr_t)sps_pipe_info, &sps_connect_info->desc.phys_base);
	goto out;

sps_connect_err:
	dma_free_coherent(pce_dev->pdev,
			sps_connect_info->desc.size,
			sps_connect_info->desc.base,
			sps_connect_info->desc.phys_base);
get_config_err:
	sps_free_endpoint(sps_pipe_info);
out:
	return rc;
}

/**
 * Disconnect and Deallocate a CE peripheral's SPS endpoint
 *
 * This function disconnect endpoint and deallocates
 * endpoint context.
 *
 * This function should only be called once typically
 * during driver remove.
 *
 * @pce_dev - Pointer to qce_device structure
 * @ep   - Pointer to sps endpoint data structure
 *
 */
static void qce_sps_exit_ep_conn(struct qce_device *pce_dev,
				struct qce_sps_ep_conn_data *ep)
{
	struct sps_pipe *sps_pipe_info = ep->pipe;
	struct sps_connect *sps_connect_info = &ep->connect;

	sps_disconnect(sps_pipe_info);
	dma_free_coherent(pce_dev->pdev,
			sps_connect_info->desc.size,
			sps_connect_info->desc.base,
			sps_connect_info->desc.phys_base);
	sps_free_endpoint(sps_pipe_info);
}

static void qce_sps_release_bam(struct qce_device *pce_dev)
{
	struct bam_registration_info *pbam;

	mutex_lock(&bam_register_lock);
	pbam = pce_dev->pbam;
	if (pbam == NULL)
		goto ret;

	pbam->cnt--;
	if (pbam->cnt > 0)
		goto ret;

	if (pce_dev->ce_sps.bam_handle) {
		sps_deregister_bam_device(pce_dev->ce_sps.bam_handle);

		pr_debug("deregister bam handle 0x%lx\n",
					pce_dev->ce_sps.bam_handle);
		pce_dev->ce_sps.bam_handle = 0;
	}
	iounmap(pbam->bam_iobase);
	pr_debug("delete bam 0x%x\n", pbam->bam_mem);
	list_del(&pbam->qlist);
	kfree(pbam);

ret:
	pce_dev->pbam = NULL;
	mutex_unlock(&bam_register_lock);
}

static int qce_sps_get_bam(struct qce_device *pce_dev)
{
	int rc = 0;
	struct sps_bam_props bam = {0};
	struct bam_registration_info *pbam = NULL;
	struct bam_registration_info *p;

	mutex_lock(&bam_register_lock);

	list_for_each_entry(p, &qce50_bam_list, qlist) {
		if (p->bam_mem == pce_dev->bam_mem) {
			pbam = p;  /* found */
			break;
		}
	}

	if (pbam) {
		pr_debug("found bam 0x%x\n", pbam->bam_mem);
		pbam->cnt++;
		pce_dev->ce_sps.bam_handle =  pbam->handle;
		pce_dev->ce_sps.bam_mem = pbam->bam_mem;
		pce_dev->ce_sps.bam_iobase = pbam->bam_iobase;
		pce_dev->pbam = pbam;
		goto ret;
	}

	pbam = kzalloc(sizeof(struct  bam_registration_info), GFP_KERNEL);
	if (!pbam) {
		pr_err("qce50 Memory allocation of bam FAIL, error %ld\n",
						PTR_ERR(pbam));

		rc = -ENOMEM;
		goto ret;
	}
	pbam->cnt = 1;
	pbam->bam_mem = pce_dev->bam_mem;
	pbam->bam_iobase = ioremap(pce_dev->bam_mem, pce_dev->bam_mem_size);
	if (!pbam->bam_iobase) {
		kfree(pbam);
		rc = -ENOMEM;
		pr_err("Can not map BAM io memory\n");
		goto ret;
	}
	pce_dev->ce_sps.bam_mem = pbam->bam_mem;
	pce_dev->ce_sps.bam_iobase = pbam->bam_iobase;
	pbam->handle = 0;
	pr_debug("allocate bam 0x%x\n", pbam->bam_mem);
	bam.phys_addr = pce_dev->ce_sps.bam_mem;
	bam.virt_addr = pce_dev->ce_sps.bam_iobase;

	/*
	 * This threshold controls when the BAM publish
	 * the descriptor size on the sideband interface.
	 * SPS HW will only be used when
	 * data transfer size >  64 bytes.
	 */
	bam.summing_threshold = 64;
	/* SPS driver wll handle the crypto BAM IRQ */
	bam.irq = (u32)pce_dev->ce_sps.bam_irq;

        bam.manage = SPS_BAM_MGR_LOCAL;

	bam.ee = pce_dev->is_dakota ? 1 : 0;

	pr_debug("bam physical base=0x%lx\n", (uintptr_t)bam.phys_addr);
	pr_debug("bam virtual base=0x%p\n", bam.virt_addr);

	/* Register CE Peripheral BAM device to SPS driver */
	rc = sps_register_bam_device(&bam, &pbam->handle);
	if (rc) {
		pr_err("sps_register_bam_device() failed! err=%d", rc);
		rc = -EIO;
		iounmap(pbam->bam_iobase);
		kfree(pbam);
		goto ret;
	}

	pce_dev->pbam = pbam;
	list_add_tail(&pbam->qlist, &qce50_bam_list);
	pce_dev->ce_sps.bam_handle =  pbam->handle;

ret:
	mutex_unlock(&bam_register_lock);

	return rc;
}
/**
 * Initialize SPS HW connected with CE core
 *
 * This function register BAM HW resources with
 * SPS driver and then initialize 2 SPS endpoints
 *
 * This function should only be called once typically
 * during driver probe.
 *
 * @pce_dev - Pointer to qce_device structure
 *
 * @return - 0 if successful else negative value.
 *
 */
static int qce_sps_init(struct qce_device *pce_dev)
{
	int rc = 0;

	rc = qce_sps_get_bam(pce_dev);
	if (rc)
		return rc;
	pr_debug("BAM device registered. bam_handle=0x%lx\n",
		pce_dev->ce_sps.bam_handle);

	rc = qce_sps_init_ep_conn(pce_dev, &pce_dev->ce_sps.producer, true);
	if (rc)
		goto sps_connect_producer_err;
	rc = qce_sps_init_ep_conn(pce_dev, &pce_dev->ce_sps.consumer, false);
	if (rc)
		goto sps_connect_consumer_err;

	pr_info(" Qualcomm MSM CE-BAM at 0x%016llx irq %d\n",
		(unsigned long long)pce_dev->ce_sps.bam_mem,
		(unsigned int)pce_dev->ce_sps.bam_irq);
	return rc;

sps_connect_consumer_err:
	qce_sps_exit_ep_conn(pce_dev, &pce_dev->ce_sps.producer);
sps_connect_producer_err:
	qce_sps_release_bam(pce_dev);
	return rc;
}

/**
 * De-initialize SPS HW connected with CE core
 *
 * This function deinitialize SPS endpoints and then
 * deregisters BAM resources from SPS driver.
 *
 * This function should only be called once typically
 * during driver remove.
 *
 * @pce_dev - Pointer to qce_device structure
 *
 */
static void qce_sps_exit(struct qce_device *pce_dev)
{
	qce_sps_exit_ep_conn(pce_dev, &pce_dev->ce_sps.consumer);
	qce_sps_exit_ep_conn(pce_dev, &pce_dev->ce_sps.producer);
	qce_sps_release_bam(pce_dev);
}

static void qce_init_ce_cfg_val(void)
{
        static bool cfg_init = false;
        if (cfg_init) return;
        cfg_init = true;

	/* Initialize encr_cfg register for AES alg */
	reg.encr_cfg_aes_cbc_128 =
		(CRYPTO_ENCR_KEY_SZ_AES128 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CBC << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_cbc_256 =
		(CRYPTO_ENCR_KEY_SZ_AES256 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CBC << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_ctr_128 =
		(CRYPTO_ENCR_KEY_SZ_AES128 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CTR << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_ctr_256 =
		(CRYPTO_ENCR_KEY_SZ_AES256 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CTR << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_xts_128 =
		(CRYPTO_ENCR_KEY_SZ_AES128 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_XTS << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_xts_256 =
		(CRYPTO_ENCR_KEY_SZ_AES256 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_XTS << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_ecb_128 =
		(CRYPTO_ENCR_KEY_SZ_AES128 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_ECB << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_ecb_256 =
		(CRYPTO_ENCR_KEY_SZ_AES256 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_ECB << CRYPTO_ENCR_MODE);

	reg.encr_cfg_aes_ccm_128 =
		(CRYPTO_ENCR_KEY_SZ_AES128 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CCM << CRYPTO_ENCR_MODE)|
		(CRYPTO_LAST_CCM_XFR << CRYPTO_LAST_CCM);

	reg.encr_cfg_aes_ccm_256 =
		(CRYPTO_ENCR_KEY_SZ_AES256 << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_AES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CCM << CRYPTO_ENCR_MODE) |
		(CRYPTO_LAST_CCM_XFR << CRYPTO_LAST_CCM);

	/* Initialize encr_cfg register for DES alg */
	reg.encr_cfg_des_ecb =
		(CRYPTO_ENCR_KEY_SZ_DES << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_DES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_ECB << CRYPTO_ENCR_MODE);

	reg.encr_cfg_des_cbc =
		(CRYPTO_ENCR_KEY_SZ_DES << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_DES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CBC << CRYPTO_ENCR_MODE);

	reg.encr_cfg_3des_ecb =
		(CRYPTO_ENCR_KEY_SZ_3DES << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_DES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_ECB << CRYPTO_ENCR_MODE);

	reg.encr_cfg_3des_cbc =
		(CRYPTO_ENCR_KEY_SZ_3DES << CRYPTO_ENCR_KEY_SZ) |
		(CRYPTO_ENCR_ALG_DES << CRYPTO_ENCR_ALG) |
		(CRYPTO_ENCR_MODE_CBC << CRYPTO_ENCR_MODE);

	/* Initialize auth_cfg register for CMAC alg */
	reg.auth_cfg_cmac_128 =
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST) |
		(CRYPTO_AUTH_MODE_CMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_ENUM_16_BYTES << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_AES << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_KEY_SZ_AES128 << CRYPTO_AUTH_KEY_SIZE);

	reg.auth_cfg_cmac_256 =
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST) |
		(CRYPTO_AUTH_MODE_CMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_ENUM_16_BYTES << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_AES << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_KEY_SZ_AES256 << CRYPTO_AUTH_KEY_SIZE);

	/* Initialize auth_cfg register for HMAC alg */
	reg.auth_cfg_hmac_sha1 =
		(CRYPTO_AUTH_MODE_HMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA1 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_POS_BEFORE << CRYPTO_AUTH_POS);

	reg.auth_cfg_hmac_sha256 =
		(CRYPTO_AUTH_MODE_HMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA256 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_POS_BEFORE << CRYPTO_AUTH_POS);

	/* Initialize auth_cfg register for SHA1/256 alg */
	reg.auth_cfg_sha1 =
		(CRYPTO_AUTH_MODE_HASH << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA1 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_POS_BEFORE << CRYPTO_AUTH_POS);

	reg.auth_cfg_sha256 =
		(CRYPTO_AUTH_MODE_HASH << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA256 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_POS_BEFORE << CRYPTO_AUTH_POS);

	/* Initialize auth_cfg register for AEAD alg */
	reg.auth_cfg_aead_sha1_hmac =
		(CRYPTO_AUTH_MODE_HMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA1 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST);

	reg.auth_cfg_aead_sha256_hmac =
		(CRYPTO_AUTH_MODE_HMAC << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_SIZE_SHA256 << CRYPTO_AUTH_SIZE) |
		(CRYPTO_AUTH_ALG_SHA << CRYPTO_AUTH_ALG) |
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST);

	reg.auth_cfg_aes_ccm_128 =
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST) |
		(CRYPTO_AUTH_MODE_CCM << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_ALG_AES << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_KEY_SZ_AES128 << CRYPTO_AUTH_KEY_SIZE) |
		((MAX_NONCE/sizeof(uint32_t)) << CRYPTO_AUTH_NONCE_NUM_WORDS);
	reg.auth_cfg_aes_ccm_128 &= ~(1 << CRYPTO_USE_HW_KEY_AUTH);

	reg.auth_cfg_aes_ccm_256 =
		(1 << CRYPTO_LAST) | (1 << CRYPTO_FIRST) |
		(CRYPTO_AUTH_MODE_CCM << CRYPTO_AUTH_MODE)|
		(CRYPTO_AUTH_ALG_AES << CRYPTO_AUTH_ALG) |
		(CRYPTO_AUTH_KEY_SZ_AES256 << CRYPTO_AUTH_KEY_SIZE) |
		((MAX_NONCE/sizeof(uint32_t)) << CRYPTO_AUTH_NONCE_NUM_WORDS);
	reg.auth_cfg_aes_ccm_256 &= ~(1 << CRYPTO_USE_HW_KEY_AUTH);
}

inline void qce_add_reg(unsigned int iobase,
                        struct sps_command_element **cmd_ptr,
                        const u32 reg, const u32 data) {
    (*cmd_ptr)->addr = (uint32_t)(reg + iobase);
    (*cmd_ptr)->data = data;
    (*cmd_ptr)->mask = 0xFFFFFFFF;
    (*cmd_ptr)->command = 0;
    (*cmd_ptr)->reserved = 0;
    (*cmd_ptr)++;
}

// XXX cbc only
int _setup_aead_const(struct qce_device *pce_dev,
                    struct sps_command_element **vaddr,
                    const struct qcrypto_cipher_ctx *cipher_ctx,
                    const uint32_t crypt_offset)
{
    const unsigned int iobase = pce_dev->phy_iobase;
    uintptr_t ce_vaddr_start = (uintptr_t)(*vaddr);
    uint32_t key_reg;
    uint32_t i;
    struct sps_command_element *ce_vaddr = (struct sps_command_element *)(*vaddr);

    if (unlikely(!pce_dev->init_once)) {
        pce_dev->init_once = 1;
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_CONFIG_REG, pce_dev->crypto_cfg_be);
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_AUTH_SEG_START_REG, 0);
        //qce_add_reg(&ce_vaddr, CRYPTO_AUTH_BYTECNT0_REG, 0);
        //qce_add_reg(&ce_vaddr, CRYPTO_AUTH_BYTECNT1_REG, 0);

        if (cipher_ctx->mode == QCE_MODE_CTR) {
            qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR_MASK_REG0, 0);
            qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR_MASK_REG1, 0);
            qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR_MASK_REG2, 0);
            qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR_MASK_REG, 0xffffffff);
        }
    }

    qce_add_reg(iobase, &ce_vaddr, CRYPTO_ENCR_SEG_START_REG, crypt_offset & 0xffff);

    // auth key
    key_reg = cipher_ctx->auth_key_len / sizeof(uint32_t);
    for (i = 0; i < key_reg; i++)
        qce_add_reg(iobase, &ce_vaddr, (CRYPTO_AUTH_KEY0_REG + i * sizeof(uint32_t)),
                ((uint32_t*) cipher_ctx->auth_key)[i]);

    // clear bigger key remains
    for (; i < 8; i++)
        qce_add_reg(iobase, &ce_vaddr, (CRYPTO_AUTH_KEY0_REG + i * sizeof(uint32_t)), 0);

    // enc key
    key_reg = cipher_ctx->enc_key_len / sizeof(uint32_t);
    for (i = 0; i < key_reg; i++)
        qce_add_reg(iobase, &ce_vaddr, (CRYPTO_ENCR_KEY0_REG + i * sizeof(uint32_t)),
                ((uint32_t*) cipher_ctx->enc_key)[i]);

    *vaddr = ce_vaddr;
    return (char*) ce_vaddr - (char*) ce_vaddr_start;
}

int _setup_aead_dynamic(struct sps_command_element **vaddr,
                            struct crypto_async_request *async_req,
                            const struct qcrypto_cipher_ctx *cipher_ctx,
                            const unsigned char *iv,
                            const uint32_t cryptlen,
                            const uint32_t totallen_in)
{
    uintptr_t ce_vaddr_start = (uintptr_t)(*vaddr);
    struct aead_request *areq = container_of(async_req, struct aead_request, base);
    struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(areq);
    unsigned int iobase = rctx->pce->phy_iobase;
    uint32_t enciv_in_word;
    uint32_t a_cfg;
    uint32_t encr_cfg;
    uint32_t iv_reg;
    uint32_t *iv_init;
    const char cbc = cipher_ctx->mode == QCE_MODE_CBC;
    int i;

    struct sps_command_element *ce_vaddr = (struct sps_command_element *)(*vaddr);

    qce_add_reg(iobase, &ce_vaddr, CRYPTO_STATUS_REG, 0);
    qce_add_reg(iobase, &ce_vaddr, CRYPTO_SEG_SIZE_REG, totallen_in);
    qce_add_reg(iobase, &ce_vaddr, CRYPTO_AUTH_SEG_SIZE_REG, totallen_in);
    qce_add_reg(iobase, &ce_vaddr, CRYPTO_ENCR_SEG_SIZE_REG, cryptlen);

    if (cipher_ctx->auth_alg == QCE_HASH_SHA1_HMAC) {
        a_cfg = reg.auth_cfg_aead_sha1_hmac;
    } else {
        a_cfg = reg.auth_cfg_aead_sha256_hmac;
    }
    a_cfg &= ~(CRYPTO_AUTH_POS_MASK);
    if (rctx->dir == QCE_ENCRYPT) a_cfg |= (CRYPTO_AUTH_POS_AFTER << CRYPTO_AUTH_POS);
    else a_cfg |= (CRYPTO_AUTH_POS_BEFORE << CRYPTO_AUTH_POS);
    qce_add_reg(iobase, &ce_vaddr, CRYPTO_AUTH_SEG_CFG_REG, a_cfg);

    switch (cipher_ctx->cipher_alg) {
        case CIPHER_ALG_AES:
            if (cipher_ctx->enc_key_len ==  AES128_KEY_SIZE) {
                encr_cfg = cbc ? reg.encr_cfg_aes_cbc_128
                                : reg.encr_cfg_aes_ctr_128;
            } else if (cipher_ctx->enc_key_len ==  AES256_KEY_SIZE) {
                encr_cfg = cbc ? reg.encr_cfg_aes_cbc_256
                                : reg.encr_cfg_aes_ctr_256;
            } else {
                return -EINVAL;
            }
            enciv_in_word = 4;
            break;

        case CIPHER_ALG_DES:
            encr_cfg = reg.encr_cfg_des_cbc;
            enciv_in_word = 2;
            break;

        case CIPHER_ALG_3DES:
            encr_cfg = reg.encr_cfg_3des_cbc;
            enciv_in_word = 2;
            break;

        default:
            return -EINVAL;
    };

    qce_add_reg(iobase, &ce_vaddr, CRYPTO_ENCR_SEG_CFG_REG,
            encr_cfg | ((rctx->dir == QCE_ENCRYPT) ? (1 << CRYPTO_ENCODE) : 0));

    // enc iv
    if (cbc) {
        for (i = 0; i < enciv_in_word; i++)
            qce_add_reg(iobase, &ce_vaddr, (CRYPTO_CNTR0_IV0_REG + i * sizeof(uint32_t)), ((uint32_t*) iv)[i]);
    } else {
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR0_IV0_REG, cipher_ctx->nonce);
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR1_IV1_REG, *(uint32_t*) &iv[0]);
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR2_IV2_REG, *(uint32_t*) &iv[4]);
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_CNTR3_IV3_REG, 0x01000000);
    }

    // auth iv
    if (cipher_ctx->auth_alg == QCE_HASH_SHA1_HMAC) {
        iv_reg = 5;
        iv_init = _std_init_vector_sha1;
    } else {
        iv_reg = 8;
        iv_init = _std_init_vector_sha256;
    }
    for (i = 0; i < iv_reg; i++)
        qce_add_reg(iobase, &ce_vaddr, CRYPTO_AUTH_IV0_REG + i * sizeof(uint32_t), be32_to_cpu(iv_init[i]));

    qce_add_reg(iobase, &ce_vaddr, CRYPTO_GOPROC_REG,
                (1 << CRYPTO_GO)
                //| (1 << CRYPTO_CLR_CNTXT)
                | (1 << CRYPTO_RESULTS_DUMP));

    *vaddr = ce_vaddr;
    return (char*) ce_vaddr - (char*) ce_vaddr_start;
}

inline void _qce_set_flag(struct sps_transfer *sps_bam_pipe, uint32_t flag) {
    struct sps_iovec *iovec = sps_bam_pipe->iovec + sps_bam_pipe->iovec_count - 1;
    iovec->flags |= flag;
}


inline int _qce_sps_add_data(uint32_t addr, uint32_t len,
                            struct sps_transfer *sps_bam_pipe, int ivcount)
{
	uint32_t data_cnt;
        struct sps_iovec *iovec = sps_bam_pipe->iovec +
                                        sps_bam_pipe->iovec_count;

	while (len > 0) {
                if (sps_bam_pipe->iovec_count == ivcount) {
                    return 1;
                }

		if (len > SPS_MAX_PKT_SIZE)
			data_cnt = SPS_MAX_PKT_SIZE;
		else
			data_cnt = len;
		iovec->size = data_cnt;
		iovec->addr = addr;
		iovec->flags = 0;
                iovec++;
		sps_bam_pipe->iovec_count++;
		addr += data_cnt;
		len -= data_cnt;
	}
	return 0;
}

static int _qce_sps_add_sg_data(struct scatterlist *sg_src, uint32_t nbytes,
                                struct sps_transfer *sps_bam_pipe, int ivcount)
{
	uint32_t addr, data_cnt, len;
        struct sps_iovec *iovec = sps_bam_pipe->iovec + sps_bam_pipe->iovec_count;
	while ((nbytes > 0) && (sg_src != NULL)) {
		len = min(nbytes, sg_dma_len(sg_src));
		nbytes -= len;
		addr = sg_dma_address(sg_src);
		while (len > 0) {
                        if (unlikely(sps_bam_pipe->iovec_count == ivcount)) {
                            return 1;
                        }
			if (len > SPS_MAX_PKT_SIZE) {
				data_cnt = SPS_MAX_PKT_SIZE;
				iovec->size = data_cnt;
				iovec->addr = addr;
				iovec->flags = 0;
			} else {
				data_cnt = len;
				iovec->size = data_cnt;
				iovec->addr = addr;
				iovec->flags = 0;
			}
                        iovec++;
			sps_bam_pipe->iovec_count++;
			addr += data_cnt;
			len -= data_cnt;
		}
		sg_src = sg_next(sg_src);
	}
	return 0;
}

inline void _qce_sps_add_cmd(uint32_t flag, uint32_t cmd, int size,
                            struct sps_transfer *sps_bam_pipe)
{
        struct sps_iovec *iovec = sps_bam_pipe->iovec + sps_bam_pipe->iovec_count;
	iovec->size = size;
	iovec->addr = cmd;
	iovec->flags = SPS_IOVEC_FLAG_CMD | flag;
	sps_bam_pipe->iovec_count++;
}

int qce_aead_req(void *handle, struct crypto_async_request *async_req,
                                struct qcrypto_cipher_req_ctx *cipher_rctx,
                                int cryptlen)
{
	struct qce_device *pce_dev = (struct qce_device *) handle;
	struct aead_request *areq = container_of(async_req,
				struct aead_request, base);
	struct crypto_aead *aead = crypto_aead_reqtfm(areq);
	struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(areq);
	const int ivsize = crypto_aead_ivsize(aead);
        const int aadsize = areq->assoclen + ivsize;
        const int ivcount = 1 + rctx->src_nents;
        struct sps_iovec iv[ivcount];
	struct sps_transfer in_transfer;
	struct sps_transfer out_transfer;
	int rc = 0;

        // XXX the bottleneck is in descriptor transfer and because of that
        // transmitting cmd in two parts: static and dynamic, will decrease performance

        // TODO other optimization can be done by using pipe keys for only two contexts
        // those keys are kept in CE memory and will allow faster CE setup
        // to use them, highspeed mode must be enabled and odd/even bit set

	in_transfer.iovec_count = 0;
        in_transfer.iovec_phys = 1;
	in_transfer.iovec = iv;
	in_transfer.user = NULL;

        _qce_sps_add_cmd(0, (uint32_t) rctx->phy_cmd_in + rctx->cmd_size_const,
                        rctx->cmd_size - rctx->cmd_size_const, &in_transfer);
        if (!rctx->bounce
                ? _qce_sps_add_sg_data(areq->src, cryptlen + aadsize, &in_transfer, ivcount)
                : _qce_sps_add_data(rctx->phy_bounce, cryptlen + aadsize, &in_transfer, ivcount)) {
            return -ENXIO;
        }
        _qce_set_flag(&in_transfer, SPS_IOVEC_FLAG_EOT | SPS_IOVEC_FLAG_NWD);

        rc = sps_transfer_fast(pce_dev->ce_sps.consumer.pipe, &in_transfer);
        if (rc) {
                pr_err("sps_xfr() fail (consumer pipe=0x%lx) rc = %d\n",
                        (uintptr_t)pce_dev->ce_sps.consumer.pipe, rc);
                return rc;
        }

	out_transfer.iovec_count = 0;
        out_transfer.iovec_phys = 1;
	out_transfer.iovec = iv;
	out_transfer.user = pce_dev->ce_sps.producer.pipe;

        if (!rctx->bounce
                ? _qce_sps_add_sg_data(areq->src, cryptlen + aadsize, &out_transfer, ivcount)
                : _qce_sps_add_data(rctx->phy_bounce, cryptlen + aadsize, &out_transfer, ivcount)) {
            return -ENXIO;
        }
        if (_qce_sps_add_data(rctx->phy_result_in, CRYPTO_RESULT_DUMP_SIZE, &out_transfer, ivcount))
                return -ENXIO;
        _qce_set_flag(&out_transfer, SPS_IOVEC_FLAG_INT);
	rc = sps_transfer_fast(pce_dev->ce_sps.producer.pipe, &out_transfer);
	if (rc) {
		pr_err("sps_xfr() fail (producer pipe=0x%lx) rc = %d\n",
				(uintptr_t)pce_dev->ce_sps.producer.pipe, rc);
                return rc;
	}
	return 0;
}

static int __qce_get_device_tree_data(struct platform_device *pdev,
		struct qce_device *pce_dev)
{
	struct resource *resource;
	int rc = 0;

        if (!pdev->dev.of_node) return -EINVAL;

        pce_dev->is_dakota = of_machine_is_compatible("rb3011") == 0;

	if (pce_dev->is_dakota) {
            pce_dev->ce_sps.pipe_pair_index = 1;

        } else { // akronite
            int i;
            const char *c = strstr(pdev->name, "qcrypto");
            if (!c) return -EINVAL;

            int id = c[7]; // qcrypto[N]
            if (id != 0) id -= '1';

            if (!id) {
                for (i = 0; i < 4; ++i) {
                    const char ce[] = { 'c', 'e', '1' + i, 0 };
                    struct reset_control* eng = devm_reset_control_get_exclusive(&pdev->dev, ce);
                    if (IS_ERR(eng)) {
                        printk("reset not found: %s\n", ce);
                        return PTR_ERR(eng);
                    }

                    reset_control_assert(eng);
                    rc = reset_control_reset(eng);
                    if (rc) {
                        printk("reset failed for %s: %x \n", ce, rc);
                        return rc;
                    }
                    reset_control_deassert(eng);
                }

                struct reset_control* ahb = devm_reset_control_get_exclusive(&pdev->dev, "ahb");
                if (IS_ERR(ahb)) {
                    printk("reset not found: ahb\n");
                    return PTR_ERR(ahb);
                }

                reset_control_assert(ahb);
                rc = reset_control_reset(ahb);
                if (rc) {
                    printk("reset failed for ahb: %x \n", rc);
                    return rc;
                }
                reset_control_deassert(ahb);
            }

            pce_dev->ce_sps.pipe_pair_index = 0;
            pce_dev->ce_sps.ce_device = id;
        }
	pce_dev->ce_sps.dest_pipe_index	= 2 * pce_dev->ce_sps.pipe_pair_index;
	pce_dev->ce_sps.src_pipe_index	= pce_dev->ce_sps.dest_pipe_index + 1;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"crypto-base");
	if (resource) {
                int rs = resource_size(resource);
                if (!pce_dev->is_dakota) {
                    // ipq806x BAM and CE spaces overlap
                    resource->start -= CRYPTO_BASE;
                    rs = 0x20000;
                }
		pce_dev->phy_iobase = resource->start;
		pce_dev->iobase = ioremap(resource->start, rs);
		if (!pce_dev->iobase) {
			pr_err("Can not map CRYPTO io memory\n");
			return -ENOMEM;
		}
	} else {
		pr_err("CRYPTO HW mem unavailable.\n");
		return -ENODEV;
	}

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							"crypto-bam-base");
	if (resource) {
		pce_dev->bam_mem = resource->start;
		pce_dev->bam_mem_size = resource_size(resource);
	} else {
		pr_err("CRYPTO BAM mem unavailable.\n");
		rc = -ENODEV;
		goto err_getting_bam_info;
	}

	resource  = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (resource) {
		pce_dev->ce_sps.bam_irq = resource->start;
	} else {
		pr_err("CRYPTO BAM IRQ unavailable.\n");
		goto err_dev;
	}
	return rc;
err_dev:
	if (pce_dev->ce_sps.bam_iobase)
		iounmap(pce_dev->ce_sps.bam_iobase);

err_getting_bam_info:
	if (pce_dev->iobase)
		iounmap(pce_dev->iobase);

	return rc;
}

static int __qce_init_clk(struct qce_device *pce_dev)
{
	int rc = 0;

	if (!pce_dev->is_dakota && pce_dev->ce_sps.ce_device) return 0;

		pce_dev->ce_core_clk = of_clk_get_by_name(pce_dev->pdev->of_node, "core_clk");
		if (IS_ERR(pce_dev->ce_core_clk)) {
			rc = PTR_ERR(pce_dev->ce_core_clk);
			pr_err("Unable to get CE core clk\n");
			goto exit_put_core_src_clk;
		}
		pce_dev->ce_clk = of_clk_get_by_name(pce_dev->pdev->of_node, "iface_clk");
		if (IS_ERR(pce_dev->ce_clk)) {
			rc = PTR_ERR(pce_dev->ce_clk);
			pr_err("Unable to get CE interface clk\n");
			goto exit_put_core_clk;
		}

		pce_dev->ce_bus_clk = of_clk_get_by_name(pce_dev->pdev->of_node, "bus_clk");
		if (IS_ERR(pce_dev->ce_bus_clk)) {
			rc = PTR_ERR(pce_dev->ce_bus_clk);
			pr_err("Unable to get CE BUS interface clk\n");
			goto exit_put_iface_clk;
		}
	return rc;

exit_put_iface_clk:
	clk_put(pce_dev->ce_clk);
exit_put_core_clk:
	if (pce_dev->ce_core_clk)
            clk_put(pce_dev->ce_core_clk);
exit_put_core_src_clk:
	pr_err("Unable to init CE clks, rc = %d\n", rc);
	return rc;
}

static void __qce_deinit_clk(struct qce_device *pce_dev)
{
	if (pce_dev->ce_bus_clk)
		clk_put(pce_dev->ce_bus_clk);
	if (pce_dev->ce_clk)
		clk_put(pce_dev->ce_clk);
	if (pce_dev->ce_core_clk)
		clk_put(pce_dev->ce_core_clk);
}

static void boost_rate(struct clk *clk, const char *name) {
    //int r = clk_get_rate(clk);
    clk_set_rate(clk, INT_MAX);
    //r = clk_get_rate(clk);
}

int qce_enable_clk(void *handle)
{
	struct qce_device *pce_dev = (struct qce_device *)handle;
	int rc = 0;

	if (!pce_dev->is_dakota && pce_dev->ce_sps.ce_device) return 0;

        if (pce_dev->ce_core_clk) {
                boost_rate(pce_dev->ce_core_clk, "core");
                rc = clk_prepare_enable(pce_dev->ce_core_clk);
                if (rc) {
                        pr_err("Unable to enable/prepare CE core clk\n");
                        return rc;
                }
        }

	if (pce_dev->ce_clk) {
                boost_rate(pce_dev->ce_clk, "iface");
		rc = clk_prepare_enable(pce_dev->ce_clk);
		if (rc) {
			pr_err("Unable to enable/prepare CE iface clk\n");
			goto exit_disable_core_clk;
		}
	}

	if (pce_dev->ce_bus_clk) {
                boost_rate(pce_dev->ce_bus_clk, "bus");
		rc = clk_prepare_enable(pce_dev->ce_bus_clk);
		if (rc) {
			pr_err("Unable to enable/prepare CE BUS clk\n");
			goto exit_disable_ce_clk;
		}
	}
	return rc;

exit_disable_ce_clk:
	clk_disable_unprepare(pce_dev->ce_clk);
exit_disable_core_clk:
		clk_disable_unprepare(pce_dev->ce_core_clk);
	return rc;
}
EXPORT_SYMBOL(qce_enable_clk);

int qce_disable_clk(void *handle)
{
	struct qce_device *pce_dev = (struct qce_device *) handle;
	int rc = 0;

	if (pce_dev->ce_bus_clk)
		clk_disable_unprepare(pce_dev->ce_bus_clk);
	if (pce_dev->ce_clk)
		clk_disable_unprepare(pce_dev->ce_clk);
        if (pce_dev->ce_core_clk)
                clk_disable_unprepare(pce_dev->ce_core_clk);

	return rc;
}
EXPORT_SYMBOL(qce_disable_clk);

/* crypto engine open function. */
struct qce_device* qce_open(struct platform_device *pdev, int *rc)
{
	struct qce_device *pce_dev = kzalloc(sizeof(struct qce_device), GFP_KERNEL);
	if (!pce_dev) {
		*rc = -ENOMEM;
		pr_err("Can not allocate memory: %d\n", *rc);
		return NULL;
	}

	pce_dev->pdev = &pdev->dev;
        pce_dev->cpu = -1;
        atomic_set(&pce_dev->desc_in_count, 0);
        atomic_set(&pce_dev->bh_wait, 0);

        *rc = __qce_get_device_tree_data(pdev, pce_dev);
        if (*rc)
            goto err_pce_dev;

	*rc = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (*rc)
		goto err_mem;

	*rc = __qce_init_clk(pce_dev);
	if (*rc)
		goto err_mem;

	*rc = qce_enable_clk(pce_dev);
	if (*rc)
		goto err_enable_clk;

	if (_probe_ce_engine(pce_dev)) {
		*rc = -ENXIO;
		goto err;
	}
	*rc = 0;

	qce_init_ce_cfg_val();

	const uint32_t beats = (MAX_CE_BAM_BURST_SIZE >> 3) - 1;
	pce_dev->crypto_cfg_be = (beats << CRYPTO_REQ_SIZE)
		| BIT(CRYPTO_MASK_DOUT_INTR) | BIT(CRYPTO_MASK_DIN_INTR)
		| BIT(CRYPTO_MASK_OP_DONE_INTR) | (0 << CRYPTO_HIGH_SPD_EN_N)
		| (pce_dev->ce_sps.pipe_pair_index << CRYPTO_PIPE_SET_SELECT)
                | (1 << CRYPTO_LITTLE_ENDIAN_MODE);

	*rc  = qce_sps_init(pce_dev);
	if (*rc)
		goto err;

        pce_dev->ce_sps.producer.event.callback = _qce_aead_complete;
        pce_dev->ce_sps.producer.event.options = SPS_O_DESC_DONE;
        pce_dev->ce_sps.producer.event.user = pce_dev;
        *rc = sps_register_event(pce_dev->ce_sps.producer.pipe,
                                    &pce_dev->ce_sps.producer.event);
        if (*rc) {
            pr_err("Producer callback registration failed rc = %d\n", *rc);
            goto err;
        }

        engine[engines++] = pce_dev;

	return pce_dev;
err:
	qce_disable_clk(pce_dev);

err_enable_clk:
	__qce_deinit_clk(pce_dev);

err_mem:
err_pce_dev:
	kfree(pce_dev);
	return NULL;
}
EXPORT_SYMBOL(qce_open);

/* crypto engine close function. */
int qce_close(void *handle)
{
	struct qce_device *pce_dev = (struct qce_device *) handle;

	if (handle == NULL)
		return -ENODEV;

	qce_enable_clk(pce_dev);
	qce_sps_exit(pce_dev);

	if (pce_dev->iobase)
		iounmap(pce_dev->iobase);

	qce_disable_clk(pce_dev);
	__qce_deinit_clk(pce_dev);

	kfree(handle);

	return 0;
}
EXPORT_SYMBOL(qce_close);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Crypto Engine driver");
