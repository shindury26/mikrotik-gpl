/* Qualcomm Crypto driver
 *
 * Copyright (c) 2010-2014, The Linux Foundation. All rights reserved.
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

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/crypto.h>
#include <linux/kernel.h>
#include <linux/rtnetlink.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/debugfs.h>
#include <linux/workqueue.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/cache.h>
#include <linux/smp.h>
#include <linux/proc_fs.h>

#include <crypto/ctr.h>
#include <crypto/des.h>
#include <crypto/aes.h>
#include <crypto/sha.h>
#include <crypto/hash.h>
#include <crypto/algapi.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/aead.h>

#include <linux/msm-bus.h>
#include "qcrypto.h"
#include "qcryptohw_50.h"

#define DEBUG_MAX_FNAME  16
#define DEBUG_MAX_RW_BUF 2048

extern struct qce_device *engine[];
extern unsigned engines;

/*
 * For crypto 5.0 which has burst size alignment requirement.
 */
#define MAX_ALIGN_SIZE  0x40
static const char PROCNAME[] = "cryptohw";
static struct proc_dir_entry *entry = NULL;

#define RNG_MAX PAGE_SIZE
static unsigned char rng_salt[32];
static unsigned char *rng_pool = NULL;
static atomic_t rng_used;
static unsigned rng_salted[NR_CPUS] = { 0, 0, 0, 0 };

static int hard_irq_cpu = -1;
static spinlock_t lock;

#define OUT_RING_SIZE (RING_SIZE * 2 * 16)
static struct comp_queue_t {
    struct tasklet_struct tasklet;
    struct qcrypto_resp_ctx *out_ring[OUT_RING_SIZE];
    int head;
    int tail;
    atomic_t wait;
} comp_queue[NR_CPUS];

static void _qcrypto_register_timer(struct work_struct *work);
DECLARE_DELAYED_WORK(reg_work, _qcrypto_register_timer);

static inline bool is_sw_fallback(struct qcrypto_cipher_ctx *ctx) {
    if (!ctx->fallback) return false;
    if (ctx->enc_key_len == AES_KEYSIZE_192) return true;
    return false;
}

static inline int rng_get(unsigned char *rng, int size) {
    unsigned used = atomic_add_return(size, &rng_used) - size;
    unsigned ofs = used % RNG_MAX;
    if (ofs + size > RNG_MAX) ofs = 0;

    memcpy(rng, &rng_pool[ofs], size);
    crypto_xor(rng, rng_salt, size);
    return size;
}

static void sched_cpu_tasklet(void *data) {
    tasklet_hi_schedule(data);
}

inline bool is_dakota(void) { return engines == 1; }

static void req_done(unsigned long cpu) {
    struct comp_queue_t *cq = &comp_queue[cpu];
    int i;

    if (is_dakota() && cpu == hard_irq_cpu) {
        for_each_online_cpu(i) {
            struct comp_queue_t *cq = &comp_queue[i];
            if (i == hard_irq_cpu) continue;
            if (!atomic_read(&cq->wait)) continue;
            smp_call_function_single(i, &sched_cpu_tasklet, &cq->tasklet, 0);
        }
    }

    while (atomic_read(&cq->wait)) {
        atomic_dec(&cq->wait);

        struct qcrypto_resp_ctx *arsp = cq->out_ring[cq->tail];
        struct crypto_async_request *areq = arsp->areq;
        struct aead_request *req = container_of(areq,
                struct aead_request, base);
        struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(req);
        struct qce_device *pce = rctx->pce;
        struct crypto_aead *aead = crypto_aead_reqtfm(req);
        char *icv = (char *) rctx->result.auth_iv;
        const int authsize = crypto_aead_authsize(aead);
        const int aadsize = req->assoclen;
        const int bs = req->cryptlen + aadsize + (rctx->dir != QCE_ENCRYPT ? 0 : authsize);
        int ret = 0;

        if (!rctx->bounce) {
            dma_sync_sg_for_cpu(pce->pdev, req->src, rctx->src_nents, DMA_BIDIRECTIONAL);
            dma_unmap_sg(pce->pdev, req->src, rctx->src_nents, DMA_BIDIRECTIONAL);
        } else {
            dma_sync_single_for_cpu(pce->pdev, rctx->phy_bounce, bs, DMA_BIDIRECTIONAL);
            dma_unmap_single(pce->pdev, rctx->phy_bounce, bs, DMA_BIDIRECTIONAL);
        }
        dma_sync_single_for_cpu(pce->pdev, rctx->phy_result_in, CRYPTO_RESULT_DUMP_SIZE, DMA_FROM_DEVICE);
        dma_unmap_single(pce->pdev, rctx->phy_result_in, CRYPTO_RESULT_DUMP_SIZE, DMA_FROM_DEVICE);
        dma_unmap_single(pce->pdev, rctx->phy_cmd_in, rctx->cmd_size, DMA_TO_DEVICE);

        atomic_dec(&pce->bh_wait);
        cq->tail = (cq->tail + 1) & (OUT_RING_SIZE - 1);

        // 0x02000006
        // 0x0c180042 // hsd, access err set
        // 0x1d180242

        //#define CRYPTO_MAC_FAILED			31
        //XXX tikai CCM vai citas modes ari strada?
        if (unlikely(rctx->result.status != 0xc180042)) {
            ret = rctx->result.status;
            if (ret & ((1 << CRYPTO_SW_ERR)
                        | (1 << CRYPTO_AXI_ERR)
                        | (1 <<  CRYPTO_HSD_ERR))) {
                    printk(KERN_ERR"aead operation error. Status %x\n", ret);
            } else if ((ret & (1 << CRYPTO_OPERATION_DONE)) == 0) {
                    printk(KERN_ERR"aead operation not done? Status %x\n", ret);
            }

            printk(KERN_ERR "crypto fail, status %x\n", ret);
        }

        // extract bounce
        if (rctx->bounce) {
            scatterwalk_map_and_copy(rctx->bounce, req->src, 0, bs, 1);
            kfree(rctx->bounce);
        }

        // XXX salt IV from hmac
        rng_salt[rng_salted[cpu]++] = *icv;
        if (rng_salted[cpu] == sizeof(rng_salt)) rng_salted[cpu] = 0;

        // icv
        if (rctx->dir == QCE_ENCRYPT) {
            scatterwalk_map_and_copy(icv, req->src,
                    aadsize + req->cryptlen,
                    authsize, 1);

        } else {
            unsigned char tmp[SHA256_DIGESTSIZE];
            scatterwalk_map_and_copy(tmp, req->src,
                    aadsize + req->cryptlen - authsize,
                    authsize, 0);
            if (memcmp(icv, tmp, authsize)) {
                ret = -EBADMSG;
                pce->auth_fails++;
            }
        }
        rctx->rsp_entry.res = ret;
        arsp->areq->complete(arsp->areq, arsp->res);
    }
}

void _qce_aead_complete(struct sps_event_notify *notify) {
    struct qce_device *pce = notify->user;
    struct qcrypto_resp_ctx *arsp = pce->ring[pce->ring_tail];
    struct comp_queue_t *cq = &comp_queue[pce->cpu];

    if (unlikely(notify->event_id != SPS_EVENT_DESC_DONE)) {
        printk("crypto unhandled irq: %d \n", notify->event_id);
        return;
    }

    pce->ring[pce->ring_tail] = NULL;
    pce->ring_tail = (pce->ring_tail + 1) & (RING_SIZE - 1);
    atomic_sub(arsp->ents, &pce->desc_in_count);
    atomic_inc(&pce->bh_wait);

    pce->done_seq++;
    cq->out_ring[cq->head] = arsp;

    if (pce->is_dakota) {
        if (unlikely(hard_irq_cpu == -1)) hard_irq_cpu = smp_processor_id();
        if (pce->cpu != hard_irq_cpu) smp_wmb();
    }

    if (atomic_inc_return(&cq->wait) == 1) {
        tasklet_hi_schedule(&comp_queue[pce->is_dakota ? hard_irq_cpu : pce->cpu].tasklet);
    }
    cq->head = (cq->head + 1) & (OUT_RING_SIZE - 1);
}

static inline int _start_qcrypto_process(struct crypto_async_request *async_req, struct qce_device *pce) {
    struct comp_queue_t *cq = &comp_queue[pce->cpu];
    struct qcrypto_cipher_ctx *cipher_ctx = crypto_tfm_ctx(async_req->tfm);
    struct aead_request *areq = container_of(async_req, struct aead_request, base);
    struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(areq);
    struct qcrypto_resp_ctx *arsp = &rctx->rsp_entry;
    struct crypto_aead *aead = crypto_aead_reqtfm(areq);
    const unsigned char *iv = sg_virt(areq->src) + areq->assoclen;
    const int authsize = crypto_aead_authsize(aead);
    const int cryptlen = areq->cryptlen - (rctx->dir == QCE_ENCRYPT ? 0 : authsize) - crypto_aead_ivsize(aead);
    const int aadsize = areq->assoclen + crypto_aead_ivsize(aead);
    const int totallen = cryptlen + aadsize;
    const int bs = totallen + authsize;
    int ret = 0;

    if (unlikely(areq->src != areq->dst)) {
	return -EBADMSG;
    }

    if (rctx->dir == QCE_ENCRYPT) rng_get(iv, crypto_aead_ivsize(aead));

    rctx->pce = pce;
    struct sps_command_element *pvaddr =
        (struct sps_command_element *) rctx->cmd;
    rctx->cmd_size_const = _setup_aead_const(pce, &pvaddr, cipher_ctx, aadsize);
    rctx->cmd_size = rctx->cmd_size_const
                    + _setup_aead_dynamic(&pvaddr, async_req, cipher_ctx, iv,
                                            cryptlen, totallen);
    if (unlikely(rctx->cmd_size < 0)) {
        pr_err("crypto cmd setup failed\n");
        return -EINVAL;
    }

    if (unlikely(totallen > SPS_MAX_PKT_SIZE)) { // > 32k
        rctx->bounce = kmalloc(bs, GFP_ATOMIC);
        if (unlikely(!rctx->bounce)) return -ENOMEM;
        scatterwalk_map_and_copy(rctx->bounce, areq->src, 0, bs, 0);
        rctx->phy_bounce = dma_map_single(pce->pdev, rctx->bounce, bs, DMA_BIDIRECTIONAL);
        dma_sync_single_for_device(pce->pdev, rctx->phy_bounce, bs, DMA_BIDIRECTIONAL);
        arsp->ents = 1 + 1;
        pce->bounce_req++;
    } else {
        rctx->bounce = NULL;
        dma_map_sg(pce->pdev, areq->src, rctx->src_nents, DMA_BIDIRECTIONAL);
        dma_sync_sg_for_device(pce->pdev, areq->src, rctx->src_nents, DMA_BIDIRECTIONAL);
        arsp->ents = 1 + rctx->src_nents;
    }

    rctx->phy_cmd_in = dma_map_single(pce->pdev, rctx->cmd, rctx->cmd_size, DMA_TO_DEVICE);
    dma_sync_single_for_device(pce->pdev, rctx->phy_cmd_in, rctx->cmd_size, DMA_TO_DEVICE);
    rctx->phy_result_in = dma_map_single(pce->pdev, &rctx->result, CRYPTO_RESULT_DUMP_SIZE, DMA_FROM_DEVICE);

    arsp->res = -EINPROGRESS;
    arsp->areq = async_req;

    spin_lock_bh(&lock);
    if (pce->ring_tail == ((pce->ring_head + 1) & (RING_SIZE - 1))
            || atomic_read(&pce->desc_in_count) >= QCE_MAX_NUM_DSCR - 1 - arsp->ents
            || atomic_read(&cq->wait) > OUT_RING_SIZE - 1 - RING_SIZE) {
        pce->busy_req++;
        spin_unlock_bh(&lock);
        if (!rctx->bounce) dma_unmap_sg(pce->pdev, areq->src, rctx->src_nents, DMA_BIDIRECTIONAL);
        else dma_unmap_single(pce->pdev, rctx->phy_bounce, bs, DMA_BIDIRECTIONAL);
        return -ENOMEM;
    }

    if (pce->last_context == cipher_ctx) {
        pce->same_context++;
    } else {
        pce->last_context = cipher_ctx;
        rctx->cmd_size_const = 0;
    }

    pce->ring[pce->ring_head] = arsp;
    smp_wmb();
    pce->ring_head = (pce->ring_head + 1) & (RING_SIZE - 1);
    ret = qce_aead_req(pce, async_req, rctx, cryptlen);
    if (likely(!ret)) {
        pce->total_req++;
        atomic_add(arsp->ents, &pce->desc_in_count);
        spin_unlock_bh(&lock);
        return -EINPROGRESS;
    }

    spin_unlock_bh(&lock);
    panic("crypto error: submit failed\n");
}

static int _qcrypto_queue_req(struct crypto_async_request *async_req)
{
    static unsigned char rr[2] = { 0, 0 };
    struct aead_request *aead_req = container_of(async_req, struct aead_request, base);
    struct qcrypto_cipher_req_ctx *cipher_rctx = aead_request_ctx(aead_req);
    struct crypto_aead *aead = crypto_aead_reqtfm(aead_req);

    const int e = is_dakota() ? 0 : ((rr[smp_processor_id()]++ & 1) + smp_processor_id() * 2);
    struct qce_device *pce = engine[e];

    cipher_rctx->src_nents = sg_nents_for_len(aead_req->src,
            aead_req->cryptlen + aead_req->assoclen
            - (cipher_rctx->dir == QCE_ENCRYPT ? 0 : crypto_aead_authsize(aead)));
    if (unlikely(cipher_rctx->src_nents < 0)) return cipher_rctx->src_nents;

    return _start_qcrypto_process(async_req, pce);
}

static int _qcrypto_aead_setauthsize(struct crypto_aead *tfm, unsigned int authsize)
{
    struct qcrypto_cipher_ctx *ctx = crypto_aead_ctx(tfm);

    if (ctx->fallback) {
        return crypto_aead_setauthsize(ctx->fallback, authsize);
    }
    return 0;
}

static int _qcrypto_aead_setkey(struct crypto_aead *tfm, const u8 *key,
			unsigned int keylen)
{
	struct qcrypto_cipher_ctx *ctx = crypto_aead_ctx(tfm);
        struct crypto_authenc_keys keys;

	int res = crypto_authenc_extractkeys(&keys, key, keylen);
        if (res) {
            printk(KERN_WARNING "bad key %x \n", res);
            goto badkey;
        }

	ctx->enc_key_len = keys.enckeylen;
	ctx->auth_key_len = keys.authkeylen;

        if (ctx->mode == QCE_MODE_CTR) {
            ctx->enc_key_len -= 4;
            memcpy(&ctx->nonce, keys.enckey + ctx->enc_key_len, 4);
        }

	if (ctx->enc_key_len >= QCRYPTO_MAX_KEY_SIZE
                || ctx->auth_key_len >= QCRYPTO_MAX_KEY_SIZE)
            goto badkey;

        if (is_sw_fallback(ctx)) {
            return crypto_aead_setkey(ctx->fallback, key, keylen);
        }

	memset(ctx->auth_key, 0, QCRYPTO_MAX_KEY_SIZE);
	memcpy(ctx->auth_key, keys.authkey, ctx->auth_key_len);
	memcpy(ctx->enc_key, keys.enckey, ctx->enc_key_len);

	return 0;
badkey:
	ctx->enc_key_len = 0;
	return -EINVAL;
}

static int _qcrypto_aead_encrypt(struct aead_request *req)
{
        struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(req);

        if (unlikely(is_sw_fallback(ctx))) {
            struct crypto_aead *authenc = crypto_aead_reqtfm(req);
            int ret;
            aead_request_set_tfm(req, ctx->fallback);
            ret = crypto_aead_encrypt(req);
            aead_request_set_tfm(req, authenc);
            return ret;
        }

	rctx->dir = QCE_ENCRYPT;
	return _qcrypto_queue_req(&req->base);
}

static int _qcrypto_aead_decrypt(struct aead_request *req)
{
        struct qcrypto_cipher_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
	struct qcrypto_cipher_req_ctx *rctx = aead_request_ctx(req);

        if (unlikely(is_sw_fallback(ctx))) {
            struct crypto_aead *authenc = crypto_aead_reqtfm(req);
            int ret;
            aead_request_set_tfm(req, ctx->fallback);
            ret = crypto_aead_decrypt(req);
            aead_request_set_tfm(req, authenc);
            return ret;
        }

	rctx->dir = QCE_DECRYPT;
	return _qcrypto_queue_req(&req->base);
}

struct qcrypto_alg {
	struct aead_alg alg;
	enum qce_cipher_alg_enum cipher_alg;
	enum qce_cipher_mode_enum mode;
	enum qce_hash_alg_enum  auth_alg;
};

static int _qcrypto_cra_aead_init(struct crypto_tfm *tfm)
{
        struct crypto_aead *aead = __crypto_aead_cast(tfm);
        struct aead_alg *aalg = crypto_aead_alg(aead);
        struct qcrypto_alg *qalg = container_of(aalg, struct qcrypto_alg, alg);
	struct qcrypto_cipher_ctx *tctx = crypto_tfm_ctx(tfm);

	tctx->auth_alg = qalg->auth_alg;
	tctx->cipher_alg = qalg->cipher_alg;
	tctx->mode = qalg->mode;
        tctx->fallback = NULL;

        if (tctx->cipher_alg == CIPHER_ALG_AES) {
            struct crypto_alg *alg = tfm->__crt_alg;
            tctx->fallback = crypto_alloc_aead(alg->cra_name, 0,
                                CRYPTO_ALG_ASYNC | CRYPTO_ALG_NEED_FALLBACK);
            if (IS_ERR(tctx->fallback)) {
                printk(KERN_ERR "Error allocating fallback algo %s\n", alg->cra_name);
                return PTR_ERR(tctx->fallback);
            }
        }

        crypto_aead_set_reqsize(aead, sizeof(struct qcrypto_cipher_req_ctx));
	return 0;
};

static void _qcrypto_cra_aead_exit(struct crypto_tfm *tfm) {
    struct qcrypto_cipher_ctx *tctx = crypto_tfm_ctx(tfm);
    if (tctx->fallback) crypto_free_aead(tctx->fallback);
};

static struct qcrypto_alg _qcrypto_aead_algos[] = {
    // cbc
    {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha1),cbc(aes)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-aes",
		.cra_blocksize  = AES_BLOCK_SIZE,
            },
            .ivsize         = AES_BLOCK_SIZE,
            .maxauthsize    = SHA1_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_AES,
        .auth_alg = QCE_HASH_SHA1_HMAC,
        .mode = QCE_MODE_CBC,
    }, {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha1),cbc(des)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-des",
		.cra_blocksize  = DES_BLOCK_SIZE,
            },
            .ivsize         = DES_BLOCK_SIZE,
            .maxauthsize    = SHA1_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_DES,
        .auth_alg = QCE_HASH_SHA1_HMAC,
        .mode = QCE_MODE_CBC,
    }, {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha1),cbc(des3_ede)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-cbc-3des",
		.cra_blocksize  = DES3_EDE_BLOCK_SIZE,
            },
            .ivsize         = DES3_EDE_BLOCK_SIZE,
            .maxauthsize    = SHA1_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_3DES,
        .auth_alg = QCE_HASH_SHA1_HMAC,
        .mode = QCE_MODE_CBC,
    }, {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha256),cbc(aes)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha256-cbc-aes",
		.cra_blocksize  = AES_BLOCK_SIZE,
            },
            .ivsize         = AES_BLOCK_SIZE,
            .maxauthsize    = SHA256_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_AES,
        .auth_alg = QCE_HASH_SHA256_HMAC,
        .mode = QCE_MODE_CBC,
    }, {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha256),cbc(des)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha256-cbc-des",
		.cra_blocksize  = DES_BLOCK_SIZE,
            },
            .ivsize         = DES_BLOCK_SIZE,
            .maxauthsize    = SHA256_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_DES,
        .auth_alg = QCE_HASH_SHA256_HMAC,
        .mode = QCE_MODE_CBC,
    }, {
	.alg = {
            .base = {
		.cra_name	= "echainiv(authenc(hmac(sha256),cbc(des3_ede)))",
		.cra_driver_name = "qcrypto-aead-hmac-sha256-cbc-3des",
		.cra_blocksize  = DES3_EDE_BLOCK_SIZE,
            },
            .ivsize         = DES3_EDE_BLOCK_SIZE,
            .maxauthsize    = SHA256_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_3DES,
        .auth_alg = QCE_HASH_SHA256_HMAC,
        .mode = QCE_MODE_CBC,
    },
    // ctr
    {
	.alg = {
            .base = {
		.cra_name	= "seqiv(authenc(hmac(sha1),rfc3686(ctr(aes))))",
		.cra_driver_name = "qcrypto-aead-hmac-sha1-rfc3686-ctr-aes",
		.cra_blocksize  = AES_BLOCK_SIZE,
            },
            .ivsize         = 8,
            .maxauthsize    = SHA1_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_AES,
        .auth_alg = QCE_HASH_SHA1_HMAC,
        .mode = QCE_MODE_CTR,
    }, {
	.alg = {
            .base = {
		.cra_name	= "seqiv(authenc(hmac(sha256),rfc3686(ctr(aes))))",
		.cra_driver_name = "qcrypto-aead-hmac-sha256-rfc3686-ctr-aes",
		.cra_blocksize  = AES_BLOCK_SIZE,
            },
            .ivsize         = 8,
            .maxauthsize    = SHA256_DIGEST_SIZE,
	},
        .cipher_alg = CIPHER_ALG_AES,
        .auth_alg = QCE_HASH_SHA256_HMAC,
        .mode = QCE_MODE_CTR,
    },
};

static int  _qcrypto_probe(struct platform_device *pdev)
{
	int i, rc;

        struct qce_device *pce = qce_open(pdev, &rc);
	if (pce == NULL) return -1;

	platform_set_drvdata(pdev, pce);

        if (!pce->is_dakota && engines != 4) return 0;

        rng_pool = kmalloc(RNG_MAX, GFP_ATOMIC);
        if (!rng_pool) {
            printk(KERN_ERR "crypto: rng_pool failed\n");
            return -2;
        }
        atomic_set(&rng_used, RNG_MAX);
        get_random_bytes(rng_pool, RNG_MAX);
        get_random_bytes(rng_salt, sizeof(rng_salt));

        for_each_online_cpu(i) {
            struct comp_queue_t *cq = &comp_queue[i];
            tasklet_init(&cq->tasklet, req_done, (unsigned long) i);
            atomic_set(&cq->wait, 0);
            cq->head = cq->tail = 0;
        }

        schedule_delayed_work(&reg_work, 10);
        return 0;
}

static void _qcrypto_register_timer(struct work_struct *work) {
        int i;
        for (i = 0; i < engines; ++i) {
            struct qce_device *pce = engine[i];
            const int cpu_map[] = { 0, 0, 1, 1 };
            pce->cpu = cpu_map[i];

            if (irq_can_set_affinity(pce->ce_sps.bam_irq)) {
                printk("bam_irq: irq_set_affinity() engine: %d cpu:%d\n", i, pce->cpu);
                int ret = irq_set_affinity(pce->ce_sps.bam_irq, cpumask_of(pce->cpu));
                if (ret) {
                    printk("bam_irq: irq_set_affinity() failed: %d\n", ret);
                }
            }
        }

        for (i = 0; i < ARRAY_SIZE(_qcrypto_aead_algos); i++) {
		struct aead_alg *alg = &_qcrypto_aead_algos[i].alg;
		alg->base.cra_priority = 3000;
		alg->base.cra_flags = CRYPTO_ALG_ASYNC;
		alg->base.cra_ctxsize = sizeof(struct qcrypto_cipher_ctx);
		alg->base.cra_alignmask = 0;
		alg->base.cra_module = THIS_MODULE;
		alg->base.cra_init = _qcrypto_cra_aead_init;
		alg->base.cra_exit = _qcrypto_cra_aead_exit;
		alg->setauthsize = _qcrypto_aead_setauthsize;
		alg->setkey = _qcrypto_aead_setkey;
		alg->encrypt = _qcrypto_aead_encrypt;
		alg->decrypt = _qcrypto_aead_decrypt;

                if (_qcrypto_aead_algos[i].cipher_alg == CIPHER_ALG_AES) {
                    alg->base.cra_flags |= CRYPTO_ALG_NEED_FALLBACK;
                }
                int rc = crypto_register_aead(alg);
                if (rc) {
                        printk("%s alg registration failed\n",
                                alg->base.cra_name);
                } else {
                        printk("%s\n", alg->base.cra_name);
                }
        }
};

static int _qcrypto_remove(struct platform_device *pdev)
{
        struct qce_device *pengine = platform_get_drvdata(pdev);
        int i;

	if (!pengine) return 0;

        for_each_online_cpu(i) {
            struct comp_queue_t *cq = &comp_queue[i];
            tasklet_kill(&cq->tasklet);
        }

        for (i = 0; i < ARRAY_SIZE(_qcrypto_aead_algos); i++) {
            crypto_unregister_aead(&_qcrypto_aead_algos[i].alg);
	}

	qce_close(pengine);

	return 0;
}

static struct of_device_id qcrypto_match[] = {
	{	.compatible = "qcom,qcrypto",
	},
	{}
};

static struct platform_driver _qualcomm_crypto = {
	.probe          = _qcrypto_probe,
	.remove         = _qcrypto_remove,
	.driver         = {
		.owner  = THIS_MODULE,
		.name   = "qcrypto",
		.of_match_table = qcrypto_match,
	},
};

static int crypto_proc_read(struct seq_file *m, void *data) {
    int i;

    for (i = 0; i < engines; ++i) {
    struct qce_device *pengine = engine[i];
    int used = pengine->ring_head - pengine->ring_tail;
    int ds = pengine->done_seq;
    int rs = pengine->total_req;
    int bs = pengine->busy_req;
    int es = pengine->err_req;
    int br = pengine->bounce_req;
    int sc = pengine->same_context;
    int af = pengine->auth_fails;
    int di = atomic_read(&pengine->desc_in_count);
    int bw = atomic_read(&pengine->bh_wait);

    seq_printf(m, "---------------- engine: %u\n", i);
    if (used < 0) used += (RING_SIZE - 1);
    seq_printf(m, "ring in use: %u\n", used);
    seq_printf(m, "desc in use: %u\n", di);
    seq_printf(m, "busy: %u\n", bs);
    seq_printf(m, "failed submit: %u\n", es);
    seq_printf(m, "requested: %u\n", rs);
    seq_printf(m, "done: %u\n", ds);
    seq_printf(m, "bounce: %u\n", br);
    seq_printf(m, "failed auth: %u\n", af);
    seq_printf(m, "same context: %u\n", sc);
    seq_printf(m, "hw active: %u\n", rs - ds);
    seq_printf(m, "bh active: %u\n", bw);
    }

    seq_printf(m, "---------------- \n");
    seq_printf(m, "ring size: %u\n", RING_SIZE);
    seq_printf(m, "rng used: %u\n", atomic_read(&rng_used));
    for_each_online_cpu(i) {
        struct comp_queue_t *cq = &comp_queue[i];
        int w = atomic_read(&cq->wait);
        int u = cq->head - cq->tail;
        if (u < 0) u += (OUT_RING_SIZE - 1);
        seq_printf(m, "\tcpu %d active: %d %d\n", i, u, w);
    }
    return 0;
}

static int proc_open(struct inode *inode, struct file *file) {
       return single_open(file, crypto_proc_read, NULL);
}

static const struct proc_ops proc_fops = {
       .proc_open      = proc_open,
       .proc_read      = seq_read,
       .proc_lseek     = seq_lseek,
       .proc_release   = single_release,
};

static int __init _qcrypto_init(void) {
	entry = proc_create(PROCNAME, 0666, NULL, &proc_fops);
        if (entry == NULL) {
            printk(KERN_ERR "crypto: unable to create %s\n", PROCNAME);
            return -1;
        }

	spin_lock_init(&lock);
	return platform_driver_register(&_qualcomm_crypto);
}

static void __exit _qcrypto_exit(void)
{
	pr_debug("%s Unregister QCRYPTO\n", __func__);
        if (rng_pool) kfree(rng_pool);
	platform_driver_unregister(&_qualcomm_crypto);
        if (entry) proc_remove(entry);
}

module_init(_qcrypto_init);
module_exit(_qcrypto_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Qualcomm Crypto driver");
