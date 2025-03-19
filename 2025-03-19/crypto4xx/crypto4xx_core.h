/**
 * AMCC SoC PPC4xx Crypto Driver
 *
 * Copyright (c) 2008 Applied Micro Circuits Corporation.
 * All rights reserved. James Hsiao <jhsiao@amcc.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This is the header file for AMCC Crypto offload Linux device driver for
 * use with Linux CryptoAPI.

 */

#ifndef __CRYPTO4XX_CORE_H__
#define __CRYPTO4XX_CORE_H__

#include <crypto/internal/hash.h>
#include <crypto/aead.h>

#define PPC460SX_SDR0_SRST                      0x201
#define PPC405EX_SDR0_SRST                      0x200
#define PPC460EX_SDR0_SRST                      0x201
#define PPC460EX_CE_RESET                       0x08000000
#define PPC460SX_CE_RESET                       0x20000000
#define PPC405EX_CE_RESET                       0x00000008

#define CRYPTO4XX_CRYPTO_PRIORITY		300
#define PPC4XX_LAST_PD				63
#define PPC4XX_NUM_PD				64
#define PPC4XX_LAST_GD				1023
#define PPC4XX_NUM_GD				1024
#define PPC4XX_LAST_SD				63
#define PPC4XX_NUM_SD				64
#define PPC4XX_SD_BUFFER_SIZE			2048

#define PD_ENTRY_INUSE				1
#define PD_ENTRY_FREE				0
#define ERING_WAS_FULL				0xffffffff

#define _4XX_ENABLE_CRYPT   1
#define _4XX_ENABLE_HASH    0
#define _4XX_ENABLE_AEAD    0

struct crypto4xx_device;

struct pd_uinfo {
	struct crypto4xx_device *dev;
	u32   state;
	u32 using_sd;
	u32 first_gd;		/* first gather discriptor
				used by this packet */
	u32 num_gd;             /* number of gather discriptor
				used by this packet */
	u32 first_sd;		/* first scatter discriptor
				used by this packet */
	u32 num_sd;		/* number of scatter discriptors
				used by this packet */
	void *sa_va;		/* shadow sa, when using cp from ctx->sa */
	u32 sa_pa;
	void *sr_va;		/* state record for shadow sa */
	u32 sr_pa;
	struct scatterlist *dest_va;
	struct crypto_async_request *async_req; 	/* base crypto request
							for this packet */
};

struct crypto4xx_device {
	struct crypto4xx_core_device *core_dev;
	char *name;
	u64  ce_phy_address;
	void __iomem *ce_base;

	void *pdr;			/* base address of packet
					descriptor ring */
	dma_addr_t pdr_pa;		/* physical address used to
					program ce pdr_base_register */
	void *gdr;                      /* gather descriptor ring */
	dma_addr_t gdr_pa;		/* physical address used to
					program ce gdr_base_register */
	void *sdr;			/* scatter descriptor ring */
	dma_addr_t sdr_pa;		/* physical address used to
					program ce sdr_base_register */
	void *scatter_buffer_va;
	dma_addr_t scatter_buffer_pa;
	u32 scatter_buffer_size;

	void *shadow_sa_pool;		/* pool of memory for sa in pd_uinfo */
	dma_addr_t shadow_sa_pool_pa;
	void *shadow_sr_pool;		/* pool of memory for sr in pd_uinfo */
	dma_addr_t shadow_sr_pool_pa;
	u32 pdr_tail;
	u32 pdr_head;
	u32 gdr_tail;
	u32 gdr_head;
	u32 sdr_tail;
	u32 sdr_head;
	void *pdr_uinfo;
	struct list_head alg_list;	/* List of algorithm supported
					by this device */
};

struct crypto4xx_core_device {
	struct device *device;
	struct platform_device *ofdev;
	struct crypto4xx_device *dev;
	u32 int_status;
	u32 irq;
	spinlock_t lock;
};

struct crypto4xx_ctx_base {
	struct crypto4xx_device *dev;
	void *sa_in;
	dma_addr_t sa_in_dma_addr;
	void *sa_out;
	dma_addr_t sa_out_dma_addr;
	void *state_record;
	dma_addr_t state_record_dma_addr;
	u32 offset_to_sr_ptr;           /* offset to state ptr, in dynamic sa */
	u32 direction;
	u32 pd_ctl_len;
	u32 pd_ctl;
	u32 is_hash;
	u32 hash_final;
	u32 sa_len;
        // used only for ciphers
        u32 is_gcm;
	u32 ctr_aes;
	u32 bypass;
};

struct crypto4xx_cipher_tfm_ctx {
        struct crypto4xx_ctx_base base;
        struct crypto_skcipher *fallback;
	dma_addr_t arc4_state_record_dma_addr;
	void *arc4_state_record;
	u32 append_icv;
	u32 init_arc4;
};

struct crypto4xx_cipher_req_ctx {
};

struct crypto4xx_ahash_tfm_ctx {
    struct crypto4xx_device *dev;
    struct crypto_ahash *fallback;
    u32 digest_size;
    u32 block_size;
    u32 sa_len;
    u8 hash;
    u8 hash_mode;
    u8 iv[2 * 512];
};

struct crypto4xx_ahash_req_ctx {
    struct crypto4xx_ctx_base base;
    struct ahash_request fallback_req;
};

struct crypto4xx_alg_common {
	u32 type;
	union {
		struct crypto_alg cipher;
		struct ahash_alg hash;
		struct aead_alg aead;
	} u;
};

struct crypto4xx_alg {
	struct list_head  entry;
	struct crypto4xx_alg_common alg;
	struct crypto4xx_device *dev;
};

static inline struct crypto4xx_alg *crypto_alg_to_crypto4xx_alg(
	struct crypto_alg *x)
{
	switch (x->cra_flags & CRYPTO_ALG_TYPE_MASK) {
	case CRYPTO_ALG_TYPE_AHASH:
		return container_of(__crypto_ahash_alg(x),
				    struct crypto4xx_alg, alg.u.hash);
	}

	return container_of(x, struct crypto4xx_alg, alg.u.cipher);
}

extern int crypto4xx_alloc_sa(struct crypto4xx_ctx_base *ctx, u32 size);
extern void crypto4xx_free_sa(struct crypto4xx_ctx_base *ctx);
extern u32 crypto4xx_alloc_state_record(struct crypto4xx_ctx_base *ctx);
extern u32 get_dynamic_sa_offset_state_ptr_field(struct crypto4xx_ctx_base *ctx);
extern u32 get_dynamic_sa_offset_key_field(struct crypto4xx_ctx_base *ctx);
extern u32 get_dynamic_sa_iv_size(struct crypto4xx_ctx_base *ctx);
extern void crypto4xx_memcpy_le(unsigned int *dst,
				const unsigned char *buf, int len);
extern u32 crypto4xx_build_pd(struct crypto_async_request *req,
			      struct crypto4xx_ctx_base *ctx,
			      struct scatterlist *src,
			      struct scatterlist *dst,
			      unsigned int datalen,
			      u32 aad_len,
			      void *iv, u32 iv_len);
extern int crypto4xx_setkey_aes_cbc(struct crypto_ablkcipher *cipher,
				    const u8 *key, unsigned int keylen);
extern int crypto4xx_setkey_3des_cbc(struct crypto_ablkcipher *cipher,
				     const u8 *key, unsigned int keylen);
extern int crypto4xx_setkey_3des_ecb(struct crypto_ablkcipher *cipher,
				     const u8 *key, unsigned int keylen);
extern int crypto4xx_encrypt(struct ablkcipher_request *req);
extern int crypto4xx_decrypt(struct ablkcipher_request *req);

// hashing
extern int crypto4xx_hash_digest(struct ahash_request *req);
extern int crypto4xx_hash_final(struct ahash_request *req);
extern int crypto4xx_hash_update(struct ahash_request *req);
extern int crypto4xx_hash_init(struct ahash_request *req);
extern void crypto4xx_hash_alg_exit(struct crypto_tfm *tfm);
extern int crypto4xx_md5_alg_init(struct crypto_tfm *tfm);
extern int crypto4xx_md5_hmac_init(struct crypto_tfm *tfm);
extern int crypto4xx_md5_hmac_setkey(struct crypto_ahash *hash, const u8 *key,
			      unsigned int keylen);
extern int crypto4xx_sha1_alg_init(struct crypto_tfm *tfm);
extern int crypto4xx_sha1_hmac_init(struct crypto_tfm *tfm);
extern int crypto4xx_sha1_hmac_setkey(struct crypto_ahash *hash, const u8 *key,
			       unsigned int keylen);
extern int crypto4xx_sha2_alg_init(struct crypto_tfm *tfm);
extern int crypto4xx_sha2_hmac_init(struct crypto_tfm *tfm);
extern int crypto4xx_sha2_hmac_setkey(struct crypto_ahash *hash,
			       const u8 *key,
			       unsigned int keylen);
extern u32 get_dynamic_sa_offset_inner_digest(struct crypto4xx_ctx_base *ctx);
extern u32 get_dynamic_sa_offset_outer_digest(struct crypto4xx_ctx_base *ctx);
extern int crypto4xx_pre_compute_hmac(struct crypto4xx_ahash_tfm_ctx *ctx,
			       void *key,
			       unsigned int keylen,
			       unsigned int bs,
			       unsigned char ha,
			       unsigned char digs);

int crypto4xx_setkey_aes_ecb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen);
int crypto4xx_setkey_aes_ofb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen);
int crypto4xx_setkey_aes_cfb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen);
int crypto4xx_setkey_aes_ctr(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen);
#if _4XX_ENABLE_AEAD
int crypto4xx_setkey_aes_gcm(struct crypto_aead *cipher,
			     const u8 *key, unsigned int keylen);
int crypto4xx_setkey_aes_ccm(struct crypto_aead *cipher,
			     const u8 *key, unsigned int keylen);

int crypto4xx_encrypt_aes_gcm(struct aead_request *req);
int crypto4xx_decrypt_aes_gcm(struct aead_request *req);
int crypto4xx_encrypt_aes_ccm(struct aead_request *req);
int crypto4xx_decrypt_aes_ccm(struct aead_request *req);
#endif
int crypto4xx_encrypt_ctr(struct ablkcipher_request *req);
int crypto4xx_decrypt_ctr(struct ablkcipher_request *req);
int crypto4xx_setauthsize_aes(struct crypto_aead *ciper,
			      unsigned int authsize);
extern u32 get_dynamic_sa_offset_arc4_state_ptr(struct crypto4xx_ctx_base *ctx);
int crypto4xx_setauthsize_aes_ccm(struct crypto_aead *ciper,
				  unsigned int authsize);

#if _4XX_ENABLE_HASH
/* From crypto/md5.c */
extern unsigned int crypto4xx_sa_hash_tbl[3][6];
#endif

#endif
