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
 * This file implements the Linux crypto algorithms.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/spinlock_types.h>
#include <linux/scatterlist.h>
#include <linux/crypto.h>
#include <linux/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/aead.h>
#include <linux/dma-mapping.h>
#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/md5.h>
#include <crypto/authenc.h>
#include <crypto/skcipher.h>
#include "crypto4xx_reg_def.h"
#include "crypto4xx_sa.h"
#include "crypto4xx_core.h"


void crypto4xx_free_state_record(struct crypto4xx_ctx_base *ctx);

#if _4XX_ENABLE_HASH
static const char md5_zero[SA_HASH_ALG_MD5_DIGEST_SIZE] = {
    0xd4, 0x1d, 0x8c, 0xd9, 0x8f, 0x00, 0xb2, 0x04,
    0xe9, 0x80, 0x09, 0x98, 0xec, 0xf8, 0x42, 0x7e,
};
static const char sha1_zero[SA_HASH_ALG_SHA1_DIGEST_SIZE] = {
    0xda, 0x39, 0xa3, 0xee, 0x5e, 0x6b, 0x4b, 0x0d, 0x32,
    0x55, 0xbf, 0xef, 0x95, 0x60, 0x18, 0x90, 0xaf, 0xd8,
    0x07, 0x09
};

static void md5_get_immediate_hash(struct crypto_tfm *tfm, u8 *data)
{
	struct shash_desc **desc = crypto_tfm_ctx(tfm);
	struct md5_state *mctx = shash_desc_ctx(*desc);
	int i;

	for (i = 0; i < MD5_HASH_WORDS; i++) {
		*data++ = mctx->hash[i] & 0xFF;
		*data++ = (mctx->hash[i] >> 8) & 0xFF;
		*data++ = (mctx->hash[i] >> 16) & 0xFF;
		*data++ = (mctx->hash[i] >> 24) & 0xFF;
	}
}

static void sha1_get_immediate_hash(struct crypto_tfm *tfm, u8 *data)
{
	struct shash_desc **desc = crypto_tfm_ctx(tfm);
	struct sha1_state *sctx = shash_desc_ctx(*desc);
	int i;

	for (i = 0; i < 5; i++) {
		*data++ = sctx->state[i] & 0xFF;
		*data++ = (sctx->state[i] >> 8) & 0xFF;
		*data++ = (sctx->state[i] >> 16) & 0xFF;
		*data++ = (sctx->state[i] >> 24) & 0xFF;
	}
}

static void sha256_get_immediate_hash(struct crypto_tfm *tfm, u8 *data)
{
	struct shash_desc **desc = crypto_tfm_ctx(tfm);
	struct sha256_state *sctx = shash_desc_ctx(*desc);
	int i;

	for (i = 0; i < 8; i++) {
		*data++ = sctx->state[i] & 0xFF;
		*data++ = (sctx->state[i] >> 8) & 0xFF;
		*data++ = (sctx->state[i] >> 16) & 0xFF;
		*data++ = (sctx->state[i] >> 24) & 0xFF;
	}
}

static void sha512_get_immediate_hash(struct crypto_tfm *tfm, u8 *data)
{
	struct shash_desc **desc = crypto_tfm_ctx(tfm);
	struct sha512_state *sctx = shash_desc_ctx(*desc);
	int i;

	for (i = 0; i < 8; i++) {
		*data++ = (sctx->state[i] >> 32) & 0xFF;
		*data++ = (sctx->state[i] >> 40) & 0xFF;
		*data++ = (sctx->state[i] >> 48) & 0xFF;
		*data++ = (sctx->state[i] >> 56) & 0xFF;
		*data++ = sctx->state[i] & 0xFF;
		*data++ = (sctx->state[i] >> 8) & 0xFF;
		*data++ = (sctx->state[i] >> 16) & 0xFF;
		*data++ = (sctx->state[i] >> 24) & 0xFF;
	}
}
#endif

void set_dynamic_sa_command_0(struct dynamic_sa_ctl *sa, u32 save_h,
			      u32 save_iv, u32 ld_h, u32 ld_iv, u32 hdr_proc,
			      u32 h, u32 c, u32 pad_type, u32 op_grp, u32 op,
			      u32 dir)
{
	sa->sa_command_0.w = 0;
	sa->sa_command_0.bf.save_hash_state = save_h;
	sa->sa_command_0.bf.save_iv = save_iv;
	sa->sa_command_0.bf.load_hash_state = ld_h;
	sa->sa_command_0.bf.load_iv = ld_iv;
	sa->sa_command_0.bf.hdr_proc = hdr_proc;
	sa->sa_command_0.bf.hash_alg = h;
	sa->sa_command_0.bf.cipher_alg = c;
	sa->sa_command_0.bf.pad_type = pad_type & 3;
	sa->sa_command_0.bf.extend_pad = pad_type >> 2;
	sa->sa_command_0.bf.op_group = op_grp;
	sa->sa_command_0.bf.opcode = op;
	sa->sa_command_0.bf.dir = dir;
}

void set_dynamic_sa_command_1(struct dynamic_sa_ctl *sa, u32 cm, u32 hmac_mc,
			      u32 cfb, u32 esn, u32 sn_mask, u32 mute,
			      u32 cp_pad, u32 cp_pay, u32 cp_hdr)
{
	sa->sa_command_1.w = 0;
	sa->sa_command_1.bf.crypto_mode31 = (cm & 4) >> 2;
	sa->sa_command_1.bf.crypto_mode9_8 = (cm & 3);
	sa->sa_command_1.bf.feedback_mode = cfb,
	sa->sa_command_1.bf.sa_rev = 1;
	sa->sa_command_1.bf.hmac_muting = hmac_mc;
	sa->sa_command_1.bf.extended_seq_num = esn;
	sa->sa_command_1.bf.seq_num_mask = sn_mask;
	sa->sa_command_1.bf.mutable_bit_proc = mute;
	sa->sa_command_1.bf.copy_pad = cp_pad;
	sa->sa_command_1.bf.copy_payload = cp_pay;
	sa->sa_command_1.bf.copy_hdr = cp_hdr;
}

#if _4XX_ENABLE_HASH
/** Table lookup for SA Hash Digest length and
 *  Hash Contents (based on Hash type)
 */
unsigned int crypto4xx_sa_hash_tbl[3][HASH_ALG_MAX_CNT] = {
	/* Hash Contents */
	{ SA_HASH128_CONTENTS, SA_HASH160_CONTENTS, SA_HASH256_CONTENTS,
	SA_HASH256_CONTENTS, SA_HASH512_CONTENTS, SA_HASH512_CONTENTS },
	/* Digest len */
	{4 * 4, 5 * 4, 7 * 4, 8 * 4, 12 * 4, 16 * 4},
	/* SA Length */
	{ SA_HASH128_LEN, SA_HASH160_LEN, SA_HASH256_LEN, SA_HASH256_LEN,
	SA_HASH512_LEN, SA_HASH512_LEN }
};

/** Table lookup for Hash Algorithms based on Hash type, used in
 *  crypto4xx_pre_compute_hmac()
 */
char *crypto4xx_hash_alg_map_tbl[HASH_ALG_MAX_CNT] = CRYPTO4XX_MAC_ALGS;

void crypto4xx_compute_immediate_hash(struct crypto_tfm *child_tfm, u8 *data,
				     unsigned char ha)
{
	switch (ha) {
	case SA_HASH_ALG_MD5:
		md5_get_immediate_hash(child_tfm, data);
		break;
	case SA_HASH_ALG_SHA1:
		sha1_get_immediate_hash(child_tfm, data);
		break;
	case SA_HASH_ALG_SHA256:
	case SA_HASH_ALG_SHA224:
		sha256_get_immediate_hash(child_tfm, data);
		break;
	case SA_HASH_ALG_SHA384:
	case SA_HASH_ALG_SHA512:
		sha512_get_immediate_hash(child_tfm, data);
		break;
	default:
		break;
	}
}

int crypto4xx_pre_compute_hmac(struct crypto4xx_ahash_tfm_ctx *ctx,
			       void *key,
			       unsigned int keylen,
			       unsigned int bs,
			       unsigned char ha,
			       unsigned char digs)
{
	u8 *ipad = NULL;
	u8 *opad;
	struct crypto_shash *child_hash = NULL;
	struct crypto_tfm *child_tfm;
	char *child_name = NULL;
	int i, rc = 0;
	int ds;

	BUG_ON(ha >= HASH_ALG_MAX_CNT);
	child_name = crypto4xx_hash_alg_map_tbl[ha];
	child_hash = crypto_alloc_shash(child_name, 0, 0);
	if (IS_ERR(child_hash)) {
		rc = PTR_ERR(child_hash);
		printk(KERN_ERR "failed to load "
				"transform for %s error %d\n",
				child_name, rc);
		return rc;
	}
        
        {
        SHASH_DESC_ON_STACK(desc, child_hash);

        ipad = ctx->iv;
	opad = ipad + bs;
	child_tfm = crypto_shash_tfm(child_hash);
	ds = crypto_shash_digestsize(child_hash);
	desc->tfm = child_hash;
	desc->flags = 0;
        
        ctx->block_size = bs;
        ctx->digest_size = ds;
	if (keylen > bs) {
		rc = crypto_shash_init(desc);
		if (rc < 0)
			goto err_alg_hash_key;
		rc = crypto_shash_update(desc, key, keylen);
		if (rc < 0)
			goto err_alg_hash_key;
		rc = crypto_shash_final(desc, ipad);
		if (rc < 0)
			goto err_alg_hash_key;
		keylen = ds;
	} else {
		memcpy(ipad, key, keylen);
	}
	memset(ipad + keylen, 0, bs-keylen);
	memcpy(opad, ipad, bs);

	for (i = 0; i < bs; i++) {
		ipad[i] ^= 0x36;
		opad[i] ^= 0x5c;
	}

	rc = crypto_shash_init(desc);
	if (rc < 0)
		goto err_alg_hash_key;
	rc = crypto_shash_update(desc, ipad, bs);
	if (rc < 0)
		goto err_alg_hash_key;
	
        if (ha == SA_HASH_ALG_SHA224)
		ds = SHA256_DIGEST_SIZE;
	else if (ha == SA_HASH_ALG_SHA384)
		ds = SHA512_DIGEST_SIZE;
	crypto4xx_compute_immediate_hash(child_tfm, ipad, ha);

	rc = crypto_shash_init(desc);
	if (rc < 0)
		goto err_alg_hash_key;

	rc = crypto_shash_update(desc, opad, bs);
	if (rc < 0)
		goto err_alg_hash_key;
	
        }
        crypto4xx_compute_immediate_hash(child_tfm, opad, ha);

	crypto_free_shash(child_hash);
        return 0;

err_alg_hash_key:
	crypto_free_shash(child_hash);
	return rc;
}
#endif

int crypto4xx_compute_gcm_hash_key_sw(struct crypto4xx_cipher_tfm_ctx *tctx,
				      const u8 *key,
				      unsigned int keylen)
{
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct crypto_skcipher *aes_tfm = NULL;
	struct blkcipher_desc 	desc;
	struct scatterlist sg[1];
	char src[16];
	int rc = 0;

	aes_tfm = crypto_alloc_skcipher("ecb(aes)", 0, 0);
	if (IS_ERR(aes_tfm)) {
		printk(KERN_ERR "failed to load transform for %ld\n",
		       PTR_ERR(aes_tfm));
		rc = PTR_ERR(aes_tfm);
		return rc;
	}
#if 0
            SKCIPHER_REQUEST_ON_STACK(fr, tctx->fallback);
            skcipher_request_set_tfm(fr, tctx->fallback);
            skcipher_request_set_callback(fr, req->base.flags, NULL, NULL);
            skcipher_request_set_crypt(fr, req->src, req->dst,
                                        req->nbytes, req->info);
            return crypto_skcipher_encrypt(fr);

	desc.tfm    = aes_tfm;
	desc.flags  = 0;

	memset(src, 0, 16);
	rc = crypto_blkcipher_setkey(aes_tfm, key, keylen);
	if (rc) {
		printk(KERN_ERR "setkey() failed flags=%x\n",
		       crypto_blkcipher_get_flags(aes_tfm));
		goto out;
	}

	sg_init_one(sg, src, 16);
	rc = crypto_blkcipher_encrypt(&desc, sg, sg, 16);
	if (rc)
		goto out;
	crypto4xx_memcpy_le(ctx->sa_in +
			get_dynamic_sa_offset_inner_digest(ctx), (const unsigned char*) src, 16);

out:
	crypto_free_blkcipher(aes_tfm);
#endif
	return rc;
}

/**
 * 3DES/DES Functions
 *
 */
static int crypto4xx_setkey_3des(struct crypto_ablkcipher *cipher,
				 const u8 *key,
				 unsigned int keylen,
				 unsigned char cm,
				 unsigned char fb)
{
	struct crypto_tfm    *tfm = crypto_ablkcipher_tfm(cipher);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct dynamic_sa_ctl *sa;
	int rc;

	if (keylen != DES_KEY_SIZE && keylen != DES3_EDE_KEY_SIZE) {
		crypto_ablkcipher_set_flags(cipher,
			CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	if (keylen == DES_KEY_SIZE) {
		u32 tmp[32];
		rc = des_ekey(tmp, key);
		if (unlikely(rc == 0) &&
				  (tfm->crt_flags & CRYPTO_TFM_REQ_WEAK_KEY)) {
			crypto_ablkcipher_set_flags(cipher,
				CRYPTO_TFM_RES_WEAK_KEY);
			return -EINVAL;
		}
	}

	/* Create SA */
	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, keylen == 8 ? SA_DES_LEN : SA_3DES_LEN);
	if (rc) {
		return rc;
        }
	/*
	 *  state record will state in base ctx, so iv and
	 *  hash result can be reused
	 *  also don't need to alloc each packet coming
	 */
	if (ctx->state_record == NULL) {
		rc = crypto4xx_alloc_state_record(ctx);
		if (rc) {
			crypto4xx_free_sa(ctx);
			return rc;
		}
	}
        if (tctx->fallback) crypto_skcipher_setkey(tctx->fallback, key, keylen);

	/* Setup SA */
	ctx->direction = DIR_INBOUND;
	ctx->hash_final = 0;

	sa = (struct dynamic_sa_ctl *) ctx->sa_in;
	set_dynamic_sa_command_0(sa, SA_NOT_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_NULL,
				 SA_CIPHER_ALG_DES,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_DECRYPT, DIR_INBOUND);

	set_dynamic_sa_command_1(sa, cm, SA_HASH_MODE_HASH,
				 fb, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	if (keylen == DES_KEY_SIZE) {
		crypto4xx_memcpy_le(((struct dynamic_sa_des *) sa)->key,
				      key, keylen);
		((struct dynamic_sa_des *)sa)->ctrl.sa_contents =
				SA_DES_CONTENTS;
		sa->sa_command_0.bf.cipher_alg = SA_CIPHER_ALG_DES;
	} else {
		crypto4xx_memcpy_le(((struct dynamic_sa_3des *) sa)->key,
				      key, keylen);
		((struct dynamic_sa_3des *)sa)->ctrl.sa_contents =
				SA_3DES_CONTENTS;
		sa->sa_command_0.bf.cipher_alg = SA_CIPHER_ALG_3DES;
	}

	memcpy((void *)(ctx->sa_in +
			get_dynamic_sa_offset_state_ptr_field(ctx)),
			(void *)&ctx->state_record_dma_addr, 4);
	ctx->offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(ctx);
	ctx->is_hash = 0;
	sa->sa_command_0.bf.dir = DIR_INBOUND;
	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;

	return 0;
}

int crypto4xx_setkey_3des_cfb(struct crypto_ablkcipher *cipher,
			      const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_3des(cipher, key, keylen,
					CRYPTO_MODE_CFB,
					CRYPTO_FEEDBACK_MODE_8BIT_CFB);
}

int crypto4xx_setkey_3des_ofb(struct crypto_ablkcipher *cipher,
			      const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_3des(cipher, key, keylen,
				       CRYPTO_MODE_OFB,
				       CRYPTO_FEEDBACK_MODE_64BIT_OFB);
}

int crypto4xx_setkey_3des_cbc(struct crypto_ablkcipher *cipher,
			      const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_3des(cipher, key, keylen,
				     CRYPTO_MODE_CBC,
				     CRYPTO_FEEDBACK_MODE_NO_FB);
}

int crypto4xx_setkey_3des_ecb(struct crypto_ablkcipher *cipher,
			      const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_3des(cipher, key, keylen,
				     CRYPTO_MODE_ECB,
				     CRYPTO_FEEDBACK_MODE_NO_FB);
}


int crypto4xx_encrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        int x = 0;

	ctx->direction = DIR_OUTBOUND;
	ctx->hash_final = 0;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x1;

	x = crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				   req->nbytes, 0, req->info,
				   get_dynamic_sa_iv_size(ctx));

        if (x != -EINPROGRESS && tctx->fallback) {
            SKCIPHER_REQUEST_ON_STACK(fr, tctx->fallback);
            skcipher_request_set_tfm(fr, tctx->fallback);
            skcipher_request_set_callback(fr, req->base.flags, NULL, NULL);
            skcipher_request_set_crypt(fr, req->src, req->dst,
                                        req->nbytes, req->info);
            return crypto_skcipher_encrypt(fr);
        }

        return x;
}

int crypto4xx_decrypt(struct ablkcipher_request *req)
{
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        int x = 0;

	ctx->hash_final = 0;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x1;
	ctx->direction = DIR_INBOUND;

	x = crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				   req->nbytes, 0, req->info,
				   get_dynamic_sa_iv_size(ctx));

        if (x != -EINPROGRESS && tctx->fallback) {
            SKCIPHER_REQUEST_ON_STACK(fr, tctx->fallback);
            skcipher_request_set_tfm(fr, tctx->fallback);
            skcipher_request_set_callback(fr, req->base.flags, NULL, NULL);
            skcipher_request_set_crypt(fr, req->src, req->dst,
                                        req->nbytes, req->info);
            return crypto_skcipher_decrypt(fr);
        }

        return x;
}

int crypto4xx_encrypt_ctr(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;

	ctx->hash_final = 0;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x1;
	ctx->direction = DIR_OUTBOUND;

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->nbytes, 0,
				  req->info,
				  crypto_ablkcipher_ivsize(ablkcipher));
}

int crypto4xx_decrypt_ctr(struct ablkcipher_request *req)
{
	struct crypto_ablkcipher *ablkcipher = crypto_ablkcipher_reqtfm(req);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;

	ctx->hash_final = 0;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x1;
	ctx->direction = DIR_INBOUND;

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->nbytes, 0,
				  req->info,
				  crypto_ablkcipher_ivsize(ablkcipher));
}

/**
 * AES Functions
 */
static int crypto4xx_setkey_aes(struct crypto_ablkcipher *cipher,
				const u8 *key,
				unsigned int keylen,
				unsigned char cm,
				u8 fb)
{
	struct crypto_tfm *tfm = crypto_ablkcipher_tfm(cipher);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct dynamic_sa_ctl *sa;
	int    rc;

	if (keylen != AES_KEYSIZE_256 &&
		keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_128) {
		crypto_ablkcipher_set_flags(cipher,
				CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	/* Create SA */
	if (ctx->sa_in || ctx->sa_out)
		crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_LEN + (keylen-16) / 4);
	if (rc)
		return rc;

	if (ctx->state_record == 0) {
		rc = crypto4xx_alloc_state_record(ctx);
		if (rc) {
			crypto4xx_free_sa(ctx);
			return rc;
		}
	}
        if (tctx->fallback) crypto_skcipher_setkey(tctx->fallback, key, keylen);

	/* Setup SA */
	sa = (struct dynamic_sa_ctl *) ctx->sa_in;
	ctx->hash_final = 0;

	set_dynamic_sa_command_0(sa, SA_NOT_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_NULL,
				 SA_CIPHER_ALG_AES, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_DECRYPT,
				 DIR_INBOUND);

	set_dynamic_sa_command_1(sa, cm, SA_HASH_MODE_HASH,
				 fb, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_NOT_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);
	crypto4xx_memcpy_le(ctx->sa_in + get_dynamic_sa_offset_key_field(ctx),
			    key, keylen);
	sa->sa_contents = SA_AES_CONTENTS | (keylen << 2);
	sa->sa_command_1.bf.key_len = keylen >> 3;
	ctx->is_hash = 0;
	ctx->direction = DIR_INBOUND;
	memcpy(ctx->sa_in + get_dynamic_sa_offset_state_ptr_field(ctx),
			(void *)&ctx->state_record_dma_addr, 4);
	ctx->offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(ctx);

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;

	return 0;
}

int crypto4xx_setkey_aes_ecb(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_ECB,
				    CRYPTO_FEEDBACK_MODE_NO_FB);
}

int crypto4xx_setkey_aes_cbc(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_CBC,
				    CRYPTO_FEEDBACK_MODE_NO_FB);
}

int crypto4xx_setkey_aes_ctr(struct crypto_ablkcipher *cipher,
			     const u8 *key, unsigned int keylen)
{
	struct crypto_tfm    *tfm = crypto_ablkcipher_tfm(cipher);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct dynamic_sa_ctl *sa;
	u32 cnt = 1;
	int    rc;
	u32 cm = CRYPTO_MODE_AES_CTR;

	keylen -= 4;
	/* Create SA */
	if (ctx->sa_in || ctx->sa_out)
		 crypto4xx_free_sa(ctx);

	if (keylen != AES_KEYSIZE_256 &&
		   keylen != AES_KEYSIZE_192 && keylen != AES_KEYSIZE_128) {
		crypto_ablkcipher_set_flags(cipher,
					    CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -EINVAL;
	}

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_LEN + (keylen-16) / 4);
	if (rc)
		return rc;

	if (ctx->state_record == 0) {
		rc = crypto4xx_alloc_state_record(ctx);
		if (rc) {
			crypto4xx_free_sa(ctx);
			return rc;
		}
	}

	sa = (struct dynamic_sa_ctl *) ctx->sa_in;
	ctx->hash_final = 0;
	ctx->ctr_aes = 1;

        if (tctx->fallback) crypto_skcipher_setkey(tctx->fallback, key, keylen);

	/* Setup SA */
	set_dynamic_sa_command_0(sa, SA_NOT_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_NULL,
				 SA_CIPHER_ALG_AES, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_ENCRYPT,
				 DIR_INBOUND);
	set_dynamic_sa_command_1(sa, cm, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB,
				 SA_EXTENDED_SN_OFF, SA_SEQ_MASK_OFF,
				 SA_MC_ENABLE, SA_NOT_COPY_PAD,
				 SA_NOT_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	crypto4xx_memcpy_le(ctx->sa_in + get_dynamic_sa_offset_key_field(ctx),
			    key, keylen);
	sa->sa_contents = SA_AES_CONTENTS | (keylen << 2);
	sa->sa_command_1.bf.key_len = keylen >> 3;

	ctx->direction = DIR_INBOUND;
	memcpy(ctx->sa_in + get_dynamic_sa_offset_state_ptr_field(ctx),
	       (void *)&ctx->state_record_dma_addr, 4);
	ctx->offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(ctx);

	crypto4xx_memcpy_le(ctx->state_record, key + keylen, 4);
	crypto4xx_memcpy_le(ctx->state_record + 12, (void *)&cnt, 4);

	sa->sa_command_0.bf.dir = DIR_INBOUND;

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;

	return 0;
}

int crypto4xx_setkey_aes_cfb(struct crypto_ablkcipher *cipher,
					  const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_CFB,
				    CRYPTO_FEEDBACK_MODE_128BIT_CFB);
}

int crypto4xx_setkey_aes_ofb(struct crypto_ablkcipher *cipher,
					  const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_OFB,
				    CRYPTO_FEEDBACK_MODE_64BIT_OFB);
}

int crypto4xx_setkey_aes_icm(struct crypto_ablkcipher *cipher,
					  const u8 *key, unsigned int keylen)
{
	return crypto4xx_setkey_aes(cipher, key, keylen, CRYPTO_MODE_AES_ICM,
				    CRYPTO_FEEDBACK_MODE_NO_FB);
}

#if _4XX_ENABLE_AEAD
/**
 * AES-GCM Functions
 */
static inline int crypto4xx_aes_gcm_validate_keylen(unsigned int keylen)
{
	switch (keylen) {
	case 16:
	case 20:
	case 24:
	case 30:
	case 32:
	case 36:
		return 0;
	default:
		printk(KERN_ERR "crypto4xx_setkey_aes_gcm: "
				"ERROR keylen = 0x%08x\n", keylen);
		return -EINVAL;
	}
	return -EINVAL;
}

int crypto4xx_setkey_aes_gcm(struct crypto_aead *cipher,
				     const u8 *key, unsigned int keylen)

{
	struct crypto_tfm    *tfm = crypto_aead_tfm(cipher);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct dynamic_sa_ctl *sa;
	int    rc = 0;

	u32 cm = 4;

	if (crypto4xx_aes_gcm_validate_keylen(keylen) != 0) {
		printk(KERN_ERR "crypto4xx_setkey_aes_gcm:"
				"ERROR keylen = 0x%08x\n", keylen);
		crypto_aead_set_flags(cipher, CRYPTO_TFM_RES_BAD_KEY_LEN);
	    return -EINVAL;
	}

	if (ctx->sa_in || ctx->sa_out)
		 crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_GCM_LEN + (keylen-16) / 4);
	if (rc)
		return rc;

	if (ctx->state_record == 0) {
		rc = crypto4xx_alloc_state_record(ctx);
		if (rc)
			goto err;
	}

	sa  = (struct dynamic_sa_ctl *) ctx->sa_in;

	sa->sa_contents = SA_AES_GCM_CONTENTS | (keylen << 2);
	sa->sa_command_1.bf.key_len = keylen >> 3;

	ctx->direction = DIR_INBOUND;
	crypto4xx_memcpy_le(ctx->sa_in + get_dynamic_sa_offset_key_field(ctx),
			    key, keylen);

	memcpy(ctx->sa_in + get_dynamic_sa_offset_state_ptr_field(ctx),
	       (void *)&ctx->state_record_dma_addr, 4);

	rc = crypto4xx_compute_gcm_hash_key_sw(tctx, key, keylen);
	if (rc) {
		printk(KERN_ERR "GCM hash key setting failed = %d\n", rc);
		goto err;
	}

	ctx->offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(ctx);
	ctx->is_gcm = 1;
	ctx->hash_final = 1;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x11;

	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_GHASH,
				 SA_CIPHER_ALG_AES, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_HASH_DECRYPT,
				 DIR_INBOUND);

	sa->sa_command_1.bf.crypto_mode31 = (cm & 4) >> 2;
	sa->sa_command_1.bf.crypto_mode9_8 = (cm & 3);
	sa->sa_command_1.bf.feedback_mode = 0;

	sa->sa_command_1.bf.hash_crypto_offset = 0;
	sa->sa_command_1.bf.sa_rev = 1;
	sa->sa_command_1.bf.copy_payload = 1;

	sa->sa_command_1.bf.copy_pad = 0;
	sa->sa_command_1.bf.copy_hdr = 0;
	sa->sa_command_1.bf.mutable_bit_proc = 1;
	sa->sa_command_1.bf.seq_num_mask = 1;

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	sa->sa_command_0.bf.dir = DIR_OUTBOUND;
	sa->sa_command_0.bf.opcode = SA_OPCODE_ENCRYPT_HASH;

	return 0;
err:
	crypto4xx_free_sa(ctx);
	return rc;
}

int crypto4xx_encrypt_aes_gcm(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct crypto4xx_cipher_tfm_ctx *tctx  = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;

	ctx->direction = DIR_OUTBOUND;
	tctx->append_icv = 1;

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->cryptlen, req->assoclen,
				  req->iv, crypto_aead_ivsize(aead));
}

int crypto4xx_decrypt_aes_gcm(struct aead_request *req)
{
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct crypto4xx_cipher_tfm_ctx *tctx  = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	int len = req->cryptlen - crypto_aead_authsize(aead);

	ctx->direction = DIR_INBOUND;
	tctx->append_icv = 0;
	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  len, req->assoclen,
				  req->iv, crypto_aead_ivsize(aead));
}

/**
 * AES-CCM Functions
 */
int crypto4xx_setauthsize_aes(struct crypto_aead *ciper,
			      unsigned int authsize)
{
	switch (authsize) {
	case 8:
	case 12:
	case 16:
	case 10:
		break;
	default:
		return -EINVAL;
	}

	crypto_aead_setauthsize(ciper, authsize);
	return 0;
}

int crypto4xx_setkey_aes_ccm(struct crypto_aead *cipher, const u8 *key,
				unsigned int keylen)
{
	struct crypto_tfm    *tfm = crypto_aead_tfm(cipher);
	struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct dynamic_sa_ctl *sa;
	int rc = 0;

	if (ctx->sa_in || ctx->sa_out)
		 crypto4xx_free_sa(ctx);

	rc = crypto4xx_alloc_sa(ctx, SA_AES128_CCM_LEN + (keylen-16) / 4);
	if (rc)
		return rc;

	if (ctx->state_record == 0) {
		rc = crypto4xx_alloc_state_record(ctx);
		if (rc) {
			crypto4xx_free_sa(ctx);
			return rc;
		}
	}

	/* Setup SA */
	sa  = (struct dynamic_sa_ctl *) ctx->sa_in;
	sa->sa_contents = SA_AES_CCM_CONTENTS | (keylen << 2);

	set_dynamic_sa_command_0(sa, SA_NOT_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_CBC_MAC,
				 SA_CIPHER_ALG_AES,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_HASH_DECRYPT, DIR_INBOUND);

	sa->sa_command_0.bf.digest_len = 0;
	sa->sa_command_1.bf.key_len = keylen >> 3;
	ctx->direction = DIR_INBOUND;
	tctx->append_icv = 0;
	ctx->is_gcm = 0;
	ctx->hash_final = 1;
	ctx->is_hash = 0;
	ctx->pd_ctl = 0x11;

	crypto4xx_memcpy_le(ctx->sa_in + get_dynamic_sa_offset_key_field(ctx),
			    key, keylen);
	memcpy(ctx->sa_in + get_dynamic_sa_offset_state_ptr_field(ctx),
	       (void *)&ctx->state_record_dma_addr, 4);
	ctx->offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(ctx);

	set_dynamic_sa_command_1(sa, CRYPTO_MODE_AES_CTR, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	memcpy(ctx->sa_out, ctx->sa_in, ctx->sa_len * 4);
	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_LOAD_HASH_FROM_SA, SA_LOAD_IV_FROM_STATE,
				 SA_NO_HEADER_PROC, SA_HASH_ALG_CBC_MAC,
				 SA_CIPHER_ALG_AES,
				 SA_PAD_TYPE_ZERO, SA_OP_GROUP_BASIC,
				 SA_OPCODE_ENCRYPT_HASH, DIR_OUTBOUND);
	set_dynamic_sa_command_1(sa, CRYPTO_MODE_AES_CTR, SA_HASH_MODE_HASH,
				 CRYPTO_FEEDBACK_MODE_NO_FB, SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	return 0;
}

int crypto4xx_encrypt_aes_ccm(struct aead_request *req)
{
	struct crypto4xx_cipher_tfm_ctx *tctx  = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct dynamic_sa_ctl *sa;

	ctx->direction = DIR_OUTBOUND;

	sa = (struct dynamic_sa_ctl *) ctx->sa_out;
	if (req->assoclen)
		sa->sa_command_1.bf.hash_crypto_offset = req->assoclen >> 2;

	sa->sa_command_0.bf.digest_len = (crypto_aead_authsize(aead) >> 2);
	if ((req->iv[0] & 7) == 1)
		sa->sa_command_1.bf.crypto_mode9_8 = 1;

	tctx->append_icv = 1;
	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->cryptlen, req->assoclen,
				  req->iv, 16);
}

int crypto4xx_decrypt_aes_ccm(struct aead_request *req)
{
	struct crypto4xx_cipher_tfm_ctx   *tctx  = crypto_tfm_ctx(req->base.tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
	struct crypto_aead *aead = crypto_aead_reqtfm(req);
	struct dynamic_sa_ctl *sa;

	/* Support only counter field length of 2 and 4 bytes */
	if ((req->iv[0] & 0x7) != 1 && (req->iv[0] & 0x7) != 3) {
		printk(KERN_ERR "algorithm AES-CCM "
				"unsupported counter length %d\n",
			req->iv[0] & 0x7);
		return -EINVAL;
	}

	ctx->direction = DIR_INBOUND;
	sa = (struct dynamic_sa_ctl *) ctx->sa_in;

	sa->sa_command_0.bf.digest_len = (crypto_aead_authsize(aead) >> 2);
	if ((req->iv[0] & 7) == 1)
		sa->sa_command_1.bf.crypto_mode9_8 = 1;
	else
		sa->sa_command_1.bf.crypto_mode9_8 = 0;

	return crypto4xx_build_pd(&req->base, ctx, req->src, req->dst,
				  req->cryptlen, req->assoclen,
				  req->iv, 16);
}
#endif


#if _4XX_ENABLE_HASH
/**
 * Support MD5/SHA/HMAC Hashing Algorithms
 *
 */
static int crypto4xx_hash_alg_init(struct crypto_tfm *tfm,
				   unsigned int sa_len,
				   unsigned char ha,
				   unsigned char hm)
{
	struct crypto_alg *alg = tfm->__crt_alg;
	struct crypto4xx_alg *my_alg = crypto_alg_to_crypto4xx_alg(alg);
	struct crypto4xx_ahash_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
	const char *fallback_driver_name = alg->cra_name;

	struct crypto_ahash *fallback_tfm = crypto_alloc_ahash(fallback_driver_name, 0,
					  CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(fallback_tfm)) {
            printk(KERN_WARNING
                   "Fallback driver '%s' could not be loaded!\n",
                   fallback_driver_name);
            tctx->fallback = NULL;
            crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
                             sizeof(struct crypto4xx_ahash_req_ctx));
	} else {
	    tctx->fallback = fallback_tfm;
            crypto_ahash_set_reqsize(__crypto_ahash_cast(tfm),
				 sizeof(struct crypto4xx_ahash_req_ctx)
                                 + crypto_ahash_reqsize(tctx->fallback));
        }
	tctx->dev = my_alg->dev;
        tctx->sa_len = sa_len;
        tctx->hash = ha;
        tctx->hash_mode = hm;
        tctx->digest_size = 0;
        tctx->block_size = 0;
	return 0;
}

void crypto4xx_hash_alg_exit(struct crypto_tfm *tfm) {
	struct crypto4xx_ahash_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
	if (tctx->fallback) crypto_free_ahash(tctx->fallback);
}

static int crypto4xx_hash_empty_test(struct ahash_request *req) {
    struct crypto4xx_ahash_tfm_ctx *ctx = crypto_tfm_ctx(req->base.tfm);
    switch (ctx->hash) {
        case SA_HASH_ALG_MD5:
            memcpy(req->result, md5_zero, SA_HASH_ALG_MD5_DIGEST_SIZE);
            break;
        case SA_HASH_ALG_SHA1:
            memcpy(req->result, sha1_zero, SA_HASH_ALG_SHA1_DIGEST_SIZE);
            break;
    }
    return 0;
}

int crypto4xx_hash_init(struct ahash_request *req)
{
	struct crypto4xx_ahash_req_ctx *ctx = ahash_request_ctx(req);
	struct crypto4xx_ahash_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
	struct dynamic_sa_ctl *sa;
        int rc;

        memset(ctx, 0, sizeof(*ctx));
        ctx->base.dev = tctx->dev;
	ctx->base.is_hash = 1;
	ctx->base.hash_final = 0;
	ctx->base.pd_ctl = 0x11;
	ctx->base.direction = DIR_INBOUND;

	// Create SA
        rc = crypto4xx_alloc_sa(&ctx->base, tctx->sa_len);
        if (rc) {
            return rc;
        }

        rc = crypto4xx_alloc_state_record(&ctx->base);
        if (rc)
            goto err;

	sa = (struct dynamic_sa_ctl *) ctx->base.sa_in;
	// Setup hash algorithm and hash mode
	set_dynamic_sa_command_0(sa, SA_SAVE_HASH, SA_NOT_SAVE_IV,
				 SA_NOT_LOAD_HASH, SA_LOAD_IV_FROM_SA,
				 SA_NO_HEADER_PROC,
				 tctx->hash, SA_CIPHER_ALG_NULL, SA_PAD_TYPE_ZERO,
				 SA_OP_GROUP_BASIC, SA_OPCODE_HASH,
				 DIR_INBOUND);
	set_dynamic_sa_command_1(sa, 0, tctx->hash_mode,
				 CRYPTO_FEEDBACK_MODE_NO_FB,
				 SA_EXTENDED_SN_OFF,
				 SA_SEQ_MASK_OFF, SA_MC_ENABLE,
				 SA_NOT_COPY_PAD, SA_NOT_COPY_PAYLOAD,
				 SA_NOT_COPY_HDR);

	BUG_ON(tctx->hash >= HASH_ALG_MAX_CNT);
	sa->sa_contents = crypto4xx_sa_hash_tbl[0][tctx->hash];
	memcpy((ctx->base.sa_in) + get_dynamic_sa_offset_state_ptr_field(&ctx->base),
	       (void *)&ctx->base.state_record_dma_addr, 4);
	ctx->base.offset_to_sr_ptr = get_dynamic_sa_offset_state_ptr_field(&ctx->base);
        
        if (tctx->block_size) {
            crypto4xx_memcpy_le(ctx->base.sa_in +
                    get_dynamic_sa_offset_inner_digest(&ctx->base), tctx->iv, tctx->digest_size);
            crypto4xx_memcpy_le(ctx->base.sa_in +
                    get_dynamic_sa_offset_outer_digest(&ctx->base), tctx->iv + tctx->block_size, tctx->digest_size);

            memcpy(ctx->base.sa_out, ctx->base.sa_in, ctx->base.sa_len * 4);
            sa = (struct dynamic_sa_ctl *) ctx->base.sa_out;
            sa->sa_command_0.bf.dir = DIR_OUTBOUND;
        }
	return 0;

err:
	crypto4xx_free_sa(&ctx->base);
	return rc;
}

int crypto4xx_hash_update(struct ahash_request *req)
{
	struct crypto4xx_ahash_req_ctx *ctx = ahash_request_ctx(req);

	if (unlikely(req->nbytes == 0)) return crypto4xx_hash_empty_test(req);

	return crypto4xx_build_pd(&req->base, &ctx->base, req->src,
				  (struct scatterlist *) req->result,
				  req->nbytes, 0, NULL, 0);
}

int crypto4xx_hash_final(struct ahash_request *req)
{
    //struct crypto4xx_ahash_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
    return 0;
}

int crypto4xx_hash_digest(struct ahash_request *req)
{
	struct crypto4xx_ahash_req_ctx *ctx = ahash_request_ctx(req);
        struct crypto4xx_ahash_tfm_ctx *tctx = crypto_tfm_ctx(req->base.tfm);
        struct ahash_request *fr = &ctx->fallback_req;
        int rc;
	
        if (unlikely(req->nbytes == 0)) return crypto4xx_hash_empty_test(req);
        
        if ( (tctx->dev->pdr_head + 1) % PPC4XX_NUM_PD == tctx->dev->pdr_tail) {
            rc = -ENOMEM;
            goto fallback;
        }

        rc = crypto4xx_hash_init(req);
        if (rc) goto fallback;

	ctx->base.hash_final = 1;
	rc = crypto4xx_build_pd(&req->base, &ctx->base, req->src,
				  (struct scatterlist *) req->result,
				  req->nbytes, 0, NULL, 0);
        if (rc == -EINPROGRESS) return rc;
        
        crypto4xx_free_sa(&ctx->base);
        crypto4xx_free_state_record(&ctx->base);

fallback:
        if (!tctx->fallback) return rc;

        ahash_request_set_tfm(fr, tctx->fallback);
        fr->base.flags = req->base.flags & CRYPTO_TFM_REQ_MAY_SLEEP;
        fr->nbytes = req->nbytes;
        fr->src = req->src;
        fr->result = req->result;
        return crypto_ahash_digest(fr);
}

int crypto4xx_hash_hmac_setkey(struct crypto_ahash *hash,
				      const u8 *key,
				      unsigned int keylen,
				      unsigned int max_keylen)
{
	struct crypto_tfm	*tfm = crypto_ahash_tfm(hash);
	struct crypto4xx_ahash_tfm_ctx *ctx = crypto_tfm_ctx(tfm);
	int bs 	= crypto_tfm_alg_blocksize(tfm);
	int ds 	= crypto_ahash_digestsize(hash);
	int rc;
	
        if (keylen > max_keylen) {
		crypto_ahash_set_flags(hash, CRYPTO_TFM_RES_BAD_KEY_LEN);
		return -1;
	}
	rc = crypto4xx_pre_compute_hmac(ctx, (void *)key, keylen, bs, ctx->hash, ds);
	if (rc) {
		printk(KERN_ERR "Hmac Initial Digest Calculation failed\n");
                return rc;
	}
        if (ctx->fallback) {
            rc = crypto_ahash_setkey(ctx->fallback, key, keylen);
            if (rc) {
                    printk(KERN_ERR "Hmac Fallback Initial Digest Calculation failed\n");
                    return rc;
            }
        }
	return 0;
}

/**
 * MD5 Algorithm
 */

int crypto4xx_md5_alg_init(struct crypto_tfm *tfm)
{
	return crypto4xx_hash_alg_init(tfm, SA_HASH128_LEN, SA_HASH_ALG_MD5,
				       SA_HASH_MODE_HASH);
}

int crypto4xx_md5_hmac_init(struct crypto_tfm *tfm)
{
	return crypto4xx_hash_alg_init(tfm, SA_HASH128_LEN, SA_HASH_ALG_MD5,
				       SA_HASH_MODE_HMAC);
}

int crypto4xx_md5_hmac_setkey(struct crypto_ahash *hash, const u8 *key,
			      unsigned int keylen)
{
	return crypto4xx_hash_hmac_setkey(hash, key, keylen, 256);
}

/**
 * SHA1 and SHA2 Algorithm
 *
 */
int crypto4xx_sha1_alg_init(struct crypto_tfm *tfm)
{
	return crypto4xx_hash_alg_init(tfm, SA_HASH160_LEN, SA_HASH_ALG_SHA1,
				       SA_HASH_MODE_HASH);
}

int crypto4xx_sha1_hmac_init(struct crypto_tfm *tfm)
{
	return crypto4xx_hash_alg_init(tfm, SA_HASH160_LEN, SA_HASH_ALG_SHA1,
				       SA_HASH_MODE_HMAC);
}

int crypto4xx_sha1_hmac_setkey(struct crypto_ahash *hash, const u8 *key,
			       unsigned int keylen)
{
	return crypto4xx_hash_hmac_setkey(hash, key, keylen, 256);
}

int crypto4xx_sha2_alg_init(struct crypto_tfm *tfm)
{
	int ds = crypto_ahash_digestsize(__crypto_ahash_cast(tfm));
	u8 ha;

	switch (ds) {
	default:
	case 256/8:
		ha = SA_HASH_ALG_SHA256;
		break;
	case 224/8:
		ha = SA_HASH_ALG_SHA224;
		break;
	case 512/8:
		ha = SA_HASH_ALG_SHA512;
		break;
	case 384/8:
		ha = SA_HASH_ALG_SHA384;
		break;
	}
	BUG_ON(ha >= HASH_ALG_MAX_CNT);

	return crypto4xx_hash_alg_init(tfm,
				       crypto4xx_sa_hash_tbl[2][ha], ha, SA_HASH_MODE_HASH);
}

int crypto4xx_sha2_hmac_init(struct crypto_tfm *tfm)
{
	int ds = crypto_ahash_digestsize(__crypto_ahash_cast(tfm));
	u8 ha;

	switch (ds) {
	default:
	case 256/8:
		ha = SA_HASH_ALG_SHA256;
		break;
	case 224/8:
		ha = SA_HASH_ALG_SHA224;
		break;
	case 512/8:
		ha = SA_HASH_ALG_SHA512;
		break;
	case 384/8:
		ha = SA_HASH_ALG_SHA384;
		break;
	}
	BUG_ON(ha >= HASH_ALG_MAX_CNT);

	return crypto4xx_hash_alg_init(tfm,
				       crypto4xx_sa_hash_tbl[2][ha], ha, SA_HASH_MODE_HMAC);
}

int crypto4xx_sha2_hmac_setkey(struct crypto_ahash *hash,
				      const u8 *key,
				      unsigned int keylen)
{
	int ds = crypto_ahash_digestsize(hash);
	unsigned char ha;

	switch (ds) {
	default:
	case 256/8:
		ha = SA_HASH_ALG_SHA256;
		break;
	case 224/8:
		ha = SA_HASH_ALG_SHA224;
		break;
	case 512/8:
		ha = SA_HASH_ALG_SHA512;
		break;
	case 384/8:
		ha = SA_HASH_ALG_SHA384;
		break;
	}
	BUG_ON(ha >= HASH_ALG_MAX_CNT);

	return crypto4xx_hash_hmac_setkey(hash, key, keylen, 512);
}
#endif

