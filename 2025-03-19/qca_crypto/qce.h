/* Qualcomm Crypto Engine driver API
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


#ifndef __CRYPTO_MSM_QCE_H
#define __CRYPTO_MSM_QCE_H

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/crypto.h>

#include <crypto/algapi.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/aead.h>
#include <crypto/authenc.h>
#include <crypto/scatterwalk.h>

#if 0
#define printx(...) printk(__VA_ARGS__)
#else
#define printx(...) 
#endif

/* SHA digest size  in bytes */
#define SHA256_DIGESTSIZE		32
#define SHA1_DIGESTSIZE			20

#define AES_CE_BLOCK_SIZE		16

/* key size in bytes */
#define HMAC_KEY_SIZE			(SHA1_DIGESTSIZE)    /* hmac-sha1 */
#define SHA_HMAC_KEY_SIZE		64
#define DES_KEY_SIZE			8
#define TRIPLE_DES_KEY_SIZE		24
#define AES128_KEY_SIZE			16
#define AES192_KEY_SIZE			24
#define AES256_KEY_SIZE			32
#define MAX_CIPHER_KEY_SIZE		AES256_KEY_SIZE

/* iv length in bytes */
#define AES_IV_LENGTH			16
#define DES_IV_LENGTH                   8
#define MAX_IV_LENGTH			AES_IV_LENGTH

/* Maximum Nonce bytes  */
#define MAX_NONCE  16

/* Cipher algorithms supported */
enum qce_cipher_alg_enum {
	CIPHER_ALG_DES = 0,
	CIPHER_ALG_3DES = 1,
	CIPHER_ALG_AES = 2,
	CIPHER_ALG_LAST
};

/* Hash and hmac algorithms supported */
enum qce_hash_alg_enum {
	QCE_HASH_SHA1   = 0,
	QCE_HASH_SHA256 = 1,
	QCE_HASH_SHA1_HMAC   = 2,
	QCE_HASH_SHA256_HMAC = 3,
	QCE_HASH_AES_CMAC = 4,
	QCE_HASH_LAST
};

/* Cipher encryption/decryption operations */
enum qce_cipher_dir_enum {
	QCE_ENCRYPT = 0,
	QCE_DECRYPT = 1,
	QCE_CIPHER_DIR_LAST
};

/* Cipher algorithms modes */
enum qce_cipher_mode_enum {
	QCE_MODE_CBC = 0,
	QCE_MODE_ECB = 1,
	QCE_MODE_CTR = 2,
	QCE_MODE_XTS = 3,
	QCE_MODE_CCM = 4,
	QCE_CIPHER_MODE_LAST
};

struct qce_device* qce_open(struct platform_device *pdev, int *rc);
int qce_close(void *handle);
struct qcrypto_cipher_ctx;
struct sps_command_element;
struct qce_device;
int _setup_aead_const(struct qce_device *pce,
                    struct sps_command_element **vaddr,
                    const struct qcrypto_cipher_ctx *cipher_ctx,
                    const uint32_t crypt_offset);
int _setup_aead_dynamic(struct sps_command_element **vaddr,
                            struct crypto_async_request *async_req,
                            const struct qcrypto_cipher_ctx *cipher_ctx,
                            const unsigned char *iv,
                            const uint32_t cryptlen,
                            const uint32_t totallen_in);
struct qcrypto_cipher_req_ctx;
int qce_aead_req(void *handle,
                struct crypto_async_request *async_req,
                struct qcrypto_cipher_req_ctx *cipher_rctx,
                int cryptlen);
int qce_enable_clk(void *handle);
int qce_disable_clk(void *handle);

#endif /* __CRYPTO_MSM_QCE_H */
