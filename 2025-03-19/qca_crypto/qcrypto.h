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

#ifndef _DRIVERS_CRYPTO_MSM_QCRYPTO_H_
#define _DRIVERS_CRYPTO_MSM_QCRYPTO_H_

#include <linux/crypto.h>
#include <crypto/hash.h>
#include "qce.h"
#include "qce50.h"

#define QCRYPTO_CTX_KEY_MASK		0x000000ff
#define QCRYPTO_CTX_USE_HW_KEY		0x00000001
#define QCRYPTO_CTX_USE_PIPE_KEY	0x00000002

#define QCRYPTO_MAX_KEY_SIZE	64
#define QCRYPTO_MAX_IV_LENGTH	16

#define QCRYPTO_DYNAMIC_CMD_LENGTH  (512 + 128)

struct qcrypto_cipher_ctx {
	enum qce_hash_alg_enum  auth_alg;
	enum qce_cipher_alg_enum cipher_alg;
	enum qce_cipher_mode_enum mode;
	unsigned int enc_key_len;
	unsigned int auth_key_len;
        struct crypto_aead *fallback;
	u8 auth_key[QCRYPTO_MAX_KEY_SIZE];
	u8 enc_key[QCRYPTO_MAX_KEY_SIZE];
	int nonce;
};

struct qcrypto_resp_ctx {
	struct crypto_async_request *areq;
	int res;
	int ents;
};

struct qcrypto_cipher_req_ctx {
	struct qcrypto_resp_ctx rsp_entry;
        struct qce_device *pce;
	unsigned char *bounce;
	dma_addr_t phy_bounce;
	dma_addr_t phy_result_in;
	dma_addr_t phy_cmd_in;
	int cmd_size;
	int cmd_size_const;
	int src_nents;
	enum qce_cipher_dir_enum dir;

        char cmd[QCRYPTO_DYNAMIC_CMD_LENGTH]
            __attribute__((aligned(MAX_CE_BAM_BURST_SIZE)));
	struct ce_result_dump_format result;
};

#endif /* _DRIVERS_CRYPTO_MSM_QCRYPTO_H */
