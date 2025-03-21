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
 * This file implements AMCC crypto offload Linux device driver for use with
 * Linux CryptoAPI.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/spinlock_types.h>
#include <linux/random.h>
#include <linux/crypto.h>
#include <crypto/internal/aead.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/highmem.h>
#include <linux/timer.h>
#include <asm/dcr.h>
#include <asm/dcr-regs.h>
#include <asm/cacheflush.h>
#include <crypto/aes.h>
#include <crypto/des.h>
#include <crypto/sha.h>
#include <crypto/ctr.h>
#include <crypto/skcipher.h>
#include "crypto4xx_reg_def.h"
#include "crypto4xx_core.h"
#include "crypto4xx_sa.h"

#define PPC4XX_SEC_VERSION_STR                        "0.5"

/**
 * PPC4xx Crypto Engine Initialization Routine
 */
static struct timer_list crypto4xx_timer;
static void crypto4xx_interrupt_timer(unsigned long data);

static void crypto4xx_hw_init(struct crypto4xx_device *dev)
{
        union ce_ring_size ring_size;
        union ce_ring_contol ring_ctrl;
        union ce_part_ring_size part_ring_size;
        union ce_io_threshold io_threshold;
        u32 rand_num;
        union ce_pe_dma_cfg pe_dma_cfg;

        writel(PPC4XX_BYTE_ORDER, dev->ce_base + CRYPTO4XX_BYTE_ORDER_CFG);
        /* setup pe dma, include reset sg, pdr and pe, then release reset */
        pe_dma_cfg.w = 0;
        pe_dma_cfg.bf.bo_sgpd_en = 1;
        pe_dma_cfg.bf.bo_data_en = 0;
        pe_dma_cfg.bf.bo_sa_en = 1;
        pe_dma_cfg.bf.bo_pd_en = 1;
        pe_dma_cfg.bf.dynamic_sa_en = 1;
        pe_dma_cfg.bf.reset_sg = 1;
        pe_dma_cfg.bf.reset_pdr = 1;
        pe_dma_cfg.bf.reset_pe = 1;
        writel(pe_dma_cfg.w, dev->ce_base + CRYPTO4XX_PE_DMA_CFG);
        /* un reset pe,sg and pdr */
        pe_dma_cfg.bf.pe_mode = 0;
        pe_dma_cfg.bf.reset_sg = 0;
        pe_dma_cfg.bf.reset_pdr = 0;
        pe_dma_cfg.bf.reset_pe = 0;
        pe_dma_cfg.bf.bo_td_en = 0;
        writel(pe_dma_cfg.w, dev->ce_base + CRYPTO4XX_PE_DMA_CFG);
        writel(dev->pdr_pa, dev->ce_base + CRYPTO4XX_PDR_BASE);
        writel(dev->pdr_pa, dev->ce_base + CRYPTO4XX_RDR_BASE);
        writel(PPC4XX_PRNG_CTRL_AUTO_EN, dev->ce_base + CRYPTO4XX_PRNG_CTRL);
        get_random_bytes(&rand_num, sizeof(rand_num));
        writel(rand_num, dev->ce_base + CRYPTO4XX_PRNG_SEED_L);
        get_random_bytes(&rand_num, sizeof(rand_num));
        writel(rand_num, dev->ce_base + CRYPTO4XX_PRNG_SEED_H);
        ring_size.w = 0;
        ring_size.bf.ring_offset = PPC4XX_PD_SIZE;
        ring_size.bf.ring_size   = PPC4XX_NUM_PD;
        writel(ring_size.w, dev->ce_base + CRYPTO4XX_RING_SIZE);
        ring_ctrl.w = 0;
        writel(ring_ctrl.w, dev->ce_base + CRYPTO4XX_RING_CTRL);
        writel(PPC4XX_DC_3DES_EN, dev->ce_base + CRYPTO4XX_DEVICE_CTRL);
        writel(dev->gdr_pa, dev->ce_base + CRYPTO4XX_GATH_RING_BASE);
        writel(dev->sdr_pa, dev->ce_base + CRYPTO4XX_SCAT_RING_BASE);
        part_ring_size.w = 0;
        part_ring_size.bf.sdr_size = PPC4XX_SDR_SIZE;
        part_ring_size.bf.gdr_size = PPC4XX_GDR_SIZE;
        writel(part_ring_size.w, dev->ce_base + CRYPTO4XX_PART_RING_SIZE);
        writel(PPC4XX_SD_BUFFER_SIZE, dev->ce_base + CRYPTO4XX_PART_RING_CFG);
        io_threshold.w = 0;
        io_threshold.bf.output_threshold = PPC4XX_OUTPUT_THRESHOLD;
        io_threshold.bf.input_threshold  = PPC4XX_INPUT_THRESHOLD;
        writel(io_threshold.w, dev->ce_base + CRYPTO4XX_IO_THRESHOLD);
        writel(0, dev->ce_base + CRYPTO4XX_PDR_BASE_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_RDR_BASE_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_PKT_SRC_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_PKT_DEST_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_SA_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_GATH_RING_BASE_UADDR);
        writel(0, dev->ce_base + CRYPTO4XX_SCAT_RING_BASE_UADDR);
        /* un reset pe,sg and pdr */
        pe_dma_cfg.bf.pe_mode = 1;
        pe_dma_cfg.bf.reset_sg = 0;
        pe_dma_cfg.bf.reset_pdr = 0;
        pe_dma_cfg.bf.reset_pe = 0;
        pe_dma_cfg.bf.bo_td_en = 0;
        writel(pe_dma_cfg.w, dev->ce_base + CRYPTO4XX_PE_DMA_CFG);
        /*clear all pending interrupt*/
        writel(PPC4XX_INTERRUPT_CLR, dev->ce_base + CRYPTO4XX_INT_MASK_STAT);
        // XXX - trigger interrupt on each descriptor
        // should make some inteligent adjuster if this can improve performance
        writel(PPC4XX_INT_DESCR_CNT, dev->ce_base + CRYPTO4XX_INT_DESCR_CNT);
        writel(PPC4XX_INT_CFG, dev->ce_base + CRYPTO4XX_INT_CFG);
        writel(PPC4XX_PD_DONE_INT, dev->ce_base + CRYPTO4XX_INT_EN);
}

int crypto4xx_alloc_sa(struct crypto4xx_ctx_base *ctx, u32 size)
{
        const u32 bsize = size * 4;
        ctx->sa_in = dma_alloc_coherent(ctx->dev->core_dev->device, bsize * 2,
                                        &ctx->sa_in_dma_addr, GFP_ATOMIC);
        if (ctx->sa_in == NULL)
                return -ENOMEM;

        ctx->sa_out = ctx->sa_in + bsize;
        ctx->sa_out_dma_addr = ctx->sa_in_dma_addr + bsize;

        memset(ctx->sa_in, 0, bsize * 2);
        ctx->sa_len = size;

        return 0;
}

void crypto4xx_free_sa(struct crypto4xx_ctx_base *ctx)
{
        if (ctx->sa_in != NULL)
                dma_free_coherent(ctx->dev->core_dev->device, ctx->sa_len * 4 * 2,
                                  ctx->sa_in, ctx->sa_in_dma_addr);

        ctx->sa_in_dma_addr = 0;
        ctx->sa_out_dma_addr = 0;
        ctx->sa_in = NULL;
        ctx->sa_out = NULL;
        ctx->sa_len = 0;
}

u32 crypto4xx_alloc_state_record(struct crypto4xx_ctx_base *ctx)
{
        ctx->state_record = dma_alloc_coherent(ctx->dev->core_dev->device,
                                sizeof(struct sa_state_record),
                                &ctx->state_record_dma_addr, GFP_ATOMIC);
        if (!ctx->state_record)
                return -ENOMEM;
        memset(ctx->state_record, 0, sizeof(struct sa_state_record));

        return 0;
}

void crypto4xx_free_state_record(struct crypto4xx_ctx_base *ctx)
{
        if (ctx->state_record != NULL)
                dma_free_coherent(ctx->dev->core_dev->device,
                                  sizeof(struct sa_state_record),
                                  ctx->state_record,
                                  ctx->state_record_dma_addr);
        ctx->state_record = NULL;
        ctx->state_record_dma_addr = 0;
}

u32 crypto4xx_alloc_arc4_state_record(struct crypto4xx_cipher_tfm_ctx *tctx)
{
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        tctx->arc4_state_record = dma_alloc_coherent(ctx->dev->core_dev->device,
                        sizeof(struct arc4_sr),
                        &tctx->arc4_state_record_dma_addr,
                        GFP_ATOMIC);
        if (!tctx->arc4_state_record)
                return -ENOMEM;

        memset(tctx->arc4_state_record, 0, sizeof(struct arc4_sr));

        return 0;
}

void crypto4xx_free_arc4_state_record(struct crypto4xx_cipher_tfm_ctx *tctx)
{
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        if (tctx->arc4_state_record != NULL) {
                dma_free_coherent(ctx->dev->core_dev->device,
                                  sizeof(struct arc4_sr),
                                  tctx->arc4_state_record,
                                  tctx->arc4_state_record_dma_addr);
        }
        tctx->arc4_state_record_dma_addr = 0;
}

/**
 * alloc memory for the gather ring
 * no need to alloc buf for the ring
 * gdr_tail, gdr_head and gdr_count are initialized by this function
 */
static u32 crypto4xx_build_pdr(struct crypto4xx_device *dev)
{
        int i;
        struct pd_uinfo *pd_uinfo;
        dev->pdr = dma_alloc_coherent(dev->core_dev->device,
                                      sizeof(struct ce_pd) * PPC4XX_NUM_PD,
                                      &dev->pdr_pa, GFP_ATOMIC);
        if (!dev->pdr)
                return -ENOMEM;

        dev->pdr_uinfo = kzalloc(sizeof(struct pd_uinfo) * PPC4XX_NUM_PD,
                                GFP_KERNEL);
        if (!dev->pdr_uinfo) {
                dma_free_coherent(dev->core_dev->device,
                                  sizeof(struct ce_pd) * PPC4XX_NUM_PD,
                                  dev->pdr,
                                  dev->pdr_pa);
                return -ENOMEM;
        }
        memset(dev->pdr, 0,  sizeof(struct ce_pd) * PPC4XX_NUM_PD);
        dev->shadow_sa_pool = dma_alloc_coherent(dev->core_dev->device,
                                   256 * PPC4XX_NUM_PD,
                                   &dev->shadow_sa_pool_pa,
                                   GFP_ATOMIC);
        if (!dev->shadow_sa_pool)
                return -ENOMEM;

        dev->shadow_sr_pool = dma_alloc_coherent(dev->core_dev->device,
                         sizeof(struct sa_state_record) * PPC4XX_NUM_PD,
                         &dev->shadow_sr_pool_pa, GFP_ATOMIC);
        if (!dev->shadow_sr_pool)
                return -ENOMEM;
        for (i = 0; i < PPC4XX_NUM_PD; i++) {
                pd_uinfo = (struct pd_uinfo *) (dev->pdr_uinfo +
                                                sizeof(struct pd_uinfo) * i);

                /* alloc 256 bytes which is enough for any kind of dynamic sa */
                pd_uinfo->sa_va = dev->shadow_sa_pool + 256 * i;
                pd_uinfo->sa_pa = dev->shadow_sa_pool_pa + 256 * i;

                /* alloc state record */
                pd_uinfo->sr_va = dev->shadow_sr_pool +
                    sizeof(struct sa_state_record) * i;
                pd_uinfo->sr_pa = dev->shadow_sr_pool_pa +
                    sizeof(struct sa_state_record) * i;
        }

        return 0;
}

static void crypto4xx_destroy_pdr(struct crypto4xx_device *dev)
{
        if (dev->pdr != NULL)
                dma_free_coherent(dev->core_dev->device,
                                  sizeof(struct ce_pd) * PPC4XX_NUM_PD,
                                  dev->pdr, dev->pdr_pa);
        if (dev->shadow_sa_pool)
                dma_free_coherent(dev->core_dev->device, 256 * PPC4XX_NUM_PD,
                                  dev->shadow_sa_pool, dev->shadow_sa_pool_pa);
        if (dev->shadow_sr_pool)
                dma_free_coherent(dev->core_dev->device,
                        sizeof(struct sa_state_record) * PPC4XX_NUM_PD,
                        dev->shadow_sr_pool, dev->shadow_sr_pool_pa);

        kfree(dev->pdr_uinfo);
}

static u32 crypto4xx_get_pd_from_pdr_nolock(struct crypto4xx_device *dev)
{
        u32 retval;
        u32 tmp;

        retval = dev->pdr_head;
        tmp = (dev->pdr_head + 1) % PPC4XX_NUM_PD;

        if (tmp == dev->pdr_tail)
                return ERING_WAS_FULL;

        dev->pdr_head = tmp;

        return retval;
}

static u32 crypto4xx_put_pd_to_pdr(struct crypto4xx_device *dev, u32 idx)
{
        struct pd_uinfo *pd_uinfo;
        unsigned long flags;

        pd_uinfo = (struct pd_uinfo *)(dev->pdr_uinfo +
                                       sizeof(struct pd_uinfo) * idx);
        spin_lock_irqsave(&dev->core_dev->lock, flags);
        if (dev->pdr_tail != PPC4XX_LAST_PD)
                dev->pdr_tail++;
        else
                dev->pdr_tail = 0;
        pd_uinfo->state = PD_ENTRY_FREE;
        spin_unlock_irqrestore(&dev->core_dev->lock, flags);

        return 0;
}

static struct ce_pd *crypto4xx_get_pdp(struct crypto4xx_device *dev,
                                       dma_addr_t *pd_dma, u32 idx)
{
        *pd_dma = dev->pdr_pa + sizeof(struct ce_pd) * idx;

        return dev->pdr + sizeof(struct ce_pd) * idx;
}

/**
 * alloc memory for the gather ring
 * no need to alloc buf for the ring
 * gdr_tail, gdr_head and gdr_count are initialized by this function
 */
static u32 crypto4xx_build_gdr(struct crypto4xx_device *dev)
{
        dev->gdr = dma_alloc_coherent(dev->core_dev->device,
                                      sizeof(struct ce_gd) * PPC4XX_NUM_GD,
                                      &dev->gdr_pa, GFP_ATOMIC);
        if (!dev->gdr)
                return -ENOMEM;

        memset(dev->gdr, 0, sizeof(struct ce_gd) * PPC4XX_NUM_GD);

        return 0;
}

static inline void crypto4xx_destroy_gdr(struct crypto4xx_device *dev)
{
        dma_free_coherent(dev->core_dev->device,
                          sizeof(struct ce_gd) * PPC4XX_NUM_GD,
                          dev->gdr, dev->gdr_pa);
}

/*
 * when this function is called.
 * preemption or interrupt must be disabled
 */
u32 crypto4xx_get_n_gd(struct crypto4xx_device *dev, int n)
{
        u32 retval;
        u32 tmp;
        if (n >= PPC4XX_NUM_GD)
                return ERING_WAS_FULL;

        retval = dev->gdr_head;
        tmp = (dev->gdr_head + n) % PPC4XX_NUM_GD;
        if (dev->gdr_head > dev->gdr_tail) {
                if (tmp < dev->gdr_head && tmp >= dev->gdr_tail)
                        return ERING_WAS_FULL;
        } else if (dev->gdr_head < dev->gdr_tail) {
                if (tmp < dev->gdr_head || tmp >= dev->gdr_tail)
                        return ERING_WAS_FULL;
        }
        dev->gdr_head = tmp;

        return retval;
}

static u32 crypto4xx_put_gd_to_gdr(struct crypto4xx_device *dev)
{
        unsigned long flags;

        spin_lock_irqsave(&dev->core_dev->lock, flags);
        if (dev->gdr_tail == dev->gdr_head) {
                spin_unlock_irqrestore(&dev->core_dev->lock, flags);
                return 0;
        }

        if (dev->gdr_tail != PPC4XX_LAST_GD)
                dev->gdr_tail++;
        else
                dev->gdr_tail = 0;

        spin_unlock_irqrestore(&dev->core_dev->lock, flags);

        return 0;
}

static inline struct ce_gd *crypto4xx_get_gdp(struct crypto4xx_device *dev,
                                              dma_addr_t *gd_dma, u32 idx)
{
        *gd_dma = dev->gdr_pa + sizeof(struct ce_gd) * idx;

        return (struct ce_gd *) (dev->gdr + sizeof(struct ce_gd) * idx);
}

/**
 * alloc memory for the scatter ring
 * need to alloc buf for the ring
 * sdr_tail, sdr_head and sdr_count are initialized by this function
 */
static u32 crypto4xx_build_sdr(struct crypto4xx_device *dev)
{
        int i;
        struct ce_sd *sd_array;

        /* alloc memory for scatter descriptor ring */
        dev->sdr = dma_alloc_coherent(dev->core_dev->device,
                                      sizeof(struct ce_sd) * PPC4XX_NUM_SD,
                                      &dev->sdr_pa, GFP_ATOMIC);
        if (!dev->sdr)
                return -ENOMEM;

        dev->scatter_buffer_size = PPC4XX_SD_BUFFER_SIZE;
        dev->scatter_buffer_va =
                dma_alloc_coherent(dev->core_dev->device,
                        dev->scatter_buffer_size * PPC4XX_NUM_SD,
                        &dev->scatter_buffer_pa, GFP_ATOMIC);
        if (!dev->scatter_buffer_va) {
                dma_free_coherent(dev->core_dev->device,
                                  sizeof(struct ce_sd) * PPC4XX_NUM_SD,
                                  dev->sdr, dev->sdr_pa);
                return -ENOMEM;
        }

        sd_array = dev->sdr;

        for (i = 0; i < PPC4XX_NUM_SD; i++) {
                sd_array[i].ptr = dev->scatter_buffer_pa +
                                  dev->scatter_buffer_size * i;
        }

        return 0;
}

static void crypto4xx_destroy_sdr(struct crypto4xx_device *dev)
{
        if (dev->sdr != NULL)
                dma_free_coherent(dev->core_dev->device,
                                  sizeof(struct ce_sd) * PPC4XX_NUM_SD,
                                  dev->sdr, dev->sdr_pa);

        if (dev->scatter_buffer_va != NULL)
                dma_free_coherent(dev->core_dev->device,
                                  dev->scatter_buffer_size * PPC4XX_NUM_SD,
                                  dev->scatter_buffer_va,
                                  dev->scatter_buffer_pa);
}

/*
 * when this function is called.
 * preemption or interrupt must be disabled
 */
static u32 crypto4xx_get_n_sd(struct crypto4xx_device *dev, int n)
{
        u32 retval;
        u32 tmp;

        if (n >= PPC4XX_NUM_SD)
                return ERING_WAS_FULL;

        retval = dev->sdr_head;
        tmp = (dev->sdr_head + n) % PPC4XX_NUM_SD;
        if (dev->sdr_head > dev->gdr_tail) {
                if (tmp < dev->sdr_head && tmp >= dev->sdr_tail)
                        return ERING_WAS_FULL;
        } else if (dev->sdr_head < dev->sdr_tail) {
                if (tmp < dev->sdr_head || tmp >= dev->sdr_tail)
                        return ERING_WAS_FULL;
        } /* the head = tail, or empty case is already take cared */
        dev->sdr_head = tmp;

        return retval;
}

static u32 crypto4xx_put_sd_to_sdr(struct crypto4xx_device *dev)
{
        unsigned long flags;

        spin_lock_irqsave(&dev->core_dev->lock, flags);
        if (dev->sdr_tail == dev->sdr_head) {
                spin_unlock_irqrestore(&dev->core_dev->lock, flags);
                return 0;
        }
        if (dev->sdr_tail != PPC4XX_LAST_SD)
                dev->sdr_tail++;
        else
                dev->sdr_tail = 0;
        spin_unlock_irqrestore(&dev->core_dev->lock, flags);

        return 0;
}

static inline struct ce_sd *crypto4xx_get_sdp(struct crypto4xx_device *dev,
                                              dma_addr_t *sd_dma, u32 idx)
{
        *sd_dma = dev->sdr_pa + sizeof(struct ce_sd) * idx;

        return  (struct ce_sd *)(dev->sdr + sizeof(struct ce_sd) * idx);
}

static u32 crypto4xx_fill_one_page(struct crypto4xx_device *dev,
                                   dma_addr_t *addr, u32 *length,
                                   u32 *idx, u32 *offset, u32 *nbytes)
{
        u32 len;

        if (*length > dev->scatter_buffer_size) {
                memcpy(phys_to_virt(*addr),
                        dev->scatter_buffer_va +
                        *idx * dev->scatter_buffer_size + *offset,
                        dev->scatter_buffer_size);
                *offset = 0;
                *length -= dev->scatter_buffer_size;
                *nbytes -= dev->scatter_buffer_size;
                if (*idx == PPC4XX_LAST_SD)
                        *idx = 0;
                else
                        (*idx)++;
                *addr = *addr +  dev->scatter_buffer_size;
                return 1;
        } else if (*length < dev->scatter_buffer_size) {
                memcpy(phys_to_virt(*addr),
                        dev->scatter_buffer_va +
                        *idx * dev->scatter_buffer_size + *offset, *length);
                if ((*offset + *length) == dev->scatter_buffer_size) {
                        if (*idx == PPC4XX_LAST_SD)
                                *idx = 0;
                        else
                                (*idx)++;
                        *nbytes -= *length;
                        *offset = 0;
                } else {
                        *nbytes -= *length;
                        *offset += *length;
                }

                return 0;
        } else {
                len = (*nbytes <= dev->scatter_buffer_size) ?
                                (*nbytes) : dev->scatter_buffer_size;
                memcpy(phys_to_virt(*addr),
                        dev->scatter_buffer_va +
                        *idx * dev->scatter_buffer_size + *offset,
                        len);
                *offset = 0;
                *nbytes -= len;

                if (*idx == PPC4XX_LAST_SD)
                        *idx = 0;
                else
                        (*idx)++;

                return 0;
        }
}

static void crypto4xx_copy_pkt_to_dst(struct crypto4xx_device *dev,
                                      struct ce_pd *pd,
                                      struct pd_uinfo *pd_uinfo,
                                      u32 nbytes,
                                      struct scatterlist *dst)
{
        dma_addr_t addr;
        u32 this_sd;
        u32 offset;
        u32 len;
        u32 sg_len;
        struct scatterlist *sg = dst;

        this_sd = pd_uinfo->first_sd;
        offset = 0;
        while (nbytes) {
                sg_len = sg->length;

                addr = dma_map_page(dev->core_dev->device, sg_page(sg),
                                sg->offset, sg->length, DMA_TO_DEVICE);

                if (offset == 0) {
                        len = (nbytes <= sg->length) ? nbytes : sg->length;
                        while (crypto4xx_fill_one_page(dev, &addr, &len,
                                &this_sd, &offset, &nbytes))
                                ;
                        if (!nbytes)
                                return;
                } else {
                        len = (nbytes <= (dev->scatter_buffer_size - offset)) ?
                                nbytes : (dev->scatter_buffer_size - offset);
                        len = (sg->length < len) ? sg->length : len;
                        while (crypto4xx_fill_one_page(dev, &addr, &len,
                                               &this_sd, &offset, &nbytes))
                                ;
                        if (!nbytes)
                                return;
                        sg_len -= len;
                        if (sg_len) {
                                addr += len;
                                while (crypto4xx_fill_one_page(dev, &addr,
                                        &sg_len, &this_sd, &offset, &nbytes))
                                        ;
                        }
                }
                dma_unmap_page(dev->core_dev->device, addr, sg->length, DMA_TO_DEVICE);
                if (sg_is_last(sg)) {
                    WARN(nbytes, "crypto4xx: %d byte shortage on dst buffer!", nbytes);
                    break;
                }
                sg = sg_next(sg);
        }
}

static u32 crypto4xx_copy_digest_to_dst(struct pd_uinfo *pd_uinfo,
                                        struct crypto4xx_ctx_base *ctx)
{
#if _4XX_ENABLE_HASH
        struct dynamic_sa_ctl *sa = (struct dynamic_sa_ctl *) ctx->sa_in;
        struct sa_state_record *state_record =
                                (struct sa_state_record *) pd_uinfo->sr_va;

        switch (sa->sa_command_0.bf.hash_alg) {
        case SA_HASH_ALG_KASUMI_f9:
                crypto4xx_memcpy_le((void *)pd_uinfo->dest_va,
                                     (u8 *)state_record->save_digest, 8);
                break;
        case SA_HASH_ALG_AES_XCBC_MAC_128:
                crypto4xx_memcpy_le((void *)pd_uinfo->dest_va,
                                     (u8 *) state_record->save_digest, 16);
                break;
        case SA_HASH_ALG_MD5:
                crypto4xx_memcpy_le((void *)pd_uinfo->dest_va,
                                     (u8 *) state_record->save_digest,
                                     SA_HASH_ALG_MD5_DIGEST_SIZE);
                break;
        default:
                memcpy((void *)pd_uinfo->dest_va,
                        state_record->save_digest,
                        crypto4xx_sa_hash_tbl[1][sa->sa_command_0.bf.hash_alg]);
                break;
        }
#endif

        return 0;
}

static void crypto4xx_ret_sg_desc(struct crypto4xx_device *dev,
                                  struct pd_uinfo *pd_uinfo)
{
        int i;
        struct ce_gd *gd;
        if (pd_uinfo->num_gd) {
                for (i = 0; i < pd_uinfo->num_gd; i++) {
                    gd = (struct ce_gd *) (dev->gdr + sizeof(struct ce_gd) * pd_uinfo->first_gd);
                    dma_unmap_page(dev->core_dev->device,
                                    gd->ptr,
                                    gd->ctl_len.len,
                                    DMA_TO_DEVICE);
                        crypto4xx_put_gd_to_gdr(dev);
                    ++pd_uinfo->first_gd;
                }
                pd_uinfo->first_gd = 0xffffffff;
                pd_uinfo->num_gd = 0;
        }
        if (pd_uinfo->num_sd) {
                for (i = 0; i < pd_uinfo->num_sd; i++)
                        crypto4xx_put_sd_to_sdr(dev);

                pd_uinfo->first_sd = 0xffffffff;
                pd_uinfo->num_sd = 0;
        }
}

void crypto4xx_append_icv_to_end(struct crypto4xx_device *dev,
                                 struct scatterlist *dst,
                                 struct sa_state_record *sr,
                                 u32 offset,
                                 u32 len)
{
        struct scatterlist *sg;
        int i = 0;
        u32 cp_len;
        dma_addr_t addr;

        sg = &dst[i];
        while (len) {
                while (sg->length < offset) {
                        offset -= sg->length;
                        i++;
                        sg = &sg[i];
                }
                /* at here, icv could be in this sg,
                * or icv could be in the next sg
                */
                if (sg->length > offset) {
                        /* icv should be in middle of this sg */
                        addr = dma_map_page(dev->core_dev->device, sg_page(sg),
                                            sg->offset,
                                            sg->length, DMA_TO_DEVICE);
                        cp_len = (sg->length-offset >= len) ? len :
                                        sg->length-offset;
                        len -= cp_len;
                        crypto4xx_memcpy_le((u32 *)(phys_to_virt(addr)
                                        + offset),
                                        (u8 *)sr->save_digest, cp_len);
                        dma_unmap_page(dev->core_dev->device, addr,
                                        sg->length, DMA_TO_DEVICE);
                } else {
                        /* start from begin of next sg*/
                        i++;
                        sg = &sg[i];
                        offset = 0;
                        addr = dma_map_page(dev->core_dev->device, sg_page(sg),
                                            sg->offset,
                                            sg->length, DMA_FROM_DEVICE);
                        cp_len = (sg->length >= len) ? len : sg->length;
                        len -= cp_len;
                        crypto4xx_memcpy_le((u32 *) (phys_to_virt(addr)
                                        + offset),
                                        (u8 *) sr->save_digest, cp_len);
                        dma_unmap_page(dev->core_dev->device, addr,
                                        sg->length, DMA_FROM_DEVICE);
                }
                i++;
                sg = &sg[i];
        }
}

static u32 crypto4xx_ablkcipher_done(struct crypto4xx_device *dev,
                                     struct pd_uinfo *pd_uinfo,
                                     struct ce_pd *pd)
{
        struct crypto4xx_cipher_tfm_ctx *ctx;
        struct ablkcipher_request *ablk_req;
        struct scatterlist *dst;
        dma_addr_t addr;

        ablk_req = ablkcipher_request_cast(pd_uinfo->async_req);
        ctx  = crypto_tfm_ctx(ablk_req->base.tfm);

        if (pd_uinfo->using_sd) {
                crypto4xx_copy_pkt_to_dst(dev, pd, pd_uinfo, ablk_req->nbytes,
                                          ablk_req->dst);
        } else {
                dst = pd_uinfo->dest_va;
                addr = dma_map_page(dev->core_dev->device, sg_page(dst),
                                    dst->offset, dst->length, DMA_FROM_DEVICE);
        }
        crypto4xx_ret_sg_desc(dev, pd_uinfo);
        if (ablk_req->base.complete != NULL)
                ablk_req->base.complete(&ablk_req->base, 0);

        if (!pd_uinfo->using_sd)
            dma_unmap_page(dev->core_dev->device, addr, dst->length, DMA_FROM_DEVICE);

        return 0;
}

static u32 crypto4xx_ahash_done(struct crypto4xx_device *dev,
                                struct pd_uinfo *pd_uinfo)
{
	struct crypto4xx_ahash_req_ctx *ctx;
        struct ahash_request *ahash_req;
        ahash_req = ahash_request_cast(pd_uinfo->async_req);
	ctx = ahash_request_ctx(ahash_req);

        crypto4xx_copy_digest_to_dst(pd_uinfo, &ctx->base);
        crypto4xx_ret_sg_desc(dev, pd_uinfo);
        
        crypto4xx_free_sa(&ctx->base);
        crypto4xx_free_state_record(&ctx->base);
        
        /* call user provided callback function x */
        if (ahash_req->base.complete != NULL)
                ahash_req->base.complete(&ahash_req->base, 0);

        return 0;
}

static u32 crypto4xx_aead_done(struct crypto4xx_device *dev,
                        struct pd_uinfo *pd_uinfo,
                        struct ce_pd *pd)
{
        struct aead_request *aead_req;
        struct crypto4xx_cipher_tfm_ctx *ctx;
        struct scatterlist *dst;
        dma_addr_t addr;
        struct crypto_aead *aead;

        aead_req = container_of(pd_uinfo->async_req,
                                struct aead_request, base);
        aead = crypto_aead_reqtfm(aead_req);
        ctx  = crypto_tfm_ctx(aead_req->base.tfm);

        if (pd_uinfo->using_sd) {
                crypto4xx_copy_pkt_to_dst(dev, pd, pd_uinfo,
                                          pd->pd_ctl_len.bf.pkt_len,
                                          aead_req->dst);
        } else {
                dst = pd_uinfo->dest_va;
                addr = dma_map_page(dev->core_dev->device, sg_page(dst),
                                    dst->offset,
                                    dst->length, DMA_FROM_DEVICE);
        }

        if (ctx->append_icv != 0) {
                dst = pd_uinfo->dest_va;
                crypto4xx_append_icv_to_end(dev, dst,
                                            (struct sa_state_record *)
                                            pd_uinfo->sr_va,
                                            aead_req->cryptlen,
                                            crypto_aead_authsize(aead));
        }
        crypto4xx_ret_sg_desc(dev, pd_uinfo);
        /* call user provided callback function x */
        if (aead_req->base.complete != NULL)
                aead_req->base.complete(&aead_req->base, 0);

        if (!pd_uinfo->using_sd)
            dma_unmap_page(dev->core_dev->device, addr, dst->length, DMA_FROM_DEVICE);
        return 0;
}

u32 crypto4xx_pd_done(struct crypto4xx_device *dev, u32 idx)
{

        struct ce_pd *pd;
        struct pd_uinfo *pd_uinfo;

        pd =  dev->pdr + sizeof(struct ce_pd)*idx;
        pd_uinfo = dev->pdr_uinfo + sizeof(struct pd_uinfo)*idx;

        if (crypto_tfm_alg_type(pd_uinfo->async_req->tfm) ==
                        CRYPTO_ALG_TYPE_AEAD)
                return crypto4xx_aead_done(dev, pd_uinfo, pd);
        else if (crypto_tfm_alg_type(pd_uinfo->async_req->tfm) ==
                        CRYPTO_ALG_TYPE_ABLKCIPHER)
                return crypto4xx_ablkcipher_done(dev, pd_uinfo, pd);
        else if (crypto_tfm_alg_type(pd_uinfo->async_req->tfm) ==
                        CRYPTO_ALG_TYPE_AHASH)
                return crypto4xx_ahash_done(dev, pd_uinfo);
        return 0;
}

/**
 * Note: Only use this function to copy items that is word aligned.
 */
void crypto4xx_memcpy_le(unsigned int *dst,
                         const unsigned char *buf,
                         int len)
{
        u8 *tmp;
        for (; len >= 4; buf += 4, len -= 4)
                *dst++ = cpu_to_le32(*(unsigned int *) buf);

        tmp = (u8 *)dst;
        switch (len) {
        case 3:
                *tmp++ = 0;
                *tmp++ = *(buf+2);
                *tmp++ = *(buf+1);
                *tmp++ = *buf;
                break;
        case 2:
                *tmp++ = 0;
                *tmp++ = 0;
                *tmp++ = *(buf+1);
                *tmp++ = *buf;
                break;
        case 1:
                *tmp++ = 0;
                *tmp++ = 0;
                *tmp++ = 0;
                *tmp++ = *buf;
                break;
        default:
                break;
        }
}

static void crypto4xx_stop_all(struct crypto4xx_core_device *core_dev)
{
        crypto4xx_destroy_pdr(core_dev->dev);
        crypto4xx_destroy_gdr(core_dev->dev);
        crypto4xx_destroy_sdr(core_dev->dev);
        dev_set_drvdata(core_dev->device, NULL);
        iounmap(core_dev->dev->ce_base);
        kfree(core_dev->dev);
        kfree(core_dev);
}

#if 0
void crypto4xx_return_pd(struct crypto4xx_device *dev,
                         u32 pd_entry, struct ce_pd *pd,
                         struct pd_uinfo *pd_uinfo)
{
        /* irq should be already disabled */
        dev->pdr_head = pd_entry;
        pd->pd_ctl.w = 0;
        pd->pd_ctl_len.w = 0;
        pd_uinfo->state = PD_ENTRY_FREE;
}
#endif

/*
 * derive number of elements in scatterlist
 * Shamlessly copy from talitos.c
 */
static int get_sg_count(struct scatterlist *sg_list, int nbytes)
{
        struct scatterlist *sg = sg_list;
        int sg_nents = 0;

        while (nbytes) {
                sg_nents++;
                if (sg->length > nbytes)
                        break;
                nbytes -= sg->length;
                sg = sg_next(sg);
        }

        return sg_nents;
}

static u32 get_next_gd(u32 current)
{
        if (current != PPC4XX_LAST_GD)
                return current + 1;
        else
                return 0;
}

static u32 get_next_sd(u32 current)
{
        if (current != PPC4XX_LAST_SD)
                return current + 1;
        else
                return 0;
}

u32 crypto4xx_build_pd(struct crypto_async_request *req,
                       struct crypto4xx_ctx_base *ctx,
                       struct scatterlist *src,
                       struct scatterlist *dst,
                       unsigned int datalen,
                       u32 aad_len,
                       void *iv, u32 iv_len)
{
        struct crypto4xx_device *dev = ctx->dev;
        dma_addr_t addr, pd_dma, sd_dma, gd_dma;
        struct dynamic_sa_ctl *sa;
        struct scatterlist *sg;
        struct ce_gd *gd;
        struct ce_pd *pd;
        u32 num_gd, num_sd;
        u32 fst_gd = 0xffffffff;
        u32 fst_sd = 0xffffffff;
        u32 pd_entry;
        unsigned long flags;
        struct pd_uinfo *pd_uinfo = NULL;
        unsigned int nbytes = datalen, idx;
        u32 gd_idx = 0;
        unsigned int aadlen = 0;

        /* figure how many gd is needed */
                num_gd = get_sg_count(src, datalen);
                if (num_gd == 1)
                        num_gd = 0;

        /* figure how many sd is needed */
        if (sg_is_last(dst) || ctx->is_hash) {
                num_sd = 0;
        } else {
                if (datalen > PPC4XX_SD_BUFFER_SIZE) {
                        num_sd = datalen / PPC4XX_SD_BUFFER_SIZE;
                        if (datalen % PPC4XX_SD_BUFFER_SIZE)
                                num_sd++;
                } else {
                        num_sd = 1;
                }
        }

        /*
         * The follow section of code needs to be protected
         * The gather ring and scatter ring needs to be consecutive
         * In case of run out of any kind of descriptor, the descriptor
         * already got must be return the original place.
         */
        spin_lock_irqsave(&dev->core_dev->lock, flags);
        if (num_gd) {
                fst_gd = crypto4xx_get_n_gd(dev, num_gd);
                if (fst_gd == ERING_WAS_FULL) {
                        spin_unlock_irqrestore(&dev->core_dev->lock, flags);
                        return -EAGAIN;
                }
        }
        if (num_sd) {
                fst_sd = crypto4xx_get_n_sd(dev, num_sd);
                if (fst_sd == ERING_WAS_FULL) {
                        if (num_gd)
                                dev->gdr_head = fst_gd;
                        spin_unlock_irqrestore(&dev->core_dev->lock, flags);
                        return -EAGAIN;
                }
        }
        pd_entry = crypto4xx_get_pd_from_pdr_nolock(dev);
        if (pd_entry == ERING_WAS_FULL) {
                if (num_gd)
                        dev->gdr_head = fst_gd;
                if (num_sd)
                        dev->sdr_head = fst_sd;
                spin_unlock_irqrestore(&dev->core_dev->lock, flags);
                return -EAGAIN;
        }
        spin_unlock_irqrestore(&dev->core_dev->lock, flags);

        pd_uinfo = (struct pd_uinfo *)(dev->pdr_uinfo +
                                       sizeof(struct pd_uinfo) * pd_entry);
        pd_uinfo->async_req = req;
        pd_uinfo->num_gd = num_gd;
        pd_uinfo->num_sd = num_sd;
        pd_uinfo->state = PD_ENTRY_INUSE;

        pd = crypto4xx_get_pdp(dev, &pd_dma, pd_entry);
        if (iv_len || ctx->is_hash) {
                pd->sa = pd_uinfo->sa_pa;
                sa = (struct dynamic_sa_ctl *) pd_uinfo->sa_va;
                if (ctx->direction == DIR_INBOUND)
                        memcpy(sa, ctx->sa_in, ctx->sa_len * 4);
                else
                        memcpy(sa, ctx->sa_out, ctx->sa_len * 4);
                memcpy((void *) sa + ctx->offset_to_sr_ptr,
                        &pd_uinfo->sr_pa, 4);

                if (iv_len) {
                        if (ctx->ctr_aes) {
                                /* First the nonce */
                                memcpy(pd_uinfo->sr_va, ctx->state_record,
                                       CTR_RFC3686_NONCE_SIZE);
                                /* Copy the IV that is passed through
                                 * each operation
                                 */
                                crypto4xx_memcpy_le(pd_uinfo->sr_va +
                                        CTR_RFC3686_NONCE_SIZE, iv, iv_len);
                        } else
                                crypto4xx_memcpy_le(pd_uinfo->sr_va,
                                                iv, iv_len);
                }
                if (ctx->is_gcm || ctx->ctr_aes) {
                        u32 seq = 1;
                        /*For GCM and CTR(AES) algs adding the counter value*/
                        crypto4xx_memcpy_le(pd_uinfo->sr_va + 12,
                                            (void *)&seq,  4);
                }
        } else {
                if (ctx->direction == DIR_INBOUND) {
                        pd->sa = ctx->sa_in_dma_addr;
                        sa = (struct dynamic_sa_ctl *) ctx->sa_in;
                } else {
                        pd->sa = ctx->sa_out_dma_addr;
                        sa = (struct dynamic_sa_ctl *) ctx->sa_out;
                }
        }
        pd->sa_len = ctx->sa_len;
        if (num_gd) {
                /* get first gd we are going to use */
                gd_idx = fst_gd;
                pd_uinfo->first_gd = fst_gd;
                pd_uinfo->num_gd = num_gd;
                gd = crypto4xx_get_gdp(dev, &gd_dma, gd_idx);
                pd->src = gd_dma;
                /* enable gather */
                sa->sa_command_0.bf.gather = 1;
                idx = 0;
                /* walk the sg, and setup gather array */
                sg = src;
                while (nbytes) {
                        addr = dma_map_page(dev->core_dev->device, sg_page(sg),
                                    sg->offset, sg->length, DMA_TO_DEVICE);

                        gd->ptr = addr;
                        gd->ctl_len.len = sg->length;
                        gd->ctl_len.done = 0;
                        gd->ctl_len.ready = 1;
                        if (sg->length >= nbytes)
                                break;
                        nbytes -= sg->length;
                        gd_idx = get_next_gd(gd_idx);
                        gd = crypto4xx_get_gdp(dev, &gd_dma, gd_idx);
                        idx++;
                        sg = sg_next(sg);
                }
        } else {
// TODO - not unmapped
                pd->src = (u32)dma_map_page(dev->core_dev->device, sg_page(src),
                                src->offset, src->length, DMA_TO_DEVICE);
                /*
                 * Disable gather in sa command
                 */
                sa->sa_command_0.bf.gather = 0;
                /*
                 * Indicate gather array is not used
                 */
                pd_uinfo->first_gd = 0xffffffff;
                pd_uinfo->num_gd = 0;
        }
        if (num_sd) {
                struct ce_sd *sd = NULL;
                u32 sd_idx = fst_sd;
                nbytes = datalen;
                sa->sa_command_0.bf.scatter = 1;
                pd_uinfo->using_sd = 1;
                pd_uinfo->dest_va = dst;
                pd_uinfo->first_sd = fst_sd;
                pd_uinfo->num_sd = num_sd;
                sd = crypto4xx_get_sdp(dev, &sd_dma, sd_idx);
                pd->dest = sd_dma;
                /* setup scatter descriptor */
                sd->ctl.done = 0;
                sd->ctl.rdy = 1;
                /* sd->ptr should be setup by sd_init routine*/
                idx = 0;
                if (nbytes >= PPC4XX_SD_BUFFER_SIZE)
                        nbytes -= PPC4XX_SD_BUFFER_SIZE;
                else
                        nbytes = 0;
                while (nbytes) {
                        sd_idx = get_next_sd(sd_idx);
                        sd = crypto4xx_get_sdp(dev, &sd_dma, sd_idx);
                        /* setup scatter descriptor */
                        sd->ctl.done = 0;
                        sd->ctl.rdy = 1;
                        if (nbytes >= PPC4XX_SD_BUFFER_SIZE)
                                nbytes -= PPC4XX_SD_BUFFER_SIZE;
                        else
                                /*
                                 * SD entry can hold PPC4XX_SD_BUFFER_SIZE,
                                 * which is more than nbytes, so done.
                                 */
                                nbytes = 0;
                }
        } else {
                /*
                 * we know application give us dst a whole piece of memory
                 * no need to use scatter ring.
                 * In case of is_hash, the icv is always at end of src data.
                 */
                pd_uinfo->using_sd = 0;
                pd_uinfo->first_sd = 0xffffffff;
                pd_uinfo->num_sd = 0;
                pd_uinfo->dest_va = dst;
                sa->sa_command_0.bf.scatter = 0;
                if (ctx->is_hash)
                        pd->dest = virt_to_phys((void *)dst);
                else
// TODO - not unmapped
                        pd->dest = (u32)dma_map_page(dev->core_dev->device,
                                        sg_page(dst), dst->offset,
                                        dst->length, DMA_TO_DEVICE);
        }

        sa->sa_command_1.bf.hash_crypto_offset = (aad_len >> 2);
        pd->pd_ctl.w = ctx->pd_ctl;
        pd->pd_ctl_len.w = 0x00400000 | (ctx->bypass << 24) |
                        (datalen + aad_len);
        // set to ring size
        if (dev->pdr_head > dev->pdr_tail) {
            aadlen = dev->pdr_head - dev->pdr_tail;
        } else {
            aadlen =  PPC4XX_NUM_PD - (dev->pdr_tail - dev->pdr_head);
        }
        writel(aadlen , dev->ce_base + CRYPTO4XX_INT_DESCR_CNT);
        wmb();
        /* write any value to push engine to read a pd */
        writel(1, dev->ce_base + CRYPTO4XX_INT_DESCR_RD);
        return -EINPROGRESS;
}

// Encryptoion Algorithm Registration Functions
static int crypto4xx_alg_init(struct crypto_tfm *tfm)
{
        struct crypto_alg *alg = tfm->__crt_alg;
        struct crypto4xx_alg *amcc_alg = crypto_alg_to_crypto4xx_alg(alg);
        struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        const char *fallback_driver_name = alg->cra_name;

        tctx->fallback = crypto_alloc_skcipher(fallback_driver_name,
                                                0, CRYPTO_ALG_NEED_FALLBACK);
	if (IS_ERR(tctx->fallback)) {
            printk(KERN_WARNING
                   "Fallback driver '%s' could not be loaded!\n",
                   fallback_driver_name);
            tctx->fallback = NULL;
            tfm->crt_ablkcipher.reqsize = 0;
        } else {
            tfm->crt_ablkcipher.reqsize =
                sizeof(struct crypto4xx_cipher_req_ctx)
                + crypto_blkcipher_tfm(tctx->fallback)->crt_ablkcipher.reqsize;
        }
        tctx->append_icv = 0;
        tctx->init_arc4 = 0;
        ctx->dev = amcc_alg->dev;
        ctx->sa_in = NULL;
        ctx->sa_out = NULL;
        ctx->sa_in_dma_addr = 0;
        ctx->sa_out_dma_addr = 0;
        ctx->sa_len = 0;
        ctx->is_gcm = 0;
        return 0;
}

static void crypto4xx_alg_exit(struct crypto_tfm *tfm)
{
        struct crypto4xx_cipher_tfm_ctx *tctx = crypto_tfm_ctx(tfm);
        struct crypto4xx_ctx_base *ctx = &tctx->base;
        crypto4xx_free_sa(ctx);
        crypto4xx_free_state_record(ctx);
	if (tctx->fallback) crypto_free_skcipher(tctx->fallback);
}

static int crypto4xx_register_alg(struct crypto4xx_device *sec_dev,
                           struct crypto4xx_alg_common *crypto_alg,
                           int array_size)
{
        struct crypto4xx_alg *alg;
        int i;
        int rc = 0;

        for (i = 0; i < array_size; i++) {
                alg = kzalloc(sizeof(struct crypto4xx_alg), GFP_KERNEL);
                if (!alg)
                        return -ENOMEM;

                alg->alg = crypto_alg[i];
                alg->dev = sec_dev;

                switch (alg->alg.type) {
                case CRYPTO_ALG_TYPE_AHASH:
                        alg->alg.u.hash.halg.base.cra_flags |= CRYPTO_ALG_NEED_FALLBACK;
                        rc = crypto_register_ahash(&alg->alg.u.hash);
                        break;
                case CRYPTO_ALG_TYPE_AEAD:
                        rc = crypto_register_aead(&alg->alg.u.aead);
                        break;
                default:
                        alg->alg.u.cipher.cra_flags |= CRYPTO_ALG_NEED_FALLBACK;
                        rc = crypto_register_alg(&alg->alg.u.cipher);
                        break;
                }

                if (rc) {
                        list_del(&alg->entry);
                        kfree(alg);
                } else {
                        list_add_tail(&alg->entry, &sec_dev->alg_list);
                }
        }

        return 0;
}

static void crypto4xx_unregister_alg(struct crypto4xx_device *sec_dev)
{
        struct crypto4xx_alg *alg, *tmp;

        list_for_each_entry_safe(alg, tmp, &sec_dev->alg_list, entry) {
                list_del(&alg->entry);
                switch (alg->alg.type) {
                case CRYPTO_ALG_TYPE_AHASH:
                        crypto_unregister_ahash(&alg->alg.u.hash);
                        break;

                default:
                        crypto_unregister_alg(&alg->alg.u.cipher);
                }
                kfree(alg);
        }
}

static void crypto4xx_interrupt_timer(unsigned long data)
{
        struct crypto4xx_core_device *core_dev =
                                    (struct crypto4xx_core_device *) data;
        struct pd_uinfo *pd_uinfo;
        struct ce_pd *pd;
        u32 tail;

        while (core_dev->dev->pdr_head != core_dev->dev->pdr_tail) {
                tail = core_dev->dev->pdr_tail;
                pd_uinfo = core_dev->dev->pdr_uinfo +
                        sizeof(struct pd_uinfo)*tail;
                pd = core_dev->dev->pdr + sizeof(struct ce_pd) * tail;
                if ((pd_uinfo->state == PD_ENTRY_INUSE) &&
                                   pd->pd_ctl.bf.pe_done &&
                                   !pd->pd_ctl.bf.host_ready) {
                        pd->pd_ctl.bf.pe_done = 0;
                        crypto4xx_pd_done(core_dev->dev, tail);
                        crypto4xx_put_pd_to_pdr(core_dev->dev, tail);
                } else {
                    // we found some unprocessed descriptors
                    if (core_dev->dev->pdr_head > core_dev->dev->pdr_tail) {
                        tail = core_dev->dev->pdr_head - core_dev->dev->pdr_tail;
                    } else {
                        tail = PPC4XX_NUM_PD - (core_dev->dev->pdr_tail - core_dev->dev->pdr_head);
                    }
                    writel(tail, core_dev->dev->ce_base + CRYPTO4XX_INT_DESCR_CNT);
                    wmb();
                    writel(1, core_dev->dev->ce_base + CRYPTO4XX_INT_DESCR_RD);
                    /* if tail not done, break */
                    break;
                }
        }
}

/**
 * Top Half of isr.
 */
static irqreturn_t crypto4xx_ce_interrupt_handler(int irq, void *data)
{
        //unsigned long flags;
        struct device *dev = (struct device *)data;
        struct crypto4xx_core_device *core_dev = dev_get_drvdata(dev);

        if (core_dev->dev->ce_base == 0)
                return 0;

        writel(PPC4XX_INTERRUPT_CLR,
               core_dev->dev->ce_base + CRYPTO4XX_INT_MASK_STAT);
        mod_timer(&crypto4xx_timer, jiffies + 1);

        return IRQ_HANDLED;
}

/**
 * Supported Crypto Algorithms
 */
struct crypto4xx_alg_common crypto4xx_alg[] = {
#if _4XX_ENABLE_CRYPT
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        /* Crypto DES ECB, CBC,  modes */
        {
            .cra_name           = "cbc(des)",
            .cra_driver_name    = "ppc4xx-cbc-des",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = DES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_u = {
                .ablkcipher = {
                    .min_keysize        = DES_KEY_SIZE,
                    .max_keysize        = DES_KEY_SIZE,
                    .ivsize             = DES_BLOCK_SIZE,
                    .setkey             = crypto4xx_setkey_3des_cbc,
                    .encrypt            = crypto4xx_encrypt,
                    .decrypt            = crypto4xx_decrypt,
                }
            }
        }},
#if 0
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "ecb(des)",
            .cra_driver_name    = "ppc4xx-ecb-des",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = DES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = DES_KEY_SIZE,
                .max_keysize        = DES_KEY_SIZE,
                .setkey             = crypto4xx_setkey_3des_ecb,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
#endif
    /* Crypto 3DES ECB, CBC, CFB, and OFB modes */
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "cbc(des3_ede)",
            .cra_driver_name    = "ppc4xx-cbc-3des",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = DES3_EDE_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = DES3_EDE_KEY_SIZE,
                .max_keysize        = DES3_EDE_KEY_SIZE,
                .ivsize             = DES3_EDE_BLOCK_SIZE,
                .setkey             = crypto4xx_setkey_3des_cbc,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
#if 0
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "ecb(des3_ede)",
            .cra_driver_name    = "ppc4xx-ecb-3des",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = DES3_EDE_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = DES3_EDE_KEY_SIZE,
                .max_keysize        = DES3_EDE_KEY_SIZE,
                .setkey             = crypto4xx_setkey_3des_ecb,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
#endif
    /* Crypto AES modes */
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "cbc(aes)",
            .cra_driver_name    = "cbc-aes-ppc4xx",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = AES_MIN_KEY_SIZE,
                .max_keysize        = AES_MAX_KEY_SIZE,
                .ivsize             = AES_IV_SIZE,
                .setkey             = crypto4xx_setkey_aes_cbc,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
#if 0
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "ofb(aes)",
            .cra_driver_name    = "ppc4xx-ofb-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = AES_MIN_KEY_SIZE,
                .max_keysize        = AES_MAX_KEY_SIZE,
                .ivsize             = AES_BLOCK_SIZE,
                .setkey             = crypto4xx_setkey_aes_ofb,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "cfb(aes)",
            .cra_driver_name    = "ppc4xx-cfb-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = AES_MIN_KEY_SIZE,
                .max_keysize        = AES_MAX_KEY_SIZE,
                .ivsize             = AES_BLOCK_SIZE,
                .setkey             = crypto4xx_setkey_aes_cfb,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
    /* Crypto AES ECB, CBC, CTR, GCM, CCM, and GMAC modes */
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "ecb(aes)",
            .cra_driver_name    = "ppc4xx-ecb-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = AES_MIN_KEY_SIZE,
                .max_keysize        = AES_MAX_KEY_SIZE,
                .setkey             = crypto4xx_setkey_aes_ecb,
                .encrypt            = crypto4xx_encrypt,
                .decrypt            = crypto4xx_decrypt,
            }
        }},
#endif
    { .type = CRYPTO_ALG_TYPE_ABLKCIPHER, .u.cipher =
        {
            .cra_name           = "rfc3686(ctr(aes))",
            .cra_driver_name    = "ppc4xx-ctr-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_ABLKCIPHER | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = CTR_RFC3686_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_type           = &crypto_ablkcipher_type,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
            .cra_ablkcipher = {
                .min_keysize        = AES_MIN_KEY_SIZE,
                .max_keysize        = AES_MAX_KEY_SIZE,
                .ivsize             = 8,
                .setkey             = crypto4xx_setkey_aes_ctr,
                .encrypt            = crypto4xx_encrypt_ctr,
                .decrypt            = crypto4xx_decrypt_ctr,
            }
        }},
#endif
#if _4XX_ENABLE_AEAD
    /* AEAD Algorithms */
    { .type = CRYPTO_ALG_TYPE_AEAD, .u.aead =
        {
        .base = {
            .cra_name           = "echainiv(gcm(aes))",
            .cra_driver_name    = "ppc4xx-gcm-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
        },
                .maxauthsize        = 16,
                .ivsize             = 12,
                .setkey             = crypto4xx_setkey_aes_gcm,
                .setauthsize        = crypto4xx_setauthsize_aes,
                .encrypt            = crypto4xx_encrypt_aes_gcm,
                .decrypt            = crypto4xx_decrypt_aes_gcm,
        }},
    { .type = CRYPTO_ALG_TYPE_AEAD, .u.aead =
        {
        .base = {
            .cra_name           = "echainiv(ccm(aes))",
            .cra_driver_name    = "ppc4xx-ccm-aes",
            .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
            .cra_flags          = CRYPTO_ALG_TYPE_AEAD | CRYPTO_ALG_ASYNC,
            .cra_blocksize      = AES_BLOCK_SIZE,
            .cra_ctxsize        = sizeof(struct crypto4xx_cipher_tfm_ctx),
            .cra_alignmask      = 0,
            .cra_init           = crypto4xx_alg_init,
            .cra_exit           = crypto4xx_alg_exit,
            .cra_module         = THIS_MODULE,
        },
                .ivsize             = AES_BLOCK_SIZE,
                .maxauthsize        = 16,
                .setkey             = crypto4xx_setkey_aes_ccm,
                .setauthsize        = crypto4xx_setauthsize_aes,
                .encrypt            = crypto4xx_encrypt_aes_ccm,
                .decrypt            = crypto4xx_decrypt_aes_ccm,
        }},
#endif
#if _4XX_ENABLE_HASH
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        /* Hash MD5 */
        {
            .init               = crypto4xx_hash_init,
            .update             = crypto4xx_hash_update,
            .final              = crypto4xx_hash_final,
            .digest             = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SA_HASH_ALG_MD5_DIGEST_SIZE,
                .base = {
                    .cra_name           = "md5",
                    .cra_driver_name    = "ppc4xx-md5",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = 64,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_md5_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        /* Hash MD5-HMAC */
        {
            .init               = crypto4xx_hash_init,
            .update             = crypto4xx_hash_update,
            .final              = crypto4xx_hash_final,
            .digest             = crypto4xx_hash_digest,
            .setkey             = crypto4xx_md5_hmac_setkey,
            .halg = {
                .digestsize             = SA_HASH_ALG_MD5_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(md5)",
                    .cra_driver_name    = "ppc4xx-hmac-md5",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = 64,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_md5_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        /* Hash SHA1, SHA2 and HMAC */
        {
            .init               = crypto4xx_hash_init,
            .update             = crypto4xx_hash_update,
            .final              = crypto4xx_hash_final,
            .digest             = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SHA1_DIGEST_SIZE,
                .base = {
                    .cra_name           = "sha1",
                    .cra_driver_name    = "ppc4xx-sha1",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA1_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha1_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init               = crypto4xx_hash_init,
            .update             = crypto4xx_hash_update,
            .final              = crypto4xx_hash_final,
            .digest             = crypto4xx_hash_digest,
            .setkey             = crypto4xx_sha1_hmac_setkey,
            .halg = {
                .digestsize             = SHA1_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(sha1)",
                    .cra_driver_name    = "ppc4xx-hmac-sha1",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA1_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha1_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SHA224_DIGEST_SIZE,
                .base = {
                    .cra_name           = "sha224",
                    .cra_driver_name    = "ppc4xx-sha224",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA224_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .setkey                 = crypto4xx_sha2_hmac_setkey,
            .halg = {
                .digestsize             = SHA224_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(sha224)",
                    .cra_driver_name    = "ppc4xx-hmac-sha224",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA224_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SHA256_DIGEST_SIZE,
                .base = {
                    .cra_name           = "sha256",
                    .cra_driver_name    = "ppc4xx-sha256",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA256_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .setkey                 = crypto4xx_sha2_hmac_setkey,
            .halg = {
                .digestsize             = SHA256_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(sha256)",
                    .cra_driver_name    = "ppc4xx-hmac-sha256",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA256_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SHA384_DIGEST_SIZE,
                .base = {
                    .cra_name           = "sha384",
                    .cra_driver_name    = "ppc4xx-sha384",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA384_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .setkey                 = crypto4xx_sha2_hmac_setkey,
            .halg = {
                .digestsize             = SHA384_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(sha384)",
                    .cra_driver_name    = "ppc4xx-hmac-sha384",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA384_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .halg = {
                .digestsize             = SHA512_DIGEST_SIZE,
                .base = {
                    .cra_name           = "sha512",
                    .cra_driver_name    = "ppc4xx-sha512",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA512_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_alg_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
    { .type = CRYPTO_ALG_TYPE_AHASH, .u.hash =
        {
            .init                   = crypto4xx_hash_init,
            .update                 = crypto4xx_hash_update,
            .final                  = crypto4xx_hash_final,
            .digest                 = crypto4xx_hash_digest,
            .setkey                 = crypto4xx_sha2_hmac_setkey,
            .halg = {
                .digestsize             = SHA512_DIGEST_SIZE,
                .base = {
                    .cra_name           = "hmac(sha512)",
                    .cra_driver_name    = "ppc4xx-hmac-sha512",
                    .cra_priority       = CRYPTO4XX_CRYPTO_PRIORITY,
                    .cra_flags          = CRYPTO_ALG_TYPE_AHASH | CRYPTO_ALG_ASYNC,
                    .cra_blocksize      = SHA512_BLOCK_SIZE,
                    .cra_ctxsize        = sizeof(struct crypto4xx_ahash_tfm_ctx),
                    .cra_alignmask      = 0,
                    .cra_type           = &crypto_ahash_type,
                    .cra_init           = crypto4xx_sha2_hmac_init,
                    .cra_exit           = crypto4xx_hash_alg_exit,
                    .cra_module         = THIS_MODULE,
                }
            }
        }},
#endif
};

/**
 * Module Initialization Routine
 */
static int crypto4xx_probe(struct platform_device *ofdev)
{
        int rc;
        struct resource res;
        struct device *dev = &ofdev->dev;
        struct crypto4xx_core_device *core_dev;

        rc = of_address_to_resource(ofdev->dev.of_node, 0, &res);
        if (rc)
                return -ENODEV;

        if (of_find_compatible_node(NULL, NULL, "amcc,ppc460ex-crypto")) {
                mtdcri(SDR0, PPC460EX_SDR0_SRST,
                       mfdcri(SDR0, PPC460EX_SDR0_SRST) | PPC460EX_CE_RESET);
                mtdcri(SDR0, PPC460EX_SDR0_SRST,
                       mfdcri(SDR0, PPC460EX_SDR0_SRST) & ~PPC460EX_CE_RESET);
        } else if (of_find_compatible_node(NULL, NULL,
                        "amcc,ppc405ex-crypto")) {
                mtdcri(SDR0, PPC405EX_SDR0_SRST,
                       mfdcri(SDR0, PPC405EX_SDR0_SRST) | PPC405EX_CE_RESET);
                mtdcri(SDR0, PPC405EX_SDR0_SRST,
                       mfdcri(SDR0, PPC405EX_SDR0_SRST) & ~PPC405EX_CE_RESET);
        } else if (of_find_compatible_node(NULL, NULL,
                        "amcc,ppc460sx-crypto")) {
                mtdcri(SDR0, PPC460SX_SDR0_SRST,
                       mfdcri(SDR0, PPC460SX_SDR0_SRST) | PPC460SX_CE_RESET);
                mtdcri(SDR0, PPC460SX_SDR0_SRST,
                       mfdcri(SDR0, PPC460SX_SDR0_SRST) & ~PPC460SX_CE_RESET);
        } else {
                printk(KERN_ERR "Crypto Function Not supported!\n");
                return -EINVAL;
        }

        core_dev = kzalloc(sizeof(struct crypto4xx_core_device), GFP_KERNEL);
        if (!core_dev)
                return -ENOMEM;

        dev_set_drvdata(dev, core_dev);
        core_dev->ofdev = ofdev;
        core_dev->dev = kzalloc(sizeof(struct crypto4xx_device), GFP_KERNEL);
        if (!core_dev->dev)
                goto err_alloc_dev;

        core_dev->dev->core_dev = core_dev;
        core_dev->device = dev;
        spin_lock_init(&core_dev->lock);
        INIT_LIST_HEAD(&core_dev->dev->alg_list);
        rc = crypto4xx_build_pdr(core_dev->dev);
        if (rc)
                goto err_build_pdr;

        rc = crypto4xx_build_gdr(core_dev->dev);
        if (rc)
                goto err_build_gdr;

        rc = crypto4xx_build_sdr(core_dev->dev);
        if (rc)
                goto err_build_sdr;

        /* Register for Crypto isr, Crypto Engine IRQ */
        core_dev->irq = irq_of_parse_and_map(ofdev->dev.of_node, 0);
        rc = request_irq(core_dev->irq, crypto4xx_ce_interrupt_handler, 0,
                         core_dev->dev->name, dev);
        if (rc)
                goto err_request_irq;

        core_dev->dev->ce_base = of_iomap(ofdev->dev.of_node, 0);
        if (!core_dev->dev->ce_base) {
                dev_err(dev, "failed to of_iomap\n");
                goto err_iomap;
        }

        /* need to setup pdr, rdr, gdr and sdr before this */
        crypto4xx_hw_init(core_dev->dev);

        /* timer for bottom half processing */
        crypto4xx_timer.function = crypto4xx_interrupt_timer;
        crypto4xx_timer.data = (int) core_dev;
        init_timer(&crypto4xx_timer);

        /* Register security algorithms with Linux CryptoAPI */
        rc = crypto4xx_register_alg(core_dev->dev, crypto4xx_alg,
                               ARRAY_SIZE(crypto4xx_alg));
        if (rc)
                goto err_start_dev;

        return 0;

err_start_dev:
        del_timer_sync(&crypto4xx_timer);
        iounmap(core_dev->dev->ce_base);
err_iomap:
        free_irq(core_dev->irq, dev);
        irq_dispose_mapping(core_dev->irq);
err_request_irq:
        crypto4xx_destroy_sdr(core_dev->dev);
err_build_sdr:
        crypto4xx_destroy_gdr(core_dev->dev);
err_build_gdr:
        crypto4xx_destroy_pdr(core_dev->dev);
err_build_pdr:
        kfree(core_dev->dev);
err_alloc_dev:
        kfree(core_dev);

        return rc;
}

static int __exit crypto4xx_remove(struct platform_device *ofdev)
{
        struct device *dev = &ofdev->dev;
        struct crypto4xx_core_device *core_dev = dev_get_drvdata(dev);

        free_irq(core_dev->irq, dev);
        irq_dispose_mapping(core_dev->irq);

        del_timer_sync(&crypto4xx_timer);
        /* Un-register with Linux CryptoAPI */
        crypto4xx_unregister_alg(core_dev->dev);
        /* Free all allocated memory */
        crypto4xx_stop_all(core_dev);
        return 0;
}

static const struct of_device_id crypto4xx_match[] = {
        { .compatible      = "amcc,ppc4xx-crypto",},
        { },
};

static struct platform_driver crypto4xx_driver = {
        .driver = {
                .name = "crypto4xx",
                .owner = THIS_MODULE,
                .of_match_table = crypto4xx_match,
        },
        .probe                = crypto4xx_probe,
        .remove                = crypto4xx_remove,
};

static int __init crypto4xx_init(void)
{
        return platform_driver_register(&crypto4xx_driver);
}

static void __exit crypto4xx_exit(void)
{
        platform_driver_unregister(&crypto4xx_driver);
}

module_init(crypto4xx_init);
module_exit(crypto4xx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("James Hsiao <jhsiao@amcc.com>");
MODULE_DESCRIPTION("Driver for AMCC PPC4xx crypto accelerator");

