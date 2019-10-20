/*
 * based on original include/linux/dmapool.h
 * Copyright (C) 2014-2015 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Allocation pools for DMAable (coherent) memory.
 *
 */

#ifndef LINUX_DMAPOOL_H
#define	LINUX_DMAPOOL_H

#include <asm/io.h>
#include <asm/scatterlist.h>

struct dma_pool *dma_pool_create(const char *name, struct device *dev,
			size_t size, size_t align, size_t allocation);

struct dma_pool *dma_wc_pool_create(const char *name, struct device *dev,
			size_t size, size_t align, size_t allocation);


void dma_pool_destroy(struct dma_pool *pool);

void *dma_pool_alloc(struct dma_pool *pool, gfp_t mem_flags,
		     dma_addr_t *handle);

void dma_pool_free(struct dma_pool *pool, void *vaddr, dma_addr_t addr);

/*
 * Managed DMA pool
 */
struct dma_pool *dmam_pool_create(const char *name, struct device *dev,
				  size_t size, size_t align, size_t allocation);
void dmam_pool_destroy(struct dma_pool *pool);

#endif

