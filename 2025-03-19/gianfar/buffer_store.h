#ifndef BUFFER_STORE_H
#define BUFFER_STORE_H

//#define OVERLAP_BUFFER_STORE
#ifdef OVERLAP_BUFFER_STORE

struct buffer_store {
    unsigned count;
    unsigned total_size;
    unsigned non_overlap_bits;
    unsigned dma_direction;

    unsigned char *data;
    unsigned pdata;
};

int buffer_store_init(struct buffer_store *s,
	unsigned count, unsigned size, unsigned non_overlap_bits,
	unsigned dma_direction) {
    unsigned needed_mem = count * (1 << non_overlap_bits) + size;
//    printk("alloc overlap store %u\n", needed_mem);
    s->count = count;
    s->total_size = needed_mem;
    s->dma_direction = dma_direction;
    s->non_overlap_bits = non_overlap_bits;
    s->data = kzalloc(needed_mem, GFP_ATOMIC);
    if (!s->data) {
	return -ENOMEM;
    }
    s->pdata = dma_map_single(NULL, s->data, needed_mem, dma_direction);
    return 0;
}

void buffer_store_free(struct buffer_store *s) {
    if (s->pdata) {
	dma_unmap_single(NULL, s->pdata, s->total_size, s->dma_direction);
	s->pdata = 0;
    }
    if (s->data) {
	kfree(s->data);
	s->data = NULL;
    }
}

static inline unsigned char *buffer_store_get_buffer(
    struct buffer_store *s, unsigned num) {
    return s->data + (num << s->non_overlap_bits);
}

static inline unsigned buffer_store_get_dma_buffer(
    struct buffer_store *s, unsigned num) {
    return s->pdata + (num << s->non_overlap_bits);
}

#else

/*
struct buffer_store {
    unsigned count;
    unsigned size;
    unsigned dma_direction;

    unsigned char **data;
    unsigned *pdata;
};

int buffer_store_init(struct buffer_store *s,
	unsigned count, unsigned size, unsigned non_overlap_bits,
	unsigned dma_direction) {
    unsigned i;
//    printk("alloc non-overlap store %u\n", count * size);
    s->count = count;
    s->size = size;
    s->dma_direction = dma_direction;
    s->data = kzalloc(count * sizeof(unsigned char *), GFP_ATOMIC);
    if (!s->data) {
	return -ENOMEM;
    }
    s->pdata = kzalloc(count * sizeof(unsigned), GFP_ATOMIC);
    if (!s->pdata) {
	return -ENOMEM;
    }

    for (i = 0; i < s->count; ++i) {
	s->data[i] = kzalloc(size, GFP_ATOMIC);
	if (!s->data[i]) {
	    return -ENOMEM;
	}
	s->pdata[i] = dma_map_single(
	    NULL, s->data[i], size, dma_direction);
    }
    return 0;
}

void buffer_store_free(struct buffer_store *s) {
    unsigned i;
    if (s->pdata) {
	for (i = 0; i < s->count; ++i) {
	    if (s->pdata[i]) {
		dma_unmap_single(
		    NULL, s->pdata[i], s->size, s->dma_direction);
	    }
	}
	kfree(s->pdata);
	s->pdata = NULL;
    }
    if (s->data) {
	for (i = 0; i < s->count; ++i) {
	    if (s->data[i]) {
		kfree(s->data[i]);
	    }
	}
	kfree(s->data);
	s->data = NULL;
    }
}

static inline unsigned char *buffer_store_get_buffer(
    struct buffer_store *s, unsigned num) {
    return s->data[num];
}

static inline unsigned buffer_store_get_dma_buffer(
    struct buffer_store *s, unsigned num) {
    return s->pdata[num];
}
*/



struct buffer_store {
    unsigned count;
    unsigned size;
    unsigned dma_direction;

    unsigned char **base_data;
    unsigned char **data;
    unsigned *pdata;
};

int buffer_store_init(struct buffer_store *s,
	unsigned count, unsigned size, unsigned non_overlap_bits,
	unsigned dma_direction) {
    unsigned i;
//    printk("alloc non-overlap store %u\n", count * size);
    s->count = count;
    s->size = size;
    s->dma_direction = dma_direction;
    s->base_data = kzalloc(count * sizeof(unsigned char *), GFP_ATOMIC);
    if (!s->base_data) {
	return -ENOMEM;
    }
    s->data = kzalloc(count * sizeof(unsigned char *), GFP_ATOMIC);
    if (!s->data) {
	return -ENOMEM;
    }
    s->pdata = kzalloc(count * sizeof(unsigned), GFP_ATOMIC);
    if (!s->pdata) {
	return -ENOMEM;
    }

    for (i = 0; i < s->count; ++i) {
	s->base_data[i] = kzalloc(size + 32 * 32, GFP_ATOMIC);
	if (!s->base_data[i]) {
	    return -ENOMEM;
	}
	s->data[i] = s->base_data[i] + (i % 32) * 32;
	s->pdata[i] = dma_map_single(
	    NULL, s->data[i], size, dma_direction);
    }
    return 0;
}

void buffer_store_free(struct buffer_store *s) {
    unsigned i;
    if (s->pdata) {
	for (i = 0; i < s->count; ++i) {
	    if (s->pdata[i]) {
		dma_unmap_single(
		    NULL, s->pdata[i], s->size, s->dma_direction);
	    }
	}
	kfree(s->pdata);
	s->pdata = NULL;
    }
    if (s->data) {
	kfree(s->data);
	s->data = NULL;
    }
    if (s->base_data) {
	for (i = 0; i < s->count; ++i) {
	    if (s->base_data[i]) {
		kfree(s->base_data[i]);
	    }
	}
	kfree(s->base_data);
	s->base_data = NULL;
    }
}

static inline unsigned char *buffer_store_get_buffer(
    struct buffer_store *s, unsigned num) {
    return s->data[num];
}

static inline unsigned buffer_store_get_dma_buffer(
    struct buffer_store *s, unsigned num) {
    return s->pdata[num];
}

#endif

#endif
