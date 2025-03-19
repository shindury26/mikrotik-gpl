#include <linux/voltage.h>
#include <linux/sort.h>

void reset_sampler(struct SamplerData *sampler) {
    sampler->flip = 1;
    sampler->index = 0;
}

static void sample_data(struct SamplerData *sampler, int should_pair) {
    unsigned diff = get_usec_time() - sampler->time;
    if (diff > 0 && sampler->index < SAMPLE_COUNT) {	
	if (sampler->flip) {
	    sampler->data[sampler->index] = diff;
	    if (!should_pair) sampler->index++;
	}
	else {
	    sampler->data[sampler->index++] += diff;
	}
	if (should_pair) sampler->flip = ! sampler->flip;
    }    
    sampler->time = get_usec_time();
}

void sample_simple_data(struct SamplerData *sampler) {
    sample_data(sampler, 0);
}

void sample_paired_data(struct SamplerData *sampler) {
    sample_data(sampler, 1);
}

void reset_time(struct SamplerData *sampler) {
    sampler->time = get_usec_time();
}

static int cmp(const void *a, const void *b) {
    const unsigned *x = a;
    const unsigned *y = b;    
    return *x > *y ? 1 : (*x < *y ? -1 : 0);
}

#define FRACTION 20 // 1/20 = 5%

unsigned calculate_n_reset(struct SamplerData *sampler) {
    unsigned ret;
    unsigned i = 0;
    unsigned x = 0;
    unsigned max_a = 0;
    unsigned max_b = 0;
    unsigned *data = sampler->data;
    sort(sampler->data, sampler->index, sizeof(unsigned), cmp, NULL);
    for (i = 1; i < sampler->index; i++) {
	if (i - x - 1 > max_b - max_a) {
	    max_a = x;
	    max_b = i - 1;
	}
	if (data[i] - data[i - 1] > data[i - 1] / FRACTION) {
	    x = i;
	}
    }
    ret = sampler->data[(max_a + max_b) / 2];
    reset_sampler(sampler);
    return ret;
}

