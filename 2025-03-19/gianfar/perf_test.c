#ifdef STATS
#include <linux/switch.h>

#include "perf_core.h"
#include "perf_cpu.h"
#include "gfar.h"

#define TIMES 2000
#define DATA_SIZE (128 * 1024)
#define DATA_SAMPLES (DATA_SIZE / sizeof(unsigned))
static unsigned *data0;
static unsigned *data1;


static inline void prefetch_l2(const void *x) {
    __asm__ __volatile__ ("dcbt 1,%0" : : "r" (x));
}


static inline void cache_lock(const void *x) {
    __asm__ __volatile__ ("dcbtls 0,%0" : : "r" (x));
//    __asm__ __volatile__ ("dcbtls 1,%0" : : "r" (x));
}

unsigned test_0(unsigned *data) {
    unsigned i;
    unsigned k;
    unsigned v = 0;

/*
//    printk("data: %p\n", data);
    for (i = 0; i < DATA_SAMPLES; ++i) {
//	cache_lock(&data[i]);
	data[i] = i;
//	v += data[i];
    }
*/

    for (k = 0; k < TIMES; ++k) {
	for (i = 0; i < DATA_SAMPLES; i += 1) {
//	    prefetch_l2(&data[i]);
	    data[i] = 6 + k * i + data[i];
//	    v += data[i] * k;
	}
    }


    return v;
}

unsigned test_1(unsigned *data) {
    return test_0(data);
}

static void test_do(void *xxx) {
    if (smp_processor_id() == 0) {
	test_0(data0);
    }
    if (smp_processor_id() == 1) {
	test_1(data1);
    }
}

void gfar_perf(int output);
void perf_test(void) {
    if (!sram_vaddr) {
	return;
    }

    printk("******* test\n");

    data0 = (unsigned *)sram_vaddr;
    data1 = (unsigned *)(sram_vaddr + DATA_SIZE);

    printk("test in %lu %u cache lines: %s\n", jiffies, smp_processor_id(),
	    perf_cpu_format_number(sizeof(data0) / 32));
//    printk("total iterations: %s\n",
//	    perf_cpu_format_number(TIMES * sizeof(data0) / sizeof(unsigned)));
    printk("total cache lines accessed: %s\n",
	    perf_cpu_format_number(TIMES * sizeof(data0) / 32));
    gfar_perf(0);
#ifdef CONFIG_SMP
    on_each_cpu(test_do, NULL, 0);
#else
    {
	unsigned long flags;
	local_irq_save(flags);
	test_do(NULL);
	local_irq_restore(flags);
    }
#endif
    gfar_perf(1);
}


#endif
