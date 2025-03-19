/*
 * Fan control driver for PowerPC
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/sort.h>
#include <linux/of_irq.h>

#include <asm/rb_aux.h>
#include <asm/io.h>

#include "sampler.h"

#define GPIO(x) (0x80000000 >> (x))

extern void inc_rsf(int num);

MODULE_LICENSE("GPL");

static void release_intr(unsigned intr_num, unsigned *dummy) {
    if (intr_num) free_irq(intr_num, dummy);
}

static unsigned get_usec_time(void) {
    long long ticks = mfspr(SPRN_TBWL);
    return div_u64(1000000 * ticks, ppc_tb_freq);
}

static int type = IRQF_TRIGGER_FALLING;
static int init_interrupt(const char *name,
		    unsigned *intr_num, 
		    unsigned *dummy, 
		    void *handler) {
    struct device_node *node;
    if ((node = of_find_node_by_name(NULL, name))) {
	*intr_num = irq_of_parse_and_map(node, 0);
	if (request_irq(*intr_num, handler, type, name, dummy)) {
	    *intr_num = 0;
	}	
	of_node_put(node);
	return 0;
    }
    else {
	return -EINVAL;
    }
}

static void fan_gpio(unsigned state) {
    static int fan_on = 0;

    if (!fan_on) {
	fan_on = get_gpio_def("fan_on");
    }
    if (fan_on < 0) {
	return;
    }
    if (fan_on) {
	gpio_set_value(fan_on, state);
    }
}

void main_fan(int num) {
    fan_gpio(0);
}

void aux_fan(int num) {
    fan_gpio(1);
}

static unsigned sense_dummy;
static unsigned sense_int_num = 0;

static struct sampler ss;

static irqreturn_t sense_intr(int irq, void *data) {
    // rising edge gives much more stable results than falling edge (RB1100AHx2)
    if (do_sample(&ss, get_usec_time(), type == IRQF_TRIGGER_RISING)) {
	inc_rsf(0);
    }

    if (type == IRQF_TRIGGER_RISING) {
	type = IRQF_TRIGGER_FALLING;
    }
    else {
	type = IRQF_TRIGGER_RISING;
    }
    irq_set_irq_type(sense_int_num, type);
    return IRQ_HANDLED;
}

extern unsigned (*get_rpm)(int num);

static unsigned e500_get_rpm(int num) {
    return sampler_get_rpm(&ss, get_usec_time());
}

void specific_fan_start(void) {
}

void specific_fan_stop(void) {
}

int specific_fan_init(void) {
    reset_sampler(&ss);
    get_rpm = e500_get_rpm;
    return init_interrupt("sense", &sense_int_num, &sense_dummy, &sense_intr);
}

void specific_fan_exit(void) {
    release_intr(sense_int_num, &sense_dummy);
    specific_fan_stop();
}
