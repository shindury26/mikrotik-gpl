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

#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/io.h>

#include "sampler.h"

#define GPIO(x) (0x80000000 >> (x))

extern void inc_rsf(int num);

static unsigned dummy_dev;

MODULE_LICENSE("GPL");

static unsigned *gpio_data = NULL;
static unsigned *gpio_base = NULL;

static unsigned fan_on_pin = 0;

static unsigned fancon_int_num;
static int type = IRQF_TRIGGER_FALLING;

static struct sampler ss;

static unsigned get_usec_time(void) {
    long long ticks = mfspr(SPRN_TBWL);
    return div_u64(1000000 * ticks, ppc_tb_freq);
}

static irqreturn_t mpc_fan_intr(int irq, void *data) {
    if (do_sample(&ss, get_usec_time(), type == IRQF_TRIGGER_RISING)) {
	inc_rsf(0);
    }

    if (type == IRQF_TRIGGER_RISING) {
	type = IRQF_TRIGGER_FALLING;
    }
    else {
	type = IRQF_TRIGGER_RISING;
    }
    irq_set_irq_type(fancon_int_num, type);
    return IRQ_HANDLED;
}

extern unsigned (*get_rpm)(int num);

static unsigned mpc_get_rpm(int num) {
    return sampler_get_rpm(&ss, get_usec_time());
}

void main_fan(int num) {
    out_be32(gpio_data, in_be32(gpio_data) & ~GPIO(fan_on_pin));
}

void aux_fan(int num) {
    out_be32(gpio_data, in_be32(gpio_data) | GPIO(fan_on_pin));
}

int specific_fan_init(void) {
    struct device_node *gpio;
    struct device_node *fancon;
    const unsigned *fan_on;

    struct resource res;

    fancon = of_find_node_by_name(NULL, "fancon");
    if (!fancon) return -1;

    fan_on = of_get_property(fancon, "fan_on", NULL);
    fan_on_pin = fan_on[1];

    fancon_int_num = irq_of_parse_and_map(fancon, 0);
    if (!fancon_int_num) {
	printk(KERN_ERR "fancon: interrupt not provided\n");
	return -1;
    }

    of_node_put(fancon);

    gpio = of_find_node_by_phandle(fan_on[0]);
    if (!gpio || of_address_to_resource(gpio, 0, &res)) return -1;
    of_node_put(gpio);

    gpio_data = ioremap(res.start, res.end - res.start + 1);

    if (request_irq(fancon_int_num,
		    &mpc_fan_intr,
		    type,
		    "fancon", 
		    &dummy_dev)) {
	printk("fancon: could not allocate interrupt\n");
    }
    get_rpm = mpc_get_rpm;
    return 0;
}

void specific_fan_start(void) {
}

void specific_fan_stop(void) {
}

void specific_fan_exit(void) {
    free_irq(fancon_int_num, &dummy_dev);
    specific_fan_stop();

    if (gpio_data) iounmap(gpio_data);
    if (gpio_base) iounmap(gpio_base);
}
