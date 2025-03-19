#include <linux/irq.h>
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
#include <linux/cycles.h>

#include <linux/of_device.h>
#include <asm/rb.h>

#include "sampler.h"

MODULE_LICENSE("GPL");

extern unsigned (*get_rpm)(int);
extern void inc_rsf(int num);
extern int dual_control;

#define CYCLES_PER_USEC (CYCLES_PER_SEC / 1000000)

static unsigned long get_usecs(void) {
    return cycles_get_cycles() / CYCLES_PER_USEC;
}

void main_fan(int num) {
    set_gpio(0, GPO_FAN_ON(num));
}

void aux_fan(int num) {
    set_gpio(GPO_FAN_ON(num), GPO_FAN_ON(num));
}

static struct sampler ss[2];
irqreturn_t fancon_irq(int irq, void *dummy) {
    int i;
    unsigned long now = get_usecs();
    static unsigned long level[2] =  { 0, 0 };
    unsigned long state = gpio_irq_refresh(fan_sense_gpios());
    for (i = 0; i < fan_sense_count(); i++) {
	unsigned long pin = GPI_SENSE(i + 1);
	unsigned long current_level = get_gpio() & pin;
	if (state & pin && level[i] != current_level) {
	    level[i] = current_level;
	    if (do_sample(&ss[i], now, !current_level)) {
		inc_rsf(dual_control ? i : 0);
	    }
	}
    }
    return IRQ_HANDLED;
}

static unsigned get_rpm_from_unit(int num) {
    return sampler_get_rpm(&ss[num], get_usecs());
}

static int irq_num;
static unsigned long dummy;
int specific_fan_init(void) {
    int ret, i;

    if (is_name_prefix("CCR1072")
	|| is_board_type("ccr1036r2")
	|| is_board_type("ccr1016r2")) {
	return -1;
    }

    if (is_name_prefix("CCR1036-8G")
	|| is_name_prefix("CCR1036-12S")) {
	dual_control = 1;
    }

    for (i = 0; i < 2; i++) {
	reset_sampler(&ss[i]);
    }
    get_rpm = get_rpm_from_unit;

    irq_num = irq_alloc_hwirq(-1);
    if (irq_num <= 0) {
	printk("fancon: could not create_irq\n");
	return -ENXIO;
    }
    tile_irq_activate(irq_num, TILE_IRQ_PERCPU);
    ret = request_irq(irq_num, &fancon_irq, IRQF_SHARED, "Fancon", &dummy);
    if (ret) {
	printk("fancon: could not request_irq %u: %d\n", irq_num, ret);
	irq_free_hwirq(irq_num);
	return ret;
    }
    gpio_irq(irq_num, fan_sense_gpios());

    printk("fancon: got irq num %d\n", irq_num);
    return 0;
}

void specific_fan_start(void) {
}

void specific_fan_stop(void) {
}

void specific_fan_exit(void) {
    free_irq(irq_num, &dummy);
    irq_free_hwirq(irq_num);
}
