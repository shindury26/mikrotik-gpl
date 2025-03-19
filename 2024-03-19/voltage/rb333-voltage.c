#include <linux/voltage.h>
#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <asm/time.h>
#include <asm/io.h>

extern struct MeasurementData measurement;

static unsigned char *gtm = NULL;
static unsigned clock_rate = 0;

#define GTCFR2		((unsigned *)(gtm + 0x04))
#define GTMDR3_4	((unsigned *)(gtm + 0x20))
#define GTCNR3_4	((unsigned *)(gtm + 0x2c))

#define GTCFR2_VAL	0x99000000   
#define GTCFR2_ENABLE	0x10000000   
#define GTMDR3_4_VAL	0x00000003

#define GPIO(x)		(0x80000000 >> (x))

#define LAST_CLOCK	0
#define LAST_MEASURE	1

static unsigned last[2] = { 0, 0 };

static unsigned get_counter(void) {
    return (jiffies + 1) * tb_ticks_per_jiffy - get_dec();
}

void rb333_voltage_timeout(struct timer_list *t) {
    unsigned long flags;
    local_irq_save(flags);
    if (in_be32(GTCFR2) == GTCFR2_VAL) {
	last[LAST_MEASURE] = in_be32(GTCNR3_4);
	last[LAST_CLOCK] = get_counter() - clock_rate;
    }
    else {
        if (in_be32(GTCFR2) & GTCFR2_ENABLE) {
	    /* 
	     *  voltage monitor shares timer with beeper
	     * if beeper is running try again little later
	     */
            mod_timer(&measurement.timer, jiffies + (HZ >> 4));
	    goto unlock;
        }  
    }
    out_be32(GTCFR2, GTCFR2_VAL);
    out_be32(GTMDR3_4, GTMDR3_4_VAL);    
    out_be32(GTCNR3_4, 0);
    clock_rate = get_counter();
    mod_timer(&measurement.timer, jiffies + HZ);
  unlock:
    local_irq_restore(flags);
}

int init_rb333_voltage(void) {
    int retval = -EINVAL;
    struct resource res;
    struct device_node *beeper;

    measurement.style = RB333_STYLE;
    measurement.voltage_ptr = (char *) last;
    measurement.voltage_size = sizeof(last);
    measurement.timeout = rb333_voltage_timeout;
	
    beeper = of_find_node_by_name(NULL, "beeper");
    if (beeper) {
	retval = of_address_to_resource(beeper, 0, &res);
	of_node_put(beeper);
	if (!retval) {
	    printk("voltage: map %08x-%08x\n",
		   (unsigned int) res.start,
		   (unsigned int) res.end);
	    gtm = ioremap(res.start, res.end - res.start + 1);
	}
	else {
	    printk("voltage: no region specified\n");
	}
    }
    else {
	printk("voltage: node not found\n");
    }	    
    
    return retval;
}













