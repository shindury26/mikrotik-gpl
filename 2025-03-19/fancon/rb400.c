#include <linux/interrupt.h>
#include <asm/addrspace.h> 
#include <asm/rb/rb400.h>

#define FANCON_PIN		11
#define FANCON_IRQ_NUM		(GPIO_IRQ_BASE + FANCON_PIN)

static unsigned dummy_dev;
extern void inc_rsf(int num);
extern unsigned (*get_rpm)(int);
extern int get_last_rsf(int num);
extern int get_spinning(int num);

extern void (*mips_main_fan)(void);
extern void (*mips_aux_fan)(void);
extern void (*mips_fan_start)(void);
extern void (*mips_fan_stop)(void);

static void rb400_main_fan(void) {
    rb400_change_cfg(CFG_BIT_FAN, 0);
}

static void rb400_aux_fan(void) {
    rb400_change_cfg(0, CFG_BIT_FAN);
}

static irqreturn_t rb400_fan_intr(int irq, void *data) {
    inc_rsf(0);
    return IRQ_HANDLED;
}

static void rb400_fan_start(void) {
    if (request_irq(FANCON_IRQ_NUM,
		    &rb400_fan_intr,
		    IRQF_SHARED,
		    "fancon",
		    &dummy_dev)) {
	printk("fancon: interrupt allocation failed\n");
    }
    else {
	printk("fancon: interrupt ok\n");
    }
}

static void rb400_fan_stop(void) {
    free_irq(FANCON_IRQ_NUM, &dummy_dev);
}

static unsigned get_rpm_simple(int num) {
    return 30 * get_spinning(num) * get_last_rsf(num);
}

void rb400_register_fancon(void) {
    mips_main_fan = &rb400_main_fan;
    mips_aux_fan = &rb400_aux_fan;
    mips_fan_start = &rb400_fan_start;
    mips_fan_stop = &rb400_fan_stop;
    get_rpm = get_rpm_simple;
}
