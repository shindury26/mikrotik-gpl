/*
 * Fan control driver for RB400
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
#include <linux/fan.h>

#define FAN_MINOR 250

struct fan_data {
    int rsf;
    int last_rsf;
    int main_works;
    int spinning;
};

int dual_control = 0;
static int running = 0;
static struct fan_data fans[2];

void main_fan(int num);
void aux_fan(int num);

void inc_rsf(int num) {
    fans[num].rsf++;
}

void set_fan(int num, int status) {
    (status ? main_fan : aux_fan)(num);
    fans[num].main_works = status ? 1 : 0;
}

int specific_fan_init(void);
void specific_fan_exit(void);
void specific_fan_start(void);
void specific_fan_stop(void);

struct timer_list fan_timer;

static unsigned rpm_not_implemented(int num) { return -1; }
unsigned (*get_rpm)(int num) = &rpm_not_implemented;

int get_last_rsf(int num) {
    return fans[num].last_rsf;
}

int get_spinning(int num) {
    return fans[num].spinning;
}

#define FOR_ALL_FANS \
    int i; \
    struct fan_data *fan = &fans[0]; \
    for (i = 0; i <= dual_control; fan = &fans[++i])
	
static void fan_timeout(struct timer_list *t) {
    FOR_ALL_FANS {
	if (!fan->rsf && running) {
	    set_fan(i, !fan->main_works);
	    fan->spinning = 0;
	}
	else {
	    fan->last_rsf = fan->rsf;
	    fan->spinning = 1;
	    fan->rsf = 0;
	}
    }
   
    mod_timer(&fan_timer, jiffies + HZ);
}

static void fan_start(void) {
    printk("fan: starting\n");
    if (running) return;
    specific_fan_start();
    running = 1;
}

static void fan_stop(void) {
    printk("fan: stoping\n");
    if (!running) return;
    specific_fan_stop();
    running = 0;
}

long fan_ioctl(struct file *file_p,
	       unsigned int cmd, 
	       unsigned long arg) {

//    printk("fan_ioctl, cmd: 0x%x, arg: 0x%lx\n", cmd, arg);
    switch (cmd) {
    case FAN_GET_RPM:
	if (arg < 2) return get_rpm(arg);
	return 0;
    case FAN_SWITCH: {
	int i;
	for (i = 0; i <= dual_control; ++i) {
	    set_fan(i, arg & SWITCH_FANx(i));
	}
	if (arg & SWITCH_MODE) fan_start(); else fan_stop();
	return 0;
    }
    case FAN_RESET_CNT: {
	FOR_ALL_FANS {
	    fan->rsf = fan->last_rsf = 0;
	}
	return 0;
    }
    case FAN_GET_INFO: {
	unsigned info = 0;
	FOR_ALL_FANS {
	    info |= ((running << INFO_RUNNING) 
		     | (fan->spinning << INFO_SPINNING) 
		     | ((!(fan->rsf + fan->last_rsf)) << INFO_FAILING)
		     | (fan->main_works << INFO_MAIN_OK)) << INFO_OFFSET(i);
	}
	return info;
    }
    default:
	return -1;
    }
}

static struct file_operations fan_fops = {
    owner:	THIS_MODULE,
#ifdef CONFIG_TILE
    compat_ioctl:	fan_ioctl,
#else
    unlocked_ioctl:	fan_ioctl,
#endif
};

static struct miscdevice fan_miscdev = {
    minor:	FAN_MINOR,
    name:	"fancon",
    fops:	&fan_fops,
};

static int fan_init(void) {
    int retval;
    int i;
    printk("fan: init\n");

    retval = misc_register(&fan_miscdev);
    if (retval) {
        printk("fan: failed to register char device\n");
        return retval;
    }
    
    retval = specific_fan_init();
    if (retval) {
	misc_deregister(&fan_miscdev);
	return retval;
    }

    timer_setup(&fan_timer, fan_timeout, 0);
    mod_timer(&fan_timer, jiffies + (HZ >> 2));

    for (i = 0; i <= dual_control; ++i) { set_fan(i, 1); }

    printk("fan: loaded\n");
    fan_start();

    return retval;
}

static void fan_exit(void) {
    unsigned long flags;

    misc_deregister(&fan_miscdev);

    del_timer(&fan_timer);
    local_irq_save(flags);
    specific_fan_exit();    
    local_irq_restore(flags);

    printk("fan: terminating\n");
}

module_init(fan_init);
module_exit(fan_exit);
