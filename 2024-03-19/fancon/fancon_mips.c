/*
 * Fan control driver for RB400
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/notifier.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <asm/bootinfo.h>
#include <asm/rb/boards.h>

void rb400_register_fancon(void);

void (*mips_main_fan)(void);
void (*mips_aux_fan)(void);
void (*mips_fan_start)(void);
void (*mips_fan_stop)(void);

void main_fan(int num) {
    mips_main_fan();
}

void aux_fan(int num) {
    mips_aux_fan();
}

void specific_fan_start(void) {
    mips_fan_start();
}

void specific_fan_stop(void) {
    mips_fan_stop();
}

int specific_fan_init(void) {
#ifdef CONFIG_CPU_BIG_ENDIAN
    if (mips_machgroup == MACH_GROUP_MT_RB400) {
	switch (mips_machtype) {
	    /* all boards that have fans falls through */
	case MACH_MT_RB433:
	case MACH_MT_RB433U:
	case MACH_MT_RB435G:
	case MACH_MT_RB493:
	case MACH_MT_RB493G:
	    rb400_register_fancon();
	    break;
	default:
	    return -1;
	}
    }
    else
#endif
	return -1;

    return 0;
}

void specific_fan_exit(void) {
    specific_fan_stop();
}
