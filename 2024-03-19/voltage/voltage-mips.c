#include <linux/types.h>
#include <asm/rb/boards.h>
#include <asm/mipsregs.h> 
#include <linux/errno.h>
#include <linux/flash.h>
#include <linux/module.h>

extern unsigned long mips_machtype;
extern unsigned mips_hpt_frequency;

static unsigned mips_usec_freq;
unsigned get_usec_time(void) {
    return read_c0_count() / mips_usec_freq;
}

int is400group(void) {
    return mips_machgroup == MACH_GROUP_MT_RB400;
}

int is450G(void) {
    return is400group() && mips_machtype == MACH_MT_RB450G;	
}

int is493G(void) {
    return is400group() && mips_machtype == MACH_MT_RB493G;	
}

int is435G(void) {
    return is400group() && mips_machtype == MACH_MT_RB435G;	
}

int is411L(void) {
    return (is400group() && (mips_machtype == MACH_MT_RB411L
			     || mips_machtype == MACH_MT_RB411UL));
}

int is411U(void) {
    return (is400group() 
	    && (mips_machtype == MACH_MT_RB411U
		|| (hw_options & HW_OPT_HAS_VOLTAGE)));
}

int is700group(void) {
    return mips_machgroup == MACH_GROUP_MT_RB700;
}

int is433L_series(void) {
    return mips_machtype == MACH_MT_RB433GL
	|| mips_machtype == MACH_MT_RB433UL
	|| mips_machtype == MACH_MT_RB411G
	|| mips_machtype == MACH_MT_RB433L;
}

extern int init_rb400_common_voltage(void);
extern int init_rb450G_voltage(void);
extern int init_ath8316(void);

extern unsigned hw_options;

int sxt_has_duty_cycle_voltage(void) {
    return mips_machtype == MACH_MT_RB_SXTG 
	|| mips_machtype == MACH_MT_RB711GT 
	|| (hw_options & HW_OPT_PULSE_DUTY_CYCLE);
}

int specific_voltage_init(void) {
    mips_usec_freq = mips_hpt_frequency / 1000000;
    if (mips_machgroup == MACH_GROUP_MT_VM
	|| mips_machgroup == MACH_GROUP_MT_MUSIC
	|| mips_machtype == MACH_MT_RB750G) return -EINVAL;
    
    if (mips_machgroup == MACH_GROUP_MT_RB700
	&& mips_machtype == MACH_MT_OMNI_SC) return -EINVAL;

    if (is435G() || is433L_series()
	|| sxt_has_duty_cycle_voltage()
	|| mips_machtype == MACH_MT_RB953GS) {
	return init_ath8316();
    }

    if ((mips_machtype == MACH_MT_RB411L
	 || mips_machtype == MACH_MT_RB493)
	&& !(hw_options & HW_OPT_HAS_VOLTAGE))  return -EINVAL;

    return (is450G() || is493G() || is411U() || is700group()
	    ? init_rb450G_voltage()
	    : init_rb400_common_voltage());
}
