#include <asm/time.h>

#include <linux/errno.h>
#include <linux/of_device.h>
#include <linux/voltage.h>

extern unsigned long mips_machtype;

#if defined(CONFIG_4xx)
extern int init_rb1200_voltage(void);
#endif
extern int init_rb333_voltage(void);
extern int init_rb800_voltage(void);
extern int init_rb1100_voltage(void);

static const char *board_name;

static bool is_name_prefix(char *str) { 
    unsigned len = strlen(str);
    if (strlen(board_name) < len) return 0;    
    return (memcmp(board_name, str, len) == 0) ? 1 : 0;
}

unsigned get_usec_time(void) {
#if defined(CONFIG_4xx)
    long long ticks = mftbl();
#else
    long long ticks = mfspr(SPRN_TBWL);
#endif
    return div_u64(1000000 * ticks, ppc_tb_freq);
}

static inline unsigned mfspr_pvr(void) {
	unsigned rval;

	asm volatile("mfspr %0, 0x11f" : "=r" (rval));
	return rval;
}

#if !defined(CONFIG_4xx)
extern int init_rb1120_voltage(int, int);
#endif

static int run_specific_init(void) {
#if defined(CONFIG_4xx)
    if (is_name_prefix("RB12")) return init_rb1200_voltage();
#endif
    if (is_name_prefix("RB33")) return init_rb333_voltage();
    if (is_name_prefix("RB80")) return init_rb800_voltage();
#if !defined(CONFIG_4xx)
    if (is_name_prefix("RB85")) return init_rb1120_voltage(0, RB850G_STYLE);
#endif
    if (is_name_prefix("RB11")) {
#if !defined(CONFIG_4xx)
	if ((mfspr_pvr() & ~0xff) == 0x80211000) {
	    return init_rb1120_voltage(1, RB1120_STYLE);
	}
#endif
	return init_rb1100_voltage();
    }
    return -1;
}

static void get_board_model(void) {
    struct device_node *root = of_find_node_by_path("/");
    if (root) {
	int size;
	board_name = (char *) of_get_property(root, "model", &size);
	of_node_put(root);
    }
}

int specific_voltage_init(void) {
    get_board_model();
    return run_specific_init();
}
