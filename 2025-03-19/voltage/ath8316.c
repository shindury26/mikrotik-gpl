#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/voltage.h>
#include <asm/rb/boards.h>
#include <asm/rb/rb400.h>

#define AG7100_ADDR_SHIFT	8

#define AG7100_MII_MGMT_CMD	0x24
#define AG7100_MII_MGMT_ADDRESS	0x28
#define AG7100_MII_MGMT_CTRL	0x2c
#define AG7100_MII_MGMT_STATUS	0x30
#define AG7100_MII_MGMT_IND	0x34

#define AG7100_MGMT_CMD_READ	0x1

#define REG(reg)		(* (volatile unsigned long *) (reg))

#define GPIO_BASE		0x18040000
#define AR7100_GE0_BASE		0x19000000
#define MONITOR_IRQ_NUM		(GPIO_IRQ_BASE + ffs(pin) - 1)

static unsigned voltage_base;
static unsigned ge0_base = 0;

#define GPIO_OUTPUT_EN		(voltage_base + 0x00)
#define GPIO_INPUT		(voltage_base + 0x04)
#define GPIO_OUTPUT		(voltage_base + 0x08)

extern struct MeasurementData measurement;

static struct SamplerData lo_sampler;
static struct SamplerData hi_sampler;
static unsigned termo[2] = { 0, 0 };
static unsigned electro[2] = { 0, 0 };
static unsigned pin = 1 << 3;
static unsigned dummy_dev = 0;

enum {
    VOLTAGE_MONITOR,
    CURRENT_MONITOR,
    BOARD_TEMPERATURE,
};

static int ag7100_mii_read(int phy, int reg) {
    unsigned short addr = (phy << AG7100_ADDR_SHIFT) | reg, val;
    int rddata;
    int ii = 1000000;

    REG(ge0_base + AG7100_MII_MGMT_CMD) = 0;
    REG(ge0_base + AG7100_MII_MGMT_ADDRESS) = addr; 
    REG(ge0_base + AG7100_MII_MGMT_CMD) = AG7100_MGMT_CMD_READ;

    do {
        rddata = REG(ge0_base + AG7100_MII_MGMT_IND) & 0x1;
    } while (rddata && --ii);

    val = REG(ge0_base + AG7100_MII_MGMT_STATUS);
    REG(ge0_base + AG7100_MII_MGMT_CMD) = 0;
    return val;
}

static void ag7100_mii_write(int phy, int reg, int data) {
    int rddata;
    int ii = 1000000;

    REG(ge0_base + AG7100_MII_MGMT_ADDRESS) = (phy << AG7100_ADDR_SHIFT) | reg;
    REG(ge0_base + AG7100_MII_MGMT_CTRL) = data;

    do {
        rddata = REG(ge0_base + AG7100_MII_MGMT_IND) & 0x1;
    } while (rddata && --ii);
}

static int mii_read_raw(int phy, int reg) {
    phy &= 0x1f;
    reg &= 0x1f;
    return ag7100_mii_read(phy, reg);
}

static void mii_write_raw(int phy, int reg, int data) {
    phy &= 0x1f;
    reg &= 0x1f;
    data &= 0xffff;
    return ag7100_mii_write(phy, reg, data);
}

static unsigned ath_read_reg(unsigned addr) {
    unsigned data;
    unsigned long flags;
    local_irq_save(flags);

    mii_write_raw(0x18, 0, addr >> 9);
    data = mii_read_raw(0x10 | ((addr >> 6) & 0x7), addr >> 1);
    data |= mii_read_raw(0x10 | ((addr >> 6) & 0x7), (addr >> 1) | 1) << 16;

    local_irq_restore(flags);
    return data;
}

static void ath_write_reg(unsigned addr, unsigned val) {
    unsigned long flags;
    local_irq_save(flags);

    mii_write_raw(0x18, 0, addr >> 9);
    mii_write_raw(0x10 | ((addr >> 6) & 0x7), (addr >> 1) | 1, val >> 16);
    mii_write_raw(0x10 | ((addr >> 6) & 0x7), addr >> 1, val);

    local_irq_restore(flags);
}

// exponential moving average
static unsigned ema(unsigned value, unsigned history, unsigned fraction) {
    if (history) {
	return value / fraction + history * (fraction - 1) / fraction;
    }
    else {
	return value;
    }
}

extern unsigned long mips_machtype;
static unsigned switch_pin = 0;

#define RB953_SWITCH_PIN 0x2000

static void change_source(unsigned value) {
    static unsigned termo_history = 0;
    static int source = -1;
    static unsigned reg_last = 0;
    unsigned reg;
    if (mips_machtype == MACH_MT_RB953GS) {
	static int current_state = RB953_SWITCH_PIN;
	if (current_state) electro[1] = value; else termo[1] = value;
	int new_state = current_state ^ RB953_SWITCH_PIN;
	rb_change_cfg(current_state, new_state);
	current_state = new_state;
	return;
    }
    if (switch_pin) {
	if (REG(GPIO_OUTPUT) & switch_pin) {
	    REG(GPIO_OUTPUT) &= ~switch_pin;
	    electro[1] = value;
	}
	else {
	    REG(GPIO_OUTPUT) |= switch_pin;
	    termo[1] = value;
	}
	return;
    }
    if (is433L_series() || is411L()) {
	electro[1] = value;
	return;
    }

    reg = ath_read_reg(0xB4);
    if (reg != reg_last && reg_last != 0) {
	printk("voltage: ath8316 reset, resample source %d\n", source);
	reg_last = (reg & 0x3FFF3FFF) | (reg_last & 0xC000C000);
	ath_write_reg(0xB4, reg_last);
	return;
    }
    reg &= ~0xC000C000;

    switch (source) {
    case VOLTAGE_MONITOR:
	// printk("VOLTAGE_MONITOR %u\n", value);
	electro[1] = value;
	source = CURRENT_MONITOR;
	reg |= 0x80008000;
	break;
    case CURRENT_MONITOR:
	// printk("CURRENT_MONITOR %u\n", value);
	electro[0] = value;
	source = BOARD_TEMPERATURE;
	reg |= 0x00008000;
	break;
    case BOARD_TEMPERATURE:
	termo_history = ema(value, termo_history, 4);
	// printk("BOARD_TEMPERATURE %u %u\n", value, termo_history);
	termo[0] = termo_history;
	// fall through
    case -1:
	source = VOLTAGE_MONITOR;
	reg |= 0x80000000;
	break;
    default:
	printk("voltage: this is serious error!\n");
	break;
    }
    reg_last = reg;
    ath_write_reg(0xB4, reg);
}

static void rb435G_pulse_timeout(struct timer_list *t) {
    static int delay = 0;

    unsigned lo, hi;
    unsigned long flags;    
    local_irq_save(flags);
    hi = calculate_n_reset(&hi_sampler);
    lo = calculate_n_reset(&lo_sampler);

    if (!delay) {
	unsigned div = (lo + hi);
	change_source(div ? 100000 * lo / div : 0);
	// change source and give it a time to sink in
	mod_timer(&measurement.timer, jiffies + HZ / 2);
    }
    else {
	mod_timer(&measurement.timer, jiffies + HZ);
    }
    delay = !delay;
    local_irq_restore(flags);
}

static void rb435G_pulse_exit(void) {
    free_irq(MONITOR_IRQ_NUM, &dummy_dev);
    iounmap((const volatile void *) voltage_base);
    iounmap((const volatile void *) ge0_base);
}

static irqreturn_t intr_handler(int irq, void *data) {
    if (REG(GPIO_INPUT) & pin) {
	reset_time(&hi_sampler);
	sample_simple_data(&lo_sampler);
    }
    else {
	reset_time(&lo_sampler);
	sample_simple_data(&hi_sampler);
    }
    return IRQ_HANDLED;
}

int init_ath8316(void) {
    int ret;
    printk("init_ath8316\n");    
    voltage_base = (unsigned) ioremap(GPIO_BASE, PAGE_SIZE);
    
    if (mips_machtype == MACH_MT_RB953GS) {
	pin = BIT(11);
	REG(GPIO_OUTPUT_EN) |= pin;
	rb_change_cfg(0, RB953_SWITCH_PIN);
    }
    else if (is433L_series() || is411L()) {
	pin = 1 << 10;
	REG(GPIO_OUTPUT_EN) &= ~pin;
    }
    else if (sxt_has_duty_cycle_voltage()) {
	pin = 1 << 10;
	switch_pin = 1 << 9;
	REG(GPIO_OUTPUT_EN) &= ~pin;
	REG(GPIO_OUTPUT_EN) |= switch_pin;
	if (mips_machtype == MACH_MT_RB912G
	    || mips_machtype == MACH_MT_GROOVE52) {
	    pin = 1 << 9;
	    switch_pin = 1 << 10;
	    REG(voltage_base + 0x34) &= ~(0xff << 16);
	}
	REG(GPIO_OUTPUT) |= switch_pin;
    }
    else {
	ge0_base = (unsigned) ioremap(AR7100_GE0_BASE, PAGE_SIZE);
    }

    measurement.style = RB435G_STYLE;
    measurement.voltage_ptr = (char *) electro;
    measurement.voltage_size = sizeof(electro);
    measurement.temperature_ptr = (char *) termo;
    measurement.temperature_size = sizeof(termo);
    measurement.timeout = rb435G_pulse_timeout;
    measurement.specific_exit = rb435G_pulse_exit;

    if (sxt_has_duty_cycle_voltage() || mips_machtype == MACH_MT_RB953GS) {
	measurement.style = RBSXTG_STYLE;
    }
    
    reset_sampler(&lo_sampler);
    reset_sampler(&hi_sampler);

    ret = request_irq(MONITOR_IRQ_NUM, &intr_handler,
		      IRQF_SHARED, "monitor", &dummy_dev);

    return ret;
}


