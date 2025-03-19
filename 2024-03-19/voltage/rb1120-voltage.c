#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/voltage.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <asm/rb_aux.h>

extern struct MeasurementData measurement;

static struct SamplerData lo_sampler;
static struct SamplerData hi_sampler;

static unsigned termo[2] = { 0, 0 };
static unsigned electro[2] = { 0, 0 };

static unsigned pulse_dummy;
static unsigned pulse_int_num = 0;

static int has_current_monitor = 1;

enum {
    CURRENT_MONITOR = 0,
    CPU_TEMPERATURE,
    BOARD_TEMPERATURE,
    VOLTAGE_MONITOR,
};

static void pulse_exit(void) {
    release_intr(pulse_int_num, &pulse_dummy);
}

static void change_monsel(int source, int i) {
    static unsigned monsel[] = { 0, 0 };

    if (!monsel[i]) {
	char name[] = "monsel0";
	name[6] = name[6] + i;
	monsel[i] = get_gpio_def(name);
//	printk("voltage: monsel[%u] = %u\n", i, monsel[i]);
    }
    if (monsel[i]) {
	gpio_set_value(monsel[i], (source >> i) & 1);
    }
}

static void change_source(unsigned value) {
    static int source = -1;

    // printk("voltage: S:%u\tM:%u\n", source, value);
    switch (source) {
    case CPU_TEMPERATURE:
	termo[1] = value;
	// printk("cpu temperature reading: %u\n", value);
	source = VOLTAGE_MONITOR;
	break;
    case VOLTAGE_MONITOR:
	electro[1] = value;
	source = has_current_monitor ? CURRENT_MONITOR : BOARD_TEMPERATURE;
	break;
    case CURRENT_MONITOR:
	electro[0] = value;
	source = BOARD_TEMPERATURE;
	break;
    case BOARD_TEMPERATURE:
	termo[0] = value;
	// printk("board temperature reading: %u\n", value);
	// fall through
    case -1:
	source = CPU_TEMPERATURE;
	break;
    default:
	printk("voltage: this is serious error!\n");
	break;
    }

    change_monsel(source, 0);
    change_monsel(source, 1);
}

static void pulse_timeout(struct timer_list *t) {
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

static int invType = IRQF_TRIGGER_RISING;
static int type = IRQF_TRIGGER_FALLING;
static irqreturn_t pulse_intr(int irq, void *data) {
    if (type == invType) {
	type = IRQF_TRIGGER_FALLING;
	reset_time(&hi_sampler);
	sample_simple_data(&lo_sampler);
    }
    else {
	type = invType;
	reset_time(&lo_sampler);
	sample_simple_data(&hi_sampler);
    }
    irq_set_irq_type(pulse_int_num, type);
    return IRQ_HANDLED;
}

int init_rb1120_voltage(int use_current_monitor, int style) {
    int ret;
    printk("init_rb1120_voltage\n");    

    measurement.style = style;
    measurement.voltage_ptr = (char *) electro;
    measurement.voltage_size = sizeof(electro);
    measurement.temperature_ptr = (char *) termo;
    measurement.temperature_size = sizeof(termo);
    measurement.timeout = pulse_timeout;
    measurement.specific_exit = pulse_exit;
    has_current_monitor = use_current_monitor;

    reset_sampler(&lo_sampler);
    reset_sampler(&hi_sampler);

    ret = init_interrupt("pulse", &pulse_int_num, &pulse_dummy, &pulse_intr);
    printk("pulse_int_num = %08x\n", pulse_int_num);

    if (ret < 0) {
	unsigned num = get_gpio_def("monpulse");
	pulse_int_num = gpio_to_irq(num);
	invType = IRQ_TYPE_EDGE_BOTH;
	return request_irq(pulse_int_num, &pulse_intr, type, 
			   "pulse monitor", &pulse_dummy);
    }
    else {
	return ret;
    }
}


