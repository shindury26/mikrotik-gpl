#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/voltage.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>

static unsigned termo_dummy;
static unsigned termo_int_num = 0;
static unsigned voltage_dummy;
static unsigned voltage_int_num = 0;

extern struct MeasurementData measurement;

void release_intr(unsigned intr_num, unsigned *dummy) {
    if (intr_num) free_irq(intr_num, dummy);
}

static void rb800_voltage_exit(void) {
    release_intr(termo_int_num, &termo_dummy);
    release_intr(voltage_int_num, &voltage_dummy);
}

static struct SamplerData vs;
static struct SamplerData ts;

static unsigned voltage[2] = { 0, 0 };
static unsigned voltage_irq_type = IRQF_TRIGGER_FALLING;
static irqreturn_t voltage_intr(int irq, void *data) {
    voltage_irq_type = 
	(voltage_irq_type == IRQF_TRIGGER_RISING)
	? IRQF_TRIGGER_FALLING
	: IRQF_TRIGGER_RISING;
    irq_set_irq_type(voltage_int_num, voltage_irq_type);

    sample_paired_data(&vs);
    return IRQ_HANDLED;
}

static unsigned termo[2] = { 0, 0 };
static irqreturn_t termo_intr(int irq, void *data) {
    sample_paired_data(&ts);
    return IRQ_HANDLED;
}

void rb800_voltage_timeout(struct timer_list *tx) {
    unsigned long flags;
    local_irq_save(flags);
    
    termo[1] = calculate_n_reset(&ts);
    voltage[1] = calculate_n_reset(&vs);

    mod_timer(&measurement.timer, jiffies + HZ / 2);
    local_irq_restore(flags);
}

int init_interrupt(const char *name,
		    unsigned *intr_num, 
		    unsigned *dummy, 
		    void *handler) {
    struct device_node *node;
    if ((node = of_find_node_by_name(NULL, name))) {
	*intr_num = irq_of_parse_and_map(node, 0);
	if (*intr_num) {
	    int type = IRQF_TRIGGER_FALLING;
	    if (request_irq(*intr_num, handler, type, name, dummy)) {
		*intr_num = 0;
	    }
	}	
	of_node_put(node);
	return 0;
    }
    else {
	return -EINVAL;
    }
}

int init_rb800_voltage(void) {
    measurement.style = RB800_STYLE;
    measurement.voltage_ptr = (char *) voltage;
    measurement.voltage_size = sizeof(voltage);
    measurement.temperature_ptr = (char *) termo;
    measurement.temperature_size = sizeof(termo);
    measurement.timeout = rb800_voltage_timeout;
    measurement.specific_exit = rb800_voltage_exit;

    reset_sampler(&vs);
    reset_sampler(&ts);

    init_interrupt("voltage", &voltage_int_num, &voltage_dummy, &voltage_intr);
    init_interrupt("temperature", &termo_int_num, &termo_dummy, &termo_intr);

    return 0;
}
