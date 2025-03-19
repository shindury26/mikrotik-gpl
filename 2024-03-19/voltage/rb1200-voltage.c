#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/voltage.h>
#include <linux/irq.h>

#include <asm/time.h>

void change_latch(unsigned char set, unsigned char clear) { }

static unsigned pulse_dummy;
static unsigned pulse_int_num = 0;

extern struct MeasurementData measurement;

static void rb1200_pulse_exit(void) {
    release_intr(pulse_int_num, &pulse_dummy);
}

static struct SamplerData sampler;
static unsigned termo[2] = { 0, 0 };
static unsigned electro[2] = { 0, 0 };

static int type = IRQF_TRIGGER_FALLING;
static irqreturn_t pulse_intr(int irq, void *data) {
    type = (type == IRQF_TRIGGER_RISING)
		? IRQF_TRIGGER_FALLING
		: IRQF_TRIGGER_RISING;
    irq_set_irq_type(pulse_int_num, type);
    sample_paired_data(&sampler);
    return IRQ_HANDLED;
}

enum {
    CPU_TEMPERATURE = 0,
    VOLTAGE_MONITOR = 1,
    PCB_TEMPERATURE = 2,
};

static unsigned source = CPU_TEMPERATURE;

void switch_monitor(unsigned state);

void change_source(void) {
    source = (source >= 2) ? 0 : (source + 1);
    switch_monitor(source);
}

void rb1200_pulse_timeout(struct timer_list *t) {
    int delay = 0;
    unsigned sample;
    unsigned long flags;    
    local_irq_save(flags);
    sample = calculate_n_reset(&sampler);

    if (!delay) {
	switch (source) {
	case PCB_TEMPERATURE:
	    termo[0] = sample;
	    break;
	case CPU_TEMPERATURE:
	    termo[1] = sample;
	    break;
	case VOLTAGE_MONITOR:
	    electro[1] = sample;
	    break;
	}
	change_source();
    }

    mod_timer(&measurement.timer, jiffies + HZ / 2);
    delay = !delay;
    local_irq_restore(flags);
}

int init_rb1200_voltage(void) {
    printk("init_rb1200_voltage\n");    

    change_source();

    measurement.style = RB1200_STYLE;
    measurement.voltage_ptr = (char *) electro;
    measurement.voltage_size = sizeof(electro);
    measurement.temperature_ptr = (char *) termo;
    measurement.temperature_size = sizeof(termo);
    measurement.timeout = rb1200_pulse_timeout;
    measurement.specific_exit = rb1200_pulse_exit;

    reset_sampler(&sampler);

    init_interrupt("pulse", &pulse_int_num, &pulse_dummy, &pulse_intr);

    return 0;
}
