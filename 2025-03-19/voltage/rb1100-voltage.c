#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/voltage.h>
#include <linux/irq.h>
#include <linux/of_device.h>
#include <asm/rb_aux.h>

enum {
    CURRENT_MONITOR,
    BOARD_TEMPERATURE,
    CPU_TEMPERATURE,
    VOLTAGE_MONITOR,
};

static unsigned pulse_dummy;
static unsigned pulse_int_num = 0;

extern struct MeasurementData measurement;

static void rb1100_pulse_exit(void) {
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

static unsigned source = VOLTAGE_MONITOR;

static void change_source(void) {
    do { /*
	  * It was decided not to implement current monitor on RB1100,
	  * To avoid using extra 0 ohm resistor it must be disabled in driver
	  */
	source = (source + 1) & 3;
    } while (source == CURRENT_MONITOR);
    change_latch(source << 4, 3 << 4);    
}

void rb1100_pulse_timeout(struct timer_list *t) {
    unsigned sample;
    unsigned long flags;
    local_irq_save(flags);
    sample = calculate_n_reset(&sampler);    
    switch (source) {
    case CURRENT_MONITOR:
	electro[0] = sample;
	break;
    case BOARD_TEMPERATURE:
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

    mod_timer(&measurement.timer, jiffies + HZ / 2);
    local_irq_restore(flags);
}

int init_rb1100_voltage(void) {
    printk("init_rb1100_voltage\n");    

    change_source();

    measurement.style = RB1100_STYLE;
    measurement.voltage_ptr = (char *) electro;
    measurement.voltage_size = sizeof(electro);
    measurement.temperature_ptr = (char *) termo;
    measurement.temperature_size = sizeof(termo);
    measurement.timeout = rb1100_pulse_timeout;
    measurement.specific_exit = rb1100_pulse_exit;

    reset_sampler(&sampler);

    init_interrupt("pulse", &pulse_int_num, &pulse_dummy, &pulse_intr);

    return 0;
}
