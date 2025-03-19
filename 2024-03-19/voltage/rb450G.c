#include <linux/voltage.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/flash.h>
#include <asm/rb/rb400.h>
#include <asm/rb/boards.h>

extern unsigned long mips_machtype;

#define VOLTAGE_IRQ_NUM		(GPIO_IRQ_BASE + ffs(voltage_pin) - 1)
#define TEMPERATURE_IRQ_NUM	(GPIO_IRQ_BASE + ffs(temperature_pin) - 1)

#define GPIO(x)			(1 << (x))
#define GPIO_BASE		0x18040000

static unsigned voltage_pin = 0;
static unsigned temperature_pin = 0;
static void __iomem *voltage_base;

#define GPIO_OUTPUT_EN		((unsigned long) voltage_base + 0x00)
#define GPIO_INPUT		((unsigned long) voltage_base + 0x04)
#define GPIO_SET		((unsigned long) voltage_base + 0x0c)
#define GPIO_CLEAR		((unsigned long) voltage_base + 0x10)
#define GPIO_FUNCTION		((unsigned long) voltage_base + 0x28)

static struct SamplerData vs;
static struct SamplerData ts;

extern struct MeasurementData measurement;
uint8_t switch_cmd = SPI_CMD_MON_TEMP;

static unsigned dummy_dev = 0;
static unsigned voltage[2] = { 0, 0 };
static unsigned temperature[2] = { 0, 0 };
/*
 * On boards where Xilinx switches between voltage and temperature
 * time_is_right variable indicates that Xilinx has been switched
 */
static int time_is_right = 0;
static int is_voltage = 1;
static int spi_done = 1;

static struct msg_async {
    struct spi_message m;
    struct spi_transfer t;
} msg;

static void rb400_set_bits(unsigned reg, unsigned bits) {
    rb400_writel(rb400_readl(reg) | bits, reg);
}

static void rb400_clear_bits(unsigned reg, unsigned bits) {
    rb400_writel(rb400_readl(reg) & (~bits), reg);
}

void muxed_voltage_timeout(struct timer_list *t) {
    unsigned long flags;
    struct spi_device *spi;
    local_irq_save(flags);
    time_is_right = 0;
    spi = rb400_spi_get();
    if (spi && spi_done) {
	spi_done = 0;
	spi_async(spi, &msg.m);
    }
    local_irq_restore(flags);
}

void simple_voltage_timeout(struct timer_list *t) {
    voltage[1] = calculate_n_reset(&vs);
    if (temperature_pin) temperature[1] = calculate_n_reset(&ts);
    mod_timer(&measurement.timer, jiffies + HZ);
}

static unsigned switch_gpio = 0;
void rb7xx_voltage_timeout(struct timer_list *t) {
    if (is_voltage) {
	voltage[1] = calculate_n_reset(&vs);
	rb400_writel(switch_gpio, GPIO_CLEAR);	
    }
    else {
	temperature[1] = calculate_n_reset(&ts);
	rb400_writel(switch_gpio, GPIO_SET);
    }
    is_voltage = !is_voltage;
    mod_timer(&measurement.timer, jiffies + HZ);
}

static irqreturn_t v_intr_handler(int irq, void *data) {
    if (time_is_right) sample_paired_data(is_voltage ? &vs : &ts);
    return IRQ_HANDLED;
}

static irqreturn_t t_intr_handler(int irq, void *data) {
    if (time_is_right) sample_paired_data(&ts);
    return IRQ_HANDLED;
}

static int v_irq_state = -EINVAL;
static int t_irq_state = -EINVAL;

void rb450G_free_irq(int *state, int num) {
    if (*state >= 0) {
	free_irq(num, &dummy_dev);
	*state = -EINVAL;
    }
}

void rb450G_exit(void) {
    rb450G_free_irq(&v_irq_state, VOLTAGE_IRQ_NUM);
    rb450G_free_irq(&t_irq_state, TEMPERATURE_IRQ_NUM);
    iounmap(voltage_base);
}

static void spi_async_call_back(void *msg) {    
    unsigned long flags;
    local_irq_save(flags);    
    if (is_voltage) {
	voltage[1] = calculate_n_reset(&vs);
    }
    else {
	temperature[1] = calculate_n_reset(&ts);
    }
    switch_cmd = is_voltage ? SPI_CMD_MON_VOLTAGE : SPI_CMD_MON_TEMP;
    mod_timer(&measurement.timer, jiffies + HZ);
    is_voltage = !is_voltage;
    time_is_right = 1;
    spi_done = 1;

    local_irq_restore(flags);
}

int init_rb450G_voltage(void) {
    printk("init_rb450G_voltage\n");
    voltage_base = ioremap(GPIO_BASE, PAGE_SIZE);

    msg.t.tx_buf = &switch_cmd;
    msg.t.len = 1;
    spi_message_init(&msg.m);
    spi_message_add_tail(&msg.t, &msg.m);
    msg.m.complete = spi_async_call_back;
    msg.m.context = &msg;

    measurement.style = RB450G_STYLE;

    measurement.voltage_ptr = (char *) voltage;
    measurement.voltage_size = sizeof(voltage);
    measurement.temperature_ptr = (char *) temperature;
    measurement.temperature_size = sizeof(temperature);
    measurement.specific_exit = &rb450G_exit;

    reset_sampler(&vs);
    reset_sampler(&ts);

    if (is700group()) {
	if (hw_options & HW_OPT_HAS_VOLTAGE) {
	    switch_gpio = GPIO(9);
	    measurement.style = RB711_STYLE;
	}
	else if (mips_machtype == MACH_MT_RB_OMNI) {
	    switch_gpio = GPIO(2);
	    measurement.style = RB_OMNI_STYLE;
	}
	else {
	    return -EINVAL;
	}
	voltage_pin = GPIO(10);
    }
    else if (is411L() || mips_machtype == MACH_MT_RB411G) {
	voltage_pin = GPIO(10);
	rb400_clear_bits(GPIO_OUTPUT_EN, voltage_pin);
    }
    else if (is493G()) {
	voltage_pin = GPIO(3);
    }
    else {
	voltage_pin = GPIO(11);
    }

    if (is450G() || is493G()) {
	measurement.timeout = &muxed_voltage_timeout;
    }
    else {
	time_is_right = 1; // on other boards time is always right
	if (switch_gpio) {
	    measurement.timeout = &rb7xx_voltage_timeout;
	    rb400_set_bits(GPIO_OUTPUT_EN, switch_gpio);
	    rb400_writel(switch_gpio, GPIO_SET);
	}
	else {
	    measurement.timeout = &simple_voltage_timeout;
	}
    }
    
    v_irq_state = request_irq(VOLTAGE_IRQ_NUM, &v_intr_handler,
			      IRQF_SHARED, "voltage", &dummy_dev);

    if (v_irq_state == 0 && temperature_pin) {
	t_irq_state = request_irq(TEMPERATURE_IRQ_NUM, &t_intr_handler,
				  IRQF_SHARED, "temperature", &dummy_dev);
    }

    return v_irq_state != 0 || !temperature_pin ? v_irq_state : t_irq_state;
}

