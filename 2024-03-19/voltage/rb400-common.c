#include <linux/sort.h>
#include <linux/voltage.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>
#include <asm/rb/rb400.h>

#define SAMPLES			12
#define BORDER			3

extern struct MeasurementData measurement;

static unsigned last[2] = { 0, 0 };

static unsigned reading;
static unsigned spi_done = 1;
const uint8_t voltage_cmd = SPI_CMD_READ_VOLTS;

static struct msg_async {
    struct spi_message m;
    struct spi_transfer t[2];
} msg;

static void get_reading(void) {
    struct spi_device *spi = rb400_spi_get();
    if (spi && spi_done) {
	spi_done = 0;
	spi_async(spi, &msg.m);
    }
}

static int cmp(const void *a, const void *b) {
    const unsigned *x = a;
    const unsigned *y = b;    
    return *x > *y ? 1 : (*x < *y ? -1 : 0);
}

int init_rb450G_voltage(void);

static struct workqueue_struct *voltage_work_queue;

static void do_register_device(struct work_struct *work) {
    register_voltage_device();
}

static void do_fallback_to_other_driver(struct work_struct *work) {
    voltage_exit();
    device_state = CAN_REGISTER_DEVICE;
    init_rb450G_voltage();
    voltage_start(0);
}

DECLARE_WORK(register_device,
	     do_register_device);
DECLARE_WORK(fallback_to_other_driver,
	     do_fallback_to_other_driver);

static void spi_async_call_back(void *msg) {
    static int failures = 0;
    static int startup = SAMPLES;
    static unsigned array[SAMPLES];

    reading = swab(reading);
    if (reading != 0 && reading != 0xffffffff) {
	failures = 0;
	if (device_state == DO_NOT_REGISTER_DEVICE
	    && !work_pending(&register_device)) {
	    /* it works, now we can register device */
	    queue_work(voltage_work_queue, &register_device);
	}
	++last[0];
	if (last[0] > 1 && last[0] < SAMPLES) last[1] = reading;
	startup--;	
	array[startup] = reading;
	if (startup == 0) {
	    last[1] = 0;
	    sort(array, SAMPLES, sizeof(unsigned), cmp, NULL);
	    last[1] = (array[BORDER] + array[SAMPLES - BORDER]) / 2;
	    startup = SAMPLES;
	}
    }
    else {
	printk("failure: %08x\n", reading);
	failures++;
	if (failures > 3) {
	    /* this voltage monitor does not work, try other one */
	    if (!work_pending(&fallback_to_other_driver)) {
		queue_work(voltage_work_queue, &fallback_to_other_driver);
	    }
	    return;
	}
    }
    mod_timer(&measurement.timer, jiffies + (HZ / 2));
    spi_done = 1;
}

void rb400_common_voltage_timeout(struct timer_list *t) {
    unsigned long flags;
    local_irq_save(flags);
    get_reading();
    local_irq_restore(flags);
}

int init_rb400_common_voltage(void) {
    printk("init_rb400_common_voltage\n");

    /*
     * do not register device for now, 
     * first make sure that driver works
     */
    device_state = DO_NOT_REGISTER_DEVICE;

    voltage_work_queue = create_singlethread_workqueue("voltage_work_queue");  

    measurement.style = RB400_COMMON_STYLE;
    measurement.voltage_ptr = (char *) last;
    measurement.voltage_size = sizeof(last);
    measurement.timeout = &rb400_common_voltage_timeout;

    msg.t[0].tx_buf = &voltage_cmd;
    msg.t[0].len = 1;
    msg.t[1].rx_buf = (uint8_t *) &reading;
    msg.t[1].len = 4;
    spi_message_init(&msg.m);
    spi_message_add_tail(&msg.t[0], &msg.m);
    spi_message_add_tail(&msg.t[1], &msg.m);	
    msg.m.complete = spi_async_call_back;
    msg.m.context = &msg;
    return 0;
}

