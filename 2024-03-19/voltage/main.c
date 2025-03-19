#include <linux/miscdevice.h>
#include <linux/voltage.h>
#include <linux/module.h>

#define MEASUREMENTS  1
#define VOLTAGE_MINOR 252

MODULE_LICENSE("GPL");

struct MeasurementData measurement = { };

long voltage_ioctl(struct file *file_p,
		   unsigned int cmd, 
		   unsigned long arg) {
    int retval = -1;
    switch(cmd) {
    case GET_MEASURING_STYLE:
	retval = measurement.style;
	break;
    case GET_VOLTAGE:
	copy_to_user((char *) arg,
		     measurement.voltage_ptr, 
		     measurement.voltage_size);
	break;
    case GET_TEMPERATURE:	
	copy_to_user((char *) arg,
		     measurement.temperature_ptr, 
		     measurement.temperature_size);
	break;
    }
    return retval;
}

static struct file_operations voltage_fops = {
    owner:	THIS_MODULE,
    unlocked_ioctl:	voltage_ioctl,
};

static struct miscdevice voltage_miscdev = {
    minor:	VOLTAGE_MINOR,
    name:	"voltage",
    fops:	&voltage_fops,
};

void specific_voltage_exit(void);

int device_state = CAN_REGISTER_DEVICE;

void voltage_exit(void) {
    del_timer(&measurement.timer);
    measurement.specific_exit();
    if (device_state == DEVICE_REGISTERED) {
	misc_deregister(&voltage_miscdev);
	device_state = CAN_REGISTER_DEVICE;
    }
    printk("voltage: terminating\n");
}

int specific_voltage_init(void);

void noop(void) { }

int register_voltage_device(void) {
    int retval = misc_register(&voltage_miscdev);
    if (retval) {
	printk("voltage: failed to register char device\n");
    }
    else {
	device_state = DEVICE_REGISTERED;
    }
    return retval;
}

int voltage_start(int retval) {
    timer_setup(&measurement.timer, measurement.timeout, 0);
    mod_timer(&measurement.timer, jiffies);
    if (device_state == CAN_REGISTER_DEVICE) {
	retval = register_voltage_device();
    }
    return retval;
}

static int voltage_init(void) {
    int retval;
    printk("voltage: init\n");	

    measurement.specific_exit = noop;
    retval = specific_voltage_init();

    if (retval == 0) {
	retval = voltage_start(retval);
    }
    else {
	voltage_exit();
    }

    return retval;
}

module_init(voltage_init);
module_exit(voltage_exit);













