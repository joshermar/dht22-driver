#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>
#include <linux/miscdevice.h>

#define POLL_INTERVAL 60000  // Sensor polling interval (milliseconds)
#define MSG_PFX "dht22: "
#define MICRO(micros) (micros*1000)

MODULE_AUTHOR("Josh Martinez");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPIO driver for DHT22 temperature & humdity sensor");

static int gpio_pin = -1;
module_param(gpio_pin, int, 0);
MODULE_PARM_DESC(gpio_pin, "Physical GPIO pin number + GPIO base");

static struct gpio_desc *gdesc_sensor;
static struct delayed_work wk_sensor_read;

static char dht22_rhum[8];
static char dht22_temp[8];

static bool sensor_poll(void);

static void handler_sensor_read(struct work_struct *work)
{
    sensor_poll();
    schedule_delayed_work(&wk_sensor_read, msecs_to_jiffies(POLL_INTERVAL));
}

static ssize_t dht22_rhum_read(struct file *filp, char __user *buf, size_t len, loff_t *f_pos)
{
    size_t remaining = strlen(dht22_rhum) - *f_pos;

    if (!remaining) {
        return 0;
    }

    size_t copy_len = min(len, remaining);

    size_t copied = copy_len - copy_to_user(buf, dht22_rhum + *f_pos, copy_len);

    *f_pos += copied;

    return copied;
}

static const struct file_operations dht22_rhum_fops = {
    .owner = THIS_MODULE,
    .read = dht22_rhum_read
};

static struct miscdevice dht22_rhum_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dht22_rhum",
    .fops = &dht22_rhum_fops,
    .mode = 0444,
};

static ssize_t dht22_temp_read(struct file *filp, char __user *buf, size_t len, loff_t *f_pos)
{
    size_t remaining = strlen(dht22_temp) - *f_pos;

    if (!remaining) {
        return 0;
    }

    size_t copy_len = min(len, remaining);

    size_t copied = copy_len - copy_to_user(buf, dht22_temp + *f_pos, copy_len);

    *f_pos += copied;

    return copied;
}

static const struct file_operations dht22_temp_fops = {
    .owner = THIS_MODULE,
    .read = dht22_temp_read
};

static struct miscdevice dht22_temp_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dht22_temp",
    .fops = &dht22_temp_fops,
    .mode = 0444,
};

static int __init my_module_init(void)
{

    if (gpio_pin < 0) {
        return -EINVAL;
    }

    gdesc_sensor = gpio_to_desc(gpio_pin);
    if (!gdesc_sensor) {
        pr_err(MSG_PFX "Failed to get descriptor for GPIO %d\n", gpio_pin);
        return -ENXIO;
    }

    // TODO: Poll sensor for initial reading before proceeding

    INIT_DELAYED_WORK(&wk_sensor_read, handler_sensor_read);
    if (!schedule_delayed_work(&wk_sensor_read, 0)) {
        pr_err(MSG_PFX "Failed to schedule work item\n");
        return -EFAULT;
    }

    int ret;

    ret = misc_register(&dht22_rhum_miscdev);

    if (ret) {
        pr_err(MSG_PFX "Failed to register misc device\n");
    } else {
        pr_info(MSG_PFX "Registered misc device: /dev/%s\n", dht22_rhum_miscdev.name);
    }

    ret = misc_register(&dht22_temp_miscdev);

    if (ret) {
        pr_err(MSG_PFX "Failed to register misc device\n");
    } else {
        pr_info(MSG_PFX "Registered misc device: /dev/%s\n", dht22_temp_miscdev.name);
    }

    pr_info(MSG_PFX "Module is loaded\n");

    return ret;
}

static void __exit my_module_exit(void)
{
    cancel_delayed_work_sync(&wk_sensor_read);
    gpiod_put(gdesc_sensor);

    misc_deregister(&dht22_rhum_miscdev);
    misc_deregister(&dht22_temp_miscdev);

    pr_info(MSG_PFX "Module is unloaded\n");
}

module_init(my_module_init);
module_exit(my_module_exit);

static void dht22_initialize(void)
{
    // Set to output, hold low for "at least 1-10ms"    
    gpiod_direction_output(gdesc_sensor, 0);
    mdelay(1);

    // Hold up before switching to input
    gpiod_set_value(gdesc_sensor, 1);
    udelay(50);

    gpiod_direction_input(gdesc_sensor);
}


static bool dht22_read_raw(u8 *raw_data)
{
    u64 time_start;

    // Wait for DHT to pull the line low
    time_start= ktime_get_ns();
    while (gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            return false;
        }
    }

    // Wait for DHT to pull the line high ("AM2302 will pull low the bus 80us as response signal")
    time_start = ktime_get_ns();
    while (!gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            return false;
        }
    }

    // Wait for DHT to pull the line low again ("AM2302 pulls up 80us for preparation to send data")
    time_start = ktime_get_ns();
    while (gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            return false;
        }
    }

    // Read sensor data
    for (int i = 0; i < 40; i++) {
        
        u64 duration_low;  // Used to synchronize cycle cutoff for measuring high

        time_start = ktime_get_ns();
        // Wait for line to go high ("transmission begin with low-voltage-level that last 50us")
        while (!gpiod_get_value(gdesc_sensor)) {
            duration_low = ktime_get_ns() - time_start;
            if (duration_low > MICRO(100)) {
                return false;
            }
        }

        u64 duration_high;

        // Wait for the line to go low again
        time_start = ktime_get_ns();
        while (gpiod_get_value(gdesc_sensor)) {
            duration_high = ktime_get_ns() - time_start;
            if (duration_high > MICRO(100)) {
                return false;
            }
        }

        // Use the previous low phase as a reference to determine if high phase is 1 or 0
        if (duration_high > duration_low) {
            raw_data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }
    return true;
}

static void record_data(u8 *raw_data)
{
    // First 16 bits represent relative humidty
    u16 rhum = (raw_data[0] << 8) | raw_data[1];

    snprintf(dht22_rhum, sizeof(dht22_rhum), "%u\n", rhum);

    // Next 16 bits represent temperature (sign–magnitude representation)
    s16 temp = ((raw_data[2] & 0x7F) << 8) | raw_data[3];

    if (raw_data[2] & 0x80) {
        temp *= -1;  // Brrrr
    }

    snprintf(dht22_temp, sizeof(dht22_temp), "%d\n", temp);
}

static bool sensor_poll(void)
{
    u8 raw_data[5] = {0};

    dht22_initialize();

    // Critical section
    local_irq_disable();

    bool read_success = dht22_read_raw(raw_data);

    // End of critical section
    local_irq_enable();

    if (!read_success) {
        pr_err(MSG_PFX "Timed out while reading sensor data\n");
        return false;
    }

    // Check data for consistency
    if (raw_data[4] != ((raw_data[0] + raw_data[1] + raw_data[2] + raw_data[3]) & 0xFF)) {
        pr_err(MSG_PFX "Sensor data checksum failure\n");
        return false;
    }

    record_data(raw_data);

    return true;
}
