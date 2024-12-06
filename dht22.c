#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/gpio/consumer.h>
#include <linux/workqueue.h>

#define POLL_INTERVAL 60000  // Sensor polling interval (milliseconds)
#define MSG_PFX "dht22: "
#define MSG_INIT_TIMEOUT MSG_PFX "Sensor timed out while initializing\n"
#define MSG_READ_TIMEOUT MSG_PFX "Sensor timed out while reading data\n"
#define MICRO(micros) (micros*1000)

/* Module info */
MODULE_AUTHOR("Josh Martinez");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("GPIO driver for DHT22 temperature & humdity sensor");

/* Module parameter */
static int gpio_pin = -1;
module_param(gpio_pin, int, 0);
MODULE_PARM_DESC(gpio_pin, "Physical GPIO pin number + GPIO base");

static struct gpio_desc *gdesc_sensor;
static struct delayed_work wk_sensor_read;

static bool sensor_read(void);

static void handler_sensor_read(struct work_struct *work)
{
    sensor_read();
    schedule_delayed_work(&wk_sensor_read, msecs_to_jiffies(POLL_INTERVAL));
}

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

    INIT_DELAYED_WORK(&wk_sensor_read, handler_sensor_read);
    if (!schedule_delayed_work(&wk_sensor_read, 0)) {
        pr_err(MSG_PFX "Failed to schedule work item\n");
        return -EFAULT;
    }

    pr_info(MSG_PFX "Module is loaded\n");
    return 0;
}

static void __exit my_module_exit(void)
{
    cancel_delayed_work_sync(&wk_sensor_read);
    // gpiod_put(gdesc_sensor);
    pr_info(MSG_PFX "Module is unloaded!\n");
}

module_init(my_module_init);
module_exit(my_module_exit);

static bool sensor_read(void)
{
    uint8_t raw_data[5] = {0};

    /* INITIALIZE SENSOR */

    // Set to output, hold low for "at least 1-10ms"    
    gpiod_direction_output(gdesc_sensor, 0);
    mdelay(1);

    // Hold up for "20-40us"
    gpiod_set_value(gdesc_sensor, 1);
    udelay(50);

    // Set to input
    gpiod_direction_input(gdesc_sensor);

    /* WAIT FOR SENSOR TO RESPOND */

    // Start of critical section
    local_irq_disable();

    u64 time_start;

    time_start= ktime_get_ns();
    // Wait for DHT to pull the line low
    while (gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            pr_err(MSG_INIT_TIMEOUT);
            goto timeout;
        }
    }

    time_start = ktime_get_ns();
    // Wait for DHT to pull the line high ("AM2302 will pull low the bus 80us as response signal")
    while (!gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            pr_err(MSG_INIT_TIMEOUT);
            goto timeout;
        }
    }

    time_start = ktime_get_ns();
    // Wait for DHT to pull the line low again ("AM2302 pulls up 80us for preparation to send data")
    while (gpiod_get_value(gdesc_sensor)) {
        if (ktime_get_ns() - time_start > MICRO(100)) {
            pr_err(MSG_INIT_TIMEOUT);
            goto timeout;
        }
    }

    /* READ SENSOR DATA */

    for (int i = 0; i < 40; i++) {
        
        u64 duration_low;  // Used to synchronize cycle cutoff for measuring high

        time_start = ktime_get_ns();
        // Wait for line to go high ("transmission begin with low-voltage-level that last 50us")
        while (!gpiod_get_value(gdesc_sensor)) {
            duration_low = ktime_get_ns() - time_start;
            if (duration_low > MICRO(100)) {
                pr_err(MSG_READ_TIMEOUT);
                goto timeout;
            }
        }

        u64 duration_high;

        // Wait for the line to go low again
        time_start = ktime_get_ns();
        while (gpiod_get_value(gdesc_sensor)) {
            duration_high = ktime_get_ns() - time_start;
            if (duration_high > MICRO(100)) {
                pr_err(MSG_READ_TIMEOUT);
                goto timeout;
            }
        }

        // Use the previous low phase as a reference to determine if high phase is 1 or 0
        if (duration_high > duration_low) {
            raw_data[i / 8] |= (1 << (7 - (i % 8)));
        }
    }

    local_irq_enable();

    /* RUN CHECKSUM */

    if (raw_data[4] == (uint8_t)(raw_data[0] + raw_data[1] +raw_data[2] + raw_data[3])) {
        int rhum = (raw_data[0] << 8) | raw_data[1];
        int temp = (raw_data[2] << 8) | raw_data[3];
        pr_info(MSG_PFX "Sensor data: temp=%d, rhum=%d\n", temp, rhum);
        return true;
    } else {
        pr_err(MSG_PFX "Sensor data failed checksum\n");
        return false;
    }

timeout:
    local_irq_enable();
    return false;
}
