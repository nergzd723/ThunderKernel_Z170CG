/*
 * An I2C driver for SMSC CAP1106.
 *
 * Copyright (c) 2013, ASUSTek Corporation.
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
//#include <linux/earlysuspend.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/workqueue.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/acpi.h>
#include <asm/intel-mid.h>

#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_net.h>
#include <../../arch/x86/platform/asustek/include/asustek_boardinfo.h>


/*
 * Debug Utility
 */
#define CAP_DEBUG(fmt, arg...)	\
		pr_debug("CAP1106: [%s] " fmt , __func__ , ##arg)
#define CAP_INFO(fmt, arg...)	\
		pr_info("CAP1106: [%s] " fmt , __func__ , ##arg)
#define CAP_ERROR(fmt, arg...)	\
		pr_err("CAP1106: [%s] " fmt , __func__ , ##arg)

#define CAP_SDEV_NAME "ril_proximity"

#define CAP1106_I2C_NAME "cap1106"
#define CAP1106_ACPI_NAME "CAP1106"
#define CAP1106_SAR_DET_GPIO 94		/* SAR_DET_3G */
#define CAP1106_SAR_DET_GPIO_NAME "SAR_DET_3G"
extern unsigned int entry_mode;
/*
 * CAP1106 Sensor 1 & 2 I2C Client & Device Name
 */
#define CAP_SENSOR_1_I2C_CLIENT_NAME	"CAP1106:00"
#define CAP_SENSOR_1_I2C_DEVICE_NAME	"2-0028"
#define CAP_SENSOR_2_I2C_CLIENT_NAME	"CAP1106:01"
#define CAP_SENSOR_2_I2C_DEVICE_NAME	"5-0028"

/*
 * CAP1106 Register Name & address
 */
#define MAIN_CONTROL			0x00
#define SENSOR_INPUT_STATUS		0x03
#define SENSOR_INPUT_2_DELTA_COUNT 0x11
#define SENSOR_INPUT_6_DELTA_COUNT	0x15
#define SENSITIVITY_CONTROL		0x1F
#define CONFIGURATION			0x20
#define CONFIGURATION2			0x44
#define SENSOR_INPUT_ENABLE		0x21
#define SENSOR_INPUT_CONFIGURATION	0x22
#define AVERAGING_AND_SAMPLING_CONFING	0x24
#define CALIBRATION_ACTIVATE		0x26
#define INTERRUPT_ENABLE		0x27
#define REPEAT_RATE_ENABLE		0x28
#define MULTIPLE_TOUCH_CONFIG		0x2A
#define SENSOR_INPUT_2_THRESHOLD	0x31
#define SENSOR_INPUT_6_THRESHOLD	0x35
#define SENSOR_INPUT_NOISE_THRESHOLD	0x38
#define SENSOR_INPUT_2_BASE_COUNT	0x51
#define SENSOR_INPUT_6_BASE_COUNT	0x55

/*
 * Global Variable
 */
struct cap1106_data {
	struct attribute_group attrs;
	struct i2c_client *client;
	struct workqueue_struct *cap_wq;
	struct delayed_work work;
	struct delayed_work checking_work;
	int enable;
	int obj_detect;
	int overflow_status;
    int app2mdm_enable;
	int sar_det_gpio;
	char *sar_det_gpio_name;
	int irq_gpio;
	struct miscdevice cap1106_misc_dev;	// for I2C stress test
};

static DEFINE_MUTEX(cap_mtx);
static struct cap1106_data *pivate_data;
static struct switch_dev cap_sdev;
static int is_wood_sensitivity = 0;
static int ac2 = 0; // Accumulated Count Ch2
static int ac6 = 0; // Accumulated Count Ch6
static int ac_limit = 10;
static int force_enable = 1;
static bool bSkip_Checking = false;
static int bWood_Enable = 1;

/*
 * Function Declaration
 */
static int cap1106_probe(struct i2c_client *client,
        const struct i2c_device_id *id);
static int cap1106_remove(struct i2c_client *client);
static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg);
static int cap1106_resume(struct i2c_client *client);
static s32 cap1106_read_reg(struct i2c_client *client, u8 command);
static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value);
static int __init cap1106_init(void);
static void __exit cap1106_exit(void);
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev);
static void cap1106_work_function(struct work_struct *work);
static int cap1106_init_sensor(struct i2c_client *client);
static int cap1106_config_irq(struct i2c_client *client);
static void cap1106_enable_sensor(struct i2c_client *client, int enable);
static ssize_t show_attrs_handler(struct device *dev,
        struct device_attribute *devattr, char *buf);
static ssize_t store_attrs_handler(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

/*
 * I2C Driver Structure
 */
static const struct i2c_device_id cap1106_id[] = {
    { CAP1106_I2C_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cap1106_id);

struct acpi_device_id cap1106_acpi_match[] = {
    { CAP1106_ACPI_NAME, 0 }
};
MODULE_DEVICE_TABLE(acpi, cap1106_id);

#ifdef CONFIG_OF
static struct of_device_id cap1106_match_table[] = {
	{.compatible = "smsc,cap1106,sr" },
	{},
};
static struct of_device_id cap1106_match_table_sr[] = {
	{.compatible = "smsc,cap1106,sr" },
	{},
};
static struct of_device_id cap1106_match_table_er2[] = {
	{.compatible = "smsc,cap1106,er2" },
	{},
};
#else
#define cap1106_match_table NULL
#endif

static struct i2c_driver cap1106_driver = {
	.driver = {
		.name = CAP1106_I2C_NAME,
		.owner = THIS_MODULE,
        .acpi_match_table = ACPI_PTR(cap1106_acpi_match),
#ifdef CONFIG_OF
	.of_match_table = cap1106_match_table,
#endif
	},
	.probe		= cap1106_probe,
	.remove		= cap1106_remove,
	.resume		= cap1106_resume,
	.suspend	= cap1106_suspend,
	.id_table	= cap1106_id,
};

/*
 * I2C stress test
 */
#define CAP1106_IOC_MAGIC 0xF3
#define CAP1106_IOC_MAXNR 2
#define CAP1106_POLL_DATA _IOR(CAP1106_IOC_MAGIC, 2, int)

#define CAP1106_IOCTL_START_HEAVY	2
#define CAP1106_IOCTL_START_NORMAL	1
#define CAP1106_IOCTL_END		0

#define START_NORMAL	(HZ)
#define START_HEAVY	(HZ)

static int stress_test_poll_mode = 0;
struct delayed_work cap1106_stress_test_poll_work;
static struct workqueue_struct *cap1106_stress_test_work_queue;

/**************************************
 * Device Attributes Sysfs Show/Store *
 **************************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
DEVICE_ATTR(obj_detect, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensitivity, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_gain, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_2_delta_count, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensor_input_2_th, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_6_delta_count, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensor_input_6_th, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_noise_th, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_input_status, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensing_cycle, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_onoff, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_recal, 0644, show_attrs_handler, store_attrs_handler);
DEVICE_ATTR(sensor_app2mdm_sar, 0644, NULL, store_attrs_handler);
DEVICE_ATTR(sensor_main, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(cap_status, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(pad_cap_status, 0644, show_attrs_handler, NULL);
DEVICE_ATTR(sensor_wood_enable, 0644, show_attrs_handler, store_attrs_handler);


static struct attribute *cap1106_attr_deb[] = {
	&dev_attr_obj_detect.attr,			// 1
	&dev_attr_sensitivity.attr,			// 2
	&dev_attr_sensor_gain.attr,			// 3
	&dev_attr_sensor_input_2_delta_count.attr,	// 4
	&dev_attr_sensor_input_2_th.attr,		// 5
	&dev_attr_sensor_input_6_delta_count.attr,	// 6
	&dev_attr_sensor_input_6_th.attr,		// 7
	&dev_attr_sensor_input_noise_th.attr,		// 8
	&dev_attr_sensor_input_status.attr,		// 9
	&dev_attr_sensing_cycle.attr,			// 10
	&dev_attr_sensor_onoff.attr,			// 11
	&dev_attr_sensor_recal.attr,			// 12
	&dev_attr_sensor_app2mdm_sar.attr,		// 13
	&dev_attr_sensor_main.attr,			// 14
	&dev_attr_cap_status.attr,			// 15
	&dev_attr_pad_cap_status.attr,			// 16
    &dev_attr_sensor_wood_enable.attr,			// 17
	NULL
};

static ssize_t show_attrs_handler(struct device *dev,
        struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cap1106_data *data = i2c_get_clientdata(client);
	const char *attr_name = devattr->attr.name;
	int ret = -1;
	int i;
	int retry = 3;
	int read_i2c_test_ret = 0;
	int write_i2c_test_ret = 0;

	CAP_DEBUG("devattr->attr->name: %s\n", devattr->attr.name);

	mutex_lock(&cap_mtx);
	if (data->enable) {
		if (!strcmp(attr_name, dev_attr_obj_detect.attr.name)) {
			ret = sprintf(buf, "%d\n", data->obj_detect);
		} else if (!strcmp(attr_name, dev_attr_sensitivity.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSITIVITY_CONTROL));
		} else if (!strcmp(attr_name, dev_attr_sensor_gain.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, MAIN_CONTROL) >> 6);

		} else if (!strcmp(attr_name,
		        dev_attr_sensor_input_2_delta_count.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_2_DELTA_COUNT));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_2_th.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_2_THRESHOLD));

		} else if (!strcmp(attr_name,
		        dev_attr_sensor_input_6_delta_count.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_6_DELTA_COUNT));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_6_th.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_6_THRESHOLD));
		} else if (!strcmp(attr_name,
		        dev_attr_sensor_input_noise_th.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_NOISE_THRESHOLD));
		} else if (!strcmp(attr_name, dev_attr_sensor_input_status.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, SENSOR_INPUT_STATUS));
		} else if (!strcmp(attr_name, dev_attr_sensing_cycle.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, AVERAGING_AND_SAMPLING_CONFING));
		} else if (!strcmp(attr_name, dev_attr_sensor_onoff.attr.name)) {
			ret = sprintf(buf, "%d\n", data->enable);
		} else if (!strcmp(attr_name, dev_attr_sensor_recal.attr.name)) {
			ret = sprintf(buf,
			        cap1106_read_reg(client, CALIBRATION_ACTIVATE) == 0x0 ? "OK\n" : "FAIL\n");
		} else if (!strcmp(attr_name, dev_attr_sensor_main.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, MAIN_CONTROL));
		}
		else if (!strcmp(attr_name, dev_attr_pad_cap_status.attr.name)) {
			//work around before INT is checked
			cap1106_write_reg(client, MAIN_CONTROL,0x80);

			int intstatus,cs2 = 0,cs6 = 0;
			intstatus = cap1106_read_reg(client, SENSOR_INPUT_STATUS);
			if(intstatus == 0x2) // CS2 INT
				cs2 = 1;
			else if(intstatus == 0x20) // CS6 INT
				cs6 = 1;
			else if(intstatus == 0x22){ // CS6 and CS2 INT
				cs2 = 1;
				cs6 = 1;
			}
			ret = sprintf(buf, "[%d,%d]\n",cs2,cs6);
		} else if (!strcmp(attr_name, dev_attr_sensor_wood_enable.attr.name)) {
			ret = sprintf(buf, "%02X\n", bWood_Enable);
		}
		} else {
		if (!strcmp(attr_name, dev_attr_sensor_main.attr.name)) {
			ret = sprintf(buf, "%02X\n", cap1106_read_reg(client, MAIN_CONTROL));
		} else {
			ret = sprintf(buf, "SENSOR DISABLED\n");
		}
	}

	if (!strcmp(attr_name, dev_attr_cap_status.attr.name)) {
		for(i=0;i<retry;i++){
			read_i2c_test_ret = cap1106_read_reg(client, MAIN_CONTROL);
			if( read_i2c_test_ret<0 ){
				printk("cap1106 , check cap read i2c status retry i=%d\n",i);
			}else{
				i=retry;
			}
		}
		for(i=0;i<retry;i++){
			write_i2c_test_ret = cap1106_write_reg(client, SENSITIVITY_CONTROL, 0x2F);
			if( write_i2c_test_ret<0 ){
				printk("cap1106 , check cap write i2c status retry i=%d\n",i);
			}else{
				i=retry;
			}
		}
		if(read_i2c_test_ret>=0 && write_i2c_test_ret>=0){
			ret = sprintf(buf, "%d\n", 1);
		}else{
			ret = sprintf(buf, "%d\n", 0);
		}
	}

	mutex_unlock(&cap_mtx);

	return ret;
}

static ssize_t store_attrs_handler(struct device *dev,
        struct device_attribute *devattr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cap1106_data *data = i2c_get_clientdata(client);
	const char *attr_name = devattr->attr.name;
	unsigned long value;

	if (strict_strtoul(buf, 16, &value)) return -EINVAL;

	CAP_DEBUG("devattr->attr->name: %s, value: 0x%lX\n",
	        devattr->attr.name, value);

	mutex_lock(&cap_mtx);
	if (data->enable) {
		if (!strcmp(attr_name, dev_attr_sensitivity.attr.name)) {
			cap1106_write_reg(client, SENSITIVITY_CONTROL, value & 0x7F);
		} else if (!strcmp(attr_name, dev_attr_sensor_gain.attr.name)) {
			cap1106_write_reg(client, MAIN_CONTROL,
			        (cap1106_read_reg(client, MAIN_CONTROL) & 0x3F) | ((value & 0x03) << 6));

		} else if (!strcmp(attr_name, dev_attr_sensor_input_2_th.attr.name)) {
			cap1106_write_reg(client, SENSOR_INPUT_2_THRESHOLD, value & 0x7F);

		} else if (!strcmp(attr_name, dev_attr_sensor_input_6_th.attr.name)) {
			cap1106_write_reg(client, SENSOR_INPUT_6_THRESHOLD, value & 0x7F);
		} else if (!strcmp(attr_name,
		        dev_attr_sensor_input_noise_th.attr.name)) {
			cap1106_write_reg(client, SENSOR_INPUT_NOISE_THRESHOLD, value & 0x03);
		} else if (!strcmp(attr_name, dev_attr_sensing_cycle.attr.name)) {
			cap1106_write_reg(client, AVERAGING_AND_SAMPLING_CONFING, value & 0x7F);
		} else if (!strcmp(attr_name, dev_attr_sensor_onoff.attr.name)) {
			if (value == 0) {
				force_enable = 0;
				cap1106_enable_sensor(client, 0);
			}
		} else if (!strcmp(attr_name, dev_attr_sensor_recal.attr.name)) {
			cap1106_write_reg(client, CALIBRATION_ACTIVATE, 0x22);
		} else if (!strcmp(attr_name, dev_attr_sensor_app2mdm_sar.attr.name)) {
			gpio_set_value(data->sar_det_gpio, value);
		} else if (!strcmp(attr_name, dev_attr_sensor_wood_enable.attr.name)) {
			if (value == 0)
				bWood_Enable = 0;
			else
				bWood_Enable = 1;
		}
	} else {
		if (!strcmp(attr_name, dev_attr_sensor_onoff.attr.name)) {
			if (value == 1) {
				force_enable = 1;
				cap1106_enable_sensor(client, 1);
			}
		}
	}
	mutex_unlock(&cap_mtx);

	return strnlen(buf, count);;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*******************************
 * Callbacks for switch device *
 *******************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t print_cap_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "prox_sar_det");
}

static ssize_t print_cap_state(struct switch_dev *sdev, char *buf)
{
	if (switch_get_state(sdev))
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*****************************
 * I2C stress test functions *
 *****************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void cap1106_get_input_values(struct i2c_client *client) {

	int status;
	int bc6, dc6; // Base Count Ch6, Delta Count Ch6
	status = cap1106_read_reg(client, SENSOR_INPUT_STATUS);
	dc6 = cap1106_read_reg(client, SENSOR_INPUT_6_DELTA_COUNT);
	bc6 = cap1106_read_reg(client, SENSOR_INPUT_6_BASE_COUNT);

	printk(KERN_DEBUG "[%s] status: 0x%02X, bc6=0x%02X, dc6=0x%02X\n", __func__, status, bc6, dc6);
}

static void cap1106_stress_test_poll(struct work_struct * work)
{
	cap1106_get_input_values(pivate_data->client);
	if(stress_test_poll_mode == 0)
		msleep(5);

	queue_delayed_work(cap1106_stress_test_work_queue, &cap1106_stress_test_poll_work,
			stress_test_poll_mode);
}

long cap1106_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	printk(KERN_DEBUG "------> CAP1106 IOCTL for stress test <------\n");
	if (_IOC_TYPE(cmd) != CAP1106_IOC_MAGIC)
		return -ENOTTY;

	if (_IOC_NR(cmd) > CAP1106_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case CAP1106_POLL_DATA:
			if (arg == CAP1106_IOCTL_START_HEAVY){
				printk(KERN_DEBUG "Cap sensor cap1106: ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(cap1106_stress_test_work_queue,
						&cap1106_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == CAP1106_IOCTL_START_NORMAL){
				printk(KERN_DEBUG "Cap sensor cap1106: ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(cap1106_stress_test_work_queue,
						&cap1106_stress_test_poll_work, stress_test_poll_mode);
			} else if  (arg == CAP1106_IOCTL_END){
				printk(KERN_DEBUG "Cap sensor cap1106: ioctl end\n");
				cancel_delayed_work_sync(&cap1106_stress_test_poll_work);
			} else {
				return -ENOTTY;
			}
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}

int cap1106_open(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "Cap sensor cap1106: %s\n", __func__);
	return 0;
}

int cap1106_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "Cap sensor cap1106: %s\n", __func__);
	return 0;
}

struct file_operations cap1106_fops = {
	.owner = THIS_MODULE,
	.open = cap1106_open,
	.release = cap1106_release,
	.unlocked_ioctl = cap1106_ioctl,
};
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*****************************
 * I2C stress test functions *
 *****************************/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void cap1106_enable_sensor(struct i2c_client *client, int enable)
{
	long reg_value;

	struct cap1106_data *data = i2c_get_clientdata(client);

	if (data->enable != enable) {
		reg_value = cap1106_read_reg(client, MAIN_CONTROL);
		if (enable) {
			cap1106_write_reg(client, MAIN_CONTROL, (reg_value & 0xEF) | (!enable << 4));
			// Time to first conversion is 200ms (Max)
			/*- workaround capsensor will switch VDD of on sofia -*/
			int rc;
			rc = cap1106_init_sensor(client);
			if (rc) {
				CAP_ERROR("Sensor initialization failed!\n");
			}

			bSkip_Checking = false;  // if checking_work_function is in progress, checking_work_function must skip.
			queue_delayed_work(data->cap_wq, &data->work,
			        msecs_to_jiffies(200));
			enable_irq(data->irq_gpio);
			queue_delayed_work(data->cap_wq, &data->checking_work,
			        msecs_to_jiffies(1000));
		} else {
			disable_irq(data->irq_gpio);
			bSkip_Checking = true;  // if checking_work_function is in progress, checking_work_function must skip.
			//cancel_delayed_work_sync(&data->work);
			//cancel_delayed_work_sync(&data->checking_work);
			//flush_workqueue(data->cap_wq);
			switch_set_state(&cap_sdev, 0);
			cap1106_write_reg(client, MAIN_CONTROL, (reg_value & 0xEF) | (!enable << 4));
		}
		data->enable = enable;

		CAP_INFO("data->enable: %d\n", data->enable);
	}
}

static s32 cap1106_read_reg(struct i2c_client *client, u8 command)
{
	return i2c_smbus_read_byte_data(client, command);
}

static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value)
{
	return i2c_smbus_write_byte_data(client, command, value);
}

static void cap1106_work_function(struct work_struct *work)
{
	int status;
//	int bc6, dc6; // Base Count Ch6, Delta Count Ch6
	int bc2, bc6; // Base Count Ch2, Ch6
	int dc2, dc6; // Delta Count Ch2, Ch6
	struct cap1106_data *data =
	        container_of((struct delayed_work *)work, struct cap1106_data, work);

	disable_irq(data->irq_gpio);
	// Clear INT, keep GAIN, STBY, DSLEEP
	cap1106_write_reg(data->client, MAIN_CONTROL,
	        cap1106_read_reg(data->client, MAIN_CONTROL) & 0xF0);
	status = cap1106_read_reg(data->client, SENSOR_INPUT_STATUS);
	dc2 = cap1106_read_reg(data->client, SENSOR_INPUT_2_DELTA_COUNT);
	dc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_DELTA_COUNT);
	bc2 = cap1106_read_reg(data->client, SENSOR_INPUT_2_BASE_COUNT);
	bc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_BASE_COUNT);

	CAP_DEBUG(
	        "status: 0x%02X, bc2=0x%02X, dc2=0x%02X, bc6=0x%02X, dc6=0x%02X\n",
	        status, bc2, dc2, bc6, dc6);

//	CAP_DEBUG("status: 0x%02X, bc6=0x%02X, dc6=0x%02X\n", status, bc6, dc6);
	if (bWood_Enable) {	
		if (is_wood_sensitivity == 0) {
			data->obj_detect = (status & 0x22) ? 1 : 0;
			CAP_DEBUG("obj_detect: %d\n", data->obj_detect);
			switch_set_state(&cap_sdev, data->obj_detect);
			if (data->app2mdm_enable) {
				gpio_set_value(data->sar_det_gpio, data->obj_detect);
			}
			if ((status == 0x02 && dc2 == 0x7F) || (status == 0x20 && dc6 == 0x7F)
			        || (status == 0x22 && (dc2 == 0x7F || dc6 == 0x7F))) {
				CAP_DEBUG("is_wood_sensitivity = 1\n");
				//set sensitivity and threshold for wood touch
				cap1106_write_reg(data->client, SENSITIVITY_CONTROL, 0x4F);
				cap1106_write_reg(data->client, SENSOR_INPUT_2_THRESHOLD, 0x5F);
				cap1106_write_reg(data->client, SENSOR_INPUT_6_THRESHOLD, 0x5F);
				is_wood_sensitivity = 1;
				data->overflow_status = status;
				ac2 = 0;
				ac6 = 0;
			} else {
				if (dc2 >= 0x0A && dc2 <= 0x3F) ac2++;
				if (dc6 >= 0x0A && dc6 <= 0x3F) ac6++;

				CAP_DEBUG("ac2=%d, ac6=%d\n", ac2, ac6);
//				CAP_DEBUG("ac6=%d\n", ac6);
				if (ac2 >= ac_limit || ac6 >= ac_limit) {
					CAP_DEBUG("+++ FORCE RECALIBRATION +++\n");
					cap1106_write_reg(data->client, CALIBRATION_ACTIVATE, 0x22);
					ac2 = 0;
					ac6 = 0;
				}
			}
		}
	}
	enable_irq(data->irq_gpio);
}

static irqreturn_t cap1106_interrupt_handler(int irq, void *dev)
{
//	CAP_INFO("cap1106_interrupt_handler handle interrupt!!\n");
	struct cap1106_data *data = i2c_get_clientdata(dev);
	queue_delayed_work(data->cap_wq, &data->work, 0);
	return IRQ_HANDLED;
}

#ifdef CONFIG_OF
static int cap1106_parse_dt(struct device *dev,
			struct cap1106_data *pdata)
{
	struct device_node *np = dev->of_node;


	pdata->sar_det_gpio = of_get_named_gpio_flags(dev->of_node,"smsc,cap1106-gpio-irq", 0, NULL);

	if (pdata->sar_det_gpio <= 0)
		dev_err(dev, "Invalid gpio num %d \n",pdata->sar_det_gpio);

	pdata->irq_gpio = irq_of_parse_and_map(np,0);

	printk(KERN_INFO "cap1106_parse_dt Result  gpio num = %d irq num = %d\n",pdata->sar_det_gpio,pdata->irq_gpio);

	return 0;
}
#else
static int cap1106_parse_dt(struct device *dev,
			struct cap1106_data *pdata)
{
	return -EINVAL;
}
#endif


static int cap1106_config_irq(struct i2c_client *client)
{
	int rc = 0;

	struct cap1106_data *data = i2c_get_clientdata(client);

	rc = cap1106_parse_dt(&client->dev, data);
	if (rc) {
		CAP_ERROR("Failed to parse device tree\n");
		rc = -EINVAL;
	}

	if (gpio_is_valid(data->sar_det_gpio)) {
		rc = gpio_request(data->sar_det_gpio, data->sar_det_gpio_name);
		if (rc) {
			CAP_ERROR("SAR_DET_3G_gpio_request_failed, rc=%d\n", rc);
			goto err_SAR_DET_3G_gpio_request_failed;
		}

		rc = gpio_direction_input(data->sar_det_gpio);
		if (rc) {
			CAP_ERROR("SAR_DET_3G_gpio_direction_input_failed, rc=%d\n", rc);
			goto err_SAR_DET_3G_gpio_direction_input_failed;
		}

		rc = request_irq(data->irq_gpio, cap1106_interrupt_handler,
		        IRQF_TRIGGER_FALLING, data->sar_det_gpio_name, client);
		if (rc) {
			CAP_ERROR("SAR_DET_3G_request_irq_failed, rc=%d\n", rc);
			goto err_SAR_DET_3G_request_irq_failed;
		}

		CAP_INFO("GPIO=%02d, VALUE=%d, IRQ=%d OK\n",
		        data->sar_det_gpio, gpio_get_value(data->sar_det_gpio), data->irq_gpio);
	}

	return 0;

err_SAR_DET_3G_request_irq_failed:
err_SAR_DET_3G_gpio_direction_input_failed:
	gpio_free(data->sar_det_gpio);
err_SAR_DET_3G_gpio_request_failed:
// err_APP2MDM_SAR_gpio_direction_output_failed:
//	gpio_free(data->sar_gpio);
// err_APP2MDM_SAR_gpio_request_failed:
	return rc;
}

static int cap1106_init_sensor(struct i2c_client *client)
{
	u8 bIdx;
	int rc = 0;

	const u8 cap1106_init_table[] = {
		SENSITIVITY_CONTROL, 0x2F, // Data sensitivity
		CONFIGURATION, 0x20, // MAX duration disable
		SENSOR_INPUT_ENABLE, 0x22, // Enable CS6 and CS2.
		SENSOR_INPUT_CONFIGURATION, 0xFF, // MAX duration time to max , repeat period time to max
		AVERAGING_AND_SAMPLING_CONFING, 0x39, // Digital count update time to 140*64ms
		INTERRUPT_ENABLE, 0x22, // Enable INT. for CS6 and CS2.
		REPEAT_RATE_ENABLE, 0x22, // Disable repeat irq
		MULTIPLE_TOUCH_CONFIG, 0x00, // All channel run in the same time
		SENSOR_INPUT_2_THRESHOLD, 0x0A, // Threshold of CS 2
		SENSOR_INPUT_6_THRESHOLD, 0x0A, // Threshold of CS 6
		CALIBRATION_ACTIVATE, 0x22, // Force re-cal CS6 and CS2
		CONFIGURATION2, 0x44, // Disable RF Noise filter
		MAIN_CONTROL, 0x80, // Reset INT. bit.
	};

	// struct cap1106_data *data = i2c_get_clientdata(client);

	for (bIdx = 0; bIdx < sizeof(cap1106_init_table) / sizeof(cap1106_init_table[0]); bIdx += 2) {
		if ((rc = cap1106_write_reg(client, cap1106_init_table[bIdx],
			cap1106_init_table[bIdx + 1]))) {
			CAP_ERROR("I2C write error, rc=0x%X\n", rc);
			break;
		}
	}

	CAP_INFO("I2C_NAME: %s, I2C_ADDR: 0x%X, %s\n", client->name, client->addr,
			rc ? "FAIL" : "OK");

	return rc;
}

static void cap1106_checking_work_function(struct work_struct *work)
{
	int status;
//	int bc6, dc6;
	int bc2, bc6;
	int dc2, dc6;

	struct cap1106_data *data =
	        container_of((struct delayed_work *)work, struct cap1106_data, checking_work);

	mutex_lock(&cap_mtx);

	CAP_DEBUG("+\n");
	if( bSkip_Checking )
	{
		// skip this round and contiune next round
		CAP_DEBUG("bSkip_Checking!!\n");
		mutex_unlock(&cap_mtx);
		return;
	}
	if (is_wood_sensitivity == 1) {
		if (data->enable) {
			status = cap1106_read_reg(data->client, SENSOR_INPUT_STATUS);
			dc2 = cap1106_read_reg(data->client, SENSOR_INPUT_2_DELTA_COUNT);
			dc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_DELTA_COUNT);
			bc2 = cap1106_read_reg(data->client, SENSOR_INPUT_2_BASE_COUNT);
			bc6 = cap1106_read_reg(data->client, SENSOR_INPUT_6_BASE_COUNT);

			CAP_DEBUG(
			        "status: 0x%02X, bc2=0x%02X, dc2=0x%02X, bc6=0x%02X, dc6=0x%02X\n",
			        status, bc2, dc2, bc6, dc6);
			if ((dc2 == 0x00 && dc6 == 0x00)
					|| (dc2 == 0xFF && dc6 == 0xFF)
			        || (dc2 == 0x00 && dc6 == 0xFF)
			        || (dc2 == 0xFF && dc6 == 0x00)
			        || (data->overflow_status == 0x02 && (dc2 > 0x5F) && (dc2 <= 0x7F))
			        || (data->overflow_status == 0x20 && (dc6 > 0x5F) && (dc6 <= 0x7F))
			        || (data->overflow_status == 0x22 && (((dc2 > 0x5F) && (dc2 <= 0x7F)) || ((dc6 > 0x5F) && (dc6 <= 0x7F))))) {
				CAP_DEBUG("is_wood_sensitivity = 0\n");
				//set sensitivity and threshold for 2cm body distance
				cap1106_write_reg(data->client, SENSITIVITY_CONTROL, 0x6F);
				cap1106_write_reg(data->client, SENSOR_INPUT_2_THRESHOLD, 0x0E);
				cap1106_write_reg(data->client, SENSOR_INPUT_6_THRESHOLD, 0x0A);
				is_wood_sensitivity = 0;
				queue_delayed_work(data->cap_wq, &data->work, 0);
			}
		} else {
                    printk(KERN_DEBUG "cap1106: data->enable: %d\n", data->enable);
                }
	}
	queue_delayed_work(data->cap_wq, &data->checking_work, msecs_to_jiffies(1000));

	CAP_DEBUG("-\n");
	mutex_unlock(&cap_mtx);

}

static int cap1106_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
{

	int rc = 0;

    printk(KERN_DEBUG "cap1106 probe ++\n");

    CAP_INFO("I2C_ADDR: 0x%X, I2C_CLIENT_NAME: %s, I2C_DEVICE_NAME: %s\n",
			client->addr, client->name, dev_name(&client->dev));



//	/* only enable one cap sensor, temporary solution */
//	if (!strcmp(client->name, CAP_SENSOR_2_I2C_CLIENT_NAME)) {
//		CAP_DEBUG("skip sensor 2\n");
//		rc = -ENODEV;
//		goto err_kzalloc_failed;
//	}

	struct cap1106_data *data;
	// struct cap1106_platform_data *pdata;

	data = kzalloc(sizeof(struct cap1106_data), GFP_KERNEL);
	if (!data) {
		CAP_ERROR("kzalloc failed!\n");
		rc = -ENOMEM;
		goto err_kzalloc_failed;
	}

	data->cap_wq = create_singlethread_workqueue("cap_wq");
	if (!data->cap_wq) {
		CAP_ERROR("create_singlethread_workqueue failed!\n");
		rc = -ENOMEM;
		goto err_create_singlethread_workqueue_failed;
	}

	INIT_DELAYED_WORK(&data->work, cap1106_work_function);
	INIT_DELAYED_WORK(&data->checking_work, cap1106_checking_work_function);

        data->client = client;
        i2c_set_clientdata(client, data);

        data->app2mdm_enable = 0;
//        data->sar_det_gpio = CAP1106_SAR_DET_GPIO;
        data->sar_det_gpio_name = CAP1106_SAR_DET_GPIO_NAME;
//        data->irq_gpio = gpio_to_irq(data->sar_det_gpio);
//
//        CAP_INFO("data->app2mdm_enable: %d\n", data->app2mdm_enable);
//        CAP_INFO("data->det_gpio: %d\n", data->sar_det_gpio);
//        CAP_INFO("data->det_gpio_name: %s\n", data->sar_det_gpio_name);
//        CAP_INFO("data->irq_gpio: %d\n", data->irq_gpio);

	data->client->flags = 0;
	// strlcpy(data->client->name, CAP1106_I2C_NAME, I2C_NAME_SIZE);
	data->enable = 0;

	rc = cap1106_init_sensor(data->client);
	if (rc) {
		CAP_ERROR("Sensor initialization failed!\n");
		goto err_init_sensor_failed;
	}

	data->attrs.attrs = cap1106_attr_deb;

	rc = sysfs_create_group(&data->client->dev.kobj, &data->attrs);
	if (rc) {
		CAP_ERROR("Create the sysfs group failed!\n");
		goto err_create_sysfs_group_failed;
	}

	/* register switch class */
	cap_sdev.name = CAP_SDEV_NAME;
	cap_sdev.print_name = print_cap_name;
	cap_sdev.print_state = print_cap_state;

	rc = switch_dev_register(&cap_sdev);
	if (rc) {
		CAP_ERROR("Switch device registration failed!\n");
		goto err_register_switch_class_failed;
	}

	rc = cap1106_config_irq(data->client);
	if (rc) {
                printk(KERN_DEBUG "cap1106 sensor INT config failed...\n");
		CAP_ERROR("Sensor INT configuration failed!\n");
		goto err_config_irq_failed;
	}

	data->enable = 1;
	data->overflow_status = 0x0;
	queue_delayed_work(data->cap_wq, &data->work, msecs_to_jiffies(200));
	queue_delayed_work(data->cap_wq, &data->checking_work, msecs_to_jiffies(1000));

	/* init I2C stress work queue */
	cap1106_stress_test_work_queue = create_singlethread_workqueue("i2c_cap1106_wq");
	if(!cap1106_stress_test_work_queue){
		printk(KERN_DEBUG "Cap sensor cap1106: unable to create i2c stress test workqueue\n");
		goto err_create_stress_test_workqueue_failed;
	}
	INIT_DELAYED_WORK(&cap1106_stress_test_poll_work, cap1106_stress_test_poll);

	/* init misc device for I2C stress test */
	data->cap1106_misc_dev.minor = MISC_DYNAMIC_MINOR;
	data->cap1106_misc_dev.name = "cap1106";
	data->cap1106_misc_dev.fops  = &cap1106_fops;
	rc = misc_register(&data->cap1106_misc_dev);
	if (rc) {
		printk(KERN_DEBUG "Cap sensor cap1106: register misc dev fail\n");
		goto err_register_misc_dev_failed;
	}

	pivate_data = data;

	CAP_INFO("OK\n");
        printk(KERN_DEBUG "cap1106 probe --\n");

	return 0;

err_register_misc_dev_failed:
	destroy_workqueue(cap1106_stress_test_work_queue);
err_create_stress_test_workqueue_failed:
err_config_irq_failed:
err_register_switch_class_failed:
	sysfs_remove_group(&data->client->dev.kobj, &data->attrs);
err_create_sysfs_group_failed:
err_init_sensor_failed:
	destroy_workqueue(data->cap_wq);
err_create_singlethread_workqueue_failed:
	kfree(data);
err_kzalloc_failed:
	return rc;
}

static int cap1106_remove(struct i2c_client *client)
{
	struct cap1106_data *data = i2c_get_clientdata(client);
	CAP_DEBUG("+\n");
	switch_dev_unregister(&cap_sdev);
	misc_deregister(&data->cap1106_misc_dev);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);
	free_irq(data->irq_gpio, client);
	if (data->cap_wq) destroy_workqueue(data->cap_wq);
	kfree(data);
	CAP_DEBUG("-\n");
	return 0;
}

static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg)
{
	CAP_DEBUG("+\n");
	mutex_lock(&cap_mtx);
	cap1106_enable_sensor(client, 0);
	mutex_unlock(&cap_mtx);
	CAP_DEBUG("-\n");
	return 0;
}

static int cap1106_resume(struct i2c_client *client)
{
	CAP_DEBUG("+\n");
	mutex_lock(&cap_mtx);
	if (force_enable) cap1106_enable_sensor(client, 1);
	mutex_unlock(&cap_mtx);
	CAP_DEBUG("-\n");
	return 0;
}

static int __init cap1106_init(void)
{
	int rc = 0;
	CAP_INFO("system entry_mode = %d \n",entry_mode);
	if(entry_mode != 1 ){
		CAP_INFO(" Not in MOS, no need cap driver. \n");
		return rc;
	}

	printk(KERN_DEBUG "cap1106 init\n");
	CAP_INFO("\n");
	

/*
 * 	enum{
 *	G850T,
 *	G900T,//3G: WCDMA: 900/2100 2G: EDGE/GSM:
 *      G850_WC,//"3G: WCDMA: 850/900/1900/2100	with CAP
 *	DEFAULT,	
 *	}
 */
    CAP_INFO("%s : rf_id is %d \n", __func__, asustek_boardinfo_get(FUN_RF_SKU_ID));
    if(asustek_boardinfo_get(FUN_RF_SKU_ID) != 2)
		return 0;
    
    int hardware_id = 0;
    hardware_id=asustek_boardinfo_get(FUN_HARDWARE_ID);
    CAP_INFO("%s : hardware_id is %d \n", __func__, hardware_id);

    //SR:0 ER:1 ER2:2 PR:3
    if (hardware_id == 0x00 || hardware_id == 0x01) {
    	CAP_INFO("%s : hardware_id is SR or ER1 !\n", __func__);
    	cap1106_driver.driver.of_match_table = cap1106_match_table_sr;
    }
    /*else if  (hardware_id == 0x03) {
        PINFO("%s : hardware_id is ER2 !\n", __func__);
        bma2x2_driver.driver.of_match_table = cap1106_match_table[2];
    }
    */
    else {
    	CAP_INFO(" %s : hardware_id is ER2 or others !\n", __func__);
    	cap1106_driver.driver.of_match_table = cap1106_match_table_er2;
    }

	return i2c_add_driver(&cap1106_driver);
}

static void __exit cap1106_exit(void)
{
	CAP_INFO("\n");
	i2c_del_driver(&cap1106_driver);
}

module_init(cap1106_init);
module_exit(cap1106_exit);
MODULE_DESCRIPTION("SMSC CAP1106 Driver");
MODULE_LICENSE("GPL");
