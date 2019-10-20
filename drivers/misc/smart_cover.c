#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <../../arch/x86/platform/asustek/include/asustek_boardinfo.h>
#include "smart_cover.h"

#define COVER_ON 1
#define COVER_OFF 0

#define BATTERY_ON 0
#define BATTERY_OFF 1

#define DETECT_ON_DEBOUNCE_TIME 3000 //ms
#define DETECT_OFF_DEBOUNCE_TIME 1000 //ms

#define POWER_BANK_VALUE 49
#define AUDIO_SLEEVE_VALUE 50

#define ISN_ADDR  0
#define SSN_ADDR 32
#define MN_ADDR  52
#define RS_ADDR  57
#define CT_ADDR  61
#define ACT_ADDR 63

#define ISN_SIZE 32 // 00-1F
#define SSN_SIZE 20 // 20-33
#define MN_SIZE   5 // 34-38
#define RS_SIZE   4 // 39-3C
#define CT_SIZE   1 // 3D
#define ACT_SIZE  1 // 3F

enum smart_cover_state {
	NO_COVER,
	HAS_COVER,
	HAS_COVER_NO_POWER
};

enum smart_cover_callback_list_class {
	DETECT,
	DETECT_PIN,
	BATTERY_STATUS
};

struct smart_cover_data {
	int state;
	char *type;
	int detect_irq;
	int detect_gpio;
	int i2c_power_gpio;
	int battery_status_gpio;
	struct switch_dev sdev;
	int attach_debounce_time;
	int unattach_debounce_time;
	struct wake_lock wake_lock;
};

struct activate_data {
	bool is_cached;
	u8 value;
};

struct callback_data {
	int class;
	char *cover_type;
	char *tag;
	void (*function)(bool);
	struct callback_data *next;
};

static bool is_probe_successfully = false;
static bool is_checked_detection_during_boot = false;
static int state_change_mutex = 0;
static struct callback_data *callback_head = NULL;
static struct callback_data *callback_prev;
static struct callback_data *callback_cur;
static struct smart_cover_data *cover;
static struct delayed_work cover_work;
static struct class *cover_class = NULL;
static struct device *cover_dev = NULL;
static struct activate_data act_data;

extern int ht24lc02_write(u8 reg, u8 value);
extern int ht24lc02_read(u8 reg);
void cover_detect_register(char *, char *, void(*)(bool));
void cover_detect_unregister(char *, char *);
void cover_detect(void);

static int write_eeprom(u8 addr, u8 value);
static int read_eeprom(u8 addr);
static void callback_register(int , char *, char *, void(*function)(bool));
static void callback_unregister(int , char *, char *);
static void no_cover_state(void);
static void has_cover_state(void);
static void has_cover_no_power_state(void);
static void state_check_for_battery(void);

EXPORT_SYMBOL(cover_detect_register);
EXPORT_SYMBOL(cover_detect_unregister);
EXPORT_SYMBOL(cover_detect);

//sysfs implementation++
static int read_eeprom_for_attr(u8 , u8 , char *);
static int write_eeprom_for_attr(u8 , u8, const char *, size_t);
static int ISN_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return read_eeprom_for_attr(ISN_ADDR, ISN_SIZE, buf);}
static int ISN_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){return write_eeprom_for_attr(ISN_ADDR, ISN_SIZE, buf, count);}
static int SSN_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return read_eeprom_for_attr(SSN_ADDR, SSN_SIZE, buf);}
static int SSN_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){return write_eeprom_for_attr(SSN_ADDR, SSN_SIZE, buf, count);}
static int MN_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return read_eeprom_for_attr(MN_ADDR, MN_SIZE, buf);}
static int MN_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){return write_eeprom_for_attr(MN_ADDR, MN_SIZE, buf, count);}
static int RS_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return read_eeprom_for_attr(RS_ADDR, RS_SIZE, buf);}
static int RS_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){return write_eeprom_for_attr(RS_ADDR, RS_SIZE, buf, count);}
static int CT_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return read_eeprom_for_attr(CT_ADDR, CT_SIZE, buf);}
static int CT_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){return write_eeprom_for_attr(CT_ADDR, CT_SIZE, buf, count);}
static int ACT_read_attr(struct device *, struct device_attribute *, char *);
static int ACT_write_attr(struct device *, struct device_attribute *, const char *, size_t);
static int attached_read_attr(struct device *dev, struct device_attribute *attr, char *buf){return snprintf(buf, 2, "%d", gpio_get_value(cover->detect_gpio));}
static int EINT10_read_attr(struct device *, struct device_attribute *, char *);

static DEVICE_ATTR(ISN, 0444, ISN_read_attr, ISN_write_attr);
static DEVICE_ATTR(SSN, 0444, SSN_read_attr, SSN_write_attr);
static DEVICE_ATTR(modelName, 0444, MN_read_attr, MN_write_attr);
static DEVICE_ATTR(reserved, 0444, RS_read_attr, RS_write_attr);
static DEVICE_ATTR(coverType, 0444, CT_read_attr, CT_write_attr);
static DEVICE_ATTR(activated, 0666, ACT_read_attr, ACT_write_attr);
static DEVICE_ATTR(attached, 0444, attached_read_attr, NULL);
static DEVICE_ATTR(EINT10, 0444, EINT10_read_attr, NULL);

static const struct device_attribute *cover_attrs[] = {
	&dev_attr_ISN,
	&dev_attr_SSN,
	&dev_attr_modelName,
	&dev_attr_reserved,
	&dev_attr_coverType,
	&dev_attr_activated,
	&dev_attr_attached,
	&dev_attr_EINT10,
};
//sysfs implementation--
void cover_detect(void)
{
	pr_info("smart_cover: cover detect called by eeprom\n");
	
	if(is_probe_successfully)
	{
		state_change_mutex = 1;
		no_cover_state();
		state_change_mutex = 0;
		is_checked_detection_during_boot = true;
	}
	else
		pr_err("smart_cover: probe not successfully, so do nothing\n");
}

void cover_detect_register(char *cover_type, char *tag, void(*function)(bool))
{
	callback_register(DETECT, cover_type, tag, function);
}

void cover_detect_unregister(char *cover_type, char *tag)
{
	callback_unregister(DETECT, cover_type, tag);
}

void cover_detect_pin_register(char *tag, void(*function)(bool))
{
	callback_register(DETECT_PIN, "", tag, function);
}

void cover_detect_pin_unregister(char *tag)
{
	callback_unregister(DETECT_PIN, "", tag);
}

void cover_battery_status_register(char *tag, void(*function)(bool))
{
	callback_register(BATTERY_STATUS, "", tag, function);
}

void cover_battery_status_unregister(char *tag)
{
	callback_unregister(BATTERY_STATUS, "", tag);
}

bool is_cover_attached(void)
{
	if(is_probe_successfully)
		return gpio_get_value(cover->detect_gpio);
	else
	{
		pr_err("smart_cover: probe not successfully, so know nothing\n");
		return false;
	}
}

bool is_cover_battery_low(void)
{
	int value;

	if(is_probe_successfully)
	{
		if(!state_change_mutex)
		{
			wake_lock(&cover->wake_lock);
			state_check_for_battery();
			wake_unlock(&cover->wake_lock);
		}

		value = gpio_get_value(cover->battery_status_gpio);
		return (value == 1) ? true : false;
	}
	else
	{
		pr_err("smart_cover: probe not successfully, so know nothing\n");
		return true;
	}
}

int get_cover_type(void)
{
	if(is_probe_successfully)
	{
		if(strcmp(cover->type, "power_bank") == 0)
			return POWER_BANK;
		else if(strcmp(cover->type, "audio_sleeve") == 0)
			return AUDIO_SLEEVE;
		else
			return -1;
	}
	else
	{
		pr_err("smart_cover: probe not successfully, can not get cover type\n");
		return -1;
	}
}

static void callback_list_add(int class, char *cover_type, char *tag, void(*function)(bool))
{
	struct callback_data *item;

	item = kzalloc(sizeof(struct callback_data), GFP_KERNEL);
	if (!item)
		pr_err("smart_cover: Couldn't allocate callback data\n");

	item->class = class;
	item->cover_type = cover_type;
	item->tag = tag;
	item->function = function;
	item->next = NULL;

	callback_cur = callback_head;

	if(callback_cur == NULL)
	{
		callback_head = item;
	}
	else
	{
		while(callback_cur->next != NULL)
			callback_cur = callback_cur->next;

		callback_cur->next = item;
	}
}

static void callback_list_delete(int class, char *cover_type, char *tag)
{
	callback_prev = callback_head;
	callback_cur = callback_head;

	while(callback_cur != NULL)
	{
		if(callback_cur->class == class)
		{
			if(class == DETECT)
			{
				if(strcmp(callback_cur->cover_type, cover_type) == 0 && strcmp(callback_cur->tag, tag) == 0)
					break;
			}
			else
			{
				if(strcmp(callback_cur->tag, tag) == 0)
					break;
			}
		}
		callback_prev = callback_cur;
		callback_cur = callback_cur->next;
	}

	if(callback_cur != NULL)
	{
		//if the deleted item is the head of list
		if(callback_cur == callback_head)
			callback_head = callback_cur->next;
		else
			callback_prev->next = callback_cur->next;

		memset(callback_cur, 0, sizeof(struct callback_data));
		kfree(callback_cur);
	}
}

static void callback_list_clear(void)
{
	callback_prev = callback_head;
	callback_cur = callback_head;

	while(callback_cur != NULL)
	{
		callback_prev = callback_cur;
		callback_cur = callback_cur->next;
		memset(callback_prev, 0, sizeof(struct callback_data));
		kfree(callback_prev);
	}
}

static void callback_register(int class, char *cover_type, char *tag, void(*function)(bool))
{
	int value = -1;

	callback_list_add(class, cover_type, tag, function);

	if(is_checked_detection_during_boot)
	{
		pr_info("smart_cover: cover_detect_register find already checked during boot!!\n");
		value = gpio_get_value(cover->detect_gpio);
		if(value == COVER_ON)
		{
			if(strcmp(cover->type, cover_type) == 0)
			{
				pr_info("smart_cover: cover_detect_register has cover and the type of function matches, so callback this function!!\n");
				function(true);
			}
			else
				pr_info("smart_cover: cover_detect_register has cover but the type of function does not match , so do nothing!!\n");
		}
		else
			pr_info("smart_cover: cover_detect_register has no cover, do nothing!!\n");
	}		
	else
		pr_info("smart_cover: cover_detect_register find no checked during boot, so wait for it to handle!!\n");
}

static void callback_unregister(int class, char *cover_type, char *tag)
{
	callback_list_delete(class, cover_type, tag);
}

static int ACT_read_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;

	if(act_data.is_cached)
	{
		pr_info("smart_cover: activated value has cached\n");
		return snprintf(buf, 2, "%c", act_data.value);
	}
	else
	{
		ret = read_eeprom_for_attr(ACT_ADDR, ACT_SIZE, buf);

		if(*buf != 'x')
		{
			act_data.value = *buf;
			act_data.is_cached = true;
			pr_info("smart_cover: activated value is no cached, so read from eeprom is %c\n", act_data.value);
		}
		else
			pr_err("smart_cover: eeprom can not be read\n");

		return ret;
	}
}

static int ACT_write_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = write_eeprom_for_attr(ACT_ADDR, ACT_SIZE, buf, count);
	if(ret >= 0)
	{
		act_data.value = *buf;
		act_data.is_cached = true;
		pr_info("smart_cover: activated value is written to eeprom, and cached value also change to %c\n", act_data.value);
	}

	return ret;
}

static int EINT10_read_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	int value;

	value = gpio_get_value(cover->detect_gpio);

	return snprintf(buf, 2, "%d", value);
}

static int read_eeprom_for_attr(u8 addr, u8 size, char *buf)
{
	int value;
	unsigned char byte;
	char *data;
	u8 offset = 0;

	data = kzalloc(sizeof(char)*(size+1), GFP_KERNEL);

	for(offset=0; offset<size; offset++)
	{
		value = read_eeprom(addr+offset);
		if(value < 0)
		{
			pr_err("smart_cover: can not read eeprom\n");
			return snprintf(buf, 2, "%c", 'x');
		}
		else
		{
			byte = value & 0xFF;

			//if the data is blank (0xFF), stop reading and return the data we already read
			if(byte == 0xFF)
			{
				*(data+offset) = '\0';
				return snprintf(buf, offset+1, "%s", data);
			}
			else
				*(data+offset) = value & 0xFF;
		}
	}

	*(data+offset) = '\0';

	return snprintf(buf, size+1, "%s", data);
}

static int write_eeprom_for_attr(u8 addr, u8 size, const char *buf, size_t count)
{
	int ret;
	u8 offset;

	for(offset=0; offset<size; offset++)
	{
		ret = write_eeprom(addr+offset, *(buf+offset));
		if(ret < 0)
		{
			pr_err("smart_cover: can not write eeprom\n");
			return ret;
		}
	}

	return strnlen(buf, count);
}

static void callback_trigger(int class, char *cover_type, bool status)
{
	callback_cur = callback_head;

	while(callback_cur != NULL)
	{
		if(callback_cur->class == class)
		{
			if(class == DETECT)
			{
				if(strcmp(callback_cur->cover_type, cover_type) == 0)
					callback_cur->function(status);
			}
			else
				callback_cur->function(status);
		}

		callback_cur = callback_cur->next;
	}
}

static ssize_t print_cover_state(struct switch_dev *sdev, char *buf)
{
	int value;

	value = gpio_get_value(cover->battery_status_gpio);

	pr_info("smart_cover: get EINT6 value is %d\n", value);

	if(value == 0)
		return sprintf(buf, "1\n");
	else
		return sprintf(buf, "0\n");
}

static int write_eeprom(u8 addr, u8 value)
{
	return (ht24lc02_write((u8)addr, (u8)value));
}

static int read_eeprom(u8 addr)
{
	return (ht24lc02_read((u8)addr));
}

static bool identify(void)
{
	int value;
	bool ret;

	pr_info("smart_cover: cover identify\n");

	value = read_eeprom(61);
	pr_info("smart_cover: coverType from eeprom 61 is %d\n", value);

	if(value > 0)
	{
		switch(value)
		{
			case AUDIO_SLEEVE_VALUE:
				cover->type = "audio_sleeve";
				ret = true;
				break;
			case POWER_BANK_VALUE:
				cover->type = "power_bank";
				ret = true;
				break;
			//default now is power bank
			default:
				pr_info("smart_cover: cover type is not supported, but now view it as power bank\n");
				cover->type = "power_bank";
				ret = true;
				break;
		}
	}
	else
	{
		pr_err("smart_cover: can not read eeprom\n");
		ret = false;
	}

	return ret;
}

static void i2c_power_switch(bool on)
{
	int ret = 0;

	if(on)
		ret = gpio_direction_output(cover->i2c_power_gpio, 1);
	else
		ret = gpio_direction_output(cover->i2c_power_gpio, 0);

	msleep(500);

	if (ret < 0)
		pr_err("smart_cover: Failed to set GPIO %d\n", cover->i2c_power_gpio);
}

static void state_check_for_battery(void)
{
	int value;

	pr_info("smart_cover: state_check_for_battery\n");

	switch(cover->state)
	{
		case NO_COVER:
			pr_info("smart_cover: (no_cover_state), cover is off again, do nothing\n");
			break;
		case HAS_COVER:
			value = gpio_get_value(cover->battery_status_gpio);

			if(value == BATTERY_OFF)
			{
				pr_info("smart_cover: (has_cover_state), battery is off\n");
				callback_trigger(DETECT, cover->type, false);
				callback_trigger(BATTERY_STATUS, "", false);
				i2c_power_switch(false);
				cover->state = HAS_COVER_NO_POWER;
			}
			else
				pr_info("smart_cover: (has_cover_state), battery is still on, do nothing\n");
				
			break;
		case HAS_COVER_NO_POWER:
			value = gpio_get_value(cover->battery_status_gpio);

			if(value == BATTERY_ON)
			{
				pr_info("smart_cover: (has_cover_no_power), battery is on\n");
				i2c_power_switch(true);
				if(identify())
				{
					pr_info("smart_cover: type support, cover type is %s\n", cover->type);
					callback_trigger(DETECT, cover->type, true);
					callback_trigger(BATTERY_STATUS, "", true);
					cover->state = HAS_COVER;
				}
				else
				{
					pr_info("smart_cover: type not support\n");
					i2c_power_switch(false);
				}
			}
			else
				pr_info("smart_cover: (has_cover_no_power), battery is still off, do nothing\n");

			break;
	}
}

static void no_cover_state(void)
{
	int value;

	value = gpio_get_value(cover->detect_gpio);

	if(value == COVER_ON)
	{
		pr_info("smart_cover: (%s), cover is on\n", __func__);

		value = gpio_get_value(cover->battery_status_gpio);

		if(value == BATTERY_ON)
		{
			i2c_power_switch(true);

			if(identify())
			{
				pr_info("smart_cover: type support, cover type is %s\n", cover->type);
				callback_trigger(DETECT, cover->type, true);
				callback_trigger(DETECT_PIN, "", true);
				callback_trigger(BATTERY_STATUS, "", true);
				cover->state = HAS_COVER;
			}
			else
			{
				pr_info("smart_cover: type not support\n");
				i2c_power_switch(false);
			}
		}
		else
		{
			pr_info("smart_cover: (%s), battery is off\n", __func__);
			callback_trigger(DETECT_PIN, "", true);
			cover->state = HAS_COVER_NO_POWER;
		}
	}
	else
	{
		pr_info("smart_cover: (%s), cover is off again, do nothing\n", __func__);
	}
}

static void has_cover_state(void)
{
	int value;

	value = gpio_get_value(cover->detect_gpio);

	if(value == COVER_OFF)
	{
		pr_info("smart_cover: (%s), cover is off\n", __func__);
		callback_trigger(DETECT, cover->type, false);
		callback_trigger(DETECT_PIN, "", false);
		callback_trigger(BATTERY_STATUS, "", false);
		i2c_power_switch(false);
		act_data.is_cached = false;
		cover->type = "";
		cover->state = NO_COVER;
	}
	else
	{
		pr_info("smart_cover: (%s), cover is on again, do nothing\n", __func__);
	}
}

static void has_cover_no_power_state(void)
{
	int value;

	value = gpio_get_value(cover->detect_gpio);

	if(value == COVER_OFF)
	{
		pr_info("smart_cover: (%s), cover is off\n", __func__);
		callback_trigger(DETECT_PIN, "", false);
		act_data.is_cached = false;
		cover->type = "";
		cover->state = NO_COVER;
	}
	else
	{
		pr_info("smart_cover: (%s), cover is on again, do nothing\n", __func__);
	}
}

static void work_function(struct work_struct *dat)
{
	//distribute to each state handler
	switch(cover->state)
	{
		case NO_COVER:
			no_cover_state();
			break;
		case HAS_COVER:
			has_cover_state();
			break;
		case HAS_COVER_NO_POWER:
			has_cover_no_power_state();
			break;
	}

	state_change_mutex = 0;
	enable_irq(cover->detect_irq);
	wake_unlock(&cover->wake_lock);
}

static irqreturn_t smart_cover_irq_handler(int irq, void *data)
{
	int value;

	pr_info("smart_cover: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ smart cover interrupt\n");

	//ensure the right irq and eeprom can be read
	if(irq == cover->detect_irq && is_checked_detection_during_boot)
	{
		if(!state_change_mutex)
		{
			wake_lock(&cover->wake_lock);
			state_change_mutex = 1;
			disable_irq_nosync(cover->detect_irq);
			value = gpio_get_value(cover->detect_gpio);

			if(value == COVER_ON)
				schedule_delayed_work(&cover_work, msecs_to_jiffies(cover->attach_debounce_time));
			else
				schedule_delayed_work(&cover_work, msecs_to_jiffies(cover->unattach_debounce_time));
		}
		else
			pr_info("smart_cover: irq handler is working for last irq, so does not serve you\n");
	}

	return IRQ_HANDLED;
}

static int battery_status_gpio_register(struct platform_device *pdev)
{
	int ret = 0;

	cover->battery_status_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "asus,cover-battery-status", 0, NULL);
	pr_info("smart_cover: cover battery status gpio = %d \n", cover->battery_status_gpio);

	if (!gpio_is_valid(cover->battery_status_gpio))
	{
		pr_err("smart_cover: Invalid GPIO %d\n", cover->battery_status_gpio);
		return -1;
	}

	ret = gpio_request(cover->battery_status_gpio, "cover-battery-status");
	if (ret < 0)
	{
		pr_err("smart_cover: request battery status gpio fail!\n");
		return -1;
	}

	ret = gpio_direction_input(cover->battery_status_gpio);
	if (ret < 0)
	{
		pr_err("smart_cover: Failed to configure direction for GPIO %d\n", cover->battery_status_gpio);
		goto fail;
	}

	return 0;

fail:
	gpio_free(cover->battery_status_gpio);
	return -1;
}

static int i2c_power_enable_gpio_register(struct platform_device *pdev)
{
	int ret = 0;

	cover->i2c_power_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "asus,i2c-power-enable", 0, NULL);
	pr_info("smart_cover: cover i2c power gpio = %d \n", cover->i2c_power_gpio);

	if (!gpio_is_valid(cover->i2c_power_gpio))
	{
		pr_err("smart_cover: Invalid GPIO %d\n", cover->i2c_power_gpio);
		return -2;
	}

	ret = gpio_request(cover->i2c_power_gpio, "MULT_I2C_EN");
	if (ret < 0)
	{
		pr_err("smart_cover: request i2c power enable gpio fail!\n");
		return -1;
	}
	
	return 0;
}

static int interrupt_register(struct platform_device *pdev)
{
	int ret = 0;

	cover->detect_gpio = of_get_named_gpio_flags(pdev->dev.of_node, "asus,cover-irq", 0, NULL);
	pr_info("smart_cover: detect gpio = %d \n", cover->detect_gpio);

	if (!gpio_is_valid(cover->detect_gpio))
	{
		pr_err("smart_cover: Invalid GPIO %d\n", cover->detect_gpio);
		return -2;
	}

	ret = gpio_request(cover->detect_gpio, "cover-int");
	if (ret < 0)
	{
		pr_err("smart_cover: request detect gpio fail!\n");
		return -1;
	}

	ret = gpio_direction_input(cover->detect_gpio);
	if (ret < 0)
	{
		pr_err("smart_cover: Failed to configure direction for GPIO %d\n", cover->detect_gpio);
		return -1;
	}

	cover->detect_irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (!cover->detect_irq)
	{
		pr_err("smart_cover: Unable to retrieve IRQ for smart cover\n");
		return -1;
	}

	ret = request_irq(cover->detect_irq, smart_cover_irq_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, "smart_cover", cover);
	if (ret != 0)
	{
		pr_err("smart_cover: Failed to register irq!\n");
		return -1;
	}

	return 0;	
}

static int switch_dev_reg(void)
{
	int ret = 0;

	cover->sdev.name = "smart_cover";
	cover->sdev.state = 0;
	cover->sdev.print_state = print_cover_state;

	ret = switch_dev_register(&cover->sdev);
	if (ret) 
	{
		pr_err("smart_cover: Switch device registration failed!\n");
		return -1;
	}

	return 0;
}

static int workqueue_register(void)
{
	INIT_DEFERRABLE_WORK(&cover_work, work_function);
	return 0;
}

static int create_sysfs_interfaces(void)
{
	int i, ret;

	cover_class = class_create(THIS_MODULE, "cover");
	if (IS_ERR(cover_class)) {
		ret = PTR_ERR(cover_class);
		cover_class = NULL;
		pr_info("smart_cover: could not allocate cover_class\n");
		goto cover_class_error;
	}

	cover_dev= device_create(cover_class, NULL, 0, "%s", "smart_cover");

	if(cover_dev == NULL)
		goto cover_device_error;

	for(i=0; i<ARRAY_SIZE(cover_attrs); i++)
		if (device_create_file(cover_dev, cover_attrs[i]))
			goto cover_create_file_error;

	return 0;

cover_create_file_error:
	device_remove_file(cover_dev, cover_attrs[i]);
cover_device_error:
	class_destroy(cover_class);
cover_class_error:
	pr_info("smart_cover: unable to create interface\n");
	return -1;
}

static int smart_cover_probe(struct platform_device *pdev)
{
	int ret = 0;
	
	pr_info("smart_cover: smart cover probe!\n");
	cover = kzalloc(sizeof(struct smart_cover_data), GFP_KERNEL);
	if(!cover)
	{
		pr_err("smart_cover: Couldn't allocate driver data\n");
		goto fail;
	}
	cover->attach_debounce_time = DETECT_ON_DEBOUNCE_TIME;
	cover->unattach_debounce_time = DETECT_OFF_DEBOUNCE_TIME;
	cover->type = "";
	cover->state = NO_COVER;

	act_data.is_cached = false;
	act_data.value = 0;

	ret = workqueue_register();
	if(ret == -1)
		goto fail;

	ret = i2c_power_enable_gpio_register(pdev);
	if(ret == -1)
		goto fail_i2c_power;
	else if(ret == -2)
		goto fail;

	ret = interrupt_register(pdev);
	if(ret == -1)
		goto fail_interrupt;
	else if(ret == -2)
		goto fail_i2c_power;

	ret = battery_status_gpio_register(pdev);
	if(ret == -1)
		goto fail_interrupt;

	ret = switch_dev_reg();
	if(ret == -1)
		goto fail_interrupt;

	ret = create_sysfs_interfaces();
	if(ret)
		goto fail_switch;

	wake_lock_init(&cover->wake_lock, WAKE_LOCK_SUSPEND, "smart_cover");

	is_probe_successfully = true;

	return ret;

fail_switch:
	switch_dev_unregister(&cover->sdev);
fail_interrupt:
	gpio_free(cover->detect_gpio);
fail_i2c_power:
	gpio_free(cover->i2c_power_gpio);
fail:
	ret = -EINVAL;
	return ret;
}

static int smart_cover_remove(struct platform_device *pdev)
{
	int i;

	free_irq(cover->detect_irq, NULL);
	cancel_delayed_work_sync(&cover_work);
	for(i=0; i<ARRAY_SIZE(cover_attrs); i++)
		device_remove_file(cover_dev, cover_attrs[i]);
	class_destroy(cover_class);
	switch_dev_unregister(&cover->sdev);
	if(gpio_is_valid(cover->detect_gpio))
		gpio_free(cover->detect_gpio);
	if(gpio_is_valid(cover->battery_status_gpio))
		gpio_free(cover->battery_status_gpio);
	if(gpio_is_valid(cover->i2c_power_gpio))
		gpio_free(cover->i2c_power_gpio);
	callback_list_clear();
	memset(cover, 0, sizeof(struct smart_cover_data));
	kfree(cover);
	is_probe_successfully = false;
	is_checked_detection_during_boot = false;
	state_change_mutex = 0;
	wake_lock_destroy(&cover->wake_lock);

	return 0;
}

static int smart_cover_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("smart_cover: suspend\n");

	//reset variables if irq work is interrupted by suspend
	if(state_change_mutex)
	{
		state_change_mutex = 0;
		enable_irq(cover->detect_irq);
		wake_unlock(&cover->wake_lock);
	}
	disable_irq(cover->detect_irq);
	cancel_delayed_work_sync(&cover_work);

	return 0;
}

static int smart_cover_resume(struct platform_device *pdev)
{
	pr_info("smart_cover: resume\n");
	enable_irq(cover->detect_irq);
	return 0;
}

static struct of_device_id smart_cover_of_match[] = {
    { .compatible = "asus,smart_cover", },
    { },
};

static struct platform_driver smart_cover_driver = {
	.probe		= smart_cover_probe,
	.remove		= smart_cover_remove,
	.suspend	= smart_cover_suspend,
	.resume		= smart_cover_resume,
	.driver		= {
		.name	= "smart_cover",
		.owner	= THIS_MODULE,
        	.of_match_table = smart_cover_of_match,
	},
};

static int __init smart_cover_init(void)
{
	int hardware_id = 0;

	hardware_id = asustek_boardinfo_get(FUN_HARDWARE_ID);

	pr_info("smart_cover: hardware id is %d\n", hardware_id);

	//SR:0 ER:1 ER2:2 prePR:3 PR:4
	if(hardware_id >= ER2)
	{
		pr_info("smart_cover: ER2 or newer than ER2, init!\n");
		return platform_driver_register(&smart_cover_driver);
	}
	else
	{
		pr_info("smart_cover: older than ER2, do not init!\n");
		return -EPERM;
	}
}
static void __exit smart_cover_exit(void)
{
	platform_driver_unregister(&smart_cover_driver);
}

late_initcall(smart_cover_init);
module_exit(smart_cover_exit);

MODULE_DESCRIPTION("Smart cover detection");
MODULE_LICENSE("GPL");
