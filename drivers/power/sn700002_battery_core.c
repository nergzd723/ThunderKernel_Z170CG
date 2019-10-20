/*
 * Copyright (c) 2015, ASUSTek, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <asm/unaligned.h>
#include <linux/miscdevice.h>
#include <linux/wakelock.h>
#include <linux/proc_fs.h>
#include <linux/poll.h>
#include <linux/jiffies.h>

#include <linux/power/smb345-charger.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>

#include "asus_battery.h"

DEFINE_MUTEX(sn700002_dev_info_mutex);

#define SMBUS_RETRY                                      3

#define GPIOPIN_LOW_BATTERY_DETECT	                 36
#define BATTERY_POLLING_RATE	                         60
#define DELAY_FOR_CORRECT_CHARGER_STATUS	         5
#define TEMP_KELVIN_TO_CELCIUS		                 2731
#define MAXIMAL_VALID_BATTERY_TEMP	                 200

#define BATTERY_MANUFACTURER_SIZE	                 12
#define BATTERY_NAME_SIZE                                8

/* Battery flags bit definitions */
#define BATT_STS_DSG                                     0x0001
#define BATT_STS_FC                                      0x0200

/* Debug Message */
#define BAT_NOTICE(format, arg...) \
	pr_notice("%s " format , __func__ , ## arg)
#define BAT_ERR(format, arg...) \
	pr_err(format , ## arg)

/* Global variable */
unsigned battery_cable_status = 0;
unsigned battery_driver_ready = 0;
static int ac_on;
static int usb_on;
static unsigned int	battery_current;
static unsigned int     battery_remaining_capacity;
struct workqueue_struct *battery_poll_work_queue = NULL;
static int g_battery_id = 1;
static int g_fw_cfg_version;
static int g_fw_gauge_id;

/* Functions declaration */
static int sn700002_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static int bq27621_unseal(void);
static void main_force_update_flow(void);
static void main_update_flow(void);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#ifdef ASUS_ENG_BUILD
bool eng_charging_limit = true;
#endif


#define sn700002_IOC_MAGIC 0xF9
#define sn700002_IOC_MAXNR 5
#define sn700002_POLL_DATA _IOR(sn700002_IOC_MAGIC, sn700002_IOC_MAXNR, int)

#define sn700002_IOCTL_START_HEAVY 2
#define sn700002_IOCTL_START_NORMAL 1
#define sn700002_IOCTL_END 0

#define START_NORMAL    (10 * (HZ))
#define START_HEAVY     (HZ)
static int stress_test_poll_mode;
struct delayed_work sn700002_stress_test_poll_work;
static struct workqueue_struct *sn700002_stress_test_work_queue;
static wait_queue_head_t poll_wait_queue_head_t;
static bool flag_pollin = true;

#define BIT0  0x00000001
#define BIT1  0x00000002
#define BIT2  0x00000004
#define BIT3  0x00000008
#define BIT4  0x00000010
#define BIT5  0x00000020
#define BIT6  0x00000040
#define BIT7  0x00000080

#define BIT8  0x00000100
#define BIT9  0x00000200
#define BIT10 0x00000400
#define BIT11 0x00000800
#define BIT12 0x00001000
#define BIT13 0x00002000
#define BIT14 0x00004000
#define BIT15 0x00008000

#define sn700002_DATA(_psp, _addr, _min_value, _max_value)	\
	{								\
		.psp = POWER_SUPPLY_PROP_##_psp,	\
		.addr = _addr,				\
		.min_value = _min_value,		\
		.max_value = _max_value,	\
	}

enum {
	REG_MANUFACTURER_DATA,
	REG_STATE_OF_HEALTH,
	REG_TEMPERATURE,
	REG_VOLTAGE,
	REG_CURRENT,
	REG_TIME_TO_EMPTY,
	REG_TIME_TO_FULL,
	REG_STATUS,
	REG_CAPACITY,
	REG_SERIAL_NUMBER,
	REG_MAX
};

enum {
	UNKNOWN,
	CHARGING,
	DISCHARGING,
	NOTCHARGING,
	FULL,
};

enum charger_type {
	CHARGER_TYPE_BATTERY = 0,
	CHARGER_TYPE_AC,
	CHARGER_TYPE_USB,
	CHARGER_TYPE_NUM,
	CHARGER_TYPE_FORCE32 = 0x7FFFFFFF
};

static struct sn700002_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} sn700002_data[] = {
	[REG_MANUFACTURER_DATA] = sn700002_DATA(PRESENT, 0, 0, 65535),
	[REG_STATE_OF_HEALTH] = sn700002_DATA(HEALTH, 0, 0, 65535),
	[REG_TEMPERATURE] = sn700002_DATA(TEMP, 0x02, 0, 65535),
	[REG_VOLTAGE] = sn700002_DATA(VOLTAGE_NOW, 0x04, 0, 6000),
	[REG_CURRENT] = sn700002_DATA(CURRENT_NOW, 0x10, -32768, 32767),
	[REG_CAPACITY] = sn700002_DATA(CAPACITY, 0x1c, 0, 100),
};

static enum power_supply_property sn700002_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_AMBIENT,
};

static bool is_charging_full(void)
{
	int charger_status;

	charger_status = smb345_get_charging_status();
	if (charger_status == POWER_SUPPLY_STATUS_FULL) {
		BAT_NOTICE("[%s] charging full\n", __func__);
		return true;
	} else {
		return false;
	}
}

static bool is_charging(void)
{
	int charger_status;

	charger_status = smb345_get_charging_status();

	if ((charger_status == POWER_SUPPLY_STATUS_FULL) ||
		(charger_status == POWER_SUPPLY_STATUS_CHARGING)) {
		return true;
	} else {
		return false;
	}
}

void check_cabe_type(void)
{
	if (battery_cable_status == USB_ADAPTER) {
		ac_on = 1;
		usb_on = 0;
	} else if (battery_cable_status == USB_PC) {
		usb_on = 1;
		ac_on  = 0;
	} else {
		ac_on  = 0;
		usb_on = 0;
	}
}

#ifdef SN700002_CALLBACK_FUNC
static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on)
			val->intval =  1;
		else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval =  1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (psy->type == POWER_SUPPLY_TYPE_USB
				|| psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (!smb345_has_charger_error() ||
				(smb345_get_charging_status()
					== POWER_SUPPLY_STATUS_CHARGING))
				val->intval = 1;
			else
				val->intval = 0;
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
	return ret;
}
#endif

#ifdef SN700002_CALLBACK_FUNC
static char *supply_list[] = {
	"battery",
	"ac",
	"usb",
};
#endif

static void sn700002_battery_external_power_changed(struct power_supply *psy)
{

	if (power_supply_am_i_supplied(psy)) {
		switch (get_charger_type()) {
		case USB_IN:
			battery_cable_status = USB_PC;
			BAT_NOTICE("[%s] USB_PC_CABLE\n", __func__);
			break;
		case AC_IN:
			battery_cable_status = USB_ADAPTER;
			BAT_NOTICE("[%s] AC_ADAPTER_CABLE\n", __func__);
			break;
		case CABLE_OUT:
			battery_cable_status = NO_CABLE;
			BAT_NOTICE("[%s] NO_CABLE\n", __func__);
			break;
		}
	} else {
		battery_cable_status = NO_CABLE;
		BAT_NOTICE("[%s] NO_CABLE\n", __func__);
	}
	check_cabe_type();
}

static struct power_supply sn700002_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= sn700002_properties,
		.num_properties = ARRAY_SIZE(sn700002_properties),
		.get_property	= sn700002_get_property,
#ifndef	SN700002_CALLBACK_FUNC
		.external_power_changed	=
			sn700002_battery_external_power_changed,
#endif
	},
#ifdef SN700002_CALLBACK_FUNC
	{
		.name		= "ac",
		.type		= POWER_SUPPLY_TYPE_MAINS,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property	= power_get_property,
	},
	{
		.name		= "usb",
		.type		= POWER_SUPPLY_TYPE_USB,
		.supplied_to	= supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
};

static struct sn700002_device_info {
	struct i2c_client *client;
	struct work_struct fw_work;
	struct delayed_work battery_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct miscdevice battery_misc;
	struct wake_lock low_battery_wake_lock;
	struct wake_lock cable_event_wake_lock;
	int smbus_status;
	int battery_present;
	int low_battery_present;
	int gpio_battery_detect;
	int gpio_low_battery_detect;
	int irq_low_battery_detect;
	int irq_battery_detect;
	int bat_status;
	int bat_temp;
	int bat_vol;
	int bat_current;
	int bat_capacity;
	unsigned int old_capacity;
	unsigned int cap_err;
	unsigned int old_temperature;
	unsigned int temp_err;
	unsigned int prj_id;
	spinlock_t lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct  early_suspend es;
#endif
} *sn700002_device;

static int sn700002_read_i2c(u8 reg, int *rt_value, int b_single)
{
	struct i2c_client *client = sn700002_device->client;
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	} else {
		dev_err(&client->dev,
			"I2C read from addr 0x%02X error with code: %d\n",
			reg, err);
	}

	return err;
}

int sn700002_smbus_read_data(int reg_offset, int byte, int *rt_value)
{
	s32 ret = -EINVAL;
	int count = 0;

	mutex_lock(&sn700002_dev_info_mutex);
	do {
		ret = sn700002_read_i2c(sn700002_data[reg_offset].addr,
								rt_value, 0);
	} while ((ret < 0) && (++count <= SMBUS_RETRY));
	mutex_unlock(&sn700002_dev_info_mutex);

	return ret;
}

int sn700002_smbus_read_data_reg(u8 reg, int byte, int *rt_value)
{
	s32 ret = -EINVAL;
	int count = 0;

	mutex_lock(&sn700002_dev_info_mutex);
	do {
		ret = sn700002_read_i2c(reg, rt_value, 0);
	} while ((ret < 0) && (++count <= SMBUS_RETRY));
	mutex_unlock(&sn700002_dev_info_mutex);

	return ret;
}

int sn700002_smbus_write_data(int reg_offset, int byte, unsigned int value)
{
	s32 ret = -EINVAL;
	int count = 0;

	do {
		if (byte)
			ret = i2c_smbus_write_byte_data(sn700002_device->client,
					sn700002_data[reg_offset].addr,
					value & 0xFF);
		else
			ret = i2c_smbus_write_word_data(sn700002_device->client,
					sn700002_data[reg_offset].addr,
					value & 0xFFFF);
	} while ((ret < 0) && (++count <= SMBUS_RETRY));

	return ret;
}

static ssize_t show_battery_smbus_status(struct device *dev,
				struct device_attribute *devattr, char *buf)
{
	int status = !sn700002_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO,
		show_battery_smbus_status, NULL);

static struct attribute *battery_smbus_attributes[] = {
	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

static int sn700002_get_capacity(void)
{
	s32 temp_capacity;
	int remainingcapacity = 0;
	int retry_count = SMBUS_RETRY;

	do {
		sn700002_device->smbus_status = sn700002_smbus_read_data(
						REG_CAPACITY,
						0,
						&sn700002_device->bat_capacity);
		if (sn700002_device->smbus_status < 0) {
			dev_err(&sn700002_device->client->dev,
					"%s: i2c read for %d failed sn700002_device->cap_err=%u\n",
					__func__, REG_CAPACITY,
					sn700002_device->cap_err);
			if ((sn700002_device->cap_err > 5)
				|| (sn700002_device->old_capacity == 0xFF)) {
				return -EINVAL;
			} else {
				sn700002_device->cap_err++;
				BAT_NOTICE("cap_err=%u use old capacity=%u\n",
						sn700002_device->cap_err,
						sn700002_device->old_capacity);
				return 0;
			}
		}

		retry_count--;
		if (sn700002_device->bat_capacity > 0)
			break;
		else {
			BAT_DBG_E("%s: Battery capacity = %d, retry %d times\n",
					__func__, sn700002_device->bat_capacity,
					retry_count);
			msleep(50);
		}
	} while (retry_count > 0);

	temp_capacity = ((sn700002_device->bat_capacity >= 100) ?
					100 : sn700002_device->bat_capacity);
	/* start: for mapping %99 to 100%. Lose 84%*/
	if (temp_capacity == 99)
		temp_capacity = 100;
	if ((temp_capacity >= 84) && (temp_capacity <= 98))
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if ((temp_capacity > 70) && (temp_capacity < 80))
		temp_capacity -= 1;
	else if ((temp_capacity > 60) && (temp_capacity <= 70))
		temp_capacity -= 2;
	else if ((temp_capacity > 50) && (temp_capacity <= 60))
		temp_capacity -= 3;
	else if ((temp_capacity > 30) && (temp_capacity <= 50))
		temp_capacity -= 4;
	else if ((temp_capacity >= 0) && (temp_capacity <= 30))
		temp_capacity -= 5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity < 0) ? 0 : temp_capacity);

	sn700002_device->old_capacity = temp_capacity;
	sn700002_device->cap_err = 0;

	BAT_NOTICE("user capacity:%u%% ,gauge capacity: %u%%\n",
			sn700002_device->old_capacity,
			sn700002_device->bat_capacity);

	/* for PSChange App tool */
	sn700002_device->smbus_status = sn700002_smbus_read_data_reg(0x0C,
							0 , &remainingcapacity);
	if (sn700002_device->smbus_status >= 0) {
		battery_remaining_capacity = remainingcapacity;
		BAT_NOTICE("remaining capacity= %d\n", remainingcapacity);
	}
	return 0;
}

static int sn700002_get_current(void)
{
	int rt_value = 0;

	sn700002_device->smbus_status =
		sn700002_smbus_read_data_reg(sn700002_data[REG_CURRENT].addr,
								0, &rt_value);
	if (sn700002_device->smbus_status < 0) {
		dev_err(&sn700002_device->client->dev,
			"%s: i2c read for %d failed\n",
			__func__, sn700002_data[REG_CURRENT].addr);
	}

	if (rt_value & BIT15)
		rt_value |= 0xFFFF0000;

	rt_value += 0x10000;

	if (rt_value >= 0)
		rt_value -= 0x10000;

	sn700002_device->bat_current = rt_value;

	return rt_value;
}

static int sn700002_get_temperature(void)
{
	int rt_value = 0;

	sn700002_device->smbus_status =
		sn700002_smbus_read_data_reg(
			sn700002_data[REG_TEMPERATURE].addr, 0, &rt_value);
	if (sn700002_device->smbus_status < 0)
		dev_err(&sn700002_device->client->dev,
				"%s: i2c read for %d failed\n",
				__func__, sn700002_data[REG_TEMPERATURE].addr);

	sn700002_device->bat_temp = rt_value;
	sn700002_device->old_temperature =
		sn700002_device->bat_temp - TEMP_KELVIN_TO_CELCIUS;

	return (rt_value - TEMP_KELVIN_TO_CELCIUS);
}

void sn700002_update_all(void)
{
	s32 ret;
	int rt_value = 0;
	static const char * const status_text[] = {
		"Unknown", "Charging",
		"Discharging", "Not charging", "Full"
	};

	/* 1. capacity */
	sn700002_get_capacity();

	/* 2. voltage */
	sn700002_device->smbus_status =
		sn700002_smbus_read_data_reg(sn700002_data[REG_VOLTAGE].addr,
								0, &rt_value);
	if (sn700002_device->smbus_status < 0)
		dev_err(&sn700002_device->client->dev,
				"%s: i2c read for %d failed\n",
				__func__, sn700002_data[REG_VOLTAGE].addr);
	sn700002_device->bat_vol = rt_value;
	BAT_NOTICE("voltage_now= %d mV\n", sn700002_device->bat_vol);

	/* 3. current */
	sn700002_get_current();
	BAT_NOTICE("current_now= %d mA\n", sn700002_device->bat_current);

	/* 4. temperature */
	sn700002_get_temperature();
	BAT_NOTICE("temperature= %d (0.1 Celsius degrees)\n",
				sn700002_device->old_temperature);

	/* 5. status */
	aicl_dete_worker(NULL);

	ret = is_charging_full() ?
			FULL : is_charging() ?
				CHARGING : DISCHARGING;
	sn700002_device->bat_status = ret;
	BAT_NOTICE("status: %s\n", status_text[ret]);

	return;
}

static void sn700002_stress_test_poll(struct work_struct *work)
{
	unsigned long polling_time = stress_test_poll_mode;
	int ret = 0, percentage = 50;

	if (!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		polling_time = msecs_to_jiffies(1000);
		goto out;
	}

	ret = sn700002_smbus_read_data_reg(sn700002_data[REG_CAPACITY].addr,
							0 , &percentage);
	if (ret < 0)
		BAT_ERR("sn700002 gauge i2c fail !!!!!\n");
	else
		BAT_NOTICE("stress test percentage = %d, polling = %d sec\n",
					percentage , stress_test_poll_mode / HZ);
	if (!smb345_has_charger_error())
		BAT_NOTICE("smb345 charger status success, polling = %d sec\n",
					stress_test_poll_mode / HZ);
	else
		BAT_ERR("smb345 charger status fail !!!!!\n");

out:
	queue_delayed_work(sn700002_stress_test_work_queue,
			&sn700002_stress_test_poll_work, polling_time);
}

int sn700002_open(struct inode *inode, struct file *filp)
{
	return 0;
}

int sn700002_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int sn700002_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;

	poll_wait(filp, &poll_wait_queue_head_t, wait);
	if (flag_pollin == true) {
		mask |= POLLIN;
		flag_pollin = false;
	}

	return mask;
}

long sn700002_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;
	if (_IOC_TYPE(cmd) != sn700002_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > sn700002_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
				(void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ,
				(void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
	case sn700002_POLL_DATA:
		if (arg == sn700002_IOCTL_START_HEAVY) {
			pr_debug("battery ba27541 : ioctl heavy\n");
			stress_test_poll_mode = START_HEAVY;
			queue_delayed_work(sn700002_stress_test_work_queue,
					&sn700002_stress_test_poll_work,
					stress_test_poll_mode);
		} else if (arg == sn700002_IOCTL_START_NORMAL) {
			pr_err("battery ba27541 : ioctl normal\n");
			stress_test_poll_mode = START_NORMAL;
			queue_delayed_work(sn700002_stress_test_work_queue,
					&sn700002_stress_test_poll_work,
					stress_test_poll_mode);
		} else if  (arg == sn700002_IOCTL_END) {
			pr_err("light sensor sn700002 : ioctl end\n");
			cancel_delayed_work_sync(
					&sn700002_stress_test_poll_work);
		} else
			return -ENOTTY;
		break;
	default: /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations sn700002_fops = {
	.owner = THIS_MODULE,
	.open = sn700002_open,
	.release = sn700002_release,
	.poll = sn700002_poll,
	.unlocked_ioctl = sn700002_ioctl,
};

static void sn700002_fw_work(struct work_struct *work)
{
	main_update_flow();
	battery_driver_ready = 1;
}

static void battery_status_poll(struct work_struct *work)
{
	unsigned long polling_time = msecs_to_jiffies(60000);
	struct sn700002_device_info *batt_dev =
		container_of(work, struct sn700002_device_info,
				status_poll_work.work);
	const char * const status_text[] = {
		"Unknown", "Charging", "Discharging",
		"Not charging", "Full"
	};

	if (!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		polling_time = msecs_to_jiffies(1000);
		goto out;
	}

	sn700002_update_all();

	power_supply_changed(&sn700002_supply[CHARGER_TYPE_BATTERY]);

	mdelay(10);

	if (sn700002_device->old_temperature >= 630) {
		BAT_NOTICE("Critical condition!! -> temperature\n");
		polling_time = BATTERY_CRITICAL_POLL_TIME;
	} else if (sn700002_device->old_temperature >= 500) {
		BAT_NOTICE("Nearly critical condition!! -> temperature\n");
		polling_time = 10 * HZ;
	}

	if ((sn700002_device->old_capacity >= 50)
				&& (sn700002_device->old_capacity <= 100))
		polling_time = (60 * HZ) < polling_time ?
					(60 * HZ) : polling_time;
	else if ((sn700002_device->old_capacity >= 20)
				&& (sn700002_device->old_capacity <= 49))
		polling_time = (30 * HZ) < polling_time ?
					(30 * HZ) : polling_time;
	else if ((sn700002_device->old_capacity >= 5)
			&& (sn700002_device->old_capacity <= 19))
		polling_time = (10 * HZ) < polling_time ?
					(10 * HZ) : polling_time;
	else if ((sn700002_device->old_capacity >= 0)
			&& sn700002_device->old_capacity <= 4)
		polling_time = (5 * HZ) < polling_time ?
			(5 * HZ) : polling_time;
	else {
		BAT_NOTICE(
			"*** Batt percentage out of range (<0 or >100) ***\n");
		polling_time = 5 * HZ;
	}

	pr_debug("<BATT> battery info ");
	pr_debug("(P:%d %%(%d %%), V:%d mV, C:%d mA, T:%d.%d P: %d secs S: %s)\n",
		sn700002_device->old_capacity,
		sn700002_device->bat_capacity,
		sn700002_device->bat_vol,
		sn700002_device->bat_current,
		sn700002_device->old_temperature/10,
		sn700002_device->old_temperature%10,
		polling_time/HZ,
		status_text[sn700002_device->bat_status]);

out:
	/* Schedule next polling */
	queue_delayed_work(battery_poll_work_queue,
			&batt_dev->status_poll_work, polling_time);
}

static void low_low_battery_check(struct work_struct *work)
{
	cancel_delayed_work(&sn700002_device->status_poll_work);
	queue_delayed_work(battery_poll_work_queue,
			   &sn700002_device->status_poll_work, msecs_to_jiffies(100));
	msleep(2000);
	enable_irq(sn700002_device->irq_low_battery_detect);
}

#ifdef SN700002_CALLBACK_FUNC
void sn700002_battery_callback(unsigned USB_PC_state)
{
	int old_cable_status = battery_cable_status;
	battery_cable_status = USB_PC_state;
	unsigned long polling_time = msecs_to_jiffies(100);

	pr_debug("sn700002_battery_callback USB_PC_state = %x\n", USB_PC_state);

	if (!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		polling_time = msecs_to_jiffies(1000);
		goto out;
	}

	check_cabe_type();

out:
	cancel_delayed_work(&sn700002_device->status_poll_work);
	queue_delayed_work(battery_poll_work_queue,
			   &sn700002_device->status_poll_work, polling_time);
}
EXPORT_SYMBOL(sn700002_battery_callback);
#endif

static int sn700002_get_health(enum power_supply_property psp,
				union power_supply_propval *val)
{
	if (psp == POWER_SUPPLY_PROP_PRESENT)
		val->intval = 1;
	else if (psp == POWER_SUPPLY_PROP_HEALTH)
		val->intval = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sn700002_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val) {

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_HEALTH:
		if (sn700002_get_health(psp, val))
			goto error;
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;

	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = sn700002_device->old_capacity;
		break;

	case POWER_SUPPLY_PROP_STATUS:
		if (is_charging()) {
			if (is_charging_full())
				val->intval = POWER_SUPPLY_STATUS_FULL;
			else
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
		} else {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = sn700002_device->bat_vol*1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = sn700002_device->bat_current;
		break;
	case POWER_SUPPLY_PROP_TEMP:
	case POWER_SUPPLY_PROP_TEMP_AMBIENT:
		val->intval = sn700002_device->old_temperature;
		break;
	default:
		dev_err(&sn700002_device->client->dev,
				"%s: INVALID property psp=%u\n", __func__, psp);
		return -EINVAL;
	}
	return 0;
error:
	return -EINVAL;
}

static int sn700002_proc_info_dump_read(struct seq_file *m, void *p)
{
	static int bq_batt_percentage;
	static int bq_batt_volt;
	static int bq_batt_current;
	static int bq_batt_temp;
	static int bq_batt_remaining_capacity;
	static int bq_batt_full_charge_capacity;

	sn700002_smbus_read_data_reg(0x0E, 0, &bq_batt_full_charge_capacity);
	sn700002_smbus_read_data_reg(0x0C, 0, &bq_batt_remaining_capacity);
	sn700002_smbus_read_data_reg(sn700002_data[REG_CAPACITY].addr,
						0, &bq_batt_percentage);
	sn700002_smbus_read_data_reg(sn700002_data[REG_VOLTAGE].addr,
						0, &bq_batt_volt);
	bq_batt_current = sn700002_get_current();
	bq_batt_temp    = sn700002_get_temperature();

	seq_printf(m, "LMD(mAh): %d\n", bq_batt_full_charge_capacity);
	seq_printf(m, "NAC(mAh): %d\n", bq_batt_remaining_capacity);
	seq_printf(m, "RSOC: %d\n", bq_batt_percentage);
	seq_printf(m, "USOC: %d\n", sn700002_device->old_capacity);
	seq_printf(m, "voltage(mV): %d\n", bq_batt_volt);
	seq_printf(m, "average_current(mA): %d\n", bq_batt_current);
	seq_printf(m, "temp: %d\n", bq_batt_temp);

	return 0;
}

static int proc_sn700002_test_info_dump_open(struct inode *inode,
							struct file *file)
{
	return single_open(file, sn700002_proc_info_dump_read, NULL);
}

static const struct file_operations proc_sn700002_test_info_dump_ops = {
	.open		= proc_sn700002_test_info_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};

static int sn700002_register_upilogger_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("bq27520_test_info_dump",
			0664, NULL, &proc_sn700002_test_info_dump_ops);
	if (!entry) {
		pr_err("[%s] Unable to create sn700002_test_info_dump\n",
				__func__);
		return -EINVAL;
	}
	return 0;
}

static bool checkBatteryInvalidStatus(void)
{
	int   capacity;

	sn700002_smbus_read_data_reg(sn700002_data[REG_CAPACITY].addr,
								0 , &capacity);

	/*  check if battery capacity is too low,
	 *  if yes notify system to do shutdown */
	if (capacity <= 5) {
		BAT_NOTICE("===  battery capacity is too low in sleep!!!\n");
		return true;
	}
	return false;
}

#if defined(CONFIG_PM)
static int sn700002_suspend(struct device *dev)
{
	cancel_delayed_work_sync(&sn700002_device->status_poll_work);
	return 0;
}

/* any smbus transaction will wake up pad */
static int sn700002_resume(struct device *dev)
{
	if (checkBatteryInvalidStatus())
		queue_delayed_work(battery_poll_work_queue,
				&sn700002_device->status_poll_work, 0);
	else
		queue_delayed_work(battery_poll_work_queue,
				&sn700002_device->status_poll_work,
				5 * HZ);
	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sn700002_early_suspend(struct early_suspend *h)
{
}

static void sn700002_late_resume(struct early_suspend *h)
{
	queue_delayed_work(battery_poll_work_queue,
			&sn700002_device->status_poll_work, 0);
}
#endif

#define LATEST_GAUGE_ID_LGC             0x3489
#define LATEST_GAUGE_ID_ATL             0x3490
#define LATEST_FW_CFG_VERSION           0x00
#define LATEST_FW_CFG_VERSION_LGC       6
#define LATEST_FW_CFG_VERSION_ATL       5

enum update_status {
	UPDATE_GAUGE_VERSION = -6,
	UPDATE_INVALID_BATTID = -5,
	UPDATE_PROCESS_FAIL = -4,
	UPDATE_ERR_MATCH_OP_BUF = -3,
	UPDATE_CHECK_MODE_FAIL = -2,
	UPDATE_VOLT_NOT_ENOUGH = -1,
	UPDATE_NONE = 0,
	UPDATE_OK,
	UPDATE_FROM_ROM_MODE,
};

#define OP_ROM_READ     0
#define OP_ROM_WRITE    1
#define OP_ROM_CMP      2
#define OP_ROM_END      3
#define OP_I2C_START    4
#define OP_I2C_READ     5
#define OP_I2C_WRITE    6
#define OP_I2C_CMP      7
#define OP_WAIT         8

#define RETRY_COUNT 3

#define PROC_TRUE -256
#define PROC_FALSE -257
#define ERROR_CODE_I2C_FAILURE    -99999

#define BATTERY_ID_FILE_PATH "/factory/battery_cell_id"

struct update_op {
	u32 bq_op;
	u32 off; /* OP_WAIT not use */
	u32 arg;
};

enum cell_type {
	TYPE_COS_LIGHT = 0,
	TYPE_LG,
	TYPE_ATL,
};

struct bq27xx_dffs_data {
	u32 cell_type;
	u32 num_op;
	struct update_op *op;
};

static struct bq27xx_dffs_data *default_bq27xx_fw;

static int asus_get_battery_id(void)
{
	int ret = 0;
	int len = 0;
	char buf[30] = {0};
	struct file *fp = NULL;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(BATTERY_ID_FILE_PATH, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		BAT_DBG_E("%s: open file failed, ret = %lu\n",
				__func__, IS_ERR(fp));
		return IS_ERR(fp)*(-1);
	}
	len = fp->f_op->read(fp, buf, 30, &fp->f_pos);
	BAT_DBG("%s: file length = %d, content = %s\n", __func__, len, buf);
	set_fs(old_fs);
	filp_close(fp, NULL);

	if (buf[0] == '0')
		g_battery_id = 0;
	if (buf[0] == '1')
		g_battery_id = 1;

	return ret;
}

static int bq27520_write_i2c(struct i2c_client *client, u8 reg,
						int value, int b_single)
{
	struct i2c_msg msg;
	unsigned char data[3];

	if (!client || !client->adapter)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = value & 0x00FF;
	data[2] = (value & 0xFF00) >> 8;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = b_single ? 2 : 3;
	msg.buf = data;

	if (i2c_transfer(client->adapter, &msg, 1) < 0) {
#ifdef ASUS_ENG_BUILD
		dev_err(&client->dev, "I2C write to 0x%02X error with code: %d\n",
			reg, -EIO);
#endif
		return -EIO;
	}

	return 0;
}

static int bq27520_read_i2c(struct i2c_client *client, u8 reg,
					int *rt_value, int b_single)
{
	struct i2c_msg msg[1];
	unsigned char data[2];
	int err;

	if (!client || !client->adapter)
		return -ENODEV;

	msg->addr = client->addr;
	msg->flags = 0;
	msg->len = 1;
	msg->buf = data;

	data[0] = reg;
	err = i2c_transfer(client->adapter, msg, 1);

	if (err >= 0) {
		if (!b_single)
			msg->len = 2;
		else
			msg->len = 1;

		msg->flags = I2C_M_RD;
		err = i2c_transfer(client->adapter, msg, 1);
		if (err >= 0) {
			if (!b_single)
				*rt_value = get_unaligned_le16(data);
			else
				*rt_value = data[0];

			return 0;
		}
	}
#ifdef ASUS_ENG_BUILD
	dev_err(&client->dev, "I2C read from addr 0x%02X error with code: %d\n",
		reg, err);
#endif
	return err;
}

static int bq27520_cmp_i2c(int reg_off, int value)
{
	int retry = 3;
	int val = 0;
	int ret = 0;

	BAT_NOTICE("[%s] enter\n", __func__);

	while (retry--) {
		ret = bq27520_read_i2c(sn700002_device->client,
						reg_off, &val, 1);
		if (ret < 0)
			continue;
		break;
	};

	if (!retry && ret < 0)
		return ret;

	return val == value ? PROC_TRUE : PROC_FALSE;
}
#ifdef NOT_YET
static int bq27520_asus_battery_dev_read_fw_cfg_version(void)
{
	int fw_cfg_ver = 0;
	int ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x3F, 0x01, 1);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Get fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}

	udelay(800);
	ret = bq27520_read_i2c(sn700002_device->client, 0x40, &fw_cfg_ver, 0);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Read fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}

	return fw_cfg_ver;
}

static int bq27621_asus_battery_dev_read_write_fw_cfg_version(bool is_write)
{
	int ret = 0, flag = 0, old_csum = 0, csum = 0;
	int old_DV = 0, new_DV = 0;
	int temp = 0, count = 0;

	/* 1. UNSEAL it by sending the appropriate
	 * keys to Control( ) (0x00 and 0x01) */
	ret = bq27621_unseal();
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"UNSEAL device error %d.\n", ret);
		return ret;
	}

	/* 2. Send SET_CFGUPDATE subcommand, Control(0x0013) */
	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x13, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Send SET_CFGUPDATE subcommand error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Send SET_CFGUPDATE subcommand error %d.\n", ret);
		return ret;
	}

	/*
	 * 3. Confirm CONFIG UPDATE mode by polling Flags( )
	 *	register until bit 4 is set.
	 *	May take up to 1 second.
	 */
	count = 0;
	while (((flag & 0x10) >> 4) != 1) {
		ret = bq27520_read_i2c(sn700002_device->client, 0x06, &flag, 1);
		if (ret < 0) {
			dev_err(&sn700002_device->client->dev,
				"Confirm CONFIG UPDATE mode error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] flags = %x\n", __func__, flag);

		if (count > 6)
			return -301;

		count++;
		msleep(500);
	}

	/*
	 * 4. Write 0x00 using BlockDataControl( ) command (0x61)
	 * to enable block data memory control
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x61, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Enable block data memory control error %d.\n", ret);
		return ret;
	}

	/* 5. Write 0x40 using the DataClass( ) command (0x3E)
	 * to access the State subclass (64 decimal, 0x40 hex)
	 * containing the DF Version parameter.
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x3E, 0x40, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Access the State subclass error %d.\n", ret);
		return ret;
	}

	/*
	 * 6. Write the block offset location using DataBlock( ) command (0x3F).
	 *    Note: To access data located at offset 0 to 31 use offset = 0x00.
	 *    To access data located at offset 32 to 41 use offset = 0x01.
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x3F, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Write the block offset location error %d.\n", ret);
		return ret;
	}

	/* 7. Read the 1-byte checksum using
	 * the BlockDataChecksum( ) command (0x60). */
	ret = bq27520_read_i2c(sn700002_device->client, 0x60, &old_csum, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Read the 1-byte checksum error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_csum = %x\n", __func__, old_csum);

	/* 8. Read DF Version byte at 0x43 (offset = 3). */
	ret = bq27520_read_i2c(sn700002_device->client, 0x43, &old_DV, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Read DF Version error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_DV = %x\n", __func__, old_DV);

	if (is_write) {
		/* 9. Write DF Version byte at 0x43. */
		ret = bq27520_write_i2c(sn700002_device->client,
					0x43, LATEST_FW_CFG_VERSION, 1);
		if (ret < 0) {
			dev_err(&sn700002_device->client->dev,
					"Write DF Version error %d.\n", ret);
			return ret;
		}

		/* Check new DF Version */
		ret = bq27520_read_i2c(sn700002_device->client,
					0x43, &new_DV, 1);
		if (ret < 0) {
			dev_err(&sn700002_device->client->dev,
				"Check new DF Version error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] new_DV = %x\n", __func__, new_DV);

		/* 10. Compute the new block checksum. */
		temp = (255 - old_csum - old_DV) % 256;
		pr_info("%s: [M] temp = %x\n", __func__, temp);
		csum = (255 - ((temp + LATEST_FW_CFG_VERSION) % 256)) & 0xFF;
		pr_info("%s: [M] csum = %x\n", __func__, csum);

		/* 11. Write new checksum. */
		ret = bq27520_write_i2c(sn700002_device->client, 0x60, csum, 1);
		if (ret < 0) {
			dev_err(&sn700002_device->client->dev,
				"Write new checksum error %d.\n", ret);
			return ret;
		}
	}

	/*
	 * 12. Exit CONFIG UPDATE mode by sending SOFT_RESET subcommand,
	 *     Control(0x0042)
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x42, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Sending SOFT_RESET subcommand error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Sending SOFT_RESET subcommand error %d.\n", ret);
		return ret;
	}

	/*
	 * 13. Confirm CONFIG UPDATE has been exited by polling Flags( )
	 * register until bit 4 is cleared. May take up to 1 second.
	 */
	count = 0;
	while (((flag & 0x10) >> 4) != 0) {
		ret = bq27520_read_i2c(sn700002_device->client, 0x06, &flag, 1);
		if (ret < 0) {
			dev_err(&sn700002_device->client->dev,
				"Confirm CONFIG UPDATE mode error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] flags = %x\n", __func__, flag);

		if (count > 6)
			return -301;

		count++;
		msleep(500);
	}

	/* 14. Return to SEALED mode sending by the Control(0x0020) command. */
	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x20, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Return to SEALED mode error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Return to SEALED mode error %d.\n", ret);
		return ret;
	}

	return ret < 0 ? ret : old_DV;
}
#endif

static int bq27621_asus_battery_dev_read_gauge_id(void)
{
	int ret = 0;
	int gauge_id = 0;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x08, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Write control register error %d.\n", ret);
		return ret;
	}

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Write control register error %d.\n", ret);
		return ret;
	}

	msleep(50);

	ret = bq27520_read_i2c(sn700002_device->client, 0x00, &gauge_id, 0);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Read gauge id error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] gauge_id = %x\n", __func__, gauge_id);

	return ret < 0 ? ret : gauge_id;
}

static int bq27621_asus_battery_dev_reset(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x41, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Write control register to reset error %d.\n", ret);
		return ret;
	}

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Write control register to reset error %d.\n", ret);
		return ret;
	}

	return ret;
}

static int bq27621_asus_battery_dev_read_write_fw_cfg_version_lite(void)
{
	int ret = 0;
	int old_DV = 0;

	/* 1. UNSEAL it by sending the appropriate keys
	 * to Control() (0x00 and 0x01) */
	ret = bq27621_unseal();
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"UNSEAL device error %d.\n", ret);
		return ret;
	}

	/*
	 * 4. Write 0x00 using BlockDataControl( ) command (0x61)
	 * to enable block data memory control
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x61, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Enable block data memory control error %d.\n", ret);
		return ret;
	}

	/*
	 * 5. Write 0x40 using the DataClass( ) command (0x3E)
	 * to access the State subclass (64 decimal, 0x40 hex)
	 * containing the DF Version parameter.
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x3E, 0x40, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Access the State subclass error %d.\n", ret);
		return ret;
	}

	/*
	 * 6. Write the block offset location using DataBlock( ) command (0x3F).
	 *    Note: To access data located at offset 0 to 31 use offset = 0x00.
	 *    To access data located at offset 32 to 41 use offset = 0x01.
	 */
	ret = bq27520_write_i2c(sn700002_device->client, 0x3F, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Write the block offset location error %d.\n", ret);
		return ret;
	}

	msleep(50);

	/* 8. Read DF Version byte at 0x43 (offset = 3). */
	ret = bq27520_read_i2c(sn700002_device->client, 0x43, &old_DV, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
			"Read DF Version error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_DV = %x\n", __func__, old_DV);

	/* 14. Return to SEALED mode sending by the Control(0x0020) command. */
	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x20, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Return to SEALED mode error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(&sn700002_device->client->dev,
				"Return to SEALED mode error %d.\n", ret);
		return ret;
	}

	return ret < 0 ? ret : old_DV;
}

static int bq27520_rom_mode_wait(int m_secs)
{
	BAT_NOTICE("[%s] enter\n", __func__);
	if (m_secs < 1)
		return -EINVAL;

	msleep(m_secs);
	return 0;
}
#ifdef NOT_YET
static int device_type(void)
{
	int fw_cfg_ver = 0;
	int ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x01, 1);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Get fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Get fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}

	/* delay awhile to get version.
	 * Otherwise version data not transfer complete
	 */
	udelay(800);
	sn700002_smbus_read_data_reg(0x00, 0, &fw_cfg_ver);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Read fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}
	pr_debug("DEVICE_TYPE = 0x%04X\n", fw_cfg_ver);

	return fw_cfg_ver;
}

static int fw_type(void)
{
	int fw_cfg_ver = 0;
	int ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x02, 1);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Get fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Get fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}
	/* delay awhile to get version.
	 * Otherwise version data not transfer complete */
	udelay(800);
	sn700002_smbus_read_data_reg(0x00, 0, &fw_cfg_ver);
	if (ret) {
		dev_err(&sn700002_device->client->dev,
				"Read fw cfg version error %d.\n", ret);
		return ERROR_CODE_I2C_FAILURE;
	}
	pr_debug("FW_VERSION  = 0x%04X\n", fw_cfg_ver);

	return fw_cfg_ver;
}
#endif

static int update_fw(struct update_op *p_start, struct update_op *p_end)
{
	int ret = UPDATE_OK;

	BAT_NOTICE("(%s) Start update firmware ... ", __func__);


	ret = UPDATE_PROCESS_FAIL;
	while (p_start <= p_end) {
		int op_execute_ret = 0;
		switch (p_start->bq_op) {
		case OP_I2C_START:
		case OP_I2C_WRITE:
			op_execute_ret =
				bq27520_write_i2c(sn700002_device->client,
					p_start->off, p_start->arg, 1);
			if (op_execute_ret < 0) {
				BAT_NOTICE("   FAIL\n");
				return ret;
			}
		break;
		case OP_I2C_CMP:
			op_execute_ret =
				bq27520_cmp_i2c(p_start->off, p_start->arg);
			if (op_execute_ret != PROC_FALSE
				&& op_execute_ret != PROC_TRUE) {
				BAT_NOTICE("   FAIL %d\n", op_execute_ret);
				return ret;
			} else if (op_execute_ret == PROC_FALSE) {
				BAT_NOTICE("   FAIL\n");
				return ret;
			}
			break;

		case OP_WAIT:
			op_execute_ret = bq27520_rom_mode_wait(p_start->arg);
			if (op_execute_ret < 0) {
				BAT_NOTICE("   FAIL\n");
				return ret;
			}
			break;
		default:
			BAT_NOTICE("Not support OP\n");
			break;
		};
		p_start++;
	};

	ret = UPDATE_OK;
	BAT_NOTICE("%s Done.\n", __func__);
	return ret;
}

static int update_normal(int curr_cell_type)
{
	int ret;
	struct update_op *p_op_start = NULL, *p_op_end = NULL, *p_op_buf = NULL;
	int i, j;
	struct bq27xx_dffs_data *bq27xx_fw;
	unsigned int num_of_op_buf;

	bq27xx_fw = default_bq27xx_fw;
	num_of_op_buf =
		sizeof(default_bq27xx_fw) / sizeof(struct bq27xx_dffs_data);

	BAT_NOTICE(" fw flashing... please wait for about 10s at least.\n");
	ret = UPDATE_ERR_MATCH_OP_BUF;
	for (i = 0; i < num_of_op_buf; i++) {
		if (curr_cell_type != bq27xx_fw[i].cell_type)
			continue;
		BAT_DBG("%s: curr_cell_type=%d, cell_type=%d\n",
			__func__, curr_cell_type, bq27xx_fw[i].cell_type);
		p_op_buf = bq27xx_fw[i].op;
		p_op_start = p_op_buf;
		p_op_end = &p_op_buf[bq27xx_fw[i].num_op - 1];
		for (j = 0; j < RETRY_COUNT; j++) {
			ret = update_fw(p_op_start, p_op_end);
			if (ret == UPDATE_OK)
				break;
		}
		break;
	}

	return ret;
}


static int update_from_normal_mode(bool forceupdate)
{
	int curr_cell_type = TYPE_LG;
	int ret = UPDATE_NONE;
	int fw_cfg_version, fw_gauge_id = 0;
	int latest_fw_cfg_version = LATEST_FW_CFG_VERSION_LGC;
	int latest_fw_gauge_id = LATEST_GAUGE_ID_LGC;

	BAT_NOTICE("(%s) enter\n", __func__);
	msleep(3000);
	if (g_battery_id == 0) {
		latest_fw_cfg_version = LATEST_FW_CFG_VERSION_ATL;
		latest_fw_gauge_id = LATEST_GAUGE_ID_ATL;
		curr_cell_type = TYPE_ATL;
	} else if (g_battery_id == 1) {
		latest_fw_cfg_version = LATEST_FW_CFG_VERSION_LGC;
		latest_fw_gauge_id = LATEST_GAUGE_ID_LGC;
		curr_cell_type = TYPE_LG;
	} else {
		BAT_DBG_E("%s: Wrong battery ID: %d, use default settings\n",
				__func__, g_battery_id);
	}

	fw_cfg_version =
		bq27621_asus_battery_dev_read_write_fw_cfg_version_lite();
	if (fw_cfg_version < 0) {
		BAT_DBG_E("%s: read battery FW config version failed : %d\n",
				__func__, fw_cfg_version);
		goto Done;
	}

	fw_gauge_id = bq27621_asus_battery_dev_read_gauge_id();
	if (fw_gauge_id < 0) {
		BAT_DBG_E("%s: read battery FW gauge id failed : %d\n",
				__func__, fw_gauge_id);
		goto Done;
	}

	BAT_NOTICE(
		"(%s) current_cell_type %d, fw_cfg_version %d, ",
		__func__, curr_cell_type, fw_cfg_version);
	BAT_NOTICE(
		"latest_cfg_version %d, fw_gauge_id %x, latest_fw_gauge_id %x\n",
		latest_fw_cfg_version, fw_gauge_id, latest_fw_gauge_id);

	if (!forceupdate) {
		if (fw_cfg_version == latest_fw_cfg_version
				&& (fw_gauge_id == latest_fw_gauge_id)) {
			BAT_NOTICE("No need to flash battery cell data\n");
			BAT_NOTICE("data flash version and gauge id are equal");
			goto Done;
		}
	}

	ret = update_normal(curr_cell_type);
	if (ret < 0)
		bq27621_asus_battery_dev_reset();

Done:
	if (fw_cfg_version == latest_fw_cfg_version)
		g_fw_cfg_version = fw_cfg_version;
	else
		g_fw_cfg_version =
		bq27621_asus_battery_dev_read_write_fw_cfg_version_lite();

	if (fw_gauge_id == latest_fw_gauge_id)
		g_fw_gauge_id = fw_gauge_id;
	else
		g_fw_gauge_id = bq27621_asus_battery_dev_read_gauge_id();

	return ret;
}

static void main_force_update_flow(void)
{
	update_from_normal_mode(true);
}

static void main_update_flow(void)
{
	update_from_normal_mode(false);
}

static int bq27621_unseal(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x80, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x80, 1);
	if (ret < 0)
		return ret;

	return ret;
}

int bq27621_set_cfg_update(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn700002_device->client, 0x00, 0x03, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn700002_device->client, 0x01, 0x00, 1);
	if (ret < 0)
		return ret;

	return ret;
}

/* +++ Create /proc/driver/gaugeIC_FW +++ */
static int proc_sn700002_firmware_write(struct file *file, const char *buffer,
				size_t count, loff_t *data)
{
	asus_get_battery_id();
	BAT_DBG("%s: Battery ID = %d\n", __func__, g_battery_id);

	cancel_delayed_work(&sn700002_device->status_poll_work);
	main_force_update_flow();
	queue_delayed_work(battery_poll_work_queue,
			&sn700002_device->status_poll_work, 5 * HZ);

	return count;
}

static int proc_sn700002_firmware_read(struct seq_file *m, void *p)
{
	BAT_DBG("battery id = %d, version = %d, gauge id = %x\n",
			g_battery_id, g_fw_cfg_version, g_fw_gauge_id);

	if (g_battery_id == 0) {
		if ((g_fw_cfg_version == LATEST_FW_CFG_VERSION_ATL) &&
				(g_fw_gauge_id == LATEST_GAUGE_ID_ATL)) {
			seq_puts(m, "PASS\n");
			return 0;
		} else {
			seq_puts(m, "FAIL\n");
			return 0;
		}
	} else if (g_battery_id == 1) {
		if ((g_fw_cfg_version == LATEST_FW_CFG_VERSION_LGC)
				&& (g_fw_gauge_id == LATEST_GAUGE_ID_LGC)) {
			seq_puts(m, "PASS\n");
			return 0;
		} else {
			seq_puts(m, "FAIL\n");
			return 0;
		}
	}

	seq_printf(m, "FAIL, unknown battery id %d\n", g_battery_id);

	return 0;
}

static int proc_sn700002_firmware_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sn700002_firmware_read, NULL);
}

static const struct file_operations proc_sn700002_firmware_ops = {
	.open = proc_sn700002_firmware_open,
	.read = seq_read,
	.write = proc_sn700002_firmware_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_sn700002_firmware_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/gaugeIC_FW", 0664,
			NULL, &proc_sn700002_firmware_ops);
	if (!entry) {
		pr_err("[%s] Unable to create sn700002_firmware\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int sn700002_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int ret, i = 0, rt_value;
#ifdef CONFIG_OF
	struct device_node *np = client->dev.of_node, *child;
	int nb_bat = of_get_child_count(np);
	char *default_opbuff;
	const char *name;
	int length;
#endif

	BAT_NOTICE("(%s) client->addr= %02x\n", __func__, client->addr);

	sn700002_device = devm_kzalloc(&client->dev,
			sizeof(*sn700002_device), GFP_KERNEL);
	if (!sn700002_device) {
		BAT_ERR("Failed to allocate sn700002_device memory.\n");
		return -ENOMEM;
	}

	memset(sn700002_device, 0, sizeof(*sn700002_device));
	sn700002_device->client = client;
	i2c_set_clientdata(client, sn700002_device);

	sn700002_device->smbus_status    = 0;
	sn700002_device->cap_err         = 0;
	sn700002_device->temp_err        = 0;
	sn700002_device->old_capacity    = 50;
	sn700002_device->old_temperature = 250;
	sn700002_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;

	sn700002_device->smbus_status = sn700002_smbus_read_data_reg(
				sn700002_data[REG_CAPACITY].addr, 0, &rt_value);
	if (sn700002_device->smbus_status < 0) {
		dev_err(&sn700002_device->client->dev,
				"%s, fail to read gauge i2c\n", __func__);
		return -EINVAL;
	}

#ifdef CONFIG_OF
	/* prepare FW */
	if (!np)
		return -EINVAL;


	default_bq27xx_fw = devm_kzalloc(&client->dev,
			sizeof(struct bq27xx_dffs_data) * nb_bat,
			GFP_KERNEL);

	if (!default_bq27xx_fw)
		return -ENOMEM;

	BAT_NOTICE("%s: %d batteries\n", __func__, nb_bat);

	i = 0;
	for_each_child_of_node(np, child) {
		if (of_property_read_string(child,
					"intel,battery-type", &name)) {
			dev_err(&client->dev, "cannot get battery type\n");
			return -EINVAL;
		}

		if (strcmp(name, "LG") == 0)
			default_bq27xx_fw[i].cell_type = TYPE_LG;
		else if (strcmp(name, "COS_LIGHT") == 0)
			default_bq27xx_fw[i].cell_type = TYPE_COS_LIGHT;
		else if (strcmp(name, "ATL") == 0)
			default_bq27xx_fw[i].cell_type = TYPE_ATL;
		else {
			dev_err(&client->dev, "Unknown battery type: %s\n",
					name);
			return -EINVAL;
		}

		if (of_find_property(child, "intel,battery-fw",
					&length) == NULL)
			return -EINVAL;

		default_opbuff = devm_kzalloc(&client->dev,
				sizeof(char) * length, GFP_KERNEL);

		if (!default_opbuff)
			return -ENOMEM;

		if (of_property_read_u8_array(child, "intel,battery-fw",
					default_opbuff, length))
			return -EINVAL;

		default_bq27xx_fw[i].op =
			(struct update_op *)default_opbuff;
		default_bq27xx_fw[i].num_op =
			length / sizeof(struct update_op);

		BAT_NOTICE(" - %d: cell_type %d\n", i,
				default_bq27xx_fw[i].cell_type);
		BAT_NOTICE(" - %d: num_op %d\n", i,
				default_bq27xx_fw[i].num_op);
	}
#endif

	/* FW update */
	INIT_WORK(&sn700002_device->fw_work, sn700002_fw_work);
	schedule_work(&sn700002_device->fw_work);

	for (i = 0; i < ARRAY_SIZE(sn700002_supply); i++) {
		ret = power_supply_register(&client->dev, &sn700002_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply ,num = %d\n",
									i);
			do {
				power_supply_unregister(&sn700002_supply[i]);
			} while ((--i) >= 0);
			kfree(sn700002_device);
			return ret;
		}
	}

	battery_poll_work_queue =
		create_singlethread_workqueue("battery_workqueue");
	INIT_DELAYED_WORK(&sn700002_device->status_poll_work,
						battery_status_poll);
	INIT_DELAYED_WORK(&sn700002_device->low_low_bat_work,
						low_low_battery_check);
	cancel_delayed_work(&sn700002_device->status_poll_work);

	spin_lock_init(&sn700002_device->lock);
	wake_lock_init(&sn700002_device->low_battery_wake_lock,
				WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&sn700002_device->cable_event_wake_lock,
				WAKE_LOCK_SUSPEND, "battery_cable_event");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret)
		dev_err(&client->dev,
				"sn700002_probe: unable to create the sysfs\n");

	/* stress test */
	sn700002_stress_test_work_queue =
		create_singlethread_workqueue("i2c_battery_wq");
	if (!sn700002_stress_test_work_queue)
		pr_err("battery sn700002 : unable to create i2c stress test workqueue\n");
	INIT_DELAYED_WORK(&sn700002_stress_test_poll_work,
			sn700002_stress_test_poll);

	/* Misc device registration */
	sn700002_device->battery_misc.minor = MISC_DYNAMIC_MINOR;
	sn700002_device->battery_misc.name = "battery";
	sn700002_device->battery_misc.fops  = &sn700002_fops;
	ret = misc_register(&sn700002_device->battery_misc);
	if (ret)
		dev_err(&client->dev,
			"Cannot register sn700002 miscdev (err = %d)\n", ret);

#ifdef CONFIG_HAS_EARLYSUSPEND
	sn700002_device->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	sn700002_device->es.suspend = sn700002_early_suspend;
	sn700002_device->es.resume = sn700002_late_resume;
	register_early_suspend(&sn700002_device->es);
#endif

#if CONFIG_PROC_FS
	ret = sn700002_register_upilogger_proc_fs();
	if (ret)
		BAT_ERR("Unable to create register_upilogger_proc_fs\n");
#endif
	check_cabe_type();
	queue_delayed_work(battery_poll_work_queue,
			&sn700002_device->status_poll_work, 5 * HZ);

	ret = create_sn700002_firmware_proc_fs();
	if (ret)
		BAT_ERR("Unable to create sn700002_firmware_proc_fs\n");

	BAT_NOTICE("- %s driver registered done ====\n", client->name);

	return 0;
}

static int sn700002_remove(struct i2c_client *client)
{
	struct sn700002_device_info *sn700002_device;
	int i = 0;

	sn700002_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(sn700002_supply); i++)
		power_supply_unregister(&sn700002_supply[i]);

	if (sn700002_device) {
		wake_lock_destroy(&sn700002_device->low_battery_wake_lock);
		kfree(sn700002_device);
		sn700002_device = NULL;
	}
	return 0;
}

static const struct dev_pm_ops sn700002_pm_ops = {
	.suspend                = sn700002_suspend,
	.resume	                = sn700002_resume,
};

static const struct i2c_device_id sn700002_id[] = {
	{ "sn700002-battery", 0 },
	{},
};

static struct i2c_driver sn700002_battery_driver = {
	.driver = {
		.name	= "sn700002-battery",
		.owner	= THIS_MODULE,
		.pm	= &sn700002_pm_ops,
	},
	.probe		= sn700002_probe,
	.remove		= sn700002_remove,
	.id_table	= sn700002_id,
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CCD,
	},

	{ /* end: all zeroes */},
};

static int sn700002_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	int ret = 0;

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	return 0;
}

static int __exit sn700002_idi_remove(struct idi_peripheral_device *ididev)
{
	return 0;
}

static struct idi_peripheral_driver sn700002_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sn700002-idi",
		.pm = NULL,
	},
	.p_type = IDI_CCD,
	.id_table = idi_ids,
	.probe  = sn700002_idi_probe,
	.remove = sn700002_idi_remove,
};

static int __init sn700002_battery_init(void)
{
	int ret;
	ret = idi_register_peripheral_driver(&sn700002_idi_driver);
	if (ret) {
		pr_err("%s fail\n", __func__);
		return ret;
	}
	ret = i2c_add_driver(&sn700002_battery_driver);
	if (ret)
		dev_err(&sn700002_device->client->dev,
			"%s: i2c_add_driver failed\n", __func__);

	return ret;
}

module_init(sn700002_battery_init);

static void __exit sn700002_battery_exit(void)
{
	i2c_del_driver(&sn700002_battery_driver);
}
module_exit(sn700002_battery_exit);

MODULE_DESCRIPTION("sn700002 battery monitor driver");
MODULE_LICENSE("GPL");
