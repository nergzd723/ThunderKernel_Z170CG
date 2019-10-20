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
//#include <linux/earlysuspend.h>
#include <linux/poll.h>

#include "asus_battery.h"
#include "smb345_external_include.h"
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/switch.h>


/* Webber Jeita */
#include <linux/thermal.h>



DEFINE_MUTEX(sn200007_dev_info_mutex);
extern int max_current_set;

//#define	SN200007_CALLBACK_FUNC
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
#define BAT_NOTICE(format, arg...)	  printk(KERN_NOTICE "%s " format , __FUNCTION__ , ## arg)
#define BAT_ERR(format, arg...)		  printk(KERN_ERR format , ## arg)

/* Global variable */
unsigned battery_cable_status = 0;
unsigned battery_driver_ready = 0;
static int ac_on;
static int usb_on;
static unsigned int 	battery_current;
static unsigned int     battery_remaining_capacity;
struct workqueue_struct *battery_poll_work_queue = NULL;
struct workqueue_struct *sn200007_update_work_queue = NULL;       //[Carlisle]Modify gauge FW flow
extern void aicl_dete_worker(struct work_struct *dat);
static int g_battery_id = 1;
static int g_fw_cfg_version;
static int g_fw_gauge_id;
/* report battery version */
struct switch_dev batt_dev;  

/* Functions declaration */
//static int sn200007_get_psp(int reg_offset, enum power_supply_property psp,union power_supply_propval *val);
static int sn200007_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

// Webber ++++++++++++++++++++++
static int get_thermal_zone_temperature(char*);
#define GETTHERMALZONENULL 	-9999
// Webber ----------------------
int bq27621_is_config_update_mode(void);
int bq27621_unseal(void);
void main_force_update_flow(void);
void main_update_flow(void);

//[Carlisle] Add for soft-reset gauge +++
static int bq27621_soft_reset(void);
//[Carlisle] Add for soft-reset gauge ---

//extern unsigned  get_USB_PC_status(void);

module_param(battery_current, uint, 0644);
module_param(battery_remaining_capacity, uint, 0644);

#ifdef ASUS_ENG_BUILD
bool eng_charging_limit = true;
#endif

/* usb state ref charger driver*/
/* WB +++++++++++++++++++*/
extern int G_USB_TYPE;
/* WB -------------------*/

//[Carlisle] Add for COS fw update usage +++
extern int entry_mode;
int g_fw_success = 0;
struct wake_lock gauge_wlock;
//[Carlisle] Add for COS fw update usage ---

// [WB] ++
bool is_first = true;
// PSOC : END USER READ State Of Charge
static int PSOC;
static u32 polling_time;
extern struct wake_lock wlock_ac;
extern bool ac_wakelock_flag;
static bool FC = false;
#define FC_FLAG_FILE_PATH "/cache/full_charge_flag"
#define PSOC_VALUE_FILE_PATH "/cache/psoc_value"
static int BT = 0;
static int Charge_Full = 3450;
// [WB] --


//+++ i2c stress test
#define sn200007_IOC_MAGIC 0xF9
#define sn200007_IOC_MAXNR 5
#define sn200007_POLL_DATA _IOR(sn200007_IOC_MAGIC, sn200007_IOC_MAXNR,int )

#define sn200007_IOCTL_START_HEAVY 2
#define sn200007_IOCTL_START_NORMAL 1
#define sn200007_IOCTL_END 0

#define START_NORMAL    10 * (HZ)
#define START_HEAVY     (HZ)
static int stress_test_poll_mode = 0;
struct delayed_work sn200007_stress_test_poll_work;
static struct workqueue_struct *sn200007_stress_test_work_queue;
static wait_queue_head_t poll_wait_queue_head_t;
static bool flag_pollin = true;
//---

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

#define sn200007_DATA(_psp, _addr, _min_value, _max_value)	\
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

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static struct sn200007_device_data {
	enum power_supply_property psp;
	u8 addr;
	int min_value;
	int max_value;
} sn200007_data[] = {
        [REG_MANUFACTURER_DATA]	                = sn200007_DATA (PRESENT           ,    0,      0, 65535),
        [REG_STATE_OF_HEALTH]		        = sn200007_DATA (HEALTH            ,    0,      0, 65535),
	[REG_TEMPERATURE]			= sn200007_DATA (TEMP              , 0x02,      0, 65535),
	[REG_VOLTAGE]				= sn200007_DATA (VOLTAGE_NOW       , 0x04,      0,  6000),
	[REG_CURRENT]				= sn200007_DATA (CURRENT_NOW       , 0x10, -32768, 32767),
	[REG_CAPACITY]				= sn200007_DATA (CAPACITY          , 0x1c,      0,   100),
};

static enum power_supply_property sn200007_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_TEMP_AMBIENT,
	POWER_SUPPLY_PROP_CHARGE_FULL,
};

static bool is_charging_full(void) {
	int charger_status;

	charger_status = smb345_get_charging_status();
	if (charger_status == POWER_SUPPLY_STATUS_FULL) {
		BAT_NOTICE("[%s] charging full\n", __func__);
		return (true);
	} else {
		return (false);
	}
}

static bool is_charging(void) {
	int charger_status;

	charger_status = smb345_get_charging_status();

	if ((charger_status == POWER_SUPPLY_STATUS_FULL) ||
		(charger_status == POWER_SUPPLY_STATUS_CHARGING)) {
		return (true);
	} else {
		return (false);
	}
}

void check_cabe_type(void) {
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

static enum power_supply_property power_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy, enum power_supply_property psp, union power_supply_propval *val) {
	int ret=0;
	switch (psp) {
	    case POWER_SUPPLY_PROP_ONLINE:
		   if(psy->type == POWER_SUPPLY_TYPE_MAINS &&  ac_on)
			val->intval =  1;
		   else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
			val->intval =  1;
		   else
			val->intval = 0;
		break;
            case POWER_SUPPLY_PROP_PRESENT:
                if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
                    /* for ATD test to acquire the status about charger ic */
                    if (!smb345_has_charger_error() || smb345_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)
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

static char *supply_list[] = {
	"battery",
#ifdef SN200007_CALLBACK_FUNC
	"ac",
	"usb",
#endif
};

static void sn200007_battery_external_power_changed(struct power_supply *psy) {

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

static struct power_supply sn200007_supply[] = {
	{
		.name		= "battery",
		.type		= POWER_SUPPLY_TYPE_BATTERY,
		.properties	= sn200007_properties,
		.num_properties = ARRAY_SIZE(sn200007_properties),
		.get_property	= sn200007_get_property,
#ifndef	SN200007_CALLBACK_FUNC
		.external_power_changed	= sn200007_battery_external_power_changed,
#endif	///< end of SN200007_CALLBACK_FUNC
       },
#ifdef SN200007_CALLBACK_FUNC
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
		.properties =power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
};

static struct sn200007_device_info {
	struct i2c_client *client;
	struct delayed_work battery_stress_test;
	struct delayed_work status_poll_work;
	struct delayed_work low_low_bat_work;
	struct delayed_work update_gauge_work;       //[Carlisle] Modify gauge FW flow
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
} *sn200007_device;

static int sn200007_read_i2c(u8 reg, int *rt_value, int b_single) {
	struct i2c_client *client = sn200007_device->client;
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
           dev_err(&client->dev, "I2C read from addr 0x%02X error with code: %d\n", reg, err);
        }

	return err;
}

int sn200007_smbus_read_data(int reg_offset,int byte,int *rt_value) {
	s32 ret = -EINVAL;
	int count = 0;

        mutex_lock(&sn200007_dev_info_mutex);
	do {
		ret = sn200007_read_i2c(sn200007_data[reg_offset].addr, rt_value, 0);
	} while( (ret < 0) && ( ++count <= SMBUS_RETRY) );
        mutex_unlock(&sn200007_dev_info_mutex);

	return ret;
}

int sn200007_smbus_read_data_reg(u8 reg,int byte,int *rt_value) {
	s32 ret = -EINVAL;
	int count = 0;

        mutex_lock(&sn200007_dev_info_mutex);
	do {
		ret = sn200007_read_i2c (reg, rt_value, 0);
	} while ( (ret < 0) && (++count <= SMBUS_RETRY) );
        mutex_unlock(&sn200007_dev_info_mutex);

	return ret;
}

int sn200007_smbus_write_data(int reg_offset, int byte, unsigned int value) {
        s32 ret = -EINVAL;
        int count=0;

	do {
            if (byte) {
               ret = i2c_smbus_write_byte_data(sn200007_device->client,sn200007_data[reg_offset].addr, value & 0xFF);
	    } else{
	       ret = i2c_smbus_write_word_data(sn200007_device->client,sn200007_data[reg_offset].addr, value & 0xFFFF);
            }
	} while ( (ret < 0) && (++count <= SMBUS_RETRY) );
	return ret;
}

static ssize_t show_battery_smbus_status(struct device *dev, struct device_attribute *devattr, char *buf) {
	int status = !sn200007_device->smbus_status;
	return sprintf(buf, "%d\n", status);
}

static DEVICE_ATTR(battery_smbus, S_IWUSR | S_IRUGO, show_battery_smbus_status, NULL);

static struct attribute *battery_smbus_attributes[] = {
	&dev_attr_battery_smbus.attr,
	NULL
};

static const struct attribute_group battery_smbus_group = {
	.attrs = battery_smbus_attributes,
};

static int sn200007_get_capacity(void) {
	s32 temp_capacity;
	int remainingcapacity = 0;
	int retry_count = SMBUS_RETRY;

	do {
		sn200007_device->smbus_status = sn200007_smbus_read_data(REG_CAPACITY, 0 ,&sn200007_device->bat_capacity);
		if (sn200007_device->smbus_status < 0) {
			dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed sn200007_device->cap_err=%u\n", __func__, REG_CAPACITY, sn200007_device->cap_err);
			if(sn200007_device->cap_err> 5 || (sn200007_device->old_capacity == 0xFF)) {
				return -EINVAL;
			} else {
				sn200007_device->cap_err++;
				BAT_NOTICE("cap_err=%u use old capacity=%u\n", sn200007_device->cap_err, sn200007_device->old_capacity);
				return 0;
			}
		}

		retry_count--;
		if (sn200007_device->bat_capacity > 0)
			break;
		else {
			BAT_DBG_E("%s: Battery capacity = %d, retry %d times\n", __func__, sn200007_device->bat_capacity, retry_count);
			msleep(50);
		}
	} while (retry_count > 0);

	temp_capacity = ((sn200007_device->bat_capacity >= 100) ? 100 : sn200007_device->bat_capacity);
	/* start: for mapping %99 to 100%. Lose 84%*/
	if(temp_capacity==99)
		temp_capacity=100;
	if(temp_capacity >=84 && temp_capacity <=98)
		temp_capacity++;
	/* for mapping %99 to 100% */

	 /* lose 26% 47% 58%,69%,79% */
	if(temp_capacity >70 && temp_capacity <80)
		temp_capacity-=1;
	else if(temp_capacity >60&& temp_capacity <=70)
		temp_capacity-=2;
	else if(temp_capacity >50&& temp_capacity <=60)
		temp_capacity-=3;
	else if(temp_capacity >30&& temp_capacity <=50)
		temp_capacity-=4;
	else if(temp_capacity >=0&& temp_capacity <=30)
		temp_capacity-=5;

	/*Re-check capacity to avoid  that temp_capacity <0*/
	temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

	sn200007_device->old_capacity = temp_capacity;
	sn200007_device->cap_err=0;

	BAT_NOTICE("user capacity:%u%% ,gauge capacity: %u%%\n", sn200007_device->old_capacity, sn200007_device->bat_capacity);

        // for PSChange App tool
        sn200007_device->smbus_status = sn200007_smbus_read_data_reg(0x0C, 0 ,&remainingcapacity);
        if (sn200007_device->smbus_status >= 0) {
            battery_remaining_capacity = remainingcapacity;
	    BAT_NOTICE("remaining capacity= %d\n", remainingcapacity);
        }
	return 0;
}

static int sn200007_get_current(void) {
        int rt_value = 0;

        sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_CURRENT].addr, 0, &rt_value);
        if (sn200007_device->smbus_status < 0) {
		dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_CURRENT].addr);
	}
        if (rt_value & BIT15)  rt_value |= 0xFFFF0000; //neg. sign extend.

        rt_value += 0x10000;

        if (rt_value >= 0) rt_value -= 0x10000;
	sn200007_device->bat_current = rt_value;
        return rt_value;
}

// Webber Jeita +++++++++++++++++++++++++++++++++++++++
static int get_thermal_zone_temperature(char *thermal_zone_type){
	static struct thermal_zone_device *tzd;
	unsigned long thermal_temp;
	
	tzd = thermal_zone_get_zone_by_typename(thermal_zone_type);
	if (!tzd){
	  printk("[%s][WB] Fail to get thermal zone , zone type : %s",__func__,thermal_zone_type);
          return GETTHERMALZONENULL;
	}
	
	if(!tzd->ops->get_temp){ // thermal zone get_temp fun == null
		printk("[%s][WB] thermal zone get_temp fun == null \n",__func__);	
	}else{  // thermal zone get_temp fun != null
		tzd->ops->get_temp(tzd,&thermal_temp);
	}
	return thermal_temp;
}
// Webber Jeita ------------------------


static int sn200007_get_temperature(void) {
        int rt_value = 0;
		char thermal_type[20];

		/* Gauge battery temp*/
        sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_TEMPERATURE].addr, 0, &rt_value);
        if (sn200007_device->smbus_status < 0) {
		dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_TEMPERATURE].addr);
		}
		
		/* thermal_temp */
		sprintf(thermal_type, "%s", "battemp");
		sn200007_device->old_temperature = (get_thermal_zone_temperature(thermal_type)/100);
		if(sn200007_device->old_temperature == GETTHERMALZONENULL){
			printk("[%s] Fail to read the thermal zone. please check get_thermal_zone_temperature function , Erro Code : %d",__func__,sn200007_device->old_temperature);
		}

		if(sn200007_device->old_temperature < -300){ //Workaround to prevent thermal driver report a large negative value.
			sn200007_device->bat_temp = rt_value;
			sn200007_device->old_temperature = sn200007_device->bat_temp - TEMP_KELVIN_TO_CELCIUS;
			printk("[%s][WB] Gauge Temp = %d \n",__func__,(sn200007_device->bat_temp - TEMP_KELVIN_TO_CELCIUS));
			return (sn200007_device->old_temperature);
		}

		/* Gauge battery temp*/
        sn200007_device->bat_temp = rt_value;
		printk("[%s][WB] Theraml Zone - %s , temp = %d \n",__func__,thermal_type,sn200007_device->old_temperature);
		printk("[%s][WB] Gauge Temp = %d \n",__func__,(sn200007_device->bat_temp - TEMP_KELVIN_TO_CELCIUS));
        return (sn200007_device->old_temperature);
}

/* [WB] add for battery capacity opti policy +++++++++ */
struct file* file_open(const char* path,int flags,int rights){

	struct file* filp = NULL;
	mm_segment_t oldfs;
	int err = 0;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	filp = filp_open(path, flags, rights);
	set_fs(oldfs);
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		printk("[%s][WB] open file fail error code : %d\n",__func__,err);
		return NULL;
	}
	return filp;
}


int file_read(struct file* file, unsigned long long offset,char* data, unsigned int size){
	mm_segment_t oldfs;
	int ret;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	ret = vfs_read(file, data, size, &offset);
	printk("[%s][WB] FILE read  ret : %d\n",__func__,ret);

	set_fs(oldfs);
	return ret;
}

int file_write(struct file* file, unsigned long long offset,char* data, unsigned int size) {
	mm_segment_t oldfs;
	int ret;
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	
	ret = vfs_write(file, data, size, &offset);
	printk("[%s][WB] FILE write  ret : %d\n",__func__,ret);

	set_fs(oldfs);
	return ret;
}

void set_FC_id_to_File(int FC){
	struct file *fp = NULL;
	unsigned long long offset = 0;
	char* buf[2] = {0};
	int ret ;

	printk("[%s][WB] *********  start  *************\n",__func__);

	fp = file_open(FC_FLAG_FILE_PATH, O_CREAT | O_RDWR , 0);
	if(fp == NULL){
		BAT_DBG_E("[%s][WB] open file %s failed, ret = %d \n", __func__,FC_FLAG_FILE_PATH, IS_ERR(fp));
	}else{
		if(FC == 0){
			strcpy(buf,"0");
			printk("[%s][WB] set buf to %c\n",__func__,buf[0]);
		}else{
			strcpy(buf,"1");
			printk("[%s][WB] set buf to %c\n",__func__,buf[0]);
		}
		ret = file_write(fp,offset,buf,1);
		if(ret < 0){
			printk("[%s][WB] ret :  %d \n",__func__,ret);
		}else{
			ret = file_read(fp,offset,buf,1);
			if(ret < 0){
				printk("[%s][WB] ret = %d\n",__func__,ret);
			}else{
				printk("[%s][WB] buf = %c\n",__func__,buf[0]);
			}
		}
		filp_close(fp, NULL);
	}

	printk("[%s][WB] *********  end  *************\n",__func__);
}

void set_value_to_File(char* PATH,int value){
	struct file *fp = NULL;
	unsigned long long offset = 0;
	char* buf[4] = {0};
	int ret ;

	printk("[%s][WB] *********  start  *************\n",__func__);

	printk("[%s][WB] PATH = %s \n",__func__,PATH);
	fp = file_open(PATH, O_CREAT | O_RDWR , 0);

	if(fp == NULL){
		BAT_DBG_E("[%s][WB] open file %s failed, ret = %d \n", __func__,PATH, IS_ERR(fp));
	}else{
		sprintf(buf,"%d",value);
		printk("[%s][WB] buf = %s\n",__func__,buf);
		ret = file_write(fp,offset,buf,4);
		if(ret < 0){
			printk("[%s][WB] ret :  %d \n",__func__,ret);
		}else{
			ret = file_read(fp,offset,buf,4);
			if(ret < 0){
				printk("[%s][WB] ret = %d\n",__func__,ret);
			}else{
				printk("[%s][WB] buf = %s\n",__func__,buf);
			}
		}
		filp_close(fp, NULL);
	}

	printk("[%s][WB] *********  end  *************\n",__func__);
}

int read_value_from_File(char* PATH){

	struct file *fp = NULL;
	unsigned long long offset = 0;
	char buf[4] = {0};
	int ret ;
	long value;

	printk("[%s][WB] *********  start  *************\n",__func__);

	printk("[%s][WB] PATH = %s \n",__func__,PATH);
	fp = file_open(PATH, O_CREAT | O_RDWR , 0);

	if(fp == NULL){
		BAT_DBG_E("[%s][WB] open file %s failed, ret = %d \n", __func__,PATH, IS_ERR(fp));
	}else{
		 ret = file_read(fp,offset,buf,4);
		 if(ret < 0){
			printk("[%s][WB] ret = %d\n",__func__,ret);
			filp_close(fp, NULL);
			return ret;
		}else{
			printk("[%s][WB] buf = %s\n",__func__,buf);
			if(buf[0] == NULL){
				printk("[%s][WB] buf[0] = %c , return -1 \n",__func__,buf[0]);
				return -1;
			}else{
				printk("[%s][WB] buf[0] = %c , else case\n",__func__,buf[0]);
			}
			value = simple_strtol(buf,NULL,10);	
			printk("[%s][WB] value = %d\n",__func__,value);
			filp_close(fp, NULL);
		}
	}

	return value;
}


void get_FC_id(void){

	struct file *fp = NULL;
	unsigned long long offset = 0;
	char* buf[2] = {0};
        int ret ;	

	printk("[%s][WB] start get_FC_id \n",__func__);
	fp = file_open(FC_FLAG_FILE_PATH, O_CREAT | O_RDWR , 0);
	if(fp == NULL){
		BAT_DBG_E("[%s][WB] open file %s failed, ret = %d \n", __func__,FC_FLAG_FILE_PATH, IS_ERR(fp));
	}else{	
		ret = file_read(fp,offset,buf,1);	
		if(ret < 0){
			printk("[%s][WB] file read file ret = %d\n",__func__,ret);
		}else{
			printk("[%s][WB] buf = %c\n",__func__,buf[0]);
			if(buf[0] == '0'){
				FC = false;
				printk("[%s][WB] buf content = %c , set FC = %d\n",__func__,buf[0],FC);
			}else if(buf[0] == '1'){
				FC = true;
				printk("[%s][WB] buf content = %c , set FC = %d\n",__func__,buf[0],FC);
			}else{
				printk("[%s][WB] other condition buf content = %c \n",buf[0]);
				set_FC_id_to_File(0);
			}
		}
		filp_close(fp, NULL);
	}

	printk("[%s][WB] end get_FC_id \n",__func__);
}
/* [WB] add for battery capacity opti policy   ------------------ */


/* [WB]  for battery capacity optimization policy
 *  
 *    1. check (voltage < 3p45 V) & (USOC <= 1 % ) to setup system offffffff  
 *    2. avoid capacity rise problem (in goes to sleep for some time to wakeup )(No AC/USB plug in)
 *    3. avoid PSOC & USOC too different in suspend charging.( ex: PSOC : 6x % , USOC : 100%)
 * v1.1
 *    4. avoid discharging quickly from 100% to 95 % (PSOC value)
 *
 * v1.2
 *    1. add /factory/full_charge_flag file to save FC data
 *     for fix :
 *         	a. "can not be charged to 100% in S0 State" Problem.
 * v1.3
 *    1. check (voltage < 3p40 V) & (USOC <= 1 % ) to setup system offffffff
 *    2. recheck (voltage < 3p40v) 4 times
 *    3. modify get_Fd_id function to avoid NULL Point Problem. 
 * v1.4
 *    1. create new polling time policy
 *    	for S0 State:
 *    	a. 
 *    		15 degree <=T<=45 degree  -> 60s
 *    		T < 15 , T > 45  -> 30s
 *      b.     capacity 
 *      	 >= 50% -> 60s
 *      	 20%~49% -> 30s
 *      	 5% ~ 19% -> 30s
 *      	 0% ~ 4% -> 30s
 *      for S3 State:
 *      a.
 *      	without AC/USB -> 10min
 *      	with AC/USB -> 60s
 *
 *      	ChargingOS ->    60s
 *  V1.5 
 *
 *
 *    PSOC : end user read capacity (report to user side)
 *    USOC : mapping capacity (driver use)
 *    RSOC : capacity from gauge ic register 
 */
static void batt_capacity_opti(void){
	int rt_value;
	int count = 0;
	
	get_FC_id();

    /* first setup , PSOC = USOC  */
    if(is_first == true){
	if(sn200007_device->old_capacity<=1){
		PSOC = 1;
	}else{
		if((PSOC = read_value_from_File(PSOC_VALUE_FILE_PATH)) < 0){
			printk("[%s][WB] read data from file fail , steup PSOC = USOC \n",__func__);
			PSOC = sn200007_device->old_capacity;	
		}
		if(FC == true ){
			PSOC = 100;
			printk("[%s][WB] USOC > 1 & FC = true , PSOC set up to 100\n",__func__);
		}else{
			printk("[%s][WB] USOC > 1 & FC = false , do nothning \n",__func__);
			if(PSOC > sn200007_device->old_capacity){
				printk("[%s][WB] USOC > 1 & FC = false & PSOC > USOC ,do PSOC = USOC \n",__func__);
				PSOC = sn200007_device->old_capacity;
			}else{
				printk("[%s][WB] USOC > 1 & FC = false & PSOC <= USOC ,do nothning \n",__func__);
			}
		}
	}	
        is_first = false;    
        // Debug
        printk("[%s][WB] is first setup , PSOC = %d , USOC = %d \n",__func__,PSOC,sn200007_device->old_capacity);
     }else{
		/*Do nothing */
		// Debug +++++
        	printk("[%s][WB] more than first times, PSOC = %d , USOC = %d,do nothing...\n",__func__,PSOC,sn200007_device->old_capacity);
     }

     if( ( (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ) && (sn200007_device->old_capacity == 100) ){ // CABLE IN & USOC = 100        
		FC = true;
		set_FC_id_to_File(1);
		printk("[%s][WB] (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ) && (sn200007_device->old_capacity == 100), fc = TRUE\n",__func__);
		
		if(sn200007_device->bat_current > 0){
				// Debug 
                	printk("[%s][WB] CABLE IN : %d & current > 0 : %d \n",__func__,G_USB_TYPE,sn200007_device->bat_current);               
                	polling_time = (90*HZ);
                	// Debug 
                	printk("[%s][WB] polling_time = %d \n",__func__,polling_time);
                	if(PSOC < sn200007_device->old_capacity){
				// Debug +++
				printk("[%s][WB] PSOC < USOC \n",__func__);
				if(PSOC >= 100){
					// Debug
					printk("[%s][WB] PSOC >= 100, PSOC setup to 100 \n",__func__);
					PSOC = 100;
				}else{
					PSOC+=1;
					printk("[%s][WB] PSOC < 100 , PSOC +=1 , PSOC : %d \n",__func__,PSOC);
				}
			}else{
				/* do nothing */
				// Debug 
				printk("[%s][WB] PSOC >= USOC , do nothing !! \n",__func__);
			}

		}else{ // CABLE OUT
			// Debug 
			printk("[%s][WB] CABLE OUT \n",__func__);
	
			if(FC == false){	
				printk("[%s][WB] FC == false \n",__func__);
				if(sn200007_device->bat_vol < 3400){
                        		// Debug
                        		printk("[%s][WB] voltage < 3400 \n",__func__);
                        		if(PSOC <=1){
										do{
										msleep(2000);
										sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr, 0, &rt_value);
										if (sn200007_device->smbus_status < 0) { 
											dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_VOLTAGE].addr);
											}
										sn200007_device->bat_vol = rt_value;
										if(sn200007_device->bat_vol < 3400 ){
											count++;
											printk("[%s][WB] GAUGE IC Voltage = %d , %d times \n",__func__,sn200007_device->bat_vol,count);
										}else{
											printk("[%s][WB] GAUGE IC Voltage = %d , need to recheck\n",__func__,sn200007_device->bat_vol);
											goto Done;
										}
									}while(count < 3);
                               			/* system off */
                                		PSOC = 0;
                                		// Debug
                                		printk("[%s][WB] PSOC <=1 , setup USOC = 0 , USOC = %d , PSOC = %d \n",__func__,sn200007_device->old_capacity,PSOC);
                        		}else{
                                		PSOC -= 1;
                                		// Debug
                                		printk("[%s][WB] PSOC > 1 PSOC-=1 , PSOC : %d \n",__func__,PSOC);
                        		}
				}else{
                        		// Debug ++++
                       	 		printk("[%s][WB] sn200007_device->bat_vol >= 3400\n",__func__);

                        		if(PSOC > sn200007_device->old_capacity){
                                		if(PSOC<=1){
                                        		PSOC = 1;
							BT = 0;
                                        		// Debug ++++
                                        		printk("[%s][WB] PSOC<=1 , PSOC=1 , set BT = %d \n",__func__,BT);
                                		}else{
                                        		PSOC -=1;
							BT =0;
                                        		// Debug ++++
                                        		printk("[%s][WB] PSOC>1 , PSOC-=1, PSOC : %d , set BT = %d\n",__func__,PSOC,BT);
                                		}
					}else{
                                		/* do nothing */
                                		printk("[%s][WB] sn200007_device->bat_vol >= 3400 & PSOC <= USOC \n",__func__);
						if(BT==0){
							BT = sn200007_device->old_capacity;
							printk("[%s][WB] BT = USOC , BT = %d\n",__func__,BT);
						}else{
							if(sn200007_device->old_capacity > BT){
								BT = sn200007_device->old_capacity;
								printk("[%s][WB] USOC > BT , set BT = USOC \n",__func__);
							}else{
								printk("[%s][WB] USOC < BT ,  \n",__func__);
								printk("[%s][WB] BT-USOC = %d : %u\n",__func__,BT-sn200007_device->old_capacity,BT-sn200007_device->old_capacity);
								if((BT-(int)sn200007_device->old_capacity)>2){
									printk("[%s][WB] BT-sn200007_device->old_capacity > 2 \n",__func__);
									if(PSOC<=1){
										PSOC = 1;
										BT = 0;
										printk("[%s][WB] PSOC <=1 , set PSOC = 1 & BT = 0\n",__func__);
									}else{
										PSOC-=1;
										BT=0;
										printk("[%s][WB] PSOC>1 , set PSOC-=1 %d, BT=0\n",__func__,PSOC);
									}
								}else{
									printk("[%s][WB] BT-USOC <=2 \n",__func__);
								}
							}
						}
						
                        		}
				}
			}else{

				printk("[%s][WB] FC == true \n",__func__);
				if(sn200007_device->old_capacity <=94){
					FC = false;
					set_FC_id_to_File(0);
					printk("[%s][WB] USOC <= 94%% FC Flag setup to false \n",__func__);
				}else{
					if(is_charging() == true){
						if(PSOC >= 100){
							PSOC = 100;
							printk("[%s][WB] PSOC >=100  & is_charging !! , setup to 100%%\n",__func__);
						}else{
							PSOC +=1;
							printk("[%s][WB] PSOC < 100 & > 94 & is_charging !! , do PSOC += 1 , PSOC = %d \n",__func__,PSOC);
						}
					}else{
						FC = false;
						set_FC_id_to_File(0);
						printk("[%s][WB] PSOC  > 94  & not charging !! , set FC = false \n",__func__,FC);
					}
				}	
			}
		}
		
	}else if(( (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ) && (sn200007_device->old_capacity < 100)){ // CABLE IN & USOC < 100
		// Debug 
		printk("[%s][WB] CABLE IN , capacity %d \n",__func__, sn200007_device->old_capacity);
		
		if(sn200007_device->bat_current > 0){
			// Debug 
                	printk("[%s][WB] CABLE IN : %d & current > 0 : %d \n",__func__,G_USB_TYPE,sn200007_device->bat_current);               
                	polling_time = (90*HZ);
                	// Debug 
                	printk("[%s][WB] polling_time = %d \n",__func__,polling_time);
                	if(PSOC < sn200007_device->old_capacity){
					// Debug +++
					printk("[%s][WB] PSOC < USOC \n",__func__);
					if(PSOC >= 100){
						// Debug
						printk("[%s][WB] PSOC >= 100, PSOC setup to 100 \n",__func__);
						PSOC = 100;
					}else{
						PSOC+=1;
						printk("[%s][WB] PSOC < 100 , PSOC +=1 , PSOC : %d \n",__func__,PSOC);
					}
			}else{
				/* do nothing */
				// Debug 
				printk("[%s][WB] PSOC >= USOC , do nothing !! \n",__func__);
			}

        	}else{ // CABLE OUT
        		// Debug 
        		printk("[%s][WB] CABLE OUT \n",__func__);
	
			if(FC == false){	
				printk("[%s][WB] FC == false \n",__func__);
				if(sn200007_device->bat_vol < 3400){
                        		// Debug
                        		printk("[%s][WB] voltage < 3400 \n",__func__);
                        		if(PSOC <=1){
										do{
										msleep(2000);
										sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr, 0, &rt_value);
										if (sn200007_device->smbus_status < 0) { 
											dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_VOLTAGE].addr);
											}
										sn200007_device->bat_vol = rt_value;
										if(sn200007_device->bat_vol < 3400 ){
											count++;
											printk("[%s][WB] GAUGE IC Voltage = %d , %d times \n",__func__,sn200007_device->bat_vol,count);
										}else{
											printk("[%s][WB] GAUGE IC Voltage = %d , need to recheck\n",__func__,sn200007_device->bat_vol);
											goto Done;
										}
									}while(count < 3);
									
									
                                		/* system off */
                                		PSOC = 0;
                                		// Debug
                                		printk("[%s][WB] PSOC <=1 , setup USOC = 0 , USOC = %d , PSOC = %d \n",__func__,sn200007_device->old_capacity,PSOC);
                        		}else{
                                		PSOC -= 1;
                                		// Debug
                                		printk("[%s][WB] PSOC > 1 PSOC-=1 , PSOC : %d \n",__func__,PSOC);
                        		}
             			}else{
                        		// Debug ++++
                       	 		printk("[%s][WB] sn200007_device->bat_vol >= 3400\n",__func__);

                        		if(PSOC > sn200007_device->old_capacity){
                                		if(PSOC<=1){
                                        		PSOC = 1;
                                        		// Debug ++++
							BT = 0;
                                        		printk("[%s][WB] PSOC<=1 , PSOC=1 , set BT = %d\n",__func__,BT);
                                		}else{
                                        		PSOC -=1;
							BT = 0;
                                        		// Debug ++++
                                        		printk("[%s][WB] PSOC>1 , PSOC-=1, PSOC : %d, set BT = %d \n",__func__,PSOC,BT);
                                		}
					}else{
                                		/* do nothing */
                                		printk("[%s][WB] sn200007_device->bat_vol >= 3400 & PSOC <= USOC \n",__func__);
						if(BT==0){
							BT = sn200007_device->old_capacity;
							printk("[%s][WB] BT = USOC , BT = %d\n",__func__,BT);
						}else{

							if(sn200007_device->old_capacity > BT){
								BT = sn200007_device->old_capacity;
								printk("[%s][WB] USOC > BT , set BT = USOC \n",__func__);
							}else{
								printk("[%s][WB] USOC < BT ,  \n",__func__);
								printk("[%s][WB] BT-USOC = %d , %u \n",__func__,BT-sn200007_device->old_capacity,BT-sn200007_device->old_capacity);
								if((BT-(int)sn200007_device->old_capacity)>2){
									printk("[%s][WB] BT-sn200007_device->old_capacity > 2 \n",__func__);
									if(PSOC<=1){
										PSOC = 1;
										BT = 0;
										printk("[%s][WB] PSOC <=1 , set PSOC = 1 & BT = 0\n",__func__);
									}else{
										PSOC-=1;
										BT=0;
										printk("[%s][WB] PSOC>1 , set PSOC-=1 %d, BT=0\n",__func__,PSOC);
									}
								}else{
									printk("[%s][WB] BT-USOC <=2 \n",__func__);
								}
							}
						}
						
                        		}
               			}
			}else{

				printk("[%s][WB] FC == true \n",__func__);
				if(sn200007_device->old_capacity <=94){
					FC = false;
					set_FC_id_to_File(0);
					printk("[%s][WB] USOC <= 94%% FC Flag setup to false \n",__func__);
				}else{
					if(is_charging() == true){
						if(PSOC >= 100){
							PSOC = 100;
							printk("[%s][WB] PSOC >=100  & is_charging !! , setup to 100%%\n",__func__);
						}else{
							PSOC +=1;
							printk("[%s][WB] PSOC < 100 & > 94 & is_charging !! , do PSOC += 1 , PSOC = %d \n",__func__,PSOC);
						}
					}else{
						FC = false;
						set_FC_id_to_File(0);
						printk("[%s][WB] PSOC  > 94  & not charging !! , set FC = false \n",__func__,FC);
					}
				}	
			}
        	}
	}else{ // CABLE OUT 
		printk("[%s][WB] CABLE OUT ( !(CABLE_IN & Current > 0) ) \n",__func__);
		if(FC == false){	
				printk("[%s][WB] FC == false \n",__func__);
				if(sn200007_device->bat_vol < 3400){
                        		// Debug
                        		printk("[%s][WB] voltage < 3400 \n",__func__);
                        		if(PSOC <=1){
                                		do{
										msleep(2000);
										sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr, 0, &rt_value);
										if (sn200007_device->smbus_status < 0) { 
											dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_VOLTAGE].addr);
											}
										sn200007_device->bat_vol = rt_value;
										if(sn200007_device->bat_vol < 3400 ){
											count++;
											printk("[%s][WB] GAUGE IC Voltage = %d , %d times \n",__func__,sn200007_device->bat_vol,count);
										}else{
											printk("[%s][WB] GAUGE IC Voltage = %d , need to recheck\n",__func__,sn200007_device->bat_vol);
											goto Done;
										}
									}while(count < 3);
                                		/* system off */
                                		PSOC = 0;
                                		// Debug
                                		printk("[%s][WB] PSOC <=1 , setup USOC = 0 , USOC = %d , PSOC = %d \n",__func__,sn200007_device->old_capacity,PSOC);
                        		}else{
                                		PSOC -= 1;
                                		// Debug
                                		printk("[%s][WB] PSOC > 1 PSOC-=1 , PSOC : %d \n",__func__,PSOC);
                        		}
             			}else{
                        		// Debug ++++
                       	 		printk("[%s][WB] sn200007_device->bat_vol >= 3400\n",__func__);

                        		if(PSOC > sn200007_device->old_capacity){
                                		if(PSOC<=1){
                                        		PSOC = 1;
							BT = 0;
                                        		// Debug ++++
                                        		printk("[%s][WB] PSOC<=1 , PSOC=1, set BT = %d \n",__func__,BT);
                                		}else{
                                        		PSOC -=1;
							BT = 0;
                                        		// Debug ++++
                                        		printk("[%s][WB] PSOC>1 , PSOC-=1, PSOC : %d , set BT = %d \n",__func__,PSOC,BT);
                                		}
					}else{
                                		/* do nothing */
                                		printk("[%s][WB] sn200007_device->bat_vol >= 3400 & PSOC <= USOC \n",__func__);
										if(BT==0){
							BT = sn200007_device->old_capacity;
							printk("[%s][WB] BT = USOC , BT = %d\n",__func__,BT);
						}else{

							if(sn200007_device->old_capacity > BT){
								BT = sn200007_device->old_capacity;
								printk("[%s][WB] USOC > BT , set BT = USOC \n",__func__);
							}else{
								
								printk("[%s][WB] USOC < BT , \n",__func__);
								printk("[%s][WB] BT-USOC = %d , %u\n",__func__,BT-sn200007_device->old_capacity,BT-sn200007_device->old_capacity);
								if((BT-(int)sn200007_device->old_capacity)>2){
									printk("[%s][WB] BT-sn200007_device->old_capacity > 2 \n",__func__);
									if(PSOC<=1){
										PSOC = 1;
										BT = 0;
										printk("[%s][WB] PSOC <=1 , set PSOC = 1 & BT = 0\n",__func__);
									}else{
										PSOC-=1;
										BT=0;
										printk("[%s][WB] PSOC>1 , set PSOC-=1 %d, BT=0\n",__func__,PSOC);
									}
								}else{
									printk("[%s][WB] BT-USOC <=2 \n",__func__);
								}
							}
						}
						
                        		}
               			}
			}else{

				printk("[%s][WB] FC == true \n",__func__);
				if(sn200007_device->old_capacity <=94){
					FC = false;
					set_FC_id_to_File(0);
					printk("[%s][WB] USOC <= 94%% FC Flag setup to false \n",__func__);
				}else{
					if(is_charging() == true){
						if(PSOC >= 100){
							PSOC = 100;
							printk("[%s][WB] PSOC >=100  & is_charging !! , setup to 100%%\n",__func__);
						}else{
							PSOC +=1;
							printk("[%s][WB] PSOC < 100 & > 94 & is_charging !! , do PSOC += 1 , PSOC = %d \n",__func__,PSOC);
						}
					}else{
						FC = false;
						set_FC_id_to_File(0);
						printk("[%s][WB] PSOC  > 94  & not charging !! , set FC = false \n",__func__,FC);
					}
				}	
			}  // End FC == false
		
	} // End Cable Out

	// save PSOC to file
	// save PSOC 
	printk("[%s][WB] before Save PSOC\n",__func__);
	set_value_to_File(PSOC_VALUE_FILE_PATH,PSOC);
	printk("[%s][WB] after Save PSOC\n",__func__);
	// save PSOC

	return; 
Done:
	
		printk("[%s][WB] voltage > 3.4v , voltage = %d , need to recheck next time\n",__func__,sn200007_device->bat_vol);
	
}
// [WB]  -----


void sn200007_update_all(void) {
        s32 ret;
        int rt_value = 0;
        int temperature = 0;
        static char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

        // 1. capacity
        sn200007_get_capacity();

        // 2. voltage
        sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr, 0, &rt_value);
        if (sn200007_device->smbus_status < 0) {
		dev_err(&sn200007_device->client->dev, "%s: i2c read for %d failed\n", __func__, sn200007_data[REG_VOLTAGE].addr);
	}
        sn200007_device->bat_vol = rt_value;
	BAT_NOTICE("voltage_now= %d mV\n", sn200007_device->bat_vol);

        // 3. current
        sn200007_get_current();
	BAT_NOTICE("current_now= %d mA\n", sn200007_device->bat_current);

        // 4. temperature
        sn200007_get_temperature();
        BAT_NOTICE("temperature= %d (0.1 Celsius degrees)\n", sn200007_device->old_temperature);

	// 5. capacity optimazation
        batt_capacity_opti(); 

        // 6. status
        aicl_dete_worker(NULL);

	//ret = is_charging_full() ? FULL : is_charging() ? CHARGING : DISCHARGING;
	if(is_charging_full()){
		BAT_NOTICE("[%s][WB] Charging status : FULL \n",__func__);
	}
	ret = (PSOC == 100) ? FULL : is_charging() ? CHARGING : DISCHARGING;
        sn200007_device->bat_status = ret;
        BAT_NOTICE("status: %s \n", status_text[ret]);

	return;
}

//+++ i2c stress test
static void sn200007_stress_test_poll(struct work_struct * work) {
    int ret = 0, percentage = 50;

    ret = sn200007_smbus_read_data_reg(sn200007_data[REG_CAPACITY].addr   , 0 , &percentage);
    if (ret < 0) {
        BAT_ERR("------------> sn200007 gauge i2c fail !!!!! <--------------------\n");
    } else {
        BAT_NOTICE("------------> stress test percentage  = %d , polling = %d sec <--------------------\n", percentage , stress_test_poll_mode/HZ);
    }

    if (!smb345_has_charger_error())
        BAT_NOTICE("------------> smb345 charger status successful , polling = %d sec <--------------------\n", stress_test_poll_mode/HZ);
    else
        BAT_ERR("------------> smb345 charger status fail !!!!! <--------------------\n");

    queue_delayed_work(sn200007_stress_test_work_queue, &sn200007_stress_test_poll_work, stress_test_poll_mode);
}

int sn200007_open(struct inode *inode, struct file *filp) {
	printk("battery sn200007 : %s\n", __func__);
	return 0;
}

int sn200007_release(struct inode *inode, struct file *filp) {
	printk("battery sn200007: %s\n", __func__);
	return 0;
}

static unsigned int sn200007_poll(struct file *filp, poll_table *wait){
	unsigned int mask = 0;
	poll_wait(filp, &poll_wait_queue_head_t, wait);
	if (flag_pollin==true) {
		mask |= POLLIN;
		flag_pollin=false;
	}
	printk("battery sn200007 : %s\n", __func__);
	return mask;
}

long sn200007_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	int err = 1;
	printk("--------------------------> sn200007_ioctl \n");
	if (_IOC_TYPE(cmd) != sn200007_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > sn200007_IOC_MAXNR)
		return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case sn200007_POLL_DATA:
			if (arg == sn200007_IOCTL_START_HEAVY){
				printk("battery ba27541 : ioctl heavy\n");
				stress_test_poll_mode = START_HEAVY;
				queue_delayed_work(sn200007_stress_test_work_queue, &sn200007_stress_test_poll_work, stress_test_poll_mode);
			} else if (arg == sn200007_IOCTL_START_NORMAL){
				printk("battery ba27541 : ioctl normal\n");
				stress_test_poll_mode = START_NORMAL;
				queue_delayed_work(sn200007_stress_test_work_queue, &sn200007_stress_test_poll_work, stress_test_poll_mode);
			} else if  (arg == sn200007_IOCTL_END){
				printk("light sensor sn200007 : ioctl end\n");
				cancel_delayed_work_sync(&sn200007_stress_test_poll_work);
			} else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
        }

        return 0;
}

struct file_operations sn200007_fops = {
	.owner = THIS_MODULE,
	.open = sn200007_open,
	.release = sn200007_release,
	.poll = sn200007_poll,
	.unlocked_ioctl = sn200007_ioctl,
};

//---


static void battery_status_poll(struct work_struct *work) {
	polling_time = 60*HZ;
        struct sn200007_device_info *batt_dev = container_of(work, struct sn200007_device_info, status_poll_work.work);
        char *status_text[] = {"Unknown", "Charging", "Discharging", "Not charging", "Full"};

        if(!battery_driver_ready)
		BAT_NOTICE("battery driver not ready\n");

        sn200007_update_all();

		/* avoid do not update battery cell table in COS*/
		if(entry_mode == 4 && g_fw_success == 0 && sn200007_device->bat_vol > 3600 ){
			printk("[%s][WB] start to update battery cell table in COS\n",__func__);
			queue_delayed_work(sn200007_update_work_queue, &sn200007_device->update_gauge_work, 3*HZ);
		}
		/* avoid do not update battery cell table in COS*/
		

        power_supply_changed(&sn200007_supply[Charger_Type_Battery]);

        msleep(10);


	if (sn200007_device->old_temperature >= 150 && sn200007_device->old_temperature <= 450){
		polling_time = 90 * HZ;
	}else{
		polling_time = 30 * HZ;
		printk("[%s][WB] OTHER CASE Temp : %d\n",__func__,sn200007_device->old_temperature);
	}



	if( ( (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ) && (sn200007_device->bat_current > 0)  ){
        	printk("[%s][WB] (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ) && (sn200007_device->bat_current > 0) ,do nothing, polling_time = %d\n",__func__,polling_time);
	}else{
		if (PSOC >= 50 && PSOC <= 100)
            		polling_time = (90*HZ) < polling_time ? (90*HZ) : polling_time;
        	else if (PSOC >= 20 && PSOC <= 49)
            		polling_time = (60*HZ) < polling_time ? (60*HZ) : polling_time;
        	else if (PSOC >= 5 && PSOC <= 19)
            		polling_time = (30*HZ) < polling_time ? (30*HZ) : polling_time;
        	else if (PSOC >= 0 && PSOC <= 4)
            		polling_time = (30*HZ) < polling_time ? (30*HZ) : polling_time;
       		else {
            		BAT_NOTICE("*** Battery percentage is out of the legal range (percentage < 0 or percentage > 100) ***\n");
            		polling_time = 5*HZ;
        	}
	}
        printk("<BATT> battery info (P:%d %%(%d %%)(%d %%), V:%d mV, C:%d mA, T:%d.%d P: %d secs S: %s)\n",
                PSOC,
		sn200007_device->old_capacity,
                sn200007_device->bat_capacity,
                sn200007_device->bat_vol,
                sn200007_device->bat_current,
                sn200007_device->old_temperature/10,
                sn200007_device->old_temperature%10,
                polling_time/HZ,
                status_text[sn200007_device->bat_status]);

	/* Schedule next polling */
	queue_delayed_work(battery_poll_work_queue, &batt_dev->status_poll_work, polling_time);
}

static void low_low_battery_check(struct work_struct *work) {

	cancel_delayed_work(&sn200007_device->status_poll_work);
	queue_delayed_work(battery_poll_work_queue,&sn200007_device->status_poll_work, 0.1*HZ);
	msleep(2000);
	enable_irq(sn200007_device->irq_low_battery_detect);
}

#ifdef SN200007_CALLBACK_FUNC
void sn200007_battery_callback(unsigned USB_PC_state) {
	int old_cable_status = battery_cable_status;
	battery_cable_status = USB_PC_state;

        printk("========================================================\n");
	printk("sn200007_battery_callback  USB_PC_state = %x\n", USB_PC_state);
        printk("========================================================\n");

	if(!battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return;
	}

	check_cabe_type();

	cancel_delayed_work(&sn200007_device->status_poll_work);
	//queue_delayed_work(battery_poll_work_queue, &sn200007_device->status_poll_work,
	//	battery_cable_status ? DELAY_FOR_CORRECT_CHARGER_STATUS *HZ : 2*HZ);
        queue_delayed_work(battery_poll_work_queue, &sn200007_device->status_poll_work, 0.1*HZ);
}
EXPORT_SYMBOL(sn200007_battery_callback);
#endif

static int sn200007_get_health(enum power_supply_property psp, union power_supply_propval *val) {

	if (psp == POWER_SUPPLY_PROP_PRESENT) {
		val->intval = 1;
	} else if (psp == POWER_SUPPLY_PROP_HEALTH) {
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
	}
	return 0;
}

static int sn200007_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val) {

	switch (psp) {
		case POWER_SUPPLY_PROP_PRESENT:
		case POWER_SUPPLY_PROP_HEALTH:
			if (sn200007_get_health(psp, val))
				goto error;
			break;

		case POWER_SUPPLY_PROP_TECHNOLOGY:
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
			break;

		case POWER_SUPPLY_PROP_CAPACITY:
			// PSOC : end user read state of charge
			val->intval = PSOC;
			break;

		case POWER_SUPPLY_PROP_STATUS:
			/* INSERT POWER SOURCE */
                       if( (G_USB_TYPE == AC_IN) || (G_USB_TYPE == USB_IN) ){  
			   if(get_sw_charging_toggle()){
				   // PSOC : end user read state of charge
                           	if(PSOC == 100 ) {
					val->intval = POWER_SUPPLY_STATUS_FULL;
			   	} else {
					val->intval = POWER_SUPPLY_STATUS_CHARGING;
			   	}
			   }else{
				val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			   }
                       	} else {
			   	val->intval = POWER_SUPPLY_STATUS_DISCHARGING;                                
		        }
                       break;
		case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                       val->intval = sn200007_device->bat_vol*1000;
                       break;
		case POWER_SUPPLY_PROP_CURRENT_NOW:
                       val->intval = sn200007_device->bat_current*1000;
                       break;
		case POWER_SUPPLY_PROP_TEMP:
                case POWER_SUPPLY_PROP_TEMP_AMBIENT:
                       val->intval = sn200007_device->old_temperature;
                       break;
		case POWER_SUPPLY_PROP_CHARGE_FULL:
		       // unit in mAh
		       val->intval = Charge_Full;
		       break;
		default:
			dev_err(&sn200007_device->client->dev, "%s: INVALID property psp=%u\n", __func__,psp);
			return -EINVAL;
	}
	return 0;
error:

	return -EINVAL;
}

static int sn200007_proc_info_dump_read(struct seq_file *m, void *p) {
        static int bq_batt_percentage = 0;
        static int bq_batt_volt = 0;
        static int bq_batt_current = 0;
        static int bq_batt_temp = 0;
        static int bq_batt_remaining_capacity = 0;
        static int bq_batt_full_charge_capacity = 0;

        sn200007_smbus_read_data_reg(0x0E, 0 ,&bq_batt_full_charge_capacity);
        sn200007_smbus_read_data_reg(0x0C, 0 ,&bq_batt_remaining_capacity);
        sn200007_smbus_read_data_reg(sn200007_data[REG_CAPACITY].addr   , 0 ,&bq_batt_percentage);
        sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr    , 0 ,&bq_batt_volt);
        bq_batt_current = sn200007_get_current();
        bq_batt_temp    = sn200007_get_temperature();

        seq_printf(m,"LMD(mAh): %d\n", bq_batt_full_charge_capacity);
        seq_printf(m,"NAC(mAh): %d\n", bq_batt_remaining_capacity);
        seq_printf(m,"RSOC: %d\n", bq_batt_percentage);
        seq_printf(m,"USOC: %d\n", sn200007_device->old_capacity);
	// [WB]
	seq_printf(m,"PSOC: %d\n",PSOC);
        seq_printf(m,"voltage(mV): %d\n", bq_batt_volt);
        seq_printf(m,"average_current(mA): %d\n", bq_batt_current);
        seq_printf(m,"temp: %d\n", bq_batt_temp);

        return 0;
}

static int proc_sn200007_test_info_dump_open(struct inode *inode, struct file *file) {
	return single_open(file, sn200007_proc_info_dump_read, NULL);
}

static const struct file_operations proc_sn200007_test_info_dump_ops = {
	.open		= proc_sn200007_test_info_dump_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release
};

static int sn200007_register_upilogger_proc_fs(void) {
        struct proc_dir_entry *entry = NULL;

        entry = proc_create("bq27520_test_info_dump", 0664, NULL, &proc_sn200007_test_info_dump_ops);
        if (!entry) {
            printk("[%s]Unable to create sn200007_test_info_dump \n", __FUNCTION__);
            return -EINVAL;
        }
        return 0;
}

static bool checkBatteryInvalidStatus(void) {
    int   temperature;
    int   capacity;

    sn200007_smbus_read_data_reg(sn200007_data[REG_CAPACITY].addr, 0 ,&capacity);

    //  check if battery capacity is too low, if yes notify system to do shutdown
    if (capacity <= 5) {
         BAT_NOTICE( "===  battery capacity is too low in sleep!!!\n");;
         return true;
    }
    return false;
}

#if defined (CONFIG_PM)
static int sn200007_suspend (struct device *dev){
        printk("enter %s\n", __func__);
	cancel_delayed_work_sync(&sn200007_device->status_poll_work);
	return 0;
}

/* any smbus transaction will wake up pad */
static int sn200007_resume (struct device *dev) {
    printk("enter %s\n", __func__);
	
	queue_delayed_work(battery_poll_work_queue,&sn200007_device->status_poll_work, 0*HZ);

	return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sn200007_early_suspend(struct early_suspend *h) {
        printk("enter %s\n", __func__);
}

static void sn200007_late_resume(struct early_suspend *h) {
        printk("enter %s\n", __func__);
        queue_delayed_work(battery_poll_work_queue,&sn200007_device->status_poll_work, 0*HZ);
}
#endif

//+++++++++++++ fw update +++++++++++++++++
#include "default_factory_data.h"

#define LATEST_GAUGE_ID_LGC             0x3489
#define LATEST_GAUGE_ID_ATL             0x3490
#define LATEST_FW_CFG_VERSION           0x00
#define LATEST_FW_CFG_VERSION_LGC       6 //Data flash Version 06
#define LATEST_FW_CFG_VERSION_ATL       5


typedef enum _update_status {
    UPDATE_GAUGE_VERSION = -6,
    UPDATE_INVALID_BATTID = -5,
    UPDATE_PROCESS_FAIL = -4,
    UPDATE_ERR_MATCH_OP_BUF = -3,
    UPDATE_CHECK_MODE_FAIL = -2,
    UPDATE_VOLT_NOT_ENOUGH = -1,
    UPDATE_NONE = 0,
    UPDATE_OK,
    UPDATE_FROM_ROM_MODE,
} update_status;

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
    u32 off; //OP_WAIT not use
    u32 arg;
} ;

extern struct bq27xx_dffs_data default_bq27xx_fw[];
extern const unsigned int default_num_of_op_buf;

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
		BAT_DBG_E("%s: open file failed, ret = %d\n", __func__, IS_ERR(fp));
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

int bq27520_write_i2c(struct i2c_client *client, u8 reg, int value, int b_single)
{
	struct i2c_msg msg;
	unsigned char data[3];

	if (!client || !client->adapter)
		return -ENODEV;

	memset(data, 0, sizeof(data));
	data[0] = reg;
	data[1] = value& 0x00FF;
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

int bq27520_read_i2c(struct i2c_client *client, u8 reg, int *rt_value, int b_single)
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

int bq27520_cmp_i2c(int reg_off, int value) {
    int retry = 3;
    int val=0;
    int ret=0;

    BAT_NOTICE("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(sn200007_device->client, reg_off, &val, 1);
        if (ret < 0) continue;

	// add re-try 3 times & 400 ms delay
	if(value != val){
		printk("[%s][WB] value != val  value = %d , val = %d , remainder times %d\n",__func__,value,val,retry);
		mdelay(400);
		continue;
	}


        break;
    };

    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

int bq27520_asus_battery_dev_read_fw_cfg_version(void) {
    int fw_cfg_ver=0;
    int ret;

    ret = bq27520_write_i2c(sn200007_device->client, 0x3F, 0x01, 1);
    if (ret) {
        dev_err(sn200007_device->client, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800); //delay awhile to get version. Otherwise version data not transfer complete
    ret = bq27520_read_i2c(sn200007_device->client, 0x40, &fw_cfg_ver, 0);
    if (ret) {
        dev_err(sn200007_device->client, "Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return fw_cfg_ver;
}

int bq27621_asus_battery_dev_read_write_fw_cfg_version(bool is_write)
{
	int ret = 0, flag = 0, old_csum = 0, csum = 0;
	int old_DV = 0, new_DV = 0;
	int temp = 0, count = 0;

	/* 1. UNSEAL it by sending the appropriate keys to Control( ) (0x00 and 0x01) */
	ret = bq27621_unseal();
	if (ret < 0) {
		dev_err(sn200007_device->client, "UNSEAL device error %d.\n", ret);
		return ret;
	}

	/* 2. Send SET_CFGUPDATE subcommand, Control(0x0013) */
	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x13, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Send SET_CFGUPDATE subcommand error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Send SET_CFGUPDATE subcommand error %d.\n", ret);
		return ret;
	}

	/*
	 * 3. Confirm CONFIG UPDATE mode by polling Flags( ) register until bit 4 is set.
	 *    May take up to 1 second.
	 */
	count = 0;
	while (((flag & 0x10) >> 4) != 1) {
		ret = bq27520_read_i2c(sn200007_device->client, 0x06, &flag, 1);
		if (ret < 0) {
			dev_err(sn200007_device->client, "Confirm CONFIG UPDATE mode error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] flags = %x\n", __func__, flag);

		if (count > 6)
			return -301;

		count++;
		msleep(500);
	}

	/*
	 * 4. Write 0x00 using BlockDataControl( ) command (0x61) to enable block data
	 *    memory control
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x61, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Enable block data memory control error %d.\n", ret);
		return ret;
	}

	/*
	 * 5. Write 0x40 using the DataClass( ) command (0x3E) to access the State
	 *    subclass (64 decimal, 0x40 hex) containing the DF Version parameter.
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x3E, 0x40, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Access the State subclass error %d.\n", ret);
		return ret;
	}

	/*
	 * 6. Write the block offset location using DataBlock( ) command (0x3F).
	 *    Note: To access data located at offset 0 to 31 use offset = 0x00. To access
	 *          data located at offset 32 to 41 use offset = 0x01.
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x3F, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write the block offset location error %d.\n", ret);
		return ret;
	}

	/* 7. Read the 1-byte checksum using the BlockDataChecksum( ) command (0x60). */
	ret = bq27520_read_i2c(sn200007_device->client, 0x60, &old_csum, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Read the 1-byte checksum error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_csum = %x\n", __func__, old_csum);

	/* 8. Read DF Version byte at 0x43 (offset = 3). */
	ret = bq27520_read_i2c(sn200007_device->client, 0x43, &old_DV, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Read DF Version error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_DV = %x\n", __func__, old_DV);

	if (is_write) {
		/* 9. Write DF Version byte at 0x43. */
		ret = bq27520_write_i2c(sn200007_device->client, 0x43, LATEST_FW_CFG_VERSION, 1);
		if (ret < 0) {
			dev_err(sn200007_device->client, "Write DF Version error %d.\n", ret);
			return ret;
		}

		/* Check new DF Version */
		ret = bq27520_read_i2c(sn200007_device->client, 0x43, &new_DV, 1);
		if (ret < 0) {
			dev_err(sn200007_device->client, "Check new DF Version error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] new_DV = %x\n", __func__, new_DV);

		/* 10. Compute the new block checksum. */
		temp = (255 - old_csum - old_DV) % 256;
		pr_info("%s: [M] temp = %x\n", __func__, temp);
		csum = (255 - ((temp + LATEST_FW_CFG_VERSION) % 256)) & 0xFF;
		pr_info("%s: [M] csum = %x\n", __func__, csum);

		/* 11. Write new checksum. */
		ret = bq27520_write_i2c(sn200007_device->client, 0x60, csum, 1);
		if (ret < 0) {
			dev_err(sn200007_device->client, "Write new checksum error %d.\n", ret);
			return ret;
		}
	}

	/*
	 * 12. Exit CONFIG UPDATE mode by sending SOFT_RESET subcommand,
	 *     Control(0x0042)
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x42, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Sending SOFT_RESET subcommand error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Sending SOFT_RESET subcommand error %d.\n", ret);
		return ret;
	}

	/*
	 * 13. Confirm CONFIG UPDATE has been exited by polling Flags( ) register until bit
	 *     4 is cleared. May take up to 1 second.
	 */
	count = 0;
	while (((flag & 0x10) >> 4) != 0) {
		ret = bq27520_read_i2c(sn200007_device->client, 0x06, &flag, 1);
		if (ret < 0) {
			dev_err(sn200007_device->client, "Confirm CONFIG UPDATE mode error %d.\n", ret);
			return ret;
		}
		pr_info("%s: [M] flags = %x\n", __func__, flag);

		if (count > 6)
			return -301;

		count++;
		msleep(500);
	}

	/* 14. Return to SEALED mode sending by the Control(0x0020) command. */
	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x20, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Return to SEALED mode error %d.\n", ret);
		return ret;
	}
	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Return to SEALED mode error %d.\n", ret);
		return ret;
	}

	return ret < 0 ? ret : old_DV;
}

int bq27621_asus_battery_dev_read_gauge_id(void)
{
	int ret = 0;
	int gauge_id = 0;
	int retry_counter = 3;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x08, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write control register error %d.\n", ret);
		return ret;
	}

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write control register error %d.\n", ret);
		return ret;
	}

	msleep(50);

	ret = bq27520_read_i2c(sn200007_device->client, 0x00, &gauge_id, 0);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Read gauge id error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] gauge_id = %x\n", __func__, gauge_id);

	return ret < 0 ? ret : gauge_id;
}

int bq27621_asus_battery_dev_reset(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x41, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write control register to reset error %d.\n", ret);
		return ret;
	}

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write control register to reset error %d.\n", ret);
		return ret;
	}

	mdelay(4000);

	return ret;
}

int bq27621_asus_battery_dev_read_write_fw_cfg_version_lite(void)
{
	int ret = 0, flag = 0, old_csum = 0, csum = 0;
	int old_DV = 0, new_DV = 0;
	int temp = 0, count = 0;
	int retry_counter = 3;


	/*
	 * 1. Write 0x40 using the DataClass( ) command (0x3E) to access the State
	 *    subclass (64 decimal, 0x40 hex) containing the DF Version parameter.
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x3E, 0x40, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Access the State subclass error %d.\n", ret);
		return ret;
	}

	msleep(50);

	/*
	 * 2. Write the block offset location using DataBlock( ) command (0x3F).
	 *    Note: To access data located at offset 0 to 31 use offset = 0x00. To access
	 *          data located at offset 32 to 41 use offset = 0x01.
	 */
	ret = bq27520_write_i2c(sn200007_device->client, 0x3F, 0x00, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Write the block offset location error %d.\n", ret);
		return ret;
	}

	msleep(50);

	/* 3. Read DF Version byte at 0x43 (offset = 3). */
	ret = bq27520_read_i2c(sn200007_device->client, 0x43, &old_DV, 1);
	if (ret < 0) {
		dev_err(sn200007_device->client, "Read DF Version error %d.\n", ret);
		return ret;
	}
	pr_info("%s: [M] old_DV = %x\n", __func__, old_DV);
	return ret < 0 ? ret : old_DV;
}

int bq27520_rom_mode_wait(int m_secs) {
    BAT_NOTICE("[%s] enter \n", __func__);

    if (m_secs < 1) return -EINVAL;

    msleep(m_secs);

    return 0;
}

int device_type(void) {
    int fw_cfg_ver=0;
    int ret;

    ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x01, 1);
    if (ret) {
        dev_err(sn200007_device->client, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
    if (ret) {
        dev_err(sn200007_device->client, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800); //delay awhile to get version. Otherwise version data not transfer complete
    sn200007_smbus_read_data_reg(0x00, 0 ,&fw_cfg_ver);
    if (ret) {
        dev_err(sn200007_device->client, "Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    printk("======= DEVICE_TYPE = 0x%04X \n", fw_cfg_ver);

    return fw_cfg_ver;
}

int fw_type(void) {
    int fw_cfg_ver=0;
    int ret;

    ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x02, 1);
    if (ret) {
        dev_err(sn200007_device->client, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
    if (ret) {
        dev_err(sn200007_device->client, "Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800); //delay awhile to get version. Otherwise version data not transfer complete
    sn200007_smbus_read_data_reg(0x00, 0 ,&fw_cfg_ver);
    if (ret) {
        dev_err(sn200007_device->client, "Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    printk("======= FW_VERSION  = 0x%04X \n", fw_cfg_ver);

    return fw_cfg_ver;
}

int update_fw(struct update_op *p_start, struct update_op *p_end) {
    int ret = UPDATE_OK;
    bool FAIL_flag = false;
    int fail_count = 0;

    BAT_NOTICE("(%s) Start update firmware ... ", __func__);
    struct update_op *p_temp;

    p_temp = p_start;

    ret = UPDATE_PROCESS_FAIL;
    while (p_start <= p_end) {
        int op_execute_ret = 0;
		int cfg_ret = 0;       //[Carlisle] For gauge status check
#if 0
	/* print update command */
	BAT_DBG("OP: %d, off: %02X, val: %04X\n",
                p_start->bq_op, p_start->off, p_start->arg
        );
#endif

        switch(p_start->bq_op) {
#if 0
        case OP_ROM_END:
        case OP_ROM_WRITE:
            op_execute_ret = bq27520_rom_mode_write_i2c(p_start->off, p_start->arg, 1);
            if (op_execute_ret < 0) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            }
            break;

        case OP_ROM_CMP:
            op_execute_ret = bq27520_rom_mode_cmp(p_start->off, p_start->arg);
            if (op_execute_ret == PROC_FALSE) {
                BAT_DBG_E("   FAIL\n");
                return ret;
            } else if (op_execute_ret != PROC_FALSE && op_execute_ret != PROC_TRUE) {
                if (op_execute_ret < 0) {
                    BAT_DBG_E("   FAIL\n");
                    return ret;
                }
            }
            break;
#endif

        case OP_I2C_START:
        case OP_I2C_WRITE:
            mdelay(5);
            op_execute_ret = bq27520_write_i2c(sn200007_device->client, p_start->off, p_start->arg, 1);
            if (op_execute_ret < 0) {
                BAT_NOTICE("[%s][WB] write FAIL, ret = %d , fail -> op : %d , off : %02X , arg : %04X\n",__func__,op_execute_ret,p_temp->bq_op, p_temp->off, p_temp->arg);
            }
            break;

        case OP_I2C_CMP:
            op_execute_ret = bq27520_cmp_i2c(p_start->off, p_start->arg);
            if (op_execute_ret != PROC_FALSE && op_execute_ret != PROC_TRUE) {
                BAT_NOTICE("[%s][WB] cmp FAIL, ready to re-flash current block , ret = %d , re-try %d times\n", __func__ ,op_execute_ret,fail_count+1);
		printk("[%s][WB] FAIL block head -> op : %d , off : %02X , arg : %04X\n",__func__,p_temp->bq_op, p_temp->off, p_temp->arg);
		printk("[%s][WB] FAIL cmp -> op : %d , off : %02X ,  arg : %04X\n",__func__,p_start->bq_op, p_start->off, p_start->arg);
		p_start = p_temp;
		FAIL_flag = true;
		if(fail_count == 3){
			return ret;
		}
		fail_count++;
            } else if (op_execute_ret == PROC_FALSE) {
                BAT_NOTICE("[%s][WB] cmp FAIL(PROC_FALSE), ready to re-flash current block , ret = %d , re-try %d times\n",__func__ ,op_execute_ret,fail_count+1);
		printk("[%s][WB] FAIL block head -> op : %d , off : %02X , arg : %04X\n",__func__,p_temp->bq_op, p_temp->off, p_temp->arg);
		printk("[%s][WB] FAIL cmp -> op : %d , off : %02X ,  arg : %04X\n",__func__,p_start->bq_op, p_start->off, p_start->arg);
		p_start = p_temp;
		FAIL_flag = true;
		if(fail_count == 3){
			return ret;
		}
		fail_count++;
            }else{
		printk("[%s][WB] flash success ,  flash block head ->  op : %d off : %02x arg : %04X\n",__func__,p_temp->bq_op, p_temp->off, p_temp->arg);
		printk("[%s][WB] success cmp -> op : %d , off : %02X ,  arg : %04X\n",__func__,p_start->bq_op, p_start->off, p_start->arg);
		if((p_start+1)->bq_op == OP_I2C_CMP){
			printk("[%s][WB] next op is cmp , do nothing , cmp -> op : %d , off : %02X , arg : %04X \n",__func__,(p_start+1)->bq_op, (p_start+1)->off, (p_start+1)->arg);
		}else{
			fail_count = 0;
			p_temp = p_start + 1;
		}
	    }
            break;

        case OP_WAIT:
            op_execute_ret = bq27520_rom_mode_wait(p_start->arg);
            if (p_start->arg == 1500){
				BAT_DBG("%s: p_start->arg: %d\n", __func__, p_start->arg); //[Carlisle] For checking CFGUPD
				cfg_ret = bq27621_is_config_update_mode();
				if (cfg_ret != 1){
					BAT_NOTICE("Check CFGUPD FAIL\n");
				}
			} 
            if (op_execute_ret < 0) {
                BAT_NOTICE("   FAIL\n");
            }
            break;

        default:
            BAT_NOTICE("Not support OP \n");
            break;
        }; //end switch

	if(FAIL_flag == false){
        	p_start++;
	}else{
		printk("[%s][WB] Fail condition , p_start do nothing !!\n",__func__);
		FAIL_flag = false;
	}
    }; //end while

	ret = bq27621_soft_reset(); //Need soft-reset gauge IC after flashing FW
    ret = UPDATE_OK;
    BAT_NOTICE("%s Done.\n", __func__);

    return ret;
}


int update_normal(int curr_cell_type) {
	int ret;
	struct update_op *p_op_start = NULL, *p_op_end = NULL, *p_op_buf = NULL;
	int i,j;
	struct bq27xx_dffs_data *bq27xx_fw;
	unsigned int num_of_op_buf;

	bq27xx_fw = default_bq27xx_fw;
	num_of_op_buf = default_num_of_op_buf;

    BAT_NOTICE(" fw flashing... please wait for about 10s at least.\n");

    ret = UPDATE_ERR_MATCH_OP_BUF;
    for (i = 0; i < num_of_op_buf; i++) {
            if (curr_cell_type != bq27xx_fw[i].cell_type)
                continue;

		BAT_DBG("%s: curr_cell_type=%d, cell_type=%d\n", __func__, curr_cell_type, bq27xx_fw[i].cell_type);

            //Skip normal i2c mode op until OP_I2C_START + 1
            p_op_buf = bq27xx_fw[i].op;
            p_op_start = p_op_buf;
            p_op_end = &p_op_buf[bq27xx_fw[i].num_op-1];

            for (j=0; j < RETRY_COUNT; j++) {
		printk("[%s][WB] flash cell table %d times \n",__func__,j+1);
                ret = update_fw(p_op_start, p_op_end);
                if (ret == UPDATE_OK){ 
			Charge_Full = 3450;
			sn200007_update_all();
			power_supply_changed(&sn200007_supply[Charger_Type_Battery]);
			PSOC = sn200007_device->old_capacity;
			g_fw_success = 1; //update success 
			printk("[%s][WB] flash batt cell table is OK, setup Charge Full %d & PSOC = USOC PSOC %d \n",__func__,Charge_Full,PSOC);	
			break;
		}
            }
            break;

    }

    return ret;
}


int update_from_normal_mode(bool forceupdate) {

    int curr_cell_type=TYPE_LG;
    int ret = UPDATE_NONE;
    int fw_cfg_version, fw_gauge_id;
    int latest_fw_cfg_version = LATEST_FW_CFG_VERSION_LGC;
    int latest_fw_gauge_id = LATEST_GAUGE_ID_LGC;
    int count = 0;

    BAT_NOTICE("(%s) enter\n", __func__);

	asus_get_battery_id();
	BAT_NOTICE("(%s) battery id is %d\n", __func__, g_battery_id);

    if (g_battery_id == 0) {
	    latest_fw_cfg_version = LATEST_FW_CFG_VERSION_ATL;
	    latest_fw_gauge_id = LATEST_GAUGE_ID_ATL;
	    curr_cell_type = TYPE_ATL;
    } else if (g_battery_id == 1) {
	    latest_fw_cfg_version = LATEST_FW_CFG_VERSION_LGC;
	    latest_fw_gauge_id = LATEST_GAUGE_ID_LGC;
	    curr_cell_type = TYPE_LG;
    } else {
	    BAT_DBG_E("%s: Wrong battery ID: %d, use default settings\n", __func__, g_battery_id);
    }

   /*read gauge id from gauge ic*/
    do{
        fw_gauge_id = bq27621_asus_battery_dev_read_gauge_id();
        if (fw_gauge_id < 0) {
                count++;
                BAT_DBG_E("%s: read battery firmware gauge id failed, ret = %d, retry = %d times\n", __func__, fw_gauge_id,count);
                if(count == 3){
                        goto Done;
                }
        }else{
                break;
        }
    }while(count < RETRY_COUNT);


    /*read cfg verison from gauge ic */
    fw_cfg_version = bq27621_asus_battery_dev_read_write_fw_cfg_version_lite();
    if (fw_cfg_version < 0) {
           BAT_DBG_E("%s: read battery firmware config version failed, ret = %d\n", __func__, fw_cfg_version);
                        goto Done;
    }


    BAT_NOTICE("(%s) current_cell_type %d, fw_cfg_version %d, latest_cfg_version %d, fw_gauge_id %x, latest_fw_gauge_id %x\n",
            __func__, curr_cell_type, fw_cfg_version, latest_fw_cfg_version, fw_gauge_id, latest_fw_gauge_id);


    if (!forceupdate) {
         if (fw_cfg_version == latest_fw_cfg_version && fw_gauge_id == latest_fw_gauge_id) {
                BAT_NOTICE("========= No need to flash battery cell data due to that both data flash version and gauge id are equal =========");
                g_fw_success = 1; //update success
                goto Done;
         }
    } 

	smb345_charging_toggle(JEITA, false);
	msleep(5000);
    	ret = update_normal(curr_cell_type);
    	smb345_charging_toggle(JEITA, true);
    	if (ret < 0){
	    bq27621_asus_battery_dev_reset();
	    Charge_Full = 1202;
	    sn200007_update_all();
	    power_supply_changed(&sn200007_supply[Charger_Type_Battery]);
	    PSOC = sn200007_device->old_capacity;
	    printk("[%s][WB] flash batt default,set PSOC = USOC , PSOC =  %d,Charger_Full %d\n",__func__,PSOC,Charge_Full); 
    	}

Done:

    if (fw_cfg_version == latest_fw_cfg_version)
        g_fw_cfg_version = fw_cfg_version;
    else
        g_fw_cfg_version = bq27621_asus_battery_dev_read_write_fw_cfg_version_lite();

    if (fw_gauge_id == latest_fw_gauge_id)
	g_fw_gauge_id = fw_gauge_id;
    else
	g_fw_gauge_id = bq27621_asus_battery_dev_read_gauge_id();

    return ret;
}

/*[WB] for dev barnch "battery version" info  */
static ssize_t batt_switch_name(struct switch_dev *sdev,char *buf){

	printk("[%s][WB] GET batt_switch_name \n",__func__);

	return sprintf(buf,"%d-%d-%x\n",g_battery_id,bq27621_asus_battery_dev_read_write_fw_cfg_version_lite(),
			bq27621_asus_battery_dev_read_gauge_id());
}

void main_force_update_flow(void)
{
	update_from_normal_mode(true);
}

void main_update_flow(void) {
     if (sn200007_device) {                         
	if (!wake_lock_active(&gauge_wlock)) { 
		BAT_DBG(" %s: asus_battery_gauge_wakelock -> wake lock\n", __func__);
		wake_lock(&gauge_wlock);
	}
     }
     update_from_normal_mode(false);
     wake_unlock(&gauge_wlock);
}

//[Carlilse] Modify gauge fw update flow +++
static int sn200007_update_fw(void){
	BAT_DBG_E("%s\n", __func__);
	main_update_flow();
}
//[Carlilse] Modify gauge fw udpate flow ---

/* 3.
 * Confirm CONFIG UPDATE mode by polling Flags( ) register until bit 4 is set.
 * May take up to 1 second.
 */
int bq27621_is_config_update_mode(void)
{
	int ret = 0;
	int flag = 0;

	ret = bq27520_read_i2c(sn200007_device->client, 0x06, &flag, 1);
	if (ret < 0) {
		BAT_DBG_E("%s: Comfirm CONFIG UPDATE mode failed, %d\n",
			  __func__, ret);
		return ret;
	}

	BAT_DBG("%s: flag is %x\n", __func__, flag);

	ret = (flag & 0x10) >> 4;
	BAT_DBG("%s: return is %x\n", __func__, ret);

	return ret;
}

// ---------------- fw update ------------------

int bq27621_unseal(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x80, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x80, 1);
	if (ret < 0)
		return ret;

	return ret;
}

int bq27621_set_cfg_update(void)
{
	int ret = 0;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x03, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0)
		return ret;

	return ret;
}

/* +++ Create /proc/driver/gaugeIC_FW +++ */
static int proc_sn200007_firmware_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	asus_get_battery_id();
	BAT_DBG("%s: Battery ID = %d\n", __func__, g_battery_id);

#if 0
	BAT_DBG("%s: Input Battery ID = %s\n", __func__, buffer);

	if (buffer[0] == '0')
		g_battery_id = 0;
	else if (buffer[0] == '1')
		g_battery_id = 1;
#endif

	cancel_delayed_work(&sn200007_device->status_poll_work);
	main_force_update_flow();
	queue_delayed_work(battery_poll_work_queue, &sn200007_device->status_poll_work, 5*HZ);

	return count;
}

static int proc_sn200007_firmware_read(struct seq_file *m, void *p)
{
	BAT_DBG("battery id = %d, version = %d, gauge id = %x\n", g_battery_id, g_fw_cfg_version, g_fw_gauge_id);

	if (g_battery_id == 0) {
		if (g_fw_cfg_version == LATEST_FW_CFG_VERSION_ATL && g_fw_gauge_id == LATEST_GAUGE_ID_ATL) {
			seq_printf(m, "PASS\n");
			return 0;
		} else {
			seq_printf(m, "FAIL\n");
			return 0;
		}
	} else if (g_battery_id == 1) {
		if (g_fw_cfg_version == LATEST_FW_CFG_VERSION_LGC && g_fw_gauge_id == LATEST_GAUGE_ID_LGC) {
			seq_printf(m, "PASS\n");
			return 0;
		} else {
			seq_printf(m, "FAIL\n");
			return 0;
		}
	}

	seq_printf(m, "FAIL, unknown battery id %d\n", g_battery_id);

	return 0;
}

static int proc_sn200007_firmware_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sn200007_firmware_read, NULL);
}

static const struct file_operations proc_sn200007_firmware_ops = {
	.open = proc_sn200007_firmware_open,
	.read = seq_read,
	.write = proc_sn200007_firmware_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_sn200007_firmware_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/gaugeIC_FW", 0664, NULL, &proc_sn200007_firmware_ops);
	if (!entry) {
		printk("[%s] Unable to create sn200007_firmware\n", __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}
/* --- Create /proc/driver/gaugeIC_FW --- */

/* +++ Create /proc/driver/gaugeIC_FW_2 +++ */
static int proc_sn200007_firmware_write_2(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	asus_get_battery_id();
	BAT_DBG("%s: Battery ID = %d\n", __func__, g_battery_id);

#if 0
	BAT_DBG("%s: Input Battery ID = %s\n", __func__, buffer);

	if (buffer[0] == '0')
		g_battery_id = 0;
	else if (buffer[0] == '1')
		g_battery_id = 1;
#endif

	cancel_delayed_work(&sn200007_device->status_poll_work);
	main_update_flow();
	queue_delayed_work(battery_poll_work_queue,
		&sn200007_device->status_poll_work, 5*HZ);

	return count;
}

static int proc_sn200007_firmware_read_2(struct seq_file *m, void *p)
{
	BAT_DBG("battery id = %d, version = %d, gauge id = %x\n",
		g_battery_id, g_fw_cfg_version, g_fw_gauge_id);

	if (g_battery_id == 0) {
		if (g_fw_cfg_version == LATEST_FW_CFG_VERSION_ATL && g_fw_gauge_id == LATEST_GAUGE_ID_ATL) {
			seq_printf(m, "PASS\n");
			return 0;
		} else {
			seq_printf(m, "FAIL\n");
			return 0;
		}
	} else if (g_battery_id == 1) {
		if (g_fw_cfg_version == LATEST_FW_CFG_VERSION_LGC && g_fw_gauge_id == LATEST_GAUGE_ID_LGC) {
			seq_printf(m, "PASS\n");
			return 0;
		} else {
			seq_printf(m, "FAIL\n");
			return 0;
		}
	}

	seq_printf(m, "FAIL, unknown battery id %d\n", g_battery_id);

	return 0;
}

static int proc_sn200007_firmware_open_2(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sn200007_firmware_read_2, NULL);
}

static const struct file_operations proc_sn200007_firmware_ops_2 = {
	.open = proc_sn200007_firmware_open_2,
	.read = seq_read,
	.write = proc_sn200007_firmware_write_2,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_sn200007_firmware_proc_fs_2(void)
{
	struct proc_dir_entry *entry = NULL;

	entry =
	    proc_create("driver/gaugeIC_FW_2", 0664, NULL,
			&proc_sn200007_firmware_ops_2);
	if (!entry) {
		printk("[%s] Unable to create sn200007_firmware_2\n",
		       __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}
/* --- Create /proc/driver/gaugeIC_FW_2 --- */

// /proc/driver/gaugeIC_reset +++
static int bq27621_soft_reset(void)
{
	BAT_DBG_E(" %s:\n", __func__);

	int ret = 0;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x00, 0x42, 1);
	if (ret < 0)
		return ret;

	ret = bq27520_write_i2c(sn200007_device->client, 0x01, 0x00, 1);
	if (ret < 0)
		return ret;

	msleep(5000);

	return ret;
}

static int proc_sn200007_reset_write(struct file *file, const char *buffer, size_t count, loff_t *data)
{
	BAT_DBG_E(" %s:\n", __func__);

	int ret = 0;

	ret = bq27621_soft_reset();

	return count;
}

static int proc_sn200007_reset_read(struct seq_file *m, void *p)
{
	return 0;
}

static int proc_sn200007_reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_sn200007_reset_read, NULL);
}

static const struct file_operations proc_sn200007_reset_proc_ops = {
	.open = proc_sn200007_reset_open,
	.read = seq_read,
	.write = proc_sn200007_reset_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_sn200007_reset_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/gaugeIC_reset", 0664, NULL, &proc_sn200007_reset_proc_ops);
	if (!entry) {
		printk("[%s] Unable to create sn200007_reset\n", __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}
// /proc/driver/gaugeIC_reset ---

// *******************  Read the state-of-health information of the battery from DUT *****************

static int proc_sn200007_battery_soh_read(struct seq_file *m, void *p)
{
	int batt_temp = -99999;
	int batt_current = 0;
	int rt_value = 0;
	int ret = 0;
	int RM,FCC,DC;

	/** <Full Charge Capacity>  Ref Reg 0x0E & 0x0D **/
	ret = bq27520_read_i2c(sn200007_device->client, 0x0E, &FCC, 0);
	if(ret < 0){
		seq_printf(m, "FAIL");
		printk("[%s][WB] fail to read Full Charge Capacity register\n",__func__);
	}
	/** <Design Capacity> Ref Reg 0x3C & 0x3D  **/
	ret = bq27520_read_i2c(sn200007_device->client, 0x3C, &DC, 0);
	if(ret < 0){
		seq_printf(m, "FAIL");
		printk("[%s][WB] fail to read Design Capacity register\n",__func__); 
	}
	/** <Remain Capacity> Ref Reg 0x0C & 0x0D    **/
	ret = bq27520_read_i2c(sn200007_device->client, 0x0C, &RM, 0);
	if(ret < 0){
		seq_printf(m, "FAIL");
		printk("[%s][WB] fail to read RM register\n",__func__);	
	}
	/** <Temperature> , Ref Thermal Zone "batttemp" || Gauge IC temp **/
	batt_temp = sn200007_get_temperature();
	printk("[%s][WB] get battery temp : %d \n",__func__,batt_temp);
	/** <Voltage>  **/
	sn200007_device->smbus_status = sn200007_smbus_read_data_reg(sn200007_data[REG_VOLTAGE].addr, 0, &rt_value);
	if (sn200007_device->smbus_status < 0) {
		seq_printf(m, "FAIL");
	}
	sn200007_device->bat_vol = rt_value;
	printk("[%s][WB] get battery voltage : %d \n",__func__,sn200007_device->bat_vol);
	/** <Current>  **/
	batt_current = sn200007_get_current();
	printk("[%s][WB] get battery current : %d \n",__func__,batt_current);
	seq_printf(m,"FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(°C),VOLT=%d(mV),CUR=%d(mA)",FCC,DC,RM,(batt_temp/10),sn200007_device->bat_vol,batt_current);

	return 0;
}


static int proc_sn200007_battery_soh_open(struct inode *inode,struct file *file){
	return single_open(file, proc_sn200007_battery_soh_read,NULL);
}

static const struct file_operations proc_sn200007_read_battery_soh_proc_ops = {
	.open = proc_sn200007_battery_soh_open,
	.read = seq_read,
	.write = seq_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_sn200007_read_battery_soh_proc_fs(void){

	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/read_battery_soh",0664,NULL,&proc_sn200007_read_battery_soh_proc_ops);
	if(!entry){
		printk("[%s] Unable to create sn200007_read_battery_soh\n",__FUNCTION__);
		return -EINVAL;
	}
}

// ******************* Read the state-of-health information of the battery from DUT ******************

static int sn200007_probe(struct i2c_client *client, const struct i2c_device_id *id) {
	int ret, i = 0, rt_value;

	BAT_NOTICE("========================= (sn200007_probe) client->addr= %02x\n", client->addr);

	sn200007_device = kzalloc(sizeof(*sn200007_device), GFP_KERNEL);
	if (!sn200007_device) {
             BAT_ERR("Failed to allocate sn200007_device memory. \n");
             return -ENOMEM;
        }

	memset(sn200007_device, 0, sizeof(*sn200007_device));
	sn200007_device->client = client;
	i2c_set_clientdata(client, sn200007_device);

        sn200007_device->smbus_status    = 0;
        sn200007_device->cap_err         = 0;
	sn200007_device->temp_err        = 0;
	sn200007_device->old_capacity    = 50;
        sn200007_device->old_temperature = 250;
	sn200007_device->gpio_low_battery_detect = GPIOPIN_LOW_BATTERY_DETECT;

        sn200007_device->smbus_status = sn200007_smbus_read_data_reg( sn200007_data[REG_CAPACITY].addr, 0, &rt_value);//read gauge status
	if (sn200007_device->smbus_status < 0) {
            dev_err(&sn200007_device->client->dev, "%s, ==== fail to read gauge i2c ====\n", __func__);
            return -EINVAL;
	}

/*
	ret = set_design_voltage();
	if (ret < 0)
            dev_err(&sn200007_device->client->dev, "%s, ==== fail to set design voltage ====\n", __func__);
*/
	// FW update
//	main_update_flow();		[Carlisle]Updating from user-space

	for (i = 0; i < ARRAY_SIZE(sn200007_supply); i++) {
		ret = power_supply_register(&client->dev, &sn200007_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply ,num = %d\n", i);
                        do {
				power_supply_unregister(&sn200007_supply[i]);
			} while ((--i) >= 0);
		        kfree(sn200007_device);
			return ret;
		}
	}

	battery_poll_work_queue = create_singlethread_workqueue("battery_workqueue");
	INIT_DELAYED_WORK(&sn200007_device->status_poll_work, battery_status_poll);
	INIT_DELAYED_WORK(&sn200007_device->low_low_bat_work, low_low_battery_check);
        //INIT_DELAYED_WORK(&sn200007_device->battery_stress_test, battery_strees_test);
	cancel_delayed_work(&sn200007_device->status_poll_work);

	spin_lock_init(&sn200007_device->lock);
	wake_lock_init(&sn200007_device->low_battery_wake_lock, WAKE_LOCK_SUSPEND, "low_battery_detection");
	wake_lock_init(&sn200007_device->cable_event_wake_lock, WAKE_LOCK_SUSPEND, "battery_cable_event");
	wake_lock_init(&gauge_wlock, WAKE_LOCK_SUSPEND, "sn200007_wlock");

	/* Register sysfs */
	ret = sysfs_create_group(&client->dev.kobj, &battery_smbus_group);
	if (ret) dev_err(&client->dev, "sn200007_probe: unable to create the sysfs\n");

	/* Gauge FW update */
	sn200007_update_work_queue = create_singlethread_workqueue("gauge_update_wq");
	if(!sn200007_update_work_queue){
		printk("battery sn200007 : unable to create gauge FW update workqueue\n");
	}
	INIT_DELAYED_WORK(&sn200007_device->update_gauge_work, sn200007_update_fw);

    /* stress test */
	sn200007_stress_test_work_queue = create_singlethread_workqueue("i2c_battery_wq");
	if(!sn200007_stress_test_work_queue){
		printk("battery sn200007 : unable to create i2c stress test workqueue\n");
	}
	INIT_DELAYED_WORK(&sn200007_stress_test_poll_work, sn200007_stress_test_poll);

	/* Misc device registration */
        sn200007_device->battery_misc.minor = MISC_DYNAMIC_MINOR;
	sn200007_device->battery_misc.name = "battery";
        sn200007_device->battery_misc.fops  = &sn200007_fops;
        ret = misc_register(&sn200007_device->battery_misc);
	if (ret) dev_err(&client->dev, "Cannot register sn200007 miscdev (err = %d)\n", ret);

	/* register switch device for battery information versions report*/
	batt_dev.name = "battery";
	batt_dev.print_name = batt_switch_name;
	if(switch_dev_register(&batt_dev)<0){
		printk("[%s][WB] unable to create /sys/class/switch/battery \n",__func__);
	}


#ifdef CONFIG_HAS_EARLYSUSPEND
        sn200007_device->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
        sn200007_device->es.suspend = sn200007_early_suspend;
        sn200007_device->es.resume = sn200007_late_resume;
        register_early_suspend(&sn200007_device->es);
#endif

#if CONFIG_PROC_FS
        ret = sn200007_register_upilogger_proc_fs();
        if (ret) {
                BAT_ERR("Unable to create sn200007_register_upilogger_proc_fs\n");
        }
#endif
	
	// [WB] ++++++++++++++++
//	get_FC_flag();
	// [WB] ----------------

	battery_driver_ready = 1;
	check_cabe_type();
	queue_delayed_work(battery_poll_work_queue, &sn200007_device->status_poll_work, 5*HZ);
	if(entry_mode == 1){
		/* Update gauge fw */
		printk("[%s][WB] update battery cell table at non-COS mode\n",__func__);
		queue_delayed_work(sn200007_update_work_queue, &sn200007_device->update_gauge_work, 6*HZ);
	}
	ret = create_sn200007_firmware_proc_fs();
	if (ret) {
		BAT_ERR("Unable to create sn200007_firmware_proc_fs\n");
	}

	ret = create_sn200007_firmware_proc_fs_2();
	if (ret) {
		BAT_ERR("Unable to create sn200007_firmware_proc_fs_2\n");
	}

	ret = create_sn200007_reset_proc_fs();
	if (ret) {
		BAT_ERR("Unable to create sn200007_reset_proc_fs\n");
	}

	ret = create_sn200007_read_battery_soh_proc_fs();
	if(ret){
		BAT_ERR("Unable to create sn200007_read_battery_soh_proc_fs\n");
	}

	BAT_NOTICE("- %s driver registered done ====\n", client->name);

	return 0;
}

static int sn200007_remove(struct i2c_client *client) {
	struct sn200007_device_info *sn200007_device;
	int i = 0;

	sn200007_device = i2c_get_clientdata(client);
	for (i = 0; i < ARRAY_SIZE(sn200007_supply); i++) {
		power_supply_unregister(&sn200007_supply[i]);
	}

	if (sn200007_device) {
		wake_lock_destroy(&sn200007_device->low_battery_wake_lock);
		kfree(sn200007_device);
		sn200007_device = NULL;
	}

	wake_lock_destroy(&gauge_wlock);
	return 0;
}

static const struct dev_pm_ops sn200007_pm_ops = {
	.suspend                = sn200007_suspend,
	.resume	                = sn200007_resume,
};

static const struct i2c_device_id sn200007_id[] = {
	{ "sn200007-battery", 0 },
	{},
};

static struct i2c_driver sn200007_battery_driver = {
        .driver = {
		.name	= "sn200007-battery",
		.owner	= THIS_MODULE,
		.pm	= &sn200007_pm_ops,
	},
	.probe		= sn200007_probe,
	.remove		= sn200007_remove,
	.id_table	= sn200007_id,
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_CCD,
	},

	{ /* end: all zeroes */},
};

static int sn200007_idi_probe(struct idi_peripheral_device *ididev,
					const struct idi_device_id *id)
{
	struct resource *res;
	void __iomem *ctrl_io;
  void __iomem *pcfg_io;
	//struct smb345_charger *chrgr = &chrgr_data;
	int ret = 0;

	//spin_lock_init(&chrgr_dbg.lock);

	//CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_PROBE, 0, 0);
	pr_info("%s() - begin\n", __func__);

#if 0
	res = idi_get_resource_byname(&ididev->resources,
				IORESOURCE_MEM, "pmu_chgr_ctrl");

	if (res == NULL) {
		pr_err("getting pmu_chgr_ctrl resources failed!\n");
		return -EINVAL;
	}

	ctrl_io = ioremap(res->start, resource_size(res));

	if (!ctrl_io) {
		pr_err("mapping PMU's Charger registers failed!\n");
		return -EINVAL;
	}

	res = idi_get_resource_byname(&ididev->resources,
				IORESOURCE_MEM, "pmu_config_base");

	if (res == NULL) {
		pr_err("getting pmu_config_base resource failed!\n");
		return -EINVAL;
	}

	pcfg_io = ioremap(res->start, resource_size(res));

	if (!pcfg_io) {
		pr_err("mapping PMU's Charger registers failed!\n");
		return -EINVAL;
	}
	//chrgr->ctrl_io = ctrl_io;
    //chrgr->pcfg_io = pcfg_io;

	//chrgr->ididev = ididev;
#endif
	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return ret;
	}

	pr_info("%s() - end\n", __func__);

	return 0;
}

static int __exit sn200007_idi_remove(struct idi_peripheral_device *ididev)
{
	//CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	//iounmap(chrgr_data.ctrl_io);
	return 0;
}

static struct idi_peripheral_driver sn200007_idi_driver = {
	.driver = {
		.owner  = THIS_MODULE,
		.name   = "sn200007_idi",
		.pm = NULL,
	},
	.p_type = IDI_CCD,
	.id_table = idi_ids,
	.probe  = sn200007_idi_probe,
	.remove = sn200007_idi_remove,
};

static int __init sn200007_battery_init(void) {
	int ret;

        ret = idi_register_peripheral_driver(&sn200007_idi_driver);
	if (ret) {
                printk("----------- %s fail --------------", __func__);
		return ret;
        }

	ret = i2c_add_driver(&sn200007_battery_driver);
	if (ret)
		dev_err(&sn200007_device->client->dev, "%s: i2c_add_driver failed\n", __func__);

	return ret;
}

module_init(sn200007_battery_init);

static void __exit sn200007_battery_exit(void) {
	i2c_del_driver(&sn200007_battery_driver);
}
module_exit(sn200007_battery_exit);

MODULE_DESCRIPTION("sn200007 battery monitor driver");
MODULE_LICENSE("GPL");

