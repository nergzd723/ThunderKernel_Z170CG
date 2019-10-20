/*
 * This file is part of the AP3223, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
//#include <mach/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include "ap3223.h"
#include <../../arch/x86/platform/asustek/include/asustek_boardinfo.h>
extern unsigned int entry_mode;

/*--for of_--*/
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#define OF_PSENSOR_PIN "asus,psensor-irq"


#define AP3223_DRV_NAME		"ap3223"
#define DRIVER_VERSION		"1"


//#define PL_TIMER_DELAY 2000
#define POLLING_MODE 0

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("AP3223: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
static void lsensor_work_handler(struct work_struct *w);
#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data);
#endif
static int ap3223_set_phthres(struct i2c_client *client, int val);
static int ap3223_set_plthres(struct i2c_client *client, int val);

struct ap3223_data {
    struct i2c_client *client;
    u8 reg_cache[AP3223_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int gpio;
    int irq;
    int hsensor_enable;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
#ifdef HEARBEAT
    struct input_dev	*hsensor_input_dev;
#endif
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
    struct workqueue_struct *light_wq;
    struct delayed_work light_work;
#if POLLING_MODE
    struct timer_list pl_timer;
#endif
};

static struct ap3223_data *ap3223_data_g = NULL;
// AP3223 register
static u8 ap3223_reg[AP3223_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x14,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D, 0x30, 0x32};

// AP3223 range
static int ap3223_range[4] = {65535,16383,4095,1023};

/*-- lscali psxtalk pslow pshigh*/
static int asus_cali_value[6][4] =
{
  {100,194,86,229}, //SR
  {100,194,86,229}, //ER 
  {200,294,286,329}, //ER2
  {233,400,70,120}, //prePR
  {233,400,70,120}, //PR
  {233,398,72,144}, //MP
};


static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;

/*
 * Define for asus initial setting
 */
static int asuscali = 100;     //default offset for 65535 lux/resolution
static int asus_al_thL = 0;    //setting foe disable INT
static int asus_al_thH = 65535;  //setting foe disable INT
static int asus_calibration_lux = 1000;
static int asus_ps_thL = 86;  //test calibration value
static int asus_ps_thL_def = 86;  //test calibration value from ER2
static int asus_ps_thH = 229;  //test calibration value from ER2
static int asus_ps_thH_def = 229;  //test calibration value from ER2
static int asus_ps_xtalk = 194;  //test calibration value from ER2
static int asus_ps_xtalk_def = 194;  //test calibration value from ER2
static int asus_ps_integrated_time_ER = 0x07;  //ER factory uses 6T for integrated time
static int asus_ls_gain = 0x01;
static int PL_TIMER_DELAY = 2000;
static int L_TIMER_DELAY = 200;

static int ap3223_als_poll_time = 200; // ms , 5hz
static int ap3223_als_min_poll_time = 200; // ms , 5hz
static int ap3223_ps_poll_time = 200; // ms , 5hz
static int ap3223_ps_min_poll_time = 200; // ms , 5hz

#define LIGHTSENSOR_CALIBRATION_DATA "/factory/LightSensor.ini"
#define XTALK_CALIBRATION_DATA    	  "/factory/proximitysensor/PS_Config_Xtalk.ini"
#define HTH_CALIBRATION_DATA 		  "/factory/proximitysensor/PS_Config_Threshold_High.ini"
#define LTH_CALIBRATION_DATA		  "/factory/proximitysensor/PS_Config_Threshold_Low.ini"
#define LIGHT_POLLING                //only light sensor use polling mode

/*
 * Define for asus initial setting
 */
static ssize_t ap3223_als_set_delay(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count);

static DEFINE_MUTEX(ap3223_lock);
static DEFINE_MUTEX(ap3223_ls_lock);
static DEFINE_MUTEX(ap3223_ps_lock);
#ifdef HEARBEAT
static DEFINE_MUTEX(ap3223_heartbeat_lock);
#endif
#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}


/*
 * register access helpers
 */

static int __ap3223_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3223_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3223_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3223_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;

    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3223_get_range(struct i2c_client *client)
{
    u8 idx = __ap3223_read_reg(client, AP3223_REG_ALS_CONF,
	    AP3223_ALS_RANGE_MASK, AP3223_ALS_RANGE_SHIFT); 
    return range[idx];
}

static int ap3223_set_range(struct i2c_client *client, int range)
{
    return __ap3223_write_reg(client, AP3223_REG_ALS_CONF,
	    AP3223_ALS_RANGE_MASK, AP3223_ALS_RANGE_SHIFT, range);
}


static int ap3223_set_ir_data(struct i2c_client *client, int en)
{
    int ret = 0;

    if(en == 9) {
	ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
		AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_RESET);
	mdelay(200);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_CONF,
		AP3223_REG_PS_CONF_MASK, AP3223_REG_PS_CONF_SHIFT, 0);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_DC_1,
		AP3223_REG_PS_DC_1_MASK, AP3223_REG_PS_DC_1_SHIFT, 0);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_DC_2,
		AP3223_REG_PS_DC_2_MASK, AP3223_REG_PS_DC_2_SHIFT, 0);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_LEDD,
		AP3223_REG_PS_LEDD_MASK, AP3223_REG_PS_LEDD_SHIFT, 1);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_MEAN,
		AP3223_REG_PS_MEAN_MASK, AP3223_REG_PS_MEAN_SHIFT, 0);
	ret = __ap3223_write_reg(client, AP3223_REG_PS_PERSIS,
		AP3223_REG_PS_PERSIS_MASK, AP3223_REG_PS_PERSIS_SHIFT, 0);
	ret = ap3223_set_plthres(client, 0);
	ret = ap3223_set_phthres(client, 535);
	ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
		AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_PS_ENABLE);
    }else if(en == 0){
	ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
		AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_DOWN);
	mdelay(200);
    }

    return ret;
}
/* mode */
static int ap3223_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3223_read_reg(client, AP3223_REG_SYS_CONF,
	    AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3223_set_mode(struct i2c_client *client, int mode)
{
    int ret;

    ret = __ap3223_write_reg(client, AP3223_REG_SYS_CONF,
	    AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, mode);

    return ret;
}

/* ALS low threshold */
static int ap3223_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3223_read_reg(client, AP3223_REG_ALS_THDL_L,
	    AP3223_REG_ALS_THDL_L_MASK, AP3223_REG_ALS_THDL_L_SHIFT);
    msb = __ap3223_read_reg(client, AP3223_REG_ALS_THDL_H,
	    AP3223_REG_ALS_THDL_H_MASK, AP3223_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3223_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3223_REG_ALS_THDL_L_MASK;

    err = __ap3223_write_reg(client, AP3223_REG_ALS_THDL_L,
	    AP3223_REG_ALS_THDL_L_MASK, AP3223_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3223_write_reg(client, AP3223_REG_ALS_THDL_H,
	    AP3223_REG_ALS_THDL_H_MASK, AP3223_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3223_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3223_read_reg(client, AP3223_REG_ALS_THDH_L,
	    AP3223_REG_ALS_THDH_L_MASK, AP3223_REG_ALS_THDH_L_SHIFT);
    msb = __ap3223_read_reg(client, AP3223_REG_ALS_THDH_H,
	    AP3223_REG_ALS_THDH_H_MASK, AP3223_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3223_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3223_REG_ALS_THDH_L_MASK;

    err = __ap3223_write_reg(client, AP3223_REG_ALS_THDH_L,
	    AP3223_REG_ALS_THDH_L_MASK, AP3223_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3223_write_reg(client, AP3223_REG_ALS_THDH_H,
	    AP3223_REG_ALS_THDH_H_MASK, AP3223_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3223_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3223_read_reg(client, AP3223_REG_PS_THDL_L,
	    AP3223_REG_PS_THDL_L_MASK, AP3223_REG_PS_THDL_L_SHIFT);
    msb = __ap3223_read_reg(client, AP3223_REG_PS_THDL_H,
	    AP3223_REG_PS_THDL_H_MASK, AP3223_REG_PS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3223_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3223_REG_PS_THDL_L_MASK;

    err = __ap3223_write_reg(client, AP3223_REG_PS_THDL_L,
	    AP3223_REG_PS_THDL_L_MASK, AP3223_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3223_write_reg(client, AP3223_REG_PS_THDL_H,
	    AP3223_REG_PS_THDL_H_MASK, AP3223_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

/* PX high threshold */
static int ap3223_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3223_read_reg(client, AP3223_REG_PS_THDH_L,
	    AP3223_REG_PS_THDH_L_MASK, AP3223_REG_PS_THDH_L_SHIFT);
    msb = __ap3223_read_reg(client, AP3223_REG_PS_THDH_H,
	    AP3223_REG_PS_THDH_H_MASK, AP3223_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3223_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3223_REG_PS_THDH_L_MASK;

    err = __ap3223_write_reg(client, AP3223_REG_PS_THDH_L,
	    AP3223_REG_PS_THDH_L_MASK, AP3223_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3223_write_reg(client, AP3223_REG_PS_THDH_H,
	    AP3223_REG_PS_THDH_H_MASK, AP3223_REG_PS_THDH_H_SHIFT, msb);

    return err;
}

/* Ps calibration reg */
static int ap3223_get_pscalibration(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3223_read_reg(client, AP3223_REG_PS_CAL_L,
	    AP3223_REG_PS_THDH_L_MASK, AP3223_REG_PS_THDH_L_SHIFT);
    msb = __ap3223_read_reg(client, AP3223_REG_PS_CAL_H,
	    AP3223_REG_PS_THDH_H_MASK, AP3223_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3223_set_pscalibration(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3223_REG_PS_THDH_L_MASK;

    err = __ap3223_write_reg(client, AP3223_REG_PS_CAL_L,
	    AP3223_REG_PS_THDH_L_MASK, AP3223_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3223_write_reg(client, AP3223_REG_PS_CAL_H,
	    AP3223_REG_PS_THDH_H_MASK, AP3223_REG_PS_THDH_H_SHIFT, msb);

    return err;
}

static int ap3223_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif

    lsb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_HIGH);

    if (msb < 0)
	return msb;

    range = ap3223_get_range(client);

    tmp = (((msb << 8) | lsb) * range) >> 16;
    val = msb << 8 | lsb;
    return val;
}

static int ap3223_get_lux_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif

    lsb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, AP3223_REG_ALS_DATA_HIGH);

    if (msb < 0)
	return msb;

    range = ap3223_get_range(client);

    tmp = (((msb << 8) | lsb) * range) >> 16;
    tmp = tmp * cali / 100;
    val = msb << 8 | lsb;

    val = ( val * asus_calibration_lux ) / cali;
    return val;
}


static int ap3223_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3223_OBJ_COMMAND);
    val &= AP3223_OBJ_MASK;

    return val >> AP3223_OBJ_SHIFT;
}

static int ap3223_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3223_REG_SYS_INTSTATUS);
    val &= AP3223_REG_SYS_INT_MASK;

    return val >> AP3223_REG_SYS_INT_SHIFT;
}


static int ap3223_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, AP3223_REG_PS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    //LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = i2c_smbus_read_byte_data(client, AP3223_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

    //LDBG("%s, IR = %d\n", __func__, (u32)(msb));

    return (u32)(((msb & AL3223_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3223_REG_PS_DATA_LOW_MASK));
}




static int ap3223_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3223_get_mode(client);
    if((mode & AP3223_SYS_ALS_ENABLE) == 0){
	mode |= AP3223_SYS_ALS_ENABLE;
	ret = ap3223_set_mode(client,mode);
    }

    return ret;
}

static int ap3223_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3223_get_mode(client);
    if(mode & AP3223_SYS_ALS_ENABLE){
	mode &= ~AP3223_SYS_ALS_ENABLE;
	if(mode == AP3223_SYS_DEV_RESET)
	    mode = 0;
	ret = ap3223_set_mode(client,mode);
    }

    return ret;
}

static int ap3223_register_lsensor_device(struct i2c_client *client, struct ap3223_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "lightsensor-level";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}


static void ap3223_unregister_lsensor_device(struct i2c_client *client, struct ap3223_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}

#ifdef HEARBEAT
static int ap3223_register_heartbeat_sensor_device(struct i2c_client *client, struct ap3223_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device heartbeat sensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for heartbeat sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->hsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "heartbeat";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_REL, input_dev->evbit);
    input_set_capability(input_dev, EV_REL, ABS_WHEEL);
    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void ap3223_unregister_heartbeat_device(struct i2c_client *client, struct ap3223_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}
#endif

static int ap3223_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3223_get_mode(client);
    if((mode & AP3223_SYS_PS_ENABLE) == 0){
	mode |= AP3223_SYS_PS_ENABLE;
	ret = ap3223_set_mode(client,mode);
    }

    return ret;
}

static int ap3223_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3223_get_mode(client);
    if(mode & AP3223_SYS_PS_ENABLE){
	mode &= ~AP3223_SYS_PS_ENABLE;
	if(mode == AP3223_SYS_DEV_RESET)
	    mode = AP3223_SYS_DEV_DOWN;
	ret = ap3223_set_mode(client,mode);
    }
    return ret;
}


static int ap3223_register_psensor_device(struct i2c_client *client, struct ap3223_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    return 0;

done:
    return rc;
}

static void ap3223_unregister_psensor_device(struct i2c_client *client, struct ap3223_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3223_early_suspend;
static void ap3223_suspend(struct early_suspend *h)
{

    if (misc_ps_opened)
	ap3223_psensor_disable(ap3223_data_g->client);
    if (misc_ls_opened)
	ap3223_lsensor_disable(ap3223_data_g->client);
}

static void ap3223_resume(struct early_suspend *h)
{

    if (misc_ls_opened)
	ap3223_lsensor_enable(ap3223_data_g->client);
    if (misc_ps_opened)
	ap3223_psensor_enable(ap3223_data_g->client);
}
#endif

/*- asus read factory data -*/

static int asus_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	strcpy(buf2, buf);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}


static int asus_read_calibration_data(void)
{
    char buf[256];
    int calibration_value = 0;
    struct file *fp = NULL;
    int count = 0;
    mm_segment_t oldfs;
    int stage = 0; 
    
    int hardward_id = asustek_boardinfo_get(FUN_HARDWARE_ID) ;
    
    if (hardward_id >=prePR && hardward_id<=PR)
		stage = 4;
    else if (hardward_id >=prePR && hardward_id==MP)
		stage = 5;
    else
		stage = 2;


    /* update light cali from factory partition */
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(LIGHTSENSOR_CALIBRATION_DATA, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            cali = calibration_value;
        	LDBG("update light cal = %d \n",calibration_value);
        }else{
        	LDBG("wrong light cal = %d , use default cal %d \n",calibration_value,asus_cali_value[stage][0]);
            cali = asus_cali_value[stage][0];
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
    }else{
        /* get als calibration value failed use default value*/
        cali = asus_cali_value[stage][0];
    	LDBG("update light cal fail ,err = %d use default value %d \n",IS_ERR(fp),cali);
    	count ++ ;
    }


    /* update crosstalk from factory partition */
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(XTALK_CALIBRATION_DATA, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value >= 0){
            asus_ps_xtalk = calibration_value;
        	LDBG("update xtalk cal = %d \n",calibration_value);
        }else{
            asus_ps_xtalk = asus_cali_value[stage][1];
        	LDBG("wrong xtalk cal = %d , use default cal %d \n",calibration_value,asus_cali_value[stage][1]);
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
    }else{
    	LDBG("update xtalk cal fail , err = %d use default value %d \n",IS_ERR(fp),asus_cali_value[stage][1]);
        asus_ps_xtalk = asus_cali_value[stage][1];
        count++;
    }



	/* update HTH from factory partition */
	oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(HTH_CALIBRATION_DATA, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            asus_ps_thH = calibration_value;
        	LDBG("update thr high cal = %d \n",calibration_value);
        }else{
            asus_ps_thH = asus_cali_value[stage][3];
        	LDBG("wrong thr high cal = %d , use default value %d \n",calibration_value,asus_cali_value[stage][3]);
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
    }else{
    	asus_ps_thH = asus_cali_value[stage][3];
    	LDBG("update thr high cal fail ,  err = %d use default value %d \n",IS_ERR(fp),asus_cali_value[stage][3]);
        count++;
    }


	/* update LTH from factory partition */
    oldfs=get_fs();
    set_fs(get_ds());
    memset(buf, 0, sizeof(u8)*256);
    fp=filp_open(LTH_CALIBRATION_DATA, O_RDONLY, 0);
    if (!IS_ERR(fp)) {
        int ret = 0;
        ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
        sscanf(buf,"%d\n", &calibration_value);
        if(calibration_value > 0){
            asus_ps_thL = calibration_value;
            LDBG("update thr low cal = %d \n",calibration_value);
        }else{
            asus_ps_thL = asus_cali_value[stage][2];
        	LDBG("wrong thr low cal = %d , use default value %d \n",calibration_value,asus_cali_value[stage][2]);
        }
        filp_close(fp, NULL);
        set_fs(oldfs);
    }else{
    	asus_ps_thL = asus_cali_value[stage][2];
    	LDBG("update thr low cal fail ,  err = %d use default value %d \n",IS_ERR(fp),asus_cali_value[stage][2]);
        count++;
    }


    if (count > 0)
    	return -1;
    else
    	return 1;
}


/* asus init function */
static int asus_init_threshold(struct i2c_client *client)
{
    struct ap3223_data *data = i2c_get_clientdata(client);
    int ret = 0 ;

    /* init integrated time from ER factory */
    ret = i2c_smbus_write_byte_data(data->client, AP3223_REG_PS_INTEGR, asus_ps_integrated_time_ER);
    if (ret < 0)
    	LDBG("init integrated time ER failed ret = %d \n",ret);

    /* disable light sensor INT mode */
    ret = i2c_smbus_write_byte_data(data->client, AP3223_REG_SYS_INTCTRL, 0x80);
    if (ret < 0)
    	LDBG("disable light sensor INT mode failed ret = %d \n",ret);

    /*init ls gain*/
    if(asustek_boardinfo_get(FUN_HARDWARE_ID) >= prePR ){
    	asuscali = 300; //default offset for 16383 lux/resolution
        ret = ap3223_set_range(data->client, asus_ls_gain);
            if (ret < 0)
            	LDBG("ap3223_set_range failed ret = %d \n",ret);
    }


    /*init calibration data*/
    ret = asus_read_calibration_data();
    if (ret < 0)
    	LDBG("asus_read_calibration_data failed ret = %d \n",ret);

    /*init ps Hight/Low threshold*/
    ret = ap3223_set_plthres(data->client, asus_ps_thL);
    if (ret < 0)
    	LDBG("ap3223_set_plthres failed ret = %d \n",ret);


    ret = ap3223_set_phthres(data->client, asus_ps_thH);
    if (ret < 0)
    	LDBG("ap3223_set_phthres failed ret = %d \n",ret);

    /*init ps xtalk threshold*/
    ret = ap3223_set_pscalibration(data->client, asus_ps_xtalk);
    if (ret < 0)
    	LDBG("ap3223_set_pscalibration failed ret = %d \n",ret);


#ifdef LIGHT_POLLING
    /* init als Hight/Low threshold
     * if we use polling mode,try to config
     * not to trigger INT
     */
    ret = ap3223_set_althres(data->client, 0);
    if (ret < 0)
    	LDBG("ap3223_set_althres failed ret = %d \n",ret);

    ret = ap3223_set_ahthres(data->client, 0);
    if (ret < 0)
    	LDBG("ap3223_set_ahthres failed ret = %d \n",ret);
#else
    /*init als Hight/Low threshold*/
    ret = ap3223_set_althres(data->client, asus_al_thL);
    if (ret < 0)
    	LDBG("ap3223_set_althres failed ret = %d \n",ret);

    ret = ap3223_set_ahthres(data->client, asus_al_thH);
    if (ret < 0)
    	LDBG("ap3223_set_ahthres failed ret = %d \n",ret);
#endif


    return ret;

}


/* range */
static ssize_t ap3223_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%i\n", ap3223_get_range(data->client));
}

static ssize_t ap3223_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3223_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}




static ssize_t ap3223_store_ir_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_ir_data(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}
static ssize_t ap3223_show_ir_data(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    return 0;
}
/* mode */
static ssize_t ap3223_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_mode(data->client));
}

static ssize_t ap3223_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_mode(data->client, val);

    if (ret < 0)
	return ret;
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}


static ssize_t ap3223_ls_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
    	return -EINVAL;
    mutex_lock(&ap3223_ls_lock);
	int REG_SYS_CONF_STATE = ap3223_get_mode(data->client);
    if((mode == AP3223_SYS_ALS_ENABLE) && ap3223_get_mode(data->client) != AP3223_SYS_ALS_ENABLE) {
    	ret = asus_init_threshold(data->client);
    	ap3223_set_althres(data->client, asus_al_thL);
    	ap3223_set_ahthres(data->client, asus_al_thH);
    	misc_ls_opened = 1;

    	ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
    			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, REG_SYS_CONF_STATE |= 1 << 0);
    	if (ret < 0)
    		return ret;
#ifdef LIGHT_POLLING
    	queue_delayed_work(data->light_wq, &data->light_work,msecs_to_jiffies(L_TIMER_DELAY));
#endif
    } else {
    	if(REG_SYS_CONF_STATE == AP3223_SYS_ALS_ENABLE) // only lsensor is enabled
			ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
				AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_RESET);

		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, REG_SYS_CONF_STATE &= ~(1 << 0));
#ifdef LIGHT_POLLING
		if(&data->light_work)
			cancel_delayed_work_sync(&data->light_work);
#endif
    }
    mutex_unlock(&ap3223_ls_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
    	LDBG("Timer Error\n");
#endif
    return count;
}


static ssize_t ap3223_ps_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
    	return -EINVAL;

    mutex_lock(&ap3223_ps_lock);
	int REG_SYS_CONF_STATE = ap3223_get_mode(data->client);
    if((mode == AP3223_SYS_PS_ENABLE ) && ap3223_get_mode(data->client) != AP3223_SYS_PS_ENABLE) {
    	ret = asus_init_threshold(data->client);
		ret = ap3223_set_plthres(data->client, asus_ps_thL);
		ret = ap3223_set_phthres(data->client, asus_ps_thH);
		misc_ps_opened = 1;
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT,  REG_SYS_CONF_STATE |= 1 << 1);
		if (ret < 0)
			return ret;

		msleep(5);
		int obj = 0 ;
		if( ap3223_get_px_value(data->client) > asus_ps_thH)
		        obj = 1;
		else if(ap3223_get_px_value(data->client) < asus_ps_thL)
		        obj = 0;
		input_report_abs(data->psensor_input_dev, ABS_DISTANCE, obj);
		input_sync(data->psensor_input_dev);
    } else {
    	if(REG_SYS_CONF_STATE == AP3223_SYS_PS_ENABLE) // only psensor is enabled
			ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
				AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_RESET);

		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, REG_SYS_CONF_STATE &= ~(1 << 1));
    }
    mutex_unlock(&ap3223_ps_lock);


#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}

#ifdef HEARBEAT
static ssize_t ap3223_hs_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long mode;
    int ret;

    LDBG("mode = %s,%s\n", __func__,buf);
    if (strict_strtoul(buf, 10, &mode) < 0)
    	return -EINVAL;

    mutex_lock(&ap3223_heartbeat_lock);


    if(mode == 9) {
		data-> hsensor_enable = 1;
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_CONF,
			AP3223_REG_PS_CONF_MASK, AP3223_REG_PS_CONF_SHIFT, 0);
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_DC_1,
			AP3223_REG_PS_DC_1_MASK, AP3223_REG_PS_DC_1_SHIFT, 0);
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_DC_2,
			AP3223_REG_PS_DC_2_MASK, AP3223_REG_PS_DC_2_SHIFT, 0);
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_LEDD,
			AP3223_REG_PS_LEDD_MASK, AP3223_REG_PS_LEDD_SHIFT, 1);
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_MEAN,
			AP3223_REG_PS_MEAN_MASK, AP3223_REG_PS_MEAN_SHIFT, 0);
		ret = __ap3223_write_reg(data->client, AP3223_REG_PS_PERSIS,
			AP3223_REG_PS_PERSIS_MASK, AP3223_REG_PS_PERSIS_SHIFT, 0);
		ret = ap3223_set_plthres(data->client, 0);
		ret = ap3223_set_phthres(data->client, 535);
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_PS_ENABLE);

		if (ret < 0)
			return ret;
    } else {
		data-> hsensor_enable = 0;
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_RESET);
		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_CONF,
			AP3223_REG_SYS_CONF_MASK, AP3223_REG_SYS_CONF_SHIFT, AP3223_SYS_DEV_DOWN);
    }
    mutex_unlock(&ap3223_heartbeat_lock);
#if POLLING_MODE
    LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
#endif
    return count;
}
#endif

/* light_lux */
static ssize_t ap3223_show_als_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;

    /* No LUX data if power down */
    if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3223_get_lux_value(data->client));
}

/* light_adc raw*/
static ssize_t ap3223_show_als_adc(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;

    /* No LUX data if power down */
    if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3223_get_adc_value(data->client));
}



/* Px data */
static ssize_t ap3223_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;

    /* No Px data if power down */
    if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3223_get_px_value(data->client));
}



/* proximity object detect */
static ssize_t ap3223_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_object(data->client));
}



/* ALS low threshold */
static ssize_t ap3223_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_althres(data->client));
}

static ssize_t ap3223_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* ALS high threshold */
static ssize_t ap3223_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_ahthres(data->client));
}

static ssize_t ap3223_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* Px low threshold */
static ssize_t ap3223_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_plthres(data->client));
}

static ssize_t ap3223_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}


/* Px high threshold */
static ssize_t ap3223_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_phthres(data->client));
}

static ssize_t ap3223_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

/* Ps calibration reg */
static ssize_t ap3223_show_pscalibration(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    return sprintf(buf, "%d\n", ap3223_get_pscalibration(data->client));
}

static ssize_t ap3223_store_pscalibration(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3223_set_pscalibration(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}



/* calibration */
static ssize_t ap3223_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3223_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    int val, lux;
    char tmp[10];

    LDBG("DEBUG ap3223_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3223_get_mode(data->client) == AP3223_SYS_DEV_DOWN)
    {
    	printk("Please power up first!");
    	return -EINVAL;
    }

//    sscanf(buf, "%d %s", &stdls, tmp);
//
//    if (!strncmp(tmp, "-setcv", 6))
//    {
//	cali = stdls;
//	return -EBUSY;
//    }
//
//    if (stdls < 0)
//    {
//	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
//		Set calibration factor to 100.\n", stdls);
//	return -EBUSY;
//    }
//
//    lux = ap3223_get_adc_value(data->client);
//    cali = stdls * 100 / lux;
    if (strict_strtoul(buf, 10, &val) < 0)
    	return -EINVAL;
    LDBG("DEBUG ap3223_store_calibration_state..%d\n",val);

    cali = val;

    return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3223_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    int i;
    u8 tmp;

    LDBG("DEBUG ap3223_em_read..\n");

    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap3223_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3223_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
#endif

#ifdef LIGHT_POLLING
static ssize_t ap3223_als_set_delay(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long temp;
    int delay_time;
    if (strict_strtol(buf, 10, &temp))
        return -EINVAL;
    delay_time = (int)temp /1000000; //ns to ms
    LDBG("ap3223_als_set_delay delay_time = %d\n",delay_time);
    if(delay_time < ap3223_als_min_poll_time){
    	L_TIMER_DELAY = ap3223_als_min_poll_time;
    }else{
    	L_TIMER_DELAY = delay_time;
    }
    LDBG("als_poll_time = %d\n",L_TIMER_DELAY);
    return strnlen(buf, count);
}

static ssize_t ap3223_als_get_delay(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", L_TIMER_DELAY);
}
#endif

#ifdef PROX_POLLING
static ssize_t ap3223_ps_set_delay(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
    long temp;
    int delay_time;
    if (strict_strtol(buf, 10, &temp))
        return -EINVAL;
    delay_time = (int)temp /1000000; //ns to ms
    printk("ap3223_ps_set_delay delay_time = %d\n",delay_time);
    if( delay_time < ap3223_ps_min_poll_time){
    	PL_TIMER_DELAY = ap3223_ps_min_poll_time;
    }else{
    	PL_TIMER_DELAY = delay_time;
    }
    printk("ap3223_ps_poll_time = %d\n",PL_TIMER_DELAY);
    return strnlen(buf, count);
}

static ssize_t ap3223_ps_get_delay(struct device *dev ,
                                 struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", PL_TIMER_DELAY);
}
#endif

static ssize_t ap3223_als_status(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int write_i2c_err = 0;
    int read_i2c_err = 0;
    int i;
    int retry_time = 5;

    struct ap3223_data *data = ap3223_data_g;

    /*- enable sensor -*/
    for(i=0;i<retry_time;i++){
        write_i2c_err = ap3223_ls_enable(dev,attr,"1",0);
        if( write_i2c_err<0 ){
            printk("check als write i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    for(i=0;i<retry_time;i++){
        read_i2c_err = ap3223_get_adc_value(data->client);
        if(read_i2c_err<0){
            printk("check als read i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }
    /*- disable sensor -*/
    //TODO read enable status and set enable mode
    ap3223_ls_enable(dev,attr,"0",0);

    if(write_i2c_err<0 || read_i2c_err<0){
        return sprintf(buf, "%d\n", 0);
    }else{
        return sprintf(buf, "%d\n", 1);
    }
}

static ssize_t ap3223_ps_status(struct device *dev,
                 struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    int write_i2c_err = 0;
    int read_i2c_err = 0;
    int i;
    int retry_time = 5;

    struct ap3223_data *data = ap3223_data_g;

    for(i=0;i<retry_time;i++){
        write_i2c_err = ap3223_ps_enable(dev,attr,"2",0);
        if( write_i2c_err<0 ){
            printk("ap3223 , check ps write i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    for(i=0;i<retry_time;i++){
        read_i2c_err = ap3223_get_px_value(data->client);
        if(read_i2c_err<0){
            printk("ap3223 , check ps read i2c status retry i=%d\n",i);
        }else{
            i=retry_time;
        }
    }

    /*- disable sensor -*/
    //TODO read enable status and set enable mode

    ap3223_ps_enable(dev,attr,"0",0);
    if(write_i2c_err<0 || read_i2c_err<0){
        return sprintf(buf, "%d\n", 0);
    }else{
        return sprintf(buf, "%d\n", 1);
    }
}

static ssize_t ap3223_als_enable_disable_poll_event(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
//    long enable;
//    printk("ap3223 : %s\n",__func__);
//    if (strict_strtol(buf, 10, &enable))
//        return -EINVAL;
//    if ((enable != 1) && (enable != 0))
//        return -EINVAL;
//
//    if(enable == 1){
//        printk("ap3223 : ap3223 als poll enable\n");
//        printk("ap3223 : ap3223 first event no wait \n");
//        ap3223_als_flag_pollin = true;
//        queue_delayed_work(ap3223_als_poll_work_queue, &ap3223_als_report_poll_event_work, 0);
//    }else{
//        printk("ap3223 : ap3223 als poll disable\n");
//        cancel_delayed_work_sync(&ap3223_als_report_poll_event_work);
//    }
//    return strnlen(buf, count);
	return 1;
}

static ssize_t ap3223_ps_enable_disable_poll_event(struct device *dev, struct device_attribute *attr,
        const char *buf , size_t count){
//    long enable;
//    printk("ap3323 : %s\n",__func__);
//    if (strict_strtol(buf, 10, &enable))
//        return -EINVAL;
//    if ((enable != 1) && (enable != 0))
//        return -EINVAL;
//
//    if(enable == 1){
//        printk("ap3323 : ap3323 ps poll enable\n");
//        ap3323_ps_init_poll_count = 0;
//        ap3323_ps_irq_data_count = PS_INIT_POLL_MAX_COUNT;//+++ add 10 count for init polling
//        ap3323_ps_flag_pollin = true;
//        queue_delayed_work(ap3323_ps_poll_work_queue, &ap3323_ps_report_poll_event_work, msecs_to_jiffies(0));
//        ap3323_ps_enabled = true;
//    }else{
//        printk("ap3323 : ap3323 ps poll disable\n");
//        cancel_delayed_work_sync(&ap3323_ps_report_poll_event_work);
//        ap3323_ps_enabled = false;
//    }
//    return strnlen(buf, count);
	return 1 ;
}

/* update calibration from nvm*/
static ssize_t ap3223_update_cali_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct ap3223_data *data = ap3223_data_g;
    int ret = 0;
    ret = asus_init_threshold(data->client);

    if (ret < 0)
        return sprintf(buf, "%d\n", 0);
    else
        return sprintf(buf, "%d\n", 1);
}

static ssize_t ap3223_update_cali_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct ap3223_data *data = ap3223_data_g;

    return 1;
}


static struct device_attribute attributes[] = {
    __ATTR(range, S_IWUSR | S_IRUGO, ap3223_show_range, ap3223_store_range),
    __ATTR(mode, 0664, ap3223_show_mode, ap3223_store_mode),
    __ATTR(lsensor, 0664, ap3223_show_mode, ap3223_ls_enable),
    __ATTR(psensor, 0664, ap3223_show_mode, ap3223_ps_enable),
#ifdef HEARBEAT
    __ATTR(hsensor, 0664, ap3223_show_mode, ap3223_hs_enable),
#endif
    __ATTR(lux, S_IRUGO, ap3223_show_als_lux, NULL),
    __ATTR(pxvalue, S_IRUGO, ap3223_show_pxvalue, NULL),
    __ATTR(object, S_IRUGO, ap3223_show_object, NULL),
    __ATTR(althres, S_IWUSR | S_IRUGO, ap3223_show_althres, ap3223_store_althres),
    __ATTR(ahthres, S_IWUSR | S_IRUGO, ap3223_show_ahthres, ap3223_store_ahthres),
    __ATTR(plthres, S_IWUSR | S_IRUGO, ap3223_show_plthres, ap3223_store_plthres),
    __ATTR(phthres, S_IWUSR | S_IRUGO, ap3223_show_phthres, ap3223_store_phthres),
    __ATTR(calibration, S_IWUSR | S_IRUGO, ap3223_show_calibration_state, ap3223_store_calibration_state),
    __ATTR(ir_data, 0664, ap3223_show_ir_data, ap3223_store_ir_data),
#ifdef LSC_DBG
    __ATTR(em, S_IWUSR | S_IRUGO, ap3223_em_read, ap3223_em_write),
#endif

};

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO, ap3223_show_range, ap3223_store_range);
static DEVICE_ATTR(mode, 0664, ap3223_show_mode, ap3223_store_mode);
static DEVICE_ATTR(lsensor, 0664, ap3223_show_mode, ap3223_ls_enable);
static DEVICE_ATTR(psensor, 0664, ap3223_show_mode, ap3223_ps_enable);
#ifdef HEARBEAT
    static DEVICE_ATTR(hsensor, 0664, ap3223_show_mode, ap3223_hs_enable);
#endif
static DEVICE_ATTR(als_lux, S_IRUGO, ap3223_show_als_lux, NULL);
static DEVICE_ATTR(als_adc, S_IRUGO, ap3223_show_als_adc, NULL);
static DEVICE_ATTR(pxvalue, S_IRUGO, ap3223_show_pxvalue, NULL);
static DEVICE_ATTR(object, S_IRUGO, ap3223_show_object, NULL);
static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO, ap3223_show_althres, ap3223_store_althres);
static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO, ap3223_show_ahthres, ap3223_store_ahthres);
static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO, ap3223_show_plthres, ap3223_store_plthres);
static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO, ap3223_show_phthres, ap3223_store_phthres);
static DEVICE_ATTR(pscalibration, S_IWUSR | S_IRUGO, ap3223_show_pscalibration, ap3223_store_pscalibration);
static DEVICE_ATTR(lscalibration, S_IWUSR | S_IRUGO, ap3223_show_calibration_state, ap3223_store_calibration_state);
static DEVICE_ATTR(ir_data, 0664, ap3223_show_ir_data, ap3223_store_ir_data);
#ifdef LSC_DBG
static DEVICE_ATTR(em, S_IWUSR | S_IRUGO, ap3223_em_read, ap3223_em_write);
#endif
#ifdef LIGHT_POLLING
static DEVICE_ATTR(als_poll_time,0664,ap3223_als_get_delay,ap3223_als_set_delay);
#endif
#ifdef PROX_POLLING
static DEVICE_ATTR(ps_poll_time,0664,ap3223_ps_get_delay,ap3223_ps_set_delay);
#endif
static DEVICE_ATTR(als_status,0664,ap3223_als_status,ap3223_als_enable_disable_poll_event);
static DEVICE_ATTR(ps_status,0664,ap3223_ps_status,ap3223_ps_enable_disable_poll_event);
static DEVICE_ATTR(update_cali,0664,ap3223_update_cali_show,ap3223_update_cali_store);



static struct attribute *ap3223_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_mode.attr,
    &dev_attr_lsensor.attr,
    &dev_attr_psensor.attr,
#ifdef HEARBEAT
    &dev_attr_hsensor.attr,
#endif
    &dev_attr_als_lux.attr,
    &dev_attr_als_adc.attr,
    &dev_attr_pxvalue.attr,
    &dev_attr_object.attr,
    &dev_attr_althres.attr,
    &dev_attr_ahthres.attr,
    &dev_attr_plthres.attr,
    &dev_attr_phthres.attr,
    &dev_attr_pscalibration.attr,
    &dev_attr_lscalibration.attr,
    &dev_attr_ir_data.attr,
#ifdef LSC_DBG
    &dev_attr_em.attr,
#endif
#ifdef LIGHT_POLLING
    &dev_attr_als_poll_time.attr,
#endif
#ifdef PROX_POLLING
    &dev_attr_ps_poll_time.attr,
#endif
    &dev_attr_als_status.attr,
    &dev_attr_ps_status.attr,
    &dev_attr_update_cali.attr,
    NULL
};

static const struct attribute_group ap3223_attr_group = {
    .attrs = ap3223_attributes,
};

static int create_sysfs_interfaces(struct ap3223_data *sensor)
{
    int i;
    struct class *ap3223_class = NULL;
    struct device *ap3223_dev = NULL;
    int ret;

    ap3223_class = class_create(THIS_MODULE, "sensors");
    if (IS_ERR(ap3223_class)) {
    	ret = PTR_ERR(ap3223_class);
    	ap3223_class = NULL;
    	LDBG("%s: could not allocate ap3223_class, ret = %d\n", __func__, ret);
    	goto ap3223_class_error;
    }

    ap3223_dev= device_create(ap3223_class,
	    NULL, 0, "%s", "di_sensors");

    if(ap3223_dev == NULL)
    	goto ap3223_device_error;

    for (i = 0; i < ARRAY_SIZE(attributes); i++){
    	if (device_create_file(ap3223_dev, attributes + i))
    		goto ap3223_create_file_error;
    }
    return 0;

ap3223_create_file_error:
    for ( ; i >= 0; i--)
	device_remove_file(ap3223_dev, attributes + i);

ap3223_device_error:
    class_destroy(ap3223_class);
ap3223_class_error:
    dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
    return -1;
}
static int ap3223_init_client(struct i2c_client *client)
{
    struct ap3223_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG ap3223_init_client..\n");

    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);

	if (v < 0)
	    return -ENODEV;

	data->reg_cache[i] = v;
    }
    /* set defaults */

    ap3223_set_range(client, AP3223_ALS_RANGE_0);

    ap3223_set_mode(data->client, AP3223_SYS_DEV_DOWN);


    return 0;
}

#if POLLING_MODE
static void pl_timer_callback(unsigned long pl_data)
{
    struct ap3223_data *data;
    int ret =0;

    data = ap3223_data_g;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&ap3223_data_g->pl_timer, jiffies + usecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
    	LDBG("Timer Error\n");

}
#endif
#ifdef LIGHT_POLLING
static void lsensor_work_handler(struct work_struct *w)
{
    struct ap3223_data *data =	container_of(w, struct ap3223_data, light_work);
    int ret;
    int value;

	value = ap3223_get_lux_value(data->client);
	ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_INTSTATUS,
			AP3223_REG_SYS_INT_AMASK, AP3223_REG_SYS_INT_LS_SHIFT, 0);
	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
	input_sync(data->lsensor_input_dev);

    queue_delayed_work(data->light_wq, &data->light_work, msecs_to_jiffies(L_TIMER_DELAY));
}
#endif

static void plsensor_work_handler(struct work_struct *w)
{

    struct ap3223_data *data =	container_of(w, struct ap3223_data, plsensor_work);
    u8 int_stat;
    int pxvalue;
    int obj;
    int ret;
    int value;

    int_stat = ap3223_get_intstat(data->client);

#ifndef LIGHT_POLLING
    // ALS int
    if (int_stat & AP3223_REG_SYS_INT_AMASK)
    {
//	LDBG("LS INT Status: %0x\n", int_stat);

    	value = ap3223_get_lux_value(data->client);
    	ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_INTSTATUS,
    			AP3223_REG_SYS_INT_AMASK, AP3223_REG_SYS_INT_LS_SHIFT, 0);
    	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
    	input_sync(data->lsensor_input_dev);
    }
#endif
    // PX int
    if (int_stat & AP3223_REG_SYS_INT_PMASK)
    {
		LDBG("PS INT Status: %0x\n", int_stat);
		obj = ap3223_get_object(data->client);
		pxvalue = ap3223_get_px_value(data->client);

		ret = __ap3223_write_reg(data->client, AP3223_REG_SYS_INTSTATUS,
			AP3223_REG_SYS_INT_PMASK, AP3223_REG_SYS_INT_PS_SHIFT, 0);
		LDBG("%s\n", obj ? "obj near":"obj far");
		input_report_abs(data->psensor_input_dev, ABS_DISTANCE, obj);
		input_sync(data->psensor_input_dev);
#ifdef HEARBEAT
		if(data->hsensor_enable) {
			LDBG("pxvalue = %d\n", pxvalue);
			input_report_rel(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
			input_sync(data->hsensor_input_dev);
		}
#endif
    }

    enable_irq(data->client->irq);
}
/*
 * I2C layer
 */

static irqreturn_t ap3223_irq(int irq, void *data_)
{
    struct ap3223_data *data = data_;

//    LDBG("interrupt\n");
    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &data->plsensor_work);

    return IRQ_HANDLED;
}

static int ap3223_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3223_data *data;
    int err = 0;

    LDBG("ap3223_probe\n");

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
    	err = -EIO;
    	goto exit_free_gpio;
    }

    reg_array = ap3223_reg;
    range = ap3223_range;
    reg_num = AP3223_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct ap3223_data), GFP_KERNEL);
    if (!data){
    	err = -ENOMEM;
		goto exit_free_gpio;
    }

    data->gpio = of_get_named_gpio_flags(client->dev.of_node, OF_PSENSOR_PIN, 0, NULL);
    client->irq = irq_of_parse_and_map(client->dev.of_node,0);

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;

    /* initialize the AP3223 chip */
    err = ap3223_init_client(client);
    if (err){
    	dev_err(&client->dev, "failed to init_client\n");
    	goto exit_kfree;
    }

    err = ap3223_register_lsensor_device(client,data);
    if (err){
    	dev_err(&client->dev, "failed to register_lsensor_device\n");
    	goto exit_kfree;
    }

    err = ap3223_register_psensor_device(client, data);
    if (err) {
    	dev_err(&client->dev, "failed to register_psensor_device\n");
    	goto exit_free_ls_device;
    }
#ifdef HEARBEAT
    err = ap3223_register_heartbeat_sensor_device(client, data);
    if (err) {
    	dev_err(&client->dev, "failed to register_heartbeatsensor_device\n");
    	goto exit_free_heartbeats_device;
    }
#endif
//    err = create_sysfs_interfaces(data);
    err = sysfs_create_group(&client->dev.kobj, &ap3223_attr_group);
    if (err){
    	dev_err(&client->dev, "failed to register sysfs_create_group\n");
    	goto exit_free_ps_device;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    ap3223_early_suspend.suspend = ap3223_suspend;
    ap3223_early_suspend.resume  = ap3223_resume;
    ap3223_early_suspend.level   = 0x02;
    register_early_suspend(&ap3223_early_suspend);
#endif

    err = gpio_request(data->gpio,"ALS_INT#");
    if(err){
    	dev_err(&client->dev, "err: %d, request gpio %d ALS_INT# fail!!!\n",err,data->gpio);
    }

    err = request_irq(client->irq,ap3223_irq,
	    IRQF_TRIGGER_FALLING,
	    "ap3223", data);
    if (err) {
    	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
    	goto exit_free_ps_device;
    }

    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
    if (!data->plsensor_wq) {
    	LDBG("%s: create workqueue failed\n", __func__);
    	err = -ENOMEM;
    	goto err_create_wq_failed;
    }

    INIT_WORK(&data->plsensor_work, plsensor_work_handler);

#ifdef LIGHT_POLLING
    data->light_wq = create_singlethread_workqueue("lsensor_wq");
    if (!data->light_wq) {
    	LDBG("%s: create workqueue failed\n", __func__);
    	err = -ENOMEM;
    	goto err_create_lwq_failed;
    }

    INIT_DELAYED_WORK(&data->light_work, lsensor_work_handler);
#endif

#if POLLING_MODE
    LDBG("Timer module installing\n");
    setup_timer(&data->pl_timer, pl_timer_callback, 0);
#endif


    ap3223_data_g = data;

//    /*init for asus threadshould*/
//    err = asus_init_threshold(client);
//    if (err)
//        LDBG("asus_init_threshold fail err = %d\n",err);


    /*-- create inpute device node --*/
	err = sysfs_create_group(&data->lsensor_input_dev->dev.kobj,
			&ap3223_attr_group);
    if (err)
    	goto error_lsinput_sysfs;

	err = sysfs_create_group(&data->psensor_input_dev->dev.kobj,
			&ap3223_attr_group);

    if (err)
    	goto error_psinput_sysfs;


    dev_info(&client->dev, "Driver version %s success\n", DRIVER_VERSION);
    return 0;

error_psinput_sysfs:
  	input_unregister_device(data->psensor_input_dev);
error_lsinput_sysfs:
  	input_unregister_device(data->lsensor_input_dev);
#ifdef LIGHT_POLLING
err_create_lwq_failed:
destroy_workqueue(data->plsensor_wq);
#endif
err_create_wq_failed:
#if POLLING_MODE
    if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
    ap3223_unregister_psensor_device(client,data);
#ifdef HEARBEAT
exit_free_heartbeats_device:
    ap3223_unregister_heartbeat_device(client,data);
#endif
exit_free_ls_device:
    ap3223_unregister_lsensor_device(client,data);

exit_kfree:
    kfree(data);

exit_free_gpio:
    return err;

    dev_err(&client->dev, "Driver probe failed\n");

}

static int ap3223_remove(struct i2c_client *client)
{
    struct ap3223_data *data = i2c_get_clientdata(client);
    free_irq(data->irq, data);

    ap3223_unregister_psensor_device(client,data);
    ap3223_unregister_lsensor_device(client,data);
#ifdef HEARBEAT
    ap3223_unregister_heartbeat_device(client,data);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ap3223_early_suspend);
#endif

    ap3223_set_mode(data->client, 0);
    sysfs_remove_group(&client->dev.kobj, &ap3223_attr_group);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#if POLLING_MODE
    if(&data->pl_timer)
	del_timer(&data->pl_timer);
#endif
    return 0;
}


#ifdef CONFIG_OF
static struct of_device_id ap3223_match_table[] = {
	{.compatible = "dynaimage,ap3223,sr" },
	{},
};
static struct of_device_id ap3223_match_table_sr[] = {
	{.compatible = "dynaimage,ap3223,sr" },
	{},
};
static struct of_device_id ap3223_match_table_er2[] = {
	{.compatible = "dynaimage,ap3223,er2" },
	{},
};
#else
#define ap3223_match_table NULL
#endif

static const struct i2c_device_id ap3223_id[] = {
    { AP3223_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3223_id);

static struct i2c_driver ap3223_driver = {
    .driver = {
	.name	= AP3223_DRV_NAME,
	.owner	= THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = ap3223_match_table,
#endif
    },
    .probe	= ap3223_probe,
    .remove	= ap3223_remove,
    .id_table = ap3223_id,
};

static int __init ap3223_init(void)
{
    int ret = 0;

    LDBG("system entry_mode = %d \n",entry_mode);
	if(entry_mode != 1 ){
		LDBG(" Not in MOS, no need light/proximity driver. \n");
        return ret;
	}


    LDBG("ap3223_init\n");
    LDBG("PCL_33 value is %d\n", gpio_get_value(33));

    int hardware_id = 0;
    hardware_id=asustek_boardinfo_get(FUN_HARDWARE_ID);
    LDBG("%s : hardware_id is %d \n", __func__, hardware_id);

    //SR:0 ER:1 ER2:2 PR:3
    if (hardware_id == 0x00 || hardware_id == 0x01) {
    	if(hardware_id == 0x01)
    	    gpio_set_value(75,1); //VDD12_CADIZ_EN
    	LDBG("%s : hardware_id is SR or ER1 !\n", __func__);
    	ap3223_driver.driver.of_match_table = ap3223_match_table_sr;
    }
    /*else if  (hardware_id == 0x03) {
        PINFO("%s : hardware_id is ER2 !\n", __func__);
        bma2x2_driver.driver.of_match_table = ap3223_match_table[2];
    }
    */
    else {
    	LDBG(" %s : hardware_id is ER2 or others !\n", __func__);
	    gpio_set_value(33,1); //SENSOR_1V8_EN
    	ap3223_driver.driver.of_match_table = ap3223_match_table_er2;
    }

    ret = i2c_add_driver(&ap3223_driver);
    return ret;	

}

static void __exit ap3223_exit(void)
{
    i2c_del_driver(&ap3223_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3223 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3223_init);
module_exit(ap3223_exit);



