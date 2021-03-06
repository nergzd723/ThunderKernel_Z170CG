/* drivers/input/touchscreen/ftxxxx_ts.c
*
* FocalTech ftxxxx TouchScreen driver.
*
* Copyright (c) 2014  Focaltech Ltd.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include <linux/async.h>

#include <linux/i2c.h>
#include <linux/input.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>/*ori #include <mach/irqs.h>*/
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include "ftxxxx_ts.h"
#include <linux/switch.h>
#include <linux/gpio.h>/*ori #include <mach/gpio.h>*/
/*#include <mach/map.h>
#include <mach/regs-clock.h>
#include <mach/regs-gpio.h>
#include <plat/gpio-cfg.h>
#include <linux/gpio.h>
*/
/*#include "linux/input/proximity_class.h"*/
#include <linux/pinctrl/consumer.h>
#include <linux/input/ft3x27.h>
#include <../arch/x86/platform/asustek/include/asustek_boardinfo.h>
#include <linux/of_irq.h>
#include "ftxxxx_ex_fun.h"
#include <linux/power_supply.h>

#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define FTS_CTL_IIC 

/*sometimes up event missed due to packet lost
the workaround is to check every slot(0~9) while
reporting touch event
*/
#define EVENT_SLOT_CHECK

/* ENG BUILD NOT NEED INCLUDE GESTURE */
#ifdef ASUS_FACTORY

#else
#define FTS_GESTRUE
#endif


#ifdef FTS_GESTRUE
bool has_gesture = false;
bool kernel_suspend = false;
#endif

/*#define CONFIG_PM*/

#ifdef EVENT_SLOT_CHECK
int points_status[10] = {0,0,0,0,0,0,0,0,0,0};
#endif

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

/*zax 20140922*/
#define KEY_GESTURE_U		KEY_POWER
#define KEY_GESTURE_V		KEY_V
#define KEY_GESTURE_W		KEY_W
#define KEY_GESTURE_E		KEY_E
#define KEY_GESTURE_C		KEY_C
#define KEY_GESTURE_S		KEY_S
#define KEY_GESTURE_Z		KEY_Z


#define GESTURE_DOUBLECLICK     	0x24
#define GESTURE_V			0x54
#define GESTURE_W			0x31
#define GESTURE_E                       0x33
#define GESTURE_C			0x34
#define GESTURE_S                       0x46
#define GESTURE_Z			0x65


#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME 62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

int gestrue_id = 0;

#ifdef SYSFS_DEBUG
#include "ftxxxx_ex_fun.h"
#endif

/*ASUS_BSP jacob kung: add for debug mask +++ */
#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;

static unsigned int gPrint_point = 0;

static unsigned int lcm_match = 0; 
extern unsigned int entry_mode;

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define focal_debug(level, ...) do { if (debug >= (level)) pr_info(__VA_ARGS__); } while (0)
/*ASUS_BSP jacob kung: add for debug mask --- */

int focal_init_success = 0;
int TPID = -1;
unsigned char IC_FW;
u8 B_VenderID;
u8 F_VenderID;
char B_projectcode[8];
u8 F_projectcode;
u8 gesture_state;
#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
int cos_flag = false;
#endif

static void ftxxxx_ts_suspend(struct early_suspend *handler);
static void ftxxxx_ts_resume(struct early_suspend *handler);
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend focal_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = ftxxxx_ts_suspend,
	.resume = ftxxxx_ts_resume,
};
#endif

struct ftxxxx_ts_data *ftxxxx_ts;
static bool touch_down_up_status;

#define TOUCH_MAX_X						720
#define TOUCH_MAX_Y						1280

#define ANDROID_INPUT_PROTOCOL_B

/* jacob add for i2c retey and if i2c error countor > 10 reset IC */
#define IICReadWriteRetryTime	3
static int IICErrorCountor;
/* jacob add for i2c retey  and if i2c error countor > 10 reset IC */

/*#define FTXXXX_RESET_PIN	88//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_RESET_PIN_NAME	"ft3417-rst"
/*#define FTXXXX_INT_PIN	62//EXYNOS4_GPJ0(3) //S5PV210_GPB(2)*/
#define FTXXXX_INT_PIN_NAME	"ft3417-int"

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
#define FTXXXX_TP_EN_NAME	"ft3417-tp_en"
#endif

/*
*ftxxxx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret;
	int retry = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 2);

			if (ret >= 0)
				break;

			msleep(1);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};

		for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 1);

			if (ret >= 0)
				break;

			msleep(1);
		}
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read error.  error code = %d \n", __func__, ret);
		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read/write error over 10 times !! \n", __func__);
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: excute reset IC process !! \n", __func__);
			queue_delayed_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work, 0);
			return ret;
		}

		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}
/*write data by i2c*/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int retry = 0;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	for (retry = 0; retry < IICReadWriteRetryTime; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);

		if (ret >= 0)
			break;

		msleep(1);
	}

	if (retry == IICReadWriteRetryTime) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c write error.  error code = %d \n", __func__, ret);

		IICErrorCountor += 1;

		if (IICErrorCountor >= 10) {
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: i2c read/write error over 10 times !! \n", __func__);
			dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: excute reset IC process !! \n", __func__);
			queue_delayed_work(ftxxxx_ts->reset_wq, &ftxxxx_ts->reset_ic_work, 0);
			return ret;
		}

		return ret;
	}

	IICErrorCountor = 0;

	return ret;
}

/*ASUS_BSP Jacob : add for creating virtual_key_maps +++*/

#define MAX_LEN		200

static ssize_t focalTP_virtual_keys_register(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	char *virtual_keys = 	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":160:2500:30:30" "\n" \

				__stringify(EV_KEY) ":" __stringify(KEY_HOME) ":360:2500:30:30" "\n" \

				__stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":560:2500:30:30" "\n" ;

	return snprintf(buf, strnlen(virtual_keys, MAX_LEN) + 1 , "%s",	virtual_keys);

}

static struct kobj_attribute focalTP_virtual_keys_attr = {

	.attr = {
		.name = "virtualkeys.focal-touchscreen",
		.mode = S_IRWXUGO,
	},

	.show = &focalTP_virtual_keys_register,

};

static struct attribute *virtual_key_properties_attrs[] = {

	&focalTP_virtual_keys_attr.attr,

	NULL

};

static struct attribute_group virtual_key_properties_attr_group = {

	.attrs = virtual_key_properties_attrs,

};

struct kobject *focal_virtual_key_properties_kobj;

/*ASUS_BSP Jacob : add for creating virtual_key_maps ---*/

u8 get_focal_tp_fw(void)
{
	u8 fwver = 0;

	if (ftxxxx_read_reg(ftxxxx_ts->client, FTXXXX_REG_FW_VER, &fwver) < 0)
		return -1;
	else
		return fwver;
}

static ssize_t focal_show_tpfwver(struct switch_dev *sdev, char *buf)
{
	int num_read_chars = 0;
	IC_FW = get_focal_tp_fw();

	if (IC_FW == 255) {
		printk("[Focal][Touch] %s :  read FW fail \n ", __func__);
		num_read_chars = snprintf(buf, PAGE_SIZE, "get tp fw version fail!\n");
	} else {
		printk("[Focal][Touch] %s :  touch FW = 0x%x\n ", __func__, IC_FW);
		num_read_chars = snprintf(buf, PAGE_SIZE, "%d\n", IC_FW);
	}
	return num_read_chars;
}

static void check_gesture(struct ftxxxx_ts_data *data, int gesture_id)
{

	printk("[Focal][Touch] %s :  gesture_id = 0x%x\n ", __func__, gesture_id);

	switch (gesture_id) {
/*	case GESTURE_LEFT:
		input_report_key(data->input_dev, KEY_GESTURE_LEFT, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_LEFT, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_RIGHT:
		input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_UP:
		input_report_key(data->input_dev, KEY_GESTURE_UP, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_UP, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_DOWN:
		input_report_key(data->input_dev, KEY_GESTURE_DOWN, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_DOWN, 0);
		input_sync(data->input_dev);
		break;
*/
	case GESTURE_DOUBLECLICK:
		input_report_key(data->input_dev, KEY_GESTURE_U, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_U, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_V:
		input_report_key(data->input_dev, KEY_GESTURE_V, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_V, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_W:
		input_report_key(data->input_dev, KEY_GESTURE_W, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_W, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_E:
		input_report_key(data->input_dev, KEY_GESTURE_E, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_E, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_C:
		input_report_key(data->input_dev, KEY_GESTURE_C, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_C, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_S:
		input_report_key(data->input_dev, KEY_GESTURE_S, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_S, 0);
		input_sync(data->input_dev);
		break;

	case GESTURE_Z:
		input_report_key(data->input_dev, KEY_GESTURE_Z, 1);
		input_sync(data->input_dev);
		input_report_key(data->input_dev, KEY_GESTURE_Z, 0);
		input_sync(data->input_dev);
		break;

	default:

		break;

	}

}


static int fts_read_Gestruedata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
	unsigned char leave_buf[0];
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;
	int gestrue_id = 0;

	pointnum = 0;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	printk("[Focal][Touch] %s : tpd read FTS_GESTRUE_POINTS_HEADER.\n", __func__);

	if (ret < 0) {
		printk("[Focal][TOUCH_ERR] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	/*if (fts_updateinfo_curr.CHIP_ID==0x54)*/
	/*{*/
			 gestrue_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;

		if ((pointnum * 4 + 8) < 255) {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
		} else {
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 255);
			ret = ftxxxx_i2c_Read(data->client, buf, 0, buf+255, (pointnum * 4 + 8) - 255);
		}
		if (ret < 0) {
			printk("[Focal][TOUCH_ERR] %s read touchdata failed.\n", __func__);
			return ret;
		}
	check_gesture(data, gestrue_id);
	for (i = 0; i < pointnum; i++) {
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[1 + (4 * i)]) & 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}

	// Leave Gesture Mode
	leave_buf[0] = 0xd0;
	ftxxxx_i2c_Write(data->client, leave_buf, 1);

	return -1;
/*}*/
/*
	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		check_gesture(gestrue_id);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;

	if((pointnum * 4 + 8)<255) {
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	}
	else
	{
		 ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
		 ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf, pointnum);
	check_gesture(gestrue_id);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;*/
}

/*static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 8);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	if (0x24 == buf[0]) {
		gestrue_id = 0x24;
		printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);
		return -1;
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
	Read two times
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (8));
	ret = ftxxxx_i2c_Read(data->client, buf, 0, (buf+8), (8));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gestrue_id = fetch_object_sample(buf,pointnum);

	printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}

	return -1;
}*/

/*Read touch point information when the interrupt  is asserted.*/
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	struct ts_event *event = &data->event;
	memset(event, 0, sizeof(*event));
	//u8 buf[POINT_READ_BUF] = { 0 };
	u8 buf[64] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	//ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 64);
	if (ret < 0) {
		dev_err(&data->client->dev, "[Focal][TOUCH_ERR] %s : read touchdata failed.\n", __func__);
		return ret;
	}

	/*Ft_Printf_Touchdata(data,buf);*/	/*打印报点调试信息*/

	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		event->pressure[i] =
			(buf[FT_TOUCH_XY_POS + FT_TOUCH_STEP * i]);
		event->area[i] =
			(buf[FT_TOUCH_MISC + FT_TOUCH_STEP * i]) >> 4;

		//event->pressure[i] = 200;
		//event->area[i] = 50;

		focal_debug(DEBUG_VERBOSE, "id=%d event=%d x=%d y=%d pressure=%d area=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
	}

	/*event->pressure = FT_PRESS;*/
	//event->pressure = 200;
	//event->area = 50;

	return 0;
}

/*
*report the point information
*/
static void ftxxxx_report_value(struct ftxxxx_ts_data *data)
{	
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;

#ifdef EVENT_SLOT_CHECK
	for (i = 0; i < 10; i++) {
		if(points_status[i] == 2) {
			points_status[i] = 1;
		}
	}
#endif	
	 
	/*protocol B*/
	for (i = 0; i < event->touch_point; i++) {
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2) {
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);	
			if(unlikely(gPrint_point)) {
			    printk("[Focal][Touch] id = %d, report_abs_X = %d, report_abs_Y = %d, pressure = %d, area = %d \n", event->au8_finger_id[i], event->au16_x[i], event->au16_y[i], event->pressure[i], event->area[i]);
			}
#ifdef EVENT_SLOT_CHECK
			points_status[event->au8_finger_id[i]] = 2;
#endif
		} else {
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#ifdef EVENT_SLOT_CHECK
			points_status[event->au8_finger_id[i]] = 0;
#endif
		}
	}

#ifdef EVENT_SLOT_CHECK		
	for (i = 0; i < 10; i++) {
		if(points_status[i] == 1) {
			points_status[i] = 0;
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			
		}
	}
#endif

	if(event->touch_point == uppoint) {
		input_report_key(data->input_dev, BTN_TOUCH, 0);
		touch_down_up_status = 0;
		pr_debug("[Focal][Touch] touch up !\n");
	} else {
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
		if (touch_down_up_status == 0) {
			touch_down_up_status = 1;
			pr_debug("[Focal][Touch] touch down !\n");
		}
			
	}
	input_sync(data->input_dev);
}


void force_release_pos(struct ftxxxx_ts_data *data)
{
	printk("[Focal][Touch] touch clear\n");
	int i;
	struct ts_event *event = &data->event;
	for (i = 0; i < event->touch_point; i++) {
		input_mt_slot(data->input_dev, event->au8_finger_id[i]);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
#ifdef EVENT_SLOT_CHECK
		points_status[event->au8_finger_id[i]] = 0;
#endif
	}

#ifdef EVENT_SLOT_CHECK		
	for (i = 0; i < 10; i++) {
		if(points_status[i] > 0) {
			points_status[i] = 0;
			input_mt_slot(data->input_dev, i);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			
		}
	}
#endif
	
	input_sync(data->input_dev);
}

static void focal_read_gesture(void)
{
	//printk("[Focal][Touch] %s : Touch gesture work \n", __func__);
	i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0xd0, 1, &gesture_state);
        //printk("tpd fts_read_Gestruedata state=%d\n", gesture_state);
	if (gesture_state == 1)
		fts_read_Gestruedata(ftxxxx_ts);
}


/*The ftxxxx device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ftxxxx_ts_interrupt(int irq, void *dev_id)
{
/*	struct ftxxxx_ts_data *ftxxxx_ts = dev_id; ASUS jacob use globle ftxxxx_ts data*/
	int ret = 0;

#ifdef FTS_GESTRUE
	if (ftxxxx_ts->suspend_flag && (ftxxxx_ts->dclick == 1 || ftxxxx_ts->gesture_all == 1)) {
		if(kernel_suspend) {
			has_gesture = true;
		} else {
			focal_read_gesture();
		}
		return IRQ_HANDLED;
	}
#endif
	ret = ftxxxx_read_Touchdata(ftxxxx_ts);

	if (ret == 0)
		ftxxxx_report_value(ftxxxx_ts);

	return IRQ_HANDLED;
}

void ftxxxx_reset_tp(int HighOrLow)
{
	pr_info("[Focal][Touch] %s : set tp reset pin to %d\n", __func__, HighOrLow);
	gpio_set_value(ftxxxx_ts->pdata->rst_gpio, HighOrLow);
}

int ftxxxx_read_tp_id(void)
{
	int temp = 0, temp1 = 0;
	temp = gpio_get_value(TPID_1) > 0 ? 1 : 0;
	temp1 |= temp;
	temp = gpio_get_value(TPID_2) > 0 ? 1 : 0;
	temp1 |= temp << 1;
	temp = gpio_get_value(TPID_3) > 0 ? 1 : 0;
	temp1 |= temp << 2;

	printk("[Focal][Touch] %s : TPID = %d !\n", __func__, temp1);

	printk("[Focal][Touch] %s : TPID1 = %d  TPID2 = %d TPID3 = %d !\n", __func__, gpio_get_value(TPID_1), gpio_get_value(TPID_2), gpio_get_value(TPID_3));

	TPID = temp1;

	return TPID;
}

void set_event_printable(int on) {
	gPrint_point = on;
}


int cat_event_printable(void){
        return gPrint_point;
}

int cat_lcm_match(void){
	/*0 : LCM ID & vendor ID mismatch
	1: LCM ID & vendor ID match*/
	return lcm_match;
}

void set_lcm_match(int mat) {
	lcm_match = mat;
	printk("[Focal][TOUCH] %s : lcm_match = %d \n", __func__,lcm_match);
}

//write USB mode to 0x8B
static void focal_set_usb(void) 
{
	uint8_t buf[2] = {0};
	if (ftxxxx_ts->usb_status == 0x02) {	/*no AC */
		buf[0] = 0x8B;
		buf[1] = 0x00;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		printk("[Focal][Touch] set no AC to MCU\n");
	} else if ((ftxxxx_ts->usb_status == 0x00) || (ftxxxx_ts->usb_status == 0x01)) {	/*AC plug in*/
		buf[0] = 0x8B;
		buf[1] = 0x01;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		printk("[Focal][Touch] set AC in to MCU\n");
	}
	return;
}

static void focal_cable_status(struct work_struct *work)
{
	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	printk("[Focal][Touch] cable_status=%d, init_success=%d.\n", ftxxxx_ts->usb_status, ftxxxx_ts->init_success);

	if (ftxxxx_ts->init_success == 1) {
		focal_set_usb();
	}

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}


static void focal_do_reset(void)
{
	ftxxxx_irq_switch(false);
	ftxxxx_reset_tp(0);
	msleep(20);
	ftxxxx_reset_tp(1);
	msleep(300);
	IICErrorCountor = 0;
	ftxxxx_irq_switch(true);
	return;
}

static void focal_reset_ic_work(struct work_struct *work)
{

	struct ftxxxx_ts_data *ts = ftxxxx_ts;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	focal_do_reset();	

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;

}

static void focal_resume_work(struct work_struct *work)
{
	struct ftxxxx_ts_data *ts = ftxxxx_ts;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (!ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] IC did not enter suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		wake_unlock(&ftxxxx_ts->wake_lock);

		return;

	}

	printk("[Focal][Touch] %s : Touch resume \n", __func__);

	if (ftxxxx_ts->reset_pin_status == 1) {

#ifdef FTS_GESTRUE
		if ((ts->dclick == 1 && ts->gesture_all == 1) ||(ts->dclick == 0 && ts->gesture_all == 1) ||(ts->dclick == 1 && ts->gesture_all == 0) ){
			ftxxxx_write_reg(ts->client, 0xD0, 0x00);  //leave gesture mode
			ftxxxx_ts->suspend_flag = 0; //turn off suspend_flag earlier 
		}
#endif

		focal_do_reset();//reset ic for both gesture and suspend mode

		focal_set_usb();

	} else {
		printk("[Focal][Touch] %s : ftxxxx_ts->reset_pin_status set to 0 ! skip reset IC \n", __func__);
	}

#ifdef FTS_GESTRUE
	if(ts->dclick == 0 && ts->gesture_all == 0) {
		ftxxxx_irq_switch(true);  //leave suspend mode,need to enable irq
	}
#else
	ftxxxx_irq_switch(true);
#endif

	ftxxxx_ts->suspend_flag = 0;

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}

static int ftxxxx_power_switch(int on)
{
	int ret = 0;
#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
        msleep(20);
        ret = gpio_direction_output(ftxxxx_ts->pdata->tp_en_gpio, on);
        if (ret) {
                        printk("[Focal][TOUCH_ERR] %s: set %s gpio to output %d failed, ret = %d\n",
                                __func__, FTXXXX_TP_EN_NAME, on, ret);
                        return ret;
        }
#endif
	return ret;
}

static int fts_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;




//usleep(100);


	ret = gpio_request(ftxxxx_ts->pdata->rst_gpio, FTXXXX_RESET_PIN_NAME);
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
	}

	/*ret = gpio_request(TPID_1, "TPID_1");
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %d for TPID failed, ret = %d\n",
			__func__, TPID_1, ret);
		return ret;
	}

	ret = gpio_request(TPID_2, "TPID_2");
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %d for TPID failed, ret = %d\n",
			__func__, TPID_2, ret);
		return ret;
	}

	ret = gpio_request(TPID_3, "TPID_3");
	if (ret) {
		printk("[Focal][TOUCH_ERR] %s: request GPIO %d for TPID failed, ret = %d\n",
			__func__, TPID_3, ret);
		return ret;
	}*/

/*
	s3c_gpio_setpull(FTXXXX_RESET_PIN, S3C_GPIO_PULL_NONE);
	* s3c_gpio_cfgpin(FTXXXX_RESET_PIN, S3C_GPIO_SFN(1));//  S3C_GPIO_OUTPUT
	* gpio_set_value(FTXXXX_RESET_PIN,  0);
*/



	ret = gpio_direction_output(ftxxxx_ts->pdata->rst_gpio, 0);	
	if (ret) {
			printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put low failed, ret = %d\n",
				__func__, FTXXXX_RESET_PIN_NAME, ret);
			return ret;
	}

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
        msleep(20);
        ret = gpio_request(ftxxxx_ts->pdata->tp_en_gpio, FTXXXX_TP_EN_NAME);
        if (ret) {
                printk("[Focal][TOUCH_ERR] %s: request GPIO %s for tp en failed, ret = %d\n",
                        __func__, FTXXXX_TP_EN_NAME, ret);
                return ret;
        }
#endif

	ret = ftxxxx_power_switch(true);	
	if (ret) {
		printk("[Focal][TOUCH_ERR] ftxxxx_power_switch on fail,ret = %d\n", ret);
                return ret;
	}

	msleep(20);
	
	ret = gpio_direction_output(ftxxxx_ts->pdata->rst_gpio, 1);	/*asus change reset set high*/
	if (ret) {
			printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put high failed, ret = %d\n",
				__func__, FTXXXX_RESET_PIN_NAME, ret);
			return ret;
	}

	#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C) 
	msleep(200);
	#endif

	#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	//Focal modify for vendor ID unstable issue
	mdelay(500);
	#endif

	//ret = gpio_direction_input(TPID_1);	/* asus TPID 1 */
	/*if (ret) {
			printk("[Focal][TOUCH_ERR] %s: set %d gpio to out put high failed, ret = %d\n",
				__func__, TPID_1, ret);
			return ret;
		}
	gpio_set_value(TPID_1, 1);*/

	//ret = gpio_direction_input(TPID_2);	/* asus TPID 2 */
	/*if (ret) {
			printk("[Focal][TOUCH_ERR] %s: set %d gpio to out put high failed, ret = %d\n",
				__func__, TPID_2, ret);
			return ret;
		}
	gpio_set_value(TPID_2, 1);*/

	//ret = gpio_direction_input(TPID_3);	/* asus TPID 3 */
	/*if (ret) {
			printk("[Focal][TOUCH_ERR] %s: set %d gpio to out put high failed, ret = %d\n",
				__func__, TPID_3, ret);
			return ret;
		}
	gpio_set_value(TPID_3, 1);*/

	return ret;
}

static void fts_un_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
}


#ifdef CONFIG_OF

#define OF_FTXXXX_PIN_RESET	"focaltech,reset-gpio"
#define OF_FTXXXX_IRQ	"focaltech,irq-gpio"

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
#define OF_FTXXXX_TP_EN	"focaltech,tp_en-gpio"
#endif

static int ft3x27_get_dt_coords(struct device *dev, char *name,
                                struct focal_i2c_platform_data *pdata)
{
        u32 coords[FT_COORDS_ARR_SIZE];
        struct property *prop;
        struct device_node *np = dev->of_node;
        int coords_size, rc;

        prop = of_find_property(np, name, NULL);
        if (!prop)
                return -EINVAL;
        if (!prop->value)
                return -ENODATA;

        coords_size = prop->length / sizeof(u32);
        if (coords_size != FT_COORDS_ARR_SIZE) {
                dev_err(dev, "invalid %s\n", name);
                return -EINVAL;
        }

        rc = of_property_read_u32_array(np, name, coords, coords_size);
        if (rc && (rc != -EINVAL)) {
                dev_err(dev, "Unable to read %s\n", name);
                return rc;
        }

        if (!strcmp(name, "focaltech,panel-coords")) {
                pdata->abs_x_min = coords[0];
                pdata->abs_y_min = coords[1];
                pdata->abs_x_max = coords[2];
                pdata->abs_y_max = coords[3];
        } else if (!strcmp(name, "focaltech,display-coords")) {
                pdata->abs_x_min = coords[0];
                pdata->abs_y_min = coords[1];
                pdata->abs_x_max = coords[2];
                pdata->abs_y_max = coords[3];
        } else {
                dev_err(dev, "unsupported property %s\n", name);
                return -EINVAL;
        }
	return 0;
}
static struct focal_i2c_platform_data *ftxxxx_ts_of_get_platdata(
		struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	
	struct focal_i2c_platform_data *ftxxxx_pdata;

	int ret;

	ftxxxx_pdata = devm_kzalloc(&client->dev,
			sizeof(*ftxxxx_pdata), GFP_KERNEL);
	if (!ftxxxx_pdata)
		return ERR_PTR(-ENOMEM);

	/* pinctrl */
	ftxxxx_pdata->pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR(ftxxxx_pdata->pinctrl)) {
		ret = PTR_ERR(ftxxxx_pdata->pinctrl);
		goto out;
	}

	ftxxxx_pdata->pins_default = pinctrl_lookup_state(
			ftxxxx_pdata->pinctrl, PINCTRL_STATE_DEFAULT);
	if (IS_ERR(ftxxxx_pdata->pins_default))
		dev_err(&client->dev, "could not get default pinstate\n");

	ftxxxx_pdata->pins_sleep = pinctrl_lookup_state(
			ftxxxx_pdata->pinctrl, PINCTRL_STATE_SLEEP);
	if (IS_ERR(ftxxxx_pdata->pins_sleep))
		dev_err(&client->dev, "could not get sleep pinstate\n");

	ftxxxx_pdata->pins_inactive = pinctrl_lookup_state(
			ftxxxx_pdata->pinctrl, "inactive");
	if (IS_ERR(ftxxxx_pdata->pins_inactive))
		dev_err(&client->dev, "could not get inactive pinstate\n");

	/* gpio reset */
	ftxxxx_pdata->rst_gpio = of_get_named_gpio_flags(client->dev.of_node,
			OF_FTXXXX_PIN_RESET, 0, NULL);
	client->irq = irq_of_parse_and_map(client->dev.of_node,0);

	
	if (ftxxxx_pdata->rst_gpio <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FTXXXX_PIN_RESET);
		ret = -EINVAL;
		goto out;
	}

	/* gpio irq */
	ftxxxx_pdata->intr_gpio = of_get_named_gpio_flags(client->dev.of_node,
			OF_FTXXXX_IRQ, 0, NULL);
	if (ftxxxx_pdata->intr_gpio <= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FTXXXX_IRQ);
		ret = -EINVAL;
		goto out;
	}

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	/* tp en irq */
	ftxxxx_pdata->tp_en_gpio = of_get_named_gpio_flags(client->dev.of_node,
			OF_FTXXXX_TP_EN, 0, NULL);
	if (ftxxxx_pdata->tp_en_gpio<= 0) {
		dev_err(&client->dev,
			"error getting gpio for %s\n", OF_FTXXXX_TP_EN);
		ret = -EINVAL;
		goto out;
	}
#endif

	/* pm */
	ftxxxx_pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(ftxxxx_pdata->pm_platdata)) {
		dev_warn(&client->dev, "Error during device state pm init\n");
		ret = PTR_ERR(ftxxxx_pdata->pm_platdata);
		goto out;
	}

	ft3x27_get_dt_coords(&client->dev, "focaltech,display-coords", ftxxxx_pdata);

	return ftxxxx_pdata;

out:
	return ERR_PTR(ret);
}
#endif

int setTouchACMode(int usb_state)
{
	ftxxxx_ts->usb_status = usb_state;
	printk("[Focal][Touch]%s: ftxxxx_ts->usb_status = %d, usb_state = %d \n", __func__, ftxxxx_ts->usb_status,usb_state);

	if(!ftxxxx_ts->suspend_flag) {//if kernel is in suspend,dont write to MCU until focal_resume_work
		queue_work(ftxxxx_ts->usb_wq, &ftxxxx_ts->usb_detect_work);
	}
	return 0;
}

static int cable_status_notify(struct notifier_block *self,
			       unsigned long action, void *dev)
{

	//if (ischargerSuspend) {
	//	printk(KERN_INFO
	//	       "%s chager is suspend but USB still notify !!!\n",
	//	       __func__);
	//	wake_lock(&smb345_dev->wakelock);
	//	isUSBSuspendNotify = true;
	//	return NOTIFY_OK;
	//}

	switch (action) {
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		printk("[Focal][Touch] POWER_SUPPLY_CHARGER_TYPE_USB_SDP !!!\n");
		//action = USB_IN;
		setTouchACMode(0);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		printk( "[Focal][Touch] POWER_SUPPLY_CHARGER_TYPE_USB_CDP !!!\n");
		//action = AC_IN;
		setTouchACMode(1);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		printk("[Focal][Touch] POWER_SUPPLY_CHARGER_TYPE_USB_DCP !!!\n");
		//action = AC_IN;
		setTouchACMode(1);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		printk( "[Focal][Touch] POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK !!!\n");
		//action = AC_IN;
		setTouchACMode(1);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_SE1:
		printk( "[Focal][Touch] POWER_SUPPLY_CHARGER_TYPE_SE1 !!!\n");
		//action = AC_IN;
		setTouchACMode(1);
		break;
#if 0
	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED:
		printk(KERN_INFO
		       "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED !!!\n",
		       __func__);
		action = ENABLE_5V;
		setTouchACMode(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG_DISCONNECTED:
		printk(KERN_INFO
		       "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG_DISCONNECTED !!!\n",
		       __func__);
		action = DISABLE_5V;
		setTouchACMode(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG !!!\n",
		       __func__);
		action = ENABLE_5V;
		setTouchACMode(action);
		break;

#endif

	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_NONE !!!\n",
		       __func__);
		//action = CABLE_OUT;
		setTouchACMode(2);
		break;

	default:
		printk(KERN_INFO "%s no status = %d !!!\n", __func__,(int)action);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier = {
	.notifier_call = cable_status_notify,
};

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);

static int check_project_id(void)
{
	int project_id = 0;
	int error = -1;
	int ret = 0;
	project_id = asustek_boardinfo_get(FUN_PROJECT_ID);
	printk("[Focal][Touch] PROJECT ID :%d\n", project_id);

#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	if (project_id < 7 || project_id > 14) {
		printk("[Focal][Touch] PROJECT ID is not Z170 series\n");
		return error;
	}
#endif

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	if (project_id < 15 || project_id > 16) {
		printk("[Focal][Touch] PROJECT ID is not Z370 series\n");
		return error;
	}
#endif
	return ret;
}

static int check_lcm_id(void)
{
	int lcm_id = 0;
	int error = -1;
	int ret = 0;
	lcm_id = asustek_boardinfo_get(FUN_LCM_ID);
	printk("[Focal][Touch] LCM ID :%d\n", lcm_id);
	if (lcm_id != BOE_FOCAL){
		printk("[Focal][Touch] LCM ID is not BOE\n");
		return error;
	}
	return ret;
}

static int ftxxxx_ts_shutdown(struct i2c_client *client)
{
	int ret = 0;

		ftxxxx_irq_switch(false);

        ret = gpio_direction_output(ftxxxx_ts->pdata->rst_gpio, 0);
        if (ret) {
                        printk("[Focal][TOUCH_ERR] %s: set %s gpio to out put low failed, ret = %d\n",
                                __func__, FTXXXX_RESET_PIN_NAME, ret);
                        return ret;
        }
	msleep(5);
	ret = ftxxxx_power_switch(false);
	return ret;	
}

static void init_gesture_value(struct ftxxxx_ts_data *ftxxxx_ts)
{
	ftxxxx_ts->dclick = 1;
	ftxxxx_ts->gesture_all = 1;
	ftxxxx_ts->gesture_wec = 0x1A;
	ftxxxx_ts->gesture_s = 1;
	ftxxxx_ts->gesture_v = 1;
	ftxxxx_ts->gesture_z = 1;
}

#ifdef FTS_GESTRUE
static int ftxxxx_kernel_resume(struct i2c_client *client)
{
	if(has_gesture) {
		has_gesture = false;
		focal_read_gesture();
    }
    kernel_suspend = false;
    return 0;
}

static int ftxxxx_kernel_suspend(struct i2c_client *client, pm_message_t mesg)
{
        kernel_suspend = true;
        return 0;
}
#endif

static int ftxxxx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct focal_i2c_platform_data *pdata;
	int err = 0;
        int retry_ven = 0;
        int retry_check = 0;

	// Check Project ID
	err = check_project_id();
	if (err < 0){
		printk("[Focal][Touch] Project ID return probe\n");
		return err;
	}


#ifdef CONFIG_OF
	pdata = client->dev.platform_data =
		ftxxxx_ts_of_get_platdata(client);
	if (IS_ERR(pdata)) {
		err = PTR_ERR(pdata);
		return err;
	}
#else
	pdata = (struct focal_i2c_platform_data *)client->dev.platform_data;
#endif

	struct input_dev *input_dev;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;

	printk("[Focal][Touch] FTxxxx prob process Start !\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
	goto exit_check_functionality_failed;
	}

	ftxxxx_ts = kzalloc(sizeof(struct ftxxxx_ts_data), GFP_KERNEL);
	if (!ftxxxx_ts) {
		err = -ENOMEM;
		printk("[Focal][TOUCH_ERR] %s: alloc ftxxxx_ts_data failed !! \n", __func__);
	goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ftxxxx_ts);

	ftxxxx_ts->client = client;
	ftxxxx_ts->init_success = 0;
	ftxxxx_ts->suspend_flag = 0;
	ftxxxx_ts->usb_status = 0;
	ftxxxx_ts->pdata = pdata;
	ftxxxx_ts->x_max = pdata->abs_x_max;
	ftxxxx_ts->y_max = pdata->abs_y_max;
	init_gesture_value(ftxxxx_ts);
	ftxxxx_ts->int_flag = false;
	ftxxxx_ts->auto_update_fw_result = 1;

	if (0 >= ftxxxx_ts->x_max)
		ftxxxx_ts->x_max = TOUCH_MAX_X;
	if (0 >= ftxxxx_ts->y_max)
		ftxxxx_ts->y_max = TOUCH_MAX_Y;
	ftxxxx_ts->irq = ftxxxx_ts->pdata->intr_gpio;

	if (fts_init_gpio_hw (ftxxxx_ts) < 0)
		goto exit_init_gpio;

	ftxxxx_ts->reset_pin_status = 1;

        ftxxxx_ts->lcm_id= asustek_boardinfo_get(FUN_LCM_ID);
        printk("[Focal][Touch] LCM ID = %d \n",  ftxxxx_ts->lcm_id);

        ftxxxx_ts->update_check = 0;

	if (gpio_request(ftxxxx_ts->irq, FTXXXX_INT_PIN_NAME)) {
		printk("[Focal][TOUCH_ERR] %s: gpio %d request for interrupt fail.\n", __func__, ftxxxx_ts->irq);
		goto exit_irq_request_failed;
	}
	gpio_direction_input(ftxxxx_ts->irq);

	// Check FPC connect or not
	u8 fwver = get_focal_tp_fw();
	printk("[Focal][TOUCH] %s: fwver = %d\n", __func__, fwver);
	if (fwver == 255){
	  printk("[Focal][TOUCH_ERR] FPC not connect, return touch driver probe\n");
	  goto exit_irq_request_failed;
	}

	/*err = request_threaded_irq(ftxxxx_ts->client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		ftxxxx_ts);
*/

	/*if (ftxxxx_ts->client->irq < 0) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: request irq fail. \n", __func__);
		goto exit_irq_request_failed;
	}*/

	//disable_irq(ftxxxx_ts->client->irq);	/*need mutex protect, should add latter*/

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: failed to allocate input device\n", __func__);
		goto exit_input_dev_alloc_failed;
	}

	ftxxxx_ts->input_dev = input_dev;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ftxxxx_ts->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ftxxxx_ts->y_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);

	input_dev->name = Focal_input_dev_name;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s: failed to register input device: %s\n", __func__, dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
			
	/*make sure CTP already finish startup process */
	msleep(200);

#ifdef SYSFS_DEBUG
	printk("[Focal][Touch] ftxxxx_create_sysfs Start !\n");
	ftxxxx_create_sysfs(client);
	printk("[Focal][Touch] ftxxxx_create_sysfs End !\n");

	mutex_init(&ftxxxx_ts->g_device_mutex);

	mutex_init(&ftxxxx_ts->irq_mutex);

	wake_lock_init(&ftxxxx_ts->wake_lock, WAKE_LOCK_SUSPEND, "focal_touch_wake_lock");

	ftxxxx_ts->usb_wq = create_singlethread_workqueue("focal_usb_wq");
	if (!ftxxxx_ts->usb_wq) {
		printk("[Focal][TOUCH_ERR] %s: create usb workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->usb_detect_work, focal_cable_status);

	ftxxxx_ts->resume_wq = create_singlethread_workqueue("focal_resume_wq");
	if (!ftxxxx_ts->resume_wq) {
		printk("[Focal][TOUCH_ERR] %s: create resume workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->resume_work, focal_resume_work);

	ftxxxx_ts->reset_wq = create_singlethread_workqueue("focal_reset_ic_wq");
	if (!ftxxxx_ts->reset_wq) {
		printk("[Focal][TOUCH_ERR] %s: create reset ic workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_DELAYED_WORK(&ftxxxx_ts->reset_ic_work, focal_reset_ic_work);

	ftxxxx_ts->gesture_wq = create_singlethread_workqueue("focal_gesture_wq");
	if (!ftxxxx_ts->gesture_wq) {
		printk("[Focal][TOUCH_ERR] %s: create gesture workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}


	ftxxxx_ts->touch_sdev.name = "touch";
	ftxxxx_ts->touch_sdev.print_name = focal_show_tpfwver;

/*	if (switch_dev_register(&ftxxxx_ts->touch_sdev) < 0)
	{
		printk("[Focal][TOUCH_ERR] %s: failed to register switch_dev \n", __func__);
		goto exit_err_sdev_register_fail;
	} 
*/
	/*ASUS_BSP Jacob: add for creating virtual_key_maps +++*/

	focal_virtual_key_properties_kobj = kobject_create_and_add("board_properties", NULL);

	if (focal_virtual_key_properties_kobj)

		err = sysfs_create_group(focal_virtual_key_properties_kobj, &virtual_key_properties_attr_group);

	if (!focal_virtual_key_properties_kobj || err)

		printk("[Focal][TOUCH_ERR] %s : failed to create novaTP virtual key map! \n", __func__);

	/*ASUS_BSP Jacob: add for creating virtual_key_maps ---*/
#endif
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "[Focal][TOUCH_ERR] %s : create fts control iic driver failed\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_create_apk_debug_channel(client);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&focal_early_suspend);
#endif
	/*get some register information */
	uc_reg_addr = FTXXXX_REG_FW_VER;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] Firmware version = 0x%x\n", uc_reg_value);
		IC_FW = uc_reg_value;
		}

	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] report rate is %dHz.\n", uc_reg_value * 10);
		}

	uc_reg_addr = FTXXXX_REG_THGROUP;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] touch threshold is %d.\n", uc_reg_value * 4);
		}

#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0)
		ftxxxx_ts->init_success = 0;
	else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] VENDOR ID = 0x%x\n", uc_reg_value);
		}
#endif

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
RETRY_VENDOR_ID_READ:
	err = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	if (err < 0){
		ftxxxx_ts->init_success = 0;
		printk("[Focal][Touch] VENDOR ID read fail \n");

		if (retry_ven < 3){
                    retry_ven++;
		    msleep(50);
		    goto RETRY_VENDOR_ID_READ;
		}
	} else {
		ftxxxx_ts->init_success = 1;
		printk("[Focal][Touch] VENDOR ID(1) = 0x%x\n", uc_reg_value);
		ftxxxx_ts->vendor_id = uc_reg_value;
	}

        if(ftxxxx_ts->vendor_id == FTXXXX_VAL_VENID_CPT){
            if(ftxxxx_ts->lcm_id == CPT){
                lcm_match = 1;
		ftxxxx_ts->update_check = 1;
	    }
	}

        if(ftxxxx_ts->vendor_id == FTXXXX_VAL_VENID_BOE){
            if(ftxxxx_ts->lcm_id == BOE){
                lcm_match = 1;
		ftxxxx_ts->update_check = 1;
	    }
	}

	if(lcm_match == 0){
            if (retry_check< 3){
		printk("[Focal][Touch] lcm_match 0, retry \n");
                retry_check++;
		msleep(50);		
		goto RETRY_VENDOR_ID_READ;
	    }	
	}

	printk("[Focal][Touch] VENDOR ID(2) = 0x%x \n",ftxxxx_ts->vendor_id);

#endif

	//printk("[Focal][Touch] TP ID = %d \n", ftxxxx_read_tp_id());

#ifdef FTS_GESTRUE	/*zax 20140922*/
	/*init_para(720,1280,100,0,0);*/

	/*auc_i2c_write_buf[0] = 0xd0;*/
	/*auc_i2c_write_buf[1] = 0x01;*/
	/*ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gestrue function*/

	/*auc_i2c_write_buf[0] = 0xd1;*/
	/*auc_i2c_write_buf[1] = 0xff;*/
	/*ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);*/
	/*
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x00;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw close gestrue function 
	*/
	input_set_capability(input_dev, EV_KEY, KEY_POWER);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

	//__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	//__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	//__set_bit(KEY_GESTURE_UP, input_dev->keybit);
	//__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	__set_bit(KEY_GESTURE_U, input_dev->keybit);
	//__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	//__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	//__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
#endif

	printk("[Focal][Touch][INFO] client name = %s irq = %d\n", client->name, client->irq);
	printk("[Focal][Touch] X-RES = %d, Y-RES = %d, RST gpio = %d, gpio irq = %d, client irq = %d\n",
		ftxxxx_ts->pdata->abs_x_max, ftxxxx_ts->pdata->abs_y_max, ftxxxx_ts->pdata->rst_gpio, ftxxxx_ts->irq, ftxxxx_ts->client->irq);

	if (ftxxxx_ts->init_success == 1)
		focal_init_success = 1;

#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	if (cos_flag == true) {
		u8 sleep_mode = 0x03;
		ftxxxx_write_reg(ftxxxx_ts->client, 0xA5, sleep_mode);
		printk("[Focal][Touch] Touch in suspend mode.\n");
	} else {
#endif
	printk("[Focal][Touch]  Init charger workqueue\n");
	cable_status_register_client(&cable_status_notifier);

	printk("[Focal][Touch][INFO] Into fts_ctpm_auto_upgrade\n");
	fts_ctpm_auto_upgrade(client);

	err = request_threaded_irq(ftxxxx_ts->client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_SUSPEND, client->dev.driver->name,
		ftxxxx_ts);
#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	}
#endif

	return 0;
exit_err_sdev_register_fail:
exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ftxxxx_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ftxxxx_ts->pdata->reset);
#endif
#endif

err_create_wq_failed:
	if (ftxxxx_ts->reset_wq) {
		destroy_workqueue(ftxxxx_ts->reset_wq);
	}
	if (ftxxxx_ts->resume_wq) {
		destroy_workqueue(ftxxxx_ts->resume_wq);
	}

	if (ftxxxx_ts->usb_wq) {
		destroy_workqueue(ftxxxx_ts->usb_wq);
	}
exit_init_gpio:
	fts_un_init_gpio_hw(ftxxxx_ts);

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ftxxxx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

#ifdef CONFIG_PM


void ftxxxx_enable(int enable)
{
    if(enable) {
        ftxxxx_ts_resume(NULL);
    } else {
        ftxxxx_ts_suspend(NULL);
    }	
}

static void ftxxxx_ts_suspend(struct early_suspend *handler)
{
	printk("[Focal][Touch] %s : Touch suspend \n", __func__);
	uint8_t buf[2] = {0};

	struct ftxxxx_ts_data *ts = ftxxxx_ts;

	wake_lock(&ftxxxx_ts->wake_lock);

	mutex_lock(&ftxxxx_ts->g_device_mutex);

	if (ftxxxx_ts->suspend_flag) {

		focal_debug(DEBUG_VERBOSE, "[Focal][Touch] IC in suspend !! \n");

		mutex_unlock(&ftxxxx_ts->g_device_mutex);

		wake_unlock(&ftxxxx_ts->wake_lock);

		return;
	}
#ifdef FTS_GESTRUE/*zax 20140922*/
	has_gesture = false;
	ftxxxx_write_reg(ts->client, 0xd0, 0x01);
	if (ts->dclick == 1 && ts->gesture_all == 1) {
	        ftxxxx_write_reg(ts->client, 0xd1, 0x30);
		printk("[Focal][Touch] %s : enable double click\n", __func__);
		printk("[Focal][Touch] %s : enable gestrue \n", __func__);
	} else if (ts->dclick == 0 && ts->gesture_all == 1) {
		ftxxxx_write_reg(ts->client, 0xd1, 0x20);
		printk("[Focal][Touch] %s : disable double click\n", __func__);
		printk("[Focal][Touch] %s : enable gestrue \n", __func__);
	} else if (ts->dclick == 1 && ts->gesture_all == 0) {
		ftxxxx_write_reg(ts->client, 0xd1, 0x10);
		printk("[Focal][Touch] %s : enable double click\n", __func__);
		printk("[Focal][Touch] %s : disable gestrue \n", __func__);
	}else if (ts->dclick == 0 && ts->gesture_all == 0){
	        ftxxxx_irq_switch(false);
                buf[0] = 0xA5;
	        buf[1] = 0x03;
	        ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
		printk("[Focal][Touch] %s : disable double click\n", __func__);
		printk("[Focal][Touch] %s : disable gestrue \n", __func__);
	}

	if(ts->gesture_all == 1) {
		ftxxxx_write_reg(ts->client, 0xd2, ts->gesture_wec);
		if (ts->gesture_s)
			ftxxxx_write_reg(ts->client, 0xd5, 0x40);
		if (ts->gesture_v)
			ftxxxx_write_reg(ts->client, 0xd6, 0x10);
		if (ts->gesture_z)
			ftxxxx_write_reg(ts->client, 0xd7, 0x20);
	} else {
		printk("[Focal][Touch] %s : disable all gestrue \n", __func__);
	}
#else
	ftxxxx_irq_switch(false);
	buf[0] = 0xA5;
	buf[1] = 0x03;
	ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
#endif
	force_release_pos(ftxxxx_ts);

	ftxxxx_ts->suspend_flag = 1;

	mutex_unlock(&ftxxxx_ts->g_device_mutex);

	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}

static void ftxxxx_ts_resume(struct early_suspend *handler)
{

	queue_work(ftxxxx_ts->resume_wq, &ftxxxx_ts->resume_work);

	return;
}
#else
#define ftxxxx_ts_suspend		NULL
#define ftxxxx_ts_resume		NULL
#endif

static int ftxxxx_ts_remove(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	input_unregister_device(ftxxxx_ts->input_dev);

	cable_status_unregister_client(&cable_status_notifier);

#ifdef CONFIG_PM
	gpio_free(ftxxxx_ts->pdata->rst_gpio);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	ftxxxx_remove_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_release_apk_debug_channel();
#endif

	fts_un_init_gpio_hw(ftxxxx_ts);

	free_irq(client->irq, ftxxxx_ts);

	kfree(ftxxxx_ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ftxxxx_ts_id[] = {
	{ FTXXXX_NAME, 0 },
	{ }
};

/*MODULE_DEVICE_TABLE(i2c, ftxxxx_ts_id);*/

#ifdef CONFIG_OF
static struct of_device_id foceltech_match_table_sr[] = {
	{.compatible = "focaltech,ft3x27,sr", },
	{},
};

static struct of_device_id foceltech_match_table_er2[] = {
	{.compatible = "focaltech,ft3x27,er2", },
	{},
};

#else
#define foceltech_match_table NULL
#endif

static struct i2c_driver ftxxxx_ts_driver = {
	.probe = ftxxxx_ts_probe,
#ifdef FTS_GESTRUE
	.resume = ftxxxx_kernel_resume,
	.suspend = ftxxxx_kernel_suspend,	
#endif
	.remove = ftxxxx_ts_remove,
	.shutdown = ftxxxx_ts_shutdown,
	.id_table = ftxxxx_ts_id,
	.driver = {
		.name = FTXXXX_NAME,
		.owner = THIS_MODULE,
		.of_match_table = foceltech_match_table_sr,
/*#ifdef CONFIG_PM
		.pm				= &himax852xes_pm_ops,
#endif*/
	},
};

/*
static int __init ftxxxx_ts_init(void)
{
	int ret;
	printk("[Focal][Touch] %s : ftxxxx_ts_init !\n", __func__);
	ret = i2c_add_driver(&ftxxxx_ts_driver);
	if (ret) {
		printk(KERN_WARNING " [Focal][TOUCH_ERR] Adding ftxxxx driver failed "
			"(errno = %d)\n", ret);
	} else {
		printk("[Focal][Touch] %s : Successfully added driver %s\n", __func__,
			ftxxxx_ts_driver.driver.name);
	}
	return ret;
}
*/
/*
static void __init ftxxxx_ts_init_async(void *unused, async_cookie_t cookie)
{
	printk("[Focal][Touch] %s : ftxxxx_ts_init !\n", __func__);
	i2c_add_driver(&ftxxxx_ts_driver);
}

static int __init ftxxxx_ts_init(void)
{
	async_schedule(ftxxxx_ts_init_async, NULL);
	return 0;
}
*/

static int __init ftxxxx_ts_init(void)
{
#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	int err = 0;
	err = check_lcm_id();
	if (err < 0){
		printk("[Focal][Touch] LCM ID return probe\n");
		return err;
	}
#endif
        int ret = 0;
        printk("[Focal][Touch] %s : ftxxxx_ts_init !\n", __func__);

        printk("[Focal][Touch] system entry_mode = %d \n",entry_mode);

#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
	if(entry_mode != 1 && entry_mode != 4){
            printk("[Focal][Touch] Not in MOS and COS, don't need touch driver. \n");
            return ret;
	}
	// In COS mode
	 if(entry_mode == 4){
	 	cos_flag = true;
		printk("[Focal][Touch] : Touch in COS mode. \n");
	}
#else
	if(entry_mode != 1 ){
	    printk("[Focal][Touch] Not in MOS, don't need touch driver. \n");
	    return ret;
	}
#endif

	int hardware_id = 0;
        hardware_id=asustek_boardinfo_get(FUN_HARDWARE_ID);
        printk("[Focal][Touch] %s : hardware_id is %d \n", __func__, hardware_id);

        if (hardware_id == SR ||hardware_id == ER) {
	    printk("[Focal][Touch] %s : hardware_id is SR or ER1 !\n", __func__);
            ftxxxx_ts_driver.driver.of_match_table = foceltech_match_table_sr;
        }
	/*else if  (hardware_id == 0x03) {
            printk("[Focal][Touch] %s : hardware_id is ER2 !\n", __func__);
            ftxxxx_ts_driver_er2.driver.of_match_table = foceltech_match_table_er2;
        }
        */
        else {
            printk("[Focal][Touch] %s : hardware_id is ER2 or others !\n", __func__);
            ftxxxx_ts_driver.driver.of_match_table = foceltech_match_table_er2;
	}

        ret = i2c_add_driver(&ftxxxx_ts_driver);
        if (ret) {
                printk(KERN_WARNING " [Focal][TOUCH_ERR] Adding ftxxxx driver failed "
                        "(errno = %d)\n", ret);
        } else {
                printk("[Focal][Touch] %s : Successfully added driver %s\n", __func__,
                        ftxxxx_ts_driver.driver.name);
        }
        return ret;
}

static void __exit ftxxxx_ts_exit(void)
{
    i2c_del_driver(&ftxxxx_ts_driver);
}

module_init(ftxxxx_ts_init);
module_exit(ftxxxx_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
