/*
 * Support for spca700xa_16mpf Camera Sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __SPCA700XA_16MPF_H__
#define __SPCA700XA_16MPF_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>

//Move and modify from ov5693.h for firmware --- Wesley +++
#include <linux/wakelock.h>
//Move and modify from ov5693.h for firmware --- Wesley ---

#define V4L2_IDENT_SPCA700XA_16MPF 0001 //Peter, what is this?
#define SPCA700XA_16MPF_PIDH_ADDR 0x72CF
#define SPCA700XA_16MPF_PIDL_ADDR 0x72CE
#define SPCA700XA_16MPF_SENSOR_ID 0x7002 // Should get 0x7002 for SPCA7002


#define SPCA700XA_BAYER_ORDER V4L2_MBUS_FMT_UYVY8_1X16

#define MSG_LEN_OFFSET		2
#define MAX_FMTS		1

//parameter define+++
#define SENSOR_ISO_AUTO		0
#define SENSOR_ISO_50		1
#define SENSOR_ISO_100		2
#define SENSOR_ISO_200		3
#define SENSOR_ISO_400		4
#define SENSOR_ISO_800		5
#define SENSOR_ISO_1600         6

#define SENSOR_WDR_OFF  0
#define SENSOR_WDR_ON  1

#define SENSOR_EIS_OFF  0 //ASUS_BSP, Camera5.0
#define SENSOR_EIS_ON  1 //ASUS_BSP, Camera5.0

#define SENSOR_MOVING_OFF  0
#define SENSOR_MOVING_ON  1

#define SENSOR_AE_AWB_AF_UNLOCK  0
#define SENSOR_AE_AWB_UNLOCK  1
#define SENSOR_AE_LOCK  2
#define SENSOR_AWB_LOCK  3
#define SENSOR_AE_UNLOCK  4
#define SENSOR_AWB_UNLOCK  5
#define SENSOR_AE_AWB_LOCK  6

//parameter define---

//i2c r/w +++
#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */

struct sensor_reg {
	u16 addr;
	u16 val;
};
//i2c r/w --

//TODO: need to modify
#define SPCA700XA_16MPF_FOCAL_LENGTH_NUM	208	/*2.08mm*/
#define SPCA700XA_16MPF_FOCAL_LENGTH_DEM	100
#define SPCA700XA_16MPF_F_NUMBER_DEFAULT_NUM	24
#define SPCA700XA_16MPF_F_NUMBER_DEM	10

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define SPCA700XA_16MPF_FOCAL_LENGTH_DEFAULT 0xD00064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define SPCA700XA_16MPF_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define SPCA700XA_16MPF_F_NUMBER_RANGE 0x180a180a

//Move and modify from ov5693.h for firmware --- Wesley +++
enum iCatch_fw_update_status{
	ICATCH_FW_NO_CMD,
	ICATCH_FW_IS_BURNING,
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;

enum iCatch_flash_type{
	ICATCH_FLASH_TYPE_ST,
	ICATCH_FLASH_TYPE_SST,
};
static enum iCatch_flash_type flash_type = ICATCH_FLASH_TYPE_ST;
//Move and modify from ov5693.h for firmware --- Wesley ---

//TODO: add more resolution later
/* Supported resolutions */
enum {
	SPCA700XA_16MPF_RES_QCIF,
	SPCA700XA_16MPF_RES_QVGA,
	SPCA700XA_16MPF_RES_CIF,
	SPCA700XA_16MPF_RES_VGA,
	SPCA700XA_16MPF_RES_720P,
	SPCA700XA_16MPF_RES_960P,
	SPCA700XA_16MPF_RES_2M,
	SPCA700XA_16MPF_RES_1080P,
	SPCA700XA_16MPF_RES_5MP,
	SPCA700XA_16MPF_RES_8MP,
	SPCA700XA_16MPF_RES_13MP,
	SPCA700XA_16MPF_RES_16MP,
};
#define SPCA700XA_16MPF_RES_16MP_SIZE_H		4608
#define SPCA700XA_16MPF_RES_16MP_SIZE_V		3456
#define SPCA700XA_16MPF_RES_13MP_SIZE_H		4160
#define SPCA700XA_16MPF_RES_13MP_SIZE_V		3120
#define SPCA700XA_16MPF_RES_8MP_SIZE_H		3264
#define SPCA700XA_16MPF_RES_8MP_SIZE_V		2448
#define SPCA700XA_16MPF_RES_5MP_SIZE_H		2592
#define SPCA700XA_16MPF_RES_5MP_SIZE_V		1944
#define SPCA700XA_16MPF_RES_1080P_SIZE_H		1920
#define SPCA700XA_16MPF_RES_1080P_SIZE_V		1080
#define SPCA700XA_16MPF_RES_2M_SIZE_H		1600
#define SPCA700XA_16MPF_RES_2M_SIZE_V		1200
#define SPCA700XA_16MPF_RES_960P_SIZE_H		1280
#define SPCA700XA_16MPF_RES_960P_SIZE_V		960
#define SPCA700XA_16MPF_RES_720P_SIZE_H		1280
#define SPCA700XA_16MPF_RES_720P_SIZE_V		720
#define SPCA700XA_16MPF_RES_480P_SIZE_H		768
#define SPCA700XA_16MPF_RES_480P_SIZE_V		480
#define SPCA700XA_16MPF_RES_VGA_SIZE_H		640
#define SPCA700XA_16MPF_RES_VGA_SIZE_V		480
#define SPCA700XA_16MPF_RES_QVGA_SIZE_H		320
#define SPCA700XA_16MPF_RES_QVGA_SIZE_V		240
#define SPCA700XA_16MPF_RES_CIF_SIZE_H          352
#define SPCA700XA_16MPF_RES_CIF_SIZE_V          288
#define SPCA700XA_16MPF_RES_QCIF_SIZE_H		176
#define SPCA700XA_16MPF_RES_QCIF_SIZE_V		144

struct spca700xa_16mpf_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int real_model_id;
	unsigned int res;
	struct v4l2_ctrl *last_run_mode;

	struct mutex input_lock; /* serialize sensor's ioctl */
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *run_mode;
//Move and modify from ov5693.h for firmware --- Wesley +++
	struct wake_lock *fw_update_wakelock;
//Move and modify from ov5693.h for firmware --- Wesley ---

	struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct spca700xa_16mpf_format_struct {
	u8 *desc;
	u32 pixelformat;
	struct regval_list *regs;
};

struct spca700xa_16mpf_res_struct {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	int skip_frames;
	int row_time;
	bool used;
	struct regval_list *regs;
};

struct spca700xa_16mpf_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

struct old_fmt{
	int w;
	int h;
	unsigned int  res;
	__u32 code;
};

/*
 * Modes supported by the spca700xa_16mpf driver.
 * Please, keep them in ascending order.
 */
static struct spca700xa_16mpf_res_struct spca700xa_16mpf_res[] = {
#if 0
	{
	.desc	= "QCIF",
	.res	= SPCA700XA_16MPF_RES_QCIF,
	.width	= 176,
	.height	= 144,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
        {
        .desc   = "QVGA",
        .res    = SPCA700XA_16MPF_RES_QVGA,
        .width  = 320,
        .height = 240,
        .fps    = 30,
        .used   = 0,
        .regs   = NULL,
        .skip_frames = 0,
        },
        {
        .desc   = "CIF",
        .res    = SPCA700XA_16MPF_RES_CIF,
        .width  = 352,
        .height = 288,
        .fps    = 30,
        .used   = 0,
        .regs   = NULL,
        .skip_frames = 0,
        },
#endif
	{
	.desc	= "VGA",
	.res	= SPCA700XA_16MPF_RES_VGA,
	.width	= SPCA700XA_16MPF_RES_VGA_SIZE_H,
	.height	= SPCA700XA_16MPF_RES_VGA_SIZE_V,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
/*
	{
	.desc	= "480p",
	.res	= SPCA700XA_16MPF_RES_480P,
	.width	= 768,
	.height	= 480,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
*/
#if 0
	{
	.desc	= "720p",
	.res	= SPCA700XA_16MPF_RES_720P,
	.width	= SPCA700XA_16MPF_RES_720P_SIZE_H,
	.height	= SPCA700XA_16MPF_RES_720P_SIZE_V,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},

	{
	.desc	= "960P",
	.res	= SPCA700XA_16MPF_RES_960P,
	.width	= 1280,
	.height	= 960,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
#endif
#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	{
	.desc	= "2M",
	.res	= SPCA700XA_16MPF_RES_2M,
	.width	= SPCA700XA_16MPF_RES_2M_SIZE_H,
	.height	= SPCA700XA_16MPF_RES_2M_SIZE_V,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
#endif
#if 0
	{
	.desc	= "1080P",
	.res	= SPCA700XA_16MPF_RES_1080P,
	.width	= SPCA700XA_16MPF_RES_1080P_SIZE_H,
	.height	= SPCA700XA_16MPF_RES_1080P_SIZE_V,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "5MP",
	.res	= SPCA700XA_16MPF_RES_5MP,
	.width	= SPCA700XA_16MPF_RES_5MP_SIZE_H,
	.height = SPCA700XA_16MPF_RES_5MP_SIZE_V,
	.fps	= 30,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},

	{
	.desc	= "8MP",
	.res	= SPCA700XA_16MPF_RES_8MP,
	.width	= SPCA700XA_16MPF_RES_8MP_SIZE_H,
	.height	= SPCA700XA_16MPF_RES_8MP_SIZE_V,
	.fps	= 28,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},

	{
	.desc	= "13MP",
	.res	= SPCA700XA_16MPF_RES_13MP,
	.width	= SPCA700XA_16MPF_RES_13MP_SIZE_H,
	.height = SPCA700XA_16MPF_RES_13MP_SIZE_V,
	.fps	= 28,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
	{
	.desc	= "16MP",
	.res	= SPCA700XA_16MPF_RES_16MP,
	.width	= 4608,
	.height	= 3456,
	.fps	= 12,
	.used	= 0,
	.regs	= NULL,
	.skip_frames = 0,
	},
#endif
};
#define N_RES (ARRAY_SIZE(spca700xa_16mpf_res))

static const struct i2c_device_id spca700xa_16mpf_id[] = {
	{"spca700xa_16mpf", 0},
	{}
};

#endif
