/*
 * Support for spca700xa_16mpf Camera Sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-controls_intel.h>
#include <linux/debugfs.h>
//#include <media/v4l2-chip-ident.h>//Clark mark for build error
#include <linux/switch.h> //ASUS_BSP Wesley, Add for ISP firmware version in setting
//FW_BSP++, For lode from host
#include "app_i2c_lib_icatch_f.h"
//FW_BSP--, For load from host
//Move and modify from ov5693.c for firmware --- Wesley +++
//FW_BSP++
#include <linux/proc_fs.h>	//ASUS_BSP+++, add for ISP firmware update
#include <linux/seq_file.h>
//FW_BSP--
//Move and modify from ov5693.c for firmware --- Wesley ---
#include <linux/platform_data/platform_camera_module.h>
//#include "i7002a.h"
#include "spca700xa_16mpf.h"
#define SPCA700XA_16MPF_DRIVER_NAME "spca700xa_16mpf" //Clark add
#define to_spca700xa_16mpf_sensor(sd) container_of(sd, struct spca700xa_16mpf_device, sd)
static int spca700xa_16mpf_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt);
int LSC_DQcnt_f = 0;
#define SKIP_FIRST_COMMAND 0
#if SKIP_FIRST_COMMAND
static int first_on = 0;
#endif

/*++ new ATD request for read sensor name ++*/
#define CAMERA_PROC_MODULE_FILE "driver/FrontModule"
static unsigned char *sensor_name = SPCA700XA_16MPF_DRIVER_NAME;
static struct proc_dir_entry *camera_proc_module_file;
static int skip_cnt = 0;
static struct old_fmt old_f;
static int mHDR = 0;
static int mNightMode = 0;
static int mHDRMode = 0;

static int camera_proc_module_read(struct seq_file *buf, void *v){
	seq_printf(buf, "%s\n", sensor_name);
	return 0;
}

static int camera_proc_module_open(struct inode *inode, struct  file *file) {
	return single_open(file, camera_proc_module_read, NULL);
}

static const struct file_operations camera_module_fops = {
		.owner = THIS_MODULE,
		.open = camera_proc_module_open,
		.read = seq_read,
};

static void create_camera_proc_file(void)
{
	camera_proc_module_file = proc_create(CAMERA_PROC_MODULE_FILE, 0666, NULL, &camera_module_fops);
	if(camera_proc_module_file){
		printk("proc module file create sucessed!\n");
	}
	else{
		printk("proc module file create failed!\n");
	}
}
/*-- new ATD request for read sensor name --*/

//Add for ATD read camera status+++
static int ATD_spca700xa_16mp_status = 0;  //Add for ATD read camera status
static int iso_setting = 0;
static int retry_time = 3;
//static int sensor_mode = 0;

static int lastSceneMode = 0;
static int lastEffect = 0;
static int lastEffectAura = 0;
static int lastEV = -100;// change to a big vaule because -1 is in the EV range

static int lastWhiteBalance = -1;

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
static int camera_id =0;
#endif

extern int pltfrm_camera_module_init_gpio(struct v4l2_subdev *sd);
extern int pltfrm_camera_module_set_pinctrl_state(struct v4l2_subdev *sd, struct pinctrl_state *state);
extern int pltfrm_camera_module_release_gpio(struct v4l2_subdev *sd);
extern int pltfrm_camera_module_set_pm_state(struct v4l2_subdev *sd, enum device_pm_state state);

static ssize_t spca700xa_16mp_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get spca700xa_16mp status (%d) !!\n", __func__, ATD_spca700xa_16mp_status);
   	//Check sensor connect status, just do it  in begining for ATD camera status

	return sprintf(buf,"%d\n", ATD_spca700xa_16mp_status);
}

static DEVICE_ATTR(spca700xa_16mp_status, S_IRUGO,spca700xa_16mp_show_status,NULL);

static struct attribute *spca700xa_16mpf_attributes[] = {
	&dev_attr_spca700xa_16mp_status.attr,
	NULL
};
static int spca700xa_16mpf_dbgfs_init(void)
{
	struct dentry *debugfs_dir;
	debugfs_dir = debugfs_create_dir("camera1", NULL);
	debugfs_create_u32("camera_status",
		0644, debugfs_dir, &ATD_spca700xa_16mp_status);

	return 0;
}
//Add for ATD read camera status---

//Move and modify from ov5693.c for firmware +++ Wesley
#define SENSOR_WAIT_MS          0 /* special number to indicate this is wait time require */
#define SENSOR_TABLE_END        1 /* special number to indicate this is end of table */
#define SENSOR_BYTE_WRITE       2
#define SENSOR_WORD_WRITE       3
#define SENSOR_MASK_BYTE_WRITE  4
#define SENSOR_MASK_WORD_WRITE  5
#define SEQ_WRITE_START         6
#define SEQ_WRITE_END           7

#define SENSOR_MAX_RETRIES      3 /* max counter for retry I2C access */
#if defined (CONFIG_Z370C) || defined (CONFIG_Z370CG)
static char *FW_BIN_FILE_WITH_PATH_TK = "/system/etc/firmware/camera/TKBOOT.BIN";
static char *FW_BIN_FILE_WITH_PATH_OV = "/system/etc/firmware/camera/OVBOOT.BIN";
#define MAIN_CAMERA_ID_TK	0
#define MAIN_CAMERA_ID_OV	1
#endif 
static char  *FW_BIN_FILE_WITH_PATH = "/system/etc/firmware/camera/BOOT.BIN";


//ASUS_BSP++, For load from host
#define FW_HEADER_SIZE 16
extern int spca700xa_SPI_write(UINT8 *, UINT32);
extern int spca700xa_SPI_read(UINT8 *, UINT32); //ASUS_BSP++, for calibration
extern int spca700xa_SPI_clk_control(bool reset);
extern int spca700xa_SPI_off(bool off);

//ASUS_BSP--, For load from host

#define ISP_SDEV_NAME        "camera"  //Add for ISP firmware version in setting
struct switch_dev 	ISP_sdev_f;  //Add for ISP firmware version in setting

// i7002a firmware +++
#define SPI_CMD_BYTE_READ 	0x03
#define SPI_CMD_RD_ID 		0x9F
#define SPI_CMD_WRT_EN		0x06
#define SPI_CMD_BYTE_PROG 	0x02
#define SPI_CMD_RD_STS		0x05
#define SPI_CMD_BYTE_PROG_AAI	0xAD
#define SPI_CMD_WRT_STS_EN	0x50
#define SPI_CMD_WRT_STS 	0x01
#define SPI_CMD_WRT_DIS 	0x04
#define SPI_CMD_ERASE_ALL	0xC7

#define update_write_byte 64

/* iCatch Camera Firmware Header
 * It locates on the end of the bin file.
 * Total: 32 bytes.
 * byte[0] ~ byte[7]: 0xFF's
 * byte[8] ~ byte[11]: Compensation for Overall Checksum
 * byte[12] ~ byte[15]: Overall Checksum

 * byte[16] ~ byte[20]: 0xFF's
 * byte[21]: Front Format
 * byte[22]: Rear Lane#
 * byte[23]: Front Lane#
 * byte[24] ~ byte[25]: Rear Sensor ID
 * byte[26] ~ byte[27]: Front sensor ID
 * byte[28] ~ byte[31]: FW Version
 */

#define BIN_FILE_HEADER_SIZE 32
#define NEED_UPDATE          0x0
#define UPDATE_UNNECESSARY   0x1

static int fw_page_count = -1;
static int total_page_count = -1;
static char DEFAULT_CALIBOPT_FILE_WITH_PATH[] = "/system/bin/calibration_option.BIN";
static char FACTORY_CALIBOPT_FILE_WITH_PATH[] = "/factory/calibration_option.BIN";

//ASUS_BSP+++, add for calibration
u16 g_is_calibration_f = 0;
u8 *pIspFW_g_f=NULL, *pCalibOpt_g_f=NULL, *p3acali_g_f=NULL, *pLsc_g_f=NULL, *pLscdq_g_f=NULL;
static char *CALIBOPT_FILE_WITH_PATH = FACTORY_CALIBOPT_FILE_WITH_PATH;
static char IIIACALI_FILE_WITH_PATH[] = "/factory/3ACALI_F.BIN";
static char LSC_FILE_WITH_PATH[] = "/factory/LSC_F.BIN";
static char LSCDQ_FILE_WITH_PATH[] = "/factory/LSC_DQ_F.BIN";
#define RES_3ACALI_HEADER_SIZE	8
#define RES_LSC_HEADER_SIZE		24
#define RES_LSCDQ_HEADER_SIZE	16
//ASUS_BSP---, add for calibration

bool FW_BIN_FILE_F=false;

int SPI_ret_F=0;
UINT32 bootbin_size_g_f=0;

static u32 version_num_in_bin = 0xffffffff;
struct v4l2_subdev *fw_sd_f;
static int spca700xa_16mpf_s_power(struct v4l2_subdev *sd, int power);
static int spca700xa_16mpf_s_config(struct v4l2_subdev *sd);

//Move and modify from ov5693.c for firmware --- Wesley

// iCatch i2c r/w +
int sensor_write_reg_f(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = 0x3C;//client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	/*printk("i2c transfer\n");*/
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s : i2c transfer failed, retrying addr = %x val = %x\n", __FUNCTION__, addr, val);
		pr_err("%s : i2c transfer failed, msg.addr %x, err= 0x%d\n", __FUNCTION__, msg.addr, err);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = -EINVAL;
	}

	return err;
}

static int sensor_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 0;

              return 0;
	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	msg.addr = 0x3C;//client->addr;
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		//pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		      // addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

int sensor_read_reg_f(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = 0x3C;//client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = 0x3C;//client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2) {
		pr_info("spca700xa_16mpf: %s err: %d\n", __func__, err);
		return -EINVAL;
	}

	memcpy(val, data+2, 1);
	*val =*val&0xff;

	return 0;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
	  case 0:
	  // possibly no address, some focusers use this
	  break;

	  // cascading switch
	  case 32:
	    pBuf[count++] = (u8)((value>>24) & 0xFF);
	  case 24:
	    pBuf[count++] = (u8)((value>>16) & 0xFF);
	  case 16:
	    pBuf[count++] = (u8)((value>>8) & 0xFF);
	  case 8:
	    pBuf[count++] = (u8)(value & 0xFF);
	    break;

	  default:
	    printk("Unsupported Bit Width %d\n", width);
	    break;
	}
	return count;

}

int sensor_write_table_f(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;
	unsigned char data[10];
	u16 datasize = 0;

	//for (next = table; next->addr != SENSOR_TABLE_END; next++) {
	next = table;
	while (next->addr != SENSOR_TABLE_END) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			next +=1;
			continue;
		}
		if (next->addr == SEQ_WRITE_START) {
			next += 1;
			while (next->addr !=SEQ_WRITE_END) {
				if (datasize==0) {
					datasize += build_sequential_buffer(&data[datasize], 16, next->addr);
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				}
				else
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				if (datasize==10) {
					sensor_sequential_write_reg(client, data, datasize);
					datasize = 0;
				}
				next += 1;
			}
			sensor_sequential_write_reg(client, data, datasize); //flush out the remaining buffer.
			datasize = 0;
		}
		else {
			val = next->val;

			err = sensor_write_reg_f(client, next->addr, val);
			if (err) {
				printk("%s(%d): isensor_write_reg_f ret= 0x%x\n", __FUNCTION__, __LINE__, err);
				return err;
			}
		}
		next += 1;
	}
	return 0;
}
// iCatch i2c r/w -

static void dumpReg(struct i2c_client *client, u16 addr) {
	u16 retvalue_l = 0;
	sensor_read_reg_f(client, addr, &retvalue_l);
	printk("dumpReg 0x%04X  %x\n",addr ,retvalue_l&0xffff);
}

static void dumpReg_range(struct i2c_client *client, u16 startaddr, u16 endaddr) {
	u16 retvalue_l = 0;
	u16 addr;
	for(addr = startaddr; addr <= endaddr; addr++) {
		sensor_read_reg_f(client, addr, &retvalue_l);
		printk("dumpReg 0x%04X  %x\n",addr ,retvalue_l&0xffff);
	}
}

//Move and modify from ov5693.h for firmware --- Wesley +++
int I2C_F_SPIInit(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	//  I2CDataWrite(0x0026,0xc0);
	//  I2CDataWrite(0x4051,0x01); /* spien */
	//  I2CDataWrite(0x40e1,0x00); /* spi mode */
	//  I2CDataWrite(0x40e0,0x11); /* spi freq */
	struct sensor_reg SPI_init_seq[] = {
		{0x0026, 0xc0},
		{0x4051, 0x01},
		{0x40e1, 0x00},
		{0x40e0, 0x11},
		{SENSOR_TABLE_END, 0x0000}
	};
	printk("============ Wesley I2C_F_SPIInit ========== I2C_F_SPIInit ======== In =======\n");
	ret = sensor_write_table_f(client, SPI_init_seq);
	if(ret) {
		printk("%s: init fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_F_SPIFlashPortRead(struct v4l2_subdev *sd)
{
	u16 ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	// ret = hsI2CDataRead(0x40e4);
      sensor_read_reg_f(client, 0x40e4, &ret);
	/* polling SPI state machine ready */
#if 0
    if (I2C_SPIFlashPortWait() != SUCCESS) {
        return 0;
    }
#endif
	//ret = hsI2CDataRead(0x40e5);
      sensor_read_reg_f(client, 0x40e5, &ret);

    return (u32)ret;
}

u32 I2C_SPIFlashRead_f(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 err = 0;
	u32 i, size=0;
	u32 pageSize = 0x100;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	addr = addr * pageSize;
	size = pages*pageSize;

	// I2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7, 0x00);
	// I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);               /* Write one byte command*/
	sensor_write_reg_f(client, 0x40e3, SPI_CMD_BYTE_READ);
	// I2C_SPIFlashPortWrite((u8)(addr >> 16));               /* Send 3 bytes address*/
	// I2C_SPIFlashPortWrite((u8)(addr >> 8));
	// I2C_SPIFlashPortWrite((u8)(addr));
	sensor_write_reg_f(client, 0x40e3, (u8)(addr >> 16));
	sensor_write_reg_f(client, 0x40e3, (u8)(addr >> 8));
	sensor_write_reg_f(client, 0x40e3, (u8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_F_SPIFlashPortRead(sd);
		if((i%256)==0)
			pr_debug("%s: page count: 0x%x\n", __FUNCTION__, (i/256));
		pbuf ++;
	}

	sensor_write_reg_f(client, 0x40e7, 0x01);

	return err;
}

u32 I2C_F_SPIFlashReadId(struct v4l2_subdev *sd)
{
	u8 id[3];
	u32 ID;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	/*Wesley 0405 Note
		//I2C_F_SPIFlashWrEnable seq:
		//
		//sensor_write_reg_f(client, 0x40e7,0x00);
		//sensor_write_reg_f(client, 0x40e3,SPI_CMD_RD_ID);
		//sensor_write_reg_f(client, 0x40e7,0x01);
	*/

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7,0x00);

	//err = I2C_SPIFlashPortWrite(SPI_CMD_RD_ID); /*read ID command*/
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_RD_ID);

#if 0
	if (err != SUCCESS) {
		printf("Get serial flash ID failed\n");
		return 0;
	}
#endif

	id[0] = I2C_F_SPIFlashPortRead(sd);    /* Manufacturer's  ID */
	id[1] = I2C_F_SPIFlashPortRead(sd);    /* Device ID          */
	id[2] = I2C_F_SPIFlashPortRead(sd);    /* Manufacturer's  ID */

	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);

	pr_debug("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((u32)id[0] << 16) | ((u32)id[1] << 8) | \
    ((u32)id[2] << 0);

	return ID;
}

static const u32 stSpiIdInfo[29] =
{
	/*EON*/
	0x001C3117,
	0x001C2016,
	0x001C3116,
	0x001C3115,
	0x001C3114,
	0x001C3113,
	/*Spansion*/
	0x00012018,
	0x00010216,
	0x00010215,
	0x00010214,
	/*ST*/
	0x00202018,
	0x00202017,
	0x00202016,
	0x00202015,
	0x00202014,
	/*MXIC*/
	0x00C22018,
	0x00C22017,
	0x00C22016,
	0x00C25e16,
	0x00C22015,
	0x00C22014,
	0x00C22013,
	/*Winbond*/
	0x00EF3017,
	0x00EF3016,
	0x00EF3015,
	0x00EF3014,
	0x00EF3013,
	0x00EF5013,
	/*Fail*/
	0x00000000,
};

static const u32 sstSpiIdInfo[6] =
{
	/*ESMT*/
	0x008C4016,
	/*SST*/
	0x00BF254A,
	0x00BF2541,
	0x00BF258E,
	0x00BF258D,
	/*Fail*/
	0x00000000,
};

u32
BB_F_SerialFlashTypeCheck(
	u32 id
)
{
	u32 i=0;
	u32 fullID = 1;
	u32 shift = 0, tblId, type = 0;

	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("SST type serial flash\n");
			type = 2;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("ST Type serial flash\n");
			type = 1;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}

int I2C_F_SPIFlashWrEnable(struct v4l2_subdev *sd)
{
	int ret = 0;
	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct sensor_reg I2C_F_SPIFlashWrEnable_seq[] = {
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table_f(client, I2C_F_SPIFlashWrEnable_seq);

	if(ret) {
		printk("%s: fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_F_SPIStsRegRead(struct v4l2_subdev *sd)
{
	u32 ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_RD_STS);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_RD_STS);
	ret = I2C_F_SPIFlashPortRead(sd);

	// hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);

	return ret;
}

void I2C_F_SPITimeOutWait(struct v4l2_subdev *sd, u32 poll, u32 *ptimeOut)
{
    /* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
    u32 sts;
    u32 time = 0;
    while (1) {
        sts = I2C_F_SPIStsRegRead(sd);
        if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
            break;
        }
        time ++;
        if( *ptimeOut < time ) {
            printk("iCatch: TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
            break;
        }
    }
}

int I2C_F_SPIStChipErase(struct v4l2_subdev *sd)
{
	u32 timeout;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("iCatch: ST Chip Erasing...\n");

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	sensor_write_reg_f(client, 0x40e3,0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);

	ret = I2C_F_SPIFlashWrEnable(sd);
	if (ret) {
		printk("iCatch: ST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_F_SPITimeOutWait(sd, 0x01, &timeout);
#if 0
	ros_thread_sleep(1);
#endif
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);
	printk("iCatch: ST Chip Erased\n");
	return 0;
}

int I2C_F_SPISstChipErase(struct v4l2_subdev *sd)
{
	u32 timeout;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("iCatch: SST Chip Erasing...\n");

	ret = I2C_F_SPIFlashWrEnable(sd);
	if (ret) {
		printk("iCatch: SST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN); /*Write Status register command*/
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg_f(client, 0x40e7,0x01);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(0x02);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_STS);
	sensor_write_reg_f(client, 0x40e3,0x02);
	sensor_write_reg_f(client, 0x40e7,0x01);

	I2C_F_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_ERASE_ALL);
	sensor_write_reg_f(client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_F_SPITimeOutWait(sd, 0x01, &timeout);
	//msleep(500);
	printk("iCatch: SST Chip Erased\n");
	return 0;
}

void writeUpdateProgresstoFile_f(int page_left, int total_page_num)
{
	struct file *fp_progress = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_progress[4];
	int percentage = 0;

	percentage = 100 * (total_page_num - page_left + 1)/total_page_num;

	if(page_left % 32 == 1){
		pr_debug("%s: page:0x%x; percentage= %d;\n", __FUNCTION__, page_left, percentage);
		fp_progress = filp_open("/data/isp_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
		if ( IS_ERR_OR_NULL(fp_progress) ){
			filp_close(fp_progress, NULL);
			printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_progress");
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
		if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
			sprintf(str_progress, "%d\n", percentage);
			fp_progress->f_op->write(fp_progress,
				str_progress,
				strlen(str_progress),
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_progress, NULL);
	}
}


u32 seqI2CDataWrite_f(struct i2c_client *client, u32 a_InputAddr, u8* pBuf)
{
	int i, err;
	struct i2c_msg msg;
	unsigned char data[update_write_byte+2];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0]=  (a_InputAddr & 0xff00)>>8;
	data[1]=  a_InputAddr & 0x00ff;
	for(i=0; i < update_write_byte; i++) {
		data[i+2]= *(pBuf+i) & 0x00ff;
	}
/*
	data[2]=  *(pBuf) & 0x00ff;
	data[3]=  *(pBuf+1) & 0x00ff;
	data[4]=  *(pBuf+2) & 0x00ff;
	data[5]=  *(pBuf+3) & 0x00ff;
*/
	msg.addr = 0x3C;//client->addr;
	msg.flags = 0;
	msg.len = update_write_byte+2;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("%s : i2c transfer failed, msg.addr %x, err= 0x%x\n", __FUNCTION__, msg.addr, err);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = -EINVAL;
	}

	return err;
}


void I2C_F_7002DmemWr(
	struct v4l2_subdev *sd,
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, bank;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	bank = 0x40+bankNum;
	//I2CDataWrite(0x10A6,bank);
	sensor_write_reg_f(client, 0x10A6,bank);

	for(i=0;i<byteNum;i+=update_write_byte)
	{
		seqI2CDataWrite_f(client, (0x1800+i),(pbuf+i)); /* sequentially write DMEM */
	}

	bank = 0x40 + ((bankNum+1)%2);
	//hsI2CDataWrite(0x10A6,bank);
	sensor_write_reg_f(client, 0x10A6,bank);
}

u32 I2C_F_SPIFlashWrite_DMA(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
    u32 i, err = 0;
    u32 pageSize = 0x100, size;
    u32 rsvSec1, rsvSec2;
    u32 dmemBank = 0;
    u32 chk1=0;
    u16 temp, chk2=0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

    rsvSec1 = pages*pageSize - 0x7000;
    rsvSec2 = pages*pageSize - 0x1000;
    addr = addr * pageSize;

    /* Set DMA bytecnt as 256-1 */
    //I2CDataWrite(0x4170,0xff);
    sensor_write_reg_f(client, 0x4170,0xff);
    //I2CDataWrite(0x4171,0x00);
    sensor_write_reg_f(client, 0x4171,0x00);
    //I2CDataWrite(0x4172,0x00);
    sensor_write_reg_f(client, 0x4172,0x00);

    /* Set DMA bank & DMA start address */
    //I2CDataWrite(0x1084,0x01);
    sensor_write_reg_f(client, 0x1084,0x01);
    //I2CDataWrite(0x1080,0x00);
    sensor_write_reg_f(client, 0x1080,0x00);
    //I2CDataWrite(0x1081,0x00);
    sensor_write_reg_f(client, 0x1081,0x00);
    //I2CDataWrite(0x1082,0x00);
    sensor_write_reg_f(client, 0x1082,0x00);

    /* enable DMA checksum and reset checksum */
    //I2CDataWrite(0x4280,0x01);
    sensor_write_reg_f(client, 0x4280,0x01);
    //I2CDataWrite(0x4284,0x00);
    sensor_write_reg_f(client, 0x4284,0x00);
    //I2CDataWrite(0x4285,0x00);
    sensor_write_reg_f(client, 0x4285,0x00);
    //I2CDataWrite(0x4164,0x00);
    sensor_write_reg_f(client, 0x4164,0x00);

    size = pages * pageSize;
    for(i=0;i<size;i++)
    {
	if((i>=rsvSec2) || (i <rsvSec1))
	{
		chk1 += *(pbuf+i);
	}
	if(chk1>=0x10000)
	{
		chk1 -= 0x10000;
	}
   }

    while( pages ) {
	if((pages%0x40)==0)
	{
		pr_debug("page:0x%x",pages);
	}
	if((addr>=rsvSec1) && (addr <rsvSec2))
	{
		addr += 0x1000;
		pbuf += 0x1000;
		pages -= 0x10;
		continue;
	}
	if((pages==1))
	 {
		for (i = 0; i < pageSize ; i++) {
			pr_debug("%2x ",*(pbuf+i));
			if((i%0x10)==0x0f) pr_debug("\n");
		}
	}

    	dmemBank = pages % 2;
    	//I2CDataWrite(0x1081,dmemBank*0x20);
	sensor_write_reg_f(client, 0x1081,dmemBank*0x20);
    	//I2CDataWrite(0x1084,(1<<dmemBank));
	sensor_write_reg_f(client, 0x1084,(1<<dmemBank));
    	I2C_F_7002DmemWr(sd,dmemBank,pageSize,pbuf);

     	I2C_F_SPIFlashWrEnable(sd);
     	//I2CDataWrite(0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e7,0x00);
     	//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);               /* Write one byte command*/
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_BYTE_PROG);
     	//I2C_SPIFlashPortWrite((UINT8)(addr >> 16));               /* Send 3 bytes address*/
	sensor_write_reg_f(client, 0x40e3,(u8)(addr >> 16));
     	//I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
	sensor_write_reg_f(client, 0x40e3,(u8)(addr >> 8));
     	//I2C_SPIFlashPortWrite((UINT8)(addr));
	sensor_write_reg_f(client, 0x40e3,(u8)(addr));

    	//I2CDataWrite(0x4160,0x01);
	sensor_write_reg_f(client, 0x4160,0x01);
    	//tmrUsWait(100);/* wait for DMA done */
	udelay(100);
    	//I2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x01);
    	pbuf += pageSize;
    	addr += pageSize;
    	pages --;
    }

    //tmrUsWait(500);/* wait for DMA done */
    udelay(500);

    //temp = hsI2CDataRead(0x4285);
    sensor_read_reg_f(client, 0x4285, &temp);
    //chk2 = hsI2CDataRead(0x4284);
    sensor_read_reg_f(client, 0x4284, &chk2);
    chk2 = chk2 | (temp<<8);
    printk("checksum: 0x%x 0x%x\n",chk1,chk2);

    return err;
}

void I2C_F_SPISstStatusWrite(struct v4l2_subdev *sd, u8 dat)
{
	u32 timeout, poll;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	I2C_F_SPIFlashWrEnable(sd);

	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg_f(client, 0x40e7,0x01);

	// hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	//I2C_SPIFlashPortWrite(dat);
	//hsI2CDataWrite(0x40e7,0x01);
	sensor_write_reg_f(client, 0x40e7,0x00);
	sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_STS);
	printk("%s: dat=%d\n", __FUNCTION__, dat);
	sensor_write_reg_f(client, 0x40e3,dat);
	printk("%s: dat=%d; Done.\n", __FUNCTION__, dat);
	sensor_write_reg_f(client, 0x40e7,0x01);

	poll = 0x01;
#if 0
	if( spiDev.bus != SPI_1BIT_MODE ) {/* 1 bit mode */
		poll = 0x80;
	} else {
		poll = 0x01;
	}
#endif
    timeout = 100000;
    I2C_F_SPITimeOutWait(sd, poll, &timeout);
    //msleep(500);
    return;
}

u32 I2C_F_SPISstFlashWrite(
	struct v4l2_subdev *sd,
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 timeout = 100000;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	addr = addr * pageSize;

	printk("iCatch: SST type writing...\n");
	I2C_F_SPISstStatusWrite(sd, 0x40);

	total_page_count = (int)pages;

	while( pages ) {
		fw_page_count = (int)pages;
		writeUpdateProgresstoFile_f(fw_page_count, total_page_count);

		I2C_F_SPIFlashWrEnable(sd);
		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg_f(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI); /* Write one byte command*/
		sensor_write_reg_f(client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
		//I2C_SPIFlashPortWrite((UINT8)(addr >> 16)); /* Send 3 bytes address*/
		sensor_write_reg_f(client, 0x40e3,(u8)(addr >> 16));
		//I2C_SPIFlashPortWrite((UINT8)(addr >> 8));
		sensor_write_reg_f(client, 0x40e3,(u8)(addr >> 8));
		//I2C_SPIFlashPortWrite((UINT8)(addr));
		sensor_write_reg_f(client, 0x40e3,(u8)(addr));
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg_f(client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//I2C_SPIFlashPortWrite(*pbuf);
		sensor_write_reg_f(client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg_f(client, 0x40e7,0x01);
		timeout = 100000;
		I2C_F_SPITimeOutWait(sd, 0x01,&timeout);

		for (i = 2; i < pageSize ; i = i+2) {
			//hsI2CDataWrite(0x40e7,0x00);
			sensor_write_reg_f(client, 0x40e7,0x00);
			//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);
			sensor_write_reg_f(client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg_f(client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg_f(client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			// hsI2CDataWrite(0x40e7,0x01);
			sensor_write_reg_f(client, 0x40e7,0x01);
			timeout = 100000;
			I2C_F_SPITimeOutWait(sd, 0x01,&timeout);
		}

		// hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg_f(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg_f(client, 0x40e7,0x01);

		addr += pageSize;
		pages --;

		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg_f(client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		sensor_write_reg_f(client, 0x40e3,SPI_CMD_WRT_DIS);
		//hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg_f(client, 0x40e7,0x01);
	}
	printk("iCatch: SST type writing Done.\n");
	return err;
}

/* get_one_page_from_i7002a_f():
 *   Dump the ISP page whose index is "which_page" to "pagebuf".
 *   mclk, power & rst are requisite for getting correct page data.
 */
void get_one_page_from_i7002a_f(struct v4l2_subdev *sd, int which_page, u8* pagebuf)
{
	//int i = 0;
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg_f(client, 0x70c4,0x00);
	sensor_write_reg_f(client, 0x70c5,0x00);
	printk("%s: I2C_F_SPIInit", __FUNCTION__);
	ret = I2C_F_SPIInit(sd);
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return;
	}
	printk("%s: I2C_F_SPIFlashReadId", __FUNCTION__);
	I2C_F_SPIFlashReadId(sd);
	printk("%s: I2C_SPIFlashRead_f", __FUNCTION__);
	I2C_SPIFlashRead_f(sd, which_page, 1, pagebuf);

#if 0 // dump to kmsg ?
	printk("page#%d:\n", which_page);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0)
			printk("[%04x]", i);
		printk("%02X ",  pagebuf[i]);
		if(i%16 == 15)
			printk("\n");
	}
#endif
}

unsigned int get_fw_version_in_isp_f(struct v4l2_subdev *sd)
{
	u8 tmp_page[0x100];
	unsigned int vn = 0xABCDEF;
	int i = 0;
	int retry = 3;
	bool b_ok;

	for (i = 0; i < retry; i++) {
		int j =0;
		b_ok = true;

		/* The fw veriosn is in the page with the index, 4095.*/
		get_one_page_from_i7002a_f(sd, 4095, tmp_page);

		/* The header format looks like:
		 * FF FF FF FF FF FF FF FF XX XX XX XX XX XX XX
		 * FF FF FF FF FF XX XX XX XX XX XX XX XX XX XX
		 */
		for (j = 0; j < 8; j++) {
			if (tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j] != 0xFF) {
				printk("%s: tmp_page[0x%X]= %02X\n", __FUNCTION__,
					0x100 - BIN_FILE_HEADER_SIZE + j,
					tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j]);
				b_ok = false;
				break;
			}
		}
		if (b_ok == true)
			break;
		else {
			printk("%s: wrong page data? Try again (%d).\n", __FUNCTION__, i);
			usleep_range(10000, 11000);
		}
	}

	if (b_ok == true)
		vn = (tmp_page[0xFF - 1] <<16) | (tmp_page[0xFF - 2] << 8) | tmp_page[0xFF -3];
	printk("%s: vn=0x%X\n", __FUNCTION__, vn);
	return vn;
}

void
BB_F_WrSPIFlash(struct v4l2_subdev *sd, char* binfile_path)
{
	u32 id, type;
	u32 pages;

	u8 *pbootBuf;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	u8 checksum1_in_bin[2], checksum2_in_bin[2];
	u8 checksum1_in_isp[2], checksum2_in_isp[2];
	static unsigned int version_num_in_isp = 0xffffff;
	int firmware2_offset;
	u8 tmp_page[0x100];

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i, ret = 0;

	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	wake_lock(dev->fw_update_wakelock);
	ret = spca700xa_16mpf_s_power(sd, 1);
	goto end; //Wesley Test

	if (ret) {
		dev_err(&client->dev, "spca700xa_16mpf power-up err");
	}

	fw_update_status = ICATCH_FW_IS_BURNING;

	sensor_write_reg_f(client, 0x70c4,0x00);
	sensor_write_reg_f(client, 0x70c5,0x00);

	sensor_write_reg_f(client, 0x1011,0x01); /* CPU reset */
	sensor_write_reg_f(client, 0x001C,0x08); /* FM reset */
	sensor_write_reg_f(client, 0x001C,0x00);
	sensor_write_reg_f(client, 0x108C,0x00); /* DMA select */
	sensor_write_reg_f(client, 0x009a,0x00); /* CPU normal operation */

	/* Calculate BOOT.BIN file size. */
	fp = filp_open(binfile_path, O_RDONLY, 0);
	printk("%s: Calculate BOOT.BIN file size. L:%d\n", __FUNCTION__, __LINE__);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", binfile_path);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);
#if 0
			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			} else
#endif
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("iCatch \"%s\" not found error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
		printk("%s: bin_file_header[%d]= 0x%x\n", __FUNCTION__, i,bin_file_header[i]);
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	/* Get the checksum in bin file.
	 *   firmware2_offset
	 *     = fw1 header size
	 *     + fw1 DMEM FICDMEM size
	 *     + fw1 IMEM size
	 */
	memcpy(checksum1_in_bin, pbootBuf + 10, 2);

	firmware2_offset = 16 +
		((pbootBuf[3] << 24) | (pbootBuf[2] << 16) | (pbootBuf[1] << 8) | pbootBuf[0]) +
		((pbootBuf[7] << 24) | (pbootBuf[6] << 16) | (pbootBuf[5] << 8) | pbootBuf[4]);
	memcpy(checksum2_in_bin, pbootBuf + firmware2_offset + 10, 2);

	printk("%s: checksum in bin:%02X %02X; %02X %02X\n", __FUNCTION__,
		checksum1_in_bin[0],checksum1_in_bin[1],checksum2_in_bin[0], checksum2_in_bin[1]);

	ret = I2C_F_SPIInit(sd);
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	printk("%s: I2C_F_SPIInit.\n", __FUNCTION__);
	id = I2C_F_SPIFlashReadId(sd);

	if(id==0) {
		printk("read id failed\n");
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	type = BB_F_SerialFlashTypeCheck(id);
	if(type == 0) {
		printk("BB_F_SerialFlashTypeCheck(%d) failed\n", id);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	pages = bootbin_size/0x100;

	pr_debug("%s: pages:0x%x\n", __FUNCTION__, pages);

	/* Writing Flash here */
	printk("%s: Writing Flash here.\n", __FUNCTION__);
	if( type == 2 ) {
		flash_type = ICATCH_FLASH_TYPE_SST;
		printk("SST operation\n");
		ret = I2C_F_SPISstChipErase(sd);
		if(ret) {
			printk("%s: SST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		I2C_F_SPISstFlashWrite(sd, 0, pages, pbootBuf);
	} else if( type == 1 || type == 3 ) {
		flash_type = ICATCH_FLASH_TYPE_ST;
		printk("ST operation\n");
		ret = I2C_F_SPIStChipErase(sd);
		if(ret) {
			printk("%s: ST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		I2C_F_SPIFlashWrite_DMA(sd, 0, pages, pbootBuf); //new burn ISP flow
	} else {
		printk("type unknown: %d; Won't update iCatch FW.\n", type);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		kfree(pbootBuf);
		goto end;
	}
	kfree(pbootBuf);

	/* Check the update reult. */
	/* Compare Check sum here */
	get_one_page_from_i7002a_f(sd, 0, tmp_page);
	memcpy(checksum1_in_isp, tmp_page + 10, 2);

	if (memcmp(checksum1_in_isp, checksum1_in_bin, 2) == 0) {
		/* checksum1 PASS */
		firmware2_offset = 16 +
			((tmp_page[3] << 24) | (tmp_page[2] << 16) | (tmp_page[1] << 8) | tmp_page[0]) +
			((tmp_page[7] << 24) | (tmp_page[6] << 16) | (tmp_page[5] << 8) | tmp_page[4]);

		get_one_page_from_i7002a_f(sd, firmware2_offset >> 8, tmp_page);
		memcpy(checksum2_in_isp, tmp_page + 10, 2);

		if (memcmp(checksum2_in_isp, checksum2_in_bin, 2) == 0) {
			/* checksum2 PASS */
			version_num_in_isp = get_fw_version_in_isp_f(sd);
			if (version_num_in_isp == version_num_in_bin) {
				/* version number PASS */
				fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
				printk("%s: ICATCH FW UPDATE SUCCESS.\n", __FUNCTION__);
			} else {
				/* version number FAIL */
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				printk("%s: check version FAIL: ISP(0x%06X) != BIN(0x%06X)\n", __FUNCTION__, version_num_in_isp, version_num_in_bin);
				version_num_in_isp = 0xABCDEF;
			}
		} else {
			/* checksum2 FAIL */
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			printk("%s: checksum2 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
				__FUNCTION__, checksum2_in_isp[0], checksum2_in_isp[1],
				checksum2_in_bin[0], checksum2_in_bin[1]);
			version_num_in_isp = 0xABCDEF;
		}
	} else {
		/* checksum1 FAIL */
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		printk("%s: checksum1 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
			__FUNCTION__, checksum1_in_isp[0], checksum1_in_isp[1],
			checksum1_in_bin[0], checksum1_in_bin[1]);
		version_num_in_isp = 0xABCDEF;
	}
	pr_debug("%s: Try to power down.\n", __FUNCTION__);
end:
	//only for pin test - dont close power(1/2)
	ret = spca700xa_16mpf_s_power(sd, 0);
	if (ret) {
		dev_err(&client->dev, "spca700xa_16mpf power down err");
	}
	wake_unlock(dev->fw_update_wakelock);

}

//Move and modify from ov5693.h for firmware --- Wesley +++
//ASUS_BSP+++, add for ISP firmware update, create proc file
#ifdef	CONFIG_PROC_FS
#define	i7002a_F_PROC_FILE	"driver/i7003a_front"
static struct proc_dir_entry *i7002a_proc_front_file = NULL;

static int i7002a_proc_front_read(struct seq_file *m, void *start)
{
	int len;
	int ret=0;
	static unsigned int version_num_in_isp = 0xffffff;
   	ret = spca700xa_16mpf_s_power(fw_sd_f, 1);
	version_num_in_isp = get_fw_version_in_isp_f(fw_sd_f);
	ret = spca700xa_16mpf_s_power(fw_sd_f, 0);
	len = seq_printf(m, "version_in_isp:%x\n", version_num_in_isp);
	return len;
}

static ssize_t i7002a_proc_front_write(struct file *filp, const char __user *buff, size_t count, loff_t *pos)
{
	struct i2c_client *client = v4l2_get_subdevdata(fw_sd_f);
	unsigned int addr, value;
	u16 read_value;
	char messages[256];
	int len = 256;
	printk("%s: proc write func len=%d.\n", __FUNCTION__, len);

	if (copy_from_user(messages, buff, len))
		return -EFAULT;
        pr_info("i7002a_proc_write %s\n", messages);
	if ('f' == messages[0]) {
		/* Update ISP firmware*/
        	//i7002a_update_status = 0;
	        pr_info("i7002a firmware update start\n");
	        BB_F_WrSPIFlash(fw_sd_f, FW_BIN_FILE_WITH_PATH);
        //i7002a_update_status = 1;
	} else if(!strncmp("ri2c",messages,4)){ //Read i2c
		sscanf(&messages[5],"%x",&addr);
		pr_info("%s: addr 0x%d.\n", __func__, addr);
		sensor_read_reg_f(client, addr, &read_value);
		pr_info("I2C read REG:0x%x is 0x%x \n", addr, read_value);
	} else if(!strncmp("wi2c",messages,4)){ //Write i2c
		sscanf(&messages[5],"%x",&addr);
		sscanf(&messages[10],"%x",&value);
		pr_info(KERN_INFO "%s: addr 0x%x, val 0x%x.\n", __func__, addr, value);
		sensor_write_reg_f(client, addr, value);
	} else if(!strncmp("is_calibration",messages,14)){ //Add for calibration
		sscanf(&messages[15],"%x",&value);
		printk("%s: Calibration val for Front: %x.\n", __func__, value);
		g_is_calibration_f = value;
	} else {
		pr_info("command not support\n");
	}

	return len;
}

static int i7002a_proc_front_open(struct inode *inode, struct file *file) {
        return single_open(file, i7002a_proc_front_read, NULL);
}

static const struct file_operations i7002a_proc_front_ops = {
        .open           = i7002a_proc_front_open,
        .read           = seq_read,
        .write          = i7002a_proc_front_write,
        .llseek         = seq_lseek,
        .release        = seq_release
};


void create_i7002a_F_proc_front_file(void)
{
    i7002a_proc_front_file = proc_create(i7002a_F_PROC_FILE, 0666, NULL, &i7002a_proc_front_ops);
    if (!i7002a_proc_front_file) {
         printk("%s:Unable to create proc file \n", __FUNCTION__);
         pr_err("proc file create failed!\n");
    }else{
         printk("%s: create proc file.", __FUNCTION__);
    }
}
#endif

//ASUS_BSP---
//Move and modify from ov5693.h for firmware --- Wesley ---


int sensor_s_color_effect_f(struct v4l2_subdev *sd, int effect)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d\n", __func__, effect);

	switch (effect) {
	case V4L2_COLORFX_NONE:
		sensor_write_reg_f(client, 0x7102, 0x00);//auto
		break;
	case V4L2_COLORFX_AQUA:
		sensor_write_reg_f(client, 0x7102, 0x01);//aqua
		break;
	case V4L2_COLORFX_NEGATIVE:
		sensor_write_reg_f(client, 0x7102, 0x02);//negative
		break;
	case V4L2_COLORFX_SEPIA:
		sensor_write_reg_f(client, 0x7102, 0x03);//sepia
		break;
	case V4L2_COLORFX_BW:
		sensor_write_reg_f(client, 0x7102, 0x04);//grayscale
		break;
	case V4L2_COLORFX_VIVID:
		sensor_write_reg_f(client, 0x7102, 0x05);//vivid
		break;
        case V4L2_COLORFX_AURA:
                sensor_write_reg_f(client, 0x7102, 0x06);//aura
                sensor_write_reg_f(client, 0x7119, 0x00);//set default value
                break;
        case V4L2_COLORFX_EMBOSS:
                sensor_write_reg_f(client, 0x7102, 0x07);//vintage
                break;
        case V4L2_COLORFX_VINTAGE2:
                sensor_write_reg_f(client, 0x7102, 0x08);//vintage2
                break;
        case V4L2_COLORFX_SKETCH:
                sensor_write_reg_f(client, 0x7102, 0x09);//lomo
                break;
        case V4L2_COLORFX_RED:
                sensor_write_reg_f(client, 0x7102, 0x0A);//red
                break;
        case V4L2_COLORFX_BLUE:
                sensor_write_reg_f(client, 0x7102, 0x0B);//blue
                break;
        case V4L2_COLORFX_GREEN:
                sensor_write_reg_f(client, 0x7102, 0x0C);//green
                break;
	default:
		dev_err(&client->dev, "invalid col eff: %d", effect);
		return -ERANGE;
	}

        lastEffect = effect;
	return 0;
}

int sensor_s_white_balance_f(struct v4l2_subdev *sd, int wb_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d lastWhiteBalance = %d\n", __func__, wb_mode, lastWhiteBalance);

        if(lastWhiteBalance != wb_mode){
	switch (wb_mode) {
	case V4L2_WHITE_BALANCE_AUTO:
		sensor_write_reg_f(client, 0x710A, 0x00);//auto
		break;
	case V4L2_WHITE_BALANCE_DAYLIGHT:
		sensor_write_reg_f(client, 0x710A, 0x01);//daylight
		break;
	case V4L2_WHITE_BALANCE_CLOUDY:
		sensor_write_reg_f(client, 0x710A, 0x02);//cloudy
		break;
	case V4L2_WHITE_BALANCE_SHADE:
		sensor_write_reg_f(client, 0x710A, 0x03);//shade
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT:
		sensor_write_reg_f(client, 0x710A, 0x04);//fluorescent_L
		break;
	case V4L2_WHITE_BALANCE_FLUORESCENT_H:
		sensor_write_reg_f(client, 0x710A, 0x05);//fluorescent_H
		break;
	case V4L2_WHITE_BALANCE_INCANDESCENT:
		sensor_write_reg_f(client, 0x710A, 0x06);//tungsten
		break;
	default:
		dev_err(&client->dev, "invalid white balance mode: %d", wb_mode);
		return -ERANGE;
	}
        }
        lastWhiteBalance = wb_mode;

	return 0;
}

int sensor_s_iso_f(struct v4l2_subdev *sd, int iso);
int sensor_s_ev_f(struct v4l2_subdev *sd, int ev);
int sensor_s_effect_aura_f(struct v4l2_subdev *sd, int aura);

int sensor_s_hdrf(struct v4l2_subdev *sd, int hdr);

int sensor_s_hdrf(struct v4l2_subdev *sd, int hdr)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	printk("%s HDR enable %d", __func__, hdr);
	mHDR = hdr;
	memset(&old_f, 0 ,sizeof(old_f));
	skip_cnt = 0;
	return 0;
}

int sensor_s_scene_mode_f(struct v4l2_subdev *sd, int scene_mode)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        pr_debug("%s mode = %d\n", __func__, scene_mode);

	mNightMode = false;
	mHDRMode = false;
        switch (scene_mode) {
        case V4L2_SCENE_MODE_NONE:
                sensor_write_reg_f(client, 0x7109, 0x00);//auto
                break;
        case V4L2_SCENE_MODE_BACKLIGHT:
                sensor_write_reg_f(client, 0x7109, 0x16);//backlight
                break;
        case V4L2_SCENE_MODE_BEACH_SNOW:
                sensor_write_reg_f(client, 0x7109, 0x0B);//snow
                break;
        case V4L2_SCENE_MODE_CANDLE_LIGHT:
                sensor_write_reg_f(client, 0x7109, 0x04);//candle light
                break;
        case V4L2_SCENE_MODE_FIREWORKS:
                sensor_write_reg_f(client, 0x7109, 0x05);//firework
                break;
        case V4L2_SCENE_MODE_LANDSCAPE:
                sensor_write_reg_f(client, 0x7109, 0x06);//landscape
                break;
        case V4L2_SCENE_MODE_NIGHT:
		sensor_write_reg_f(client, 0x7134, 0x00);//set to ultra night mode
		sensor_write_reg_f(client, 0x7109, 0x07);//open night mode
		mNightMode = true;
		break;
        case V4L2_SCENE_MODE_LOW_NIGHT:
		sensor_write_reg_f(client, 0x7134, 0x01);//set to ultra night mode
		sensor_write_reg_f(client, 0x7109, 0x07);//open night mode
		break;
        case V4L2_SCENE_MODE_PARTY_INDOOR:
                sensor_write_reg_f(client, 0x7109, 0x09);//party
                break;
        case V4L2_SCENE_MODE_PORTRAIT:
                sensor_write_reg_f(client, 0x7109, 0x0A);//portrait
                break;
        case V4L2_SCENE_MODE_SPORTS:
                sensor_write_reg_f(client, 0x7109, 0x0C);//sport
                break;
        case V4L2_SCENE_MODE_SUNSET:
                sensor_write_reg_f(client, 0x7109, 0x0E);//sunset
                break;
        case V4L2_SCENE_MODE_TEXT:
                sensor_write_reg_f(client, 0x7109, 0x02);//text
                break;
        case V4L2_SCENE_MODE_HDR:
		mHDRMode = true;
                break;
        default:
                dev_err(&client->dev, "invalid snene mode: %d", scene_mode);
                return -ERANGE;
        }

        if((scene_mode == V4L2_SCENE_MODE_NONE) && (lastSceneMode == V4L2_SCENE_MODE_NIGHT)){
                sensor_s_iso_f(sd, iso_setting);
                sensor_s_color_effect_f(sd, lastEffect);
                sensor_s_effect_aura_f(sd, lastEffectAura);
                sensor_s_ev_f(sd, lastEV);
        }
        lastSceneMode = scene_mode;
        return 0;
}

int sensor_s_ev_f(struct v4l2_subdev *sd, int ev)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d lastEV = %d\n", __func__, ev, lastEV);

        if(lastEV != ev){
	switch (ev) {
	case 6:
		sensor_write_reg_f(client, 0x7103, 0x00);// +2.0
		break;
	case 5:
		sensor_write_reg_f(client, 0x7103, 0x01);// +1.7
		break;
	case 4:
		sensor_write_reg_f(client, 0x7103, 0x02);// +1.3
		break;
	case 3:
		sensor_write_reg_f(client, 0x7103, 0x03);// +1.0
		break;
	case 2:
		sensor_write_reg_f(client, 0x7103, 0x04);// +0.7
		break;
	case 1:
		sensor_write_reg_f(client, 0x7103, 0x05);// +0.3
		break;
	case 0:
		sensor_write_reg_f(client, 0x7103, 0x06);// +0
		break;
	case -1:
		sensor_write_reg_f(client, 0x7103, 0x07);// -0.3
		break;
	case -2:
		sensor_write_reg_f(client, 0x7103, 0x08);// -0.7
		break;
	case -3:
		sensor_write_reg_f(client, 0x7103, 0x09);// -1.0
		break;
	case -4:
		sensor_write_reg_f(client, 0x7103, 0x0A);// -1.3
		break;
	case -5:
		sensor_write_reg_f(client, 0x7103, 0x0B);// -1.7
		break;
	case -6:
		sensor_write_reg_f(client, 0x7103, 0x0C);// -2.0
		break;
	default:
		dev_err(&client->dev, "invalid ev: %d", ev);
		return -ERANGE;
	}
        }

        lastEV = ev;
	return 0;
}

int sensor_s_flicker_f(struct v4l2_subdev *sd, int flicker)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d\n", __func__, flicker);

	switch (flicker) {
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		sensor_write_reg_f(client, 0x7101, 0x00);//auto
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		sensor_write_reg_f(client, 0x7101, 0x01);//50hz
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		sensor_write_reg_f(client, 0x7101, 0x02);//60hz
		break;
	default:
		dev_err(&client->dev, "invalid flicker: %d", flicker);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_iso_f(struct v4l2_subdev *sd, int iso)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d\n", __func__, iso);
        iso_setting = iso;

	switch (iso) {
	case SENSOR_ISO_AUTO:
		sensor_write_reg_f(client, 0x7110, 0x00);
		break;
	case SENSOR_ISO_50:
		sensor_write_reg_f(client, 0x7110, 0x01);
		break;
	case SENSOR_ISO_100:
		sensor_write_reg_f(client, 0x7110, 0x02);
		break;
	case SENSOR_ISO_200:
		sensor_write_reg_f(client, 0x7110, 0x03);
		break;
	case SENSOR_ISO_400:
		sensor_write_reg_f(client, 0x7110, 0x04);
		break;
	case SENSOR_ISO_800:
		sensor_write_reg_f(client, 0x7110, 0x05);
		break;
        case SENSOR_ISO_1600:
                sensor_write_reg_f(client, 0x7110, 0x06);
                break;
	default:
		dev_err(&client->dev, "invalid iso: %d", iso);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_ae_metering_mode_f(struct v4l2_subdev *sd, int ae_mode)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//int reg_val;
	//int ret = 0;
	pr_debug("%s mode = %d\n", __func__, ae_mode);

	switch (ae_mode) {
	case V4L2_EXPOSURE_METERING_AVERAGE:
		sensor_write_reg_f(client, 0x710E, 0x00);//multi
		break;
	case V4L2_EXPOSURE_METERING_SPOT:
		sensor_write_reg_f(client, 0x710E, 0x01);//spot
		break;
	case V4L2_EXPOSURE_METERING_CENTER_WEIGHTED:
		sensor_write_reg_f(client, 0x710E, 0x01);//center
		break;
	default:
		dev_err(&client->dev, "invalid ae_mode: %d", ae_mode);
		return -ERANGE;
	}

	return 0;
}

int sensor_s_3a_lock_f(struct v4l2_subdev *sd, int lock)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        pr_debug("%s 3a_lock %d\n", __func__, lock);

        switch (lock) {
        case SENSOR_AE_AWB_AF_UNLOCK:
                sensor_write_reg_f(client, 0x71EB, 0x00);//AE_AWB_AF_UNLOCK
                break;
        case SENSOR_AE_AWB_UNLOCK:
                sensor_write_reg_f(client, 0x71EB, 0x02);//AE_AWB_UNLOCK
                break;
        case SENSOR_AE_LOCK:
                sensor_write_reg_f(client, 0x71EB, 0x03);//AE_LOCK
                break;
        case SENSOR_AWB_LOCK:
                sensor_write_reg_f(client, 0x71EB, 0x04);//AWB_LOCK
                break;
        case SENSOR_AE_UNLOCK:
                sensor_write_reg_f(client, 0x71EB, 0x05);//AE_UNLOCK
                break;
        case SENSOR_AWB_UNLOCK:
                sensor_write_reg_f(client, 0x71EB, 0x06);//AWB_UNLOCK
                break;
        case SENSOR_AE_AWB_LOCK:
                sensor_write_reg_f(client, 0x71EB, 0x10);//AE_AWB_LOCK
                break;
        default:
                dev_err(&client->dev, "invalid 3a_lock: %d", lock);
                return -ERANGE;
        }
        return 0;
}
#if 1
//ASUS_BSP+++, Camera5.0
int sensor_s_exposure_window_f(struct v4l2_subdev *sd, char *window)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);

        int x_left, x_right, y_top, y_bottom;
        int start_x, start_y, size;
        pr_debug("%s\n", __func__);

        sscanf(window, "%d,%d,%d,%d",
                &x_left, &y_top, &x_right, &y_bottom);

        start_x = x_left;
        start_y = y_top;
        size = x_right - x_left;

        pr_debug("%s, x:%d, y:%d, size:%d\n", __func__, start_x, start_y, size);

        sensor_write_reg_f(client, 0x7188, 0x01);//AE ROI On
        sensor_write_reg_f(client, 0x7148, size>>8);//AE ROI Size_H
        sensor_write_reg_f(client, 0x7149, size&0xFF);//AE ROI Size_L
        sensor_write_reg_f(client, 0x714A, start_x>>8);//AE ROI X_H
        sensor_write_reg_f(client, 0x714B, start_x&0xFF);//AE ROI X_L
        sensor_write_reg_f(client, 0x714C, start_y>>8);//AE ROI Y_H
        sensor_write_reg_f(client, 0x714D, start_y&0xFF);//AE ROI Y_L
        sensor_write_reg_f(client, 0x714E, 0x01);//AE ROI Trigger(Use TAE ROI)

        return 0;
}
//ASUS_BSP---, Camera5.0
#endif
int sensor_s_effect_aura_f(struct v4l2_subdev *sd, int aura)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        //u16 reg_val=0x0;
        pr_debug("%s aura %d\n", __func__, aura);

        if(aura < 64) {
                sensor_write_reg_f(client, 0x7119, aura);//set aura
                lastEffectAura = aura;
                return 0;
        }else {
                dev_err(&client->dev, "invalid aura: %d", aura);
                return -ERANGE;
        }
}

int sensor_s_moving_f(struct v4l2_subdev *sd, int moving)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        pr_debug("%s moving %d\n", __func__, moving);

        switch (moving) {
        case SENSOR_MOVING_OFF:
                sensor_write_reg_f(client, 0x7125, 0x00);//Disable Max Exposure Time function
                break;
        case SENSOR_MOVING_ON:
                sensor_write_reg_f(client, 0x7125, 0xFF);//Max Exposure Time
                break;
        default:
                dev_err(&client->dev, "invalid MOVING: %d", moving);
                return -ERANGE;
        }
        return 0;
}

int sensor_s_wdr_f(struct v4l2_subdev *sd, int wdr)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 reg_val=0x0, reg_clear_wdr=0x0, reg_set_wdr=0x0;
        pr_debug("%s wdr %d\n", __func__, wdr);

        switch (wdr) {
        case SENSOR_WDR_OFF:
                sensor_read_reg_f(client, 0x729B, &reg_val);
                reg_clear_wdr = reg_val & 0xFE;
                sensor_write_reg_f(client, 0x711B, reg_clear_wdr);//WDR off
                break;
        case SENSOR_WDR_ON:
                sensor_read_reg_f(client, 0x729B, &reg_val);
                reg_set_wdr = reg_val | 0x01;
                sensor_write_reg_f(client, 0x711B, reg_set_wdr);//WDR on
                break;
        default:
                dev_err(&client->dev, "invalid WDR: %d", wdr);
                return -ERANGE;
        }
        return 0;
}

int sensor_s_eis_f(struct v4l2_subdev *sd, int eis) //ASUS_BSP, Camera5.0
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 reg_val=0x0, reg_clear_eis=0x0, reg_set_eis=0x0;
        pr_debug("%s EIS %d\n", __func__, eis);

        switch (eis) {
        case SENSOR_EIS_OFF:
                sensor_read_reg_f(client, 0x729B, &reg_val);
                reg_set_eis = reg_val | 0x40;
                sensor_write_reg_f(client, 0x711B, reg_set_eis);//eis off
                break;
        case SENSOR_EIS_ON:
                sensor_read_reg_f(client, 0x729B, &reg_val);
                reg_clear_eis = reg_val & 0xBF;
                sensor_write_reg_f(client, 0x711B, reg_clear_eis);//eis on
                break;
        default:
                dev_err(&client->dev, "invalid EIS: %d", eis);
                return -ERANGE;
        }
        return 0;
}

//ASUS_BSP+++
int sensor_s_gsensor_data_f(struct v4l2_subdev *sd, char *data)
{
       struct i2c_client *client = v4l2_get_subdevdata(sd);
       int reg_val;
       int ret = 0;
       pr_debug("%s\n", __func__);

       int x_data, y_data, z_data;

       sscanf(data, "%x,%x,%x", &x_data, &y_data, &z_data);
       pr_debug("%s, x:%x, y:%x, z:%x\n", __func__, x_data, y_data, z_data);

       sensor_write_reg_f(client, 0x00E0, x_data);
       sensor_write_reg_f(client, 0x00E1, y_data);
       sensor_write_reg_f(client, 0x1300, z_data);

       return ret;
}
//ASUS_BSP---

//ASUS_BSP++, add for calibration
int sensor_s_write_reg_f(struct v4l2_subdev *sd, int val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_addr = 0x0;
	int reg_val = 0x0;
	int ret = 0;

	reg_addr = val >>16;
	reg_val = val & 0xFFFF;
	printk("%s reg_addr:0x%x, reg_val:0x%x\n", __func__, reg_addr, reg_val);

	ret = sensor_write_reg_f(client, reg_addr, reg_val);
	return 0;
}

int sensor_s_read_reg_f(struct v4l2_subdev *sd, int *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_addr = 0x0;
	u16 reg_val = 0x0;
	int ret = 0;

	reg_addr = *val;
	ret = sensor_read_reg_f(client, reg_addr, &reg_val);
        printk("%s Read_Addr 0x%x Read_val:0x%x\n", __func__, reg_addr, reg_val);
	*val = reg_val;
	return 0;
}

int sensor_s_read_spi_f(struct v4l2_subdev *sd, int *val)
{
	UINT8 *ucStartAddr;
	UINT32 ulTransByteCnt = 0;
	UINT8 type = 0;
	struct file *fp = NULL;
	static char *CALIBRATION_FILE_WITH_PATH;
	//struct inode *inode;
	mm_segment_t old_fs;
	loff_t offset = 0;
        struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	//int i,j;

        printk("%s, [DEBUG] 0x40E1 mode:0x02\n", __func__);
        ret = sensor_write_reg_f(client,  0x40E1, 0x02);
	printk("%s, val = %x\n", __func__, *val);
	type = (*val)>>28;
	ulTransByteCnt = (*val)&0x0FFFFFFF;
	ucStartAddr = kmalloc(ulTransByteCnt, GFP_KERNEL);

	printk("Ryant In SPI Read %s, type = %x, val = %x, ulTransByteCnt = %x\n", __func__, type, *val, ulTransByteCnt);
	spca700xa_SPI_read(ucStartAddr, ulTransByteCnt);


	switch(type) {
		case 0:
			printk("Ryant In Type 0\n");
			fp = filp_open(IIIACALI_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
			CALIBRATION_FILE_WITH_PATH = IIIACALI_FILE_WITH_PATH;
			break;
		case 1:
			printk("Ryant In Type 1\n");
			fp = filp_open(LSC_FILE_WITH_PATH, O_RDWR | O_CREAT, S_IRUGO | S_IWUSR);
			CALIBRATION_FILE_WITH_PATH = LSC_FILE_WITH_PATH;
			break;
		case 2:
			printk("Ryant In Type 2\n");
            LSC_DQcnt_f++;
            if(LSC_DQcnt_f == 1)
            {
                        printk("[DEBUG] LSC_DQcnt_f == 1\n");
			fp = filp_open(LSCDQ_FILE_WITH_PATH, O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUSR);
            }
            else if(LSC_DQcnt_f == 2)
            {
                        printk("[DEBUG] LSC_DQcnt_f == 2\n");
                        fp = filp_open(LSCDQ_FILE_WITH_PATH, O_RDWR | O_CREAT | O_APPEND, S_IRUGO | S_IWUSR);
            }
			CALIBRATION_FILE_WITH_PATH = LSCDQ_FILE_WITH_PATH;
			break;
	}


	if ( IS_ERR_OR_NULL(fp) ){
		printk("%s: open %s fail\n", __FUNCTION__, CALIBRATION_FILE_WITH_PATH);
	} else {
		printk("Ryant In File Open success! \n");
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
//add for fix write LSC_DQ header probelm ++
                if(LSC_DQcnt_f == 1)
                {
                        printk("first part of LSC_DQ to write \n");
                }
                else if(LSC_DQcnt_f == 2)
                {
                        printk("[DEBUG] LSC_DQcnt_f == 2 !!!!!!!\n");
                        fp->f_op->llseek(fp, 0, SEEK_END);
                        LSC_DQcnt_f = 0;
                }
//add for fix write LSC_DQ header probelm --
		if (fp->f_op != NULL && fp->f_op->write != NULL){
			fp->f_op->write(fp,
				ucStartAddr,
				ulTransByteCnt,
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp, NULL);
	}

	*val = 0;

	kfree(ucStartAddr);
        ret = sensor_write_reg_f(client,  0x40E1, 0x00);
        printk("%s, [DEBUG] 0x40E1 mode:0x00\n", __func__);

	return 0;
}

//ASUS_BSP--, add for calibration

int sensor_g_exposure_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        int exposure_num;
        u32 exposure_denum;
        u16 retvalue_hh, retvalue_h, retvalue_l;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72B0, &retvalue_l); //num[7:0]
        exposure_num = retvalue_l;

        ret = sensor_read_reg_f(client, 0x72B1, &retvalue_l); //denum[7:0]
        ret = sensor_read_reg_f(client, 0x72B2, &retvalue_h); //denum[15:8]
        ret = sensor_read_reg_f(client, 0x72B3, &retvalue_hh); //denum[23:16]
        exposure_denum = (retvalue_hh<<16)|(retvalue_h<<8)|(retvalue_l);

        pr_debug(" %s exposure time num %d denum %d\n", __func__, exposure_num, exposure_denum);
        //ASUS_BSP+++
        if (exposure_denum != 0){
            *val = ((exposure_num * 1000) / exposure_denum);// sofia project request this format
        }
        else
        //ASUS_BSP---
        {
            *val = ((exposure_num << 24) | exposure_denum);
        }
        return 0;
}

int sensor_g_edge_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72BA, &retvalue_0); //Capture Edge Information[7:0]
        ret = sensor_read_reg_f(client, 0x72BB, &retvalue_1); //Capture Edge Information[15:8]
        ret = sensor_read_reg_f(client, 0x72BC, &retvalue_2); //Capture Edge Information[23:16]
        ret = sensor_read_reg_f(client, 0x72BD, &retvalue_3); //Capture Edge Information[31:24]
        *val = (retvalue_3<<24)|(retvalue_2<<16)|(retvalue_1<<8)|(retvalue_0);

        pr_debug("%s, edge 0x%x\n", __func__, *val);

        return 0;
}

//For iCatch 3A information+++
int sensor_g_3a_info_ae1_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72D8, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72D9, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72DA, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72DB, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}

int sensor_g_3a_info_ae2_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72DC, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72DD, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72DE, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72DF, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}

int sensor_g_3a_info_awb1_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72E0, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72E1, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72E2, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72E3, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}

int sensor_g_3a_info_awb2_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72E4, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72E5, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72EE, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72EF, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}

int sensor_g_3a_info_af1_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72E6, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72E7, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72E8, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72E9, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}

int sensor_g_3a_info_af2_f(struct v4l2_subdev *sd, int *val)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        u16 retvalue_0, retvalue_1, retvalue_2, retvalue_3;
        int ret = 0;

        ret = sensor_read_reg_f(client, 0x72EA, &retvalue_0);
        ret = sensor_read_reg_f(client, 0x72EB, &retvalue_1);
        ret = sensor_read_reg_f(client, 0x72EC, &retvalue_2);
        ret = sensor_read_reg_f(client, 0x72ED, &retvalue_3);

        *val = ((retvalue_3 << 24) | (retvalue_2 << 16) | (retvalue_1 << 8) | retvalue_0);
        pr_debug("%s, val 0x%x\n", __func__, *val);

        return 0;
}
//For iCatch 3A information---
int sensor_g_vblanking_f(struct v4l2_subdev *sd, int *val)
{
        //struct i2c_client *client = v4l2_get_subdevdata(sd);
        *val = 2000;

        pr_debug("%s, vblanking %d\n", __func__, *val);

        return 0;
}

//ASUS_BSP++ Wesley, For load from host
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_7002SPICfg
 *  Description : external SPI configuration
 *  ucSPIMode: SPI mode
     ucSPIFreq: SPI frequency
     ulDmaAddr: SPCA7002 DMA start address
     ulTransByteCnt: buffer size to be wrote
 *  Return : status
 *------------------------------------------------------------------------*/
#if 0
static UINT8 EXISP_F_7002SPICfg(UINT8 ucSPIMode, UINT8 ucSPIFreq, UINT32 ulDmaAddr, UINT32 ulTransByteCnt, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	UINT8 status = SUCCESS;
	UINT8 ucStartBank, ucEndBank, ucRealEndBank, i;
	UINT32 udwBankEnValue=0;
	const UINT16 uwBanksize = 8192;
	const UINT32 udwMaxTransByteCnt = 0x50000;
	u16 ucReadData;

	static UINT8 regdata1[][3] = {
		{0x40, 0x10, 0x10}, /*SPI reset*/
		{0x40, 0xe0, 0x00}, /*SPI freq*/
		{0x40, 0xe1, 0x00}, /*SPI mode*/
		{0x40, 0x51, 0x01}, /*SPI enable*/
		{0x40, 0x11, 0x10}, /*DMA0 reset*/
		{0x41, 0x64, 0x01}, /*Read data from host*/
		{0x41, 0x70, 0x0f}, /*Byte Cnt Low*/
		{0x41, 0x71, 0x0f}, /*Byte Cnt Mid*/
		{0x41, 0x72, 0x0f}, /*Byte Cnt High*/
		{0x10, 0x8c, 0x00}, /*DMA master select FM*/
		{0x10, 0x80, 0x00}, /*DMA start addr Low*/
		{0x10, 0x81, 0x00}, /*DMA start addr Mid*/
		{0x10, 0x82, 0x00}, /*DMA start addr High*/
		{0x10, 0x84, 0x00}, /*DMA bank enable*/
		{0x10, 0x85, 0x00},
		{0x10, 0x86, 0x00},
		{0x10, 0x87, 0x00},
		{0x10, 0x88, 0x00},
		{0x00, 0x26, 0x00},
		{0x40, 0x03, 0x02}, /*Clear DMA0 INT status*/
	};

	regdata1[1][2] = (((UINT8)ucSPIFreq) & 0x7);
	regdata1[2][2] = (((UINT8)ucSPIMode) & 0xf);
	regdata1[5][2] = (1&0x1);

	if (udwMaxTransByteCnt < ulTransByteCnt)
		ulTransByteCnt = (udwMaxTransByteCnt-1);
	else
		ulTransByteCnt--;

	regdata1[6][2] = (ulTransByteCnt & 0xff);
	regdata1[7][2] = ((ulTransByteCnt >> 8) & 0xff);
	regdata1[8][2] = ((ulTransByteCnt >> 16) & 0xff);
	regdata1[9][2] = 0<<4;
	regdata1[10][2] = (ulDmaAddr & 0xff);
	regdata1[11][2] = ((ulDmaAddr >> 8) & 0xff);
	regdata1[12][2] = ((ulDmaAddr >> 16) & 0xff);

	ucStartBank = (ulDmaAddr&0xffffff)/uwBanksize;
	ucEndBank = ((ulDmaAddr&0xffffff)+ulTransByteCnt)/uwBanksize;
	ucRealEndBank = ucEndBank;

	if (ucEndBank > 31) {

		for (i = 32; i <= ucEndBank; i++)
			udwBankEnValue |= (1 << (i-32));

		regdata1[17][2] = (udwBankEnValue & 0xff);
		ucRealEndBank = 32;
		udwBankEnValue = 0;
	}

	for (i = ucStartBank; i <= ucRealEndBank; i++)
		udwBankEnValue |= (1 << i);

	regdata1[13][2] = (udwBankEnValue & 0xff);
	regdata1[14][2] = ((udwBankEnValue >> 8) & 0xff);
	regdata1[15][2] = ((udwBankEnValue >>16) & 0xff);
	regdata1[16][2] = ((udwBankEnValue >>24) & 0xff);

	sensor_read_reg_f(client, 0x0026, &ucReadData); /*Config the SPI pin GPIO/Function mode.*/
	ucReadData &= (~0xf);
	regdata1[18][2] = ucReadData;

	for (i = 0; i < sizeof(regdata1)/sizeof(regdata1[0]); i++)
		sensor_write_reg_f(client, ((regdata1[i][0]<<8)&0xFF00)+(regdata1[i][1]&0x00FF), regdata1[i][2]);

	return status;
}
#endif
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_7002SPICfg_slave
 *  Description : external SPI configuration
 *  ucSPIMode: SPI mode
     ulDmaAddr: SPCA7002 DMA start address
     ulTransByteCnt: buffer size to be wrote
 *  Return : status
 *------------------------------------------------------------------------*/
static UINT8 EXISP_F_7002SPICfg_slave(UINT8 ucSPIMode, UINT32 ulDmaAddr, UINT32 ulTransByteCnt, struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	UINT8 status = SUCCESS;
	UINT8 ucStartBank, ucEndBank, ucRealEndBank, i;
	UINT32 udwBankEnValue=0;
	const UINT16 uwBanksize = 8192;
	const UINT32 udwMaxTransByteCnt = 0x50000;
	u16 ucReadData;
	static UINT8 regdata1[][3] = {
		{0x40, 0x10, 0x10}, /*SPI reset*/
		{0x40, 0xe1, 0x00}, /*SPI mode*/
		{0x40, 0x51, 0x01}, /*SPI enable*/
		{0x40, 0x11, 0x10}, /*DMA0 reset*/
		{0x41, 0x64, 0x01}, /*Read data from host*/
		{0x41, 0x70, 0x0f}, /*Byte Cnt Low*/
		{0x41, 0x71, 0x0f}, /*Byte Cnt Mid*/
		{0x41, 0x72, 0x0f}, /*Byte Cnt High*/
		{0x10, 0x8c, 0x00}, /*DMA master select FM*/
		{0x10, 0x80, 0x00}, /*DMA start addr Low*/
		{0x10, 0x81, 0x00}, /*DMA start addr Mid*/
		{0x10, 0x82, 0x00}, /*DMA start addr High*/
		{0x10, 0x84, 0x00}, /*DMA bank enable*/
		{0x10, 0x85, 0x00},
		{0x10, 0x86, 0x00},
		{0x10, 0x87, 0x00},
		{0x10, 0x88, 0x00},
		{0x00, 0x26, 0x00},
		{0x40, 0x03, 0x02}, /*Clear DMA0 INT status*/
	};
	ucSPIMode = 0; //wesley, need to check?
	regdata1[1][2] = (((UINT8)ucSPIMode) & 0xf);
	regdata1[4][2] = (1&0x1);
	if (udwMaxTransByteCnt < ulTransByteCnt)
		ulTransByteCnt = (udwMaxTransByteCnt-1);
	else
		ulTransByteCnt--;
	regdata1[5][2] = (ulTransByteCnt & 0xff);
	regdata1[6][2] = ((ulTransByteCnt >> 8) & 0xff);
	regdata1[7][2] = ((ulTransByteCnt >> 16) & 0xff);
	regdata1[8][2] = 0<<4;
	regdata1[9][2] = (ulDmaAddr & 0xff);
	regdata1[10][2] = ((ulDmaAddr >> 8) & 0xff);
	regdata1[11][2] = ((ulDmaAddr >> 16) & 0xff);
	ucStartBank = (ulDmaAddr&0xffffff)/uwBanksize;
	ucEndBank = ((ulDmaAddr&0xffffff)+ulTransByteCnt)/uwBanksize;
	ucRealEndBank = ucEndBank;
	if (ucEndBank > 31) {
		for (i = 32; i <= ucEndBank; i++)
			udwBankEnValue |= (1 << (i-32));
		regdata1[16][2] = (udwBankEnValue & 0xff);
		ucRealEndBank = 32;
		udwBankEnValue = 0;
	}
	for (i = ucStartBank; i <= ucRealEndBank; i++)
		udwBankEnValue |= (1 << i);
	regdata1[12][2] = (udwBankEnValue & 0xff);
	regdata1[13][2] = ((udwBankEnValue >> 8) & 0xff);
	regdata1[14][2] = ((udwBankEnValue >>16) & 0xff);
	regdata1[15][2] = ((udwBankEnValue >>24) & 0xff);
	sensor_read_reg_f(client, 0x0026, &ucReadData); /*Config the SPI pin GPIO/Function mode.*/
	ucReadData &= (~0xf);
	regdata1[17][2] = ucReadData;
	for (i = 0; i < sizeof(regdata1)/sizeof(regdata1[0]); i++)
		sensor_write_reg_f(client, ((regdata1[i][0]<<8)&0xFF00)+(regdata1[i][1]&0x00FF), regdata1[i][2]);
	return status;
}
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_SPIDataWrite
 *  Description : write data to external ISP through SPI interface
 *  ucSPIMode: SPI mode
     ucSPIFreq: SPI frequency
     ucStartAddr: buffer to be wrote
     ulTransByteCnt: buffer size to be wrote
     ulDmaAddr: SPCA7002 DMA start address
 *  Return : status
 *------------------------------------------------------------------------*/
//static UINT8 EXISP_F_SPIDataWrite(UINT8 ucSPIMode, UINT8 ucSPIFreq, UINT8 *ucStartAddr, UINT32 ulTransByteCnt, UINT32 ulDmaAddr, struct v4l2_subdev *sd)
#define testsize 65536
#define read_spi_test 0
static UINT8 EXISP_F_SPIDataWrite(UINT8 ucSPIMode, UINT8 *ucStartAddr, UINT32 ulTransByteCnt, UINT32 ulDmaAddr, struct v4l2_subdev *sd)
{
        UINT8 status = SUCCESS;
        UINT16 uwTimeCnt = 0;
        u16 retValue;
        u16 retvalue_l = 0;
        int i = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

#if read_spi_test //Read SPI test
	ulTransByteCnt = testsize;
#endif
	printk("In EXISP_SPIDataWrite.\n");

	//EXISP_F_7002SPICfg(ucSPIMode, ucSPIFreq, ulDmaAddr, ulTransByteCnt, sd);
	EXISP_F_7002SPICfg_slave(ucSPIMode, ulDmaAddr, ulTransByteCnt, sd);
	/*Trigger the 7002 SPI DMA0*/
	sensor_write_reg_f(client, 0x4160, 0x01);

	/* Enable data transaction */

	sensor_read_reg_f(client, 0x72F8, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x72F8 %x\n",retvalue_l&0xffff);

        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4284, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x4284 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4285, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x4285 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4280, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x4280 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x0026, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x0026 (1) %x\n",retvalue_l&0xffff);
#if 0
	sensor_write_reg_f(client, 0x0026, 0xcf);
#endif
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x0026, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x0026 (2) %x\n",retvalue_l&0xffff);

	retvalue_l = 0;
	sensor_read_reg_f(client, 0x0022, &retvalue_l);
	printk("spca700xa_16mpf BEFORE Wesley SPi Test read 0x0022 %x\n",retvalue_l&0xffff);
	spca700xa_SPI_write(ucStartAddr, ulTransByteCnt);

        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4178, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4178 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4179, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4179 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x417A, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x417A %x\n",retvalue_l&0xffff);


#if 0//read_spi_test //have problem to crash spi transport


	//EXISP_F_7002SPICfg(ucSPIMode, ucSPIFreq, ulDmaAddr, ulTransByteCnt, sd);
	sensor_write_reg_f(client, 0x4164, 0x00);	
	/*Trigger the 7002 SPI DMA0*/
	sensor_write_reg_f(client, 0x4160, 0x01);

	spca700xa_SPI_read(ucStartAddr, ulTransByteCnt);

	for (i=0; i<testsize; i++) {
		printk("test[%d]=0x%x\n", i, ucStartAddr[i]);
	}
#endif

	/* Wait SPCA7002 DAM done */
	printk("spca700xa_16mpf Wesley SPi Test checksun\n");

	sensor_read_reg_f(client, 0x4284, &retvalue_l);
	printk("spca700xa_16mpf Wesley SPi Test read 0x4284 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4285, &retvalue_l);
	printk("spca700xa_16mpf Wesley SPi Test read 0x4285 %x\n",retvalue_l&0xffff);


	sensor_read_reg_f(client, 0x4003, &retValue);
	while ((retValue&0x02) != 0x02) {
		udelay(5000);
		uwTimeCnt++;
		if (1000 < uwTimeCnt) {
			printk("Wait 7002 DMA0 INT Timeout.\n");
			return status;
		}
		sensor_read_reg_f(client, 0x4003, &retValue);
	}

//Wesley, for spi test, dump DMA
#if 1
{
	UINT32 startAddr = ucStartAddr[0];
	UINT32 temp1, temp2 = 0;
	UINT32 regData = 0;
	UINT32 reg10a6=0, reg10a7 = 0;

	for (i = 0; i < 16; i++) {
		temp1 = 0x40|(startAddr / 0x2000);
		temp2 = (startAddr- (temp1-0x40)*0x2000) / 0x800;
		regData = 0x1800 + startAddr - (temp1-0x40)*0x2000 - temp2*0x800;
		if (temp1 != reg10a6) {
			reg10a6 = temp1;
			sensor_write_reg_f(client,0x10a6,reg10a6);
		}
		if (temp2 != reg10a7) {
			reg10a7 = temp2;
			sensor_write_reg_f(client,0x10a7,reg10a7);
		}
		retvalue_l = 0;
		sensor_read_reg_f(client, regData, &retvalue_l);
		printk("====WESLEY===== READ SPI from reg %d=%x\n",i,retvalue_l&0xffff);
		//*(pDataBuf+i) = sensor_read_reg_f(client, regData);
		startAddr++;
	}
	
	sensor_write_reg_f(client,0x10a6, 0x00);
	sensor_write_reg_f(client,0x10a7, 0x00);
}
#endif

#if 0
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4284, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4284 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4285, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4285 %x\n",retvalue_l&0xffff);

        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4178, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4178 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4179, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4179 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x417A, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x417A %x\n",retvalue_l&0xffff);

        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4051, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4051 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4164, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4164 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4170, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4170 %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4171, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4171 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x4172, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4172 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x108C, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x108C %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x1080, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x1080 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x1081, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x1081 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x1082, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x1082 %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x1084, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x1084 %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x40E1, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x40E1 %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x4288, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x4288 %x\n",retvalue_l&0xffff);
        retvalue_l = 0;
	sensor_read_reg_f(client, 0x1011, &retvalue_l);
	printk("spca700xa_16mpf AFTER Wesley SPi Test read 0x1011 %x\n",retvalue_l&0xffff);
#endif

	
	/* Restore SPCA7002 DMA setting */
	sensor_write_reg_f(client, 0x1084, 0);
	sensor_write_reg_f(client, 0x1085, 0);
	sensor_write_reg_f(client, 0x1086, 0);
	sensor_write_reg_f(client, 0x1087, 0);
	sensor_write_reg_f(client, 0x1088, 0);
	sensor_write_reg_f(client, 0x108c, 0);

return status;
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_LoadCode
 *  Description : Load code from host, support boot from host only
 *  ucFwIdx: Set which FW will be loaded
    is_calibration: calibration flag, calibration:1 normal:0
    pIspFw: external ISP FW pointer
    pCaliOpt: read from calibration_option.BIN
    p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
    pLsc: read from LSC.BIN or LSC_F.BIN
    pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_F_LoadCode(
	UINT8 ucFwIdx,
	UINT8 is_calibration,
	UINT8 *pIspFw,
	UINT8 *pCalibOpt,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq,
	struct v4l2_subdev *sd)
{
        UINT8 ucRet = SUCCESS;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	UINT32 t1=0, t2=0, t3=0, tmrCnt=0, i=0;
	UINT16 checksumWrite=0, checksumRead=0;
	FwHeaderInfo_t *pFwInfo;
	ispLoadCodeRet_t loadCodeRet;
	u16 retvalue=0;

        printk("In spca700xa_16mpf EXISP_F_LoadCode\n");
	if (pIspFw == NULL) {
		printk("In spca700xa_16mpf pIspFw is NULL\n");
		return LOADCODE_BOOT_FILE_ERR;
	}

	pFwInfo = (FwHeaderInfo_t *)pIspFw;
	for (i = 0; i < ucFwIdx; i++) {
		pIspFw += FW_HEADER_SIZE+pFwInfo->DmemFicdmemSize+pFwInfo->ImemSize;
		pFwInfo = (FwHeaderInfo_t *)pIspFw;
	}

	/* Modify length to 16-alignment */
	pFwInfo->DmemFicdmemSize = (pFwInfo->DmemFicdmemSize+15)&0xFFFFFFF0;
	pFwInfo->ImemSize = (pFwInfo->ImemSize+15)&0xFFFFFFF0;
	printk("@@@spca700xa_16mpf ISP FW: Get BOOT.BIN Bin File End\n");

#if 1 //update golden or calibration resource
	printk("@@@ISP FW: Update Resource Start\n");
	if (is_calibration == 1) { //calibration
		//Nothing to do
	}
	else { //normal
		memset(&loadCodeRet, 0, sizeof(ispLoadCodeRet_t));

		/* pCalibOpt check */
		if (pCalibOpt == NULL) {
			loadCodeRet.retCalibOpt= LOADCODE_CALIB_OPT_FILE_ERR;
			goto _EXIT_;
		}
		/* p3acali check */
		if (p3acali == NULL) {
			loadCodeRet.ret3acli= LOADCODE_3ACALI_FILE_ERR;
		}
		/* pLsc check */
		if (pLsc == NULL) {
			loadCodeRet.retLsc= LOADCODE_LSC_FILE_ERR;
		}
		/* pLscdq check */
		if (pLscdq == NULL) {
			loadCodeRet.retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
		}
		EXISP_F_UpdateCalibResStart(ucFwIdx, pIspFw, pFwInfo, &loadCodeRet, *pCalibOpt, p3acali, pLsc, pLscdq);
	}
	printk("@@@ISP FW: Update Resource End\n");
#endif
_EXIT_:
	if (is_calibration == 1) { //calibration
		printk("********** spca700xa_16mpf Load Golden Res **********\n");
	}
	else { //normal
		if (loadCodeRet.retCalibOpt == SUCCESS &&
			loadCodeRet.ret3acli == SUCCESS &&
			loadCodeRet.retLsc == SUCCESS &&
			loadCodeRet.retLscdq == SUCCESS) {
			printk("********** spca700xa_16mpf Load Calibration Res **********\n");
		}
		else {
			if (loadCodeRet.retCalibOpt != SUCCESS) {
				printk("********** Load Golden Res, retCalibOpt=%d **********\n", loadCodeRet.retCalibOpt);
			}
			else if (loadCodeRet.ret3acli != SUCCESS) {
				printk("********** Load Golden 3ACALI Res, ret3acli=%d **********\n", loadCodeRet.ret3acli);
			}
			else if (loadCodeRet.retLsc != SUCCESS || loadCodeRet.retLscdq == SUCCESS) {
				printk("********** Load Golden LSC Res, retLsc=%d, retLscdq=%d **********\n", loadCodeRet.retLsc, loadCodeRet.retLscdq);
			}
		}
	}

	/* CPU reset */
	sensor_write_reg_f(client, 0x1011, 1);

	/* Set imemmode*/
	t1 = pFwInfo->ImemSize/8192;
	t2 = t1-32;
	t3 = 1;
	if ((int)t2 < 0) {
		t1 = t3 << t1;
		--t1;
	}
	else {
		t3 <<= t2;
		t1 = -1U;
	}
	--t3;
	sensor_write_reg_f(client, 0x1070, t1);
	sensor_write_reg_f(client, 0x1071, t1>>8);
	sensor_write_reg_f(client, 0x1072, t1>>16);
	sensor_write_reg_f(client, 0x1073, t1>>24);
	sensor_write_reg_f(client, 0x1074, t3);

	/* @@@ Start load code to SPCA7002 */

	/* Enable checksum mechanism */
	sensor_write_reg_f(client, 0x4280, 1);

	/* Wait Ready For Load Code interrupt */
	sensor_read_reg_f(client, SP7K_RDREG_INT_STS_REG_0, &retvalue);
//	while (!(I2CDataRead(SP7K_RDREG_INT_STS_REG_0)&0x02)) {
	while (!(retvalue&0x02)) {

		tmrCnt++;
		if (tmrCnt >= 10) {
			printk("@@@ISP FW: polling RFLC bit timeout\n");
			ucRet = FAIL;
			return ucRet;
		}
		mdelay(10);
	}
	sensor_write_reg_f(client, SP7K_RDREG_INT_STS_REG_0, 0x02);

	/* Load DMEM/FICDMEM bin file Start */
	printk("@@@In spca700xa_16mpf ISP FW: Load DMEM/FICDMEM Bin File Start\n");
	pIspFw += FW_HEADER_SIZE;
	/* Reset checksum value */
	sensor_write_reg_f(client, 0x4284, 0x00);
	checksumWrite = 0;

	/* Allen */

	for (i = 0; i < 6*1024; i++) {
		checksumWrite += pIspFw[i];
	}

	printk("1st 6K=%x\n",checksumWrite);

	checksumWrite = 0;
#if 0
	for (i = 0; i < pFwInfo->DmemFicdmemSize; i++) {
		checksumWrite += pIspFw[i];
	}
	checksumWrite &= 0xFF;
#else
	for (i = 6*1024; i < pFwInfo->DmemFicdmemSize; i++) {
		checksumWrite += pIspFw[i];
	}
	checksumWrite &= 0xFF;

	printk("after 6K=%x\n",checksumWrite);
#endif
	{
	u16 retvalue_l = 0;
	sensor_read_reg_f(client, 0x72f8, &retvalue_l);
	printk("Allen Test 0 spca700xa_16mpf read 0x72f8 %x\n",retvalue_l&0xffff);
	}
	pr_info("DmemFicdmemSize: %d\n", pFwInfo->DmemFicdmemSize);
	/* Transmit DMEM/FICDMEM data */
	if(pFwInfo->DmemFicdmemSize <= 6*1024) { /* Data size <6K, load all bin file */
		printk("Data size < 6K.\n");
		ucRet = EXISP_F_SPIDataWrite(0/*SPI mode0*/, pIspFw, pFwInfo->DmemFicdmemSize, 0x0800, sd);
		sensor_write_reg_f(client, 0x1011, 0); /* Set CPU to normal operation */
	}
	else {
		u16 retvalue_l = 0;

		printk("Data size > 6K.\n");
		ucRet = EXISP_F_SPIDataWrite(0/*SPI mode0*/, pIspFw, 6*1024, 0x0800, sd); //Ryant Modify cali
		sensor_read_reg_f(client, 0x4284, &checksumRead);

		sensor_read_reg_f(client, 0x4284, &retvalue_l);
		printk("Allen Test 1 spca700xa_16mpf read 0x4284 %x\n",retvalue_l&0xffff);

		retvalue_l = 0;
		sensor_read_reg_f(client, 0x72f8, &retvalue_l);
		printk("Allen Test 1_1 spca700xa_16mpf read 0x72f8 %x\n",retvalue_l&0xffff);
		sensor_write_reg_f(client, 0x1011, 0); /* Set CPU to normal operation */

		sensor_read_reg_f(client, 0x4284, &retvalue_l);

		retvalue_l = 0;
		sensor_read_reg_f(client, 0x72f8, &retvalue_l);
		printk("Allen Test 1_2 spca700xa_16mpf read 0x72f8 %x\n",retvalue_l&0xffff);

		printk("Allen Test 2 spca700xa_16mpf read 0x4284 %x\n",retvalue_l&0xffff);

		/* Reset checksum value */
		sensor_write_reg_f(client, 0x4284, 0x00);
		ucRet = EXISP_F_SPIDataWrite(0/*SPI mode0*/, pIspFw+(6*1024), pFwInfo->DmemFicdmemSize-(6*1024), 0x0800+(6*1024), sd); //Ryant Modify cali
		sensor_read_reg_f(client, 0x4284, &retvalue_l);
		printk("Allen Test 3 spca700xa_16mpf read 0x4284 %x\n",retvalue_l&0xffff);

		retvalue_l = 0;
		sensor_read_reg_f(client, 0x72f8, &retvalue_l);
		printk("Allen Test 3_1 spca700xa_16mpf read 0x72f8 %x\n",retvalue_l&0xffff);
	}

	/* Checksum value check */
	sensor_read_reg_f(client, 0x4284, &checksumRead);
	if (checksumWrite == checksumRead) {
		printk("@@@In spca700xa_16mpf ISP FW: Checksum DMEM/FICDMEM test: OK, %x, %x\n", checksumRead, checksumWrite);
	}
	else {
		printk("@@@In spca700xa_16mpf ISP FW: Checksum DMEM/FICDMEM test: FAIL, %x, %x\n", checksumRead, checksumWrite);
		ucRet = FAIL;
		return ucRet;
	}
	printk("@@@ISP FW: Load DMEM/FICDMEM Bin File End\n");
	/* Load DMEM/FICDMEM bin file End */

	/* Load IMEM bin file Start */
	printk("@@@ISP FW: Load IMEM Bin File Start\n");
	pIspFw += pFwInfo->DmemFicdmemSize;
	/* Reset checksum value */
	sensor_write_reg_f(client, 0x4284, 0x00);
	checksumWrite = 0;
	pr_info("ImemSize: %d\n", pFwInfo->ImemSize);
	for (i = 0; i < pFwInfo->ImemSize; i++) {
		checksumWrite += pIspFw[i];
	}
	checksumWrite &= 0xFF;

	/* Transmit IMEM data */
	ucRet = EXISP_F_SPIDataWrite(0/*SPI mode0*/, pIspFw, pFwInfo->ImemSize, (320*1024)-pFwInfo->ImemSize, sd); //Ryant Modify cali
	/* Checksum value check */
	sensor_read_reg_f(client, 0x4284, &checksumRead);
	if (checksumWrite == checksumRead) {
		printk("@@@In spca700xa_16mpf ISP FW: Checksum IMEM test: OK, %x, %x\n", checksumRead, checksumWrite);
	}
	else {
		printk("@@@In spca700xa_16mpf ISP FW: Checksum IMEM test: FAIL, %x, %x\n", checksumRead, checksumWrite);
		ucRet = FAIL;
		return ucRet;
	}
	printk("@@@In spca700xa_16mpf ISP FW: Load IMEM Bin File End\n");
	/* Load IMEM bin file End */

	/* @@@ End load code to SPCA7002 */

	/* Disable checksum mechanism */
	sensor_write_reg_f(client, 0x4280, 0);

	/* Write load code end register */
	sensor_write_reg_f(client, 0x1307, 0xA5);

	return ucRet;
}
//ASUS_BSP-- Wesley, For load form host
/*Ryant Add for camera calibration ++ */
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_UpdateCalibResStart
 *  Description : Update calibration data start
 *  ucFwIdx: Set which FW will be loaded
     pIspFw: external ISP FW pointer
     pFwInfo: external ISP FW header information
     pLoadCodeRet: load code status result
     ucCaliOpt: read from calibration_option.BIN
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : none
 *------------------------------------------------------------------------*/
void EXISP_F_UpdateCalibResStart(
	UINT8 ucFwIdx,
	UINT8 *pIspFw,
	FwHeaderInfo_t *pFwInfo,
	ispLoadCodeRet_t *pLoadCodeRet,
	UINT8 ucCaliOpt,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq)
{
	printk("spca700xa_16mpf In =%s\n", __FUNCTION__);
	if ((ucFwIdx == 0) && (ucCaliOpt != 0xFF)) { //rear sensor
		ucCaliOpt = ucCaliOpt & 0x03;
	}
	else if ((ucFwIdx == 1) && (ucCaliOpt != 0xFF)){ //front sensor
		ucCaliOpt = ucCaliOpt >> 2;
	}
	printk("ucCaliOpt =%d\n", ucCaliOpt);
	switch (ucCaliOpt) {
	case 1: /* load golden 3ACALI.BIN and calibrated LSC.BIN & LSC_DQ.BIN */
		printk("Ryant In 1\n");
		if (pLsc == NULL || pLscdq == NULL) {
			pLoadCodeRet->retLsc = LOADCODE_LSC_FILE_ERR;
			pLoadCodeRet->retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
			break;
		}
		pLoadCodeRet->retLsc = EXISP_F_UpdateCalibRes(1, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		pLoadCodeRet->retLscdq = pLoadCodeRet->retLsc;
		break;
	case 2: /* load calibrated 3ACALI.BIN and golden LSC.BIN & LSC_DQ.BIN */
		printk("Ryant In 2\n");
		if (p3acali == NULL) {
			pLoadCodeRet->ret3acli = LOADCODE_3ACALI_FILE_ERR;
			break;
		}
		pLoadCodeRet->ret3acli = EXISP_F_UpdateCalibRes(0, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		break;
	case 3: /* load golden 3ACALI.BIN and golden LSC.BIN & LSC_DQ.BIN */
		break;
	default: /* load calibrated 3ACALI.BIN and calibrated LSC.BIN & LSC_DQ.BIN */
		printk("Ryant In default \n");
		if (p3acali == NULL) {
			printk("Ryant In p3acali is NULL \n");
			pLoadCodeRet->ret3acli = LOADCODE_3ACALI_FILE_ERR;
		}
		if (pLoadCodeRet->ret3acli == SUCCESS) {
			printk("Ryant In ret3acli is SUCCESS \n");
			pLoadCodeRet->ret3acli = EXISP_F_UpdateCalibRes(0, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		}
		if (pLoadCodeRet->ret3acli == LOADCODE_GET_RES_NUM_ERR) {
			printk("Ryant In ret3acli is ERR \n");
			break;
		}
		else if (pLsc == NULL || pLscdq == NULL) {
			printk("Ryant In pLsc and pLscdq are Null \n");
			pLoadCodeRet->retLsc = LOADCODE_LSC_FILE_ERR;
			pLoadCodeRet->retLscdq = LOADCODE_LSC_DQ_FILE_ERR;
			break;
		}
		pLoadCodeRet->retLsc = EXISP_F_UpdateCalibRes(1, pIspFw, pFwInfo, p3acali, pLsc, pLscdq);
		pLoadCodeRet->retLscdq = pLoadCodeRet->retLsc;
		break;
	}
}



/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_UpdateCalibRes
 *  Description : Update calibration data from host, support boot from host only
 *  idx: Set which resource will be loaded
     pIspFw: external ISP FW pointer
     pFwInfo: external ISP FW header information
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_F_UpdateCalibRes(
	UINT8 idx,
	UINT8 *pIspFw,
	FwHeaderInfo_t *pFwInfo,
	UINT8 *p3acali,
	UINT8 *pLsc,
	UINT8 *pLscdq)
{
	UINT32 iqOffset = 0, resNumCnt = 0, iqSize = 0, caliSize = 0, tempSize = 0, i;
	UINT32 start3acali = 0, startLsc = 0, startLscdq = 0, size3acali = 0, sizeLsc = 0, sizeLscdq = 0;
	UINT8 ucRet = SUCCESS;
        printk("Ryant In %s\n",__FUNCTION__);

	/* resource header and checksum check */
	ucRet = EXISP_F_ResCheck(idx, p3acali, pLsc, pLscdq);
	if (ucRet != SUCCESS) {
		goto _EXIT_;
	}

	/* find out where IQ.BIN is */
	pIspFw += FW_HEADER_SIZE + pFwInfo->DmemSize;
	if (*(pIspFw+0x38) == 0x43 &&
		*(pIspFw+0x39) == 0x41 &&
		*(pIspFw+0x3A) == 0x4C &&
		*(pIspFw+0x3B) == 0x49) {
		iqOffset = ((*(pIspFw+0x51)<<8)&0x0000FF00) + (*(pIspFw+0x50)&0x000000FF);
	}
	else {
		iqOffset = ((*(pIspFw+0x31)<<8)&0x0000FF00) + (*(pIspFw+0x30)&0x000000FF);
	}
	printk("iqOffset=%x\n", iqOffset);

	/* point to IQ.BIN start position */
	pIspFw += iqOffset;
	//printk("3.pIspFw =%x\n", pIspFw);
	/* parsing out the file size to get the start position of calibration data,
	FICDMEM file size should be 16 alignment */
	ucRet = EXISP_F_ResNumGet(&resNumCnt, pIspFw+0x10);
	if (ucRet != SUCCESS) {
		goto _EXIT_;
	}
	printk("resNumCnt=%d\n", resNumCnt);
	for (i = 0; i < resNumCnt; i++) {
		tempSize = *(pIspFw+14+(1+i)*0x10);
		tempSize += ((*(pIspFw+15+((1+i)*0x10)))<<8);
		if ((tempSize%0x10) != 0) {
			tempSize = ((tempSize+0xF)>>4)<<4;
		}
		iqSize += tempSize;
	}
	start3acali = iqSize+(1+resNumCnt+3)*0x10;
	for (i = 0; i < 3; i++) {
		tempSize = *(pIspFw+14+(1+resNumCnt+i)*0x10);
		tempSize += ((*(pIspFw+15+(1+resNumCnt+i)*0x10))<<8);
		if (i == 0) {
			size3acali = tempSize;
		}
		else if (i == 1){
			sizeLsc = tempSize;
		}
		else {
			sizeLscdq = tempSize;
		}
		if ((tempSize%0x10) != 0) {
			tempSize = ((tempSize+0xF)>>4)<<4;
		}
		caliSize += tempSize;
		if (i == 0) {
			startLsc = start3acali + caliSize;
		}
		else if (i == 1) {
			startLscdq = start3acali + caliSize;
		}
	}
	printk("In spca700xa_16mpf start3acali=%x, size3acali=%d\n", start3acali, size3acali);
	printk("In spca700xa_16mpf startLsc=%x, sizeLsc=%d\n", startLsc, sizeLsc);
	printk("In spca700xa_16mpf startLscdq=%x, size=%d\n", startLscdq, sizeLscdq);
	if (idx == 0) {
		memcpy(pIspFw+start3acali, p3acali, size3acali);
	}
	else if (idx == 1) {
		memcpy(pIspFw+startLsc, pLsc, sizeLsc);
		memcpy(pIspFw+startLscdq, pLscdq, sizeLscdq);
	}
_EXIT_:
	return ucRet;
}
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_ResCheck
 *  Description : check resource header and checksum
 *  idx: Set which resource will be loaded
     p3acali: read from 3ACALI.BIN or 3ACALI_F.BIN
     pLsc: read from LSC.BIN or LSC_F.BIN
     pLscdq: read from LSC_DQ.BIN or LSC_DQ_F.BIN
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_F_ResCheck(UINT8 idx, UINT8 *p3acali, UINT8 *pLsc, UINT8 *Lscdq)
{
	UINT8 ucRet = SUCCESS;
	UINT32 header_3ACALI = 0, header_LSC = 0, header_LSCDQ = 0;
	UINT32 chksuum_3ACALI = 0, chksuum_LSC = 0, chksuum_LSCDQ = 0, checksum = 0, dataSize = 0, i;
	printk("In spca700xa_16mpf %s\n", __FUNCTION__);
	if (idx == 0) { //3ACALI
		header_3ACALI = *(p3acali);
		header_3ACALI += (*(p3acali+1)<<8);
		header_3ACALI += (*(p3acali+2)<<16);
		header_3ACALI += (*(p3acali+3)<<24);
		if (header_3ACALI == 0xFFFFFFFF || header_3ACALI == 0x00000000) {
			printk("header_3ACALI=0x%04x\n", header_3ACALI);
			ucRet = LOADCODE_3ACALI_HEADER_ERR;
			goto _EXIT_;
		}
	}
	else if (idx == 1) { //LSC & LSC_DQ
		header_LSC = *(pLsc);
		header_LSC += (*(pLsc+1)<<8);
		header_LSC += (*(pLsc+2)<<16);
		header_LSC += (*(pLsc+3)<<24);
		if (header_LSC == 0xFFFFFFFF || header_LSC == 0x00000000) {
			printk("header_LSC=0x%04x\n", header_LSC);
			ucRet = LOADCODE_LSC_HEADER_ERR;
			goto _EXIT_;
		}
		header_LSCDQ = *(Lscdq);
		header_LSCDQ += (*(Lscdq+1)<<8);
		header_LSCDQ += (*(Lscdq+2)<<16);
		header_LSCDQ += (*(Lscdq+3)<<24);
		if (header_LSCDQ == 0xFFFFFFFF || header_LSCDQ == 0x00000000) {
			printk("header_LSCDQ=0x%04x\n", header_LSCDQ);
			ucRet = LOADCODE_LSC_DQ_HEADER_ERR;
			goto _EXIT_;
		}
	}
	if (idx == 0) { //3ACALI
		dataSize = *(p3acali+6);
		dataSize += (*(p3acali+7)<<8);
		checksum = *(p3acali+RES_3ACALI_HEADER_SIZE);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+1)<<8);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+2)<<16);
		checksum += (*(p3acali+RES_3ACALI_HEADER_SIZE+3)<<24);
		for (i = 0; i < dataSize-sizeof(UINT32); i++) {
			chksuum_3ACALI = chksuum_3ACALI + (*(p3acali+RES_3ACALI_HEADER_SIZE+sizeof(UINT32)+i));
		}
		if (chksuum_3ACALI != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_3ACALI=0x%04x\n", dataSize, checksum, chksuum_3ACALI);
			ucRet = LOADCODE_3ACALI_CHKSUM_ERR;
			goto _EXIT_;
		}
	}
	else if (idx == 1) { //LSC & LSC_DQ
		dataSize = *(pLsc+6);
		dataSize += (*(pLsc+7)<<8);
		checksum = *(pLsc+RES_LSC_HEADER_SIZE-4);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-3)<<8);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-2)<<16);
		checksum += (*(pLsc+RES_LSC_HEADER_SIZE-1)<<24);
		for (i = 0; i < dataSize; i++) {
			chksuum_LSC = chksuum_LSC + (*(pLsc+RES_LSC_HEADER_SIZE+i));
		}
		if (chksuum_LSC != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_LSC=0x%04x\n", dataSize, checksum, chksuum_LSC);
			ucRet = LOADCODE_LSC_CHKSUM_ERR;
			goto _EXIT_;
		}
		dataSize = *(Lscdq+6);
		dataSize += (*(Lscdq+7)<<8);
		checksum = *(Lscdq+RES_LSCDQ_HEADER_SIZE-4);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-3)<<8);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-2)<<16);
		checksum += (*(Lscdq+RES_LSCDQ_HEADER_SIZE-1)<<24);
		for (i = 0; i < dataSize; i++) {
			chksuum_LSCDQ = chksuum_LSCDQ + (*(Lscdq+RES_LSCDQ_HEADER_SIZE+i));
		}
		if (chksuum_LSCDQ != checksum) {
			printk("dataSize=%d, checksum=0x%04x, chksuum_LSCDQ=0x%04x\n", dataSize, checksum, chksuum_LSCDQ);
			ucRet = LOADCODE_LSC_DQ_CHKSUN_ERR;
			goto _EXIT_;
		}
	}
_EXIT_:
	return ucRet;
}
/*-------------------------------------------------------------------------
 *  Function Name : EXISP_F_ResNumGet
 *  Description : get the resource number in the IQ.BIN
 *  resNum: resource number
     pIspFw: external ISP FW pointer
 *  Return : status result
 *------------------------------------------------------------------------*/
UINT8 EXISP_F_ResNumGet(UINT32 *resNum, UINT8 *pIspFw)
{
	UINT8 i = 0, ucRet = SUCCESS;
	printk("In spca700xa_16mpf %s\n", __FUNCTION__);
	while (1) {
		if ((*(pIspFw) == 0x33) && (*(pIspFw+1) == 0x41) && (*(pIspFw+2) == 0x43) &&
			(*(pIspFw+3)==0x41) && (*(pIspFw+4)==0x4C) && (*(pIspFw+5)==0x49)) {
			break;
		}
		i++;
		pIspFw += 0x10;
		if (i > 30) {
			ucRet = LOADCODE_GET_RES_NUM_ERR;
			goto _EXIT_;
		}
	}
_EXIT_:
	*resNum = i;
	return ucRet;
}

//ASUS_BSP++ Wesley, For get FW ver from bin
int get_fw_version_in_bin_f(void)
{
	u8 *pbootBuf = NULL;

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];

	printk("%s\n", __func__);
	/* Calculate BOOT.BIN file size. */
	fp = filp_open(FW_BIN_FILE_WITH_PATH, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		printk("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", FW_BIN_FILE_WITH_PATH);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);
#if 0
			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				return -1;
			} else
#endif
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		printk("iCatch \"%s\" not found error\n", FW_BIN_FILE_WITH_PATH);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return -1;
	} else{
		printk("iCatch \"%s\" open error\n", FW_BIN_FILE_WITH_PATH);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return -1;
	}

	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	kfree(pbootBuf);

	return 0;
}
//ASUS_BSP-- Wesley, For get FW ver from bin

static int spca700xa_16mpf_init_common(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	int ret = 0;
        u16 retvalue_l = 0;

	int status;
	u8 *pbootBuf = NULL;

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	UINT32 bootbin_size = 0;

	printk("==== 0121 WESLEY === %s\n", __func__);
//ASUS_BSP+++
    lastSceneMode = 0;
    lastEffect = 0;
    lastEffectAura = 0;
    lastEV = -100;
    lastWhiteBalance = -1;
//ASUS_BSP---
#if 0
	//Wesley 20150114 ++++ , Load from spi rom
	ret = sensor_write_reg_f(client, 0x1011, 0x01);//cpu reset
	if(ret) {
		dev_err(&client->dev, "%s, I2C transfer fail, break!!\n", __func__);
		return -EINVAL;
	}
	sensor_write_reg_f(client, 0x001C, 0x08);// reset FM
	sensor_write_reg_f(client, 0x001C, 0x00);
	sensor_write_reg_f(client, 0x1010, 0x02);
	sensor_write_reg_f(client, 0x1010, 0x00);

	//ASUS_BSP+++, for calibration
	printk("Start calibration to Front camera!!\n");
	sensor_write_reg_f(client, 0x1306, 0x01);//rear camera: 0, front camera: 1, rear calibration: 2
	sensor_read_reg_f(client, 0x1306, &retvalue_l); //calibration: 2
        
	printk("spca700xa_16mpf read 0x1306 %x\n",retvalue_l&0xffff);
	//ASUS_BSP---, for calibration
	sensor_write_reg_f(client, 0x1011, 0x00);
	msleep(70); //wait for iCatch load sensor code
	//Wesley 20150114 ---- , Load from spi rom
#endif

#if 1
	//Wesley 20150114 ++++ , For load from host
	/* Calculate BOOT.BIN file size. */
	fp = filp_open(FW_BIN_FILE_WITH_PATH, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = vmalloc(bootbin_size);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", FW_BIN_FILE_WITH_PATH);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);

			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d retry_time: %d;\n", byte_count, retry_time);
				if(pbootBuf!=NULL){
					kfree(pbootBuf);
				}
				if(retry_time > 0){
					retry_time --;
					goto retry;
				} else{
					retry_time = 3;
					fw_update_status = ICATCH_FW_UPDATE_FAILED;
					goto end;
				}
			} else{
				retry_time = 3;
				printk("iCatch: BIN file size= %d bytes byte_count:%d \n", bootbin_size, byte_count);
			}
#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("init_common_ ==== 0121 WESLEY === iCatch \"%s\" not found error\n", FW_BIN_FILE_WITH_PATH);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	} else{
		pr_err("init_common_ ==== 0121 WESLEY === iCatch \"%s\" open error\n", FW_BIN_FILE_WITH_PATH);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}
	status = EXISP_F_LoadCode(0x01, g_is_calibration_f, pbootBuf, pCalibOpt_g_f, p3acali_g_f, pLsc_g_f, pLscdq_g_f, sd); //Add for calibration
	//status = EXISP_F_LoadCode(0x01, pbootBuf, bootbin_size, sd);
	fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
	sensor_write_reg_f(client, 0x1306, 0x01); /* Front_calibration: 1 */
	sensor_read_reg_f(client, 0x002c, &retvalue_l); /*DBG 0x002c -> 0x2 is Pre_ER version*/
	printk("mt9m114 read 0x002c 0x%x\n",retvalue_l&0xffff);

	if(pbootBuf!=NULL){
		vfree(pbootBuf);
	}
	//Wesley 20150114 ---- , For load from host
#endif

	/*For debug read reg ++*/
	retvalue_l = 0;
	ret = sensor_read_reg_f(client, 0x1300, &retvalue_l);
	printk("spca700xa_16mpf read 0x1300 %x\n",retvalue_l&0xffff);
	sensor_write_reg_f(client, 0x1300, 0x5a);
	ret = sensor_read_reg_f(client, 0x1300, &retvalue_l);
	printk("spca700xa_16mpf read 0x1300 %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x002c, &retvalue_l);
	printk("spca700xa_16mpf read 0x002c %x\n",retvalue_l&0xffff);
	retvalue_l = 0;
        sensor_write_reg_f(client, 0x72F8, 0xFF);
	/*For debug read reg --*/

	dev->format.code = SPCA700XA_BAYER_ORDER;

	return 0;

	//Wesley 20150114 ---- , For load from host
end:
	ret = spca700xa_16mpf_s_power(sd, 0);
	if (ret) {
		dev_err(&client->dev, "ov5693 power down err");
	}
	return -1;
	//Wesley 20150114 ---- , For load from host
retry:
       return spca700xa_16mpf_init_common(sd);
}

static int power_2v8_control(struct v4l2_subdev *sd, int val) {
	int ret = 0;
	pr_info("%s: val=%d\n", __func__, val);

	if (val > 0) {
		ret = pltfrm_camera_module_set_pin_state(sd,
			PLTFRM_CAMERA_MODULE_PIN_ICATCH_AVDD,
			PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);

			printk("intel,pd-gpio_2v8 --> 1 ret:%d\n", ret);
	} else {
		ret = pltfrm_camera_module_set_pin_state(sd,
			PLTFRM_CAMERA_MODULE_PIN_ICATCH_AVDD,
			PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);

		printk("intel,pd-gpio_2v8 --> 0 ret:%d\n", ret);
	}

	return ret;
}
static int power_down(struct v4l2_subdev *sd);
static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	printk("%s\n", __func__);

	ret = pltfrm_camera_module_init_gpio(sd);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"GPIO initialization failed (%d)\n", ret);
		goto fail_power;
	}

	/* pull low reset first */
	ret = pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,"step1 reset pull low failed (%d) \n", ret);
	}
	usleep_range(1000, 1100);

        /* pull up 1v2 */
	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_ICATCH_DVDD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,"step2, pull 1v2 failed (%d) \n", ret);
	}
        usleep_range(1000, 1100);

	/* pull up 1v8 */
	/* Enable clock and voltage to Secondary Camera Sensor	*/
	ret = pltfrm_camera_module_set_pm_state(sd, PM_STATE_D0);
       //ret = pltfrm_camera_module_s_power(sd, 1);	
       if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"set PM state failed (%d), could not power on camera\n",
			ret);
		goto fail_power;
	} else {
		pltfrm_camera_module_pr_debug(sd,
			"set PM state to %d successful, camera module is on\n",
			PM_STATE_D0);
	}

        usleep_range(1000, 1100);
        spca700xa_SPI_off(0);


	/* pull up 2v8 */
        ret = power_2v8_control(sd, 1);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,"step4, pull 2v8 failed (%d) \n", ret);
	}

	/*Need change load code flow here +++*/
	//Pull high suspend to load fw from SPI / Pull low suspend to load fw from host
	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		//PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE); // from ROM
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE); // from HOST
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,"step5, pull low suspend failed(%d) \n", ret);
	}

	//Wesley +++ , 20150114
	/*We need pull high SPI CLK here and msleep(6)*/
	spca700xa_SPI_clk_control(1);
	//msleep(6);
        mdelay(6);
	//Wesley --- , 20150114

	/* Reset control */
	ret = pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,"step6, pull low reset failed (%d) \n", ret);
	}
	usleep_range(6000, 6100); //wait 6ms

	//Wesley +++ , 20150114
	/*We need pull low SPI CLK and set all SPI pin to ALT_1 mode*/
	spca700xa_SPI_clk_control(0);
	//Wesley --- , 20150114

	/* Pull low suspend */
	ret |= pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_PD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd, "failed with error %d\n", ret);
		goto fail_power;
	}

	/* delay time for first i2c command */
	usleep_range(10000, 11000);

	pltfrm_camera_module_pr_info(sd, "sensor power-up done.\n");

	mHDRMode = false;
	mHDR = false;
	mNightMode = false;
	memset(&old_f, 0, sizeof(old_f));
	skip_cnt = 0;
	//Wesley Final test 0120+++
	/*if (!ret) {
		printk("WESLEY cam driver init before ==== 01\n");
		spca700xa_16mpf_init_common(sd);
		printk("WESLEY cam driver init over ==== 02\n");
	}else {
		printk("WESLEY ADD not init ==== 03\n");
	}*/
	//Wesley Final test 0120---

	return ret;

fail_power:
	//pltfrm_camera_module_s_power(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");
	power_down(sd);

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	int ret;
	pr_debug("%s\n", __func__);
	//pltfrm_camera_module_release(sd);

	/* pull low reset first */
	ret = pltfrm_camera_module_set_pin_state(sd,
		"intel,rst-gpio",
		PLTFRM_CAMERA_MODULE_PIN_STATE_ACTIVE);
	printk("<<< camera_reset = 0\n");
	mdelay(3);

        spca700xa_SPI_off(1);

        usleep_range(1000, 1100);
        power_2v8_control(sd, 0);

	/* power control */
	/* Disable clock and voltage to Secondary Camera Sensor  */
	ret = pltfrm_camera_module_set_pm_state(
		sd, PM_STATE_D3);
	if (IS_ERR_VALUE(ret))
		pltfrm_camera_module_pr_err(sd,
			"set PM state failed (%d), could not power off camera\n",
			ret);
	else
		pltfrm_camera_module_pr_debug(sd,
			"set PM state to %d successful, camera module is off\n",
			PM_STATE_D3);

	ret = pltfrm_camera_module_set_pin_state(sd,
		PLTFRM_CAMERA_MODULE_PIN_ICATCH_DVDD,
		PLTFRM_CAMERA_MODULE_PIN_STATE_INACTIVE);


	ret |= pltfrm_camera_module_release_gpio(sd);
	if (IS_ERR_VALUE(ret)) {
		pltfrm_camera_module_pr_err(sd,
			"GPIO release failed (%d)\n", ret);
		return ret;
	}

#if SKIP_FIRST_COMMAND
	first_on = 0;
#endif
	return ret;
}

static int spca700xa_16mpf_s_power(struct v4l2_subdev *sd, int power)
{
	printk("%s\n", __func__);
    if (power == 0)
        return power_down(sd);
	printk("====WESLEY power up in _s_power\n");
        spca700xa_16mpf_s_config(sd);
	if (power_up(sd)) {
		printk("==== WESLEY AFTER POWER UP 001 BEFORE init_comm =====\n");
		//only for pin test - dont close power(2/2)
		spca700xa_16mpf_init_common(sd);
		//only for pin test - dont close power(2/2)
		printk("==== WESLEY AFTER POWER UP 002 AFTER init_comm =====\n");
		printk("==== WESLEY NO POWER DOWN =====\n");
		return -EINVAL;
	}
	printk("==== WESLEY NO POWER DOWN but init =====\n");
	//only for pin test - dont close power(2/2)
	return spca700xa_16mpf_init_common(sd);
}

static int spca700xa_16mpf_try_res(u32 *w, u32 *h)
{
	int i;

	pr_debug("%s\n", __func__);

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (i = 0; i < N_RES; i++) {
		if ((spca700xa_16mpf_res[i].width >= *w) &&
		    (spca700xa_16mpf_res[i].height >= *h))
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (i == N_RES)
		i--;

	*w = spca700xa_16mpf_res[i].width;
	*h = spca700xa_16mpf_res[i].height;

	return 0;
}

static struct spca700xa_16mpf_res_struct *spca700xa_16mpf_to_res(u32 w, u32 h)
{
	int  index;

	pr_debug("%s\n", __func__);

	for (index = 0; index < N_RES; index++) {
		if ((spca700xa_16mpf_res[index].width == w) &&
		    (spca700xa_16mpf_res[index].height == h))
			break;
	}

	/* No mode found */
	if (index >= N_RES)
		return NULL;

	return &spca700xa_16mpf_res[index];
}

static int spca700xa_16mpf_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	return spca700xa_16mpf_try_res(&fmt->width, &fmt->height);
}

static int spca700xa_16mpf_res2size(unsigned int res, int *h_size, int *v_size)
{
	unsigned short hsize;
	unsigned short vsize;

	pr_debug("%s\n", __func__);

	switch (res) {
	case SPCA700XA_16MPF_RES_QCIF:
		hsize = SPCA700XA_16MPF_RES_QCIF_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_QCIF_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_CIF:
		hsize = SPCA700XA_16MPF_RES_CIF_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_CIF_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_QVGA:
		hsize = SPCA700XA_16MPF_RES_QVGA_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_QVGA_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_VGA:
		hsize = SPCA700XA_16MPF_RES_VGA_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_VGA_SIZE_V;
		break;
/*
	case SPCA700XA_16MPF_RES_480P:
		hsize = SPCA700XA_16MPF_RES_480P_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_480P_SIZE_V;
		break;
*/
	case SPCA700XA_16MPF_RES_720P:
		hsize = SPCA700XA_16MPF_RES_720P_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_720P_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_960P:
		hsize = SPCA700XA_16MPF_RES_960P_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_960P_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_2M:
		hsize = SPCA700XA_16MPF_RES_2M_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_2M_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_1080P:
		hsize = SPCA700XA_16MPF_RES_1080P_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_1080P_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_5MP:
		hsize = SPCA700XA_16MPF_RES_5MP_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_5MP_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_8MP:
		hsize = SPCA700XA_16MPF_RES_8MP_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_8MP_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_13MP:
		hsize = SPCA700XA_16MPF_RES_13MP_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_13MP_SIZE_V;
		break;
	case SPCA700XA_16MPF_RES_16MP:
		hsize = SPCA700XA_16MPF_RES_16MP_SIZE_H;
		vsize = SPCA700XA_16MPF_RES_16MP_SIZE_V;
		break;
	default:
		WARN(1, "%s: Resolution 0x%08x unknown\n", __func__, res);
		return -EINVAL;
	}

	if (h_size != NULL)
		*h_size = hsize;
	if (v_size != NULL)
		*v_size = vsize;
	return 0;
}

static int spca700xa_16mpf_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	int width, height;
	int ret;

	pr_debug("%s\n", __func__);

	ret = spca700xa_16mpf_res2size(dev->res, &width, &height);
	if (ret)
		return ret;
	fmt->code = dev->format.code;
	fmt->width = width;
	fmt->height = height;

	return 0;
}

static int spca700xa_16mpf_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	struct spca700xa_16mpf_res_struct *res_index;
	//struct camera_mipi_info *spca700xa_16mp_info = NULL;
	u32 width = fmt->width;
	u32 height = fmt->height;
	//int ret;
	u16 testval, i, addr;
	printk("%s(%dx%d)\n", __func__, width, height);
#if 0
	struct atomisp_sensor_mode_data *buf;

	spca700xa_16mp_info = v4l2_get_subdev_hostdata(sd);
	
	if (spca700xa_16mp_info == NULL)
		return -EINVAL;
	
	buf = &spca700xa_16mp_info->data;

	buf->binning_factor_x = 1;
	buf->binning_factor_y = 1;

	buf->vt_pix_clk_freq_mhz = 19200000;

	buf->coarse_integration_time_min = 0;
	buf->coarse_integration_time_max_margin = 0;
	buf->fine_integration_time_min = 0;
	buf->fine_integration_time_max_margin = 0;
	buf->fine_integration_time_def = 0;

	buf->frame_length_lines = 3280;
	buf->line_length_pck = 2464;
	buf->read_mode = 0;

	buf->crop_horizontal_start = 0;
	buf->crop_vertical_start = 0;

	buf->crop_horizontal_end = 3280;
	buf->crop_vertical_end = 2464;

	buf->output_width = 3280;
	buf->output_height = 2464;
#endif

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	if (mHDR && old_f.w == width && old_f.h == height) {
		printk("%s old (%dx%d)\n", __func__, old_f.w , old_f.h);
		printk("%s new (%dx%d)\n", __func__, width, height);
		printk("%s size was not changed, return\n", __func__);
		dev->res = old_f.res;
		dev->format.code = old_f.code;
		fmt->width =  old_f.w;
		fmt->height = old_f.h;
		return 0;
	}
#endif

	spca700xa_16mpf_try_res(&width, &height);
	res_index = spca700xa_16mpf_to_res(width, height);
	printk("%s RES %s selected\n", __func__, res_index->desc);

	/* Sanity check */
	if (unlikely(!res_index)) {
		WARN_ON(1);
		return -EINVAL;
	}

	if (mHDR) {
		printk("%s ===== HDR mode =====\n", __func__);
		sensor_write_reg_f(client, 0x7132, 0x00);
		sensor_write_reg_f(client, 0x710F, 0x01);  //HDR mode
		sensor_write_reg_f(client, 0x7127, 0x15);  //HDR Positive EV
		sensor_write_reg_f(client, 0x7128, 0x15);  //HDR Nagative EV
		sensor_write_reg_f(client, 0x7124, 0x00); //Reset fix frame rate

		switch (res_index->res) {
			case SPCA700XA_16MPF_RES_VGA:
				sensor_write_reg_f(client, 0x7108, 0x0B);
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_720P:
				sensor_write_reg_f(client, 0x7108, 0x04);  //test pattern 0x28
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_2M:
				sensor_write_reg_f(client, 0x7108, 0x16);
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_1080P:
				sensor_write_reg_f(client, 0x7108, 0x02); //test pattern 0x27
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_5MP:
				sensor_write_reg_f(client, 0x7108, 0x0A); //test pattern 0x2A
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_8MP:
				sensor_write_reg_f(client, 0x7108, 0x01); //test pattern 0x24
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_13MP:
				sensor_write_reg_f(client, 0x7108, 0x08); //test pattern 0x2B
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			default:
				dev_err(&client->dev, "set resolution: %d failed!\n", res_index->res);
				return -EINVAL;
		}
		sensor_write_reg_f(client, 0x7120, 0x01);
	} else {
	sensor_write_reg_f(client, 0x710F, 0x00);
#if defined(CONFIG_Z170CG) || defined(CONFIG_Z170C)
		if (mNightMode)
			sensor_write_reg_f(client, 0x7124, 0x00);
		else
			sensor_write_reg_f(client, 0x7124, 0x1E); //Reset fix frame rate
#else
		if (mHDRMode)
			sensor_write_reg_f(client, 0x7124, 0x1E);
		else
			sensor_write_reg_f(client, 0x7124, 0x00);
#endif
		switch (res_index->res) {
			case SPCA700XA_16MPF_RES_VGA:
				sensor_write_reg_f(client, 0x7106, 0x0B);
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_720P:
				sensor_write_reg_f(client, 0x7106, 0x04);  //test pattern 0x28
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_2M:
				sensor_write_reg_f(client, 0x7106, 0x16);
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_1080P:
				sensor_write_reg_f(client, 0x7106, 0x02); //test pattern 0x27
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_5MP:
				sensor_write_reg_f(client, 0x7106, 0x0A); //test pattern 0x2A
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_8MP:
				sensor_write_reg_f(client, 0x7106, 0x01); //test pattern 0x24
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			case SPCA700XA_16MPF_RES_13MP:
				sensor_write_reg_f(client, 0x7106, 0x08); //test pattern 0x2B
				dev_info(&client->dev, "%s: set for %s\n", __func__, res_index->desc);
				break;
			default:
				dev_err(&client->dev, "set resolution: %d failed!\n", res_index->res);
				return -EINVAL;
		}
		printk("%s ===== Preview mode =====\n", __func__);
		sensor_write_reg_f(client, 0x7120, 0x00);
		//wait interrupt 0x72F8.2
		for (i = 0 ; i < 500 ; i++) {
			sensor_read_reg_f(client, 0x72f8, &testval);
			pr_debug("testval=0x%X, i=%d ", testval, i);
			if (testval & 0x04) {
				sensor_write_reg_f(client, 0x72f8, 0x04);
				sensor_read_reg_f(client, 0x72f8, &testval);
				pr_debug("Clear testval=0x%X, i=%d\n",testval,i);
				break;
			}
			usleep_range(2000, 2100);
		}
		if (i == 500) {
			pr_info("Change to preview mode fail\n");
#if 1
			printk("-- Dump iCatch register now --\n");
			testval = 0;
			sensor_read_reg_f(client, 0x002C, &testval);
			printk("addr=0x002C, value=0x%x \n",testval);

			for (addr = 0x4284 ; addr <= 0x4285 ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			for (addr = 0x2030 ; addr <= 0x2033 ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			for (addr = 0x2058 ; addr <= 0x205B ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			for (addr = 0x7072 ; addr <= 0x7075 ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			for (addr = 0x7200 ; addr <= 0x727F ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			for (addr = 0x7005 ; addr <= 0x7006 ; addr++) {
				testval = 0;
				sensor_read_reg_f(client, addr, &testval);
				printk("addr=0x%x, value=0x%x \n",addr,testval);
			}
			testval = 0;
			sensor_read_reg_f(client, 0x72f8, &testval);
			printk("addr=0x72f8, value=0x%x \n",testval);
			printk("-- Dump iCatch register Down --\n");
#endif
			return -ENOMEM;
		}
	}
	// ASUS_BSP +++ get raw data
	if (fmt->code == -EINVAL)
		fmt->code = SPCA700XA_BAYER_ORDER;
	if ((fmt->code == V4L2_MBUS_FMT_SBGGR8_1X8)
		|| (fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8)
		|| (fmt->code == V4L2_MBUS_FMT_SGRBG8_1X8)
		|| (fmt->code == V4L2_MBUS_FMT_SRGGB8_1X8)) {
		/*icatch RAW8 output*/
		printk("icatch RAW8 output\n");
		sensor_write_reg_f(client, 0x7160, 0x00);
		sensor_write_reg_f(client, 0x7161, 0x00);
	} else if ((fmt->code == V4L2_MBUS_FMT_SBGGR10_1X10)
		|| (fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10)
		|| (fmt->code == V4L2_MBUS_FMT_SGRBG10_1X10)
		|| (fmt->code == V4L2_MBUS_FMT_SRGGB10_1X10)) {
		/*icatch RAW10 output*/
		printk("icatch RAW10 output\n");
		sensor_write_reg_f(client, 0x7160, 0x00);
		sensor_write_reg_f(client, 0x7161, 0x01);
	} else if ((fmt->code == V4L2_MBUS_FMT_SBGGR12_1X12)
		|| (fmt->code == V4L2_MBUS_FMT_SGBRG12_1X12)
		|| (fmt->code == V4L2_MBUS_FMT_SGRBG12_1X12)
		|| (fmt->code == V4L2_MBUS_FMT_SRGGB12_1X12)) {
		/*icatch RAW12 output*/
		printk("icatch RAW12 output\n");
		sensor_write_reg_f(client, 0x7160, 0x00);
		sensor_write_reg_f(client, 0x7161, 0x02);
	}
	// ASUS_BSP --- get raw data
	dev->res = res_index->res;
	dev->format.code = fmt->code;

	fmt->width = width;
	fmt->height = height;

	if (mHDR) {
		old_f.w = width;
		old_f.h = height;
		old_f.res = res_index->res;
		old_f.code = fmt->code;
	}

	return 0;
}

static int spca700xa_16mpf_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (SPCA700XA_16MPF_FOCAL_LENGTH_NUM << 16) | SPCA700XA_16MPF_FOCAL_LENGTH_DEM;
	return 0;
}

static int spca700xa_16mpf_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/* const f number for SPCA700XA_16MPF */
	*val = (SPCA700XA_16MPF_F_NUMBER_DEFAULT_NUM << 16) | SPCA700XA_16MPF_F_NUMBER_DEM;
	return 0;
}

static int spca700xa_16mpf_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (SPCA700XA_16MPF_F_NUMBER_DEFAULT_NUM << 24) |
		(SPCA700XA_16MPF_F_NUMBER_DEM << 16) |
		(SPCA700XA_16MPF_F_NUMBER_DEFAULT_NUM << 8) | SPCA700XA_16MPF_F_NUMBER_DEM;
	return 0;
}

/*
 * This returns EV.
 */
static int spca700xa_16mpf_get_exposure_bias(struct v4l2_subdev *sd, s32 *value)
{
	*value = 0;

	return 0;
}

/*
 * This returns ISO sensitivity.
 */
static int sensor_g_iso(struct v4l2_subdev *sd, s32 *value)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        int iso_value = 0;
        u16 retvalue_h, retvalue_l;
        int ret = 0;

        switch (iso_setting)
        {
                case 0:
                        ret = sensor_read_reg_f(client, 0x72B7, &retvalue_l); //iso[7:0]
                        ret = sensor_read_reg_f(client, 0x72B8, &retvalue_h); //iso[15:8]
                        iso_value = (retvalue_h<<8)|(retvalue_l);
                        break;
                case 1:
                        iso_value = 50;
                        break;
                case 2:
                        iso_value = 100;
                        break;
                case 3:
                        iso_value = 200;
                        break;
                case 4:
                        iso_value = 400;
                        break;
                case 5:
                        iso_value = 800;
                        break;
                case 6:
                        iso_value = 1600;
                        break;
                default:
                        iso_value = 100;
                        break;
        }
        pr_debug("%s iso exif %d\n", __func__, iso_value);

        *value = iso_value;

        return 0;
}

//TODO: add camera parameter later
static struct spca700xa_16mpf_control spca700xa_16mpf_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_AUTO_EXPOSURE_BIAS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure bias",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = spca700xa_16mpf_get_exposure_bias, //For EXIF value, need to combine with other parameter
	},
	{
		.qc = {
			.id = V4L2_CID_ISO_SENSITIVITY,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "iso",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = sensor_g_iso, //For EXIF value, need to combine with other parameter
		.tweak = sensor_s_iso_f,
	},
	{
		.qc = {
			.id = V4L2_CID_COLORFX,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "color effect",
			.minimum = 0,
			.maximum = 20,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_color_effect_f,
	},
	{
		.qc = {
			.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "white balance",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_white_balance_f,
	},
	{
		.qc = {
			.id = V4L2_CID_SCENE_MODE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "scene mode",
			.minimum = 0,
			.maximum = 13,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_scene_mode_f,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = -6,
			.maximum = 6,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_ev_f,
		.query = sensor_g_exposure_f,
	},
	{
		.qc = {
			.id = V4L2_CID_POWER_LINE_FREQUENCY,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "Light frequency filter",
			.minimum = 1,
			.maximum = 3,
			.step = 1,
			.default_value = 1,
		},
		.tweak = sensor_s_flicker_f,
	},
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_METERING,
			.type = V4L2_CTRL_TYPE_MENU,
			.name = "metering",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 1,
		},
		.tweak = sensor_s_ae_metering_mode_f,
	},
        {
                .qc = {
                        .id = V4L2_CID_EFFECT_AURA,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "aura",
                        .minimum = 0,
                        .maximum = 63,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_effect_aura_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_MOVING,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "moving",
                        .minimum = 0,
                        .maximum = 255,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_moving_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_WDR,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "wdr",
                        .minimum = 0,
                        .maximum = 1,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_wdr_f,
        },
	{
		.qc = {
			.id = V4L2_CID_HDR,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "hdr mode",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = sensor_s_hdrf,
	},
	{
                .qc = {
                        .id = V4L2_CID_EIS, //ASUS_BSP, Camera5.0
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "eis",
                        .minimum = 0,
                        .maximum = 1,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_eis_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_GSENSOR,
                        .type = V4L2_CTRL_TYPE_STRING,
                        .name = "Gsensor Data",
                        .minimum = 0,
                        .maximum = 255,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_gsensor_data_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_3A_LOCK_ISP,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "3A Lock",
                        .minimum = 0,
                        .maximum = 11,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_3a_lock_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_EXPOSURE_AREAS, //ASUS_BSP, Camera5.0
                        .type = V4L2_CTRL_TYPE_STRING,
                        .name = "Exposure Window",
                        .minimum = 0,
                        .maximum = 255,
                        .step = 1,
                        .default_value = 0,
                },
                .tweak = sensor_s_exposure_window_f,
        },
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = SPCA700XA_16MPF_FOCAL_LENGTH_DEFAULT,
			.maximum = SPCA700XA_16MPF_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = SPCA700XA_16MPF_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = spca700xa_16mpf_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = SPCA700XA_16MPF_F_NUMBER_DEFAULT,
			.maximum = SPCA700XA_16MPF_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = SPCA700XA_16MPF_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = spca700xa_16mpf_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = SPCA700XA_16MPF_F_NUMBER_RANGE,
			.maximum =  SPCA700XA_16MPF_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = SPCA700XA_16MPF_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = spca700xa_16mpf_g_fnumber_range,
	},
//ASUS_BSP++, add for calibration
	{
		.qc = {
			.id = CUSTOM_IOCTL_REG_SET,
		},
		.tweak = sensor_s_write_reg_f,
	},
	{
		.qc = {
			.id = CUSTOM_IOCTL_REG_GET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "REG get",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_s_read_reg_f,
	},
	{
		.qc = {
			.id = CUSTOM_IOCTL_SPI_GET,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "SPI get",
			.minimum = 0,
			.maximum = 9,
			.step = 1,
			.default_value = 0,
		},
		.query = sensor_s_read_spi_f,
	},
//ASUS_BSP--, add for calibration

       {
                .qc = {
                        .id = V4L2_CID_EDGE,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "Edge",
                        .minimum = 0,
                        .maximum = 255,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_edge_f,
        },

//ASUS_BSP+++, for iCatch 3A information
        {
                .qc = {
                        .id = V4L2_CID_PAN_RELATIVE,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AE1",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_ae1_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_TILT_RELATIVE,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AE2",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_ae2_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_PAN_RESET,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AWB1",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_awb1_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_TILT_RESET,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AWB2",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_awb2_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_PAN_ABSOLUTE,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AF1",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_af1_f,
        },
        {
                .qc = {
                        .id = V4L2_CID_TILT_ABSOLUTE,
                        .type = V4L2_CTRL_TYPE_INTEGER,
                        .name = "AF2",
                        .minimum = 0,
                        .maximum = 6,
                        .step = 1,
                        .default_value = 0,
                },
                .query = sensor_g_3a_info_af2_f,
        },
		{
				.qc = {
						.id = INTEL_V4L2_CID_VBLANKING,
						.type = V4L2_CTRL_TYPE_INTEGER,
						.name = "VBLANKING",
						.minimum = 0,
						.maximum = 5000,
						.step = 1,
						.default_value = 2000,
				},
				.query = sensor_g_vblanking_f,
		},
//ASUS_BSP---, for iCatch 3A information
};

#define N_CONTROLS (ARRAY_SIZE(spca700xa_16mpf_controls))

static struct spca700xa_16mpf_control *spca700xa_16mpf_find_control(__u32 id)
{
	int i;
	pr_debug("%s id %d\n", __func__, id);
	for (i = 0; i < N_CONTROLS; i++) {
		if (spca700xa_16mpf_controls[i].qc.id == id)
			return &spca700xa_16mpf_controls[i];
	}
	return NULL;
}

//ASUS_BSP++ Wesley, Add for ISP firmware version in setting
static ssize_t isp_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%x\n", version_num_in_bin);
}
//ASUS_BSP-- Wesley, Add for ISP firmware version in setting

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
static int detect_front_hm2051 (struct spca700xa_16mpf_device *dev,  struct i2c_client *client) {
	int ret = 0;
	u16 sensorid, retvalue_h, retvalue_l;

        // detect hm2051
	sensor_write_reg_f(client, 0x1011, 0x01);  /* CPU reset */

	/*Start - Power on sensor & enable clock*/
	sensor_write_reg_f(client, 0x0084, 0x14);  /* To sensor clock divider */
	sensor_write_reg_f(client, 0x0034, 0xFF);  /* Turn on all clock */
	sensor_write_reg_f(client, 0x9030, 0x7f);
	sensor_write_reg_f(client, 0x9031, 0x04);
	sensor_write_reg_f(client, 0x9034, 0xf2);
	sensor_write_reg_f(client, 0x9035, 0x07);
	sensor_write_reg_f(client, 0x9032, 0x00);
	sensor_write_reg_f(client, 0x9033, 0x00);
	usleep_range(10000, 11000); /*10ms*/
	sensor_write_reg_f(client, 0x9032, 0x1E);
	sensor_write_reg_f(client, 0x9033, 0x04);
	usleep_range(10000, 11000); /*10ms*/
	/*End - Power on sensor & enable clock */

	sensor_write_reg_f(client, 0x9008, 0x00);  /* Need to check with vincent */
	sensor_write_reg_f(client, 0x9009, 0x00);
	sensor_write_reg_f(client, 0x900A, 0x00);
	sensor_write_reg_f(client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg_f(client, 0x9138, 0x30);  /* Sub address enable */
	sensor_write_reg_f(client, 0x9140, 0x48);  /* Slave address      */
	sensor_write_reg_f(client, 0x9100, 0x03);  /* Read mode          */
	sensor_write_reg_f(client, 0x9110, 0x00);  /* Register addr MSB*/
	sensor_write_reg_f(client, 0x9112, 0x01);  /* Register addr LSB*/
	sensor_write_reg_f(client, 0x9104, 0x01);  /* Trigger I2C read   */
	usleep_range(2000, 2100);/*2ms*/
	sensor_read_reg_f(client, 0x9111, &retvalue_h);

	sensor_write_reg_f(client, 0x9110, 0x00);  /* Register addr MSB*/
	sensor_write_reg_f(client, 0x9112, 0x02);  /* Register addr LSB*/
	sensor_write_reg_f(client, 0x9104, 0x01);  /* Trigger I2C read   */
	usleep_range(2000, 2100);/*2ms*/
	sensor_read_reg_f(client, 0x9111, &retvalue_l);

        sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
	printk("detect_front_hm2051: read sensor id: 0x%04X",  sensorid);
        if (sensorid ==0x2051)
              pltfrm_camera_module_pr_info(&(dev->sd),
                      "successfully detected camera ID 0x%x\n",
                      sensorid);
        else {
              pltfrm_camera_module_pr_err(&(dev->sd),
                      "wrong camera ID, expected 0x%x, detected 0x%x\n",
                      0x2051, sensorid);
              ret = -EINVAL;
        }
	return ret;
}
#else
static int detect_front_gc0310 (struct spca700xa_16mpf_device *dev,  struct i2c_client *client) {
	int ret = 0;
	u16 sensorid, retvalue_h, retvalue_l;

        // detect gc0310
        sensor_write_reg_f(client, 0x1011, 0x01);

        sensor_write_reg_f(client, 0x0084, 0x14);
        sensor_write_reg_f(client, 0x0034, 0xFF);
        sensor_write_reg_f(client, 0x9030, 0x7F);
        sensor_write_reg_f(client, 0x9031, 0x04);
        sensor_write_reg_f(client, 0x9034, 0xF2);
        sensor_write_reg_f(client, 0x9035, 0x07);
        sensor_write_reg_f(client, 0x9032, 0x02);
        sensor_write_reg_f(client, 0x9033, 0x04);
        usleep_range(10000, 11000);
        sensor_write_reg_f(client, 0x9032, 0x1C);
        sensor_write_reg_f(client, 0x9033, 0x00);
        usleep_range(10000, 11000);

        sensor_write_reg_f(client, 0x9008, 0x00);
        sensor_write_reg_f(client, 0x9009, 0x00);
        sensor_write_reg_f(client, 0x900A, 0x00);
        sensor_write_reg_f(client, 0x900B, 0x00);

        sensor_write_reg_f(client, 0x9138, 0x10);
        sensor_write_reg_f(client, 0x9140, 0x42);
        sensor_write_reg_f(client, 0x9100, 0x03);
        sensor_write_reg_f(client, 0x9110, 0xF0);
        sensor_write_reg_f(client, 0x9104, 0x01);
        usleep_range(2000, 2100);
        sensor_read_reg_f(client, 0x9111, &retvalue_h);

        sensor_write_reg_f(client, 0x9110, 0xF1);
        sensor_write_reg_f(client, 0x9104, 0x01);
        usleep_range(2000, 2100);
        sensor_read_reg_f(client, 0x9111, &retvalue_l);

        sensorid = ((retvalue_h<<8)|(retvalue_l))&0xffff;
	printk("detect_front_gc0310: read sensor id: 0x%04X",  sensorid);
        if (sensorid == 0xa310)
              pltfrm_camera_module_pr_info(&(dev->sd),
                      "successfully detected camera ID 0x%x\n",
                      sensorid);
        else {
              pltfrm_camera_module_pr_err(&(dev->sd),
                      "wrong camera ID, expected 0x%x, detected 0x%x\n",
                      0xa310, sensorid);
              ret = -EINVAL;
        }
	return ret;
}
#endif
static int spca700xa_16mpf_detect(struct spca700xa_16mpf_device *dev, struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	int ret = 0;
        printk("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	msleep(250); //need wait more than 100ms after power on to get sensor ID

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	ret = detect_front_hm2051(dev, client);
	if(ret) {
		goto err;
	}
	//++ ASUS_BSP, new ATD request for read sensor name
	sensor_name = "hm2051_iCatch";
	//--
#else
	ret = detect_front_gc0310(dev, client);
	if(ret) {
		goto err;
	}
	//++ ASUS_BSP, new ATD request for read sensor name
	sensor_name = "gc0310_iCatch";
	//--
#endif

	//++ ASUS_BSP, new ATD request for read sensor name
	create_camera_proc_file();
	//-- ASUS_BSP

	msleep(100);
	//any failed status wil not goto here.
	ATD_spca700xa_16mp_status = 1;
	spca700xa_16mpf_dbgfs_init();

	#if 0
	if(snesorid==0x2481) {
		ATD_spca700xa_16mp_status = 1;
        	//Wesley, get bin file version
	        get_fw_version_in_bin_f();
	} else {
		ATD_spca700xa_16mp_status = 0;
		printk("%s Sensor ID is not match\n", __func__);
	}
	#endif
		
	//For icatch test +++
{
	u16 	retvalue_l = 0;
	sensor_write_reg_f(client, 0x1300, 0x5a);
	dumpReg(client, 0x1300);
	dumpReg(client,  0x002c);
	ret = sensor_read_reg_f(client, 0x1300, &retvalue_l);
        printk("read 0x1300 %d\n",retvalue_l&0xffff);
	retvalue_l = 0;
	sensor_read_reg_f(client, 0x002c, &retvalue_l);
	printk("read 0x002c %d\n",retvalue_l&0xffff);
	//For icatch test ---
}
	return 0;
err:
	pltfrm_camera_module_pr_err(&(dev->sd), "failed with error (%d)\n", ret);
	return ret;
}

#if 1
static int
spca700xa_16mpf_s_config(struct v4l2_subdev *sd)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int byte_count = 0;
	UINT32 size_CalibOpt=0, size_3acali=0, size_Lsc=0, size_Lscdq=0;
	int ret;

        printk("%s\n", __func__);

	fp = filp_open(CALIBOPT_FILE_WITH_PATH, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		printk("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			printk("Start to read %s\n", CALIBOPT_FILE_WITH_PATH);
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("\"%s\" not found error\n", CALIBOPT_FILE_WITH_PATH);
		CALIBOPT_FILE_WITH_PATH = DEFAULT_CALIBOPT_FILE_WITH_PATH;
	} else{
		pr_err("\"%s\" open error\n", CALIBOPT_FILE_WITH_PATH);
		CALIBOPT_FILE_WITH_PATH = DEFAULT_CALIBOPT_FILE_WITH_PATH;
	}
	fp = filp_open(CALIBOPT_FILE_WITH_PATH, O_RDONLY, 0);
	if ( !IS_ERR_OR_NULL(fp) ){
		printk("filp_open calibration_option.BIN success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		size_CalibOpt = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_CalibOpt);
		if (size_CalibOpt > 0) {
			pCalibOpt_g_f = kmalloc(size_CalibOpt, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				byte_count= 0;
				printk("Start to read %s\n", CALIBOPT_FILE_WITH_PATH);
				byte_count = fp->f_op->read(fp, pCalibOpt_g_f, size_CalibOpt, &fp->f_pos);
				printk("iCatch: BIN file size= %d bytes\n", size_CalibOpt);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		pr_err("iCatch \"%s\" open error\n", CALIBOPT_FILE_WITH_PATH);
	}
	fp = filp_open(IIIACALI_FILE_WITH_PATH, O_RDONLY, 0);
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open 3ACALI_F.BIN success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		size_3acali = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_3acali);
		if (size_3acali > 0) {
			p3acali_g_f = kmalloc(size_3acali, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				byte_count= 0;
				printk("Start to read %s\n", IIIACALI_FILE_WITH_PATH);
				byte_count = fp->f_op->read(fp, p3acali_g_f, size_3acali, &fp->f_pos);
				printk("iCatch: BIN file size= %d bytes\n", size_3acali);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		pr_err("iCatch \"%s\" open error\n", IIIACALI_FILE_WITH_PATH);
	}
	fp = filp_open(LSC_FILE_WITH_PATH, O_RDONLY, 0);
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open LSC_F.BIN success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		size_Lsc = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_Lsc);
		if (size_Lsc > 0) {
			pLsc_g_f = kmalloc(size_Lsc, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				byte_count= 0;
				printk("Start to read %s\n", LSC_FILE_WITH_PATH);
				byte_count = fp->f_op->read(fp, pLsc_g_f, size_Lsc, &fp->f_pos);
				printk("iCatch: BIN file size= %d bytes\n", size_Lsc);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		pr_err("iCatch \"%s\" open error\n", LSC_FILE_WITH_PATH);
	}
	fp = filp_open(LSCDQ_FILE_WITH_PATH, O_RDONLY, 0);
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open LSC_DQ_F.BIN success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		size_Lscdq = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, size_Lscdq);
		if (size_Lscdq > 0) {
			pLscdq_g_f = kmalloc(size_Lscdq, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				byte_count= 0;
				printk("Start to read %s\n", LSCDQ_FILE_WITH_PATH);
				byte_count = fp->f_op->read(fp, pLscdq_g_f, size_Lscdq, &fp->f_pos);
				printk("iCatch: BIN file size= %d bytes\n", size_Lscdq);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		pr_err("iCatch \"%s\" open error\n", LSCDQ_FILE_WITH_PATH);
	}

	return 0;
}
#endif

static int spca700xa_16mpf_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct spca700xa_16mpf_control *ctrl = spca700xa_16mpf_find_control(qc->id);
        pr_debug("%s \n", __func__);
	if (ctrl == NULL)
		return -EINVAL;
	*qc = ctrl->qc;
	return 0;
}

static int spca700xa_16mpf_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct spca700xa_16mpf_control *octrl = spca700xa_16mpf_find_control(ctrl->id);
	int ret;
        pr_debug("%s\n", __func__);

	if (octrl == NULL || !octrl->query)
		return -EINVAL;

	ret = octrl->query(sd, &ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int spca700xa_16mpf_s_ctrl_old(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct spca700xa_16mpf_control *octrl = spca700xa_16mpf_find_control(ctrl->id);
	int ret;
        pr_debug("%s\n", __func__);
	if (!octrl || !octrl->tweak)
		return -EINVAL;

	ret = octrl->tweak(sd, ctrl->value);
	if (ret < 0)
		return ret;

	return 0;
}

static int spca700xa_16mpf_s_ext_ctrls(struct v4l2_subdev *sd,struct v4l2_ext_controls *ctrls)
{
    int i;
    int ret = 0;
    pr_debug("%s\n", __func__);
    if (ctrls->count == 0)
        return -EINVAL;

    for (i = 0; i < ctrls->count; i++) {
        struct v4l2_ext_control *ctrl;
        u32 ctrl_updt = 0;

        ctrl = &ctrls->controls[i];
        switch (ctrl->id) {
            case V4L2_CID_GSENSOR:
                ret = sensor_s_gsensor_data_f(sd, ctrl->string);
                break;
	    case V4L2_CID_EXPOSURE_AREAS:
		sensor_s_exposure_window_f(sd, ctrl->string);
		break;
	    //case V4L2_CID_IRIS_RELATIVE:
		//sensor_s_roi_trigger(sd, ctrl->value);
		//break;

            default:
                ret = -EINVAL;
                break;
        }
    }
    return ret;
}

static int spca700xa_16mpf_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	//struct spca700xa_16mp_device *dev = to_spca700xa_16mp_sensor(sd);
        u32 i = 0;
        u16 testval = 0x00, retvalue = 0, value = 0, addr = 0;

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	if (mHDR && skip_cnt >= 2) {
		printk("%s skip stream %d skip_cnt %d\n", __func__, enable, skip_cnt);
		return 0;
	} else
		skip_cnt++;
#endif

	pltfrm_camera_module_pr_info(sd,"%s enable = %d\n", __func__, enable);

	if (enable) {
#if 0
		sensor_write_reg_f(client, 0x7106, 0x00);
		sensor_write_reg_f(client, 0x7120, 0x00);
		sensor_write_reg_f(client, 0x7121, 0x01);
#endif
		ret = sensor_read_reg_f(client, 0x90CC, &value); //low byte of front sensor ID
		printk("%s value 0x90CC %x\n", __func__, value);
		ret = sensor_read_reg_f(client, 0x90CD, &value); //low byte of front sensor ID
		printk("%s value 0x90CD %x\n", __func__, value);
		ret = sensor_read_reg_f(client, 0x90CE, &value); //low byte of front sensor ID
		printk("%s value 0x90CE %x\n", __func__, value);
		ret = sensor_read_reg_f(client, 0x90CF, &value); //low byte of front sensor ID
		printk("%s value 0x90CF %x\n", __func__, value);

		if (mHDR) {
			pr_info("HDR: Pass AE ready\n");
		} else {
			//wait AE ready
			for (i = 0; i < 20; i++) {
				sensor_read_reg_f(client, 0x72c3, &testval);
				printk("%s testval=0x%X, i=%d ", __func__, testval,i);
				if (testval & 0x01) {
					printk("%s AE ready\n", __func__);
					break;
				}
				usleep_range(5000, 5100);
			}
			if (i == 20)
				printk("Wait AE timeout\n");
		}
#if SKIP_FIRST_COMMAND
		if(first_on==1){
#endif
			sensor_write_reg_f(client, 0x7121, 0x01); //Streaming on
			pr_debug("%s Set stream on command\n", __func__);

			if (mNightMode) {
				sensor_write_reg_f(client, 0x711b, 0x64);
				printk("%s set 0x711B = 0x64 for night mode\n", __func__);
			} else {
				//ISP Function Control
				//bit0:WDR, bit1:Edge information, bit2:Pre-Calculate Capture ISO
				//bit3:Sensor Binning Sum function, bit4:Y average information
				//bit5:Scene information, bit6:Auto night mode function
				//Enable: Edge information, Pre-Calculate Capture ISO, Y average
				sensor_read_reg_f(client, 0x729B, &retvalue);
				value = (retvalue & 0x41) | 0x16;
				pr_debug("%s Set 0x711B = 0x%x\n", __func__, value);
				sensor_write_reg_f(client, 0x711B, value);
			}
#if SKIP_FIRST_COMMAND
		} else	{
			first_on = 1;
			printk("%s Pass stream on command\n", __func__);
		}
#endif
	} else {
		sensor_write_reg_f(client, 0x7121, 0x00); //Streaming off
		//dev->last_run_mode = dev->run_mode;
	}

	// iCatch Dump - start
#if 0
	printk("-- Dump iCatch register now --\n");
	testval = 0;
	sensor_read_reg_f(client, 0x002C, &testval);
	printk("addr=0x002C, value=0x%x \n",testval);
	sensor_read_reg_f(client, 0x0082, &testval);
	printk("addr=0x0082, value=0x%x \n",testval);
	sensor_read_reg_f(client, 0x008C, &testval);
	printk("addr=0x008C, value=0x%x \n",testval);
	sensor_read_reg_f(client, 0x90D6, &testval);
	printk("addr=0x90D6, value=0x%x \n",testval);
	sensor_read_reg_f(client, 0x2108, &testval);
	printk("addr=0x2108, value=0x%x \n",testval);
	sensor_read_reg_f(client, 0x7049, &testval);
	printk("addr=0x7049, value=0x%x \n",testval);

	for (addr=0x4284;addr<=0x4285;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x2030;addr<=0x2033;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x2058;addr<=0x205B;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x7040;addr<=0x7046;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x7070;addr<=0x7075;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x7200;addr<=0x727F;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	for (addr=0x7005;addr<=0x7006;addr++) {
			testval = 0;
			sensor_read_reg_f(client, addr, &testval);
			printk("addr=0x%x, value=0x%x \n",addr,testval);
	}
	testval = 0;
	sensor_read_reg_f(client, 0x72f8, &testval);
	printk("addr=0x72f8, value=0x%x \n",testval);
	printk("-- Dump iCatch register Down --\n");
#endif
	// iCatch Dump - end

	return 0;
}

static int
spca700xa_16mpf_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = spca700xa_16mpf_res[index].width;
	fsize->discrete.height = spca700xa_16mpf_res[index].height;

	return 0;
}

static int spca700xa_16mpf_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	//int i;

	if (index >= N_RES)
		return -EINVAL;
#if 0
	/* find out the first equal or bigger size */
	for (i = 0; i < N_RES; i++) {
		printk("res[%d]=%dx%d >= %dx%d\n", index, spca700xa_16mpf_res[i].width, spca700xa_16mpf_res[i].height, fival->width, fival->height);
		if ((spca700xa_16mpf_res[i].width >= fival->width) &&
		    (spca700xa_16mpf_res[i].height >= fival->height))
			break;
	}
	if (i == N_RES)
		i--;

	index = i;
#endif

	fival->pixel_format = SPCA700XA_BAYER_ORDER;
	fival->width = spca700xa_16mpf_res[index].width;
	fival->height = spca700xa_16mpf_res[index].height;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = spca700xa_16mpf_res[index].fps;

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	if (mHDRMode || mNightMode) {
		if (fival->width >= SPCA700XA_16MPF_RES_VGA_SIZE_H ||
			fival->height >= SPCA700XA_16MPF_RES_VGA_SIZE_V) {
			fival->width = SPCA700XA_16MPF_RES_2M_SIZE_H;
			fival->height = SPCA700XA_16MPF_RES_2M_SIZE_V;
			fival->discrete.denominator = 30;
		}
	}
#endif
	return 0;
}

static int spca700xa_16mpf_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	if (code->index >= MAX_FMTS)
		return -EINVAL;
	code->code = dev->format.code;

	return 0;
}

static int spca700xa_16mpf_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{

	unsigned int index = fse->index;


	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = spca700xa_16mpf_res[index].width;
	fse->min_height = spca700xa_16mpf_res[index].height;
	fse->max_width = spca700xa_16mpf_res[index].width;
	fse->max_height = spca700xa_16mpf_res[index].height;

	return 0;
}

static struct v4l2_mbus_framefmt *
__spca700xa_16mpf_get_pad_format(struct spca700xa_16mpf_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

	switch (which) {
#if defined(CONFIG_VIDEO_V4L2_SUBDEV_API)
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
#endif
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
spca700xa_16mpf_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct spca700xa_16mpf_device *snr = to_spca700xa_16mpf_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__spca700xa_16mpf_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
spca700xa_16mpf_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct spca700xa_16mpf_device *snr = to_spca700xa_16mpf_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__spca700xa_16mpf_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int spca700xa_16mpf_s_ctrl(struct v4l2_ctrl *ctrl)
{
	//struct spca700xa_16mpf_device *snr = container_of(
	//	ctrl->handler, struct spca700xa_16mpf_device, ctrl_handler);
        pr_debug("%s\n", __func__);
	//snr->run_mode->val = ctrl->val;

	return 0;
}

static int spca700xa_16mpf_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	int index;
	struct spca700xa_16mpf_device *snr = to_spca700xa_16mpf_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	for (index = 0; index < N_RES; index++) {
		if (spca700xa_16mpf_res[index].res == snr->res)
			break;
	}

	if (index >= N_RES)
		return -EINVAL;

	*frames = spca700xa_16mpf_res[index].skip_frames;

	return 0;
}
#if 0
static int spca700xa_16mpf_s_modes(struct v4l2_subdev *sd, u32 modes)
{
       //struct spca700xa_16mpf_device *snr = to_spca700xa_16mpf_sensor(sd);
       switch (modes) {
           case ATOMISP_RUN_MODE_VIDEO:
               pr_debug("%s mode = MODE_VIDEO\n", __func__);
               break;
           case ATOMISP_RUN_MODE_STILL_CAPTURE:
               pr_debug("%s mode = MODE_STILL_CAPTURE\n", __func__);
               break;
           case ATOMISP_RUN_MODE_CONTINUOUS_CAPTURE:
               pr_debug("%s mode = MODE_CONTINUOUS_CAPTURE\n", __func__);
               break;
           case ATOMISP_RUN_MODE_PREVIEW:
               pr_debug("%s mode = MODE_PREVIEW\n", __func__);
               break;
           default:
               return -EINVAL;
        }
        sensor_mode = modes;

        return 0;
}
#endif

static int spca700xa_16mpf_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
	if (index >= MAX_FMTS)
		return -EINVAL;

	//mutex_lock(&dev->input_lock);
	*code = dev->format.code;
	//mutex_unlock(&dev->input_lock);
	return 0;
}

static int
 spca700xa_16mpf_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
//	struct spca700xa_16mpf_device *dev = to_spca700xa_16mpf_sensor(sd);
//	dev->run_mode = param->parm.capture.capturemode;

	return 0;
}

int
spca700xa_16mpf_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	interval->interval.denominator = 30;
	interval->interval.numerator = 1;

	return 0;
}

static int spca700xa_16mpf_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	return 0;
}

static const struct v4l2_subdev_video_ops spca700xa_16mpf_video_ops = {
	.s_stream = spca700xa_16mpf_s_stream,
	.enum_framesizes = spca700xa_16mpf_enum_framesizes,
	.enum_frameintervals = spca700xa_16mpf_enum_frameintervals,
	.try_mbus_fmt = spca700xa_16mpf_try_mbus_fmt,
	.s_mbus_fmt = spca700xa_16mpf_set_mbus_fmt,
	.g_mbus_fmt = spca700xa_16mpf_get_mbus_fmt,

	.enum_mbus_fmt = spca700xa_16mpf_enum_mbus_fmt,
	.s_parm = spca700xa_16mpf_s_parm,
	.g_frame_interval = spca700xa_16mpf_g_frame_interval,
	.s_frame_interval =spca700xa_16mpf_s_frame_interval,
};

static struct v4l2_subdev_sensor_ops spca700xa_16mpf_sensor_ops = {
        .g_skip_frames = spca700xa_16mpf_g_skip_frames,
//        .s_modes = spca700xa_16mpf_s_modes,
};

static const struct v4l2_subdev_core_ops spca700xa_16mpf_core_ops = {
	.queryctrl = spca700xa_16mpf_queryctrl,
	.g_ctrl = spca700xa_16mpf_g_ctrl,
	.s_ctrl = spca700xa_16mpf_s_ctrl_old,
	.s_ext_ctrls = spca700xa_16mpf_s_ext_ctrls,
	.s_power = spca700xa_16mpf_s_power,
};

static const struct v4l2_subdev_pad_ops spca700xa_16mpf_pad_ops = {
	.enum_mbus_code = spca700xa_16mpf_enum_mbus_code,
	.enum_frame_size = spca700xa_16mpf_enum_frame_size,
	.get_fmt = spca700xa_16mpf_get_pad_format,
	.set_fmt = spca700xa_16mpf_set_pad_format,
};

static const struct v4l2_subdev_ops spca700xa_16mpf_ops = {
	.core = &spca700xa_16mpf_core_ops,
	.video = &spca700xa_16mpf_video_ops,
	.pad = &spca700xa_16mpf_pad_ops,
	.sensor = &spca700xa_16mpf_sensor_ops,
};

static const struct media_entity_operations spca700xa_16mpf_entity_ops;


static int spca700xa_16mpf_remove(struct i2c_client *client)
{
	struct spca700xa_16mpf_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct spca700xa_16mpf_device, sd);
#if 0
	dev->platform_data->csi_cfg(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
#endif

	pltfrm_camera_module_release(sd);

	v4l2_device_unregister_subdev(sd);
	// media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

static const struct v4l2_ctrl_ops ctrl_ops = {
	.s_ctrl = spca700xa_16mpf_s_ctrl,
};

static const char * const ctrl_run_mode_menu[] = {
	NULL,
	"Video",
	"Still capture",
	"Continuous capture",
	"Preview",
};

static const struct v4l2_ctrl_config ctrl_run_mode = {
	.ops = &ctrl_ops,
	.id = V4L2_CID_RUN_MODE,
	.name = "run mode",
	.type = V4L2_CTRL_TYPE_MENU,
	.min = 1,
	.def = 4,
	.max = 4,
	.qmenu = ctrl_run_mode_menu,
};

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
#define  GPIO_MAIN_CAM 56
static int get_gpio(unsigned int pin)
{
	int ret;

	if(gpio_request(pin, "PCB_ID"))
	{
		pr_info("Fail to get pcb id pin %d \n", pin);
		return 0;
	}
	gpio_direction_input(pin);
	ret = gpio_get_value_cansleep(pin);
	printk("SOC GPIO %d := %x\n", pin, ret);
	gpio_free(pin);

	return ret;
}

static int getCamID(void)
{
	int ret = -1;
	int main_cam_id = get_gpio(GPIO_MAIN_CAM);
	switch (main_cam_id) {
		case 1: 
			return MAIN_CAMERA_ID_OV;
			break;
		case 0:
		default:
			return MAIN_CAMERA_ID_TK;
			break;
	}
	return ret;
}
#endif

extern unsigned int entry_mode;
#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
extern int sub_camera_id_370;
#else
extern int camera_id_170;
#endif

static int spca700xa_16mpf_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct spca700xa_16mpf_device *dev;
	int ret = 0;

        if (entry_mode != 1) {
                dev_info(&client->dev, "return because entry_mode!=1\n");
                return ret;
        }

#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
        if(sub_camera_id_370 != 1){
                dev_info(&client->dev, "return because without iCatch\n");
                return ret;
        }
#else
        if(camera_id_170){
                dev_info(&client->dev, "return because without iCatch\n");
                return ret;
        }
#endif
	printk("%s\n", __func__);
	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "%s out of memory\n", __func__);
		return -ENOMEM;
	}

	//Move and modify from ov5693.h for firmware --- Wesley +++
	//FW_BSP++
        create_i7002a_F_proc_front_file(); //ASUS_BSP +++, add for ISP firmware update
	//FW_BSP--
	//Move and modify from ov5693.h for firmware --- Wesley ---

	//Add for ATD read camera status+++
	dev->sensor_i2c_attribute.attrs = spca700xa_16mpf_attributes;

	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	v4l2_i2c_subdev_init(&dev->sd, client, &spca700xa_16mpf_ops);
#if 0
	if (client->dev.platform_data) {
		ret = spca700xa_16mpf_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}
#endif
#if defined(CONFIG_Z370CG) || defined(CONFIG_Z370C)
	FW_BIN_FILE_WITH_PATH = FW_BIN_FILE_WITH_PATH_TK;
	camera_id = getCamID();
	if(camera_id == MAIN_CAMERA_ID_OV) {
		FW_BIN_FILE_WITH_PATH = FW_BIN_FILE_WITH_PATH_OV;
	}
#endif
	ret = pltfrm_camera_module_init(&(dev->sd), (void **)&(dev->platform_data));
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		spca700xa_16mpf_remove(client);
		return ret;
	}

	pltfrm_camera_module_pr_info(&(dev->sd), "%s %d-%04x\n",
		SPCA700XA_16MPF_DRIVER_NAME,
		i2c_adapter_id(client->adapter), client->addr);

	printk("====WESLEY power up in _probe\n");
	ret = power_up(&(dev->sd));
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		return -EINVAL;
	}

	ret = spca700xa_16mpf_detect(dev, client);
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		spca700xa_16mpf_remove(client);
		//return ret; //skip this to call power_down
	}

	/* turn off sensor, after probed */
	ret |= power_down(&(dev->sd));
	if (ret) {
		pltfrm_camera_module_pr_err(&(dev->sd),
			"failed with error %d\n", ret);
		return -EINVAL;
	}

#if 0
	/*TODO add format code here*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = SPCA700XA_BAYER_ORDER;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = v4l2_ctrl_handler_init(&dev->ctrl_handler, 1);
	if (ret) {
		spca700xa_16mpf_remove(client);
		return ret;
	}
#if 0
	dev->run_mode = v4l2_ctrl_new_custom(&dev->ctrl_handler,
					     &ctrl_run_mode, NULL);

	if (dev->ctrl_handler.error) {
		spca700xa_16mpf_remove(client);
		return dev->ctrl_handler.error;
	}
	//dev->run_mode->val = CI_MODE_PREVIEW;
	//dev->last_run_mode->val = CI_MODE_PREVIEW;
#endif
    printk("davidwangs before media_entity_init\n");
	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		spca700xa_16mpf_remove(client);
		return ret;
	}
#endif

	/* set res index to be invalid */
	dev->res = -1;
	fw_sd_f = &dev->sd;

	dev_info(&client->dev, "probing successful\n");
	return 0;
}

static struct of_device_id spca700xa_16mpf_of_match[] = {
	{.compatible = "icatch," SPCA700XA_16MPF_DRIVER_NAME "-v4l2-i2c-subdev",},
	{},
};

MODULE_DEVICE_TABLE(i2c, spca700xa_16mpf_id);

static struct i2c_driver spca700xa_16mpf_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SPCA700XA_16MPF_DRIVER_NAME,
		.of_match_table = spca700xa_16mpf_of_match//Clark add
	},
	.probe = spca700xa_16mpf_probe,
	.remove = __exit_p(spca700xa_16mpf_remove),
	.id_table = spca700xa_16mpf_id,
};
#if 1 //Clark mark
static __init int init_spca700xa_16mpf(void)
{
	return i2c_add_driver(&spca700xa_16mpf_driver);
}

static __exit void exit_spca700xa_16mpf(void)
{
	i2c_del_driver(&spca700xa_16mpf_driver);
}
#endif
//module_i2c_driver(spca700xa_16mpf_driver);
module_init(init_spca700xa_16mpf);//Clark mark
module_exit(exit_spca700xa_16mpf);//Clark mark

MODULE_AUTHOR("Intel");
MODULE_LICENSE("GPL");
