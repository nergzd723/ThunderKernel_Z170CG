/*
 * ht24lc02_eeprom.c
 * HOLTEK CMOS 2K 2-Wire Serial EEPROM
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/gpio.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>

#define ht24lc02_VCELL_REG	0x02/*ht24lc02_VCELL_MSB*/

#define ht24lc02_SOC_REG		0x04/*ht24lc02_SOC_MSB*/

#define ht24lc02_MODE_REG		0x06/*ht24lc02_MODE_MSB*/

#define ht24lc02_VER_REG		0x08/*ht24lc02_VER_MSB*/

#define ht24lc02_RCOMP_REG	0x0C/*ht24lc02_RCOMP_MSB*/

#define ht24lc02_VRESET_REG                       0x18

#define ht24lc02_STATUS_REG                      0x1A
#define ht24lc02_CMD_REG	0xFE/*ht24lc02_CMD_MSB*/	

#define ht24lc02_MODEL_ACCESS_REG			0x3E
#define ht24lc02_OCV_REG		0x0E	//add by peter
#define ht24lc02_TABLE		0x40	//add by peter


#define ht24lc02_MODEL_ACCESS_UNLOCK			0x4A57
#define ht24lc02_MODEL_ACCESS_LOCK			0x0000
#define ht24lc02_POR_CMD		0x5400 //add by peter

#define ht24lc02_DELAY		10*HZ //1000->10*HZ
#define ht24lc02_eeprom_FULL	95
#define CONFIG_ht24lc02 //add by peter

//below are from .ini file
//--------------------.ini file---------------------------------
#define INI_RCOMP 		(127)
#define INI_RCOMP_FACTOR	1

#define INI_SOCCHECKA		(228)
#define INI_SOCCHECKB		(230)
#define INI_OCVTEST 		(57408) 
#define INI_BITS		(19)
//--------------------.ini file end-------------------------------

#define VERIFY_AND_FIX 1
#define LOAD_MODEL !(VERIFY_AND_FIX)

#define ISN_SIZE 32 // 00-1F
#define SSN_SIZE 20 // 20-33
#define MN_SIZE   5 // 34-38
#define RS_SIZE   4 // 39-3C
#define CT_SIZE   2 // 3D-3E
#define ACT_SIZE  1 // 3F
#define GA_SIZE 112 // 40-AF

#define ISN_ADDR  0
#define SSN_ADDR 32
#define MN_ADDR  52
#define RS_ADDR  57
#define CT_ADDR  61
#define ACT_ADDR 63
#define GA_ADDR  64

/*
static void prepare_to_load_model(struct i2c_client *client);
static void load_model(struct i2c_client *client);
static bool verify_model_is_correct(struct i2c_client *client);
static void cleanup_model_load(struct i2c_client *client);
*/

/* Static Read Status */
// state: 1 -> In a Reading Cycle, 0 -> Not Yet
// Count helps to relay the byte that is currently read. 
int ISN_state = 0, ISN_count = 0;
int SSN_state = 0, SSN_count = 0;
int MN_state = 0, MN_count = 0;
int RS_state = 0, RS_count = 0;
int CT_state = 0, CT_count = 0;
int ACT_state = 0, ACT_count = 0;
int GA_state = 0, GA_count = 0;

/* For byte request variables */
u8 request_addr = 0, request_value = 0;
int request_status = 0;
struct i2c_client *test_i2c_clt;

extern void cover_detect(void);
int ht24lc02_write(u8 reg, u8 value);
int ht24lc02_read(u8 reg);
static int ht24lc02_write_reg(struct i2c_client *client, u8 reg, u8 value);
static int ht24lc02_read_reg(struct i2c_client *client, u8 reg);
//static int ht24lc02_write_test(struct device *dev, struct device_attribute *attr, char *buf, size_t count);
//static int ht24lc02_read_test(struct device *dev, struct device_attribute *attr, char *buf);
//static int ht24lc02_erase_all(struct device *dev, struct device_attribute *attr, char *buf);
//static int ht24lc02_write_one_all(struct device *dev, struct device_attribute *attr, char *buf);
//static int ht24lc02_erase_1f(struct device *dev, struct device_attribute *attr, char *buf, size_t count);
//static int ht24lc02_write_one_1f(struct device *dev, struct device_attribute *attr, char *buf, size_t count);
static int ht24lc02_acinfo(struct device *dev, struct device_attribute *attr, char *buf, size_t count);
//static int ht24lc02_read_EEP(struct device *dev, struct device_attribute *attr, char *buf, u8 addr, int dsize);
static int ht24lc02_read_one(struct device *dev, struct device_attribute *attr, char *buf, u8 addr, int dsize, int *state, int *count);
static int ht24lc02_ISN(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_SSN(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_MN(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_RS(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_CT(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_ACT(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_GA(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_reset_reading(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_single_byte_request(struct device *dev, struct device_attribute *attr, char *buf, size_t count);
static int ht24lc02_single_byte_return(struct device *dev, struct device_attribute *attr, char *buf);
static void ht24lc02_reset_reading_in(void);
static int ht24lc02_gpio_pull_high(struct device *dev, struct device_attribute *attr, char *buf);
static int ht24lc02_CoverStatus(struct device *dev, struct device_attribute *attr, char *buf);

/* DEVICE_ATTR Settings */
//static DEVICE_ATTR(eep_wr_test, S_IRUGO | S_IWUSR, ht24lc02_read_test, ht24lc02_write_test );
//static DEVICE_ATTR(erase_1f, S_IRUGO | S_IWUSR, ht24lc02_erase_all, ht24lc02_erase_1f);
//static DEVICE_ATTR(one_1f, S_IRUGO | S_IWUSR, ht24lc02_write_one_all, ht24lc02_write_one_1f);
static DEVICE_ATTR(acinfo, 0666, NULL, ht24lc02_acinfo );
static DEVICE_ATTR(ISN, 0666, ht24lc02_ISN, NULL );
static DEVICE_ATTR(SSN, 0666, ht24lc02_SSN, NULL );
static DEVICE_ATTR(Model_Name, 0666, ht24lc02_MN, NULL );
static DEVICE_ATTR(Reserved, 0666, ht24lc02_RS, NULL );
static DEVICE_ATTR(Cover_Type, 0666, ht24lc02_CT, NULL );
static DEVICE_ATTR(ACT_FLAG, 0666, ht24lc02_ACT, NULL );
static DEVICE_ATTR(Gauge, 0666, ht24lc02_GA, NULL );
static DEVICE_ATTR(reset_reading, 0666, ht24lc02_reset_reading, NULL );
static DEVICE_ATTR(byte_request, 0666, ht24lc02_single_byte_return, ht24lc02_single_byte_request);
static DEVICE_ATTR(gpio_high, 0666, ht24lc02_gpio_pull_high, NULL );
static DEVICE_ATTR(accessoryCover_Status, 0666, ht24lc02_CoverStatus, NULL );

/* Attribute Descriptor */

static struct attribute *eep_attrs[] = {
	//&dev_attr_eep_wr_test.attr,
	//&dev_attr_erase_1f.attr,
	//&dev_attr_one_1f.attr,
	&dev_attr_acinfo.attr,
	&dev_attr_ISN.attr,
	&dev_attr_SSN.attr,
	&dev_attr_Model_Name.attr,
	&dev_attr_Reserved.attr,
	&dev_attr_Cover_Type.attr,
	&dev_attr_ACT_FLAG.attr,
	&dev_attr_Gauge.attr,
	&dev_attr_reset_reading.attr,
	&dev_attr_byte_request.attr,
	&dev_attr_gpio_high.attr,
	&dev_attr_accessoryCover_Status.attr,
	NULL
};

/* Attribute Group */
static struct attribute_group eep_attr_group = {
	.attrs = eep_attrs,
};

struct ht24lc02_chip {
	struct i2c_client		*client;
	struct delayed_work		work;
	struct delayed_work		hand_work;
	struct power_supply		fgeeprom;
	struct power_supply		*bms_psy;	
	struct ht24lc02_platform_data	*pdata;
	
	/* State Of Connect */
	int online;
};

static void ht24lc02_reset_reading_in(void){
	ISN_state = 0, ISN_count = 0;
	SSN_state = 0, SSN_count = 0;
	MN_state = 0, MN_count = 0;
	RS_state = 0, RS_count = 0;
	CT_state = 0, CT_count = 0;
	ACT_state = 0, ACT_count = 0;
	GA_state = 0, GA_count = 0;	
	request_addr = 0, request_value = 0;
	request_status = 0;
}
static int ht24lc02_reset_reading(struct device *dev, struct device_attribute *attr, char *buf){
	// Reset reading count and status to prevent that you don't know where we go now.
	ht24lc02_reset_reading_in();
	return snprintf(buf, 50, "Reset Clear!\n");
}

static int ht24lc02_CoverStatus(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	int ret;
	ret = !gpio_get_value(60); // ER2
	printk("MULT_I2C_EN: %d\n", ret);
	return snprintf(buf, 50, "%d", ret);
}

static int ht24lc02_gpio_pull_high(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);	

	printk("MULT_I2C_EN: %d\n", gpio_get_value(15));
	//gpio_set_value(15, 0);
	gpio_direction_output(15, 1);
	printk("MULT_I2C_EN: %d\n", gpio_get_value(15));
	return snprintf(buf, 50, "Success");
}

static int ht24lc02_single_byte_request(struct device *dev, struct device_attribute *attr, char *buf, size_t count){
	struct i2c_client *client = to_i2c_client(dev);
	request_addr = 0, request_value = 0;
		
	printk("[Chihyu] ht24lc02_single_byte_request:\n");
	sscanf(buf, "%x", &request_addr);
	request_status = 1;
	request_value = ht24lc02_read_reg(client, request_addr);
	
	return snprintf(buf, 50, "Success!");
}
static int ht24lc02_single_byte_return(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	if( request_status == 1 )
	{
		if( request_value < 0 )
		{
			return snprintf(buf, 50, "Failed to read!");
			request_status = 0;
		}
		else
			return snprintf(buf, 50, "%c", request_value);
	}
	else
		return snprintf(buf, 50, "Haven't sent request address yet!");
}

static int ht24lc02_acinfo(struct device *dev, struct device_attribute *attr, char *buf, size_t count){
	struct i2c_client *client = to_i2c_client(dev);
	int wr = 0, ret = 0;
	u8 reg = 0, data = 0;
	
	printk("[Chihyu] ht24lc02_acinfo:\n");
	
	sscanf(buf, "%d %x %s", &wr, &reg, &data);
	if(wr == 1) // write reg
	{
		ret = ht24lc02_write_reg(client, reg, data);
		if(ret < 0)
			return snprintf(buf, 50, "Write Failed!");
		else
			return snprintf(buf, 50, "Success!");
	}
	else if(wr == 0)
	{
		ret = ht24lc02_read_reg(client, reg);
		if(ret < 0)
			return snprintf(buf, 50, "Read Failed!");
		else
			return snprintf(buf, 50, "%x", ret);
	}
		
	return snprintf(buf, 50, "Parameter fail, please check your input.");
}
/*
struct file* file_open(char* path, int flags, int rights) {
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int err = 0;

    oldfs = get_fs();
    set_fs(get_ds());
    filp = filp_open(path, flags, rights);
    set_fs(oldfs);
    if(IS_ERR(filp)) {
        err = PTR_ERR(filp);
        return NULL;
    }
    return filp;
}
void file_close(struct file* file){
	filp_close(file, NULL);
}

int file_read(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_read(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}
   
int file_write(struct file* file, unsigned long long offset, unsigned char* data, unsigned int size) {
    mm_segment_t oldfs;
    int ret;

    oldfs = get_fs();
    set_fs(get_ds());

    ret = vfs_write(file, data, size, &offset);

    set_fs(oldfs);
    return ret;
}

int file_sync(struct file* file){
	vfs_fsync(file, 0);
	return 0;
}*/
static int ht24lc02_read_one(struct device *dev, struct device_attribute *attr, char *buf, u8 addr, int dsize, int *state, int *count)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 data_tmp;
	printk("At Begin -> data_tmp:%x, count:%d, state:%d\n", data_tmp, *count, *state);
	
	if( *state == 0 ) // Not reading yet, set reading.
	{
		*state = 1;	*count = 0;
	}
	
	// Read one byte.
	data_tmp = ht24lc02_read_reg(client, addr + *count);
	*count = *count + 1;
	printk("data_tmp:%x, count:%d, state:%d\n", data_tmp, *count, *state);
	
	// Check if last byte, if so, set to not reading status.
	if( *count == dsize )
		*state = 0;
	if( data_tmp < 0 )
	{
		return snprintf(buf, 50, "Failed to read!");
		*state = 0;
	}
	else
		return snprintf(buf, 50, "%c", data_tmp);
}
/*
static int ht24lc02_read_EEP(struct device *dev, struct device_attribute *attr, char *buf, u8 addr, int dsize){
	struct i2c_client *client = to_i2c_client(dev);
	int i = 0;
	// char *data_str[dsize]; 
	char *data_str[50];
	u8 data_tmp;
	struct file *filep;
	filep = file_open("/data/data/ISN", O_CREAT|O_RDWR|S_IRWXU|S_IRWXG|S_IRWXO, 0644);
	if( filep == NULL )
		return sprintf( buf, "Failed!");	
	
	for(i = 0; i < dsize; i++){
		//printk("data_str[%d]:%s \n", i, data_str[i]);		
		data_tmp = ht24lc02_read_reg(client, addr+i);
		sprintf(data_str, "%x", data_tmp);
		//file_write(filep, i, ht24lc02_read_reg(client, addr+i), 1);
		file_write(filep, i, data_str, 1);
		file_sync(filep);
		printk("data_tmp:%x\n", data_tmp);
		sprintf(buf, "data_tmp:%x\n", data_tmp);
	}
	file_close(filep);
	
	return sprintf(buf, "Success save to /data/data/ISN !\n");
	//return sprintf(buf, "%s", data_str);
}*/
static int ht24lc02_ISN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, ISN_ADDR, ISN_SIZE, &ISN_state, &ISN_count);}
static int ht24lc02_SSN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, SSN_ADDR, SSN_SIZE, &SSN_state, &SSN_count);}
static int ht24lc02_MN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, MN_ADDR, MN_SIZE, &MN_state, &MN_count);}
static int ht24lc02_RS(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, RS_ADDR, RS_SIZE, &RS_state, &RS_count);}
static int ht24lc02_CT(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, CT_ADDR, CT_SIZE, &CT_state, &CT_count);}
static int ht24lc02_ACT(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, ACT_ADDR, ACT_SIZE, &ACT_state, &ACT_count);}
static int ht24lc02_GA(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_one(dev, attr, buf, GA_ADDR, GA_SIZE, &GA_state, &GA_count);}

/*
static int ht24lc02_ISN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, ISN_ADDR, ISN_SIZE);}
static int ht24lc02_SSN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, SSN_ADDR, SSN_SIZE);}
static int ht24lc02_MN(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, MN_ADDR, MN_SIZE);}
static int ht24lc02_RS(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, RS_ADDR, RS_SIZE);}
static int ht24lc02_CT(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, CT_ADDR, CT_SIZE);}
static int ht24lc02_ACT(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, ACT_ADDR, ACT_SIZE);}
static int ht24lc02_GA(struct device *dev, struct device_attribute *attr, char *buf){return ht24lc02_read_EEP(dev, attr, buf, GA_ADDR, GA_SIZE);}
*/
/*
static int ht24lc02_read_test(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	int i = 0;
	printk("[Chihyu] ht24lc02_read_test:\n");
	for(i = 0; i <= 10; i++)
	{
		ht24lc02_write_reg(client, i, i*10);		
	}	
	for(i = 0; i <= 10; i++)
	{
		snprintf(buf, 255, "addr:%02x value:%02x\n", i, ht24lc02_read_reg(client, i));
	}
	return snprintf(buf, 255, "addr:%02x value:%02x\n", i, ht24lc02_read_reg(client, i));
}
static int ht24lc02_write_test(struct device *dev, struct device_attribute *attr, char *buf, size_t count){
	struct i2c_client *client = to_i2c_client(dev);
	
	u8 addr= 0, write_byte = 0x00;
	sscanf(buf, "%x%x", &addr, &write_byte);
	printk("[Chihyu] buf: %s, addr: %x, value: %x \n" , buf, addr, write_byte);
	sscanf(buf, "%x %x", &addr, &write_byte);
	printk("[Chihyu] buf: %s, addr: %x, value: %x \n" , buf, addr, write_byte);
	ht24lc02_write_reg(client, addr, write_byte);
	
	// int j = 0;
	// u8 num_of_byte;
	// char value[300]; 
	// u8 output[300];
	// format: "reg_addr num_of_byte value"
	// sscanf(buf, "%02x %d %x", &addr, &num_of_byte, &output);
	
	char *p;
	*p = strtok(buf, " ");
	addr = (unsigned char)strtol(p, NULL, 16);
	p = strtok(NULL, " ");
	num_of_byte = strtol(p, NULL, 10);
	p = strtok(NULL, " ");
	value = (unsigned char)strtol(p, NULL, 16);
	
	// printk("[Chihyu] buf: %s, addr: %x, value: %x \n" , buf, addr, *output);
	for(j = 0; j < num_of_byte; j++)
	{
		// printk("j:%d, *value:%x, value:%x", j, *(value + num_of_byte - j), value + num_of_byte - j);
		printk("j: %d, output:%x, *output:%x", j, output+j, *(output+( num_of_byte - (j+1))));
		// sscanf(&value[ j * 2 ], "%2hhx", &output[j]);
		ht24lc02_write_reg(client, (addr+output+j), *(output+( num_of_byte - (j+1))));
		//ht24lc02_write_reg(client, (addr +( num_of_byte - (j+1))), *(output+j));
	}	
	
	return strnlen(buf, count);
}
*/
/*
static int ht24lc02_erase_all(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	int ers;

	for(ers = 0; ers <= 31; ers++)
		i2c_smbus_write_byte_data(client, ers, 0x00);	
	return snprintf(buf, 50, "erase success");
}
static int ht24lc02_write_one_all(struct device *dev, struct device_attribute *attr, char *buf){
	struct i2c_client *client = to_i2c_client(dev);
	int ers;

	for(ers = 0; ers <= 31; ers++)
		i2c_smbus_write_byte_data(client, ers, 0x01);	
	return snprintf(buf, 50, "write one all success");
}
static int ht24lc02_erase_1f(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int addr= 0, ers = 0;
	sscanf(buf, "%x", &addr);
	for(ers = 0; ers <= 31; ers++, addr++)
		i2c_smbus_write_byte_data(client, addr, 0);
	return snprintf(buf, 50, "erase from %x to %x success", addr, addr+ers);
}
static int ht24lc02_write_one_1f(struct device *dev, struct device_attribute *attr, char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int addr= 0, ers = 0;
	sscanf(buf, "%x", &addr);
	for(ers = 0; ers <= 31; ers+=0x01, addr+=0x01)
	{
		printk("[Chihyu] address:%02x ers:%x ", addr, ers);
		i2c_smbus_write_byte_data(client, addr, 1);
	}
	return snprintf(buf, 50, "write one from %x to %x success", addr, addr+ers);
}
*/
int ht24lc02_write(u8 reg, u8 value)
{
	return ht24lc02_write_reg(test_i2c_clt, reg, value);
}
EXPORT_SYMBOL(ht24lc02_write);

int ht24lc02_read(u8 reg)
{
	return ht24lc02_read_reg(test_i2c_clt, reg);
}
EXPORT_SYMBOL(ht24lc02_read);

static int ht24lc02_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, value);

	printk("[Chihyu] write_reg reg:%02x value:%02x\n", reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

static int ht24lc02_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);

	printk("[Chihyu] read_reg ret = %d\n", ret);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	return ret;
}

//get chip version
/*
static u16 ht24lc02_get_version(struct i2c_client *client)
{
    u16 fg_version = 0;
    u16 chip_version = 0;

    fg_version = ht24lc02_read_reg(client, ht24lc02_VER_REG);
    chip_version = swab16(fg_version);
  
    dev_info(&client->dev, "ht24lc02 Fuel-Gauge Ver 0x%04x\n", chip_version);
    return chip_version;
}*/

/*static void ht24lc02_get_online(struct i2c_client *client)
{
    struct ht24lc02_chip *chip = i2c_get_clientdata(client);

    if (chip->pdata->eeprom_online)
			chip->online = chip->pdata->eeprom_online();
    else
			chip->online = 1;
}*/
/*
static enum power_supply_property ht24lc02_eeprom_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
};*/

static int proc_ht24lc02_i2c(struct seq_file *m, void *p)
{
	int ret;

	ret = ht24lc02_read_reg(test_i2c_clt, 0x00);
	pr_info("%s, %x\n", __func__, ret);
	seq_printf(m, "%x\n", ret);

	return 0;
}

static int proc_ht24lc02_i2c_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_ht24lc02_i2c, NULL);
}

static ssize_t proc_ht24lc02_i2c_write(struct file *file, char __user *buffer, size_t count, loff_t *pos)
{
	int ret;

	ret = ht24lc02_write_reg(test_i2c_clt, 0x00, 0x00);
	if (ret < 0) {
		pr_err("failed to Unlock Model Access\n");
		return ret;
	}

	return count;
}

static const struct file_operations proc_ht24lc02_i2c_ops = {
	.open = proc_ht24lc02_i2c_open,
	.read = seq_read,
	.write = proc_ht24lc02_i2c_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_ht24lc02_i2c_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry =
	    proc_create("driver/ht24lc02_i2c_test", 0664, NULL,
			&proc_ht24lc02_i2c_ops);
	if (!entry) {
		printk("[%s] Unable to create ht24lc02_i2c_test\n",
		       __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}

static int ht24lc02_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);	
	struct ht24lc02_chip *chip;
	int ret;
	ht24lc02_reset_reading_in();
	
	// GPIO PULL HIGH to test
	/*ret = gpio_request(15, "MULT_I2C_EN");
	if (ret < 0){
		printk("%s: request MULT_I2C_EN gpio fail!\n", __func__);
	}
	
	printk("MULT_I2C_EN: %d\n", gpio_get_value(15));
	//gpio_set_value(15, 0);
	gpio_direction_output(15, 1);
	printk("MULT_I2C_EN: %d\n", gpio_get_value(15));
	*/
	pr_info("%s: ===== begin =====\n", __func__);
	printk("[Chihyu] At ht24lc02_probe:\n");
	test_i2c_clt = client;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->client = client;
	chip->pdata = client->dev.platform_data;

	i2c_set_clientdata(client, chip);
	
	sysfs_create_group(&client->dev.kobj, &eep_attr_group);
	
	chip->fgeeprom.name		= "ht24lc02_eeprom";
	//chip->fgeeprom.type		= POWER_SUPPLY_TYPE_eeprom;
	//chip->fgeeprom.get_property	= ht24lc02_get_property;
	//chip->fgeeprom.properties	= ht24lc02_eeprom_props;
	//chip->fgeeprom.num_properties	= ARRAY_SIZE(ht24lc02_eeprom_props);

	//ht24lc02_get_version(client);

	ret = create_ht24lc02_i2c_proc_fs();
	if (ret) {
		pr_err("%s: Unable to create ht24lc02_i2c_proc_fs\n", __func__);
	}
	/* Test For initial detect to write value 0x87 to address 0x33 */
	//printk("[Chihyu] use value:0x87 to write_reg to address:0x33\n");
	//ht24lc02_write_reg(client, 0x33, 0x87);
	//ht24lc02_read_reg(client, 0x33);
	cover_detect();

	return 0;
	/* End here for testing */
	
	/* ht24lc02_get_version(client);
  
	handle_model(client,LOAD_MODEL);
  
	INIT_DEFERRABLE_WORK(&chip->work, ht24lc02_work);
	INIT_DEFERRABLE_WORK(&chip->hand_work, ht24lc02_handle_work);
	schedule_delayed_work(&chip->hand_work,0);
	schedule_delayed_work(&chip->work, 0);

	
	return 0;
	*/
}

static int ht24lc02_remove(struct i2c_client *client)
{
	struct ht24lc02_chip *chip = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &eep_attr_group);
	power_supply_unregister(&chip->fgeeprom);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM

static int ht24lc02_suspend(struct i2c_client *client,
		pm_message_t state)
{
	struct ht24lc02_chip *chip = i2c_get_clientdata(client);

//	cancel_delayed_work(&chip->work);
	return 0;
}

static int ht24lc02_resume(struct i2c_client *client)
{
	struct ht24lc02_chip *chip = i2c_get_clientdata(client);

//	schedule_delayed_work(&chip->work, ht24lc02_DELAY);
	return 0;
}

#else

#define ht24lc02_suspend NULL
#define ht24lc02_resume NULL

#endif /* CONFIG_PM */

static struct of_device_id ht24lc02_match_table[] = {
	{ .compatible = "hol,ht24lc02-fg"},
	{ },
};

static const struct i2c_device_id ht24lc02_id[] = {
	{ "ht24lc02_eeprom", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ht24lc02_id);

static struct i2c_driver ht24lc02_i2c_driver = {
	.driver	= {
		.name	= "ht24lc02_eeprom",
		.of_match_table	= ht24lc02_match_table,
	},
	.probe		= ht24lc02_probe,
	.remove		= ht24lc02_remove,
	.suspend	= ht24lc02_suspend,
	.resume		= ht24lc02_resume,
	.id_table	= ht24lc02_id,
};
//module_i2c_driver(ht24lc02_i2c_driver);

static const struct idi_device_id idi_ids[] = {
	{
	 .vendor = IDI_ANY_ID,
	 .device = IDI_DEVICE_ID_INTEL_AG620,
	 .subdevice = IDI_SUBDEVICE_ID_ASUS_EEP,
	 },

	{ /* end: all zeroes */ },
};

static int ht24lc02_idi_probe(struct idi_peripheral_device *ididev,
			      const struct idi_device_id *id)
{
	// struct resource *res;
	// void __iomem *ctrl_io;
	// void __iomem *pcfg_io;
	int ret = 0;
	
	
	pr_info("%s() - begin\n", __func__);

	ret = idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
		       __func__);
		return ret;
	}

	pr_info("%s() - end\n", __func__);

	return 0;
}

static int __exit ht24lc02_idi_remove(struct idi_peripheral_device *ididev)
{
	pr_info("%s\n", __func__);

	return 0;
}

static struct idi_peripheral_driver ht24lc02_idi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ht24lc02_idi",
		   .pm = NULL,
		   },
	.p_type = IDI_EEP,
	.id_table = idi_ids,
	.probe = ht24lc02_idi_probe,
	.remove = ht24lc02_idi_remove,
};

static int __init ht24lc02_eeprom_init(void)
{
	int ret;

	ret = idi_register_peripheral_driver(&ht24lc02_idi_driver);
	if (ret) {
		printk("----------- %s fail --------------", __func__);
		return ret;
	}

	ret = i2c_add_driver(&ht24lc02_i2c_driver);
	if (ret)
		pr_err("%s: i2c_add_driver failed\n", __func__);

	return ret;
}

static void __exit ht24lc02_eeprom_exit(void)
{
	i2c_del_driver(&ht24lc02_i2c_driver);
}


MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("ht24lc02 EEPROM driver");
MODULE_LICENSE("GPL");

module_init(ht24lc02_eeprom_init);
module_exit(ht24lc02_eeprom_exit);
