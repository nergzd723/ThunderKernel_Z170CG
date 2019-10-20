
#include "sony_cxd4756gf.h"
#include "sony_cadiz_parameter/BOE_TV070WXM-TU1.h"

#if SUPPORT_IOCONTROL
#include "cadiz_ioctl.h"
#endif
#include <../arch/x86/platform/asustek/include/asustek_boardinfo.h>

#define QCT_GPIO_OFFSET	911

#define CADIZ_I2C_ADDR	0x11 //SLAVEAD pin high
#define CADIZ_I2C_ADAPTER	6

#define RETRY_TIMES	5

#if DEBUG_BOOT_REGISTERS
static char* boot1_path = "/sdcard/Cadiz_boot1.csv";
static char* boot2_path = "/sdcard/Cadiz_i2c_boot2.csv";
static char* boot3_path = "/sdcard/Cadiz_boot3.csv";
static char* boot4_path = "/sdcard/Cadiz_boot4.csv";
static char* ipc_ibc_path = "/sdcard/Cadiz_ipc_ibc.csv";
#endif
static char* check_is_mp_path = "/factory/isMPdevice";
static int lcm_bl_en_gpio = 46;
static int lcm_vhigh_gpio = 45;
static int cadiz_reset_gpio = 77;
static int cadiz_sysclk_en_gpio = 76;
static int cadiz_pwr_en_gpio = 75;
static int asus_hardware_id;
static int enable_log;
static int enable_cabc_selection_function;
static int disable_cabc = 0;
//Cadiz CABC Setting Regs
static int ipc_ibc0 = 0;
static int ipc_ibc1 = 1;
static int ipc_ibc6 = 6;
static int ipc_ibc11 = 11;
static int ipc_ibc231 = 231;

/*
static int cadiz_reset_gpio;
static int cadiz_pwr_en_gpio;
static int cadiz_sysclk_en_gpio;
*/
static struct i2c_client *cadiz_client;
static struct cadiz_i2c_reg_val* p_boot1;
static struct cadiz_i2c_reg_val* p_boot2;
static struct cadiz_i2c_reg_val* p_boot3;
static struct cadiz_i2c_reg_val* p_boot4;
static struct cadiz_i2c_reg_val* p_ipc_ibc;
static int len_boot1;
static int len_boot2;
static int len_boot3;
static int len_boot4;
static int len_ipc_ibc;
static int check_tables_exist(void);
static int save_tables_and_pass_regs(struct cadiz_regs*, int save);

#if SUPPORT_IOCONTROL
static struct miscdevice cadiz_dev;
static struct cadiz_regs cadiz_regs;
static u16 dbg_read_reg;
#endif
static struct work_struct set_brightness_work;
static int cadiz_bl_level;
extern void cadiz_led_disable_fsys(int);
extern int rockchip_disp_init(void);
extern void rockchip_disp_exit(void);

/*add for different lcm*/
static struct cadiz_i2c_reg_val* boot1;
static struct cadiz_i2c_reg_val* boot2;
static struct cadiz_i2c_reg_val* boot3;
static struct cadiz_i2c_reg_val* boot4;
static struct cadiz_i2c_reg_val* ipc_ibc;
static int len_boot1, len_boot2, len_boot3, len_boot4, len_ipc_ibc;


void cadiz_power(int val)
{
	pr_info("%s: val=%d\n", __func__, val);
	if(asus_hardware_id == ER)
		return 0;
	if (val > 0)
		gpio_set_value_cansleep(cadiz_pwr_en_gpio, 1);
	else
		gpio_set_value_cansleep(cadiz_pwr_en_gpio, 0);
}

void cadiz_sysclk_power(int val)
{
	pr_info("%s: val=%d\n", __func__, val);
	if (val > 0)
		gpio_set_value_cansleep(cadiz_sysclk_en_gpio, 1);
	else
		gpio_set_value_cansleep(cadiz_sysclk_en_gpio, 0);
}

void lcm_vhigh_on_off(int val)
{
	pr_info("%s: val=%d\n", __func__, val);
	if (val > 0)
		gpio_set_value_cansleep(lcm_vhigh_gpio, 1);
	else
		gpio_set_value_cansleep(lcm_vhigh_gpio, 0);
}

void lcm_bl_on_off(int val)
{
        pr_info("%s: val=%d\n", __func__, val);
	struct file *fp;
        fp = filp_open(check_is_mp_path, O_RDONLY, 0);
        if (IS_ERR_OR_NULL(fp)) {
                pr_info("%s: device is not mp !!!\n", __func__);
        }else{
                pr_info("%s: device is mp and need to disable fsys !!!\n", __func__);
                filp_close(fp, NULL);
                cadiz_led_disable_fsys(1);
        }
        if (val > 0)
                gpio_set_value_cansleep(lcm_bl_en_gpio, 1);
        else
                gpio_set_value_cansleep(lcm_bl_en_gpio, 0);
}

void cadiz_reset(int val)
{
	pr_info("%s: val=%d\n", __func__, val);
	if (val > 0)
		gpio_set_value_cansleep(cadiz_reset_gpio, 1);
	else
		gpio_set_value_cansleep(cadiz_reset_gpio, 0);
}

static int cadiz_i2c_reg_read(struct i2c_client *client, u16 reg, u8 *value)
{
	int r;
	int retry_cnt = 0;
	u8 tx_data[] = {
		reg >> 8 ,      //SubAddress(MSB) 8bit
		reg & 0xff,     //SubAddress(LSB) 8bit
	};

	u8 rx_data[1]={0};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
		{
			.addr = client->addr,
			.flags = I2C_M_RD,
			.buf = rx_data,
			.len = ARRAY_SIZE(rx_data),
		},
	};

	if (!client) {
		pr_err("no cadiz i2c\n");
			return 0;
	}

retry:
	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));

	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x error %d (retry_cnt=%d)\n",
				__func__, reg, r, retry_cnt);
		if (retry_cnt < RETRY_TIMES) {
			retry_cnt++;
			mdelay(1);
			goto retry;
		} else
			return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x msgs %d\n", __func__,
				reg, r);
		return -EAGAIN;
	}

	*value = rx_data[0];

	return r;
}

static int cadiz_i2c_reg_write(struct i2c_client *client, u16 reg, u8 value)
{
	int r;
	int retry_cnt = 0;
	u8 tx_data[] = {
		reg >> 8 ,      //SubAddress(MSB) 8bit
		reg & 0xff,     //SubAddress(LSB) 8bit
		value & 0xff,   //Data 8bit
	};
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.buf = tx_data,
			.len = ARRAY_SIZE(tx_data),
		},
	};

	if (!client) {
		pr_err("no cadiz i2c\n");
			return 0;
	}

retry:
	r = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (r < 0) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x error %d (retry_cnt=%d)\n",
				__func__, reg, value, r, retry_cnt);
		if (retry_cnt < RETRY_TIMES) {
			retry_cnt++;
			mdelay(1);
			goto retry;
		} else
			return r;
	}

	if (r < ARRAY_SIZE(msgs)) {
		dev_err(&client->dev, "%s: reg 0x%04x val 0x%08x msgs %d\n",
				__func__, reg, value, r);
		return -EAGAIN;
	}

	return r;
}

static int cadiz_set_brightness(struct work_struct *work){
        struct cadiz_i2c_reg_val cadiz_reg;
        static bool power_on;
        cadiz_reg.reg = 0xC28;
        cadiz_reg.val = cadiz_bl_level;
        if(enable_log)
                pr_info("%s: cadiz level=0x%x\n", __func__, cadiz_bl_level);
        if (cadiz_bl_level) {
                if (!power_on) {
			msleep(50);
                        lcm_bl_on_off(1);
                }
                mdelay(10);
                cadiz_setup_regs(&cadiz_reg, 1);
                power_on = true;
        }else{
                cadiz_setup_regs(&cadiz_reg, 1);
                if (power_on) {
                        lcm_bl_on_off(0);
                }
                power_on = false;
        }
        return 0;
}

int cadiz_trigger_brightness_work(u8 bl_level){
        cadiz_bl_level = bl_level;
        if(enable_log)
                pr_info("%s: cadiz level=0x%x\n", __func__, bl_level);
        schedule_work(&set_brightness_work);
        return 0;
}

int cadiz_enable_i2c(void)
{
	pr_info("%s ++\n", __func__);
	return cadiz_i2c_reg_write(cadiz_client, 0x0830, 0x00);
	pr_info("%s --\n", __func__);
}

int cadiz_wait_tx_init_done(void)
{
#define CADIZ_POLLING_COUNT 10
#define MAGIC_REG	0x0207
#define MAGIC_VALUE 0x40

	int cnt, r;
	u8 rdata;

	pr_info("%s ++\n", __func__);

	for(cnt = 0; cnt < CADIZ_POLLING_COUNT; cnt++) {
		r = cadiz_i2c_reg_read(cadiz_client, MAGIC_REG, &rdata);

		if (r < 0) {
			break;
		} else if (rdata != MAGIC_VALUE) {
			usleep_range(10000, 10000);
			continue;
		}else {
			pr_info("wait count = %d\n", cnt);
			break;
		}
	}

	if (rdata!= MAGIC_VALUE) {
		pr_err("%s: Polling FAIL :0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n"
			, __func__, MAGIC_REG, rdata, r, MAGIC_VALUE);
		return -EIO;
	} else {
		pr_info("%s: Polling SUCCESS\n", __func__);
	}

	pr_info("%s --\n", __func__);
	return r;

#undef CADIZ_POLLING_COUNT
#undef MAGIC_REG
#undef MAGIC_VALUE
}

int cadiz_wait_trans_complete(void)
{
#define CADIZ_POLLING_COUNT 50
#define MAGIC_REG	0x0404
#define MAGIC_VALUE 0x38

	int cnt, r;
	u8 rdata;

	pr_info("%s ++\n", __func__);

	for(cnt = 0; cnt < CADIZ_POLLING_COUNT; cnt++) {
		r = cadiz_i2c_reg_read(cadiz_client, MAGIC_REG, &rdata);

		if (r < 0) {
			break;
		} else if (rdata != MAGIC_VALUE) {
			usleep_range(10000, 10000);
			continue;
		}else {
			pr_info("wait count = %d\n", cnt);
			break;
		}
	}

	if (rdata!= MAGIC_VALUE) {
		pr_err("%s: Polling FAIL :0x%04xh = 0x%02x (r=%d)(expect=0x%02x)\n"
			, __func__, MAGIC_REG, rdata, r, MAGIC_VALUE);
		return -EIO;
	} else {
		pr_info("%s: Polling SUCCESS\n", __func__);
	}

	pr_info("%s --\n", __func__);
	return r;

#undef CADIZ_POLLING_COUNT
#undef MAGIC_REG
#undef MAGIC_VALUE
}

#if SUPPORT_REG_READ_WRITE
static u8 AscIItoByte(u8 Ascii)
{
	u8 B;
	if (Ascii >= 0x61)
		B = Ascii-0x57;
	else
		B = Ascii >= 0x41 ? Ascii-0x37 : Ascii-0x30;
	return B;
}

static ssize_t dbg_reg_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int dbg_reg_write(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char dbg_buf[20];
	u16 dbg_reg;
	u8 dbg_val;
	int r, j;
	struct cadiz_regs cadiz_regs;

	if(check_tables_exist())
		return -EFAULT;
	if(count > sizeof(dbg_buf))
		return -EFAULT;
	if(copy_from_user(dbg_buf, buf, count))
		return -EFAULT;
	dbg_buf[count] = '\0';

	//sscanf(dbg_buf, "%4x[ ]%2x", &dbg_reg, &dbg_val);
	dbg_reg = AscIItoByte(dbg_buf[0]) << 12 |
		  AscIItoByte(dbg_buf[1]) << 8 |
		  AscIItoByte(dbg_buf[2]) << 4 |
		  AscIItoByte(dbg_buf[3]);
	dbg_val = AscIItoByte(dbg_buf[5]) << 4 |
		  AscIItoByte(dbg_buf[6]);

	pr_info("%s: reg=0x%x, val=0x%x"
		, __func__, dbg_reg, dbg_val);

	cadiz_regs.len = 1;
	cadiz_regs.regs[0][0] = dbg_reg;
	cadiz_regs.regs[0][1] = dbg_val;
	r = save_tables_and_pass_regs(&cadiz_regs, 0);
	if(r < 0)
		return r;

	return count;
}

static const struct file_operations dbg_write_reg_fops = {
	.open		= dbg_reg_open,
	.write		= dbg_reg_write,
};

static int dbg_read_reg_write(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	char dbg_buf[20];
	u16 dbg_reg;

	if(count > sizeof(dbg_buf))
		return -EFAULT;
	if(copy_from_user(dbg_buf, buf, count))
		return -EFAULT;
	dbg_buf[count] = '\0';

	//sscanf(dbg_buf, "%4x[ ]%2x", &dbg_reg, &dbg_val);
	dbg_reg = AscIItoByte(dbg_buf[0]) << 12 |
		  AscIItoByte(dbg_buf[1]) << 8 |
		  AscIItoByte(dbg_buf[2]) << 4 |
		  AscIItoByte(dbg_buf[3]);

	pr_info("%s: to read reg=0x%04x\n"
		, __func__, dbg_reg);

	dbg_read_reg = dbg_reg;

	return count;
}

static ssize_t dbg_read_reg_read(struct file *file, char __user *user_buf,
		size_t count, loff_t *ppos)
{
	char buf[20];
	u16 reg = dbg_read_reg;
	u8 val;
	int r;

	r = cadiz_i2c_reg_read(cadiz_client, reg, &val);
	if (r < 0)
		snprintf(buf, 20, "%s\n", "i2c read fail...");
	else
		snprintf(buf, 20, "0x%04x 0x%02x\n", reg, val);

	return simple_read_from_buffer(user_buf, count, ppos,
			buf, strlen(buf));
}

static const struct file_operations dbg_read_reg_fops = {
	.open		= simple_open,
	.write		= dbg_read_reg_write,
	.read		= dbg_read_reg_read,
};
#endif

#if DEBUG_BOOT_REGISTERS
static int read_registers_from_file(char* path, struct cadiz_i2c_reg_val* a)
{
	mm_segment_t old_fs;
	struct file *fp;
	int byte_cnt, f_size, i;
	int num_head_line_skip = 2;
	int line_cnt = 1;
	int start_read = 0;
	u16 read_reg;
	u8 read_val;
	u8 *pbuf;

	fp = filp_open(path, O_RDONLY, 0);
	if (!IS_ERR_OR_NULL(fp)) {
		pr_info("%s opened success\n", path);
		f_size = fp->f_dentry->d_inode->i_size;
		pr_info("f_size=%d\n", f_size);
		pbuf = kmalloc(f_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if (fp->f_op && fp->f_op->read) {
			pr_info("Start to read\n");
			byte_cnt = fp->f_op->read(fp, pbuf, f_size, &fp->f_pos);
			if(byte_cnt <= 0) {
				pr_err("EOF or error. last byte_cnt= %d;\n", byte_cnt);
				kfree(pbuf);
				return -1;
			}
			for (i = 0; i < f_size; i++) {
#if 0
				pr_info("pbuf[%d]: c=%c, d=%d\n", i, pbuf[i], pbuf[i]);
#endif
				if(pbuf[i] == 0xA) { //new line
					line_cnt ++;
					start_read = 1;
					continue;
				}
				if(line_cnt <= num_head_line_skip) {
					start_read = 0;
					continue;
				}
				if(start_read) {
					start_read = 0;
					read_reg = AscIItoByte(pbuf[i]) << 12;
					i++;
					read_reg |= AscIItoByte(pbuf[i]) << 8;
					i++;
					read_reg |= AscIItoByte(pbuf[i]) << 4;
					i++;
					read_reg |= AscIItoByte(pbuf[i]);

					i++;
					if (pbuf[i] != ',') {
						pr_err("invalid format pbuf[%d] = %c, stop\n", i, pbuf[i]);
						return -1;
					}
					i++;
					read_val = AscIItoByte(pbuf[i]) << 4;
					i++;
					read_val |= AscIItoByte(pbuf[i]);
#if 0
					pr_info("line_cnt=%d, read_reg=0x%04x, read_val=0x%02x\n"
						, line_cnt, read_reg, read_val);
#endif
					a[line_cnt - num_head_line_skip - 1].reg = read_reg;
					a[line_cnt - num_head_line_skip - 1].val = read_val;
				}
			}
			pr_info("End of read\n");
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else {
		pr_err("%s opened fail\n", path);
		return -1;
	}
	return 0;
}
#endif

int cadiz_setup_boot1(void)
{
	int i, r, len;
	int ret = 0;
#if DEBUG_BOOT_REGISTERS
	struct cadiz_i2c_reg_val debug_boot1[173];
#endif

	pr_info("%s ++\n", __func__);

#if DEBUG_BOOT_REGISTERS
	ret = read_registers_from_file(boot1_path, debug_boot1);
	if(ret < 0) {
		pr_err("%s: read fail\n", __func__);
		return -EAGAIN;
	}

	len = sizeof(debug_boot1)/sizeof(debug_boot1[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot1[i].reg, debug_boot1[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	//len = sizeof(boot1) / sizeof(boot1[0]);
	len = len_boot1; 
	pr_info("len = %d len_boot1 = %d\n",len,len_boot1);

	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, boot1[i].reg, boot1[i].val);
		if (r < 0)
			return -EIO;
	}
	if(!p_boot1) {
		p_boot1 = boot1;
		len_boot1 = len;
	}
#endif

	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_boot2(void)
{
	int i, r, len;
	int ret = 0;
#if DEBUG_BOOT_REGISTERS
	struct cadiz_i2c_reg_val debug_boot2[12];
#endif

	pr_info("%s ++\n", __func__);

#if DEBUG_BOOT_REGISTERS
	ret = read_registers_from_file(boot2_path, debug_boot2);
	if(ret < 0) {
		pr_err("%s: read fail\n", __func__);
		return -EAGAIN;
	}

	len = sizeof(debug_boot2)/sizeof(debug_boot2[0]);
	pr_info("len = %d len_boot2 = %d\n",len,len_boot2);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot2[i].reg, debug_boot2[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	//len = sizeof(boot2) / sizeof(boot2[0]);
	len = len_boot2;
	pr_info("len = %d len_boot2 = %d\n",len,len_boot2);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, boot2[i].reg, boot2[i].val);
		if (r < 0)
			return -EIO;
	}
	if(!p_boot2) {
		p_boot2 = boot2;
		len_boot2 = len;
	}
#endif

	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_boot3(void)
{
	int i, r, len;
	int ret = 0;
#if DEBUG_BOOT_REGISTERS
	struct cadiz_i2c_reg_val debug_boot3[3];
#endif

	pr_info("%s ++\n", __func__);

#if DEBUG_BOOT_REGISTERS
	ret = read_registers_from_file(boot3_path, debug_boot3);
	if(ret < 0) {
		pr_err("%s: read fail\n", __func__);
		return -EAGAIN;
	}

	len = sizeof(debug_boot3)/sizeof(debug_boot3[0]);
	pr_info("len = %d len_boot1 = %d\n",len,len_boot3);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot3[i].reg, debug_boot3[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	//len = sizeof(boot3) / sizeof(boot3[0]);
	len = len_boot3;
	pr_info("len = %d len_boot3 = %d\n",len,len_boot3);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, boot3[i].reg, boot3[i].val);
		if (r < 0)
			return -EIO;
	}
	if(!p_boot3) {
		p_boot3 = boot3;
		len_boot3 = len;
	}
#endif

	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_boot4(void)
{
	int i, r, len;
	int ret = 0;
#if DEBUG_BOOT_REGISTERS
	struct cadiz_i2c_reg_val debug_boot4[4];
#endif

	pr_info("%s ++\n", __func__);

#if DEBUG_BOOT_REGISTERS
	ret = read_registers_from_file(boot4_path, debug_boot4);
	if(ret < 0) {
		pr_err("%s: read fail\n", __func__);
		return -EAGAIN;
	}

	len = sizeof(debug_boot4)/sizeof(debug_boot4[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_boot4[i].reg, debug_boot4[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	//len = sizeof(boot4) / sizeof(boot4[0]);
	len = len_boot4;
	pr_info("len = %d len_boot4 = %d\n",len,len_boot4);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, boot4[i].reg, boot4[i].val);
		if (r < 0)
			return -EIO;
	}
	if(!p_boot4) {
		p_boot4 = boot4;
		len_boot4 = len;
	}
#endif

	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_ipc_ibc(void)
{
	int i, r, len;
	int ret = 0;
#if DEBUG_BOOT_REGISTERS
	struct cadiz_i2c_reg_val debug_ipc_ibc[232];
#endif

	pr_info("%s ++\n", __func__);

#if DEBUG_BOOT_REGISTERS
	ret = read_registers_from_file(ipc_ibc_path, debug_ipc_ibc);
	if(ret < 0) {
		pr_err("%s: read fail\n", __func__);
		return -EAGAIN;
	}

	len = sizeof(debug_ipc_ibc)/sizeof(debug_ipc_ibc[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, debug_ipc_ibc[i].reg, debug_ipc_ibc[i].val);
		if (r < 0)
			return -EIO;
	}
#else
	//len = sizeof(ipc_ibc) / sizeof(ipc_ibc[0]);
	len = len_ipc_ibc;
	pr_info("len = %d len_ibc = %d\n",len,len_ipc_ibc);
        if(enable_cabc_selection_function){
                if(disable_cabc){
                        ipc_ibc[ipc_ibc0].val = 0x01;
                        ipc_ibc[ipc_ibc1].val = 0x00;
                        ipc_ibc[ipc_ibc6].val = 0x00;
                        ipc_ibc[ipc_ibc11].val = 0x01;
                        ipc_ibc[ipc_ibc231].val = 0x40;
                        pr_info(" ==== Cadiz Disable CABC  ==== \n");
                }else{
                        ipc_ibc[ipc_ibc0].val = 0x00;
                        ipc_ibc[ipc_ibc1].val = 0x02;
                        ipc_ibc[ipc_ibc6].val = 0x11;
                        ipc_ibc[ipc_ibc11].val = 0x51;
                        ipc_ibc[ipc_ibc231].val = 0x00;
                        pr_info(" ==== Cadiz Enable CABC  ==== \n");
                }
        }
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, ipc_ibc[i].reg, ipc_ibc[i].val);
		if (r < 0)
			return -EIO;
	}
	if(!p_ipc_ibc) {
		p_ipc_ibc = ipc_ibc;
		len_ipc_ibc = len;
	}
#endif
	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_pwm_reg(void)
{
	int ret = 0;
	pr_info("%s ++\n", __func__);
#if USE_CADIZ_PWM
	int i, r, len;
	len = sizeof(pwm) / sizeof(pwm[0]);
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, pwm[i].reg, pwm[i].val);
		if (r < 0)
			return -EIO;
	}
	pr_info("%s pwm form Cadiz!!!\n", __func__);
#endif
	pr_info("%s --\n", __func__);
	return ret;
}

int cadiz_setup_regs(struct cadiz_i2c_reg_val* regs, int len)
{
	int i, r;
	for(i = 0; i < len; i++) {
		r = cadiz_i2c_reg_write(cadiz_client, regs[i].reg, regs[i].val);
		if (r < 0)
			return -EIO;
	}
	return 0;
}

static int check_tables_exist(void)
{
	if(!p_boot1 || !p_boot2 || !p_boot3 || !p_boot4 || !p_ipc_ibc)
		return -1;
	else
		return 0;
}

static int save_tables_and_pass_regs(struct cadiz_regs* regs, int save)
{
	int i, j, r, len;
	u16 t_reg;
	u8  t_val;

	len = regs->len;

	for(i = 0; i < len; i++) {

		t_reg = (u16)(regs->regs[i][0]);
		t_val = (u8)(regs->regs[i][1]);

                if(!save) {
                        r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
                        if (r < 0)
                                return -EAGAIN;
                        continue;
                }

		//1. check if belong to ipc_ibc
		for(j = 0; j < len_ipc_ibc; j++)
			if(t_reg == p_ipc_ibc[j].reg)
				break;
		if(j == len_ipc_ibc) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to ipc_ibc\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EIO;

			//save the change
			p_ipc_ibc[j].reg = t_reg;
			p_ipc_ibc[j].val = t_val;
			continue;
		}
		//Most of changes will be expected in ipc_ibc, if not, check further.
		//2. if not, check boot1
		for(j = 0; j < len_boot1; j++)
			if(t_reg == p_boot1[j].reg)
				break;
		if(j == len_boot1) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot1\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EIO;

			//save the change
			p_boot1[j].reg = t_reg;
			p_boot1[j].val = t_val;
			continue;
		}
/*
		//3. if not, check boot2
		for(j = 0; j < len_boot2; j++)
			if(t_reg == p_boot2[j].reg)
				break;
		if(j == len_boot2) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot2\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EIO;

			//save the change
			p_boot2[j].reg = t_reg;
			p_boot2[j].val = t_val;
			continue;
		}

		//4. if not, check boot3
		for(j = 0; j < len_boot3; j++)
			if(t_reg == p_boot3[j].reg)
				break;
		if(j == len_boot3) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot3\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EIO;

			//save the change
			p_boot3[j].reg = t_reg;
			p_boot3[j].val = t_val;
			continue;
		}

		//5. if not, check boot4
		for(j = 0; j < len_boot4; j++)
			if(t_reg == p_boot4[j].reg)
				break;
		if(j == len_boot4) {
			dev_info(&cadiz_client->dev,
					"reg: 0x%08x does not belong to boot4\n", t_reg);
		} else {
			r = cadiz_i2c_reg_write(cadiz_client, t_reg, t_val);
			if (r < 0)
				return -EIO;

			//save the change
			p_boot4[j].reg = t_reg;
			p_boot4[j].val = t_val;
			continue;
		}
*/
		//the change does not belong to all tables
		dev_err(&cadiz_client->dev,
				"invalid reg: 0x%04x(0x%02x)\n", t_reg, t_val);
		return -EINVAL;
	}
	return 0;
}

#if SUPPORT_IOCONTROL
static long cadiz_dev_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
#define CADIZ_POLLING_COUNT 50

	dev_dbg(&cadiz_client->dev, "%s\n", __func__);
	switch(cmd) {
	case CADIZ_IOCTL_SET_REGISTERS:
	{
                struct cadiz_regs cadiz_regs;
		int i, r;

		if(check_tables_exist())
			return -EFAULT;

		memset(&cadiz_regs, 0, sizeof(cadiz_regs));
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EFAULT;
		}

		if(cadiz_regs.len > MAX_CADIZ_REGS || cadiz_regs.len <= 0) {
			dev_err(&cadiz_client->dev, "invalid value:%d\n", cadiz_regs.len);
			return -EINVAL;
		}

		r =  save_tables_and_pass_regs(&cadiz_regs, 1);
		if(r < 0)
			return r;

		break;
	}
	case CADIZ_IOCTL_GET_REGISTERS:
	{
		struct cadiz_regs cadiz_regs;
		u16 cur_reg;
		u8  cur_val;
		int i, r;

		memset(&cadiz_regs, 0, sizeof(cadiz_regs));
		if (copy_from_user(&cadiz_regs, (void __user *)arg,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "get arg error\n");
			return -EFAULT;
		}

		if(cadiz_regs.len > MAX_CADIZ_REGS || cadiz_regs.len <= 0) {
			dev_err(&cadiz_client->dev, "invalid value:%d\n", cadiz_regs.len);
			return -EINVAL;
		}

		for(i = 0; i< cadiz_regs.len; i++) {
			cur_reg = (u16)cadiz_regs.regs[i][0];
			cur_val = 0;
			r = cadiz_i2c_reg_read(cadiz_client, cur_reg, &cur_val);
			if (r < 0)
				return -EAGAIN;

			 cadiz_regs.regs[i][1] = (int)cur_val;
                        if(enable_log)
                                dev_info(&cadiz_client->dev, " READ[%d]: reg = 0x%02x, val = 0x%02x\n", i, cur_reg, cur_val);
		}

		if (copy_to_user((void __user *)arg, &cadiz_regs,
					sizeof(cadiz_regs))) {
			dev_err(&cadiz_client->dev, "pass arg error\n");
			return -EFAULT;
		}
		break;
	}
	default:
		dev_err(&cadiz_client->dev, "%s:unknown cmd=%d\n", __func__, cmd);
		return -EINVAL;
	}
	return 0;
#undef CADIZ_POLLING_COUNT
}

static int cadiz_dev_open(struct inode *inode, struct file *filp)
{
	dev_dbg(&cadiz_client->dev, "%s\n", __func__);
	return nonseekable_open(inode, filp);
}

static const struct file_operations cadiz_dev_fops = {
	.owner = THIS_MODULE,
	.open  = cadiz_dev_open,
	.unlocked_ioctl = cadiz_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = cadiz_dev_ioctl,
#endif
};
#endif

static int cadiz_i2c_probe(struct i2c_client *client,
                const struct i2c_device_id *id)
{
	pr_info("%s +++++++++++++++++++++++++++++++++++++++++\n", __func__);
	int asus_panel_id = asustek_boardinfo_get(FUN_LCM_ID);

        cadiz_client = NULL;
	p_boot1   = NULL;
	p_boot2   = NULL;
	p_boot3   = NULL;
	p_boot4   = NULL;
	p_ipc_ibc = NULL;
	len_boot1   = 0;
	len_boot2   = 0;
	len_boot3   = 0;
	len_boot4   = 0;
	len_ipc_ibc = 0;

        asus_hardware_id = asustek_boardinfo_get(FUN_HARDWARE_ID);
        switch(asus_hardware_id){
                case SR:
                        lcm_bl_en_gpio = 71;
                        printk("SR SKU backlight enable pin: %d \n",lcm_bl_en_gpio);
                        break;
                case ER:
                        lcm_bl_en_gpio = 46;
                        printk("ER SKU backlight enable pin: %d \n",lcm_bl_en_gpio);
                        break;
                case ER2:
                        lcm_bl_en_gpio = 46;
                        printk("ER2 SKU backlight enable pin: %d \n",lcm_bl_en_gpio);
                        break;
                default:
                        pr_info("No hardware id (%d) defined \n",asus_hardware_id);
        }

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality() failed\n");
		return -ENODEV;
	}

	cadiz_client = client;
	pr_info("%s: i2c device name=%s, addr=0x%x, adapter nr=%d\n", __func__,
			cadiz_client->name, cadiz_client->addr, cadiz_client->adapter->nr);

	if (gpio_request(cadiz_reset_gpio, "cadiz_rst")) {
		pr_err("failed to requeset cadiz_rst\n");
		return -ENODEV;
	}

	gpio_direction_output(cadiz_reset_gpio, 1);

	if (gpio_request(cadiz_sysclk_en_gpio, "cadiz_sysclk_en")) {
		pr_err("failed to requeset cadiz_sysclk_en\n");
		return -ENODEV;
	}
	gpio_direction_output(cadiz_sysclk_en_gpio, 1);

        if (gpio_request(lcm_bl_en_gpio, "lcm_bl_en_gpio")) {
                pr_err("failed to requeset cadiz_sysclk_en\n");
                return -ENODEV;
        }
        gpio_direction_output(lcm_bl_en_gpio, 1);

	if (gpio_request(cadiz_pwr_en_gpio, "cadiz_pwr_en")) {
		pr_err("failed to requeset cadiz_pwr_en\n");
		return -ENODEV;
	}
	gpio_direction_output(cadiz_pwr_en_gpio, 1);

	if (gpio_request(lcm_vhigh_gpio, "lcm_vhigh_gpio")) {
		pr_err("failed to requeset lcm_vhigh_gpio\n");
		return -ENODEV;
	}
	gpio_direction_output(lcm_vhigh_gpio, 1);
	//set boot command

	switch(asus_panel_id){
		case BOE:
                        pr_info("=BOE Panel command set=\n");
                        boot1 = boot1_boe;
                        boot2 = boot2_boe;
                        boot3 = boot3_boe;
                        boot4 = boot4_boe;
                        ipc_ibc = ipc_ibc_boe;
                        len_boot1 = sizeof(boot1_boe) / sizeof(boot1_boe[0]);
                        len_boot2 = sizeof(boot2_boe) / sizeof(boot2_boe[0]);
                        len_boot3 = sizeof(boot3_boe) / sizeof(boot3_boe[0]);
                        len_boot4 = sizeof(boot4_boe) / sizeof(boot4_boe[0]);
                        len_ipc_ibc = sizeof(ipc_ibc_boe) / sizeof(ipc_ibc_boe[0]);
                        break;
                case CPT:
                        pr_info("=CPT Panel command set=\n");
                        boot1 = boot1_cpt;
                        boot2 = boot2_cpt;
                        boot3 = boot3_cpt;
                        boot4 = boot4_cpt;
                        ipc_ibc = ipc_ibc_cpt;
                        len_boot1 = sizeof(boot1_cpt) / sizeof(boot1_cpt[0]);
                        len_boot2 = sizeof(boot2_cpt) / sizeof(boot2_cpt[0]);
                        len_boot3 = sizeof(boot3_cpt) / sizeof(boot3_cpt[0]);
                        len_boot4 = sizeof(boot4_cpt) / sizeof(boot4_cpt[0]);
                        len_ipc_ibc = sizeof(ipc_ibc_cpt) / sizeof(ipc_ibc_cpt[0]);
                        break;
        }
        //Avoid below pointers null as bringing up from bootloader
	p_boot1   = boot1;
	p_boot2   = boot2;
	p_boot3   = boot3;
	p_boot4   = boot4;
	p_ipc_ibc = ipc_ibc;

        rockchip_disp_init();
	INIT_WORK(&set_brightness_work, cadiz_set_brightness);

#if SUPPORT_IOCONTROL
	cadiz_dev.minor = MISC_DYNAMIC_MINOR;
	cadiz_dev.name = "cadiz";
	cadiz_dev.fops = &cadiz_dev_fops;
	if(misc_register(&cadiz_dev)) {
		dev_err(&client->dev, "%s:fail to register misc device\n", __func__);
	}
#endif

#if SUPPORT_REG_READ_WRITE
	struct dentry *dent = debugfs_create_dir("cadiz", NULL);
	debugfs_create_file("write_reg", S_IRUGO | S_IWUSR, dent, NULL, &dbg_write_reg_fops);
	debugfs_create_file("read_reg", S_IRUGO | S_IWUSR, dent, NULL, &dbg_read_reg_fops);
	debugfs_create_u32("enable_cabc_selection_function", S_IRUGO | S_IWUSR , dent, &enable_cabc_selection_function);
	debugfs_create_u32("disable_cabc", S_IRUGO | S_IWUSR , dent, &disable_cabc);
	debugfs_create_u32("enable_log", S_IRUGO | S_IWUSR , dent, &enable_log);
#endif
	pr_info("%s ----------------------------------------------------------\n", __func__);
	return 0;
}

static int cadiz_i2c_remove(struct i2c_client *client)
{
        return 0;
}

#ifdef CONFIG_OF
static struct of_device_id cadiz_match_table[] = {
    { .compatible = "sony_cadiz,cxd4756gf",},
    { },
};
#endif

static const struct i2c_device_id cadiz_id[] = {
       {"cxd4756gf", 0},
       {}
};
MODULE_DEVICE_TABLE(i2c, cadiz_id);

static struct i2c_driver cadiz_i2c_driver = {
       .driver = {
               .name = "cxd4756gf",
               .owner  = THIS_MODULE,
#ifdef CONFIG_OF
               .of_match_table = cadiz_match_table,
#endif
       },
       .id_table = cadiz_id,
       .probe = cadiz_i2c_probe,
       .remove = cadiz_i2c_remove,
};

int __init cadiz_i2c_init(void)
{
	pr_info("%s\n", __func__);
	return i2c_add_driver(&cadiz_i2c_driver);
}

void __exit cadiz_i2c_exit(void)
{
        i2c_del_driver(&cadiz_i2c_driver);
        rockchip_disp_exit();
}

arch_initcall(cadiz_i2c_init);
module_exit(cadiz_i2c_exit);
