#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include "splendid.h"
#include <linux/rockchip_fb.h>

static struct miscdevice splendid_dev;
extern struct rockchip_vop_driver *rockchip_dev;

extern bool mipi_dsi_state;
static long splendid_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	u32 *dsp_lut = NULL;
	size_t size = 256 * 4;
	splendid_gamma_settings gamma;
	splendid_bcsh_settings bcsh;
	int brightness, contrast, sat_con, sin_hue, cos_hue;
	int i = 0, ret =0;

        if (!mipi_dsi_state) {
                printk("[DISPLAY] %s: skip, mipi dsi disable\n", __func__);
                return 0;
        }
	switch(cmd) {
		case SPLENDID_IOCTL_SET_GAMMA:
			printk("SPLENDID_IOCTL_SET_GAMMA\n");
			dsp_lut = devm_kzalloc(rockchip_dev->dev, size, GFP_KERNEL);
			if (!dsp_lut)
				return 0;

			if (copy_from_user(&gamma, (void __user *)arg,
					sizeof(gamma))) {
				printk("%s get arg error\n", __func__);
				return -EFAULT;
			}

			for (i = 0; i < 256; i++) {
				dsp_lut[i] = gamma.lut[i];
			}

			if (rockchip_dev->ops->set_dsp_lut)
				rockchip_dev->ops->set_dsp_lut(rockchip_dev, dsp_lut, 1);
			else
				printk("set_dsp_lut not found\n");
			break;
		case SPLENDID_IOCTL_SET_BCSH:
			printk("SPLENDID_IOCTL_SET_BCSH\n");

			if (copy_from_user(&bcsh, (void __user *)arg,
					sizeof(bcsh))) {
				printk("%s get arg error\n", __func__);
				return -EFAULT;
			}

			if (!strncmp(bcsh.cmd, "open", 4)) {
				if (rockchip_dev->ops->open_bcsh)
					ret = rockchip_dev->ops->open_bcsh(rockchip_dev, 1);
				else
					ret = -1;
			} else if (!strncmp(bcsh.cmd, "close", 5)) {
				if (rockchip_dev->ops->open_bcsh)
					ret = rockchip_dev->ops->open_bcsh(rockchip_dev, 0);
				else
					ret = -1;
			} else if (!strncmp(bcsh.cmd, "brightness", 10)) {
				ret = sscanf(bcsh.cmd, "brightness %d", &brightness);
				if (unlikely(brightness > 255)) {
					printk("brightness should be [0:255],now=%d\n\n", brightness);
					brightness = 255;
				}
				if (rockchip_dev->ops->set_dsp_bcsh_bcs)
					ret = rockchip_dev->ops->set_dsp_bcsh_bcs(rockchip_dev,
									     BRIGHTNESS,
									     brightness);
				else
					ret = -1;
			} else if (!strncmp(bcsh.cmd, "contrast", 8)) {
				ret = sscanf(bcsh.cmd, "contrast %d", &contrast);
				if (unlikely(contrast > 510)) {
					printk("contrast should be [0:510],now=%d\n", contrast);
					contrast = 510;
				}
				if (rockchip_dev->ops->set_dsp_bcsh_bcs)
					ret = rockchip_dev->ops->set_dsp_bcsh_bcs(rockchip_dev,
									     CONTRAST,
									     contrast);
				else
					ret = -1;
			} else if (!strncmp(bcsh.cmd, "sat_con", 7)) {
				ret = sscanf(bcsh.cmd, "sat_con %d", &sat_con);
				if (unlikely(sat_con > 1015)) {
					printk("sat_con should be [0:1015],now=%d\n", sat_con);
					sat_con = 1015;
				}
				if (rockchip_dev->ops->set_dsp_bcsh_bcs)
					ret = rockchip_dev->ops->set_dsp_bcsh_bcs(rockchip_dev,
									     SAT_CON, sat_con);
				else
					ret = -1;
			} else if (!strncmp(bcsh.cmd, "hue", 3)) {
				ret = sscanf(bcsh.cmd, "hue %d %d", &sin_hue, &cos_hue);
				if (unlikely(sin_hue > 511 || cos_hue > 511)) {
					printk("sin_hue=%d,cos_hue=%d\n", sin_hue, cos_hue);
				}
				if (rockchip_dev->ops->set_dsp_bcsh_hue)
					ret = rockchip_dev->ops->set_dsp_bcsh_hue(rockchip_dev,
									     sin_hue, cos_hue);
				else
					ret = -1;
			} else {
				printk("%s: SPLENDID_IOCTL_SET_BCSH format error\n", __func__);
			}

			break;
		default:
			printk("%s:unknown cmd=%d\n", __func__, cmd);
			return -EINVAL;
	}
	return 0;
}

static int splendid_open(struct inode *inode, struct file *filp)
{

        if (!mipi_dsi_state) {
                printk("[DISPLAY] %s: skip, mipi dsi disable\n", __func__);
                return 0;
        }

	printk("%s\n", __func__);
	return nonseekable_open(inode, filp);
}

static const struct file_operations splendid_fops = {
	.owner = THIS_MODULE,
	.open  = splendid_open,
	.unlocked_ioctl = splendid_ioctl,
};

static int __init splendid_init(struct platform_device *pdev)
{
	int ret = 0;

	splendid_dev.minor = MISC_DYNAMIC_MINOR;
	splendid_dev.name = "splendid";
	splendid_dev.fops = &splendid_fops;
	if(misc_register(&splendid_dev)) {
		printk("%s:fail to register misc device\n", __func__);
	}

	return ret;
}

static void __exit splendid_exit(struct platform_device *pdev)
{
	return;
}

module_init(splendid_init);
module_exit(splendid_exit);

MODULE_DESCRIPTION("splendid");
MODULE_LICENSE("GPL v2");
