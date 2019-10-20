/*
* Copyright (C) 2015 ASUSTek COMPUTER INC.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/leds.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/leds-xgold.h>
#include <linux/delay.h>
#include "../misc/sony_cxd4756gf.h"
#include <../arch/x86/platform/asustek/include/asustek_boardinfo.h>
#define CADIZ_LED_MODULE_NAME "cadiz-led"

extern int cadiz_trigger_brightness_work(u8 bl_level);

struct xgold_led_data *cadiz_led;
static int enable_func = 0;
static int disable_fsys = 1;

static int32_t cadiz_led_set_backlight(struct device *dev)
{
	struct xgold_led_data *led = dev_get_drvdata(dev);
        u8 bl_level;
        pr_debug("%s(%#x) -->\n", __func__, led->led_brightness);
	if(enable_func)
		iowrite32(disable_fsys ? 0x4000 : 0x4001, led->mmio);
        mdelay(10);
	mutex_lock(&led->lock);
        //mapping from 0 ~ 255 to 0 ~ 127
        bl_level = (u8)(led->led_brightness*127/255);
        pr_debug("%s: brightness level=%d, cadiz level=0x%x\n", __func__, led->led_brightness, bl_level);
        cadiz_trigger_brightness_work(bl_level);
        mutex_unlock(&led->lock);
        return 0;
}

void cadiz_led_disable_fsys(int disable){
        iowrite32(disable ? 0x4000 : 0x4001, cadiz_led->mmio);
}
EXPORT_SYMBOL(cadiz_led_disable_fsys);

static int32_t cadiz_led_probe(struct platform_device *pdev)
{
       struct xgold_led_data *led;
       struct device *dev = &pdev->dev;
       struct resource *fsys_res;
       int asus_hardware_id = asustek_boardinfo_get(FUN_HARDWARE_ID);
       asus_hardware_id = ER;
       pr_info("%s :asus_hardware_id = %d\n",__func__,asus_hardware_id);
       if(asus_hardware_id == SR){
              pr_info("%s : SR SKU cadiz Led don't probe\n",__func__);
              return 0;
       }
       pr_info("cadiz backlight driver probed\n");
       pr_info("%s +++++++++++++++++++++++++++++++++++++++++\n", __func__);

       led = devm_kzalloc(dev, sizeof(struct xgold_led_data), GFP_KERNEL);
       if (!led) {
               dev_err(dev, "not enough memory for driver data\n");
               return -ENOMEM;
       }
//       led->set_gpio = cadiz_set_gpio;

       led->set_backlight = cadiz_led_set_backlight;
       led->np = pdev->dev.of_node;
       dev_set_drvdata(dev, led);
       led->pdev = pdev;
        cadiz_led = led;
	fsys_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ag620-fsys");
	if (fsys_res) {
		dev_err(dev, "HW resources available\n");
		led->physio = fsys_res->start;
		led->mmio = devm_ioremap(dev, fsys_res->start,
				resource_size(fsys_res));
		if (!led->mmio) {
			dev_err(dev, "IO remap operation failed\n");
			return -ENODEV;
		}
	} else
		dev_err(dev, "no HW resources available\n");

	struct dentry *dent = debugfs_create_dir("cadiz-led", NULL);
	debugfs_create_u32("enable_func", S_IRUGO | S_IWUSR , dent, &enable_func);
	debugfs_create_u32("disable_fsys", S_IRUGO | S_IWUSR , dent, &disable_fsys);
       pr_info("%s ----------------------------------------------------------\n", __func__);
       return xgold_led_probe(pdev);
}


static const struct of_device_id cadiz_led_of_match[] = {
	{
		.compatible = "asus,cadiz-led",
	},
	{},
};

static int cadiz_led_remove(struct platform_device *pdev)
{
               return xgold_led_remove(pdev);
}

static struct platform_driver cadiz_led_driver = {
       .driver = {
               .name = CADIZ_LED_MODULE_NAME,
               .owner = THIS_MODULE,
               .pm = &xgold_led_pm,
		.of_match_table = cadiz_led_of_match,
       },
       .probe = cadiz_led_probe,
       .remove = cadiz_led_remove,
};

static int __init cadiz_led_init(void)
{
       return platform_driver_register(&cadiz_led_driver);
}

static void __exit cadiz_led_exit(void)
{
       platform_driver_unregister(&cadiz_led_driver);
}

module_init(cadiz_led_init);
module_exit(cadiz_led_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Javid Shu <javid_shu@asus.com>");
MODULE_DESCRIPTION("Cadiz backlight driver");
MODULE_DEVICE_TABLE(of, cadiz_led_of_match);
