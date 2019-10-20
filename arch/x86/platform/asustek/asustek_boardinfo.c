/*
 * Copyright (C) 2014 ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/string.h>
#include <asm/intel-mid.h>

#include "include/asustek_boardinfo.h"
#include "boardinfo.h"

static bool flush;
func_handle *gfuns = NULL;

static int hardware_id;

struct id_stuffing {
	int value;
	bool flash_en;
	idfuntion_handler func;
};

int set_boardinfo_tab(func_handle *handler_table) {
	int ret = -1;
	if (handler_table != NULL) {
        printk("get handler_table\n");
		
		gfuns = handler_table;
	printk("pass get handler_table\n");

		ret = 0;
	}
	return ret;
}

//-------------------- Tool function end----------------
void start_handle(func_handle *handler_table)
{
	// TODO: get all boardinfo IDs on probe
}

//-------------------- Export function ------------------------
int asustek_boardinfo_get(int fun_num)
{
		printk("enter asustek_boardinfo_get==\n");
	if ((fun_num > (FUN_ID_MAX - 1)) || (fun_num < 0)) {
		//printk("asustek b error\n");
		pr_err("ASUSTek: Error Function number\n");
		return -1;
	}

	if (flush && (gfuns[fun_num].func != NULL)) {
		printk("asustek_boardinfo_get==\n");
		gfuns[fun_num].value = gfuns[fun_num].func();
 		printk("asustek_boardinfo_get\n");
		return gfuns[fun_num].value;
	}

	return gfuns[fun_num].value;
}


//-------------------- For ATTR ---------------------------

#define ASUSTEK_PCBID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}


#define ASUSTEK_HARDWAREID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

#define ASUSTEK_RFID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}

#define ASUSTEK_PROJECTID_ATTR(module) \
static struct kobj_attribute module##_attr = { \
	.attr = { \
		.name = __stringify(module), \
		.mode = 0444, \
	}, \
	.show = module##_show, \
}




static ssize_t asustek_boardinfo_store(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	// TODO: for debug file node

	return s - buf;
}

static ssize_t asustek_boardinfo_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	// TODO: for debug file node

	return s - buf;
}


static ssize_t asustek_hardwareid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{	
	char *s = buf;

	printk("asustek_hardwareid_show\n");
	s += sprintf(buf, "%d\n", asustek_boardinfo_get(FUN_HARDWARE_ID));
	return s - buf;

}

static ssize_t asustek_rfid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	char *s = buf;
	printk("asustek_rfid_show\n");	
	s += sprintf(buf, "%d\n", asustek_boardinfo_get(FUN_RF_SKU_ID));
	return s - buf;
}

static ssize_t asustek_projectid_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{	
	char *s = buf;
	printk("asustek_projectid_show\n");
	s += sprintf(buf, "%d\n", asustek_boardinfo_get(FUN_PROJECT_ID));
	return s - buf;
}



ASUSTEK_PROJECTID_ATTR(asustek_projectid);
ASUSTEK_RFID_ATTR(asustek_rfid);
ASUSTEK_PCBID_ATTR(asustek_boardinfo);
ASUSTEK_HARDWAREID_ATTR(asustek_hardwareid);


static struct attribute *attr_list[] = {
	&asustek_boardinfo_attr.attr,
	&asustek_hardwareid_attr.attr,
	&asustek_projectid_attr.attr,
	&asustek_rfid_attr.attr,
	NULL,

};

static struct attribute_group attr_group = {
	// TODO: for debug file node
	.attrs = attr_list,
};

static int __init boardinfo_driver_probe(struct platform_device *pdev)
{
	int ret = 0;
	flush = true;
	if(boardinfo_init() < 0) {
		pr_err("ASUSTek: Init board!\n");
		return -1;
	}

	/* create a sysfs interface */
 
	printk("create a sysfs interface\n");
	ret = sysfs_create_group(&pdev->dev.kobj, &attr_group);
	printk("pass a sysfs interface\n");

	if (ret)
		pr_err("ASUSTek: Failed to create sysfs group\n");


	return ret;
}

static int boardinfo_driver_remove(struct platform_device *pdev)
{
	printk("boardinfo_driver_remove\n");
	return 0;
}

static struct platform_driver asustek_boardinfo_driver __refdata = {
	.probe = boardinfo_driver_probe,
	.remove = boardinfo_driver_remove,
	.driver = {
		.name = "asustek_boardinfo",
		.owner = THIS_MODULE,
	},
};

static int asustek_boardinfo_init(void)
{
	printk("asustek_boardinfo_init\n");
	return platform_driver_register(&asustek_boardinfo_driver);
}

/*
#if defined(CONFIG_TF103C) || defined(CONFIG_TF103CE)
module_init(asustek_boardinfo_init);
#else
rootfs_initcall(asustek_boardinfo_init);
#endif
*/
#if defined(CONFIG_Z170CG) || defined(CONFIG_Z370CG) ||  defined(CONFIG_Z170C) || defined(CONFIG_Z370C)
subsys_initcall(asustek_boardinfo_init);
#else
rootfs_initcall(asustek_boardinfo_init);
#endif

//module_init(asustek_boardinfo_init);

MODULE_DESCRIPTION("ASUSTek BoardInfo driver");
MODULE_AUTHOR("Jupiter Chen <Jupiter_Chen@asus.com>");
MODULE_LICENSE("GPL");

