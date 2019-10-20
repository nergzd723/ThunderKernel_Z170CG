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
 * GNU General #include "fixed_pcbid.h"Public License for more details.
 */

#include <linux/printk.h>
#include <linux/string.h>
#include <linux/gpio.h>
//#include <linux/mfd/intel_mid_pmic.h>
#include <linux/debugfs.h>
#include <asm/intel_scu_ipc.h>
//#include <asm/intel_scu_pmic.h>

#include "boardinfo.h"

#define  GPIO_RF_ID0 49
#define  GPIO_RF_ID1 61

#define  GPIO_MAIN_CAM 64

#define  GPIO_SUB_CAM 65

#define  GPIO_TP_ID0 66
#define  GPIO_TP_ID1 69

#define  GPIO_LCM_0 62
#define  GPIO_LCM_1 59

#define  GPIO_PROJECT_ID_0 57
#define  GPIO_PROJECT_ID_1 53
#define  GPIO_PROJECT_ID_2 34

#define  GPIO_HARDWARE_ID_0 55
#define  GPIO_HARDWARE_ID_1 48






static func_handle table[FUN_ID_MAX];

static int hardware_id = -1;
static int project_id = -1;
static int lcm_id = -1;
static int tp_id = -1;
static int sub_cam_id = -1;
static int main_cam_id = -1;
static int camera_rear_id = -1;
static int rf_sku_id = -1;
static int sim_id = -1;
static int dram_id = -1;
static int emmc_id = -1;
static int sku_id = -1;
int rc;


int get_gpio(unsigned int pin)
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
/*
int get_pmic_gpio(unsigned int pin)
{
	int iov = 0;	
	iov = intel_mid_pmic_readb(pin);
	printk("PMIC GPIO %x := %x  %d\n", pin, iov, iov);
	return (iov & 0x01);
}
*/

int get_hw_rev(void) //OK
{
	int ret = -1;
	int tmp = 0;
	tmp = get_gpio(GPIO_HARDWARE_ID_1);
	//rc = intel_scu_ipc_ioread8(0x35, &tmp);
	hardware_id = (tmp<<1);
	tmp = get_gpio(GPIO_HARDWARE_ID_0);
	//rc = intel_scu_ipc_ioread8(0x38, &tmp);
	hardware_id |= tmp;
	switch (hardware_id) {
		case 0:
			return SR;
		case 1:
			return ER;
		case 2:
			return ER2;
                case 3:
                        return PR;
		default:
			return UNKNOWN;
	}
	//printk("Get the Hardware ID \n");
	return ret;
}

/*
#define  GPIO_PROJECT_ID_0 57
#define  GPIO_PROJECT_ID_1 53
#define  GPIO_PROJECT_ID_2 34
*/
int get_project_id(void) //OK
{
	int ret = -1;
	int tmp =0;
	//printk("Get the project ID \n");
        tmp = get_gpio(GPIO_PROJECT_ID_2);
        project_id = (tmp<<2);
        tmp = get_gpio(GPIO_PROJECT_ID_1);
        project_id |= (tmp<<1);
	tmp = get_gpio(GPIO_PROJECT_ID_0);
	project_id |=tmp;
	switch (project_id) {
		case 0:
			return Z170CG_DSV;
		case 1:
			return Z170CG_SSV;
		case 2:
			return Z170CG_SSNV;
		case 3:
                        return Z170C_WIFI;	
		default:
			return UNKNOWN;
	}
	//printk("Get the project ID \n");
	return ret;
}

/*
#define  GPIO_LCM_0 62
#define  GPIO_LCM_1 59
*/
int get_lcm_id(void) //OK
{
	int ret = -1;
	int tmp =0;
	//printk("Get the lcm ID \n");
	tmp = get_gpio(GPIO_LCM_1);
	lcm_id = (tmp <<1);
	tmp = get_gpio(GPIO_LCM_0);
	lcm_id |= tmp;
	switch (lcm_id) {
		case 0:
			return KD;
		case 1:
			return BOE;
		case 2:
                        return BOE_FOCAL;
		case 3:
                        return HH_HIMAX;
		default:
		        return UNKNOWN;
	}
	//printk("Get the lcm ID \n");
	return ret;
}


/*
#define  GPIO_TP_ID0 66
#define  GPIO_TP_ID1 69
*/
int get_tp_id(void) //OK
{
	int ret = -1;
	int tmp=0;

	//printk("Get the tp ID \n");
        tmp = get_gpio(GPIO_TP_ID1);
        tp_id = (tmp<<1);
        tmp = get_gpio(GPIO_TP_ID0);
        tp_id |= tmp;
	switch (tp_id) {
	        default:
                        return UNKNOWN;
	}
	//printk("Get the tp ID \n");
	return ret;
}
/*
Sub CAM
#define  GPIO_SUB_CAM 65
*/

int get_sub_cam_id(void) //NO
{
	int ret = -1;
	//printk("Get the camera ID \n");
        sub_cam_id=get_gpio(GPIO_SUB_CAM);	
	switch (sub_cam_id) {
		case 0:
			return MGC;	
		default:
			return UNKNOWN;
	}
	//printk("Get the camera ID \n");
	return ret;
}

/*
#define  GPIO_MAIN_CAM 64
*/


int get_main_cam_id(void) //??
{
	int ret = -1;
	//printk("Get the camera front ID \n");
	main_cam_id = get_gpio(GPIO_MAIN_CAM );
	switch (main_cam_id) {
		case 0:
			return MOV;
		case 1: 
			return MHM;
		default:
			return UNKNOWN;
	}
	//printk("Get the camera front ID \n");
	return ret;
}

int get_camera_rear_id(void) 
{
	int ret = -1;

	//printk("Get the camera rear ID \n");
	camera_rear_id = get_gpio(67);
	
	switch (camera_rear_id) {
		default:
			return UNKNOWN;
	}
	//printk("Get the camera rear ID \n");
	return ret;
}


/*
#define  GPIO_RF_ID0 49
#define  GPIO_RF_ID1 61
*/

int get_rf_sku_id(void) 
{
	int ret = -1;
        int tmp=0;
	
	printk("Get the RF ID \n");

        tmp = get_gpio(GPIO_RF_ID1);
        rf_sku_id = (tmp<<1);
        tmp = get_gpio(GPIO_RF_ID0);
        rf_sku_id |= tmp;
	switch (rf_sku_id) {
		case 0:
			return G850T;
		case 1:
			return G900T;
                case 2:
                        return G850_WC;
		default:
			return UNKNOWN;
	}
	printk("Get the RF ID \n");
	return ret;
}

int get_sim_id(void) //NO
{
	int ret = -1;
	
	printk("Get the SIM ID \n");

	switch (sim_id) {
		default:
			return UNKNOWN;
	}
	printk("Get the SIM ID \n");
	return ret;
}
/*
int get_dram_id(void) //No
{
	int ret = -1;
	int tmp = 0;
	
	tmp = get_pmic_gpio(0x47);
	//rc = intel_scu_ipc_ioread8(0x3F, &tmp);
	dram_id = (tmp<<2);
	tmp = get_pmic_gpio(0x48);
	//rc = intel_scu_ipc_ioread8(0x40, &tmp);
	dram_id |= (tmp<<1);
	tmp = get_pmic_gpio(0x4A);
	//rc = intel_scu_ipc_ioread8(0x42, &tmp);
	dram_id |= tmp;
	
	switch (dram_id) {
		default:
			return UNKNOWN;
	}
	printk("Get the DRAM ID \n");
	return ret;
}
*/
/*
int get_emmc_id(void) //OK
{
	int ret = -1;
	
	emmc_id = get_pmic_gpio(0x49);
	//rc = intel_scu_ipc_ioread8(0x41, &emmc_id);
	
	switch (emmc_id) {
			return UNKNOWN;
	}
	printk("Get the eMMC ID \n");
	return ret;
}
*/
int get_sku_id(void) //No
{
	int ret = -1;
	printk("Get the SKU ID \n");	
	sku_id = get_gpio(119);
	
	switch (sku_id) {
		default:
			return UNKNOWN;
	}
	printk("Get the SKU ID \n");
	return ret;
}


// Create some debug nodes at /d/hardware/
void PCBID_add_dvfs(void)
{
	struct dentry *rul;
	struct dentry *debugfs_root;
	
	debugfs_root = debugfs_create_dir("hardware", NULL);
	//printk("Debugnode:%s\n",__FUNCTION__);
	
	rul = debugfs_create_u32("hardware_id", 0644, debugfs_root, &hardware_id);
	//printk("%s:hardware_id\n",__FUNCTION__);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("project_id", 0644, debugfs_root, &project_id);
	//printk("%s:project_id\n",__FUNCTION__);
	if (!rul)
		goto Fail;
	rul = debugfs_create_u32("lcm_id", 0644, debugfs_root, &lcm_id);
	//printk("%s:lcm_id\n",__FUNCTION__);
	if (!rul)
		goto Fail;
	//printk("%s:tp_id\n",__FUNCTION__);
	rul = debugfs_create_u32("tp_id", 0644, debugfs_root, &tp_id);
	if (!rul)
		goto Fail;
	//printk("%s:sub_cam_id\n",__FUNCTION__);
	rul = debugfs_create_u32("sub_cam_id", 0644, debugfs_root, &sub_cam_id);
	if (!rul)
		goto Fail;
	//printk("%s:main_cam_id\n",__FUNCTION__);
	rul = debugfs_create_u32("main_cam_id", 0644, debugfs_root, &main_cam_id);
	if (!rul)
		goto Fail;

  //      rul = debugfs_create_u32("camera_rear_id", 0644, debugfs_root, &camera_rear_id);
  //      if (!rul)
  //              goto Fail;

        rul = debugfs_create_u32("rf_sku_id", 0644, debugfs_root, &rf_sku_id);
	printk("%s:rf_sku_id\n",__FUNCTION__);
	if (!rul)
                goto Fail;

  //      rul = debugfs_create_u32("sim_id", 0644, debugfs_root, &sim_id);
  //      if (!rul)
  //              goto Fail;

//	rul = debugfs_create_u32("dram_id", 0644, debugfs_root, &dram_id);
//	if (!rul)
//		goto Fail;
//	rul = debugfs_create_u32("emmc_id", 0644, debugfs_root, &emmc_id);
//	if (!rul)
//		goto Fail;
  //	rul = debugfs_create_u32("sku_id", 0644, debugfs_root, &sku_id);
  //	if (!rul)
  //		goto Fail;
	
	return;
	
Fail:
	debugfs_remove_recursive(debugfs_root);
	debugfs_root = NULL;
}

int boardinfo_init()
{
	int i;
	
	memset(&table, 0, sizeof(table));
	
	ADD_FUNC(table,FUN_HARDWARE_ID,"To get the hardware revision",get_hw_rev);
	ADD_FUNC(table,FUN_PROJECT_ID,"To get the project id",get_project_id);
	ADD_FUNC(table,FUN_LCM_ID,"To get the lcm id",get_lcm_id);
	ADD_FUNC(table,FUN_TOUCH_ID,"To get the touch id",get_tp_id);
	ADD_FUNC(table,FUN_SUB_CAM_ID,"To get the sub camera id",get_sub_cam_id);
	ADD_FUNC(table,FUN_MAIN_CAM_ID,"To get the main camera id",get_main_cam_id);
// 	ADD_FUNC(table,FUN_MAIN_CAM_ID,"To get the camera rear id",get_camera_rear_id);
 	ADD_FUNC(table,FUN_RF_SKU_ID,"To get the rf sku id",get_rf_sku_id);
//	ADD_FUNC(table,FUN_DRAM_ID,"To get the dram id",get_dram_id);
//	ADD_FUNC(table,FUN_EMMC_ID,"To get the emmc id",get_emmc_id);
//	ADD_FUNC(table,FUN_SKU_ID,"To get the sku id",get_sku_id);
//	ADD_FUNC(table,FUN_SIM_ID,"To get the sim id",get_sim_id);





	
	// Export these functions to let other driver to use
	for( i = 0 ; i < FUN_ID_MAX ; i++ ){

        printk("table[%d].func = %llu\n", i, table[i].func);
		if ( table[i].func != NULL || table[i].func != 0) {
			table[i].func();
        //printk("================construct table==================\n");
		}
	}
        PCBID_add_dvfs();
	//printk("================pass add device node==============\n");	
	return set_boardinfo_tab(table);
}

