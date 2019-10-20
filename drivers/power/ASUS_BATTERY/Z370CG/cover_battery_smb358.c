/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 */
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
//#include <linux/power/smb347-asus-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>

#include "smb345_external_include.h"
#include "asus_battery.h"
#include "../../../../arch/x86/platform/asustek/include/asustek_boardinfo.h"
#include "../../../misc/smart_cover.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
//#include "../../intel_mdf_charger.h"
//#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
//#include "battery_i2c_stress_test.h"
//#include "hwid.h"
//#include "asustek_boardinfo.h"

 /**/
#include <linux/workqueue.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/pinctrl/consumer.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_controller.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_bus.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/usb/phy-intel.h>
#include <linux/time.h>
     /**/
#define TF103CE_CHARGER_ACPI                    0
#if TF103CE_CHARGER_ACPI
#include <linux/acpi.h>
#endif
static int HW_ID;
static int PROJECT_ID;
//extern int Read_HW_ID(void);
//extern int smb358_entry_mode;
int smb358_entry_mode = 1;

/*
 * usb notify callback
 */
#define USB_NOTIFY_CALLBACK

static bool ischargerSuspend = false;
static bool isUSBSuspendNotify = false;

/*************
 *** Cover ***
 *************/
#define AUDIO_SLEEVE_PR
static int g_mult_i2c_en = 15;
//extern int mult_function_n;
extern int g_cover_otg_flag;

/*****************/
/***smb register**/
/*****************/

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41

#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1000 0x30
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define CFG_VARIOUS_FUNCS            0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB        BIT(2)
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE    BIT(4)
#define CFG_VARIOUS_FUNCS_BATTERY_OV    BIT(1)
#define CFG_FLOAT_VOLTAGE            0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK    0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT    6
#define CFG_STAT                0x05
#define CFG_STAT_DISABLED            BIT(5)
#define CFG_STAT_ACTIVE_HIGH            BIT(7)
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
#define CFG_THERM                0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT    0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK    0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT    2
#define CFG_THERM_MONITOR_DISABLED        BIT(4)
#define CFG_SYSOK                0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED    BIT(2)
#define CFG_OTHER                0x09
#define CFG_OTHER_RID_MASK            0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN        0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C        0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG        0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW        BIT(5)
#define CFG_OTG                    0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK        0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT        4
#define CFG_OTG_CC_COMPENSATION_MASK        0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT        6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK    0x03
#define CFG_TEMP_LIMIT                0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK        0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT        0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK        0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT        2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK        0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT        4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK        0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT        6
#define CFG_FAULT_IRQ                0x0c
#define CFG_FAULT_IRQ_DCIN_UV            BIT(2)
#define CFG_FAULT_IRQ_OTG_UV            BIT(5)
#define CFG_STATUS_IRQ                0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT        BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER    BIT(4)
#define CFG_ADDRESS                0x0e
#define smb358_STAT_CTRL0_ADDR             0x00U
/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

#define STATUS_UPDATE_INTERVAL            (HZ * 60)

/*Charge control register*/
#define CHARGE_CONTROL_REG 0x04
#define AUTO_RECHARGE BIT(7)
#define ALLOWED_EOC BIT(6)
#define AUTO_DET BIT(2)

/*****************/
/*****************/
/*****************/

#define DEFAULT_CC 350

#define REG_IC_INFO 0x00
#define REG_CHARGE_CTRL1 0x01
#define REG_CHARGE_CTRL2 0x02
#define REG_IBAT 0x03
#define REG_VOREG 0x04
#define REG_IBUS 0x05
#define REG_INT 0x06
#define REG_STATUS 0x07
#define REG_INT_MASK 0x08
#define REG_ST_MASK 0x09
#define REG_TMR_RST 0x0a
#define REG_SAFETY 0x0f
#define REG_MONITOR 0x10
#define REG_STATE 0x1f
#define REG_ADP_CTRL 0x20
#define REG_ADP_CNT 0x21
#define REG_TMR_CTRL 0x22

/* REG_IC_INFO */
#define VENDOR 0x4
#define VENDOR_O 5
#define VENDOR_M 0x7

#define PN 0x1
#define PN_O 3
#define PN_M 0x3

#define REV_O 0
#define REV_M 0x3

/* REG_CHARGE_CTRL1 */
#define HZ_MODE_O 6
#define HZ_MODE_M 0x1

/* REG_CHARGE_CTRL2 */
#define ITERM_DIS_M 0x1
#define ITERM_DIS_O 0
#define LDO_OFF_O 4
#define BOOST_UP 5
#define BOOST_EN 6

/* REG_IBAT */
#define IOCHARGE_MIN_MA 350
#define IOCHARGE_STEP_MA 100
#define IOCHARGE_STEP_START_MA 400
#define IOCHARGE_STEP_START_REGVAL 1
#define IOCHARGE_MAX_MA 1500
#define IOCHARGE_O 4
#define IOCHARGE_M 0xf

#define DEFAULT_CC 350

/* REG_VOREG  */
#define VOREG_MIN_MV 3380
#define VOREG_STEP_MV 20
#define VOREG_MAX_MV 4440
#define VOREG_M 0x3f

#define DEFAULT_CV 3380

/* REG_IBUS */
#define IBUS_MIN_LIMIT_MA 100
#define IBUS_LIMIT_STEP_MA 400
#define IBUS_MAX_LIMIT_MA 900
#define IBUS_M 0x3
#define IBUS_O 0
#define IBUS_NO_LIMIT 3

/* REG_INT and REG_INT_MASK */
#define TSD_FLAG_O 7
#define OVP_FLAG_O 6
#define OVP_RECOV_O 1
#define BOOSTOV_O 5
#define TC_TO_O 4
#define BAT_UV_O 3
#define DBP_TO_O 3
#define TREG_FLAG_O 5
#define OT_RECOV_O 2
#define INT_MASK_ALL 0xff

/* REG_STATUS and REG_ST_MASK */
#define POK_B_O 6
#define ST_MASK_ALL 0xff

/* REG_TMR_RST */
#define TMR_RST_O 7

/* REG_SAFETY  */
#define ISAFE_O 4
#define VSAFE_O 0
#define FAN54020_CUR_LIMIT 0xf	/* 1500mA */
#define FAN54020_VOLT_LIMIT 0xf	/* 4440mV */

/* REG_STATE */
#define ST_CODE_O 4
#define ST_CODE_M 0xf
#define ST_VBUS_FAULT 0x1a
#define ST_VBUS_FAULT2 0x1d

/* PMU Register used for POK status */
#define PMU_CONFIG_O    0x00U
#define PMU_CONFIG(_base) ((_base) + PMU_CONFIG_O)

#define PMU_CONFIG_CHGDET_O (BIT(1))
#define PMU_CONFIG_CHGDET_M (BIT(1))

#define CHARGER_CONTROL_O 0x0
#define CHARGER_CONTROL(_base) ((_base) + CHARGER_CONTROL_O)
#define CHARGER_CONTROL_CIEDG_O 26
#define CHARGER_CONTROL_CIEDG_M 0x1
#define CHARGER_CONTROL_CILVL_O 25
#define CHARGER_CONTROL_CILVL_M 0x1
#define CHARGER_CONTROL_CISENS_O 24
#define CHARGER_CONTROL_CISENS_M 0x1
#define CHARGER_CONTROL_CIEN_O 23
#define CHARGER_CONTROL_CIEN_M 0x1
#define CHARGER_CONTROL_CIDBT_O 17
#define CHARGER_CONTROL_CIDBT_M 0x7
#define CHARGER_CONTROL_CHGLVL_O 1
#define CHARGER_CONTROL_CHGLVL_M 0x1
	/*chrg det */
#define CHARGER_CONTROL_CDETSENS_O 10
#define CHARGER_CONTROL_CDETSENS_M 0x1
#define CHARGER_CONTROL_CHDETLVL_O 6
#define CHARGER_CONTROL_CHDETLVL_M 0x1
#define CHARGER_CONTROL_CHDWEN_O 5
#define CHARGER_CONTROL_CHDWEN_M 0x1

#define CHARGER_CONTROL_CIEDG_FALLING 0
#define CHARGER_CONTROL_CILVL_LOW 0
#define CHARGER_CONTROL_CISENS_EDGE 1
#define CHARGER_CONTROL_CIEN_EN 0
#define CHARGER_CONTROL_CHGLVL_LOW 0
#define CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE 0
/*chrg det*/
#define CHARGER_CONTROL_CDETSENS_EDGE 1
#define CHARGER_CONTROL_CHDETLVL_LOW 0
#define CHARGER_CONTROL_CHDETLVL_HIGH 1
#define CHARGER_CONTROL_CHDWEN_EN 1

#define CHARGER_CONTROL_WR_O 0x8
#define CHARGER_CONTROL_WR(_base) ((_base) + CHARGER_CONTROL_WR_O)
#define CHARGER_CONTROL_WR_WS_O 0
#define CHARGER_CONTROL_WR_WS_M 0x1

#define MAX_NR_OF_I2C_RETRIES 1
#define CHRGR_WORK_DELAY (10*HZ)
#define BOOST_WORK_DELAY (10*HZ)
#define EVT_WAKELOCK_TIMEOUT (2*HZ)
#define EVENTS_LOG_FILENAME "events_log"
#define DBG_REGS_FILENAME "charger_regs"
#define DBG_STATE_FILENAME "charger_state"
#define LOG_LINE_LENGTH (64)
#define LINES_PER_PAGE (PAGE_SIZE/LOG_LINE_LENGTH)
#define SHADOW_REGS_NR 10

#define fit_in_range(__val, __MIN, __MAX) ((__val > __MAX) ? __MAX : \
					(__val < __MIN) ? __MIN : __val)

#define SYSFS_FAKE_VBUS_SUPPORT 1

#define SYSFS_INPUT_VAL_LEN (1)
enum smb358_chrgr_stat {
	smb358_CHRGR_STAT_UNKNOWN,
	smb358_CHRGR_STAT_READY,
	smb358_CHRGR_STAT_CHARGING,
	smb358_CHRGR_STAT_BAT_FULL,
	smb358_CHRGR_STAT_FAULT,
};

enum charger_status {
	smb358_STATUS_UNKNOWN,
	smb358_STATUS_READY,
	smb358_STATUS_FAULT,
};
struct smb358_state {
	enum charger_status status;
	int vbus;
	int cc;
	int max_cc;
	int cv;
	int iterm;
	int inlmt;
	int health;
	int cable_type;
	bool charger_enabled;
	bool charging_enabled;
	bool to_enable_boost;
	bool boost_enabled;
	unsigned int pok_b:1;
	unsigned int ovp_flag:1;
	unsigned int ovp_recov:1;
	unsigned int t32s_timer_expired:1;
	unsigned int vbus_fault:1;
	unsigned int treg_flag:1;
	unsigned int ot_recov_flag:1;
	unsigned int tsd_flag:1;

	/* Boost mode specific states */
	unsigned int bat_uv:1;
	unsigned int boost_ov:1;

};
enum {
	POK_B_VALID = 0,
	POK_B_INVAL,

	BOOSTOV_OCCURED = 1,

	BATUV_OCCURED = 1,

	TSD_OCCURRED = 1,

	OVP_OCCURRED = 1,

	OVP_RECOV_OCCURRED = 1,

	T32_TO_OCCURRED = 1,

	VBUS_FAULT = 1,

	TREG_IS_ON = 1,

	OT_RECOV_OCCURRED = 1,

	VBUS_OFF = 0,
	VBUS_ON,
};

//static int smb358_pre_config(void);
static void smb3xx_config_max_current(int);

int smb358_do_battery_jeita(int, int, int);
static int smb358_OptiCharge_Toggle(bool on);

static int battery_temp_zone = TEMP_15_50;
static int battery_fake_temp = 0xff;

struct smb358_otg_event {
	struct list_head node;
	bool param;
};
struct unfreezable_bh_struct {
	struct workqueue_struct *wq;
	struct work_struct work;
	struct wake_lock evt_wakelock;
	bool in_suspend;
	bool pending_evt;
	spinlock_t lock;
};
struct smb358_charger {
	struct mutex stat_lock;
	struct wake_lock evt_lock;
	struct wake_lock chrgr_en_wakelock;
	const char *model_name;
	const char *manufacturer;
	struct mutex lock;
	struct i2c_client *client;
	struct power_supply mains;
	struct power_supply usb;
	struct power_supply battery;
	struct power_supply pack_battery;
	bool mains_online;
	bool usb_online;
	bool charging_enabled;
	bool running;
	struct dentry *dentry;
	struct dentry *dentry2;
	struct otg_transceiver *otg;
	struct notifier_block otg_nb;
	struct work_struct otg_work;
	struct delayed_work aicl_dete_work;
	struct workqueue_struct *chrgr_work_queue;
	struct list_head otg_queue;
	spinlock_t otg_queue_lock;
	bool otg_enabled;
	bool otg_battery_uv;
	void __iomem *ctrl_io;	// For IDI
	void __iomem *pcfg_io;	// For IDI
	int fake_vbus;
	int chgint_irq;
	int chgdet_irq;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct unfreezable_bh_struct chgint_bh;
	struct unfreezable_bh_struct boost_op_bh;
	struct work_struct detect_irq_work;
	//      struct delayed_work detect_irq_work;
	struct work_struct interrupt_irq_work;
	 /**/ struct idi_peripheral_device *ididev;
	struct smb358_state state;
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;

	int vbus;
	int chg_otg_en_gpio;
	int cover_detect_gpio;
};

static int smb358_otg_notification_handler(struct notifier_block *nb,
					   unsigned long event, void *data);

static struct smb358_charger chrgr_data = {
	.model_name = "smb358",
	.manufacturer = "TBD",

	.otg_enabled = false,
	.otg_nb = {
		   .notifier_call = smb358_otg_notification_handler,
		   },

	.chgint_bh = {
		      .in_suspend = false,
		      .pending_evt = false,
		      },
	.boost_op_bh = {
			.in_suspend = false,
			.pending_evt = false,
			},
	.fake_vbus = -1,

	.state = {
		  .status = smb358_STATUS_UNKNOWN,
		  .vbus = -1,
		  .cc = 500,
		  .max_cc = 1500,
		  .cv = 3380,
		  .iterm = 0,
		  .health = POWER_SUPPLY_HEALTH_UNKNOWN,
		  .cable_type = POWER_SUPPLY_CHARGER_TYPE_NONE,
		  .charger_enabled = false,
		  .charging_enabled = true,	/* initially HZ mode is switched off */
		  .pok_b = 0,
		  .ovp_flag = 0,
		  .ovp_recov = 0,
		  .t32s_timer_expired = 0,
		  .vbus_fault = 0,
		  .treg_flag = 0,
		  .ot_recov_flag = 0,
		  .tsd_flag = 0,
		  },
};

static bool pad_pwr_supply(void);
struct wake_lock smb358_wlock;
struct wake_lock smb358_wlock_t;
//static struct smb347_charger *smb347_dev1;
//static int smb347_set_writable(struct smb347_charger *smb, bool writable);
//#define ASUS_ENG_BUILD
#ifdef ASUS_ENG_BUILD
bool smb358_eng_charging_limit2 = true;

int smb358_asus_charging_toggle_write2(struct file *file, const char *buffer,
				       size_t count, loff_t * data)
{
	BAT_DBG_E(" %s:\n", __func__);

	if (buffer[0] == '1') {
		/* turn on charging limit in eng mode */
		smb358_eng_charging_limit2 = true;
	} else if (buffer[0] == '0') {
		/* turn off charging limit in eng mode */
		smb358_eng_charging_limit2 = false;
	} else if (buffer[0] == '2') {
		/* disable charger */
		smb358_charging_toggle(JEITA, false);
	}

	smb358_aicl_dete_worker(NULL);

	return count;
}

static int asus_charging_toggle_read(struct seq_file *m, void *p)
{
	int len;

	BAT_DBG_E(" %s: smb358_eng_charging_limit2 = %s\n", __func__,
		  smb358_eng_charging_limit2 ? "true" : "false");
	seq_printf(m, "%d\n", smb358_eng_charging_limit2);

	return len;

}

static int asus_charging_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, asus_charging_toggle_read, NULL);
}

static const struct file_operations asus_eng_charging_limit_ops = {
	.open = asus_charging_toggle_open,
	.read = seq_read,
	.write = smb358_asus_charging_toggle_write2,
	.llseek = seq_lseek,
	.release = seq_release
};

int smb358_init_asus_charging_limit_toggle2(void)
{
	struct proc_dir_entry *entry = NULL;

	entry =
	    proc_create("driver/charger_limit_enable", 0666, NULL,
			&asus_eng_charging_limit_ops);
	if (!entry) {
		BAT_DBG_E("Unable to create asus_charging_toggle\n");
		return -EINVAL;
	}
	return 0;
}
#else
int smb358_init_asus_charging_limit_toggle2(void)
{
	return 0;
}
#endif

/* global charger type variable lock */
DEFINE_MUTEX(smb358_g_usb_state_lock);
static int g_usb_state = CABLE_OUT;
/* global software charging toggle lock */
DEFINE_MUTEX(smb358_g_charging_toggle_lock);

/* Check if cover is charging tablet */
int G_PACK_AC_ONLINE = 0;
EXPORT_SYMBOL(G_PACK_AC_ONLINE);

static bool g_charging_toggle = true;

static struct smb358_charger *smb358_dev;

static struct chgr_dev_func smb358_tbl;

/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
	300,
	500,
	700,
	1000,
	1200,
	1500,
	1800,
	2000,
	0,
};

/* Charge current compensation in uA */
static const unsigned int ccc_tbl[] = {
	250000,
	700000,
	900000,
	1200000,
};

#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"

#define smb358_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG            0x00
#define INPUT_CURRENT_LIMIT_REG    0x01
#define VAR_FUNC_REG            0x02
#define FLOAT_VOLTAGE_REG        0x03
#define CHG_CTRL_REG            0x04
#define STAT_TIMER_REG            0x05
#define PIN_ENABLE_CTRL_REG        0x06
#define THERM_CTRL_A_REG        0x07
#define SYSOK_USB3_SELECT_REG    0x08
#define OTHER_CTRL_A_REG        0x09
#define OTG_TLIM_THERM_CNTRL_REG                0x0A

#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG    0x0B
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK            smb358_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            smb358_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            smb358_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK            smb358_MASK(2, 6)

#define FAULT_INTERRUPT_REG        0x0C
#define STATUS_INTERRUPT_REG    0x0D
#define I2C_BUS_SLAVE_REG        0x0E	//chris: add
#define CMD_A_REG        0x30
#define CMD_B_REG        0x31
#define CMD_C_REG        0x33
#define INTERRUPT_A_REG        0x35
#define INTERRUPT_B_REG        0x36
#define INTERRUPT_C_REG        0x37
#define INTERRUPT_D_REG        0x38
#define INTERRUPT_E_REG        0x39
#define INTERRUPT_F_REG        0x3A
#define STATUS_A_REG    0x3B
#define STATUS_B_REG    0x3C
#define STATUS_C_REG    0x3D
#define STATUS_D_REG    0x3E
#define STATUS_E_REG    0x3F

/*APSD result*/
#define STATUS_D_APSD_CDP 0x01
#define STATUS_D_APSD_DCP 0x02
#define STATUS_D_APSD_SDP 0x04

/* Status bits and masks */
#define CHG_STATUS_MASK        smb358_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT        BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK            smb358_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK        smb358_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK            smb358_MASK(3, 5)
#define TERMINATION_CURRENT_MASK        smb358_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK    smb358_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK                smb358_MASK(6, 0)
#define CHG_ENABLE_BIT            BIT(1)
#define VOLATILE_W_PERM_BIT        BIT(7)
#define USB_SELECTION_BIT        BIT(1)
#define SYSTEM_FET_ENABLE_BIT    BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT    BIT(2)
#define BATT_OV_END_CHG_BIT        BIT(1)
#define VCHG_FUNCTION            BIT(0)
#define CURR_TERM_END_CHG_BIT    BIT(6)

#define OTGID_PIN_CONTROL_MASK    smb358_MASK(2, 6)
#define OTGID_PIN_CONTROL_BITS    BIT(6)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    smb358_MASK(2, 2)
#define SMB358_OTG_CURRENT_LIMIT_900mA    (BIT(2) | BIT(3))
#define SMB358_OTG_CURRENT_LIMIT_750mA    BIT(3)
#define SMB358_OTG_CURRENT_LIMIT_500mA    BIT(2)
#define SMB358_OTG_CURRENT_LIMIT_250mA    0
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    smb358_MASK(2, 0)

#define CHARGE_CURRENT_COMPENSATION         smb358_MASK(2, 6)
#define CHARGE_CURRENT_COMPENSATION_VALUE   0x00

#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS
#define SMB358_FAST_CHG_CURRENT_MASK            smb358_MASK(3, 5)
#define SMB358_TERMINATION_CURRENT_MASK         smb358_MASK(3, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_200mA    BIT(0) | BIT(1) | BIT(2)
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    BIT(5) | BIT(6) | BIT(7)
#define SMB358_FAST_CHG_CURRENT_VALUE_1500mA    BIT(5) | BIT(7)

static int smb346_soc_detect_batt_tempr(int usb_state);
static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);
static int smb358_vbus_enable(int toggle);

static char *supply_list[] = {
	"battery",
	"pack_bat",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb358_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val);

static struct power_supply smb358_power_supplies[] = {
	{
	 .name = "pack_ac",
	 .type = POWER_SUPPLY_TYPE_PACK_AC,
	 .supplied_to = supply_list,
	 .num_supplicants = ARRAY_SIZE(supply_list),
	 .properties = asus_power_properties,
	 .num_properties = ARRAY_SIZE(asus_power_properties),
	 .get_property = smb358_power_get_property,
	},
};

static int smb358_otg_notification_handler(struct notifier_block *nb,
					   unsigned long event, void *data)
{

	struct smb358_charger *chrgr = &chrgr_data;

	switch (event) {
	case INTEL_USB_DRV_VBUS:
		if (!data)
			return NOTIFY_BAD;
		chrgr->state.to_enable_boost = *((bool *) data);
		smb358_vbus_enable(chrgr->state.to_enable_boost);
		break;

	default:
		break;
	}
	return NOTIFY_OK;
}

static int smb358_power_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	int chrg_status;
	int cover_otg_flag;

	mutex_lock(&smb358_g_usb_state_lock);
	usb_state = g_usb_state;
	mutex_unlock(&smb358_g_usb_state_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (psy->type == POWER_SUPPLY_TYPE_PACK_AC) {
			if (!is_cover_attached()) {
				val->intval = 0;
				G_PACK_AC_ONLINE = 0;
			} else {
				if (get_cover_type() == POWER_BANK) {
					cover_otg_flag = (g_cover_otg_flag == NULL) ? 0 : g_cover_otg_flag;
					if (smb358_is_cover_otg() && cover_otg_flag == 0) {
						val->intval = 1;
						G_PACK_AC_ONLINE = 1;
					} else {
						val->intval = 0;
						G_PACK_AC_ONLINE = 0;
					}
				} else {
					val->intval = 0;
					G_PACK_AC_ONLINE = 0;
				}
			}
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

extern bool smb358_get_sw_charging_toggle()
{
	bool ret;

	mutex_lock(&smb358_g_charging_toggle_lock);
	ret = g_charging_toggle;
	mutex_unlock(&smb358_g_charging_toggle_lock);

	return ret;
}

extern int smb358_get_charger_type()
{
	int ret;

	if (!smb358_dev) {
		pr_err
		    ("%s Warning: smb358_dev is null due to probe function has error\n",
		     __func__);
		return -1;
	}

	mutex_lock(&smb358_g_usb_state_lock);
	ret = g_usb_state;
	mutex_unlock(&smb358_g_usb_state_lock);

	return ret;
}

int smb358_request_power_supply_changed()
{
	int ret;

	if (!smb358_dev) {
		pr_err
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return -1;
	}

//	power_supply_changed(&smb358_power_supplies[CHARGER_AC - 1]);
//	power_supply_changed(&smb358_power_supplies[CHARGER_USB - 1]);
	power_supply_changed(&smb358_power_supplies[0]);

	return ret;
}

static int smb358_register_power_supply(struct device *dev)
{
	int ret;

	ret =
	    power_supply_register(dev, &smb358_power_supplies[0]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply pack_ac\n");
		goto batt_err_reg_fail_usb;
	}
#if 0
	ret =
	    power_supply_register(dev, &smb358_power_supplies[CHARGER_USB - 1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply USB\n");
		goto batt_err_reg_fail_usb;
	}

	ret =
	    power_supply_register(dev, &smb358_power_supplies[CHARGER_AC - 1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}
#endif
	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb358_power_supplies[CHARGER_USB - 1]);
batt_err_reg_fail_usb:
	return ret;
}

static int smb358_read_reg(struct i2c_client *client, int reg, u8 * val,
			   int ifDebug)
{
	s32 ret;
	struct smb358_charger *smb358_chg;

	smb358_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb358_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb358_chg->client->dev,
			"i2c read fail: can't read from Reg%02Xh: %d\n", reg,
			ret);
		return ret;
	} else {
		*val = ret;
	}
	if (ifDebug)
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(*val));

	return 0;
}

static int smb358_write_reg(struct i2c_client *client, int reg, u8 val)
{
	s32 ret;
	struct smb358_charger *smb358_chg;

	smb358_chg = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(smb358_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb358_chg->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb358_masked_write(struct i2c_client *client, int reg, u8 mask,
			       u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb358_read_reg(client, reg, &temp, 0);
	if (rc) {
		pr_err("smb358_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	//printk("reg = %08x which val = %08x , will use mask = %08x\n", reg, temp, mask);

	temp &= ~mask;
	temp |= val & mask;
	//printk("after mask, reg = %08x will be wrote to val = %08x \n", reg, temp);
	rc = smb358_write_reg(client, reg, temp);
	if (rc) {
		pr_err("smb358_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb358_read(struct smb358_charger *smb, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_byte_data(smb->client, reg);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev,
				 "fail to read reg %02xh: %d\n", reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb358_write(struct smb358_charger *smb, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_write_byte_data(smb->client, reg, val);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev,
				 "fail to write reg %02xh: %d\n", reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb358_set_writable(struct smb358_charger *smb, bool writable)
{
	int ret;

	ret = smb358_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb358_write(smb, CMD_A, ret);
}

static int cancel_soft_hot_temp_limit(bool cancel_it)
{
	int ret;

	/* ME371MG EVB/SR1 meet this problem that smb345 stop charging
	   when IC temperature is high up to Soft Hot Limit. But Battery
	   is not full charging. We disable this limitation.
	 */

	ret = smb358_read(smb358_dev, CFG_THERM);
	if (ret < 0)
		return ret;

	ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
	if (!cancel_it)
		ret |= 0x02;	/* Float voltage compensation */

	ret = smb358_write(smb358_dev, CFG_THERM, ret);
	if (ret < 0)
		return ret;
}

/*----------------------------------------------------------------------------*/
/* JEITA function for cell temperature control by SoC
 */
int smb358_soc_control_jeita(void)
{
	int ret;

	if (!smb358_dev) {
		pr_err
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("%s:", __func__);
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	/* set hard hot limit = 72 deg.c, write 0bh[5:4]="11" */
	ret = smb358_masked_write(smb358_dev->client,
				  HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
				  BIT(5) | BIT(4), BIT(5) | BIT(4));
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n",
		       ret);
		return ret;
	}

	/* set soft hot limit behavior = no response, write 07h[1:0]="00" */
	ret = cancel_soft_hot_temp_limit(true);
	if (ret) {
		pr_err
		    ("fail to set Soft Hot Limit Behavior to No Response, ret=%d\n",
		     ret);
		return ret;
	}

	/* set soft cold temp limit = no response, write 07h[3:2]="00" */
	ret = smb358_masked_write(smb358_dev->client, 0x07, BIT(3) | BIT(2), 0);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n",
		       ret);
		return ret;
	}

	return ret;
}

void smb358_set_usb_in_500ma(void)
{
	int ret;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return;
	}
#endif

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		pr_err("fail to allow volatile register can be written\n");
		return;
	}

	/* Disable AICL - Write 02h[4]="0" */
	if (smb358_OptiCharge_Toggle(false) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to disable AICL\n", __func__);
		return;
	}

	/* Set I_USB_IN=500mA - Write 01h[7:4]="0001" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x01,
				  BIT(4) | BIT(5) | BIT(6) | BIT(7),
				  BIT(4));
	if (ret) {
		pr_err("fail to set I_USB_IN to 500mA ret=%d\n",
		       ret);
		return;
	}

	/* Enable AICL - Write 02h[4]="1" */
	if (smb358_OptiCharge_Toggle(true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to enable AICL\n", __func__);
		return;
	}
}
EXPORT_SYMBOL(smb358_set_usb_in_500ma);

/*
 * JEITA function for cell temperature control by Charger IC
 */
int smb358_charger_control_jeita(void)
{
	int ret;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	if (!smb358_dev) {
		pr_err
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("%s:", __func__);
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		pr_err("fail to allow volatile register can be written\n");
		return ret;
	}

	/* set hard hot limit = 59 deg.c, write 0bh[5:4]= "01" */
	ret = smb358_masked_write(smb358_dev->client,
				  HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
				  HARD_LIMIT_HOT_CELL_TEMP_MASK, 0x01);

	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n",
		       ret);
		return ret;
	}

	/* set soft hot limit behavior = float voltage compensation, write 07h[1:0]="10" */
	ret = cancel_soft_hot_temp_limit(false);
	if (ret) {
		pr_err
		    ("fail to set Soft Hot Limit Behavior to Float Voltage Compensation, ret=%d\n",
		     ret);
		return ret;
	}

	/* set soft cold temp limit = charge current compensation, write 07h[3:2]="01" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x07, BIT(2) | BIT(3), BIT(2));
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_HOT_CELL_TEMP_MASK, ret=%d\n",
		       ret);
		return ret;
	}

	/* charger enable, write 06h[6:5]="11" */
	ret = smb358_charging_toggle(JEITA, true);
	if (ret) {
		pr_err("fail to enable charger, ret=%d\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL(smb358_charger_control_jeita);

/*
 * JEITA function for float voltage configuration by SoC
 */
int smb358_soc_control_float_vol(int bat_temp)
{
	int ret;
	int batt_volt;

	if (!smb358_dev) {
		pr_err
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return ret;
	} else
		BAT_DBG(" %s: get battery voltage(%d)\n", __func__, batt_volt);

	/* the unit of battery temperature is 0.1C */
	if (bat_temp < FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD) {
		/* write 03h[5:0]="101010" or "101011" */
		ret = smb358_masked_write(smb358_dev->client,
					  FLOAT_VOLTAGE_REG,
					  FLOAT_VOLTAGE_MASK, 0x2B);
	} else {
		if (bat_temp < 550) {
			if (batt_volt < 4110) {
				/* write 03h[5:0]="011110" */
				ret = smb358_masked_write(smb358_dev->client,
							  FLOAT_VOLTAGE_REG,
							  FLOAT_VOLTAGE_MASK,
							  0x1E);
			} else {
				/* write 03h[5:0]="101011" */
				ret = smb358_masked_write(smb358_dev->client,
							  FLOAT_VOLTAGE_REG,
							  FLOAT_VOLTAGE_MASK,
							  0x2B);
			}
		} else {
			/* write 03h[5:0]="011110" */
			ret = smb358_masked_write(smb358_dev->client,
						  FLOAT_VOLTAGE_REG,
						  FLOAT_VOLTAGE_MASK, 0x1E);
		}
	}

	if (ret)
		pr_err("fail to set FLOAT_VOLTAGE_REG ret=%d\n", ret);

	return ret;
}

/*----------------------------------------------------------------------------*/

/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl, size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size - 1];
	return tbl[val];
}

/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
int smb358_get_aicl_results(void)
{
	int ret;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 501;
	}
#endif

	ret = smb358_read(smb358_dev, STAT_E);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		return ret;
	}

	ret &= 0x0F;
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret);
}
EXPORT_SYMBOL(smb358_get_aicl_results);

/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
	int ret;

	ret = smb358_read(smb358_dev, STAT_E);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		return ret;
	}

	ret &= 0x0F;
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret);
}

/* Acquire the value of input Results in Status Register 1 (01h)
   return the current value (unit: mA)
*/
static int get_input_results(void)
{
	int ret;

	ret = smb358_read(smb358_dev, CFG_CURRENT_LIMIT);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read CFG_CURRENT_LIMIT reg\n",
			  __func__);
		return ret;
	}

	ret &= 0xF0;
	ret >>= 4;
	BAT_DBG(" %s: read CFG_CURRENT_LIMIT, value = %d\n", __func__, ret);
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: ERROR: smb358_dev is null due to probe function has error\n",
		     __func__);
		return sprintf(buf, "%d\n", -EINVAL);
	}

	ret = smb358_read(smb358_dev, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;
	return sprintf(buf, "%d\n",
		       hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret;

	ret = smb358_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING
	    || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static char gbuffer[64];
/* Generate UUID by invoking kernel library */
static void generate_key(void)
{
	char sysctl_bootid[16];

	generate_random_uuid(sysctl_bootid);
	sprintf(gbuffer, "%pU", sysctl_bootid);
}

/* Acquire the UUID */
static ssize_t get_charge_keys(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	generate_key();
	return sprintf(buf, "%s\n", gbuffer);
}

static DEVICE_ATTR(charge_keys, S_IRUGO, get_charge_keys, NULL);
static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_charge_keys.attr,
	NULL,
};

static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#define SMB_DUMP(...) \
do { \
        local_len = sprintf(page, __VA_ARGS__); \
        len += local_len; \
        page += local_len; \
}while(0);

/*----------------------------------------------------------------------------*/

static int config_otg_regs(int toggle)
{
	int ret;

	if (toggle) {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb358_masked_write(smb358_dev->client,
					  OTG_TLIM_THERM_CNTRL_REG,
					  OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					  SMB358_OTG_CURRENT_LIMIT_250mA);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n",
			       ret);
			return ret;
		}

		/* Toggle to enable OTG function: output High */
		gpio_set_value(smb358_dev->chg_otg_en_gpio, 1);

		/* Set OTG current limit to 500mA: 0Ah[3:2]="01" */
		ret = smb358_masked_write(smb358_dev->client,
					  OTG_TLIM_THERM_CNTRL_REG,
					  OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					  SMB358_OTG_CURRENT_LIMIT_500mA);
		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n",
			       ret);
			return ret;
		}
	} else {
		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb358_masked_write(smb358_dev->client,
					  OTG_TLIM_THERM_CNTRL_REG,
					  OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					  SMB358_OTG_CURRENT_LIMIT_250mA);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n",
			       ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb358_dev->chg_otg_en_gpio, 0);
	}

	return ret;
}

/*----------------------------------------------------------------------------*/

static int otg(int toggle)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	smb358_dev->otg_enabled = (toggle > 0 ? true : false);
	return 0;
}

static int smb358_vbus_enable(int toggle)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	printk("%s(): %s\n", __func__, (toggle) ? "enable" : "disable");
	/*FIXME: so far revert boost was not ready yet */
	return 0;

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	return 0;
}

/* enable/disable AICL function */
static int smb358_OptiCharge_Toggle(bool on)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;
	else
		ret &= ~CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;

	ret = smb358_write(smb358_dev, CFG_VARIOUS_FUNCS, ret);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

/* print the value in Various Functions Register */
static int smb358_get_AICL(void)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_VARIOUS_FUNCS, BYTETOBINARY(ret));
fail:
	return ret;
}

/* print the value in CMD_B */
static int smb358_get_USB9_HC_Toggle(void)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CMD_B);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
fail:
	return ret;
}

/* enable USB5 or USB9 and HC mode function */
static int smb358_USB9_HC_Toggle(bool on)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CMD_B);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CMD_B_USB9_AND_HC_MODE;
	else
		ret &= ~CMD_B_USB9_AND_HC_MODE;
	ret = smb358_write(smb358_dev, CMD_B, ret);

fail:
	return ret;
}

/* print the value in CFG_PIN */
static int smb358_get_USB9_HC_PIN_Control(void)
{
	int ret;

	if (!smb358_dev) {
		pr_info("%s: smb358_dev is null "
			"due to driver probed isn't ready\n", __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CFG_PIN);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_PIN, BYTETOBINARY(ret));
fail:
	return ret;
}

/* enable USB5 or USB9 and HC mode pin control function */
static int smb358_USB9_HC_PIN_Control(bool on)
{
	int ret;
	u8 b = BIT(4);

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CFG_PIN);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= b;
	else
		ret &= ~b;
	ret = smb358_write(smb358_dev, CFG_PIN, ret);

fail:
	return ret;
}

/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

/* print the value in CFG_CURRENT_LIMIT */
static int smb358_get_current_limits(void)
{
	int ret;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_CURRENT_LIMIT, BYTETOBINARY(ret));
	return ret;
}

static int smb358_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret;

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		return smb358_masked_write(smb358_dev->client,
					   CFG_CURRENT_LIMIT,
					   CFG_CURRENT_LIMIT_SMB346_MASK,
					   CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
	}

	return ret;
}

/* print the value in FLOAT_VOLTAGE_REG */
static int smb358_get_chrg_voltage(void)
{
	int ret, voltage;

	if (!smb358_dev) {
		pr_info
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return -1;
	}

	ret = smb358_read(smb358_dev, FLOAT_VOLTAGE_REG);
	if (ret < 0)
		return ret;

	ret &= (BIT(0) | BIT(1) | BIT(2) | BIT(3) | BIT(4) | BIT(5));

	switch (ret) {
	case 0x1e:
		voltage = 4110;
		break;
	case 0x2a:
		voltage = 4340;
		break;
	case 0x2b:
	default:
		voltage = 4350;
		break;
	}

	return voltage;
}

static void aicl_cur_control(int usb_state)
{
	int aicl_result;

	if (usb_state != AC_IN)
		return;

	aicl_result = get_aicl_results();
	if (aicl_result > 500) {
		dev_info(&smb358_dev->client->dev,
			 "%s: do nothing when aicl result(%dmA) > 500mA.\n",
			 __func__, aicl_result);
		return;
	} else {
		dev_info(&smb358_dev->client->dev,
			 "%s: execute AICL routine control work aicl result =%d\n",
			 __func__, aicl_result);
	}

	/* Allow volatile register can be written - Write 30h[7]="1" */
	if (smb358_set_writable(smb358_dev, true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return;
	}

	smb358_pre_config();

	/* Allow volatile register can be written - Write 30h[7]="1" */
	if (smb358_set_writable(smb358_dev, true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return;
	}

	smb3xx_config_max_current(usb_state);
}

void smb358_aicl_dete_worker(struct work_struct *dat)
{
	int usb_state;
	int rsoc;

	if (!smb358_dev) {
		pr_err
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return;
	}

	mutex_lock(&smb358_g_usb_state_lock);
	usb_state = g_usb_state;
	mutex_unlock(&smb358_g_usb_state_lock);

	aicl_cur_control(usb_state);
	smb346_soc_detect_batt_tempr(usb_state);

	/* acquire battery rsoc here */
	if (get_battery_rsoc(&rsoc)) {
		dev_err(&smb358_dev->client->dev,
			" %s: fail to get battery rsoc\n", __func__);
	} else {
		if (rsoc == 1)
			smb358_dump_registers(NULL);
	}

	if (dat)
		queue_delayed_work(smb358_dev->chrgr_work_queue,
				   &smb358_dev->aicl_dete_work, 30 * HZ);
}

EXPORT_SYMBOL(smb358_aicl_dete_worker);

static void verifyFW(void)
{
	if (!smb358_dev) {
		pr_err
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return;
	}

	/* Get USB to HC mode */
	if (smb358_get_USB9_HC_Toggle() < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to get USB9 and HC mode!\n", __func__);
		return;
	}

	/* Get USB5/1/HC to register control */
	if (smb358_get_USB9_HC_PIN_Control() < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to get USB9 and HC mode pin control!\n",
			__func__);
		return;
	}

	/* Get I_USB_IN value */
	if (smb358_get_current_limits() < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to get max current limits!\n", __func__);
		return;
	}
}

#if 0
static void smb345_config_max_current_twinheadeddragon(void)
{
	if (!smb345_dev) {
		pr_err
		    ("%s: smb345_dev is null due to driver probed isn't ready\n",
		     __func__);
		return;
	}

	/* check if ACOK# = 0 */
	if (smb345_dev->pdata->inok_gpio >= 0
	    && gpio_get_value(smb345_dev->pdata->inok_gpio)) {
		dev_err(&smb345_dev->client->dev,
			"%s: system input voltage is not valid >>> INOK pin (HIGH) <<<\n",
			__func__);
	} else {
		dev_err(&smb345_dev->client->dev,
			"%s: negative gpio number: INOK!\n", __func__);
		return;
	}

	/* Allow volatile register can be written - Write 30h[7]="1" */
	if (smb345_set_writable(smb345_dev, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: smb345_set_writable failed!\n", __func__);
		return;
	}

	/* Disable AICL - Write 02h[4]="0" */
	if (smb345_OptiCharge_Toggle(false) < 0) {
		dev_err(&smb345_dev->client->dev, "%s: fail to disable AICL\n",
			__func__);
		return;
	}

	/* Set I_USB_IN=1800mA - 01h[3:0]="0110" */
	if (smb345_set_current_limits(USB_IN, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to set max current limits for TWINSHEADED\n",
			__func__);
		return;
	}

	/* Set USB to HC mode - Write 31h[1:0]="11" */
	if (smb345_USB9_HC_Toggle(true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to enable USB9 and HC mode!\n", __func__);
		return;
	}

	/* Set USB5/1/HC to register control - Write 06h[4]="0" */
	if (smb345_USB9_HC_PIN_Control(false) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to disable USB9 and HC mode pin control!\n",
			__func__);
		return;
	}

	smb345_soc_control_jeita();

	/* check if ACOK# = 0 */
	if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
		dev_err(&smb345_dev->client->dev,
			"%s: system input voltage is not valid after charge current settings\n",
			__func__);
		return;
	}

	pr_info("%s: charger type: TWINHEADEDDRAGON done.\n", __func__);
}
#endif

int smb358_otg_toggle(bool b)
{
	int ret;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	if (b == true) {
		/* Set cover OTG output current = 900mA: 0Ah[3:2]="11" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x0A,
					  BIT(2) | BIT(3),
					  BIT(2) | BIT(3));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set cover OTG output current!\n", __func__);
			goto fail;
		}

		/* Set OTG/ID Pin Control (Cover OTG enable): 09h[7:6]="01" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x09,
					  BIT(7) | BIT(6),
					  BIT(6));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set OTG/ID pin control (cover OTG enable)!\n", __func__);
			goto fail;
		}
	}
	else {
		/* Set OTG and ID i2c control: 09h[7:6]="00" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x09,
					  BIT(6) | BIT(7),
					  0);
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set OTG and ID i2c control!\n", __func__);
			goto fail;
		}
		
		/* Disable OTG: 30h[4]="0" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x30,
					  BIT(4), 0);
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable OTG!\n", __func__);
			goto fail;
		}
	}

	return 0;

fail:
	return ret;
}
EXPORT_SYMBOL(smb358_otg_toggle);

int smb358_do_cover_jeita(void)
{
	int ret;
	int batt_tempr = 250;	/* unit: 0.1C  */
	int batt_volt = 3800;	/* unit: mV  */

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	ret = smb358_soc_control_jeita();
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358 soc control jeita failed!\n", __func__);
		return ret;
	}

	// TODO
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
		return ret;
	}

	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return ret;
	}

	/* Allow volatile register can be written - Write 30h[7]="1" */
	BAT_DBG("%s: Allow volatile register can be written\n", __func__);
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return ret;
	}

	ret = smb358_do_battery_jeita(batt_tempr, batt_volt, battery_temp_zone);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358 do battery jeita failed!\n", __func__);
		return ret;
	}
}
EXPORT_SYMBOL(smb358_do_cover_jeita);

void smb358_extern_set_writable(void)
{
	int ret;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	BAT_DBG("%s: Allow volatile register can be written\n", __func__);
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
	}
}
EXPORT_SYMBOL(smb358_extern_set_writable);

int smb358_aicl_toggle(bool toggle)
{
	int ret = 0;

	if (toggle == true) {
		/* Enable cover AICL: 02h[4]="1" */
		BAT_DBG("%s: Enable cover AICL\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x02,
					  BIT(4),
					  BIT(4));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable cover AICL!\n", __func__);
			return ret;
		}
	} else {
		/* Disable cover AICL: 02h[4]="0" */
		BAT_DBG("%s: Disable cover AICL\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x02,
					  BIT(4),
					  0);
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable cover AICL!\n", __func__);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(smb358_aicl_toggle);

int smb358_suspend_charger_usbin(bool toggle)
{
	int ret = 0;

	if (toggle == true) {
		/* Suspend Cover Charger USBIN: 30h[2]="1" */
		BAT_DBG("%s: Suspend Cover Charger USBIN\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x30,
					  BIT(2),
					  BIT(2));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to suspend cover charger USBIN!\n", __func__);
			return ret;
		}
	} else {
		/* Enable Cover Charger USBIN: 30h[2]="0" */
		BAT_DBG("%s: Enable Cover Charger USBIN\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x30,
					  BIT(2),
					  0);
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable cover charger USBIN!\n", __func__);
			return ret;
		}
	}

	return ret;
}
EXPORT_SYMBOL(smb358_suspend_charger_usbin);

int smb358_set_charger_usbin(int cur)
{
	int ret = 0;

	switch (cur) {
	case 1200:
		/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
		BAT_DBG("%s: Set I_USB_IN=1200mA\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x01,
					  BIT(4) | BIT(5) | BIT(6) | BIT(7),
					  BIT(6));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set I_USB_IN=1200mA!\n", __func__);
			return ret;
		}
		break;
	case 500:
		/* Set I_USB_IN=500mA - Write 01h[7:4]="0001" */
		BAT_DBG("%s: Set I_USB_IN=500mA\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x01,
					  BIT(4) | BIT(5) | BIT(6) | BIT(7),
					  BIT(4));
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set I_USB_IN=500mA!\n", __func__);
			return ret;
		}
		break;
	case 300:
		/* Set I_USB_IN=300mA - Write 01h[7:4]="0000" */
		BAT_DBG("%s: Set I_USB_IN=300mA\n", __func__);
		ret = smb358_masked_write(smb358_dev->client,
					  0x01,
					  BIT(4) | BIT(5) | BIT(6) | BIT(7),
					  0);
		if (ret < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set I_USB_IN=300mA!\n", __func__);
			return ret;
		}
		break;
	default:
		BAT_DBG("%s: unsupported current, %d\n", __func__, cur);
	}

	return ret;
}
EXPORT_SYMBOL(smb358_set_charger_usbin);

int smb358_set_cover_charging_current(int rsoc, int pack_rsoc)
{
	int ret;
	int cover_type;

	//BAT_DBG("%s: ++++++ begin ++++++", __func__);
	BAT_DBG("%s: battery capacity is %d%%\n", __func__, rsoc);
	BAT_DBG("%s: pack battery capacity is %d%%\n", __func__, pack_rsoc);

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	cover_type = get_cover_type();

	/* Allow volatile register can be written - Write 30h[7]="1" */
	BAT_DBG("%s: Allow volatile register can be written\n", __func__);
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return ret;
	}

	/* Disable cover AICL: 02h[4]="0" */
	BAT_DBG("%s: Disable cover AICL\n", __func__);
	ret = smb358_masked_write(smb358_dev->client,
				  0x02,
				  BIT(4),
				  0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to disable cover AICL!\n", __func__);
		return ret;
	}

	/* +++++++ start to do cover and tablet charge setting_+++++++*/
	if (cover_type == AUDIO_SLEEVE) {
		BAT_DBG("%s: cover type is audio_sleeve\n", __func__);
	}
	else if (rsoc == 100) {
		BAT_DBG("%s: battery capaciy == 100%%\n", __func__);

		/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
		BAT_DBG("%s: Set I_USB_IN=1200mA\n", __func__);
		smb358_masked_write(smb358_dev->client,
					  0x01,
					  BIT(4) | BIT(5) | BIT(6) | BIT(7),
					  BIT(6));

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}

		goto done;
	}
	else if (pack_rsoc == 100) {
		BAT_DBG("%s: pack battery capaciy == 100%%\n", __func__);

		/* Set Cover Charging Disable - Write 06h[6:5]="00" */
		BAT_DBG("%s: Set Cover Charging Disable\n", __func__);
		smb358_masked_write(smb358_dev->client,
					  0x06,
					  BIT(6) | BIT(5),
					  0);

		goto done;
	}
	else if (rsoc > 70 && pack_rsoc < 70) {
		if (rsoc > 90) {
			BAT_DBG("%s: battery capaciy > 90%%, pack battery capacity < 70%%\n", __func__);

			/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
			BAT_DBG("%s: Set I_USB_IN=1200mA\n", __func__);
			smb358_masked_write(smb358_dev->client,
						  0x01,
						  BIT(4) | BIT(5) | BIT(6) | BIT(7),
						  BIT(6));

			/* Enable AICL - Write 02h[4]="1" */
			if (smb358_OptiCharge_Toggle(true) < 0) {
				dev_err(&smb358_dev->client->dev,
					"%s: fail to enable AICL\n", __func__);
				return;
			}

			goto done;
		}
	}
	else if (rsoc < 70 && pack_rsoc > 70) {
		if (pack_rsoc > 90) {
			BAT_DBG("%s: battery capacity < 70%%, pack battery capaciy > 90%%\n", __func__);

			/* Set Cover Charging Disable - Write 06h[6:5]="00" */
			BAT_DBG("%s: Set Cover Charging Disable\n", __func__);
			smb358_masked_write(smb358_dev->client,
						  0x06,
						  BIT(6) | BIT(5),
						  0);

			goto done;
		}
		else if (rsoc < 30) {
			BAT_DBG("%s: battery capacity < 30%%, pack battery capaciy > 90%%\n", __func__);

			/* Set I_USB_IN=300mA - Write 01h[7:4]="0000" */
			BAT_DBG("%s: Set I_USB_IN=300mA\n", __func__);
			smb358_masked_write(smb358_dev->client,
						  0x01,
						  BIT(4) | BIT(5) | BIT(6) | BIT(7),
						  0);

			/* Enable AICL - Write 02h[4]="1" */
			if (smb358_OptiCharge_Toggle(true) < 0) {
				dev_err(&smb358_dev->client->dev,
					"%s: fail to enable AICL\n", __func__);
				return;
			}

			goto done;
		}
	}
	else if (rsoc < 30) {
		BAT_DBG("%s: battery capacity < 30%%, pack battery capaciy > 70%%\n", __func__);

		/* Set I_USB_IN=300mA - Write 01h[7:4]="0000" */
		BAT_DBG("%s: Set I_USB_IN=300mA\n", __func__);
		smb358_masked_write(smb358_dev->client,
					  0x01,
					  BIT(4) | BIT(5) | BIT(6) | BIT(7),
					  0);

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}

		goto done;
	}

	BAT_DBG("%s: PAD 700mA, COVER 500mA\n", __func__);

	/* Set I_USB_IN=500mA - Write 01h[7:4]="0001" */
	BAT_DBG("%s: Set I_USB_IN=500mA\n", __func__);
	smb358_masked_write(smb358_dev->client,
				  0x01,
				  BIT(4) | BIT(5) | BIT(6) | BIT(7),
				  BIT(4));

	/* Enable AICL - Write 02h[4]="1" */
	if (smb358_OptiCharge_Toggle(true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to enable AICL\n", __func__);
		return;
	}

done:
	//smb358_do_cover_jeita();

	BAT_DBG("%s: smb358_set_cover_charging_current done\n", __func__);
	//BAT_DBG("%s: ++++++ end ++++++", __func__);

	return 0;
}
EXPORT_SYMBOL(smb358_set_cover_charging_current);

//extern bool ug31xx_probe_done;
bool smb358_ug31xx_probe_done = true;
static int smb358_set_charging_voltage(void)
{
	int ret;

	/* Vchg=4.34V, write 03h[5:0]="101010" */
	ret = smb358_masked_write(smb358_dev->client,
				  FLOAT_VOLTAGE_REG,
				  FLOAT_VOLTAGE_MASK,
				  BIT(1) | BIT(3) | BIT(5));

	return ret;
}

int smb358_pre_config_usb(void)
{
	int ret;

	BAT_DBG("%s: ### begin ###\n", __func__);

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	/* Set fast charge current to 1500mA: 00h[7:0]="10100110"
	 * Set pre-charge current to 150mA
	 * Set termination current to 150mA
	 */
	ret = smb358_masked_write(smb358_dev->client,
				  CHG_CURRENT_REG,
				  0xff,
				  BIT(1) | BIT(2) | BIT(5) | BIT(7));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set fast charge current!\n", __func__);
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set pre-charge current!\n", __func__);
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set termination current!\n", __func__);
		goto fail;
	}

	/* set cold soft limit current to 600mA: 0Ah[7:6]="10" */
	ret = smb358_masked_write(smb358_dev->client,
				  OTG_TLIM_THERM_CNTRL_REG,
				  BIT(6) | BIT(7), BIT(7));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set cold soft limit current!\n", __func__);
		goto fail;
	}

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	/* battery OV does not end charge cycle: 02h[1]="0" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x02,
				  BIT(1), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set that battery OV does not end charge cycle!\n",
			__func__);
		goto fail;
	}

	/* set charger voltage to 4.35V: 03h[5:0]="101011" */
	ret = smb358_set_charging_voltage();
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set charger voltage!\n", __func__);
		goto fail;
	}

	BAT_DBG("%s: ### end ###\n", __func__);

	return 0;

fail:
	BAT_DBG("%s: ### failed end ###\n", __func__);
	return ret;
}
EXPORT_SYMBOL(smb358_pre_config_usb);


int smb358_pre_config(void)
{
	int ret;

	BAT_DBG("%s: ### begin ###\n", __func__);

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	/* Set fast charge current to 1500mA: 00h[7:0]="10100110"
	 * Set pre-charge current to 150mA
	 * Set termination current to 150mA
	 */
	ret = smb358_masked_write(smb358_dev->client,
				  CHG_CURRENT_REG,
				  0xff,
				  BIT(1) | BIT(2) | BIT(5) | BIT(7));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set fast charge current!\n", __func__);
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set pre-charge current!\n", __func__);
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set termination current!\n", __func__);
		goto fail;
	}

	/* set cold soft limit current to 600mA: 0Ah[7:6]="10" */
	ret = smb358_masked_write(smb358_dev->client,
				  OTG_TLIM_THERM_CNTRL_REG,
				  BIT(6) | BIT(7), BIT(7));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set cold soft limit current!\n", __func__);
		goto fail;
	}

	/* battery OV does not end charge cycle: 02h[1]="0" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x02,
				  BIT(1), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set that battery OV does not end charge cycle!\n",
			__func__);
		goto fail;
	}

	/* set charger voltage to 4.35V: 03h[5:0]="101011" */
	ret = smb358_set_charging_voltage();
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set charger voltage!\n", __func__);
		goto fail;
	}

	/* Cancel this part after porting guide version 20150527 */
#if 0
	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	/* set OTG and ID i2c control: 09h[7:6]="00" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x09,
				  BIT(6) | BIT(7), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set OTG and ID I2C control!\n",
			__func__);
		goto fail;
	}

	/* disable OTG: 30h[4]="0" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x30,
				  BIT(4), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to disable OTG!\n",
			__func__);
		goto fail;
	}
#endif

	/* cancel after porting guide version 20150512 */
#if 0
	/* set USB suspend mode = register control: 02h[7]="1" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x02,
				  BIT(7), BIT(7));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set USB suspend mode!\n",
			__func__);
		goto fail;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		goto fail;
	}

	/* APSD disable: 04h[2]="0" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x04,
				  BIT(2), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to disable APSD!\n",
			__func__);
		goto fail;
	}

	/* USB/1/HC = I2C Control: 06h[4]="0" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x06,
				  BIT(4), 0);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set USB/1/HC = I2C Control!\n",
			__func__);
		goto fail;
	}

	/* USB/HC Mode = High-current mode: 31h[0]="1" */
	ret = smb358_masked_write(smb358_dev->client,
				  0x31,
				  BIT(0), BIT(0));
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: fail to set USB/HC Mode = High-current mode!\n",
			__func__);
		goto fail;
	}

	BAT_DBG("%s: ### end ###\n", __func__);

	return 0;

fail:
	BAT_DBG("%s: ### failed end ###\n", __func__);
	return ret;
}
EXPORT_SYMBOL(smb358_pre_config);

/*----------------------------------------------------------------------------*/

static void smb3xx_config_max_current(int usb_state)
{
	int input_result;

	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		input_result = get_input_results();
		if (input_result >= 1200) {
			dev_err(&smb358_dev->client->dev,
				"%s: now input current setting is larger than 1200 mA, no need set again!!\n",
				__func__);
			return;
		}

		/* Disable AICL - Write 02h[4]="0" */
		if (smb358_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
		if (smb358_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb358_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb358_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	}
}

/*----------------------------------------------------------------------------*/

static void smb358_config_max_current(int usb_state)
{
	if (usb_state != AC_IN && usb_state != USB_IN)
		return;

	if (!smb358_dev) {
		pr_err
		    ("%s: smb358_dev is null due to driver probed isn't ready\n",
		     __func__);
		return;
	}
#if 0
	/* check if ACOK# = 0 */
	if (smb345_dev->pdata->inok_gpio >= 0) {
		if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_err(&smb345_dev->client->dev,
				"%s: system input voltage is not valid "
				">>> INOK pin (HIGH) <<<\n", __func__);
			return;
		}
	} else {
		dev_err(&smb345_dev->client->dev,
			"%s: negative gpio number: INOK!\n", __func__);
		return;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	if (smb358_set_writable(smb358_dev, true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return;
	}

	smb358_pre_config();

	/* Allow volatile register can be written - Write 30h[7]="1" */
	if (smb358_set_writable(smb358_dev, true) < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
		return;
	}

	smb3xx_config_max_current(usb_state);

	smb358_charger_control_jeita();

#if 0
	/* check if ACOK# = 0 */
	if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
		dev_err(&smb345_dev->client->dev,
			"%s: system input voltage is not valid after charge current settings\n",
			__func__);
		return;
	}
#endif

	pr_info("%s: charger type:%d done.\n", __func__, usb_state);
}

#ifdef CONFIG_PROC_FS
int smb358_asus_charger_jeita_temp_write(struct file *file, const char *buffer,
					 size_t count, loff_t * data)
{
	BAT_DBG_E(" %s:\n", __func__);

	if (buffer[0] == '0') {
		battery_fake_temp = -50;

	} else if (buffer[0] == '1') {
		battery_fake_temp = 30;

	} else if (buffer[0] == '2') {
		battery_fake_temp = 50;

	} else if (buffer[0] == '3') {
		battery_fake_temp = 110;

	} else if (buffer[0] == '4') {
		battery_fake_temp = 150;

	} else if (buffer[0] == '5') {
		battery_fake_temp = 480;

	} else if (buffer[0] == '6') {
		battery_fake_temp = 510;

	} else if (buffer[0] == '7') {
		battery_fake_temp = 530;

	} else if (buffer[0] == '8') {
		battery_fake_temp = 560;

	} else {
		battery_fake_temp = 0xff;
	}

	return count;
}

static int asus_charger_jeita_temp_read(struct seq_file *m, void *p)
{
	int len;

	BAT_DBG_E(" %s: battery_fake_temp = %d\n", __func__, battery_fake_temp);
	return len;

}

static int asus_charger_jeita_temp_open(struct inode *inode, struct file *file)
{
	return single_open(file, asus_charger_jeita_temp_read, NULL);
}

static const struct file_operations asus_charger_jeita_temp_ops = {
	.open = asus_charger_jeita_temp_open,
	.read = seq_read,
	.write = smb358_asus_charger_jeita_temp_write,
	.llseek = seq_lseek,
	.release = seq_release
};

int smb358_proc_fs_current_control(void)
{
	struct proc_dir_entry *entry = NULL;

	entry =
	    proc_create("driver/charger_jeita_battery_fake_temp", 0664, NULL,
			&asus_charger_jeita_temp_ops);
	if (!entry) {
		BAT_DBG_E("Unable to create asus_charger_jeita_temp_ops\n");
		return -EINVAL;
	}
	return 0;
}
#else
int smb358_proc_fs_current_control(void)
{
	return 0;
}
#endif

int setSMB358Charger(int usb_state)
{
	int ret = 0;
	printk("%s()- usb_state = %d \n", __func__, usb_state);
	mutex_lock(&smb358_g_usb_state_lock);
	g_usb_state = usb_state;
	mutex_unlock(&smb358_g_usb_state_lock);

	if (smb358_dev) {
		switch (usb_state) {
		case USB_IN:
			power_supply_changed(&smb358_power_supplies
					     [CHARGER_USB - 1]);
			break;
		case AC_IN:
			power_supply_changed(&smb358_power_supplies
					     [CHARGER_AC - 1]);
			break;
		case CABLE_OUT:
			power_supply_changed(&smb358_power_supplies
					     [CHARGER_AC - 1]);
			power_supply_changed(&smb358_power_supplies
					     [CHARGER_USB - 1]);
			break;
		}
	}

	switch (usb_state) {
	case USB_IN:
		printk("%s(): USB_IN\n", __func__);
#if 0
		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_warn(&smb345_dev->client->dev,
				 "%s: >>> INOK pin (LOW) <<<\n", __func__);
#endif

			/* charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			 */
			mutex_lock(&smb358_g_usb_state_lock);
			dev_warn(&smb358_dev->client->dev,
				 "%s: config current when USB_IN\n", __func__);
			smb358_config_max_current(USB_IN);
			mutex_unlock(&smb358_g_usb_state_lock);
#if 0
		}
#endif
		break;
	case AC_IN:
		printk("%s(): AC_IN\n", __func__);
#if 0
		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_warn(&smb345_dev->client->dev,
				 "%s: >>> INOK pin (LOW) <<<\n", __func__);
#endif

			/*  charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			 */
			mutex_lock(&smb358_g_usb_state_lock);
			dev_warn(&smb358_dev->client->dev,
				 "%s: config current when AC_IN\n", __func__);
			smb358_config_max_current(AC_IN);
			mutex_unlock(&smb358_g_usb_state_lock);
#if 0
		}
#endif

		if (smb358_dev) {
			if (smb358_entry_mode == 4) {
				if (!wake_lock_active(&smb358_wlock)) {
					BAT_DBG
					    (" %s: asus_battery_power_wakelock "
					     "-> wake lock\n", __func__);
					wake_lock(&smb358_wlock);
				}
			}
		}
		break;

	case CABLE_OUT:
		printk("%s(): CABLE_OUT\n", __func__);
		if (smb358_dev->otg_enabled) {
			printk(KERN_INFO "%s USB_OTG_DISCONNECTED !!!\n",
			       __func__);
			BAT_DBG(" usb_state: DISABLE_5V\n");
			otg(0);
		}
		if (smb358_dev) {
			if (smb358_entry_mode == 4) {
				if (wake_lock_active(&smb358_wlock)) {
					BAT_DBG
					    (" %s: asus_battery_power_wakelock "
					     "-> wake unlock\n", __func__);
					/* timeout value as same as the
					   <charger.exe>\asus_global.h
					   #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
					 */
					wake_lock_timeout(&smb358_wlock_t,
							  3 * HZ);
					wake_unlock(&smb358_wlock);
				} else {	// for PC case
					wake_lock_timeout(&smb358_wlock_t,
							  3 * HZ);
				}
			}
		}
		battery_temp_zone = TEMP_15_50;
		break;

	case ENABLE_5V:
		BAT_DBG(" usb_state: ENABLE_5V\n");
		ret = otg(1);
		break;
	case DISABLE_5V:
		BAT_DBG(" usb_state: DISABLE_5V\n");
		ret = otg(0);
		break;

	default:
		BAT_DBG(" ERROR: wrong usb state value = %d\n", usb_state);
		ret = 1;
	}

	return ret;
}

EXPORT_SYMBOL(setSMB358Charger);

/* write 06h[6:5]="00" or "11" */
int smb358_charging_toggle(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	int charging_toggle;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb358_dev) {
		pr_info
		    ("Warning: smb358_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb358_g_charging_toggle_lock);
	charging_toggle = g_charging_toggle;
	mutex_unlock(&smb358_g_charging_toggle_lock);

	BAT_DBG("%s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
		__func__,
		level_str[old_lvl],
		charging_toggle ? "YES" : "NO",
		level_str[level], on ? "YES" : "NO");

	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!charging_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						BAT_DBG_E
						    ("%s: * reject RESTART charging to phone! *\n",
						     __func__);
						return -1;
					}
				}
			} else if (level == BALANCE) {
				if (!charging_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						BAT_DBG_E
						    ("%s: * reject RESTART charging to phone! *\n",
						     __func__);
						return -1;
					}
				}
			} else {
				/* what the hell are you? */
			}
		} else {
			/* want to stop charging? just do it! */
		}
	} else {
		/* it's the highest level. just do it! */
	}

	/* level value assignment */
	old_lvl = level;

	if (!on)
		BAT_DBG_E(" %s: *** charging toggle: OFF ***\n", __func__);
	else
		BAT_DBG
		    (" %s: --------------- charging toggle: ON ---------------\n",
		     __func__);

	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */
	ret = smb358_read(smb358_dev, CFG_PIN);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;
	if (on) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		   . I2C Control - "0" in Command Register disables charger */
	}

	ret = smb358_write(smb358_dev, CFG_PIN, ret);
	if (ret < 0)
		goto out;

	mutex_lock(&smb358_g_charging_toggle_lock);
	g_charging_toggle = on;
	mutex_unlock(&smb358_g_charging_toggle_lock);

out:
	return ret;
}

bool smb358_external_power_source_present()
{
	return false;
}

#if 0
static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
	struct smb345_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;
	int charger_type;

	/* wake lock to prevent system instantly
	   enter S3 while it's in resuming flow */
	wake_lock_timeout(&smb->wakelock, HZ);

	pm_runtime_get_sync(&smb->client->dev);

	if (gpio_get_value(smb->pdata->inok_gpio)) {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (HIGH) <<<\n",
			 __func__);

		mutex_lock(&smb358_g_usb_state_lock);
		g_usb_state = CABLE_OUT;
		mutex_unlock(&smb358_g_usb_state_lock);

		/* reset to default as missing external power source */
		mutex_lock(&smb358_g_charging_toggle_lock);
		g_charging_toggle = true;
		mutex_unlock(&smb358_g_charging_toggle_lock);
	} else {
		dev_warn(&smb->client->dev, "%s: >>> INOK pin (LOW) <<<\n",
			 __func__);

		/* charge current control algorithm:
		   config the charge current only when
		   Vbus is legal (a valid input voltage
		   is present)
		 */
		mutex_lock(&smb358_g_usb_state_lock);
		charger_type = g_usb_state;
		dev_warn(&smb->client->dev,
			 "%s: config current when inok interrupt\n", __func__);
		// why do 2 times by chris ??
		smb345_config_max_current(charger_type);
		mutex_unlock(&smb358_g_usb_state_lock);
	}

	pm_runtime_put_sync(&smb->client->dev);
	return IRQ_HANDLED;
}

static int smb346_otg_gpio_init(struct smb345_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	printk("@@@@@@ pdata->gp_sdio_2_clk =%d\n", pdata->gp_sdio_2_clk);
	if (pdata->gp_sdio_2_clk < 0) {
		BAT_DBG_E("%s: fail to request CHG_OTG gpio number\n",
			  __func__);
		return -1;
	}

	ret = gpio_request_one(pdata->gp_sdio_2_clk,
			       GPIOF_OUT_INIT_LOW, "smb346_otg");
	if (ret < 0)
		BAT_DBG_E("%s: request CHG_OTG gpio fail!\n", __func__);

	return ret;
}

static int smb345_inok_gpio_init(struct smb345_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->inok_gpio);
	int gpio_usb0_id;

#if 0
	gpio_usb0_id = get_gpio_by_name("USB0_ID_LS");
	if (gpio_get_value(gpio_usb0_id))
		BAT_DBG(">>> USB0_ID (HIGH) <<<\n");
	else
		BAT_DBG(">>> USB0_ID (LOW) <<<\n");
#endif
	if (gpio_get_value(pdata->inok_gpio))
		BAT_DBG(">>> INOK (HIGH) <<<\n");
	else
		BAT_DBG(">>> INOK (LOW) <<<\n");

	ret = gpio_request_one(pdata->inok_gpio, GPIOF_IN, "smb345_inok");
	if (ret < 0) {
		BAT_DBG_E("%s: request INOK gpio fail!\n", __func__);
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb345_inok_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING |
				   IRQF_ONESHOT, smb->client->name, smb);

	if (ret < 0) {
		BAT_DBG_E("%s: config INOK gpio as IRQ fail!\n", __func__);
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(pdata->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}
#endif

bool smb358_has_charger_error(void)
{
	int ret;

	if (!smb358_dev)
		return -EINVAL;

	ret = smb358_read(smb358_dev, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb358_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		status = POWER_SUPPLY_STATUS_CHARGING;
		return status;
	}
#endif

	if (!smb358_dev)
		return -EINVAL;

	ret = smb358_read(smb358_dev, STAT_C);
	if (ret < 0)
		return ret;

	irqstat_c = smb358_read(smb358_dev, IRQSTAT_C);
	if (irqstat_c < 0)
		return irqstat_c;
#if 0
	dev_info(&smb345_dev->client->dev,
		 "Charging Status: STAT_C:0x%x\n", ret);
#endif

	if ((ret & STAT_C_CHARGER_ERROR) || (ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}

		if (irqstat_c & IRQSTAT_C_TERMINATION_STAT)
			status = POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}
EXPORT_SYMBOL(smb358_get_charging_status);

#ifndef ASUS_USER_BUILD
int smb358_dump_registers(struct seq_file *s)
{
	struct smb358_charger *smb;
	int ret;
	u8 reg;

	if (s) {
		smb = s->private;
	} else {
		if (!smb358_dev) {
			BAT_DBG(" %s: smb358_dev is null!\n", __func__);
			return -1;
		} else {
			smb = smb358_dev;
		}
	}

	BAT_DBG(" %s:\n", __func__);
	BAT_DBG(" Control registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");

	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb358_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				   "\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Command registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Command registers:\n");
		seq_printf(s, "==================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}

	ret = smb358_read(smb, CMD_A);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_A, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			   "\n", CMD_A, BYTETOBINARY(ret));
	ret = smb358_read(smb, CMD_B);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_B, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			   "\n", CMD_B, BYTETOBINARY(ret));
	ret = smb358_read(smb, CMD_C);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_C, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			   "\n", CMD_C, BYTETOBINARY(ret));
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Interrupt status registers:\n");
	BAT_DBG(" ===========================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Interrupt status registers:\n");
		seq_printf(s, "===========================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb358_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				   "\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Status registers:\n");
	BAT_DBG(" =================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Status registers:\n");
		seq_printf(s, "=================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb358_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				   "\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}
#else
int smb358_dump_registers(struct seq_file *s)
{
	return 0;
}
#endif

static int smb358_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");

	smb358_dump_registers(s);

	return 0;
}

static int smb358_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb358_debugfs_show, inode->i_private);
}

static const struct file_operations smb358_debugfs_fops = {
	.open = smb358_debugfs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int smb346_routine_aicl_control(void)
{
	BAT_DBG(" %s\n", __func__);

	INIT_DELAYED_WORK(&smb358_dev->aicl_dete_work, smb358_aicl_dete_worker);
	smb358_dev->chrgr_work_queue =
	    create_singlethread_workqueue("smb346_wq");
	if (!smb358_dev->chrgr_work_queue) {
		BAT_DBG_E(" fail to create \"smb346_wq\"");
		return -ENOMEM;
	}

	queue_delayed_work(smb358_dev->chrgr_work_queue,
			   &smb358_dev->aicl_dete_work, 60 * HZ);

	return 0;
}

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_PACK_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

static inline int get_battery_temperature(int *tempr)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &val);
	if (!ret)
		*tempr = val.intval;

	return ret;
}

static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval / 1000;

	return ret;
}

int smb358_get_battery_current(int *cur)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CURRENT_NOW, &val);
	if (!ret)
		*cur = val.intval / 1000;

	return ret;
}
EXPORT_SYMBOL(smb358_get_battery_current);

static inline int get_battery_rsoc(int *rsoc)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		*rsoc = val.intval;

	return ret;
}

int smb358_do_battery_jeita(int batt_tempr, int batt_volt, int temp_zone_last)
{

	int ret, need_add_0 = 0, need_add_0_10 = 0, need_add_50_55 = 0, need_add_55 = 0;	// temperature delay protection
	int chrg_voltage;

#if defined(ASUS_ENG_BUILD) ||defined(ASUS_USERDEBUG_BUILD)
	// For purpose, use fake battery temperature to simulate real battery temperature.
	if (battery_fake_temp != 0xff)
		batt_tempr = battery_fake_temp;
#endif

	BAT_DBG_E(" %s: batt_tempr: %d 0.1C,batt_volt: %d mV\n", __func__,
		  batt_tempr, batt_volt);

	if (batt_volt <= 0) {
		BAT_DBG_E(" %s: cannot get cover battery voltage, set battery temperture to 30'C!\n", __func__);
		batt_tempr = 300;
		return ret;
	}

	switch (temp_zone_last) {
	case TEMP_1P5:
		BAT_DBG
		    (" %s: invalid battery temperature zone on last one !!! under battery temperature 0 !!! \n",
		     __func__);
		need_add_0 = 30;
		break;
	case TEMP_1P5_15:
		BAT_DBG
		    (" %s: invalid battery temperature zone on last one  !!! between battery temperature 0 and 10 !!! \n",
		     __func__);
		need_add_0_10 = 30;
		break;
	case TEMP_50_55:
		BAT_DBG
		    (" %s: invalid battery temperature zone on last one  !!! between battery temperature 50 and 55 !!! \n",
		     __func__);
		need_add_50_55 = 30;
		break;
	case TEMP_55:
		BAT_DBG
		    (" %s: invalid battery temperature zone on last one !!! upper battery temperature 55 !!! \n",
		     __func__);
		need_add_55 = 30;
		need_add_50_55 = 30;
		break;
	case TEMP_15_50:
	default:
		break;
	}

	if (batt_tempr < (0 + need_add_0)) {
		// Temp < 1.5
		battery_temp_zone = TEMP_1P5;
	} else if (batt_tempr >= (15 + need_add_0)
		   && batt_tempr < (150 + need_add_0_10)) {
		// 1.5 < Temp < 10
		battery_temp_zone = TEMP_1P5_15;
	} else if (batt_tempr >= (150 + need_add_0_10)
		   && batt_tempr < (500 - need_add_50_55)) {
		// 10 < Temp < 50
		battery_temp_zone = TEMP_15_50;
	} else if (batt_tempr >= (500 - need_add_50_55)
		   && batt_tempr < (550 - need_add_55)) {
		// 50 < Temp < 55
		battery_temp_zone = TEMP_50_55;
	} else if (batt_tempr >= (550 - need_add_55)) {
		// 55 < Temp
		battery_temp_zone = TEMP_55;
	}

	switch (battery_temp_zone) {
	case TEMP_1P5:
		/* Vchg=4.35v, write 03h[5:0]="101011" */
		ret = smb358_masked_write(smb358_dev->client,
					  FLOAT_VOLTAGE_REG,
					  FLOAT_VOLTAGE_MASK,
					  BIT(0) | BIT(1) | BIT(3) | BIT(5));

		/* set fast charge current = 600mA, write 00h[7:5]="010" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x00,
					  BIT(5) | BIT(6) | BIT(7), BIT(6));

		/* Charging Disable */
		ret = smb358_charging_toggle(JEITA, false);
		break;
	case TEMP_1P5_15:
		/* Vchg=4.35v, write 03h[5:0]="101011" */
		ret = smb358_masked_write(smb358_dev->client,
					  FLOAT_VOLTAGE_REG,
					  FLOAT_VOLTAGE_MASK,
					  BIT(0) | BIT(1) | BIT(3) | BIT(5));

		/* set fast charge current = 600mA, write 00h[7:5]="010" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x00,
					  BIT(5) | BIT(6) | BIT(7), BIT(6));

		/* Charging Enable */
		ret = smb358_charging_toggle(JEITA, true);
		break;
	case TEMP_50_55:
		chrg_voltage = smb358_get_chrg_voltage();
		if (chrg_voltage == 4350 && batt_volt >= 4110) {
			/* Vchg=4.35v, write 03h[5:0]="101011" */
			ret = smb358_masked_write(smb358_dev->client,
						  FLOAT_VOLTAGE_REG,
						  FLOAT_VOLTAGE_MASK,
						  BIT(0) | BIT(1) | BIT(3) |
						  BIT(5));

			/* set fast charge current = 1500mA, write 06h[7:5]="101" */
			ret = smb358_masked_write(smb358_dev->client,
						  0x00,
						  BIT(5) | BIT(6) | BIT(7),
						  BIT(5) | BIT(7));

			/* Charging Disable */
			ret = smb358_charging_toggle(JEITA, false);
		} else {
			/* Vchg=4.1v, write 03h[5:0]="011110" */
			ret = smb358_masked_write(smb358_dev->client,
						  FLOAT_VOLTAGE_REG,
						  FLOAT_VOLTAGE_MASK,
						  BIT(1) | BIT(2) |
						  BIT(3) | BIT(4));

			/* set fast charge current = 1500mA, write 00h[7:5]="101" */
			ret = smb358_masked_write(smb358_dev->client,
						  0x00,
						  BIT(5) | BIT(6) | BIT(7),
						  BIT(5) | BIT(7));

			/* Charging Enable */
			ret = smb358_charging_toggle(JEITA, true);
		}
		break;
	case TEMP_55:
		/* Vchg=4.35v, write 03h[5:0]="101011" */
		ret = smb358_masked_write(smb358_dev->client,
					  FLOAT_VOLTAGE_REG,
					  FLOAT_VOLTAGE_MASK,
					  BIT(0) | BIT(1) | BIT(3) | BIT(5));

		/* set fast charge current = 1500mA, write 00h[7:5]="101" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x00,
					  BIT(5) | BIT(6) | BIT(7),
					  BIT(5) | BIT(7));

		/* Charging Disable */
		ret = smb358_charging_toggle(JEITA, false);
		break;
	case TEMP_15_50:
	default:
		// normal case
		/* Vchg=4.35v, write 03h[5:0]="101011" */
		ret = smb358_masked_write(smb358_dev->client,
					  FLOAT_VOLTAGE_REG,
					  FLOAT_VOLTAGE_MASK,
					  BIT(0) | BIT(1) | BIT(3) | BIT(5));

		/* set fast charge current = 1500mA, write 00h[7:5]="101" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x00,
					  BIT(5) | BIT(6) | BIT(7),
					  BIT(5) | BIT(7));

		/* Charging Enable */
		ret = smb358_charging_toggle(JEITA, true);
		break;
	}

	return ret;
}

int smb358_battery_jeita(int batt_tempr, int batt_volt)
{
	int ret = 0;
	int rsoc, charging_toggle;

	mutex_lock(&smb358_g_charging_toggle_lock);
	charging_toggle = g_charging_toggle;
	mutex_unlock(&smb358_g_charging_toggle_lock);

#if defined(ASUS_ENG_BUILD)
	/* acquire battery rsoc here */
	if (get_battery_rsoc(&rsoc)) {
		BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
	} else {
		if (rsoc > 59 && smb358_eng_charging_limit2) {
			BAT_DBG
			    (" %s: In eng mode, Disable charger on capacity is more than 60 %% \n",
			     __func__);
			ret = smb358_charging_toggle(JEITA, false);
			goto Done;
		}
	}
#endif
	ret = smb358_do_battery_jeita(batt_tempr, batt_volt, battery_temp_zone);

Done:
	if (charging_toggle != g_charging_toggle)
		smb358_request_power_supply_changed();

	return ret;
}

int cover_smb358_recharge(int batt_volt)
{

	int ret = 0;

	if (batt_volt >= 4250) {
		/* Recharge Voltage = Vflt-200mV, Write 01h[3:2]="10" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x01, BIT(2) | BIT(3), BIT(3)
		    );
	} else {
		/* Recharge Voltage = Vflt-50mV, Write 01h[3:2]="00" */
		ret = smb358_masked_write(smb358_dev->client,
					  0x01, BIT(2) | BIT(3), 0);
	}

	return ret;
}

static int smb346_soc_detect_batt_tempr(int usb_state)
{
	int ret;
	int batt_tempr = 250;	/* unit: C  */
	int batt_volt = 3800;	/* unit: mV  */
	int rsoc, charging_toggle;

	if (usb_state != AC_IN && usb_state != USB_IN
	    && usb_state != PAD_SUPPLY)
		return 0;

	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
		return ret;
	}

	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
		return ret;
	}

	ret = cover_smb358_recharge(batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to do recharge\n", __func__);
	}
#if 0
	ret = smb358_battery_jeita(batt_tempr, batt_volt);
#endif

	mutex_lock(&smb358_g_charging_toggle_lock);
	charging_toggle = g_charging_toggle;
	mutex_unlock(&smb358_g_charging_toggle_lock);

#if defined(ASUS_ENG_BUILD)
	if (get_battery_rsoc(&rsoc)) {
		BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
	} else {
		if (rsoc > 59 && smb358_eng_charging_limit2) {
			BAT_DBG
			    (" %s: In eng mode, Disable charger on capacity is more than 60 %% \n",
			     __func__);
			ret = smb358_charging_toggle(JEITA, false);
			goto Done;
		}
	}
#endif

	smb358_charger_control_jeita();

Done:
	if (charging_toggle != g_charging_toggle)
		smb358_request_power_supply_changed();

	return ret;
}

struct workqueue_struct *smb358_charger_work_queue2 = NULL;
struct delayed_work smb358_charger_work2;

static void do_charger2(struct work_struct *work)
{
	setSMB358Charger(g_usb_state);
}

int smb358_setCharger2(int usb_state)
{
	g_usb_state = usb_state;
	queue_delayed_work(smb358_charger_work_queue2, &smb358_charger_work2,
			   0);

	return 0;
}

#ifdef USB_NOTIFY_CALLBACK
extern unsigned int query_cable_status(void);

static int cable_status_notify(struct notifier_block *self,
			       unsigned long action, void *dev)
{

	if (ischargerSuspend) {
		printk(KERN_INFO
		       "%s chager is suspend but USB still notify !!!\n",
		       __func__);
		wake_lock(&smb358_dev->wakelock);
		isUSBSuspendNotify = true;
		return NOTIFY_OK;
	}

	switch (action) {
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_SDP !!!\n",
		       __func__);
		action = USB_IN;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_CDP !!!\n",
		       __func__);
		action = AC_IN;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_DCP !!!\n",
		       __func__);
		action = AC_IN;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK !!!\n",
		       __func__);
		action = AC_IN;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_SE1:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_SE1 !!!\n",
		       __func__);
		action = AC_IN;
		smb358_setCharger2(action);
		break;
#if 0
	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED:
		printk(KERN_INFO
		       "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG_CONNECTED !!!\n",
		       __func__);
		action = ENABLE_5V;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG_DISCONNECTED:
		printk(KERN_INFO
		       "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG_DISCONNECTED !!!\n",
		       __func__);
		action = DISABLE_5V;
		smb358_setCharger2(action);
		break;
#endif

	case POWER_SUPPLY_CHARGER_TYPE_USB_OTG:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_USB_OTG !!!\n",
		       __func__);
		action = ENABLE_5V;
		smb358_setCharger2(action);
		break;

	case POWER_SUPPLY_CHARGER_TYPE_NONE:
		printk(KERN_INFO "%s POWER_SUPPLY_CHARGER_TYPE_NONE !!!\n",
		       __func__);
		action = CABLE_OUT;
		smb358_setCharger2(action);
		break;

	default:
		printk(KERN_INFO "%s no status = %d !!!\n", __func__,
		       (int)action);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier = {
	.notifier_call = cable_status_notify,
};

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);
#endif

static int smb358_configure_pmu_regs(struct smb358_charger *chrgr)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	printk("%s()\n", __func__);

	if (!chrgr->ctrl_io || !chrgr->ididev || !chrgr->pcfg_io)
		return -EINVAL;

	pm_state_en =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev, "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev,
						       "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	regval = ioread32(CHARGER_CONTROL(chrgr->ctrl_io));

	/* ChargerIRQEdge - CHARGER_CONTROL_CIEDG_FALLING */
	regval &= ~(CHARGER_CONTROL_CIEDG_M << CHARGER_CONTROL_CIEDG_O);
	regval |= (CHARGER_CONTROL_CIEDG_FALLING << CHARGER_CONTROL_CIEDG_O);

	/* ChargerIRQLevel - CHARGER_CONTROL_CILVL_LOW */
	regval &= ~(CHARGER_CONTROL_CILVL_M << CHARGER_CONTROL_CILVL_O);
	regval |= (CHARGER_CONTROL_CILVL_LOW << CHARGER_CONTROL_CILVL_O);

	/* ChargerIRQSensibility - CHARGER_CONTROL_CISENS_EDGE */
	regval &= ~(CHARGER_CONTROL_CISENS_M << CHARGER_CONTROL_CISENS_O);
	regval |= (CHARGER_CONTROL_CISENS_EDGE << CHARGER_CONTROL_CISENS_O);

	/* ChargerIRQEnable - CHARGER_CONTROL_CIEN_EN */
	regval &= ~(CHARGER_CONTROL_CIEN_M << CHARGER_CONTROL_CIEN_O);
	regval |= (CHARGER_CONTROL_CIEN_EN << CHARGER_CONTROL_CIEN_O);

	/* ChargerResetLevel - CHARGER_CONTROL_CHGLVL_LOW */
	regval &= ~(CHARGER_CONTROL_CHGLVL_M << CHARGER_CONTROL_CHGLVL_O);
	regval |= (CHARGER_CONTROL_CHGLVL_LOW << CHARGER_CONTROL_CHGLVL_O);

	/* ChargerIRQDebounce - CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE - as vbus
	   signal is clear enough (not bouncing) */
	regval &= ~(CHARGER_CONTROL_CIDBT_M << CHARGER_CONTROL_CIDBT_O);
	regval |= (CHARGER_CONTROL_IRQ_DEBOUNCE_DISABLE <<
		   CHARGER_CONTROL_CIDBT_O);

#if 1
	/* charger detection CHARGER_CONTROL_CDETSENS */
	regval &= ~(CHARGER_CONTROL_CDETSENS_M << CHARGER_CONTROL_CDETSENS_O);
	regval |= (CHARGER_CONTROL_CDETSENS_EDGE << CHARGER_CONTROL_CDETSENS_O);

	/* charger detection CHARGER_CONTROL_CHDETLVL */
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);
	regval |= (CHARGER_CONTROL_CHDETLVL_HIGH << CHARGER_CONTROL_CHDETLVL_O);

	/* charger detection CHARGER_CONTROL_CHDWEN */
	regval &= ~(CHARGER_CONTROL_CHDWEN_M << CHARGER_CONTROL_CHDWEN_O);
	regval |= (CHARGER_CONTROL_CHDWEN_EN << CHARGER_CONTROL_CHDWEN_O);

#endif

	printk("reg will write to pmu: 0x%08x\n", regval);
	iowrite32(regval, CHARGER_CONTROL(chrgr->ctrl_io));

	/* charger control WR strobe */
	iowrite32((1 << CHARGER_CONTROL_WR_WS_O),
		  CHARGER_CONTROL_WR(chrgr->ctrl_io));
	iowrite32((1 << CHARGER_CONTROL_WR_WS_O),
		  CHARGER_CONTROL_WR(chrgr->ctrl_io));

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return 0;
}

static irqreturn_t smb358_detect_irq_handler(int irq, void *data)
{
	struct smb358_charger *smb = data;
	printk("%s, IRQ %d\n", __func__, irq);
	wake_lock_timeout(&smb->wakelock, EVT_WAKELOCK_TIMEOUT);
	queue_work(system_nrt_wq, &smb->detect_irq_work);
	//queue_delayed_work(smb358_charger_work_queue2, &smb->detect_irq_work, 100);
	return IRQ_HANDLED;
}

static irqreturn_t smb358_interrupt_irq_handler(int irq, void *data)
{
#if 0
	struct smb345_charger *smb = data;
	u32 regval;
#endif
	printk("%s, IRQ %d\n", __func__, irq);
#if 0
	wake_lock_timeout(&smb->wakelock, EVT_WAKELOCK_TIMEOUT);

	regval = ioread32(CHARGER_CONTROL(smb->ctrl_io));
	printk("dump charger control reg = 0x%08x\n", regval);

	queue_work(system_nrt_wq, &smb->interrupt_irq_work);
#endif
	return IRQ_HANDLED;
}

static int smb358_i2c_read_reg(struct i2c_client *client, u8 reg_addr,
			       u8 * data)
{
	int ret, cnt = MAX_NR_OF_I2C_RETRIES;
	struct i2c_msg msgs[] = {
		{
		 .addr = client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = &reg_addr,
		 },
		{
		 .addr = client->addr,
		 .flags = I2C_M_RD,
		 .len = 1,
		 .buf = data,
		 },
	};

	do {
		ret = i2c_transfer(client->adapter, msgs, 2);
#if 0
		if (ret == 2) {
			if (reg_addr < SHADOW_REGS_NR)
				shadow_registers[reg_addr].value = *data;
			return 0;
		}
#endif
	} while (cnt--);

	pr_debug("asilveir: %s failed with err %d\n", __func__, ret);

	return ret;
}

static inline int smb358_GetSTAT_reg(struct smb358_charger *smb, u8 * data)
{

	int ret;

	mutex_lock(&smb->stat_lock);

	ret = smb358_i2c_read_reg(smb->client, smb358_STAT_CTRL0_ADDR, data);

	mutex_unlock(&smb->stat_lock);

	return ret;

}

bool smb358_is_cover_otg(void)
{
	int ret;
	u32 regval;

#ifdef AUDIO_SLEEVE_PR
	if (get_cover_type() != POWER_BANK) {
		BAT_DBG("%s: Not power bank, do nothing.\n", __func__);
		return 0;
	}
#endif

	/* Allow volatile register can be written - Write 30h[7]="1" */
	ret = smb358_set_writable(smb358_dev, true);
	if (ret < 0) {
		dev_err(&smb358_dev->client->dev,
			"%s: smb358_set_writable failed!\n", __func__);
	}

	ret = smb358_i2c_read_reg(smb358_dev->client, 0x09, &regval);
	if (ret < 0) {
		BAT_DBG_E("%s: read smb358 OTG Status fail, %d\n", __func__, ret);
		return false;
	}

	if ((regval&BIT(7))>>7 == 0 && (regval&BIT(6))>>6 == 1) {
		//BAT_DBG("%s: %d%d\n", __func__, (regval&BIT(7))>>7, (regval&BIT(6))>>6);
		return true;
	}
	else {
		//BAT_DBG("%s: %d%d\n", __func__, (regval&BIT(7))>>7, (regval&BIT(6))>>6);
		return false;
	}
}
EXPORT_SYMBOL(smb358_is_cover_otg);

#if 1
static int smb358_IsPOK(struct smb358_charger *chrgr, int *status)
{
	u32 regval;
	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	printk("%s()\n", __func__);

	if (!chrgr->ididev || !chrgr->ctrl_io)
		return -EINVAL;

	pm_state_en =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev, "enable");

	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
	    idi_peripheral_device_pm_get_state_handler(chrgr->ididev,
						       "disable");

	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	ret = idi_set_power_state(chrgr->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}
	regval = ioread32(PMU_CONFIG(chrgr->pcfg_io));

	*status = (regval & PMU_CONFIG_CHGDET_M) ? VBUS_OFF : VBUS_ON;	/* FIXME: it should be reverse */
	pr_debug("PMU POK value: %d\n", *status);

	ret = idi_set_power_state(chrgr->ididev, pm_state_dis, false);	/*FIXME DO I NEED THIS */

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);

	return ret;
}
#endif

static void smb358_interrupt_irq_worker(struct work_struct *work)
{

	int ret = -1;
	uint8_t cnt;
	u8 data;
	//int vbus_prev;
	struct smb358_charger *smb =
	    container_of(work, struct smb358_charger, interrupt_irq_work);

	//vbus_prev = smb->vbus;

	smb358_IsPOK(smb, &smb->vbus);
	pr_info("%s: vbus = %d\n", __func__, smb->vbus);

	return;
}

static int smb358_set_trigger_level(int level)
{
	u32 regval;

	struct device_state_pm_state *pm_state_en, *pm_state_dis;
	int ret;

	printk("%s, level = %d\n", __func__, level);

	if (!smb358_dev->ctrl_io || !smb358_dev->ididev || !smb358_dev->pcfg_io) {
		printk("%s, io error\n", __func__);
		return -EINVAL;
	}

	pm_state_en =
	    idi_peripheral_device_pm_get_state_handler(smb358_dev->ididev,
						       "enable");
	if (pm_state_en == NULL) {
		pr_err("Unable to get handler for PM state 'enable'!\n");
		return -EINVAL;
	}

	pm_state_dis =
	    idi_peripheral_device_pm_get_state_handler(smb358_dev->ididev,
						       "disable");
	if (pm_state_dis == NULL) {
		pr_err("Unable to get handler for PM state 'disable'!\n");
		return -EINVAL;
	}

	pr_info("Getting PM state handlers: OK\n");

	ret = idi_set_power_state(smb358_dev->ididev, pm_state_en, true);

	if (ret) {
		pr_err("setting PM state '%s' failed!\n", pm_state_en->name);
		return -EIO;
	}

	regval = ioread32(CHARGER_CONTROL(smb358_dev->ctrl_io));

	printk("reg before write to pmu: 0x%08x\n", regval);

	/* charger detection CHARGER_CONTROL_CHDETLVL */
	regval &= ~(CHARGER_CONTROL_CHDETLVL_M << CHARGER_CONTROL_CHDETLVL_O);
	regval |= (level << CHARGER_CONTROL_CHDETLVL_O);

	iowrite32(regval, CHARGER_CONTROL(smb358_dev->ctrl_io));

	printk("reg will write to pmu: 0x%08x\n", regval);

	ret = idi_set_power_state(smb358_dev->ididev, pm_state_dis, false);

	if (ret)
		pr_err("setting PM state '%s' failed!\n", pm_state_dis->name);
}

static void smb358_detect_irq_worker(struct work_struct *work)
{

	int ret = -1;
	uint8_t cnt;
	u8 data, val;
	int vbus_prev;
	struct smb358_charger *smb =
	    container_of(work, struct smb358_charger, detect_irq_work);

	printk("%s\n", __func__);

	vbus_prev = smb358_dev->vbus;
	smb358_IsPOK(smb358_dev, &smb358_dev->vbus);
	printk("vbus = %d\n", smb358_dev->vbus);

#if 1
	if (smb358_dev->vbus == VBUS_ON) {	/*calbe plugged, set high trigger to listen cable remove event */
		smb358_set_trigger_level(CHARGER_CONTROL_CHDETLVL_HIGH);
	} else {		/*calbe remove event, set low trigger to listen cable plugin event */
		smb358_set_trigger_level(CHARGER_CONTROL_CHDETLVL_LOW);
	}
#endif

	return;
}

struct smb358_charger *smb_i2c_test;

static int proc_smb358_i2c(struct seq_file *m, void *p)
{
	int ret;
#if 0
	seq_printf(m, "Control registers:\n");
	seq_printf(m, "==================\n");
	seq_printf(m, "#Addr\t#Value\n");

	smb358_dump_registers(NULL);
#endif
	ret = smb358_read(smb_i2c_test, CMD_A);

	if (ret < 0) {
		BAT_DBG_E("%s: smb358_read() failed\n", __func__);
		seq_printf(m, "0\n");
	}
	else {
		BAT_DBG_E("%s: smb358_read() = %08x\n", __func__, ret);
		seq_printf(m, "1\n");
	}

	return 0;
}

static int proc_smb358_i2c_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_smb358_i2c, NULL);
}

static ssize_t proc_smb358_i2c_write(struct file *file,
				     const char __user * buffer, size_t count,
				     loff_t * pos)
{
	int ret;
#if 0
	ret = smb358_masked_write(smb_i2c_test->client, CMD_A, BIT(7), BIT(7));
	if (ret < 0) {
		pr_err("failed to write CMD_A\n");
		return ret;
	}
#endif

	if (buffer[0] == '0') {
		BAT_DBG("%s: output pull low cover_chg_en\n", __func__);
		gpio_direction_output(78, 0);
		BAT_DBG("%s: cover_chg_en = %d\n", __func__, gpio_get_value(78));
	}
	else if (buffer[0] == '1') {
		BAT_DBG("%s: output pull high cover_chg_en\n", __func__);
		gpio_direction_output(78, 1);
		BAT_DBG("%s: cover_chg_en = %d\n", __func__, gpio_get_value(78));
	}
	else if (buffer[0] == '2') {
		BAT_DBG("%s: input cover_chg_en\n", __func__);
		gpio_direction_input(78);
		BAT_DBG("%s: cover_chg_en = %d\n", __func__, gpio_get_value(78));
	}
	else if (buffer[0] == '3') {
		BAT_DBG("%s: set cover charger\n", __func__);
		smb358_pre_config();
		smb358_set_cover_charging_current(50, 50);
	}
	else if (buffer[0] == '4') {
		BAT_DBG("%s: enable cover otg\n", __func__);
		smb358_otg_toggle(true);
	}
	else if (buffer[0] == '5') {
		BAT_DBG("%s: disable cover otg\n", __func__);
		smb358_otg_toggle(false);
	}

	return count;
}

static const struct file_operations proc_smb358_i2c_ops = {
	.open = proc_smb358_i2c_open,
	.read = seq_read,
	.write = proc_smb358_i2c_write,
	.llseek = seq_lseek,
	.release = seq_release
};

static int create_smb358_i2c_proc_fs(void)
{
	struct proc_dir_entry *entry = NULL;

	entry =
	    proc_create("driver/CoverChargerIC_Status", 0664, NULL,
			&proc_smb358_i2c_ops);
	if (!entry) {
		printk("[%s] Unable to create CoverChargerIC_Status\n", __FUNCTION__);
		return -EINVAL;
	}

	return 0;
}

static int smb358_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct smb358_charger *smb = &chrgr_data;
	int ret, val = 0;
	int charger_type;
	int mult_i2c_en;

	printk("%s \n", __func__);

#if 0
	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;
#else
	smb->client = client;
#endif

	i2c_set_clientdata(client, smb);

	/*mult_i2c_en = of_get_named_gpio(np, "mult-i2c-en", 0);
	printk("mult_i2c_en = %d\n", mult_i2c_en);
	g_mult_i2c_en = mult_i2c_en;

	ret = gpio_request(mult_i2c_en, "MULT_I2C_EN");
	if (ret < 0) {
		BAT_DBG_E("%s: request MULT_I2C_EN gpio fail!\n", __func__);
	}
	gpio_direction_output(mult_i2c_en, 1);*/

	/* Get cover detect gpio */
	smb->cover_detect_gpio = of_get_named_gpio(np, "asus,cover-irq", 0);
	if (!gpio_is_valid(smb->cover_detect_gpio)) {
		pr_err("can't get gpio: asus,cover-irq\n");
		goto fail;
	}
	BAT_DBG("%s: cover_detect_gpio = %d\n", __func__, smb->cover_detect_gpio);

	/* Get board info ID */
	/* SR:0 ER:1 ER2:2 prePR:3 PR:4 */
	HW_ID = asustek_boardinfo_get(FUN_HARDWARE_ID);
	BAT_DBG("%s: hardware id is %d\n", __func__, HW_ID);

	smb_i2c_test = smb;

	ret = create_smb358_i2c_proc_fs();
	if (ret) {
		pr_err("%s: Unable to create smb358_i2c_proc_fs\n", __func__);
	}

	/* enable register writing - chris */
	ret = smb358_set_writable(smb, true);
	if (ret < 0) {
		BAT_DBG_E("######################### fail \n");
	}
	smb358_dev = smb;

	/* Refer to smb358 Application Note 72 to solve serious problems */
	ret = smb358_masked_write(smb->client,
				  OTG_TLIM_THERM_CNTRL_REG,
				  OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				  SMB358_OTG_CURRENT_LIMIT_250mA);

	smb->dentry =
	    debugfs_create_file("smb358", S_IRUGO, NULL, smb,
				&smb358_debugfs_fops);
	if (smb->dentry == NULL)
		BAT_DBG_E("%s: create debugfs file failed. entry == NULL\n", __func__);
	else if (smb->dentry == -ENODEV)
		BAT_DBG_E("%s: create debugfs file failed. entry == -ENODEV\n", __func__);
	else
		BAT_DBG_E("%s: create debugfs file.\n", __func__);

	sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

	printk("register power supply\n");
	ret = smb358_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;

	/* init wake lock */
	wake_lock_init(&smb->wakelock, WAKE_LOCK_SUSPEND, "smb358_wakelock");
	/* INITIALIZE MUTEX, SEM AND LOCKS  */
	mutex_init(&smb->stat_lock);

	/* charge current control algorithm:
	   config the charge current only when
	   Vbus is legal (a valid input voltage
	   is present)
	 */
	printk("init workqueue\n");
	smb358_charger_work_queue2 = create_singlethread_workqueue("smb358_charger_workqueue2");	//workqueue_struct

	INIT_DELAYED_WORK(&smb358_charger_work2, do_charger2);

	INIT_WORK(&smb->detect_irq_work, smb358_detect_irq_worker);	//work_struct
	INIT_WORK(&smb->interrupt_irq_work, smb358_detect_irq_worker);

	BAT_DBG(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

pre_fail:
fail:

	return 0;
}

static int smb358_remove(struct i2c_client *client)
{
	struct smb358_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	return 0;
}

static int smb358_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __func__);

	return 0;
}

#ifdef CONFIG_PM
static int smb358_prepare(struct device *dev)
{
	struct smb358_charger *smb = dev_get_drvdata(dev);

	dev_info(&smb->client->dev, "smb358 suspend\n");

	return 0;
}

static void smb358_complete(struct device *dev)
{
	struct smb358_charger *smb = dev_get_drvdata(dev);
	dev_info(&smb->client->dev, "smb358 resume\n");
}
#else
#define smb358_prepare NULL
#define smb358_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb358_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb358_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb358_runtime_idle(struct device *dev)
{

	dev_info(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb358_runtime_suspend    NULL
#define smb358_runtime_resume    NULL
#define smb358_runtime_idle    NULL
#endif

#if TF103CE_CHARGER_ACPI
static const struct i2c_device_id smb358_id[] = {
	{"SMB0358:00", 0},
	{},
};
#else
static const struct i2c_device_id smb358_id[] = {
	{"smb358_charger", 0},
	{},
};
#endif

MODULE_DEVICE_TABLE(i2c, smb358_id);

static const struct dev_pm_ops smb358_pm_ops = {
	.prepare = smb358_prepare,
	.complete = smb358_complete,
	.runtime_suspend = smb358_runtime_suspend,
	.runtime_resume = smb358_runtime_resume,
	.runtime_idle = smb358_runtime_idle,
};

static struct i2c_driver smb358_driver = {
	.driver = {
		   .name = "smb358_charger",
		   .owner = THIS_MODULE,
		   .pm = &smb358_pm_ops,
		   },
	.probe = smb358_probe,
	.remove = smb358_remove,
	.shutdown = smb358_shutdown,
	.id_table = smb358_id,
};

static struct i2c_client *i2c_client;
static struct i2c_adapter *i2c_adap;

static struct i2c_board_info smb358_i2c_board_info = {
	.type = "smb358",
	.flags = 0x00,
	.addr = 0x6c,
	//.platform_data = &smb347_pdata,
	.archdata = NULL,
	.irq = -1,
};

#define	SMB358_I2C_ADAPTER	(5)

static int smb358_idi_probe(struct idi_peripheral_device *ididev,
			    const struct idi_device_id *id)
{
	pr_info("%s() - begin\n", __func__);
	pr_info("%s() - end\n", __func__);

	return 0;
}

static int __exit smb358_idi_remove(struct idi_peripheral_device *ididev)
{
	//CHARGER_DEBUG_REL(chrgr_dbg, CHG_DBG_IDI_REMOVE, 0, 0);
	pr_info("%s\n", __func__);

	iounmap(chrgr_data.ctrl_io);
	return 0;
}

static const struct idi_device_id idi_ids[] = {
	{
	 .vendor = IDI_ANY_ID,
	 .device = IDI_DEVICE_ID_INTEL_AG620,
	 .subdevice = IDI_SUBDEVICE_ID_ASUS_CHG,
	 },

	{ /* end: all zeroes */ },
};

static struct idi_peripheral_driver smb358_idi_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "smb358_idi",
		   .pm = NULL,
		   },
	.p_type = IDI_CHG2,
	.id_table = idi_ids,
	.probe = smb358_idi_probe,
	.remove = smb358_idi_remove,
};

static int __init smb358_init(void)
{
	int ret;
	printk("+++++++++++++ [%s] +++++++++++++++\n", __func__);

	ret = idi_register_peripheral_driver(&smb358_idi_driver);
	if (ret)
		return ret;

	ret = i2c_add_driver(&smb358_driver);
	if (ret)
		return ret;

	return 0;
}

module_init(smb358_init);

static void __exit smb358_exit(void)
{
	i2c_del_driver(&smb358_driver);
}

module_exit(smb358_exit);

MODULE_DESCRIPTION("SMB358 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb358");
