#ifndef _SMB345_EXTERNAL_INCLUDE_H
#define _SMB345_EXTERNAL_INCLUDE_H 1

#include <linux/seq_file.h>
enum {
    USB_IN,
    AC_IN,
    CABLE_OUT,
    ENABLE_5V,
    DISABLE_5V,
    PAD_SUPPLY,
};

typedef enum {
    BALANCE = 0,
    JEITA,
    FLAGS,
} charging_toggle_level_t;


enum {
    TEMP_1P5,
    TEMP_1P5_15,
    TEMP_15_50,
    TEMP_50_55,
    TEMP_55,
};

/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB345Charger(int usb_state);
extern int setSMB358Charger(int usb_state);
extern int pad_in_otg(int otg_state);
extern int request_power_supply_changed(void);
extern int smb358_request_power_supply_changed(void);
extern int get_charger_type(void);
extern int smb358_get_charger_type(void);
extern bool get_sw_charging_toggle(void);
extern bool smb358_get_sw_charging_toggle(void);
extern void aicl_dete_worker(struct work_struct *dat);
extern void smb358_aicl_dete_worker(struct work_struct *dat);
extern bool external_power_source_present(void);
extern bool smb358_external_power_source_present(void);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb345_get_charging_status(void);
extern int smb358_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb345_charging_toggle(charging_toggle_level_t level, bool on);
extern int smb358_charging_toggle(charging_toggle_level_t level, bool on);

/* To know if charger has an error
 *
 * return true means charger has an error */
bool smb345_has_charger_error(void);
bool smb358_has_charger_error(void);

/* To acquire the value from charger ic registers
 *
 * return nothing */
int smb345_dump_registers(struct seq_file *s);
int smb358_dump_registers(struct seq_file *s);

/* To acquire the value from pmic registers
 *
 * return nothing */
int pmic_dump_registers(int dump_mask, struct seq_file *s);

int pmic_get_batt_id(void);

/* To config hard/soft limit cell temperature
 * monitor for soc control JEITA function
 *
 * return 0 means config completely */
int smb345_soc_control_jeita(void);
int smb358_soc_control_jeita(void);

/* To config hard/soft limit cell temperature
 * monitor for charger control JEITA function
 *
 * return 0 means config completely */
int smb345_charger_control_jeita(void);
extern int smb358_charger_control_jeita(void);

/* To config float voltage. Must aquired
 * the battery temperature from Gauge IC
 *
 * return 0 means config completely */
int smb345_soc_control_float_vol(int bat_temp);
int smb358_soc_control_float_vol(int bat_temp);

/* For cover using */
#define COVER_ON 1
#define COVER_OFF 0
#define VBUSDET_CABLE_ON 0
#define VBUSDET_CABLE_OFF 1
static inline int get_pack_bat_rsoc(int *rsoc);
extern int smb358_pre_config(void);
extern int smb358_pre_config_usb(void);
extern int smb358_set_cover_charging_current(int, int);
extern int smb358_otg_toggle(bool);
extern int smb358_do_cover_jeita(void);
extern int smb358_get_aicl_results(void);
extern int mult_function_n;
extern void cover_detect_register(char *,char *, void(*)(bool));
extern void force_upi_ug31xx_power_supply_change(void);
extern void force_upi_ug31xx_battery_external_power_changed(void);
extern void smb358_set_usb_in_500ma(void);
extern bool smb358_is_cover_otg(void);
extern int smb358_get_battery_current(int *);
extern void smb358_extern_set_writable(void);
extern int smb358_aicl_toggle(bool toggle);
extern int smb358_suspend_charger_usbin(bool toggle);
extern int smb358_set_charger_usbin(int cur);

#define LG "1"
#define COSLIGHT "0"
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
#define SMB345_DEV_NAME "smb346"
#else
#define SMB345_DEV_NAME "smb345"
#define SMB358_DEV_NAME "smb358"
#endif
#define MSIC_CHRG_REG_DUMP_OTHERS	(1 << 3)

/* battery temperature unit: 0.1C */
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD         500
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD_PF400CG 450

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  600
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 450
#elif defined(CONFIG_FE170CG)
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#else
#define STOP_CHARGING_LOW_TEMPERATURE   0
#define STOP_CHARGING_HIGH_TEMPERATURE  550
#define FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD 500
#endif

#endif /* _SMB345_EXTERNAL_INCLUDE_H */

