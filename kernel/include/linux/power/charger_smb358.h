#ifndef _CHARGER_SMB358_H_
#define _CHARGER_SMB358_H_

/*
    there are 10 scale of AC current limit:
	300, 500, 700, 900, 1200, 1500, 1800, 2000, 2200, 2500

    when set a AC current limit, it will auto close to but less than above scale.

    e.g: if set 800 as AC current limit, i will set to 500
*/

int smb358_JEITA_rule_for_system_run(int battery_temperature, int battery_voltage_in_mV);
int smb358_JEITA_rule_for_sleeping_or_shutdown(void);
int smb358_disable_AICL_set_AC_current_limit_enable_AICL(int AC_current_limit);
int smb358_get_AICL_results_return_in_mA(void);
int smb358_enable_otg(void);
int smb358_disable_otg(void);
int smb358_AC_charging_enable_and_set_AC_current_limit(int AC_current_limit);
int smb358_USB_charging_enable(void);
int smb358_usb_and_AC_charging_disable(void);
int smb358_get_register_value(int reg);
int smb358_temp_charging_enable_or_disable(bool charging_enable);

#endif //_CHARGER_SMB358_H_

