#ifndef _GAUGE_BQ27520_H_
#define _GAUGE_BQ27520_H_

typedef unsigned short u16;

struct gauge_bq27520_platform_data_struct{
	int energy_full_design;
	int voltage_max_design;
	int voltage_min_design;
	int ADC_alert_pin;
};

int bq27520_get_battery_charging_status(void);
int bq27520_get_battery_voltage(void);
int bq27520_get_battery_capacity_in_percent(void);
int bq27520_get_battery_temperature(void);
int bq27520_get_battery_current(void);
int bq27520_get_battery_full_capacity_in_mAh(void);
int bq27520_get_battery_capacity_in_mAh(void);
int bq27520_get_battery_time_to_empty(void);
int bq27520_get_battery_cycle_count(void);
int bq27520_get_battery_flags(void);
int bq27520_get_battery_health_or_present(void);
int bq27520_get_battery_energy_full_in_watt_design(void);
int bq27520_get_battery_voltage_max_design(void);
int bq27520_get_battery_voltage_min_design(void);

// when return value <0, means didn't have ADC_alert
int bq27520_get_gauge_ADC_alert_pin(void);

// get gaugeIC FW information
u16 bq27520_get_gauge_FW_control_status(void);
u16 bq27520_get_gauge_FW_device_type(void);
u16 bq27520_get_gauge_FW_firmware_version(void);
u16 bq27520_get_gauge_FW_chemical_ID(void);
u16 bq27520_get_gauge_FW_DF_version(void);

int bq27520_get_if_pad_gaugeIC_firmware_is_updated(void);


// get batteryID from PMIC
u16 bq27520_get_batteryID_calculate_from_pmic(void);

#endif // _GAUGE_BQ27520_H
