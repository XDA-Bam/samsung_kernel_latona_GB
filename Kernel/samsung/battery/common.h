#ifndef _COMMON_H_
#define _COMMON_H_

//#define WR_ADC 


#include <linux/slab.h> // Daegil.im 2011-02-10 for build

#ifdef WR_ADC
/* Workaround to get proper adc value */
#include <linux/i2c/twl.h>
#endif

// Unit is sec.
#define MONITOR_DURATION_DUR_SLEEP		30
#define MONITOR_DEFAULT_DURATION		30
#define MONITOR_TEMP_DURATION			30
#define MONITOR_RECHG_VOL_DURATION		30


#ifdef CONFIG_MACH_SAMSUNG_P1WIFI
	#define DEFAULT_CHARGING_TIMEOUT        6 * 60 * 60
	#define DEFAULT_RECHARGING_TIMEOUT        2 * 60 * 60

	#ifdef CONFIG_SAMSUNG_LOCALE_USAGSM
    	#define CHARGE_STOP_TEMPERATURE_MAX      46000  // 46
    	#define CHARGE_RECOVER_TEMPERATURE_MAX      43000  // 43

    	#define CHARGE_STOP_TEMPERATURE_EVENT  46000  // 63
    	#define CHARGE_RECOVER_TEMPERATURE_EVENT    43000  // 58
	#else
    	#define CHARGE_STOP_TEMPERATURE_MAX      54500  // 55
    	#define CHARGE_RECOVER_TEMPERATURE_MAX      44500  // 45

    	#define CHARGE_STOP_TEMPERATURE_EVENT  54500  // 63
    	#define CHARGE_RECOVER_TEMPERATURE_EVENT    44500  // 58
	#endif

	#define CHARGE_STOP_TEMPERATURE_MIN			-1000		// 0
	#define CHARGE_RECOVER_TEMPERATURE_MIN      1800		// 3
	#define CHARGE_RECHG_VOLTAGE            	4140
#else
	#define DEFAULT_CHARGING_TIMEOUT        6 * 60 * 60
	#define DEFAULT_RECHARGING_TIMEOUT        (1 * 60 * 60 + 30 * 60)

	#define CHARGE_STOP_TEMPERATURE_MAX     	65			// 45
	#define CHARGE_RECOVER_TEMPERATURE_MAX      55		// 40

	#define CHARGE_STOP_TEMPERATURE_MIN     	-3			// 0
	#define CHARGE_RECOVER_TEMPERATURE_MIN      1	// 3
	#define CHARGE_RECHG_VOLTAGE            	4110
	#define CHARGE_RECHG_VOLTAGE_OFFMODE       	4110

	#define CHARGE_STOP_TEMPERATURE_EVENT		67		// 63
	#define CHARGE_RECOVER_TEMPERATURE_EVENT    60		// 58
#endif


#define CHARGE_FULL_CURRENT_ADC 250

#define CHARGE_DUR_ACTIVE 0
#define CHARGE_DUR_SLEEP  1

#define STATUS_CATEGORY_CABLE       1
#define STATUS_CATEGORY_CHARGING    2
#define STATUS_CATEGORY_TEMP        3
#define STATUS_CATEGORY_ETC         4

enum {
    /* power_supply.h
    POWER_SUPPLY_STATUS_UNKNOWN = 0,
    POWER_SUPPLY_STATUS_CHARGING,
    POWER_SUPPLY_STATUS_DISCHARGING,
    POWER_SUPPLY_STATUS_NOT_CHARGING,
    POWER_SUPPLY_STATUS_FULL,
    */
    POWER_SUPPLY_STATUS_CHARGING_OVERTIME = 5,
    POWER_SUPPLY_STATUS_RECHARGING_FOR_FULL = 6,
    POWER_SUPPLY_STATUS_RECHARGING_FOR_TEMP = 7,
    POWER_SUPPLY_STATUS_FULL_DUR_SLEEP = 8,
};

enum {
    BATTERY_TEMPERATURE_NORMAL = 0,
    BATTERY_TEMPERATURE_LOW,
    BATTERY_TEMPERATURE_HIGH,
};

enum {
    ETC_CABLE_IS_DISCONNECTED = 0,
};

typedef struct
{
    bool ready;

    /*Battery info*/
    struct _battery 
    {
        int  battery_technology;
        int  battery_level_ptg;
        int  battery_level_vol;
        int  battery_temp;
        int  battery_health;
        bool battery_vf_ok;
        bool battery_vol_toolow;

        int  monitor_duration;
        bool monitor_field_temp;
        bool monitor_field_rechg_vol;
        int  support_monitor_temp;
        int  support_monitor_timeout;
        int  support_monitor_full;

        int confirm_full_by_current;
        int confirm_recharge;
#ifdef CONFIG_MACH_SAMSUNG_P1WIFI
		int low_batt_comp_flag;
#endif
    }battery;

    /*Charger info*/
    struct _charger
    {
        int prev_cable_status;
        int cable_status;
#ifdef CONFIG_MACH_SAMSUNG_P1WIFI
		bool samsung_charger;
#endif
        int prev_charge_status;
        int charge_status;

        // 0x0: No Full, 0x1: Full, 0x2: 5Hours
        char full_charge_dur_sleep;
        bool is_charging;

        unsigned long long  charge_start_time;
        unsigned long charged_time;
        int charging_timeout;

        bool use_ta_nconnected_irq;

        // for adjust fuelgauge
        int fuelgauge_full_soc;
		int rechg_count;
    }charger; 

    struct workqueue_struct *sec_battery_workq;
}SEC_battery_charger_info;

#endif

