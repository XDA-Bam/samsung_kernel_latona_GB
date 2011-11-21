obj-m := samsung_battery.o
samsung_battery-y :=
ifeq ($(CONFIG_MACH_SAMSUNG_LATONA),y)
samsung_battery-y += battery_monitor.o
samsung_battery-y += charger_max8845.o
samsung_battery-y += fuelgauge_max17040.o
else
samsung_battery-y += battery_monitor_tab.o
samsung_battery-y += charger_smb136.o
samsung_battery-y += fuelgauge_max17042.o
endif
samsung_battery-y += sleep_i2c1.o
samsung_battery-y += sleep_i2c2.o
samsung_battery-y += sleep_madc.o


EXTRA_CFLAGS := -DCONFIG_SEC_BATTERY_USE_RECOVERY_MODE
clean:
	@rm -rf *.ko *.mod.* .*.cmd *.o .tmp_versions 	modules.order Module.symvers