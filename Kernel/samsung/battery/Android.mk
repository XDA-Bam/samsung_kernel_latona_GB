##
# Project Name : Samsung Android Build System
#
# Copyright 2009 by Samsung Electronics, Inc.
# All rights reserved.
#
# Project Description :
##

##
# File Name : Android.mk
#
# File Description :
#
# Author : KIM EUN GON (egstyle.kim@samsung.com)
# Dept : Symbian Lab. (Open OS S/W Group)
# Created Date : 04/Dec./2009
# Version : Baby-Raccoon
##

ifeq ($(strip $(SEC_BOARD_USE_BATTERY)),true)

LOCAL_PATH := $(call my-dir)
include $(CLEAR_VARS)

LOCAL_MODULE        := sec_battery_kobject
LOCAL_KERNEL_OBJECT := samsung_battery
LOCAL_MODULE_CLASS  := SEC_KERNEL_OBJECTS
LOCAL_SRC_FILES := \
    battery_monitor.c \
    charger_max8845.c \
    fuelgauge_max17040.c \
    sleep_i2c1.c \
    sleep_i2c2.c \
    sleep_madc.c 
    
LOCAL_ADDITIONAL_DEPENDENCIES := \
    common.h \
#    battery_monitor.h \
#    charger_max8845.h \
#    fuelgauge_max17040.h

LOCAL_CFLAGS += -DCONFIG_SEC_BATTERY_USE_RECOVERY_MODE
#include vendor/samsung/build/sec_kobject.mk

endif   ## $(SEC_BOARD_USE_BATTERY)








