##
# Copyright (C) 2010, Samsung Electronic, Co., Ltd. All Rights Reserved.
#  Written by System S/W Group, Open OS S/W R&D Team,
#  Mobile Communication Division.
##

##
# Project Name : OMAP-Samsung Linux Kernel for Android
#
# Proejct Description :
#
# Comments : tabstop = 8, shiftwidth = 8, noexpandtab
##

##
# File Name : samsung.mk
#
# File Description :
#
# Author : System Platform 2
# Dept : System S/W Group (Open OS S/W R&D Team)
# Created : 25/Nov/2011
# Version : Baby-Raccoon
##

ifeq ($(machine-y),omap2)

omap_samsung_plat = arch/arm/plat-$(word 1,$(plat-y))/
omap_samsung_plat_inc_path = $(omap_samsung_plat)include

ifdef CONFIG_SAMSUNG_HW_EMU_BOARD
OMAP_SAMSUNG_MUX_FILE_NAME = $(shell printf "mux_%s_rev_e%02d" $(CONFIG_SAMSUNG_BOARD_NAME) $(CONFIG_SAMSUNG_EMU_HW_REV))
else
OMAP_SAMSUNG_MUX_FILE_NAME = $(shell printf "mux_%s_rev_r%02d" $(CONFIG_SAMSUNG_BOARD_NAME) $(CONFIG_SAMSUNG_REL_HW_REV))
endif

export OMAP_SAMSUNG_MUX_FILE_NAME

omap_smasung_mux_cflags = \
	-include $(shell printf "%s/plat/%s.h" $(omap_samsung_plat_inc_path) $(OMAP_SAMSUNG_MUX_FILE_NAME)) \
	-D$(shell printf "_SAMSUNG_BOARD_NAME=%s" $(CONFIG_SAMSUNG_BOARD_NAME))

cflags-y += $(omap_smasung_mux_cflags)
KBUILD_CFLAGS += $(cflags-y)

endif