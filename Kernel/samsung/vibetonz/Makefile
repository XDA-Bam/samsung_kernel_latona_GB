#
# Makefile for the touch sense(vibetonz) driver
#

obj-m += vibetonz.o

vibetonz-objs += tspdrv.o 

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd .*.o.cmd Module.symvers modules.order 

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install