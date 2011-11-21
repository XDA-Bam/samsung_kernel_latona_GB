#
# Makefile for the FM radio driver
#

obj-m += Si4709_driver.o

Si4709_driver-objs += Si4709_dev.o
Si4709_driver-objs += Si4709_i2c_drv.o
Si4709_driver-objs += Si4709_main.o

all:
	@echo "$(MAKE) --no-print-directory -C $(KDIR) SUBDIRS=$(CURDIR) modules"
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules

clean:
	rm -f *.o *.ko *.mod.c *.mod.o *~ .*.cmd .*.o.cmd Module.symvers modules.order 

install:
	@$(MAKE) --no-print-directory -C $(KDIR) \
		SUBDIRS=$(CURDIR) modules_install