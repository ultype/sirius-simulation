#--------------------------------------------------------------------------------
# DAQNavi3 Makefile, VER 3.1
#    This is a general purpose makefile for various kernel versions and ARCHs.
#
# To make driver for current running kernel:
#    make
#    sudo make install
#
# To make driver for other kernel version and ARCHs(cross-compiling), the kernel
# source(or kernel headers) and cross compiler prefix must be supplied:
#    make KSRC='kernel source' CROSS_COMPILE='cross-compiler prefix'
# Note: the make target 'install' and 'uninstall' are not available.
#
#--------------------------------------------------------------------------------

# Common part--------------------------------------------------------------------
DRV_NAME := biokernbase
DRV_PATH := daqnavi

# KBuild part of makefile--------------------------------------------------------
ifneq ($(KERNELRELEASE),)

obj-m := $(DRV_NAME).o
$(DRV_NAME)-objs := main.o event.o usbreader.o memory.o misc.o isa.o

EXTRA_CFLAGS += -I$(src)/../../../inc -I$(src)/../../../inc/hw -I$(src)/../../../inc/linux

# Normal makefile----------------------------------------------------------------
else
KVER := $(shell uname -r)
KSRC ?= /lib/modules/$(KVER)/build
KMOD ?= /lib/modules/$(KVER)/kernel/drivers/$(DRV_PATH)

DRV_LIB := ../../../lib
DRV_BIN := ../../../bin

UDEV_PATH := /etc/udev/rules.d

all:
	$(MAKE) -C $(KSRC) M=$(PWD) modules
	mkdir -p $(DRV_BIN) && cp $(DRV_NAME).ko $(DRV_BIN)/
	mkdir -p $(DRV_LIB) && cp Module.symvers $(DRV_LIB)/

install:
	@echo "Intalling the advantech driver, please wait..."
	-cp 71-bionic-daq.rules $(UDEV_PATH)/
	$(shell ! [ -d $(KMOD) ] && mkdir $(KMOD))
	$(shell cp -f ./$(DRV_NAME).ko $(KMOD)/ && depmod && modprobe $(DRV_NAME))
	@echo "Done"

uninstall:
	$(shell if grep $(DRV_NAME) /proc/modules > /dev/null ; then \
	 rmmod $(DRV_NAME) ; fi)
	$(shell rm -f $(KMOD)/$(DRV_NAME).ko && depmod)
	rm -f $(UDEV_PATH)/71-bionic-daq.rules

clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

.PHONY: all clean

endif