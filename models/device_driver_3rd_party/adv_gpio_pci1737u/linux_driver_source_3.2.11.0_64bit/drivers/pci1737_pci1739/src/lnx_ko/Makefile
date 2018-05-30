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
DRV_NAME := bio1737
DRV_PATH := daqnavi

# KBuild part of makefile--------------------------------------------------------
ifneq ($(KERNELRELEASE),)

obj-m	:= $(DRV_NAME).o
$(DRV_NAME)-objs := init.o fops.o dio.o isr.o 

EXTRA_CFLAGS += -I$(src)/../../../inc -I$(src)/../../../inc/hw -I$(src)/../../../inc/linux

# Normal makefile----------------------------------------------------------------
else 
KVER := $(shell uname -r)
KSRC ?= /lib/modules/$(KVER)/build
KMOD ?= /lib/modules/$(KVER)/kernel/drivers/$(DRV_PATH)

DRV_LIB := ../../../lib
DRV_BIN := ../../../bin

all:
	-cp $(DRV_LIB)/Module.symvers ./
	$(MAKE) -C $(KSRC) M=$(PWD) modules
	-cp $(DRV_NAME).ko $(DRV_BIN)/

install:
	@echo "Intalling the advantech driver, please wait..."
	$(shell ! [ -d $(KMOD) ] && mkdir $(KMOD))
	$(shell cp -f ./$(DRV_NAME).ko $(KMOD)/ && depmod && modprobe $(DRV_NAME))
	@echo "Done"

uninstall:
	$(shell if grep $(DRV_NAME) /proc/modules > /dev/null ; then \
	 rmmod $(DRV_NAME) ; fi)
	$(shell rm -f $(KMOD)/$(DRV_NAME).ko && depmod)
	
clean:
	$(MAKE) -C $(KSRC) M=$(PWD) clean
	rm -rf *.o *~ .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order Module.symvers

.PHONY: all clean

endif