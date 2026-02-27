# SPDX-License-Identifier: GPL-3.0-or-later
# Makefile for Photonicat PM driver DKMS module

obj-m += photonicat-pm.o
photonicat-pm-y := pcat-pm-core.o \
                   pcat-pm-uart.o \
                   pcat-pm-power.o \
                   pcat-pm-rtc.o \
                   pcat-pm-hwmon.o \
                   pcat-pm-fan.o \
                   pcat-pm-ctl.o \
                   pcat-pm-sysfs.o

# Kernel build directory
KDIR ?= /lib/modules/$(shell uname -r)/build

# Module installation path
INSTALL_MOD_PATH ?= /
INSTALL_MOD_DIR ?= extra

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install:
	$(MAKE) -C $(KDIR) M=$(CURDIR) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) INSTALL_MOD_DIR=$(INSTALL_MOD_DIR) modules_install

.PHONY: all clean install
