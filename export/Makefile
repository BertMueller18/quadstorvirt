#
# linux 2.6 series Makefile

ifeq ($(KDIR),)
	KDIR ?= /lib/modules/$(shell uname -r)/build
endif

PWD	:= $(shell pwd)

ifeq ($(QUADSTOR_ROOT),)
	QUADSTOR_ROOT := /quadstor/quadstor
endif

ifneq ($(KERNELRELEASE),)

obj-m   += ldev.o coredev.o

ldev-objs := ldev_linux.o devq.o

coredev-objs := core_linux.o corelib.o sha256-ssse3-asm.o sha256_ssse3.o

EXTRA_CFLAGS += -I. -DLINUX -DEXPORT_SYMTAB -I$(KDIR)/drivers/scsi -I$(QUADSTOR_ROOT)/common -I$(QUADSTOR_ROOT)/export

EXTRA_CFLAGS += -g
#EXTRA_CFLAGS += -O2

#EXTRA_CFLAGS += -DENABLE_STATS

else
default:
	cp $(QUADSTOR_ROOT)/lib/modules/corelib.o $(PWD)
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
#	strip --strip-debug *.ko
#	strip --strip-unneeded *.ko
endif

clean:
	rm -rf *.o *.ko .*.cmd *.mod.c .tmp_versions Module.symvers .*.o.d Module.* rm *.unsigned modules.order
clobber: clean

install:
