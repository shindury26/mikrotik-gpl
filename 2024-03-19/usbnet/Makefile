MODULE += tile/fp_usbnet.ko
MODULE += mips/fp_usbnet.ko
MODULE += mmips/fp_usbnet.ko
MODULE += arm/fp_usbnet.ko
MODULE += arm64/fp_usbnet.ko
MODULE += x86_64/fp_usbnet.ko

fp_usbnet_FILES = usbnet.c cdc_ether.c rndis_host.c \
		    sim7600.c asix.c mcs7830.c
fp_usbnet_PREFIX = drivers/net/usb/

ifneq ($(KERNELRELEASE),)

EXTRA_CFLAGS := $(CFLAGS)

define moddef
obj-m += $(1).o
ifneq ($(1).o, $(2))
$(1)-y := $(2)
EXTRA_CFLAGS += $($(1)_CFLAGS)
endif
endef

$(foreach m, $(MODULE), $(eval $(call moddef,$(m:.ko=),$($(m:.ko=)_FILES:.c=.o))))

EXTRA_CFLAGS := $(subst -I, -I$(src)/, $(EXTRA_CFLAGS))

else

KERNELDIR := /lib/modules/$$(uname -r)/build

all::
	$(MAKE) -C $(KERNELDIR) M=$$(pwd) $@

endif
