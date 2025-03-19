MODULE = ulog.ko ipt_ulog.ko ebt_ulog.ko

ulog_FILES = ulog.c
ulog_PREFIX = misc

ipt_ulog_FILES = ipt_ulog.c
ipt_ulog_PREFIX = net/ipv4/netfilter

ebt_ulog_FILES = ebt_ulog.c
ebt_ulog_PREFIX = net/bridge

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
