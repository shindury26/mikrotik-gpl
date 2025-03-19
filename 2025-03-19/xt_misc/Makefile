MODULE = xt_misc.ko nf_conntrack_ipv4.ko nf_conntrack_ipv6.ko

xt_misc_FILES = xt_misc.c
xt_misc_PREFIX = net/netfilter

nf_conntrack_ipv4_FILES = nf_conntrack_ipv4.c
nf_conntrack_ipv4_PREFIX = net/netfilter

nf_conntrack_ipv6_FILES = nf_conntrack_ipv6.c
nf_conntrack_ipv6_PREFIX = net/netfilter

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
