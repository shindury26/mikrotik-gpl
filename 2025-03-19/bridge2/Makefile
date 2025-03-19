MODULE = bridge2.ko bridge2_netfilter.ko

bridge2_FILES = br_fdb.c br_input.c br_forward.c br_if.c \
		br_device.c br_ioctl.c br.c br_nf_core.c fast_path.c \
		br_multicast.c br_vlan.c br_netlink.c \
		br_dhcp_snooping.c br_vlan_tunnel.c br_netlink_tunnel.c br_vlan_options.c \
		br_mdb.c br_switchdev.c

bridge2_PREFIX = net/bridge
bridge2_CFLAGS += -O2 --param=max-inline-insns-auto=40

bridge2_netfilter_FILES = br_nf_core.c br_netfilter_hooks.c br_netfilter_ipv6.c
bridge2_netfilter_PREFIX = net/bridge


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
