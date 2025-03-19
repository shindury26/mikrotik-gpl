MODULE = x86_64/atl1c.ko \
	powerpc/atl1c.ko 440/atl1c.ko e500/atl1c.ko e500-smp/atl1c.ko \
	tile/atl1c.ko arm64/atl1c.ko arm/atl1c.ko

atl1c_FILES = atl1c.c
atl1c_PREFIX = drivers/net

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
