MODULE = powerpc/fsl_pq_mdio.ko e500/fsl_pq_mdio.ko e500-smp/fsl_pq_mdio.ko

CFLAGS += -DCONFIG_FSL_FMAN
fsl_pq_mdio_FILES = fsl_pq_mdio.c
fsl_pq_mdio_PREFIX = drivers/net

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
