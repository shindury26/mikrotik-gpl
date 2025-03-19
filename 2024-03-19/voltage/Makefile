MODULE = powerpc/voltage.ko mips/voltage.ko smips/voltage.ko 440/voltage.ko \
	e500/voltage.ko e500-smp/voltage.ko

440/voltage_FILES = main.c sampler.c voltage-ppc.c rb333-voltage.c \
	rb800-voltage.c rb1100-voltage.c rb1200-voltage.c
powerpc/voltage_FILES = main.c sampler.c voltage-ppc.c rb333-voltage.c \
	rb800-voltage.c rb1100-voltage.c rb1120-voltage.c
e500/voltage_FILES = $(powerpc/voltage_FILES)
e500-smp/voltage_FILES = $(powerpc/voltage_FILES)
mips/voltage_FILES = main.c sampler.c voltage-mips.c rb400-common.c rb450G.c \
	ath8316.c
smips/voltage_FILES = $(mips/voltage_FILES)

voltage_PREFIX = misc

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
