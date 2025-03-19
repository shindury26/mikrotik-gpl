MODULE = arm/qca_crypto.ko

qca_crypto_FILES = qce50.c qcrypto.c
qca_crypto_PREFIX = drivers/crypto
#qca_crypto_CFLAGS = -DDEBUG

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
