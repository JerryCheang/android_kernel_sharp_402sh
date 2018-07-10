ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHGRIP_FACTORY_MODE_ENABLE
endif

ifeq ($(CONFIG_MACH_REN),y)
obj-$(CONFIG_SENSOR_SHGRIP) += 002/
else
obj-$(CONFIG_SENSOR_SHGRIP)	+= 001/
endif
