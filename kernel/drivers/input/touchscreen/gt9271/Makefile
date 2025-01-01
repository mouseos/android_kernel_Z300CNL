#
# Makefile for the GT9271 touchscreen driver.
#
ifeq ($(TARGET_BUILD_VARIANT), eng)
	KBUILD_CFLAGS += -DFACTORY_IMAGE=1
else
	KBUILD_CFLAGS += -DFACTORY_IMAGE=0
endif

ifeq ($(TARGET_BUILD_VARIANT), user)
	KBUILD_CFLAGS += -DUSER_IMAGE=1
else
	KBUILD_CFLAGS += -DUSER_IMAGE=0
endif

ifeq ($(TARGET_PROJECT), FE375CL)
	KBUILD_CFLAGS += -DSUPPORT_GESTURE=1
else
	KBUILD_CFLAGS += -DSUPPORT_GESTURE=0
endif

obj-$(CONFIG_TOUCHSCREEN_GOODIX_GT9271) += gt9xx.o gt9xx_update.o goodix_tool.o gt9xx_openshort.o
