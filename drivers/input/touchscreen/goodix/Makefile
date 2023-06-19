# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the goodix touchscreen drivers.
#
ifeq ($(filter $(strip $(CONFIG_TOUCHSCREEN_GOODIX_BRL)$(CONFIG_TOUCHSCREEN_GOODIX_GT738X)),m y),)
  $(error Driver for Goodix Berlin and GT738x must not be enabled at the same time)
endif
obj-$(CONFIG_TOUCHSCREEN_GOODIX_BRL)	+= brl/
obj-$(CONFIG_TOUCHSCREEN_GOODIX_GT738X)	+= gt738x/