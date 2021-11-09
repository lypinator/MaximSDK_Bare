################################################################################
 # Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
 #
 # Permission is hereby granted, free of charge, to any person obtaining a
 # copy of this software and associated documentation files (the "Software"),
 # to deal in the Software without restriction, including without limitation
 # the rights to use, copy, modify, merge, publish, distribute, sublicense,
 # and/or sell copies of the Software, and to permit persons to whom the
 # Software is furnished to do so, subject to the following conditions:
 #
 # The above copyright notice and this permission notice shall be included
 # in all copies or substantial portions of the Software.
 #
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 # OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 # MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 # IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 # OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 # ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 # OTHER DEALINGS IN THE SOFTWARE.
 #
 # Except as contained in this notice, the name of Maxim Integrated
 # Products, Inc. shall not be used except as stated in the Maxim Integrated
 # Products, Inc. Branding Policy.
 #
 # The mere transfer of this software does not imply any licenses
 # of trade secrets, proprietary technology, copyrights, patents,
 # trademarks, maskwork rights, or any other form of intellectual
 # property whatsoever. Maxim Integrated Products, Inc. retains all
 # ownership rights.
 #
 ###############################################################################

# This is the name of the build output file
PROJECT=liblwip

ifeq "$(TARGET)" ""
$(error TARGET must be specified)
endif

TARGET_UC:=$(shell echo $(TARGET) | tr a-z A-Z)
TARGET_LC:=$(shell echo $(TARGET) | tr A-Z a-z)
$(info $(TARGET_UC))

ifeq "$(COMPILER)" ""
$(error COMPILER must be specified)
endif

# This is the path to the peripheral drivers
ifeq "$(PERIPH_DRIVER_DIR)" ""
PERIPH_DRIVER_DIR=../PeriphDrivers/$(TARGET_UC)
endif

ifeq "$(BUILD_DIR)" ""
BUILD_DIR=./Build
endif

# This is the path to the CMSIS root directory
ifeq "$(CMSIS_ROOT)" ""
CMSIS_ROOT=../CMSIS
endif

# Source files for this test (add path to VPATH below)
SRCS  = $(sort $(wildcard api/*.c))
SRCS += $(sort $(wildcard core/*.c))
SRCS += $(sort $(wildcard core/ipv4/*.c))
SRCS += $(sort $(wildcard core/ipv6/*.c))
SRCS += $(sort $(wildcard netif/*.c))
SRCS += $(sort $(wildcard netif/ppp/*.c))
SRCS += $(sort $(wildcard netif/ppp/polarssl/*.c))
SRCS += $(sort $(wildcard Maxim/*.c))

# Where to find source files for this project
VPATH  = api
VPATH += core
VPATH += core/ipv4
VPATH += core/ipv6
VPATH += netif
VPATH += netif/ppp
VPATH += netif/ppp/polarssl
VPATH += Maxim

# Where to find header files for this project
IPATH  = include
IPATH += include/compat
IPATH += include/compat/posix
IPATH += include/compat/posix/arpa
IPATH += include/compat/posix/net
IPATH += include/compat/posix/sys
IPATH += include/compat/stdc
IPATH += include/lwip
IPATH += include/lwip/apps
IPATH += include/lwip/priv
IPATH += include/lwip/prot
IPATH += include/Maxim
IPATH += include/Maxim/arch
IPATH += include/netif
IPATH += include/netif/ppp
IPATH += include/netif/ppp/polarssl
IPATH += $(PERIPH_DRIVER_DIR)/Include/$(TARGET_UC)

# Open compile guard flag
PROJ_CFLAGS+=-DWITH_LWIP

# Include the rules for building for this target
include $(CMSIS_ROOT)/Device/Maxim/$(TARGET_UC)/Source/$(COMPILER)/$(TARGET_LC).mk

# Build this as a library
.DEFAULT_GOAL := lib
