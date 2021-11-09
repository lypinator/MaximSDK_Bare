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
 
ifeq "$(LWIP_DIR)" ""
$(error LWIP_DIR must be specified")
endif

# Add to include directory list
IPATH += ${LWIP_DIR}/include
IPATH += ${LWIP_DIR}/include/Maxim

#
# check if REBUILD_LWIP parameters selected or not, 
# if REBUILD_LWIP is not define this means, rebuild
#
ifeq ($(REBUILD_LWIP),$(filter $(REBUILD_LWIP),yes Yes YES))

# Specify the build directory if not defined by the project
ifeq "$(BUILD_DIR)" ""
LWIP_BUILD_DIR=$(CURDIR)/build/LwIP
else
LWIP_BUILD_DIR=$(BUILD_DIR)/LwIP
endif

# Export paths needed by the peripheral driver makefile. Since the makefile to
# build the library will execute in a different directory, paths must be
# specified absolutely
LWIP_BUILD_DIR := ${abspath ${LWIP_BUILD_DIR}}
export TOOL_DIR := ${abspath ${TOOL_DIR}}
export CMSIS_ROOT := ${abspath ${CMSIS_ROOT}}
export PERIPH_DRIVER_DIR := ${abspath ${PERIPH_DRIVER_DIR}}

# Export other variables needed by the peripheral driver makefile
export TARGET
export COMPILER
export TARGET_MAKEFILE
export PROJ_CFLAGS
export PROJ_LDFLAGS
export MXC_OPTIMIZE_CFLAGS

# Add to library list
LIBS += ${LWIP_BUILD_DIR}/liblwip.a

# Add rule to build the Driver Library
${LWIP_BUILD_DIR}/liblwip.a: FORCE
	$(MAKE) -C ${LWIP_DIR} lib BUILD_DIR=${LWIP_BUILD_DIR}
	cp $(LWIP_BUILD_DIR)/liblwip.a $(LWIP_DIR)/liblwip.a

# Rebuild lwIP
else

# Use pre build lwIP library
PROJ_LIBS += lwip
PROJ_LDFLAGS += -L$(LWIP_DIR)

endif # end off build lwIP
	