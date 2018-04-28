#########################################################################
#
# MODULE:   JN-AN-1065-JenNet-Home-Sensor-Demo
#
# DESCRIPTION: AN1065_JN_HomeSensorEndD MakeFile
#
############################################################################
# 
# This software is owned by Jennic and/or its supplier and is protected
# under applicable copyright laws. All rights are reserved. We grant You,
# and any third parties, a license to use this software solely and
# exclusively on Jennic products. You, and any third parties must reproduce
# the copyright and warranty notice and any other legend of ownership on each
# copy or partial copy of the software.
# 
# THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
# EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
# ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
# BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
# INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
# 
# Copyright Jennic Ltd 2009. All rights reserved
#
#############################################################################
# Subversion variables
# $HeadURL:  $
# $Revision: 13257 $
# $LastChangedBy: pjtw $
# $LastChangedDate: 2009-05-26 16:11:24 +0100 (Tue, 26 May 2009) $
# $Id: Makefile 13257 2009-05-26 15:11:24Z mwild $ 
#
#############################################################################

# Application target name

TARGET = AN1065_JN_HomeSensorEndD

#  Note: Target name must be the same as the subfolder name
##############################################################################
#User definable make parameters that may be overwritten from the command line

# Default target device is the JN5139
JENNIC_CHIP ?= JN5139

##############################################################################
# Default DK2 development kit target hardware

# Set DK1 board for JN5121
ifeq ($(JENNIC_CHIP),JN5121)
   CFLAGS += -DBUILD_JN5121
   JENNIC_PCB  ?= DEVKIT1
else
   JENNIC_PCB ?= DEVKIT2
endif

ifeq ($(HIGH_POWER),TRUE)
BIN_SUFFIX ?= _hp
$(info Building High Power version ...)
CFLAGS += -DHIGH_POWER=TRUE
endif
##############################################################################
# Select the network stack (e.g. MAC, ZBPRO)

JENNIC_STACK ?= JN
# For compatibility with V1 makefile
JENIE_IMPL    ?= JN
# Sepcify device type (e.g. CR (Coordinator/router), ED (End Device))
DEVICE_TYPE ?= ED

##############################################################################
# Debug options - define DEBUG for debug type
#DEBUG ?=SW
#
#
# Define which UART to use for debug
DEBUG_PORT ?= UART1

##############################################################################
# Define TRACE to use with DBG module
#TRACE ?=1

##############################################################################
# Path definitions
# Select definitions for either single or multiple targets

# Use if application directory contains multiple targets
SDK_BASE_DIR   	 	= $(abspath ../../../..)
APP_BASE            	 	= $(abspath ../..)
APP_BLD_DIR			= $(APP_BASE)/$(TARGET)/Build
APP_SRC_DIR 	        	= $(APP_BASE)/$(TARGET)/Source
APP_COMMON_SRC_DIR = $(APP_BASE)/Common/Source


# Use if application directory contains single target
#SDK_BASE_DIR         	= $(abspath ../../..)
#APP_BASE            	= $(abspath ..)
#APP_BLD_DIR          	= $(APP_BASE)/Build
#APP_SRC_DIR          	= $(APP_BASE)/Source

##############################################################################
# Application Source files

# Note: Path to source file is found using vpath below, so only .c filename is required
APPSRC += Utils.c
APPSRC += $(TARGET).c

##############################################################################
# Additional Application Source directories
# Define any additional application directories outside the application directory
# e.g. for AppQueueApi

ADDITIONAL_SRC_DIR += $(SDK_BASE_DIR)/Common/Source

##############################################################################
# Standard Application header search paths
# JenieAppConfig must go before other INCFLAG definitions on JN5139
BASE_DIR ?= $(SDK_BASE_DIR)

include $(SDK_BASE_DIR)/Jenie/Library/JenieAppConfig.mk


INCFLAGS += -I$(APP_SRC_DIR)
INCFLAGS += -I$(APP_SRC_DIR)/..
INCFLAGS += -I$(APP_COMMON_SRC_DIR)

# Application specific include files
#Set v1 style include paths
INCFLAGS += -I$(SDK_BASE_DIR)/Chip/Common/Include
INCFLAGS += -I$(SDK_BASE_DIR)/Common/Include
ifeq ($(JENNIC_PCB),DEVKIT1)
   INCFLAGS += -I$(SDK_BASE_DIR)/Platform/DK1/Include 
else
   INCFLAGS += -I$(SDK_BASE_DIR)/Platform/DK2/Include 
endif
INCFLAGS += -I$(SDK_BASE_DIR)/Platform/Common/Include

INCFLAGS += -I$(COMPONENTS_BASE_DIR)/AppQueueApi/Include 

INCFLAGS += -I$(SDK_BASE_DIR)/Jenie/Include 

##############################################################################
# Application libraries
# Specify additional Component libraries


##############################################################################

# You should not need to edit below this line

##############################################################################
##############################################################################
# Configure for the selected chip or chip family

# Call v1 style config.mk
#BASE_DIR=$(SDK_BASE_DIR)
include $(SDK_BASE_DIR)/Common/Build/config.mk

##############################################################################
INCFLAGS += -I$(SDK_BASE_DIR)/Chip/$(JENNIC_CHIP_FAMILY)/Include

APPOBJS = $(APPSRC:.c=.o)

##############################################################################
# Application dynamic dependencies

APPDEPS = $(APPOBJS:.o=.d)

#########################################################################
# Linker

# Add application libraries before chip specific libraries to linker so
# symbols are resolved correctly (i.e. ordering is significant for GCC)

#LDLIBS := $(addsuffix _$(JENNIC_CHIP_FAMILY),$(APPLIBS)) $(LDLIBS)

# Set linker variables from V1 Style makefile
BOARDAPI_LIB  = $(BOARDAPI_BASE)/Library
BOARD_LIB     = BoardLib_$(JENNIC_CHIP_FAMILY)

STACK_BASE    = $(BASE_DIR)/Chip/$(JENNIC_CHIP_FAMILY)
STACK_BLD     = $(STACK_BASE)/Build
LINKER_FILE = AppBuild_$(JENNIC_CHIP).ld

# Tell linker to garbage collect unused section
LDFLAGS += --gc-sections
# Define entry points to linker
LDFLAGS += -u_AppColdStart -u_AppWarmStart

# Also add compiler flags to allow garbage collection
CFLAGS += -fdata-sections -ffunction-sections

# Add Jenie / JenNet libraries
LIBFILE += $(JENIE_LIB)/Jenie_Tree$(DEVICE_TYPE)Lib.a
LIBFILE += $(JENIE_LIB_COMMON)

#########################################################################
# Dependency rules

.PHONY: all clean
# Path to directories containing application source 
vpath % $(APP_SRC_DIR):$(APP_COMMON_SRC_DIR):$(ADDITIONAL_SRC_DIR)


all: $(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin

-include $(APPDEPS)
%.d:
	rm -f $*.o


%.o: %.S
	$(info Assembling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $*.d -MP
	@echo

%.o: %.c 
	$(info Compiling $< ...)
	$(CC) -c -o $(subst Source,Build,$@) $(CFLAGS) $(INCFLAGS) $< -MD -MF $*.d -MP
	@echo

$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf: $(APPOBJS)
	$(info Linking $@ ...)
# Use LD rather than gcc for JN5139
	$(LD) -L$(STACK_BLD) -o $@ $(APPOBJS) -T$(LINKER_FILE) $(LDFLAGS) --start-group $(LIBS) $(LIBFILE) --end-group
	ba-elf-size $@
	@echo

$(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).bin: $(TARGET)_$(JENNIC_CHIP)$(BIN_SUFFIX).elf 
	$(info Generating binary ...)
	$(OBJCOPY) -S -O binary $< $@
	
#########################################################################

clean:
	rm -f $(APPOBJS) $(APPDEPS) $(TARGET)_$(JENNIC_CHIP)*.bin $(TARGET)_$(JENNIC_CHIP)*.elf $(TARGET)_$(JENNIC_CHIP)*.map

#########################################################################
