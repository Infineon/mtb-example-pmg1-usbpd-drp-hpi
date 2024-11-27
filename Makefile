################################################################################
# \file Makefile
# \version 1.0
#
# \brief
# Top-level application make file.
#
################################################################################
# \copyright
# Copyright 2024, Cypress Semiconductor Corporation (an Infineon company)
# SPDX-License-Identifier: Apache-2.0
# 
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################


################################################################################
# Basic Configuration
################################################################################

# Type of ModusToolbox Makefile Options include:
#
# COMBINED    -- Top Level Makefile usually for single standalone application
# APPLICATION -- Top Level Makefile usually for multi project application
# PROJECT     -- Project Makefile under Application
#
MTB_TYPE=COMBINED

# Target board/hardware (BSP).
# To change the target, it is recommended to use the Library manager
# ('make modlibs' from command line), which will also update Eclipse IDE launch
# configurations. If TARGET is manually edited, ensure TARGET_<BSP>.mtb with a
# valid URL exists in the application, run 'make getlibs' to fetch BSP contents
# and update or regenerate launch configurations for your IDE.
TARGET=PMG1-CY7110
APP_TARGET=$(subst APP_,,$(TARGET))

# Name of application (used to derive name of final linked file).
#
# If APPNAME is edited, ensure to update or regenerate launch
# configurations for your IDE.
APPNAME=mtb-example-pmg1-usbpd-drp-hpi
APPNAME_EXT=fw1

# Name and version of the bootloader hex file resides in bootloader folder.
BOOT_NAME=mtb-example-pmg1-i2c-bootloader
BOOT_VER=1_0_0_8
BOOT_CONFIG=Custom
BOOT_DIR=bootloader/$(APP_TARGET)/$(TOOLCHAIN)
ifeq ($(APP_TARGET), $(filter $(APP_TARGET), PMG1-CY7110 PMG1-CY7112))
BOOT_LAST_ROW=0x37
else
BOOT_LAST_ROW=0x1B
endif
# Dual/Single firmware architecture option.
# DUAL -- Configuration to create dual firmware image.
# SINGLE -- Configuration to create a single firmware image.
FIRMWARE_ARCH=SINGLE

# Name of toolchain to use. Options include:
#
# GCC_ARM -- GCC provided with ModusToolbox IDE
# ARM     -- ARM Compiler (must be installed separately)
# IAR     -- IAR Compiler (must be installed separately)
#
# See also: CY_COMPILER_PATH below
TOOLCHAIN=GCC_ARM

# Default build configuration. Options include:
#
# Debug -- build with minimal optimizations, focus on debugging.
# Release -- build with full optimizations
# Custom -- build with custom configuration, set the optimization flag in CFLAGS
#
# If CONFIG is manually edited, ensure to update or regenerate launch configurations
# for your IDE.
CONFIG=Release

# If set to "true" or "1", display full command-lines when building.
VERBOSE=


################################################################################
# Advanced Configuration
################################################################################

# Enable optional code that is ordinarily disabled by default.
#
# Available components depend on the specific targeted hardware and firmware
# in use. In general, if you have
#
#    COMPONENTS=foo bar
#
# ... then code in directories named COMPONENT_foo and COMPONENT_bar will be
# added to the build
#
ifeq ($(APP_TARGET), PMG1-CY7110)
COMPONENTS=PMG1_PD3_DRP_LITE HPI_SLAVE_HVMCU
else
COMPONENTS=PMG1_PD3_DRP HPI_SLAVE_HVMCU
endif

# Like COMPONENTS, but disable optional code that was enabled by default.
DISABLE_COMPONENTS=

# By default the build system automatically looks in the Makefile's directory
# tree for source code and builds it. The SOURCES variable can be used to
# manually add source code to the build process from a location not searched
# by default, or otherwise not found by the build system.
SOURCES=

# Like SOURCES, but for include directories. Value should be paths to
# directories (without a leading -I).
INCLUDES=

# Add additional defines to the build process (without a leading -D).
# Enabled PD revision 3.0 support, VBus OV Fault Protection and Deep Sleep mode in idle states.
DEFINES=CY_PD_SINK_ONLY=0 CY_PD_SOURCE_ONLY=0 CY_PD_REV3_ENABLE=1 \
        VBUS_OVP_ENABLE=1 VBUS_UVP_ENABLE=0 VBUS_OCP_ENABLE=1 SYS_DEEPSLEEP_ENABLE=1 \
        MINOR_SVDM_VER_SUPPORT=1 CY_APP_ROLE_PREFERENCE_ENABLE=0 \
        CY_APP_POWER_ROLE_PREFERENCE_ENABLE=0

ifeq ($(APP_TARGET), PMG1-CY7110)
DEFINES+=PMG1_V5V_CHANGE_DETECT=0 VBUS_IN_DISCHARGE_EN=1 PMG1_FLIPPED_FET_CTRL=1 CCG_SRC_FET=1 \
         CY_PD_CBL_DISC_DISABLE=1 CY_PD_VCONN_DISABLE=1 CY_PD_TRY_SRC_SNK_DISABLE=1\
         CY_PD_DEBUG_ACC_DISABLE=1 CY_PD_VDM_DISABLE=1 
else
DEFINES+=PMG1_V5V_CHANGE_DETECT=1 CY_PD_CBL_DISC_DISABLE=0 CY_PD_VCONN_DISABLE=0 CY_PD_VDM_DISABLE=0\
         CY_PD_USB4_SUPPORT_ENABLE=1
endif
        
ifeq ($(APP_TARGET), PMG1-CY7112)
DEFINES+=VBUS_RCP_ENABLE=0 VBUS_SCP_ENABLE=0 VCONN_OCP_ENABLE=0
else
DEFINES+=VBUS_RCP_ENABLE=1 VBUS_SCP_ENABLE=1 VCONN_OCP_ENABLE=1
endif

################################################################################
# Application specific features
################################################################################
#
# CY_PD_BAT_STATUS_HANDLER_ENABLE Enable battery status response handling.
# CY_PD_BAT_CAPS_HANDLER_ENABLE Enable battery capability response handling.
# CY_PD_SRC_INFO_HANDLER_ENABLE Enable source info response handling
# CY_PD_REVISION_MESSAGE_ENABLE Enable handling of PD Revision message related responses.
APP_DEFINES=CY_PD_BAT_STATUS_HANDLER_ENABLE=0 CY_PD_BAT_CAPS_HANDLER_ENABLE=0 \
            CY_PD_SRC_INFO_HANDLER_ENABLE=0 CY_PD_REVISION_MESSAGE_ENABLE=0 \
            CY_CORROSION_MITIGATION_ENABLE=0

ifeq ($(APP_TARGET), $(filter $(APP_TARGET), PMG1-CY7110 PMG1-CY7112 PMG1-CY7113))
APP_DEFINES+=APP_FW_LED_ENABLE=0
else
APP_DEFINES+=APP_FW_LED_ENABLE=1
endif

################################################################################
# HPI Module Configuration
################################################################################
# 
# CY_HPI_ENABLED HPI interface enable.
# CY_HPI_PD_ENABLE Enable PD support in HPI.
# CY_HPI_PD_CMD_ENABLE Enable PD command support in HPI library.
# CY_HPI_VBUS_C_CTRL_ENABLE Enable Vbus Consumer FET control through HPI.
# CY_HPI_RW_PD_RESP_MSG_DATA Enabled handling of generic PD response data HPI command.     
HPI_DEFINES=CY_HPI_ENABLED=1 CY_HPI_PD_ENABLE=1 CY_HPI_PD_CMD_ENABLE=1 \
            CY_HPI_BB_ENABLE=0 CY_HPI_LEGACY_DUAL_APP_EN=1 \
            CY_HPI_BOOT_ENABLE=0 CY_HPI_FLASH_RW_ENABLE=1 \
            CY_HPI_PD_INTR_STATUS_ENABLE=1 CY_HPI_VDM_QUERY_SUPPORTED=0 \
            HPI_DEBUG_COMMAND_EN=0 CCG_LOAD_SHARING_ENABLE=0 CCG_UCSI_ENABLE=0 \
            CY_HPI_VBUS_C_CTRL_ENABLE=1 PD3_STATUS_ECR_ENABLE=0 \
            SEND_ALERT_ON_PWR_STATE_CHG_ENABLE=0 CY_USE_CONFIG_TABLE=0 \
            SUPPORT_BL_VER_0_1=0 CCG_DUALAPP_DISABLE=0 CY_HPI_LITE_ENABLED=1 \
            CY_HPI_RW_PD_RESP_MSG_DATA=0

#Bootloader last row
ifeq ($(APP_TARGET), $(filter $(APP_TARGET), PMG1-CY7110 PMG1-CY7112))
HPI_DEFINES+=CY_APP_BOOT_LOADER_LAST_ROW=0x37
else
HPI_DEFINES+=CY_APP_BOOT_LOADER_LAST_ROW=0x1B
endif

# Application image-1 and image-2 last row. This needs to be updated if the
# application flash layout changes.
ifeq ($(APP_TARGET), PMG1-CY7110)
HPI_DEFINES+=CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x01FE CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x1FE
else
ifeq ($(APP_TARGET), EVAL_PMG1_S1_DRP)
HPI_DEFINES+=CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x01FE CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x01FE
else
ifeq ($(APP_TARGET), PMG1-CY7112)
HPI_DEFINES+=CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x03FE CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x03FE
else # PMG1-CY7113, EVAL_PMG1_S3_DUALDRP
ifeq ($(FIRMWARE_ARCH), SINGLE)
HPI_DEFINES+=CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x03FE CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x03FE
else
HPI_DEFINES+=CY_APP_IMG1_LAST_FLASH_ROW_NUM=0x020C CY_APP_IMG2_LAST_FLASH_ROW_NUM=0x03FD
endif # ($(FIRMWARE_ARCH), SINGLE)
endif #($(APP_TARGET), PMG1-CY7112)
endif #($(APP_TARGET), EVAL_PMG1_S1_DRP)
endif #($(APP_TARGET), PMG1-CY7110)

ifeq ($(APP_TARGET), PMG1-CY7112)
HPI_DEFINES+=CY_APP_HPI_I2C_HW=SCB2 
else
HPI_DEFINES+=CY_APP_HPI_I2C_HW=SCB0
endif

DEFINES +=$(HPI_DEFINES)
DEFINES +=$(APP_DEFINES)

# Configure flash rows count for post build script.
ifeq ($(APP_TARGET), $(filter $(APP_TARGET), PMG1-CY7110 EVAL_PMG1_S1_DRP))
PMG1_FLASH_ROW_COUNT=0x200
else #APP_PMG1-CY7112, APP_PMG1-CY7113 and APP_EVAL_PMG1_S3_DUALDRP
PMG1_FLASH_ROW_COUNT=0x400
endif

# Select softfp or hardfp floating point. Default is softfp.
VFP_SELECT=

# Additional / custom C compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(CONFIG), Release)
ifeq ($(TOOLCHAIN), ARM)
CFLAGS=-flto
else 
ifeq ($(TOOLCHAIN), IAR)
CFLAGS=--no_inline --no_unroll
else
CFLAGS=-flto -Os
endif #IAR
endif #ARM
endif #Release

# Additional / custom C++ compiler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ifeq ($(CONFIG), Release)
ifeq ($(TOOLCHAIN), ARM)
CXXFLAGS=-Werror
else 
ifeq ($(TOOLCHAIN), IAR)
CXXFLAGS=
else
CXXFLAGS=-Werror
endif #IAR
endif #ARM
endif #Release

# Additional / custom assembler flags.
#
# NOTE: Includes and defines should use the INCLUDES and DEFINES variable
# above.
ASFLAGS=

# Additional / custom linker flags.
ifeq ($(CONFIG), Release)
ifeq ($(TOOLCHAIN), ARM)
LDFLAGS=--lto
else
ifeq ($(TOOLCHAIN), IAR)
LDFLAGS=
else
LDFLAGS=
endif #IAR
endif #ARM
endif #Release

# Additional / custom libraries to link in to the application.
LDLIBS=

# Path to the linker script to use (if empty, use the default linker script).
LINKER_SCRIPT_DIR=$(MTB_TOOLS__TARGET_DIR)/COMPONENT_$(MTB_RECIPE__CORE)/TOOLCHAIN_$(TOOLCHAIN)

ifeq ($(TOOLCHAIN), ARM)
LINKER_SCRIPT_SUFFIX=$(MTB_TOOLCHAIN_ARM__SUFFIX_LS)
else
ifeq ($(TOOLCHAIN), IAR)
LINKER_SCRIPT_SUFFIX=$(MTB_TOOLCHAIN_IAR__SUFFIX_LS)
else
LINKER_SCRIPT_SUFFIX=$(MTB_TOOLCHAIN_GCC_ARM__SUFFIX_LS)
endif
endif

ifeq ($(CONFIG), Release)
ifeq ($(FIRMWARE_ARCH), SINGLE)
DEFINES += CY_APP_TYPE=0 CY_APP_FIRMWARE_APP_ONLY=0 CY_APP_FIRMWARE_ARCH=0
LINKER_SCRIPT=$(LINKER_SCRIPT_DIR)/pmg1_linker_single_fw.$(LINKER_SCRIPT_SUFFIX)
else
ifeq ($(APPNAME_EXT), fw1)
DEFINES += CY_APP_TYPE=0 CY_APP_FIRMWARE_APP_ONLY=0 CY_APP_FIRMWARE_ARCH=1
LINKER_SCRIPT=$(LINKER_SCRIPT_DIR)/pmg1_linker_dual_fw1.$(LINKER_SCRIPT_SUFFIX)
else
DEFINES += CY_APP_TYPE=1 CY_APP_FIRMWARE_APP_ONLY=0 CY_APP_FIRMWARE_ARCH=1
LINKER_SCRIPT=$(LINKER_SCRIPT_DIR)/pmg1_linker_dual_fw2.$(LINKER_SCRIPT_SUFFIX)
endif
endif
else
DEFINES += CY_APP_TYPE=0 CY_APP_FIRMWARE_APP_ONLY=1 CY_APP_FIRMWARE_ARCH=0
LINKER_SCRIPT=
endif

# Custom pre-build commands to run.
PREBUILD=

# Post build command to create a composite hex file
POSTBUILD_CMD=./utils/post_build.sh \
              $(CY_MCUELFTOOL) \
              $(APPNAME_EXT) \
              $(MTB_TOOLS__OUTPUT_CONFIG_DIR)/$(APPNAME) \
              $(BOOT_DIR)/$(BOOT_NAME)_$(TARGET)_$(TOOLCHAIN)_$(BOOT_CONFIG)_$(BOOT_VER) \
              $(PMG1_FLASH_ROW_COUNT) \
              $(BOOT_LAST_ROW) \
              $(FIRMWARE_ARCH) \
              $(CY_TOOLS_DIR) \
              $(OS)

# Post build command to build firmware 2 in DUAL architecture
POSTBUILD_MAKE_FW2=+\
                   $(POSTBUILD_CMD); \
                   $(MAKE) APPNAME_EXT=fw2 build;\

# Custom post-build commands to run.
ifeq ($(CONFIG), Release)
ifeq ($(FIRMWARE_ARCH), SINGLE)
# Single application post build command
POSTBUILD= $(POSTBUILD_CMD)
else
ifeq ($(APPNAME_EXT), fw1)
# Dual application firmware 1 post build command
POSTBUILD= $(POSTBUILD_MAKE_FW2)
else
# Dual application firmware 2 post build command
POSTBUILD= $(POSTBUILD_CMD)
endif
endif
else
POSTBUILD=
endif

################################################################################
# Paths
################################################################################

# Relative path to the project directory (default is the Makefile's directory).
#
# This controls where automatic source code discovery looks for code.
CY_APP_PATH=

# Relative path to the shared repo location.
#
# All .mtb files have the format, <URI>#<COMMIT>#<LOCATION>. If the <LOCATION> field
# begins with $$ASSET_REPO$$, then the repo is deposited in the path specified by
# the CY_GETLIBS_SHARED_PATH variable. The default location is one directory level
# above the current app directory.
# This is used with CY_GETLIBS_SHARED_NAME variable, which specifies the directory name.
CY_GETLIBS_SHARED_PATH=../

# Directory name of the shared repo location.
#
CY_GETLIBS_SHARED_NAME=mtb_shared

# Absolute path to the compiler's "bin" directory.
#
# The default depends on the selected TOOLCHAIN (GCC_ARM uses the ModusToolbox
# IDE provided compiler by default).
CY_COMPILER_PATH=


# Locate ModusToolbox IDE helper tools folders in default installation
# locations for Windows, Linux, and macOS.
CY_WIN_HOME=$(subst \,/,$(USERPROFILE))
CY_TOOLS_PATHS ?= $(wildcard \
    $(CY_WIN_HOME)/ModusToolbox/tools_3* \
    $(HOME)/ModusToolbox/tools_3* \
    /Applications/ModusToolbox/tools_3*)

# If you install ModusToolbox IDE in a custom location, add the path to its
# "tools_X.Y" folder (where X and Y are the version number of the tools
# folder). Make sure you use forward slashes.
CY_TOOLS_PATHS+=

# Default to the newest installed tools folder, or the users override (if it's
# found).
CY_TOOLS_DIR=$(lastword $(sort $(wildcard $(CY_TOOLS_PATHS))))

ifeq ($(CY_TOOLS_DIR),)
$(error Unable to find any of the available CY_TOOLS_PATHS -- $(CY_TOOLS_PATHS). On Windows, use forward slashes.)
endif

$(info Tools Directory: $(CY_TOOLS_DIR))

# Path to Elf tool directory. 
CY_MCUELFTOOL_DIR=$(wildcard $(CY_TOOLS_DIR)/cymcuelftool-*)

# CY MCU ELF tool executable path.
ifeq ($(OS),Windows_NT)
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool.exe
else
    CY_MCUELFTOOL=$(CY_MCUELFTOOL_DIR)/bin/cymcuelftool
endif

$(info Elf tool : $(CY_MCUELFTOOL))
include $(CY_TOOLS_DIR)/make/start.mk
