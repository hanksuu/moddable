#
# Copyright (c) 2016-2019  Moddable Tech, Inc.
#
#   This file is part of the Moddable SDK Tools.
#
#   The Moddable SDK Tools is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   The Moddable SDK Tools is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with the Moddable SDK Tools.  If not, see <http://www.gnu.org/licenses/>.
#

BASE = $(SDK_ROOT)
SDK_BASE = $(BASE)

HW_DEBUG_OPT = -O0
HW_OPT = -O2

C_FLAGS += -march=armv8-m.main+dsp
C_FLAGS += -mthumb
C_FLAGS += -mcmse
C_FLAGS += -mfloat-abi=hard
C_FLAGS += -mfpu=fpv5-sp-d16

C_FLAGS += -g
C_FLAGS += -gdwarf-3
C_FLAGS += -nostartfiles
C_FLAGS += -nodefaultlibs
C_FLAGS += -nostdlib
C_FLAGS += -D__FPU_PRESENT

C_FLAGS += -gdwarf-3
C_FLAGS += -fstack-usage
C_FLAGS += -fdata-sections
C_FLAGS += -nostartfiles
C_FLAGS += -nostdlib
C_FLAGS += -Wall
C_FLAGS += -Wpointer-arith
#C_FLAGS += -Wstrict-prototypes
C_FLAGS += -Wundef
C_FLAGS += -Wno-write-strings
C_FLAGS += -Wno-maybe-uninitialized
C_FLAGS += --save-temps
C_FLAGS += -c
C_FLAGS += -MMD

C_FLAGS += -Wextra

C_FLAGS += -DCONFIG_PLATFORM_8721D
C_FLAGS += -DCONFIG_USE_MBEDTLS_ROM_ALG
C_FLAGS += -DCONFIG_FUNCION_O0_OPTIMIZE
C_FLAGS += -DDM_ODM_SUPPORT_TYPE=32

# avoid warning, need to check
C_FLAGS += -Wno-missing-field-initializers
#C_FLAGS += -DESP32=0
#C_FLAGS += -DMOD_TASKS=0
#C_FLAGS += -D__LP64__=0


#ifeq ($(DEBUG),1)
#	LIB_DIR = $(BUILD_DIR)/tmp/ambd/debug/lib
#else
	ifeq ($(INSTRUMENT),1)
		LIB_DIR = $(BUILD_DIR)/tmp/ambd/instrument/lib
	else
		LIB_DIR = $(BUILD_DIR)/tmp/ambd/release/lib
	endif
#endif


HOST_OS := $(shell uname)

RTK_GCC_ROOT = $(SDK_BASE)/project/realtek_amebaD_va0_example/GCC-RELEASE/project_hp/toolchain/linux/asdk-6.4.1/linux/newlib

INC_DIRS = \
	$(SDK_BASE)/include \
	$(SDK_BASE)/include/bsp \
	$(SDK_BASE)/include/qapi \
	$(XS_DIR)/../modules/base/instrumentation \
	$(BUILD_DIR)/devices/ambd \
	$(SDK_BASE)/component/os/freertos/freertos_v10.2.0/Source/include \
	$(SDK_BASE)/component/os/freertos/freertos_v10.2.0/Source/portable/IAR/RTL8721D_HP/non_secure \
	$(SDK_BASE)/project/realtek_amebaD_va0_example/inc/inc_hp \
	$(SDK_BASE)/component/common/api/wifi \
	$(SDK_BASE)/component/common/drivers/wlan/realtek/include \
	$(SDK_BASE)/component/common/api \
	$(SDK_BASE)/component/common/api/platform \
	$(SDK_BASE)/component/common/drivers/wlan/realtek/src/osdep \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/string \
	$(SDK_BASE)/component/soc/realtek/amebad/fwlib/include \
	$(SDK_BASE)/component/soc/realtek/amebad/app/monitor/include \
	$(SDK_BASE)/component/soc/realtek/amebad/app/xmodem \
	$(SDK_BASE)/component/common/network/ssl/mbedtls-2.4.0/include \
	$(SDK_BASE)/component/common/network/ssl/ssl_ram_map/rom \
	$(SDK_BASE)/component/soc/realtek/amebad/misc \
	$(SDK_BASE)/component/os/freertos \
	$(SDK_BASE)/component/soc/realtek/amebad/cmsis \
	$(SDK_BASE)/component/common/api \
	$(SDK_BASE)/component/common/api/platform \
	$(SDK_BASE)/component/common/api/wifi \
	$(SDK_BASE)/component/common/api/network \
	$(SDK_BASE)/component/common/api/network/include \
	$(SDK_BASE)/component/common/audio \
	$(SDK_BASE)/component/common/test \
	$(SDK_BASE)/component/common/example \
	$(SDK_BASE)/component/common/utilities \
	$(SDK_BASE)/component/os/freertos \
	$(SDK_BASE)/component/os/freertos/freertos_v10.2.0/Source/include \
	$(SDK_BASE)/component/os/freertos/freertos_v10.2.0/Source/portable/GCC/RTL8721D_HP/non_secure \
	$(SDK_BASE)/component/os/freertos/freertos_v10.2.0/Source/portable/GCC/RTL8721D_HP/secure \
	$(SDK_BASE)/component/soc/realtek/amebad/cmsis \
	$(SDK_BASE)/component/soc/realtek/amebad/cmsis/device \
	$(SDK_BASE)/component/soc/realtek/amebad/cmsis-dsp/Source \
	$(SDK_BASE)/component/soc/realtek/amebad/fwlib/include \
	$(SDK_BASE)/component/soc/realtek/amebad/fwlib/rom_map_hp \
	$(SDK_BASE)/component/soc/realtek/amebad/app/monitor/include \
	$(SDK_BASE)/component/soc/realtek/amebad/app/xmodem \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/include \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/string \
	$(SDK_BASE)/component/soc/realtek/amebad/misc \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/os_dep/include \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/os_dep/../ \
	$(SDK_BASE)/component/soc/realtek/amebad/swlib/string \
	$(SDK_BASE)/component/common/network/lwip/lwip_v2.0.2/src/include \
	$(SDK_BASE)/component/common/network/lwip/lwip_v2.0.2/src/include/lwip \
	$(SDK_BASE)/component/common/network/lwip/lwip_v2.0.2/port/realtek \
	$(SDK_BASE)/component/common/network \
	$(SDK_BASE)/component/common/mbed/hal \
	$(SDK_BASE)/component/common/mbed/targets/hal/rtl8721d \
	$(SDK_BASE)/component/common/mbed/api

XS_OBJ = \
	$(LIB_DIR)/xsHost.c.o \
	$(LIB_DIR)/xsPlatform.c.o \
	$(LIB_DIR)/xsAll.c.o \
	$(LIB_DIR)/xsAPI.c.o \
	$(LIB_DIR)/xsArguments.c.o \
	$(LIB_DIR)/xsArray.c.o \
	$(LIB_DIR)/xsAtomics.c.o \
	$(LIB_DIR)/xsBigInt.c.o \
	$(LIB_DIR)/xsBoolean.c.o \
	$(LIB_DIR)/xsCode.c.o \
	$(LIB_DIR)/xsCommon.c.o \
	$(LIB_DIR)/xsDataView.c.o \
	$(LIB_DIR)/xsDate.c.o \
	$(LIB_DIR)/xsDebug.c.o \
	$(LIB_DIR)/xsError.c.o \
	$(LIB_DIR)/xsFunction.c.o \
	$(LIB_DIR)/xsGenerator.c.o \
	$(LIB_DIR)/xsGlobal.c.o \
	$(LIB_DIR)/xsJSON.c.o \
	$(LIB_DIR)/xsLexical.c.o \
	$(LIB_DIR)/xsMapSet.c.o \
	$(LIB_DIR)/xsMarshall.c.o \
	$(LIB_DIR)/xsMath.c.o \
	$(LIB_DIR)/xsMemory.c.o \
	$(LIB_DIR)/xsModule.c.o \
	$(LIB_DIR)/xsNumber.c.o \
	$(LIB_DIR)/xsObject.c.o \
	$(LIB_DIR)/xsPromise.c.o \
	$(LIB_DIR)/xsProperty.c.o \
	$(LIB_DIR)/xsProxy.c.o \
	$(LIB_DIR)/xsRegExp.c.o \
	$(LIB_DIR)/xsRun.c.o \
	$(LIB_DIR)/xsScope.c.o \
	$(LIB_DIR)/xsScript.c.o \
	$(LIB_DIR)/xsSourceMap.c.o \
	$(LIB_DIR)/xsString.c.o \
	$(LIB_DIR)/xsSymbol.c.o \
	$(LIB_DIR)/xsSyntaxical.c.o \
	$(LIB_DIR)/xsTree.c.o \
	$(LIB_DIR)/xsType.c.o \
	$(LIB_DIR)/xsdtoa.c.o \
	$(LIB_DIR)/xsmc.c.o \
	$(LIB_DIR)/xsre.c.o

XS_DIRS = \
	$(XS_DIR)/includes \
	$(XS_DIR)/sources \
	$(XS_DIR)/platforms/ambd \
	$(BUILD_DIR)/devices/ambd
XS_HEADERS = \
	$(XS_DIR)/includes/xs.h \
	$(XS_DIR)/includes/xsmc.h \
	$(XS_DIR)/sources/xsAll.h \
	$(XS_DIR)/sources/xsCommon.h \
	$(XS_DIR)/platforms/ambd/xsHost.h \
	$(XS_DIR)/platforms/ambd/xsPlatform.h
HEADERS += $(XS_HEADERS)

SDK_OBJ = \
	$(TMP_DIR)/xsmain.c.o \
	$(TMP_DIR)/systemclock.c.o \
	$(TMP_DIR)/debugger.c.o
	#$(TMP_DIR)/wlan.c.o

SDK_DIRS = \
	$(BUILD_DIR)/devices/ambd/base


TOOLS_BIN = $(RTK_GCC_ROOT)/bin
TOOLS_PREFIX = arm-none-eabi

CC  = $(TOOLS_BIN)/$(TOOLS_PREFIX)-gcc
CPP = $(TOOLS_BIN)/$(TOOLS_PREFIX)-g++
LD  = $(CPP)
AR  = $(TOOLS_BIN)/$(TOOLS_PREFIX)-ar

AR_FLAGS = crs

MODDABLE_TOOLS_DIR = $(BUILD_DIR)/bin/lin/release
BUILDCLUT = $(MODDABLE_TOOLS_DIR)/buildclut
COMPRESSBMF = $(MODDABLE_TOOLS_DIR)/compressbmf
RLE4ENCODE = $(MODDABLE_TOOLS_DIR)/rle4encode
MCLOCAL = $(MODDABLE_TOOLS_DIR)/mclocal
MCREZ = $(MODDABLE_TOOLS_DIR)/mcrez
PNG2BMP = $(MODDABLE_TOOLS_DIR)/png2bmp
IMAGE2CS = $(MODDABLE_TOOLS_DIR)/image2cs
WAV2MAUD = $(MODDABLE_TOOLS_DIR)/wav2maud
BLES2GATT = $(MODDABLE_TOOLS_DIR)/bles2gatt
XSC = $(MODDABLE_TOOLS_DIR)/xsc
XSID = $(MODDABLE_TOOLS_DIR)/xsid
XSL = $(MODDABLE_TOOLS_DIR)/xsl

#	-DmxNoConsole=1
C_DEFINES = \
	-U__STRICT_ANSI__ \
	$(NET_CONFIG_FLAGS) \
	-DmxUseDefaultSharedChunks=1 \
	-DmxRun=1 \
	-DkCommodettoBitmapFormat=$(DISPLAY) \
	-DkPocoRotation=$(ROTATION)
#ifeq ($(DEBUG),1)
#	C_DEFINES += -DDEBUG=1 -DmxDebug=1
#	C_FLAGS += $(HW_DEBUG_OPT)
#else
	C_FLAGS += $(HW_OPT)
#endif
ifeq ($(INSTRUMENT),1)
	C_DEFINES += -DMODINSTRUMENTATION=1 -DmxInstrument=1
endif

cr := '\n'
sp :=  
sp += 
qs = $(subst ?,\$(sp),$1)
C_INCLUDES += $(DIRECTORIES)
C_INCLUDES += $(foreach dir,$(INC_DIRS) $(SDK_DIRS) $(XS_DIRS) $(LIB_DIR) $(TMP_DIR),-I$(call qs,$(dir)))

C_FLAGS += -gdwarf-3 -mthumb -std=c99 \
    -fno-short-enums \
    #-DDEBUG=1 \
	-c -fmessage-length=0 -mno-sched-prolog \
    -fno-builtin -ffunction-sections -fdata-sections \
    -MMD -MP \
	$(DEV_C_FLAGS)

C_FLAGS_NODATASECTION = $(C_FLAGS)

# Utility functions
#git_description = $(shell git -C  $(1) describe --tags --always --dirty 2>/dev/null)
#SRC_GIT_VERSION = $(call git_description,$(BASE)/sources)
#ESP_GIT_VERSION = $(call git_description,$(ARDUINO_ROOT))
#time_string = $(shell perl -e 'use POSIX qw(strftime); print strftime($(1), localtime());')
#BUILD_DATE = $(call time_string,"%Y-%m-%d")
#BUILD_TIME = $(call time_string,"%H:%M:%S")
#MEM_USAGE = \
#  'while (<>) { \
#      $$r += $$1 if /^\.(?:data|rodata|bss)\s+(\d+)/;\
#		  $$f += $$1 if /^\.(?:irom0\.text|text|data|rodata)\s+(\d+)/;\
#	 }\
#	 print "\# Memory usage\n";\
#	 print sprintf("\#  %-6s %6d bytes\n" x 2 ."\n", "Ram:", $$r, "Flash:", $$f);'

VPATH += $(SDK_DIRS) $(XS_DIRS)

.PHONY: all

all: $(BLE) $(LIB_DIR) $(BIN_DIR)/xs_ambd.a
	cp $(BIN_DIR)/xs_ambd.a  $(SDK_BASE)/project/realtek_amebaD_va0_example/GCC-RELEASE/project_hp/asdk/lib/application

clean:
	@echo "# Clean project"
	-rm -rf $(BIN_DIR) 2>/dev/null
	-rm -rf $(TMP_DIR) 2>/dev/null

build: all

xsbug:
	@echo "# kill serial2xsbug"
	$(shell pkill serial2xsbug)
	echo "# start xsbug"
	$(shell nohup $(BUILD_DIR)/bin/lin/release/xsbug > /dev/null 2>&1 &)
	echo "# start serial2xsbug at port $(UPLOAD_PORT)"
	$(shell nohup serial2xsbug $(UPLOAD_PORT) 115200 8N1 2>&1 &)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)
	echo "typedef struct { const char *date, *time, *src_version, *env_version;} _tBuildInfo; extern _tBuildInfo _BuildInfo;" > $(LIB_DIR)/buildinfo.h
	
$(BIN_DIR)/xs_ambd.a: $(SDK_OBJ) $(XS_OBJ) $(TMP_DIR)/mc.xs.c.o  $(TMP_DIR)/mc.resources.c.o $(OBJECTS) 
	@echo "# ld xs_ambd.bin"
	echo '#include "buildinfo.h"' > $(LIB_DIR)/buildinfo.c
	echo '_tBuildInfo _BuildInfo = {"$(BUILD_DATE)","$(BUILD_TIME)","$(SRC_GIT_VERSION)","$(ESP_GIT_VERSION)"};' >> $(LIB_DIR)/buildinfo.c
	$(CC) $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) $(LIB_DIR)/buildinfo.c -o $(LIB_DIR)/buildinfo.c.o
	$(AR) $(AR_FLAGS) $(BIN_DIR)/xs_ambd.a $^ $(LIB_DIR)/buildinfo.c.o

$(XS_OBJ): $(XS_HEADERS)
$(LIB_DIR)/xs%.c.o: xs%.c
	@echo "# cc" $(<F) "(strings in flash)"
	$(CC) $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) $< -o $@

$(LIB_DIR)/%.c.o: %.c
	@echo "# cc" $(<F) "(strings in flash)"
	$(CC) $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) $< -o $@

$(TMP_DIR)/%.c.o: %.c
	@echo "# cc" $(<F) "(strings in flash)"
	$(CC) $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) $< -o $@

$(TMP_DIR)/mc.%.c.o: $(TMP_DIR)/mc.%.c
	@echo "# cc" $(<F) "(slots in flash)"
	$(CC) $< $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS_NODATASECTION) -o $@.unmapped
	$(TOOLS_BIN)/$(TOOLS_PREFIX)-objcopy --rename-section .data.gxKeys=.rodata.gxKeys --rename-section .data.gxNames=.rodata.gxNames --rename-section .data.gxGlobals=.rodata.gxGlobals $@.unmapped $@

$(TMP_DIR)/mc.xs.c: $(MODULES) $(MANIFEST)
	@echo "# xsl modules"
	$(XSL) -b $(MODULES_DIR) -o $(TMP_DIR) $(PRELOADS) $(STRIPS) $(CREATION) $(MODULES)
	
$(TMP_DIR)/mc.resources.c: $(RESOURCES) $(MANIFEST)
	@echo "# mcrez resources"
	$(MCREZ) $(RESOURCES) -o $(TMP_DIR) -p ambd -r mc.resources.c
	
MAKEFLAGS += --jobs
ifneq ($(VERBOSE),1)
MAKEFLAGS += --silent
endif

