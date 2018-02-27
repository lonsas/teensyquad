# The name of your project (used to name the compiled .hex file)
TARGET = $(notdir $(CURDIR))

# The teensy version to use, 30, 31, or LC
TEENSY = 31

# Set to 24000000, 48000000, or 96000000 to set CPU core speed
TEENSY_CORE_SPEED = 48000000

# Some libraries will require this to be defined
# If you define this, you will break the default main.cpp
#ARDUINO = 10600

# configurable options
OPTIONS = -DUSB_SERIAL -DLAYOUT_US_ENGLISH

# directory to build in
BUILDDIR = $(abspath $(CURDIR)/build)
TESTBUILDDIR = $(abspath $(CURDIR)/build/tests)
MODELBUILDDIR = $(abspath $(CURDIR)/build/model)

#************************************************************************
# Location of Teensyduino utilities, Toolchain, and Arduino Libraries.
# To use this makefile without Arduino, copy the resources from these
# locations and edit the pathnames.  The rest of Arduino is not needed.
#************************************************************************

# path location for Teensy Loader, teensy_post_compile and teensy_reboot
TOOLSPATH = $(CURDIR)/tools

ifeq ($(OS),Windows_NT)
    $(error What is Win Dose?)
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Darwin)
        TOOLSPATH = /Applications/Arduino.app/Contents/Java/hardware/tools/
    endif
endif

# path location for Teensy 3 core
COREPATH = teensy3

# path location for Arduino libraries
LIBRARYPATH = libraries

# path location for the arm-none-eabi compiler
COMPILERPATH = $(TOOLSPATH)/arm/bin

TESTPATH = tests

#************************************************************************
# Settings below this point usually do not need to be edited
#************************************************************************

# CPPFLAGS = compiler options for C and C++
CPPFLAGS = -Wall -g -Os -mthumb -ffunction-sections -fdata-sections -nostdlib -MMD $(OPTIONS) -DTEENSYDUINO=124 -DF_CPU=$(TEENSY_CORE_SPEED) -Isrc -I$(COREPATH)
CPPFLAGS += -DARDUINO=101
# compiler options for C++ only
CXXFLAGS = -std=gnu++0x -felide-constructors -fno-exceptions -fno-rtti

# compiler options for C only
CFLAGS=-std=c99

# linker options
LDFLAGS = -Os -Wl,--gc-sections -mthumb

# additional libraries to link
LIBS = -lm

# compiler options specific to teensy version
ifeq ($(TEENSY), 30)
    CPPFLAGS += -D__MK20DX128__ -mcpu=cortex-m4
    LDSCRIPT = $(COREPATH)/mk20dx128.ld
    LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
else
    ifeq ($(TEENSY), 31)
        CPPFLAGS += -D__MK20DX256__ -mcpu=cortex-m4
        LDSCRIPT = $(COREPATH)/mk20dx256.ld
        LDFLAGS += -mcpu=cortex-m4 -T$(LDSCRIPT)
    else
        ifeq ($(TEENSY), LC)
            CPPFLAGS += -D__MKL26Z64__ -mcpu=cortex-m0plus
            LDSCRIPT = $(COREPATH)/mkl26z64.ld
            LDFLAGS += -mcpu=cortex-m0plus -T$(LDSCRIPT)
            LIBS += -larm_cortexM0l_math
        else
            $(error Invalid setting for TEENSY)
        endif
    endif
endif

# set arduino define if given
ifdef ARDUINO
	CPPFLAGS += -DARDUINO=$(ARDUINO)
else
	CPPFLAGS += -DUSING_MAKEFILE
endif

# names for the compiler programs
CC = $(abspath $(COMPILERPATH))/arm-none-eabi-gcc
CXX = $(abspath $(COMPILERPATH))/arm-none-eabi-g++
OBJCOPY = $(abspath $(COMPILERPATH))/arm-none-eabi-objcopy
SIZE = $(abspath $(COMPILERPATH))/arm-none-eabi-size



# automatically create lists of the sources and objects
LC_FILES := $(wildcard $(LIBRARYPATH)/*/*.c)
LCPP_FILES := $(wildcard $(LIBRARYPATH)/*/*.cpp)
TC_FILES := $(wildcard $(COREPATH)/*.c)
TCPP_FILES := $(wildcard $(COREPATH)/*.cpp)
C_FILES := $(wildcard src/*.c)
CPP_FILES := $(wildcard src/*.cpp)
INO_FILES := $(wildcard src/*.ino)
TESTC_FILES := $(wildcard $(TESTPATH)/*.c)
TESTCPP_FILES := $(wildcard $(TESTPATH)/*.cpp)


# include paths for libraries
L_INC = -Isrc
L_INC += -I$(LIBRARYPATH)
L_INC += $(foreach lib,$(filter %/, $(wildcard $(LIBRARYPATH)/*/)), -I$(lib))

SOURCES := $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o) $(TC_FILES:.c=.o) $(TCPP_FILES:.cpp=.o) $(LC_FILES:.c=.o) $(LCPP_FILES:.cpp=.o)
OBJS := $(foreach src,$(SOURCES), $(BUILDDIR)/$(src))

TESTABLESOURCES = src/PID.o src/GyroControl.o src/Sensor.o libraries/MadgwickAHRS/MadgwickAHRS.o src/PIDConf.o src/mix.o src/QuadState.o src/Control.o src/AngleControl.o src/Receiver.o src/EscControl.o host_src/dummyHostFunctions.o
TESTABLEOBJS := $(foreach src,$(TESTABLESOURCES), $(TESTBUILDDIR)/$(src))

TESTSOURCES = $(TESTC_FILES:.c=.o)
TESTOBJS := $(foreach src,$(TESTSOURCES), $(TESTBUILDDIR)/$(src))

MODELSOURCES = src/PID.o src/GyroControl.o src/Sensor.o libraries/MadgwickAHRS/MadgwickAHRS.o src/PIDConf.o src/mix.o src/QuadState.o src/Control.o src/AngleControl.o src/Receiver.o src/EscControl.o host_src/dummyHostFunctions.o
MODELOBJS := $(foreach src,$(MODELSOURCES), $(MODELBUILDDIR)/$(src))

MODELLIBRARY := $(MODELBUILDDIR)/teensyquad.so


all: hex

build: $(TARGET).elf

hex: $(TARGET).hex

post_compile: $(TARGET).hex
	@$(abspath $(TOOLSPATH))/teensy_post_compile -file="$(basename $<)" -path=$(CURDIR) -tools="$(abspath $(TOOLSPATH))"

reboot:
	@-$(abspath $(TOOLSPATH))/teensy_reboot
	
close_uploader:
	@sleep 2; pkill teensy

upload: post_compile reboot close_uploader

coverage: test
	mkdir -p build/coverage
	gcovr -r . --html --html-details -o build/coverage/coverage.html

model: CC = gcc
model: CPPFLAGS = -Wall -g -fPIC
model: CXXFLAGS = $(CPPFLAGS)
model: LDFLAGS = -lm -shared -lc
model: L_INC += -Imodels -Ihost_src
model: $(MODELLIBRARY)
	@echo "done"

$(MODELLIBRARY): $(MODELOBJS)
	$(CC) $(LDFLAGS) -o $@ $^
test: CC = gcc
test: CPPFLAGS = -Wall -g -fprofile-arcs -ftest-coverage
test: CXXFLAGS = $(CPPFLAGS)
test: LDFLAGS = -lcheck -lm -lgcov --coverage
test: L_INC += -Itests -Ihost_src
test: testbuild
	$(TESTBUILDDIR)/test

testbuild: $(TESTOBJS) $(TESTABLEOBJS)
	$(CC) $(LDFLAGS) -o $(TESTBUILDDIR)/test $(TESTOBJS) $(TESTABLEOBJS)

$(MODELBUILDDIR)/%.o: %.c
	@mkdir -p "$(dir $@)"
	$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(TESTBUILDDIR)/%.o: %.c
	@mkdir -p "$(dir $@)"
	$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.c
	@mkdir -p "$(dir $@)"
	$(CC) $(CPPFLAGS) $(CFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@mkdir -p "$(dir $@)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@mkdir -p "$(dir $@)"
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(L_INC) -o "$@" -x c++ -include Arduino.h -c "$<"

$(TARGET).elf: $(OBJS) $(LDSCRIPT)
	$(CC) $(LDFLAGS) -o "$@" $(OBJS) $(LIBS)

%.hex: %.elf
	@$(SIZE) "$<"
	$(OBJCOPY) -O ihex -R .eeprom "$<" "$@"

# compiler generated dependency info
-include $(OBJS:.o=.d)

clean:
	@echo Cleaning...
	@rm -rf "$(BUILDDIR)"
	@rm -f "$(TARGET).elf" "$(TARGET).hex"
