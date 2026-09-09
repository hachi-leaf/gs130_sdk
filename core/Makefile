# GS130 SDK - Core library Makefile
#
#   make                      build the library (default platform: rdkx5)
#   make <platform>           build for a platform (a dir name under src/devices/pipeline/)
#   make sample               build the sample program
#   make clean                clean

# ---- Dependencies ----

# OpenCV static linking (self-contained; no runtime .so dependency)
OPENCV_LIBS := -Wl,--start-group \
    /usr/lib/aarch64-linux-gnu/libopencv_calib3d.a \
    /usr/lib/aarch64-linux-gnu/libopencv_features2d.a \
    /usr/lib/aarch64-linux-gnu/libopencv_flann.a \
    /usr/lib/aarch64-linux-gnu/libopencv_core.a \
    /usr/lib/aarch64-linux-gnu/libopencv_imgproc.a \
    -Wl,--end-group

# Horizon runtime libraries (dynamic linking)
HBN_LIB_DIR := /usr/hobot/lib
HBN_LIBS    := -lvpf -lhbmem -lcam

# ---- Platform ----
# Available platforms = subdirectory names under src/devices/pipeline/.
# 'make <platform>' builds for it; a bare 'make' defaults to rdkx5.
PLATFORMS := $(patsubst src/devices/pipeline/%/,%,$(wildcard src/devices/pipeline/*/))

GOAL := $(firstword $(MAKECMDGOALS))
ifneq ($(filter $(GOAL),$(PLATFORMS)),)
PLATFORM := $(GOAL)
endif
PLATFORM ?= rdkx5

# the platform backend .cpp is #included by path (not compiled standalone)
PLATFORM_IMPL = -DGS130_PLATFORM_IMPL='"devices/pipeline/$(PLATFORM)/$(PLATFORM).cpp"'

# ---- Toolchain ----

CC  = gcc
CXX = g++

# ---- Flags ----

COMMON_FLAGS = -Wall -Wextra -O2 -g -fPIC
CFLAGS   = $(COMMON_FLAGS) -std=gnu11
CXXFLAGS = $(COMMON_FLAGS) -std=gnu++17 $(PLATFORM_IMPL)
INCLUDES = -Iinclude -Isrc -I/usr/include/opencv4
LDFLAGS  = -L$(HBN_LIB_DIR)
LIBS     = $(HBN_LIBS) -lm -lpthread -lrt $(OPENCV_LIBS) \
           -lz /usr/lib/aarch64-linux-gnu/libtbb.so.2 -ldl

# ---- Sources ----

# source path -> object path under build/  (.c and .cpp both -> build/....o)
to_obj = $(patsubst %.cpp,build/%.o,$(patsubst %.c,build/%.o,$1))

# auto-discovered.
# NOTE: a platform .cpp under pipeline/<platform>/ (e.g. rdkx5.cpp) is #included by
#       pipeline.cpp (single-TU dispatch), so it must NOT be compiled as a separate object.
#       The .c node files in those dirs ARE compiled normally.
C_SRCS   := $(wildcard src/devices/pipeline/$(PLATFORM)/*.c)
CPP_SRCS := $(shell find src -type f -name '*.cpp' ! -path '*/devices/pipeline/*/*.cpp')
OBJS     := $(call to_obj,$(C_SRCS) $(CPP_SRCS))

# ---- Outputs ----

VERSION       := $(shell cat ../VERSION)
VERSION_MAJOR := $(shell echo $(VERSION) | cut -d. -f1)

OUT_DIR      = out
LIB_STATIC   = $(OUT_DIR)/libgs130.a
LIB_FULL     = $(OUT_DIR)/libgs130.so.$(VERSION)
LIB_SO_MAJOR = $(OUT_DIR)/libgs130.so.$(VERSION_MAJOR)
LIB_SHARED   = $(OUT_DIR)/libgs130.so

# ---- Targets ----

all: lib

# each platform directory name is itself a build target: 'make rdkx5' etc.
.PHONY: $(PLATFORMS)
$(PLATFORMS): all

lib: $(LIB_STATIC) $(LIB_FULL)

sample: $(OUT_DIR)/sample

$(OUT_DIR)/sample: examples/sample.c $(LIB_FULL)
	@mkdir -p build $(OUT_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o build/sample.o
	$(CXX) build/sample.o -o $@ -L$(OUT_DIR) -lgs130 $(LDFLAGS) -lstdc++ -Wl,-rpath,$(CURDIR)/$(OUT_DIR)

$(LIB_STATIC): $(OBJS)
	@mkdir -p $(OUT_DIR)
	ar rcs $@ $^

$(LIB_FULL): $(OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) -shared $^ -o $@ $(LDFLAGS) $(LIBS) -Wl,-rpath,$(HBN_LIB_DIR)
	@ln -sf $(notdir $(LIB_FULL)) $(LIB_SO_MAJOR)
	@ln -sf $(notdir $(LIB_SO_MAJOR)) $(LIB_SHARED)

# ---- Rules ----

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# ---- Clean ----

clean:
	rm -rf build out

.PHONY: all lib sample clean

# ---- Per-module targets ----
# Compile a single module on demand; 'base' / 'devices' build their whole group.
# Sources are auto-discovered, so adding a file needs no Makefile edit.

# base module
.PHONY: base fifo i2c rectify tracker

base:    fifo i2c rectify tracker

fifo:                          # header-only, nothing to compile
	@echo "fifo: header-only, nothing to compile"

i2c:     $(call to_obj,$(wildcard src/base/i2c/*.cpp))
rectify: $(call to_obj,$(wildcard src/base/rectify/*.cpp))
tracker: $(call to_obj,$(wildcard src/base/tracker/*.cpp))

# devices module
.PHONY: devices eeprom imu pipeline

devices: eeprom imu pipeline

eeprom:   $(call to_obj,$(wildcard src/devices/eeprom/*.cpp))
imu:      $(call to_obj,$(wildcard src/devices/imu/*.cpp))
# pipeline glue (pipeline.cpp) + the selected platform's .c node helpers;
# the platform's .cpp backend is #included by pipeline.cpp, not compiled standalone
pipeline: $(call to_obj,$(wildcard src/devices/pipeline/*.cpp) $(wildcard src/devices/pipeline/$(PLATFORM)/*.c))
