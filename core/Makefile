# GS130W SDK - Core library Makefile
#
#   make                      构建（默认 rdkx5 平台）
#   make PLATFORM=rdks100      构建 S100 平台
#   make PLATFORM=rdks600      构建 S600 平台
#   make sample                构建示例程序
#   make clean                 清理

# 平台选择：rdkx5 / rdks100 / rdks600
PLATFORM ?= rdkx5

ifeq ($(PLATFORM),rdkx5)
PLATFORM_DEF = -DRDKX5_RUNTIME
else ifeq ($(PLATFORM),rdks100)
PLATFORM_DEF = -DRDKS100_RUNTIME
else ifeq ($(PLATFORM),rdks600)
PLATFORM_DEF = -DRDKS600_RUNTIME
else
$(error 未知平台：PLATFORM=$(PLATFORM)，可选 rdkx5 / rdks100 / rdks600)
endif

VERSION := $(shell cat ../VERSION)
VERSION_MAJOR := $(shell echo $(VERSION) | cut -d. -f1)

CC  = gcc
CXX = g++

COMMON_FLAGS = -Wall -Wextra -O2 -g -fPIC
CFLAGS   = $(COMMON_FLAGS) -std=gnu11
CXXFLAGS = $(COMMON_FLAGS) -std=gnu++17 $(PLATFORM_DEF)
INCLUDES = -Iinclude -Isrc -I/usr/include/opencv4

# OpenCV 静态链接（自包含，不依赖运行时 .so）
OPENCV_LIBS := -Wl,--start-group \
    /usr/lib/aarch64-linux-gnu/libopencv_calib3d.a \
    /usr/lib/aarch64-linux-gnu/libopencv_features2d.a \
    /usr/lib/aarch64-linux-gnu/libopencv_flann.a \
    /usr/lib/aarch64-linux-gnu/libopencv_core.a \
    /usr/lib/aarch64-linux-gnu/libopencv_imgproc.a \
    -Wl,--end-group

LDFLAGS = -L/usr/hobot/lib
LIBS = -lvpf -lhbmem -lcam -lm -lpthread -lrt $(OPENCV_LIBS) \
       -lz /usr/lib/aarch64-linux-gnu/libtbb.so.2 -ldl

C_SRCS = \
    src/devices/pipeline/rdkx5/camera.c \
    src/devices/pipeline/rdkx5/vin.c \
    src/devices/pipeline/rdkx5/isp.c \
    src/devices/pipeline/rdkx5/vse.c \
    src/devices/pipeline/rdkx5/gdc.c \
    src/devices/pipeline/rdkx5/flow.c

CPP_SRCS = \
    src/gs130w.cpp \
    src/base/i2c/i2c.cpp \
    src/base/rectify/rectify.cpp \
    src/base/tracker/tracker.cpp \
    src/devices/eeprom/eeprom.cpp \
    src/devices/eeprom/union_stereo_imu_fisheye_v1p2r0n4.cpp \
    src/devices/imu/imu.cpp \
    src/devices/imu/icm42688.cpp \
    src/devices/pipeline/pipeline.cpp

C_OBJS   = $(patsubst %.c,build/%.o,$(C_SRCS))
CPP_OBJS = $(patsubst %.cpp,build/%.o,$(CPP_SRCS))
OBJS = $(C_OBJS) $(CPP_OBJS)

OUT_DIR = out
LIB_STATIC = $(OUT_DIR)/libgs130w.a
LIB_FULL   = $(OUT_DIR)/libgs130w.so.$(VERSION)
LIB_SO_MAJOR = $(OUT_DIR)/libgs130w.so.$(VERSION_MAJOR)
LIB_SHARED = $(OUT_DIR)/libgs130w.so

all: lib

sample: $(OUT_DIR)/sample

$(OUT_DIR)/sample: examples/sample.c $(LIB_FULL) | $(OUT_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o build/sample.o
	$(CXX) build/sample.o -o $@ -L$(OUT_DIR) -lgs130w $(LDFLAGS) -lstdc++ -Wl,-rpath,$(shell pwd)/$(OUT_DIR)

lib: $(LIB_STATIC) $(LIB_FULL)

$(LIB_STATIC): $(OBJS) | $(OUT_DIR)
	ar rcs $@ $^

$(LIB_FULL): $(OBJS) | $(OUT_DIR)
	$(CXX) -shared $^ -o $@ $(LDFLAGS) $(LIBS) -Wl,-rpath,/usr/hobot/lib
	@ln -sf $(notdir $(LIB_FULL)) $(LIB_SO_MAJOR)
	@ln -sf $(notdir $(LIB_SO_MAJOR)) $(LIB_SHARED)

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

$(OUT_DIR):
	@mkdir -p $@

clean:
	rm -rf build out

.PHONY: all lib clean
