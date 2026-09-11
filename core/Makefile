# GS130 SDK - core library, samples and tools
#
#   make                      build everything for the default platform (rdkx5),
#                             including the .deb package
#   make <platform>           build everything for a platform
#                             (a directory name under src/devices/pipeline/)
#   make lib                  build the library only
#   make samples              build the sample programs only
#   make tools                build the internal diagnostic tools only
#   make deb                  pack the .deb (also part of a plain 'make')
#   make clean                remove build/ and out/
#
# 'make -j8' is safe and much faster on the X5 (8 cores: ~15 s instead of ~40 s).
#
# Layout: objects and their .d files in build/<platform>/,
#         library and binaries in out/<platform>/,
#         the .deb package in out/ (next to the platform directories, not inside one).
# A header change rebuilds exactly the objects that include it: the compiler
# writes the dependency lists with -MMD and this Makefile includes them below.

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
# platform name string, embedded for gs130_platform()
PLATFORM_STR  = -DGS130_PLATFORM='"$(PLATFORM)"'

# ---- Version ----

VERSION_FILE := ../VERSION
VERSION      := $(shell cat $(VERSION_FILE) 2>/dev/null)
ifeq ($(strip $(VERSION)),)
$(error $(VERSION_FILE) is missing or empty)
endif
VERSION_MAJOR := $(shell echo $(VERSION) | cut -d. -f1)
VERSION_DEF   = -DGS130_VERSION='"$(VERSION)"'

# ---- Toolchain ----

CC  = gcc
CXX = g++

# ---- Flags ----

COMMON_FLAGS = -Wall -Wextra -O2 -g -fPIC -MMD -MP
CFLAGS   = $(COMMON_FLAGS) -std=gnu11
CXXFLAGS = $(COMMON_FLAGS) -std=gnu++17 $(PLATFORM_IMPL) $(VERSION_DEF) $(PLATFORM_STR)
INCLUDES = -Iinclude -Isrc -I/usr/include/opencv4
# Linking against the static OpenCV archives dominates every link; ld.gold is about
# 3.5x faster than the default ld.bfd (2.4 s -> 0.7 s) and produces the same library.
# Used when it is installed, otherwise the default linker is kept.
LDFLAGS_GOLD = $(shell command -v ld.gold >/dev/null 2>&1 && echo -fuse-ld=gold)
# glibc stays dynamic: libc.so.6 / libm.so.6 / ld.so cannot be linked into a shared
# library, and libpthread/librt/libdl are part of libc here (no NEEDED of their own).
# zlib is linked statically, so libz.so.1 is not a runtime dependency either.
# The static OpenCV core calls into TBB and the image ships no libtbb.a, so
# libtbb.so.2 is the only remaining non-glibc, non-Horizon runtime dependency.
GLIBC_LIBS  := -lm -lpthread -lrt -ldl
ZLIB_STATIC := /usr/lib/aarch64-linux-gnu/libz.a
TBB_LIB     := /usr/lib/aarch64-linux-gnu/libtbb.so.2
# -static-libstdc++/-static-libgcc embed the C++ runtime, dropping libstdc++.so.6 and
# libgcc_s.so.1 from NEEDED; --exclude-libs,ALL keeps the symbols taken from those
# static archives out of the dynamic symbol table, so only the C API and the
# library's own symbols are exported.
LDFLAGS_SELF := -static-libstdc++ -static-libgcc -Wl,--exclude-libs,ALL
LDFLAGS  = -L$(HBN_LIB_DIR) $(LDFLAGS_SELF)
LIBS     = $(HBN_LIBS) $(GLIBC_LIBS) $(OPENCV_LIBS) $(ZLIB_STATIC) $(TBB_LIB)

# ---- Sources ----

# source path -> object path under build/$(PLATFORM)/  (.c and .cpp both -> build/<platform>/....o)
to_obj = $(patsubst %.cpp,build/$(PLATFORM)/%.o,$(patsubst %.c,build/$(PLATFORM)/%.o,$1))

# auto-discovered.
# NOTE: a platform .cpp under pipeline/<platform>/ (e.g. rdkx5.cpp) is #included by
#       pipeline.cpp (single-TU dispatch), so it must NOT be compiled as a separate object.
#       The .c node files in those dirs ARE compiled normally.
C_SRCS   := $(wildcard src/devices/pipeline/$(PLATFORM)/*.c)
CPP_SRCS := $(shell find src -type f -name '*.cpp' ! -path '*/devices/pipeline/*/*.cpp' ! -path '*/tools/*')
OBJS     := $(call to_obj,$(C_SRCS) $(CPP_SRCS))

# ---- Outputs ----

OUT_ROOT     = out
OUT_DIR      = $(OUT_ROOT)/$(PLATFORM)
LIB_STATIC   = $(OUT_DIR)/libgs130.a
LIB_FULL     = $(OUT_DIR)/libgs130.so.$(VERSION)
LIB_SO_MAJOR = $(OUT_DIR)/libgs130.so.$(VERSION_MAJOR)
LIB_SHARED   = $(OUT_DIR)/libgs130.so

# ---- Targets ----

# 'deb' is named rather than $(DEB_FILE) so this line does not depend on variables
# that are defined further down; the package rule itself lists its prerequisites.
all: lib samples tools deb

lib: $(LIB_STATIC) $(LIB_FULL)

# each platform directory name is itself a build target: 'make rdkx5' etc.
.PHONY: $(PLATFORMS)
$(PLATFORMS): all

# ---- Samples ----
# Each samples/*.c becomes a standalone executable; samples/gs130 dispatches to them.

SAMPLE_SRCS := $(wildcard samples/*.c)
SAMPLE_BINS := $(patsubst samples/%.c,$(OUT_DIR)/%,$(SAMPLE_SRCS))
SAMPLE_OBJS := $(patsubst samples/%.c,build/$(PLATFORM)/%.o,$(SAMPLE_SRCS))

samples: $(SAMPLE_BINS) $(OUT_DIR)/gs130

# Static pattern rule: these objects are explicit targets, not intermediates made by
# a pattern rule. make would otherwise treat an object that is only ever a prerequisite
# as an intermediate file and drop it, forcing a rebuild on every run.
$(SAMPLE_OBJS): build/$(PLATFORM)/%.o: samples/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

$(OUT_DIR)/gs130: samples/gs130
	@mkdir -p $(OUT_DIR)
	cp $< $@
	chmod +x $@

# ---- Tools ----
# Internal diagnostic tools (private headers), dispatched by the gs130 script.

TOOL_SRCS := $(wildcard src/tools/*.cpp)
TOOL_BINS := $(patsubst src/tools/%.cpp,$(OUT_DIR)/%,$(TOOL_SRCS))
TOOL_OBJS := $(patsubst src/tools/%.cpp,build/$(PLATFORM)/%.o,$(TOOL_SRCS))

tools: $(TOOL_BINS)

$(TOOL_OBJS): build/$(PLATFORM)/%.o: src/tools/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# ---- Binaries ----
# Compiling is separate from linking so each .o is a real target: the .d files below
# then make a header change rebuild the objects that include it.
# The library is an order-only prerequisite ('|'): linking needs it to exist, but a
# rebuilt library does not change these binaries (they link it dynamically), so they
# are not relinked on every library change. VERSION is a normal prerequisite because
# a major version bump changes the SONAME they record.

$(OUT_DIR)/%: build/$(PLATFORM)/%.o $(VERSION_FILE) | $(LIB_FULL)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(LDFLAGS_GOLD) $< -o $@ -L$(OUT_DIR) -lgs130 -Wl,-rpath,'$$ORIGIN'

# ---- Library ----

$(LIB_STATIC): $(OBJS)
	@mkdir -p $(OUT_DIR)
	ar rcs $@ $^

$(LIB_FULL): $(OBJS)
	@mkdir -p $(OUT_DIR)
	$(CXX) $(LDFLAGS_GOLD) -shared $^ -o $@ -Wl,-soname,$(notdir $(LIB_SO_MAJOR)) \
	    $(LDFLAGS) $(LIBS) -Wl,-rpath,$(HBN_LIB_DIR)
	@ln -sf $(notdir $(LIB_FULL)) $(LIB_SO_MAJOR)
	@ln -sf $(notdir $(LIB_SO_MAJOR)) $(LIB_SHARED)

# ---- Rules ----

build/$(PLATFORM)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

build/$(PLATFORM)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# ---- Dependencies ----
# -MMD -MP above makes the compiler write one .d per object, listing every header
# that object includes. Including them here gives make the real dependency graph,
# so editing a header rebuilds exactly the objects that include it.

DEPS := $(OBJS:.o=.d) $(SAMPLE_OBJS:.o=.d) $(TOOL_OBJS:.o=.d)

-include $(DEPS)

# ---- Clean ----

clean:
	rm -rf build out

.PHONY: all lib samples tools deb clean

# ---- Package ----
# 'make deb' builds everything and packs it into a single .deb: the shared and static
# library, both headers, and the gs130 command together with the helper binaries it
# calls from its own directory. The package is assembled with dpkg-deb directly, so
# the build needs no debhelper/fakeroot and no network.
# The platform is compiled into the library, so it rides in the version and not in the
# package name: gs130_<version>+<platform>_<arch>.deb. A repack that does not change the
# version appends a counter to the platform label, e.g. 0.0.1+rdkx5.1.

MULTIARCH     := $(shell dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || gcc -dumpmachine)
# dpkg's word for the architecture. uname says 'aarch64' for the same machine, which
# is not a valid .deb architecture, so map it for hosts without dpkg.
DEB_ARCH      := $(shell dpkg --print-architecture 2>/dev/null || uname -m | sed 's/aarch64/arm64/')
DEB_NAME      := gs130
DEB_VERSION   := $(VERSION)+$(PLATFORM)
DEB_FILE      := $(OUT_ROOT)/$(DEB_NAME)_$(DEB_VERSION)_$(DEB_ARCH).deb
DEB_ROOT      := build/deb/$(DEB_NAME)
DEB_MAINTAINER ?= hachi_leaf <xiaoye.zhang@d-robotics.cc>
# xz -0 instead of the dpkg default (zstd -19): it compresses this ~10 MB payload in
# 0.5 s instead of 13 s, for 8% more package size. Since a plain 'make' packs as well,
# that difference would otherwise show up on every build.

deb: $(DEB_FILE)

$(DEB_FILE): $(LIB_STATIC) $(LIB_FULL) $(SAMPLE_BINS) $(TOOL_BINS) $(OUT_DIR)/gs130 \
              $(VERSION_FILE)
	@mkdir -p $(OUT_ROOT)
	rm -rf $(DEB_ROOT)
	mkdir -p $(DEB_ROOT)/DEBIAN $(DEB_ROOT)/usr/bin $(DEB_ROOT)/usr/include \
	         $(DEB_ROOT)/usr/lib/$(MULTIARCH) $(DEB_ROOT)/usr/share/doc/$(DEB_NAME)
	install -m644 include/gs130.h include/gs130_define.h $(DEB_ROOT)/usr/include
	install -m644 $(LIB_STATIC) $(DEB_ROOT)/usr/lib/$(MULTIARCH)
	install -m755 $(LIB_FULL) $(DEB_ROOT)/usr/lib/$(MULTIARCH)
	ln -sf $(notdir $(LIB_FULL)) $(DEB_ROOT)/usr/lib/$(MULTIARCH)/libgs130.so.$(VERSION_MAJOR)
	ln -sf libgs130.so.$(VERSION_MAJOR) $(DEB_ROOT)/usr/lib/$(MULTIARCH)/libgs130.so
	install -m755 $(SAMPLE_BINS) $(TOOL_BINS) $(OUT_DIR)/gs130 $(DEB_ROOT)/usr/bin
	install -m644 ../LICENSE $(DEB_ROOT)/usr/share/doc/$(DEB_NAME)/copyright
	{ \
	    echo "Package: $(DEB_NAME)"; \
	    echo "Version: $(DEB_VERSION)"; \
	    echo "Architecture: $(DEB_ARCH)"; \
	    echo "Maintainer: $(DEB_MAINTAINER)"; \
	    echo "Section: misc"; \
	    echo "Priority: optional"; \
	    echo "Depends: libc6 (>= 2.34), libtbb2, hobot-multimedia"; \
	    echo "Description: GS130 stereo camera and IMU SDK for $(PLATFORM)"; \
	    echo " Camera and IMU capture library with the gs130 command line tool."; \
	    echo " The $(PLATFORM) backend is compiled into the library."; \
	} > $(DEB_ROOT)/DEBIAN/control
	printf '#!/bin/sh\nset -e\nldconfig\n' > $(DEB_ROOT)/DEBIAN/postinst
	cp $(DEB_ROOT)/DEBIAN/postinst $(DEB_ROOT)/DEBIAN/postrm
	chmod 755 $(DEB_ROOT)/DEBIAN/postinst $(DEB_ROOT)/DEBIAN/postrm
	dpkg-deb -Zxz -z0 --root-owner-group --build $(DEB_ROOT) $@

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
