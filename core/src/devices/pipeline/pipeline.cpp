/**
 * @file pipeline.cpp
 * @brief Platform dispatch: the platform backend .cpp is #included here, by path.
 *
 * The build system selects the platform by directory name and passes the backend
 * file path as -DGS130_PLATFORM_IMPL="devices/pipeline/<platform>/<platform>.cpp".
 * Adding a platform = create src/devices/pipeline/<name>/<name>.cpp; nothing else to touch.
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/pipeline.hpp"

#ifndef GS130_PLATFORM_IMPL
#  error "No platform selected: the Makefile defines GS130_PLATFORM_IMPL from the platform directory name"
#endif

#include GS130_PLATFORM_IMPL
