/**
 * @file pipeline.cpp
 * @brief Platform dispatch (compile time): explicitly define the target runtime platform macro at build time.
 *
 *   -DRDKX5_RUNTIME   / -DRDKS100_RUNTIME / -DRDKS600_RUNTIME
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "devices/pipeline/pipeline.hpp"

#if defined(RDKX5_RUNTIME)
#  include "devices/pipeline/rdkx5/rdkx5.cpp"
#elif defined(RDKS100_RUNTIME) || defined(RDKS600_RUNTIME)
#  error "S100/S600 pipeline not yet implemented"
#else
#  error "No platform selected: define RDKX5_RUNTIME / RDKS100_RUNTIME / RDKS600_RUNTIME"
#endif
