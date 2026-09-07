/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * 平台分发（编译期）：构建时显式定义目标运行时平台宏。
 *   -DRDKX5_RUNTIME   / -DRDKS100_RUNTIME / -DRDKS600_RUNTIME
 */
#include "devices/pipeline/pipeline.hpp"

#if defined(RDKX5_RUNTIME)
#  include "devices/pipeline/rdkx5/rdkx5.cpp"
#elif defined(RDKS100_RUNTIME) || defined(RDKS600_RUNTIME)
#  error "S100/S600 pipeline 尚未实现"
#else
#  error "未选择平台：请定义 RDKX5_RUNTIME / RDKS100_RUNTIME / RDKS600_RUNTIME"
#endif
