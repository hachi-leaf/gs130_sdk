/**
 * @file gs130-version.c
 * @brief Print the SDK version and the build platform
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include <stdio.h>

int main(void)
{
    printf("gs130_sdk %s (platform: %s)\n", gs130_version(), gs130_platform());
    return 0;
}
