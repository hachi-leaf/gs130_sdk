/**
 * @file gs130-eeprom-info.c
 * @brief Probe and print the EEPROM calibration model and its details
 *
 * This file is part of gs130_sdk (https://github.com/hachi-leaf/gs130_sdk).
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 * See the LICENSE file in the project root for the full license text.
 */
#include "gs130.h"
#include "gs130_define.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    if(argc < 8)return 1;
    const char *platform = argv[1];
    const char *device   = argv[2];
    const char *mode_s   = argv[3];
    int w   = atoi(argv[4]);
    int h   = atoi(argv[5]);
    int fps = atoi(argv[6]);
    int odr = atoi(argv[7]);

    gs130_camera_mode_t mode = GS130_CAMERA_MODE_RAW;
    if(!strcmp(mode_s, "resize"))     mode = GS130_CAMERA_MODE_RESIZE;
    else if(!strcmp(mode_s, "rect"))  mode = GS130_CAMERA_MODE_RECT;

    gs130_config_t cfg = GS130_CONFIG(platform, device, mode, w, h, fps, odr);
    gs130_device_t *dev = gs130_create();
    if(gs130_init(dev, &cfg) != GS130_OK){ fprintf(stderr, "init failed\n"); gs130_destroy(dev); return 1; }

    const char *name = gs130_get_eeprom_name(dev);
    if(!name){
        printf("no EEPROM detected\n");
    } else {
        const char *info = gs130_get_eeprom_info(dev);
        printf("eeprom name: %s\n%s\n", name, info ? info : "");
    }

    gs130_deinit(dev);
    gs130_destroy(dev);
    return 0;
}
