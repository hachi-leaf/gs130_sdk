/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 *
 * sample — GS130 SDK demo: dual-cam + IMU, table display, Ctrl+C to quit.
 *
 * Usage: ./sample [out_w=544] [out_h=640] [mode=2]  (mode: 0=RAW 1=RESIZE 2=RECT)
 */
#include "gs130.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static volatile int running = 1;
static void sig_handler(int sig){(void)sig;running = 0;}

static double now_s(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (double)t.tv_sec + (double)t.tv_nsec / 1e9;
}

static void print_cal(const gs130_calibration_t *cal)
{
    printf("\n---------- Calibration ----------\n");
    printf("install_angle = %d deg\n\n", cal->camera_install_angle);

    printf("+-----------+-------------+-------------+-------------+-------------+----------+\n");
    printf("| cam       | fx          | fy          | cx          | cy          | model    |\n");
    printf("+-----------+-------------+-------------+-------------+-------------+----------+\n");
    printf("| cam_right | %11.4f | %11.4f | %11.4f | %11.4f | %-8s |\n",
           cal->camera_right.fx, cal->camera_right.fy,
           cal->camera_right.cx, cal->camera_right.cy,
           cal->camera_right.dist_model == GS130_DIST_FISHEYE ? "FISHEYE" : "PINHOLE");
    printf("| cam_left  | %11.4f | %11.4f | %11.4f | %11.4f | %-8s |\n",
           cal->camera_left.fx, cal->camera_left.fy,
           cal->camera_left.cx, cal->camera_left.cy,
           cal->camera_left.dist_model == GS130_DIST_FISHEYE ? "FISHEYE" : "PINHOLE");
    printf("+-----------+-------------+-------------+-------------+-------------+----------+\n");

    for(int ci = 0; ci < 2; ci++){
        const gs130_camera_intrinsics_t *ck = (ci == 0) ? &cal->camera_right : &cal->camera_left;
        const char *cn = (ci == 0) ? "cam_right" : "cam_left";
        if(ck->dist_model == GS130_DIST_FISHEYE)
            printf("dist[%s] k1=%+.6f k2=%+.6f k3=%+.6f k4=%+.6f\n",
                   cn, ck->dist_coeffs[0], ck->dist_coeffs[1],
                   ck->dist_coeffs[2], ck->dist_coeffs[3]);
        else
            printf("dist[%s] k1=%+.6f k2=%+.6f p1=%+.6f p2=%+.6f k3=%+.6f k4=%+.6f k5=%+.6f k6=%+.6f\n",
                   cn, ck->dist_coeffs[0], ck->dist_coeffs[1],
                   ck->dist_coeffs[2], ck->dist_coeffs[3],
                   ck->dist_coeffs[4], ck->dist_coeffs[5],
                   ck->dist_coeffs[6], ck->dist_coeffs[7]);
    }

    /* R and T side by side */
    printf("\nR:\n");
    printf("         cam_right                          cam_left                           imu\n");
    for(int row = 0; row < 3; row++){
        const double *rr = &cal->camera_right_R[row * 3];
        const double *rl = &cal->camera_left_R[row * 3];
        const double *ri = &cal->imu_R[row * 3];
        printf("  [ %+.5f %+.5f %+.5f ]   [ %+.5f %+.5f %+.5f ]   [ %+.5f %+.5f %+.5f ]\n",
               rr[0], rr[1], rr[2], rl[0], rl[1], rl[2], ri[0], ri[1], ri[2]);
    }

    printf("T (m):\n");
    printf("  cam_right = [ %+.6f %+.6f %+.6f ]\n",
           cal->camera_right_T[0], cal->camera_right_T[1], cal->camera_right_T[2]);
    printf("  cam_left  = [ %+.6f %+.6f %+.6f ]\n",
           cal->camera_left_T[0], cal->camera_left_T[1], cal->camera_left_T[2]);
    printf("  imu       = [ %+.6f %+.6f %+.6f ]\n",
           cal->imu_T[0], cal->imu_T[1], cal->imu_T[2]);

    printf("\nIMU intrinsics:\n");
    printf("  accel: misalign=[%+.5f %+.5f %+.5f] scale=[%+.5f %+.5f %+.5f] bias=[%+.5f %+.5f %+.5f] noise=%.5f rw=%.5f\n",
           cal->imu.accel_misalign[0], cal->imu.accel_misalign[1], cal->imu.accel_misalign[2],
           cal->imu.accel_scale[0],    cal->imu.accel_scale[1],    cal->imu.accel_scale[2],
           cal->imu.accel_bias[0],     cal->imu.accel_bias[1],     cal->imu.accel_bias[2],
           cal->imu.accel_noise, cal->imu.accel_random_walk);
    printf("  gyro : misalign=[%+.5f %+.5f %+.5f] scale=[%+.5f %+.5f %+.5f] bias=[%+.5f %+.5f %+.5f] noise=%.5f rw=%.5f\n",
           cal->imu.gyro_misalign[0], cal->imu.gyro_misalign[1], cal->imu.gyro_misalign[2],
           cal->imu.gyro_scale[0],    cal->imu.gyro_scale[1],    cal->imu.gyro_scale[2],
           cal->imu.gyro_bias[0],     cal->imu.gyro_bias[1],     cal->imu.gyro_bias[2],
           cal->imu.gyro_noise, cal->imu.gyro_random_walk);
}

int main(int argc, char **argv)
{
    setbuf(stdout, NULL);
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    uint32_t out_w = 544, out_h = 640;
    int mode = 2;
    if(argc > 2){ out_w = (uint32_t)atoi(argv[1]); out_h = (uint32_t)atoi(argv[2]); }
    if(argc > 3) mode = atoi(argv[3]);

    gs130_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));

    cfg.camera_config.bus[0] = 4; cfg.camera_config.bus[1] = 6;
    cfg.camera_config.bus_num = 2;
    cfg.camera_config.left_addr  = 0x30;
    cfg.camera_config.right_addr = 0x32;
    cfg.camera_config.sensor_width  = 1088;
    cfg.camera_config.sensor_height = 1280;
    cfg.camera_config.fps = 30;
    cfg.camera_config.line_length  = 1400;
    cfg.camera_config.frame_length = 1500;
    cfg.camera_config.mode = (gs130_camera_mode_t)mode;
    cfg.camera_config.output_width  = out_w;
    cfg.camera_config.output_height = out_h;
    memset(cfg.camera_config.bus_mipi_rx, 0xFF, sizeof(cfg.camera_config.bus_mipi_rx));
    cfg.camera_config.bus_mipi_rx[4] = 2;
    cfg.camera_config.bus_mipi_rx[6] = 0;
    for(int i = 0; i < 32; i++) cfg.camera_config.bus_reset_gpio[i] = -1;
    cfg.camera_config.bus_reset_gpio[4] = 351;
    cfg.camera_config.bus_reset_gpio[6] = 353;
    cfg.camera_config.fsync_camera = GS130_CAMERA_RIGHT_IDX;

    cfg.imu_config.bus[0] = 4; cfg.imu_config.bus[1] = 6;
    cfg.imu_config.bus_num = 2;
    cfg.imu_config.addr = 0x68;
    cfg.imu_config.odr_hz = 200;
    cfg.imu_config.accel_fsr_g = 16;
    cfg.imu_config.gyro_fsr_dps = 2000;
    cfg.imu_config.accel_bw_sel = 0;
    cfg.imu_config.gyro_bw_sel = 0;

    cfg.eeprom_config.bus[0] = 4; cfg.eeprom_config.bus[1] = 6;
    cfg.eeprom_config.bus_num = 2;
    cfg.eeprom_config.addr = 0x50;

    cfg.camera_fifo.depth = 4;
    cfg.camera_fifo.mode  = GS130_FIFO_DROP_OLD;
    cfg.imu_fifo.depth = 1024;
    cfg.imu_fifo.mode  = GS130_FIFO_DROP_OLD;

    gs130_device_t *dev = gs130_create();
    if(!dev){ printf("FAIL: create\n"); return 1; }
    if(gs130_init(dev, &cfg) != GS130_OK){ printf("FAIL: init\n"); gs130_destroy(dev); return 1; }
    if(gs130_start(dev) != GS130_OK){ printf("FAIL: start\n"); gs130_deinit(dev); gs130_destroy(dev); return 1; }

    gs130_calibration_t cal;
    int has_cal = (gs130_get_calibration(dev, &cal) == GS130_OK);

    /* IMU timestamp dump hook: /tmp/imu_ts.txt, one "timestamp_ns fsync" per line */
    FILE *imu_dump = fopen("/tmp/imu_ts.txt", "w");
    /* camera frame ts dump: /tmp/cam_ts.txt, one "timestamp_ns" per line (right camera) */
    FILE *cam_dump = fopen("/tmp/cam_ts.txt", "w");
    if(imu_dump) printf("IMU dump -> /tmp/imu_ts.txt\n");

    uint64_t cam_count = 0, imu_count = 0, fsync_count = 0;
    gs130_image_nv12_t img_l = {0}, img_r = {0};
    gs130_imu_packet_t imu_pkt = {0};
    int has_cam = 0, has_imu = 0;

    uint64_t cam_ts_buf[50]  = {0}; int cam_ts_head = 0, cam_ts_cnt = 0;
    uint64_t imu_ts_buf[100] = {0}; int imu_ts_head = 0, imu_ts_cnt = 0;

    double t0 = now_s();

    while(running){
        while(gs130_available_camera(dev) > 0){
            free(img_l.y); free(img_l.uv); free(img_r.y); free(img_r.uv);
            img_l.y = img_l.uv = img_r.y = img_r.uv = NULL;
            if(gs130_get_nv12_frame(dev, &img_l, &img_r) != GS130_OK)break;
            cam_count++;
            has_cam = 1;
            cam_ts_buf[cam_ts_head] = img_r.timestamp_ns;
            cam_ts_head = (cam_ts_head + 1) % 50;
            if(cam_ts_cnt < 50) cam_ts_cnt++;
            if(cam_dump) fprintf(cam_dump, "%llu\n", (unsigned long long)img_r.timestamp_ns);
        }

        while(gs130_available_imu(dev) > 0){
            if(gs130_read_imu(dev, &imu_pkt) != GS130_OK)break;
            imu_count++;
            if(imu_pkt.is_fsync)fsync_count++;
            has_imu = 1;
            if(imu_dump) fprintf(imu_dump, "%llu %d\n",
                                 (unsigned long long)imu_pkt.timestamp_ns,
                                 imu_pkt.is_fsync ? 1 : 0);
            imu_ts_buf[imu_ts_head] = imu_pkt.timestamp_ns;
            imu_ts_head = (imu_ts_head + 1) % 100;
            if(imu_ts_cnt < 100) imu_ts_cnt++;
        }

        usleep(100000);

        double cam_fps = 0.0, imu_hz = 0.0;
        if(cam_ts_cnt >= 2){
            int tail = (cam_ts_head - cam_ts_cnt + 50) % 50;
            int prev = (cam_ts_head - 1 + 50) % 50;
            double dt = (double)(cam_ts_buf[prev] - cam_ts_buf[tail]) / 1e9;
            if(dt > 0.0) cam_fps = (double)(cam_ts_cnt - 1) / dt;
        }
        if(imu_ts_cnt >= 2){
            int tail = (imu_ts_head - imu_ts_cnt + 100) % 100;
            int prev = (imu_ts_head - 1 + 100) % 100;
            double dt = (double)(imu_ts_buf[prev] - imu_ts_buf[tail]) / 1e9;
            if(dt > 0.0) imu_hz = (double)(imu_ts_cnt - 1) / dt;
        }

        double elapsed = now_s() - t0;

        printf("\033[2J\033[H");
        printf("========== GS130 SDK | mode=%d out=%ux%u ==========\n", mode, out_w, out_h);
        printf("run %.1f s | cam %llu pairs | imu %llu samples (%llu FSYNC)\n\n",
               elapsed, (unsigned long long)cam_count,
               (unsigned long long)imu_count, (unsigned long long)fsync_count);

        printf("---------- Camera ----------\n");
        printf("+------+--------------------+------------+--------+\n");
        printf("| ch   | timestamp (ns)     | resolution | fps    |\n");
        printf("+------+--------------------+------------+--------+\n");
        if(has_cam){
            char r1[32], r2[32];
            snprintf(r1, sizeof(r1), "%ux%u", img_r.width, img_r.height);
            snprintf(r2, sizeof(r2), "%ux%u", img_l.width, img_l.height);
            printf("| right| %18llu | %-10s | %6.1f |\n",
                   (unsigned long long)img_r.timestamp_ns, r1, cam_fps);
            printf("| left | %18llu | %-10s | %6.1f |\n",
                   (unsigned long long)img_l.timestamp_ns, r2, cam_fps);
        }else{
            printf("| right| %18s | %-10s | %6s |\n", "-", "-", "-");
            printf("| left | %18s | %-10s | %6s |\n", "-", "-", "-");
        }
        printf("+------+--------------------+------------+--------+\n");

        printf("\n---------- IMU (ODR=%u Hz) ----------\n", cfg.imu_config.odr_hz);
        printf("+----------------------------+----------------------------+----------+----------+\n");
        printf("| %-26s | %-26s | %-8s | %-8s |\n",
               "accel (m/s^2)", "gyro (rad/s)", "temp (C)", "rate");
        printf("+----------------------------+----------------------------+----------+----------+\n");
        if(has_imu){
            printf("| %8.3f %8.3f %8.3f | %8.3f %8.3f %8.3f | %8.2f | %8.1f |\n",
                   imu_pkt.accel[0], imu_pkt.accel[1], imu_pkt.accel[2],
                   imu_pkt.gyro[0],  imu_pkt.gyro[1],  imu_pkt.gyro[2],
                   imu_pkt.temp, imu_hz);
        }else{
            printf("| %-26s | %-26s | %-8s | %-8s |\n", "-", "-", "-", "-");
        }
        printf("+----------------------------+----------------------------+----------+----------+\n");

        printf("+----------------------+--------+-----------+-------------------+\n");
        printf("| %-20s | %-6s | %-9s | %-17s |\n",
               "ts (ns)", "fsync", "total", "imu-cam (ms)");
        printf("+----------------------+--------+-----------+-------------------+\n");
        if(has_imu){
            printf("| %-20llu | %-6d | %-9llu | %-17.3f |\n",
                   (unsigned long long)imu_pkt.timestamp_ns, imu_pkt.is_fsync ? 1 : 0,
                   (unsigned long long)fsync_count,
                   has_cam ? (double)((long long)imu_pkt.timestamp_ns - (long long)img_r.timestamp_ns) / 1e6 : 0.0);
        }else{
            printf("| %-20s | %-6s | %-9s | %-17s |\n", "-", "-", "-", "-");
        }
        printf("+----------------------+--------+-----------+-------------------+\n");

        if(has_cal) print_cal(&cal);
        else printf("\n---------- Calibration ----------\n(no eeprom / calibration)\n");

        fflush(stdout);
    }
    printf("\n");

    free(img_l.y); free(img_l.uv); free(img_r.y); free(img_r.uv);
    if(imu_dump) fclose(imu_dump);
    if(cam_dump) fclose(cam_dump);

    gs130_stop(dev);
    gs130_deinit(dev);
    gs130_destroy(dev);
    return 0;
}
