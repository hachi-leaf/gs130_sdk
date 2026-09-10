/* gs130-capture: stream stereo frames and IMU packets, printing a live rate/sync line.
 *
 * usage: gs130-capture <platform> <device> <mode> <w> <h> <fps> <odr>
 *
 * Runs until Ctrl+C, then shuts the device down cleanly.
 * The camera and IMU timestamps share one clock, so the printed delta shows the
 * hardware-level sync between them.
 */
#include "gs130.h"
#include "gs130_define.h"

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

static volatile sig_atomic_t g_running = 1;

static void on_sigint(int sig)
{
    (void)sig;
    g_running = 0;
}

static void sleep_ms(int ms)
{
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (long)(ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static double elapsed_s(const struct timespec *a, const struct timespec *b)
{
    return (double)(b->tv_sec - a->tv_sec) + (double)(b->tv_nsec - a->tv_nsec) / 1e9;
}

int main(int argc, char **argv)
{
    if(argc < 8){
        fprintf(stderr,
                "usage: gs130-capture <platform> <device> <mode> <w> <h> <fps> <odr>\n");
        return 1;
    }

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
    if(gs130_init(dev, &cfg) != GS130_OK){
        fprintf(stderr, "init failed\n");
        gs130_destroy(dev);
        return 1;
    }
    if(gs130_start(dev) != GS130_OK){
        fprintf(stderr, "start failed\n");
        gs130_deinit(dev);
        gs130_destroy(dev);
        return 1;
    }

    signal(SIGINT, on_sigint);

    printf("streaming: %s %s  mode=%s  %dx%d  %dfps  odr=%dHz\n", platform, device, mode_s, w, h, fps, odr);
    printf("press Ctrl+C to stop\n");

    unsigned long cam_n = 0, imu_n = 0, cam_last = 0, imu_last = 0;
    uint64_t cam_ts = 0, imu_ts = 0;
    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    while(g_running){
        // drain the camera queue: free() the buffers, ownership belongs to us
        while(gs130_available_camera(dev) > 0){
            gs130_image_nv12_t left, right;
            if(gs130_get_nv12_frame(dev, &left, &right) != GS130_OK)break;
            cam_ts = left.timestamp_ns;
            free(left.data);
            free(right.data);
            cam_n++;
        }
        // drain the IMU queue
        while(gs130_available_imu(dev) > 0){
            gs130_imu_packet_t pkt;
            if(gs130_get_imu_packet(dev, &pkt) != GS130_OK)break;
            imu_ts = pkt.timestamp_ns;
            imu_n++;
        }

        clock_gettime(CLOCK_MONOTONIC, &t1);
        const double dt = elapsed_s(&t0, &t1);
        if(dt >= 1.0){
            const double d_ms = (cam_ts != 0 && imu_ts != 0)
                                    ? (double)((int64_t)(cam_ts - imu_ts)) / 1e6
                                    : 0.0;
            printf("cam %6.1f fps | imu %7.1f Hz | cam_ts %10.3f ms  imu_ts %10.3f ms  delta %+8.3f ms\n",
                   (double)(cam_n - cam_last) / dt,
                   (double)(imu_n - imu_last) / dt,
                   (double)cam_ts / 1e6,
                   (double)imu_ts / 1e6,
                   d_ms);
            fflush(stdout);
            t0 = t1;
            cam_last = cam_n;
            imu_last = imu_n;
        }
        sleep_ms(200);
    }

    printf("\nstopping...\n");
    gs130_stop(dev);
    gs130_deinit(dev);
    gs130_destroy(dev);
    printf("stopped: %lu stereo frames, %lu imu packets\n", cam_n, imu_n);
    return 0;
}
