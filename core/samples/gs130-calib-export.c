/* gs130-calib-export: export the EEPROM calibration as Kalibr-format YAML.
 *
 * usage: gs130-calib-export <output_dir> <platform> <device> <mode> <w> <h> <fps> <odr>
 *
 *   writes  <output_dir>/camchain.yaml     camera intrinsics + extrinsics (always)
 *           <output_dir>/imu.yaml          IMU noise + intrinsics (only if the device has an IMU)
 *
 * The file names are fixed; only the output directory is configurable.
 * The output directory is created (mkdir -p) when missing.
 */
#include "gs130.h"
#include "gs130_define.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

/* ---- misc helpers ---- */

// create <path> and any missing parent, like "mkdir -p"
static int mkdir_p(const char *path)
{
    char tmp[512];
    snprintf(tmp, sizeof(tmp), "%s", path);

    for(char *p = tmp + 1; *p != '\0'; p++){
        if(*p != '/')continue;
        *p = '\0';
        if(mkdir(tmp, 0777) != 0 && errno != EEXIST)return -1;
        *p = '/';
    }
    if(mkdir(tmp, 0777) != 0 && errno != EEXIST)return -1;
    return 0;
}

/* ---- Kalibr YAML helpers ---- */

// 4x4 homogeneous block: [R | T] with a [0 0 0 1] last row
static void write_transform(FILE *f, const char *key, const double *R, const double *T)
{
    fprintf(f, "  %s:\n", key);
    for(int i = 0; i < 3; i++)
        fprintf(f, "  - [%.12g, %.12g, %.12g, %.12g]\n",
                R[i * 3 + 0], R[i * 3 + 1], R[i * 3 + 2], T[i]);
    fprintf(f, "  - [0.0, 0.0, 0.0, 1.0]\n");
}

// one camN block: intrinsics + optional extrinsics
static void write_camera(FILE *f, int idx, gs130_device_t *dev,
                         const gs130_camera_intrinsics_t *ci,
                         int res_w, int res_h, int has_imu,
                         gs130_reference_frame_t ref, int is_left)
{
    double R[9], T[3];

    fprintf(f, "cam%d:\n", idx);
    fprintf(f, "  camera_model: pinhole\n");
    fprintf(f, "  distortion_model: %s\n",
            (ci->dist_model == GS130_DIST_FISHEYE) ? "equidistant" : "radtan");
    // Kalibr takes 4 coefficients: radtan=[k1,k2,p1,p2], equidistant=[k1,k2,k3,k4]
    fprintf(f, "  distortion_coeffs: [%.12g, %.12g, %.12g, %.12g]\n",
            ci->dist_coeffs[0], ci->dist_coeffs[1], ci->dist_coeffs[2], ci->dist_coeffs[3]);
    fprintf(f, "  intrinsics: [%.12g, %.12g, %.12g, %.12g]\n", ci->fx, ci->fy, ci->cx, ci->cy);
    fprintf(f, "  resolution: [%d, %d]\n", res_w, res_h);
    fprintf(f, "  rostopic: /cam%d/image_raw\n", idx);

    if(has_imu){
        gs130_get_relative_R(dev, GS130_REF_IMU, ref, R);
        gs130_get_relative_T(dev, GS130_REF_IMU, ref, T);
        write_transform(f, "T_cam_imu", R, T);
    }
    if(is_left){
        // stereo extrinsic: cam1 (left) from cam0 (right)
        gs130_get_relative_R(dev, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, R);
        gs130_get_relative_T(dev, GS130_REF_CAMERA_RIGHT, GS130_REF_CAMERA_LEFT, T);
        write_transform(f, "T_cn_cnm1", R, T);
    }
}

static void write_vec3(FILE *f, const char *key, const double *v, const char *indent)
{
    fprintf(f, "%s%s: [%.12g, %.12g, %.12g]\n", indent, key, v[0], v[1], v[2]);
}

static void write_mat3(FILE *f, const char *key, const double *m, const char *indent)
{
    fprintf(f, "%s%s: [%.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g, %.12g]\n",
            indent, key,
            m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
}

static void write_imu(FILE *f, const gs130_imu_intrinsics_t *imu, int odr)
{
    fprintf(f, "accelerometer_noise_density: %.12g\n", imu->accel_noise);
    fprintf(f, "accelerometer_random_walk:   %.12g\n", imu->accel_random_walk);
    fprintf(f, "gyroscope_noise_density:     %.12g\n", imu->gyro_noise);
    fprintf(f, "gyroscope_random_walk:       %.12g\n", imu->gyro_random_walk);
    fprintf(f, "rostopic: /imu0\n");
    fprintf(f, "update_rate: %d.0\n", odr);

    fprintf(f, "\n# --- gs130 extension: IMU intrinsics (not part of Kalibr's schema) ---\n");
    fprintf(f, "accelerometer:\n");
    write_mat3(f, "misalignment", imu->accel_misalign, "  ");
    write_vec3(f, "scale",        imu->accel_scale,    "  ");
    write_vec3(f, "bias",         imu->accel_bias,     "  ");
    fprintf(f, "gyroscope:\n");
    write_mat3(f, "misalignment", imu->gyro_misalign,  "  ");
    write_vec3(f, "scale",        imu->gyro_scale,     "  ");
    write_vec3(f, "bias",         imu->gyro_bias,      "  ");
}

/* ---- main ---- */

int main(int argc, char **argv)
{
    if(argc < 9){
        fprintf(stderr,
                "usage: gs130-calib-export <output_dir> <platform> <device> <mode> <w> <h> <fps> <odr>\n");
        return 1;
    }

    const char *out_dir  = argv[1];
    const char *platform = argv[2];
    const char *device   = argv[3];
    const char *mode_s   = argv[4];
    int w   = atoi(argv[5]);
    int h   = atoi(argv[6]);
    int fps = atoi(argv[7]);
    int odr = atoi(argv[8]);

    gs130_camera_mode_t mode = GS130_CAMERA_MODE_RAW;
    if(!strcmp(mode_s, "resize"))     mode = GS130_CAMERA_MODE_RESIZE;
    else if(!strcmp(mode_s, "rect"))  mode = GS130_CAMERA_MODE_RECT;

    if(mkdir_p(out_dir) != 0){
        fprintf(stderr, "cannot create output directory %s\n", out_dir);
        return 1;
    }

    // warn (do not change behaviour): only RAW mode exports the EEPROM's original calibration
    if(strcmp(mode_s, "raw") != 0){
        fprintf(stderr,
                "\033[33mwarning: mode is \"%s\", not \"raw\" -- the exported calibration is the "
                "rectified/resized one, not the EEPROM's raw calibration\033[0m\n",
                mode_s);
    }

    gs130_config_t cfg = GS130_CONFIG(platform, device, mode, w, h, fps, odr);
    gs130_device_t *dev = gs130_create();
    if(gs130_init(dev, &cfg) != GS130_OK){
        fprintf(stderr, "init failed\n");
        gs130_destroy(dev);
        return 1;
    }

    gs130_calibration_t cal;
    if(gs130_get_calibration(dev, &cal) != GS130_OK){
        fprintf(stderr, "get_calibration failed\n");
        gs130_deinit(dev);
        gs130_destroy(dev);
        return 1;
    }

    const int has_imu = (gs130_get_imu_name(dev) != NULL);
    char path[512];

    /* camchain.yaml */
    snprintf(path, sizeof(path), "%s/camchain.yaml", out_dir);
    FILE *f = fopen(path, "w");
    if(f == NULL){
        fprintf(stderr, "cannot open %s for writing\n", path);
        gs130_deinit(dev);
        gs130_destroy(dev);
        return 1;
    }
    fprintf(f, "# gs130 EEPROM calibration, Kalibr camchain format\n");
    fprintf(f, "# p_to = T * p_from (T_cam_imu: IMU -> camera, T_cn_cnm1: cam0 -> cam1)\n");
    write_camera(f, 0, dev, &cal.camera_right, w, h, has_imu, GS130_REF_CAMERA_RIGHT, 0);
    write_camera(f, 1, dev, &cal.camera_left,  w, h, has_imu, GS130_REF_CAMERA_LEFT,  1);
    fclose(f);
    printf("wrote %s\n", path);

    /* imu.yaml (only when an IMU is present) */
    if(has_imu){
        snprintf(path, sizeof(path), "%s/imu.yaml", out_dir);
        f = fopen(path, "w");
        if(f == NULL){
            fprintf(stderr, "cannot open %s for writing\n", path);
            gs130_deinit(dev);
            gs130_destroy(dev);
            return 1;
        }
        fprintf(f, "# gs130 EEPROM calibration, IMU parameters\n");
        write_imu(f, &cal.imu, odr);
        fclose(f);
        printf("wrote %s\n", path);
    } else {
        printf("no IMU on this device: imu.yaml not written\n");
    }

    gs130_deinit(dev);
    gs130_destroy(dev);
    return 0;
}
