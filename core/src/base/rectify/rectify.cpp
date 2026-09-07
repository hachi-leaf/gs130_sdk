/**
 * Copyright (c) 2026 D-Robotics.
 * SPDX-License-Identifier: MIT
 */
#include "base/rectify/rectify.hpp"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace gs130 {
namespace base {
namespace {

constexpr int    kProbeWidth   = 128;    // proxy width for the search phase; correctness is resolution-independent
constexpr int    kGridStep     = 32;     // Fisheye frame-expansion step
constexpr double kCoarseBegin  = 0.10;
constexpr double kCoarseEnd    = 2.00;
constexpr double kCoarseStep   = 0.05;
constexpr double kFineStep     = 0.005;

int dist_count(DistModel m)
{
    return (m == DistModel::Fisheye) ? 4 : 8;
}

// Check that the sampled coordinates along all four mapped borders fall inside the source image.
// Upper bound uses src-1: bilinear interpolation needs the +1 neighbor pixel.
// Checking the borders alone suffices: with center-symmetric monotone distortion the sample
// range is enclosed by the borders, so interior points never go out of bounds first.
bool border_in_source(const cv::Mat &mx, const cv::Mat &my, int src_w, int src_h)
{
    const int   w = mx.cols, h = mx.rows;
    const float x_max = static_cast<float>(src_w - 1);
    const float y_max = static_cast<float>(src_h - 1);

    auto bad = [&](float x, float y){
        return x < 0.0F || x >= x_max || y < 0.0F || y >= y_max;
    };

    for(int x = 0; x < w; ++x){
        if(bad(mx.at<float>(0, x), my.at<float>(0, x)))
            return false;
        if(bad(mx.at<float>(h - 1, x), my.at<float>(h - 1, x)))
            return false;
    }
    for(int y = 0; y < h; ++y){
        if(bad(mx.at<float>(y, 0), my.at<float>(y, 0)))
            return false;
        if(bad(mx.at<float>(y, w - 1), my.at<float>(y, w - 1)))
            return false;
    }
    return true;
}

// Stereo rectification + principal-point centering. Fisheye uses fov_scale; pinhole uses alpha=0 followed by manual focal scaling.
//
// new_image_size must be passed separately from out_size: probing always uses src_size and only the
// final generation uses out_size. Otherwise, when expanding the frame, the focal length would scale
// with the size, angular coverage would stay unchanged, the black-border check would always pass,
// and the expansion loop would never terminate.
void stereo_rectify(DistModel model,
                    const cv::Mat &lK, const cv::Mat &lD,
                    const cv::Mat &rK, const cv::Mat &rD,
                    const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                    cv::Size src_size, cv::Size out_size,
                    cv::Size new_image_size, double fov_scale,
                    cv::Mat &rect_lR, cv::Mat &rect_rR,
                    cv::Mat &proj_lP, cv::Mat &proj_rP)
{
    cv::Mat Q;
    if(model == DistModel::Fisheye){
        cv::fisheye::stereoRectify(lK, lD, rK, rD, src_size, R_r2l, t_r2l,
                                   rect_lR, rect_rR, proj_lP, proj_rP, Q,
                                   cv::fisheye::CALIB_ZERO_DISPARITY,
                                   new_image_size, 0.0, fov_scale);
    } else{
        cv::stereoRectify(lK, lD, rK, rD, src_size, R_r2l, t_r2l,
                          rect_lR, rect_rR, proj_lP, proj_rP, Q,
                          cv::CALIB_ZERO_DISPARITY, 0.0, new_image_size);
        // pinhole has no fov_scale parameter; equivalently scale the focal length in P
        proj_lP.at<double>(0, 0) /= fov_scale;
        proj_lP.at<double>(1, 1) /= fov_scale;
        proj_rP.at<double>(0, 0) /= fov_scale;
        proj_rP.at<double>(1, 1) /= fov_scale;
    }
    proj_lP.at<double>(0, 2) = out_size.width * 0.5;
    proj_lP.at<double>(1, 2) = out_size.height * 0.5;
    proj_rP.at<double>(0, 2) = out_size.width * 0.5;
    proj_rP.at<double>(1, 2) = out_size.height * 0.5;
}

void undistort_rectify_map(DistModel model,
                           const cv::Mat &K, const cv::Mat &D,
                           const cv::Mat &rect_R, const cv::Mat &P,
                           cv::Size size, cv::Mat &mx, cv::Mat &my)
{
    if(model == DistModel::Fisheye)
        cv::fisheye::initUndistortRectifyMap(K, D, rect_R, P, size, CV_32FC1, mx, my);
    else
        cv::initUndistortRectifyMap(K, D, rect_R, P, size, CV_32FC1, mx, my);
}

// Single fov_scale attempt: rectify -> proxy map -> black-border check
bool try_fov_scale(DistModel model,
                   const cv::Mat &lK, const cv::Mat &lD,
                   const cv::Mat &rK, const cv::Mat &rD,
                   const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                   cv::Size src_size, cv::Size out_size, cv::Size probe_size,
                   double fov_scale)
{
    cv::Mat rect_lR, rect_rR, proj_lP, proj_rP;
    stereo_rectify(model, lK, lD, rK, rD, R_r2l, t_r2l,
                   src_size, out_size, src_size, fov_scale,
                   rect_lR, rect_rR, proj_lP, proj_rP);

    // Scale P proportionally with the proxy size
    const double sx = static_cast<double>(probe_size.width) / out_size.width;
    const double sy = static_cast<double>(probe_size.height) / out_size.height;
    cv::Mat Pl = proj_lP.clone(), Pr = proj_rP.clone();
    for(cv::Mat *P : {&Pl, &Pr}){
        P->at<double>(0, 0) *= sx;
        P->at<double>(0, 2) *= sx;
        P->at<double>(1, 1) *= sy;
        P->at<double>(1, 2) *= sy;
    }

    cv::Mat lmx, lmy, rmx, rmy;
    undistort_rectify_map(model, lK, lD, rect_lR, Pl, probe_size, lmx, lmy);
    undistort_rectify_map(model, rK, rD, rect_rR, Pr, probe_size, rmx, rmy);

    return border_in_source(lmx, lmy, src_size.width, src_size.height) &&
           border_in_source(rmx, rmy, src_size.width, src_size.height);
}

cv::Size probe_of(cv::Size out_size)
{
    int h = static_cast<int>(kProbeWidth * static_cast<double>(out_size.height) /
                                 out_size.width + 0.5);
    if(h < 1)
        h = 1;
    return cv::Size(kProbeWidth, h);
}

// Find the largest black-border-free fov_scale: coarse search first, then fine search in its neighborhood
double find_best_fov_scale(DistModel model,
                           const cv::Mat &lK, const cv::Mat &lD,
                           const cv::Mat &rK, const cv::Mat &rD,
                           const cv::Mat &R_r2l, const cv::Mat &t_r2l,
                           cv::Size src_size, cv::Size out_size)
{
    const cv::Size probe = probe_of(out_size);

    double coarse = kCoarseBegin;
    for(double s = kCoarseBegin; s <= kCoarseEnd + 1e-4; s += kCoarseStep){
        if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                           src_size, out_size, probe, s))
            break;
        coarse = s;
    }

    double best = coarse;
    const double hi = coarse + kCoarseStep;
    for(double s = coarse + kFineStep; s <= hi + 1e-9; s += kFineStep){
        if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                           src_size, out_size, probe, s))
            break;
        best = s;
    }
    return best;
}

// Right-camera pose in the left-camera frame: R_r2l = lR*rR^T, t_r2l = lT - R_r2l*rT
void relative_pose(const double *lR, const double *lT,
                   const double *rR, const double *rT,
                   cv::Mat &R_r2l, cv::Mat &t_r2l)
{
    const cv::Mat lRm(3, 3, CV_64F, const_cast<double *>(lR));
    const cv::Mat rRm(3, 3, CV_64F, const_cast<double *>(rR));
    const cv::Mat lTm(3, 1, CV_64F, const_cast<double *>(lT));
    const cv::Mat rTm(3, 1, CV_64F, const_cast<double *>(rT));
    R_r2l = lRm * rRm.t();
    t_r2l = lTm - R_r2l * rTm;
}

cv::Mat k_mat(const CameraIntrinsics &c)
{
    return (cv::Mat_<double>(3, 3) << c.fx, 0.0, c.cx,
                                      0.0, c.fy, c.cy,
                                      0.0, 0.0, 1.0);
}

cv::Mat d_mat(const CameraIntrinsics &c, DistModel m)
{
    return cv::Mat(dist_count(m), 1, CV_64F,
                   const_cast<double *>(c.dist_coeffs)).clone();
}

// Unify the left/right focal lengths to min(fx,fy) and move the principal point to the output center
void align_focal_and_center(cv::Mat &proj_lP, cv::Mat &proj_rP, int w, int h)
{
    const double f = std::min(proj_lP.at<double>(0, 0), proj_lP.at<double>(1, 1));
    for(cv::Mat *P : {&proj_lP, &proj_rP}){
        P->at<double>(0, 0) = f;
        P->at<double>(1, 1) = f;
        P->at<double>(0, 2) = w * 0.5;
        P->at<double>(1, 2) = h * 0.5;
    }
}

void fill_map(std::vector<RemapPoint> *dst,
              const cv::Mat &mx, const cv::Mat &my, int w, int h)
{
    dst->resize(static_cast<size_t>(w) * static_cast<size_t>(h));
    for(int y = 0; y < h; ++y){
        for(int x = 0; x < w; ++x){
            RemapPoint &p = (*dst)[static_cast<size_t>(y) * w + x];
            p.x = mx.at<float>(y, x);
            p.y = my.at<float>(y, x);
        }
    }
}

// Virtual calibration write-back: zero distortion, shared focal, centered principal point, R = original R * rect_R^T, T kept unchanged.
// orig_R and the target R may alias the same memory, so clone the original R before writing back.
void write_virtual(CameraIntrinsics *k, double *R,
                   const double *orig_R,
                   const cv::Mat &P, const cv::Mat &rect_R)
{
    k->fx = P.at<double>(0, 0);
    k->fy = P.at<double>(1, 1);
    k->cx = P.at<double>(0, 2);
    k->cy = P.at<double>(1, 2);
    memset(k->dist_coeffs, 0, sizeof(k->dist_coeffs));
    k->K[0] = k->fx; k->K[2] = k->cx;
    k->K[4] = k->fy; k->K[5] = k->cy;

    const cv::Mat oR = cv::Mat(3, 3, CV_64F,
                               const_cast<double *>(orig_R)).clone();
    const cv::Mat virtR = oR * rect_R.t();
    memcpy(R, virtR.ptr<double>(), 9 * sizeof(double));
}

} // namespace

Status stereo_rectify(StereoImuModel *cal,
                      uint32_t src_w, uint32_t src_h,
                      uint32_t *grid_w, uint32_t *grid_h,
                      std::vector<RemapPoint> *left_map,
                      std::vector<RemapPoint> *right_map)
{
    if(cal == nullptr || grid_w == nullptr || grid_h == nullptr ||
        left_map == nullptr || right_map == nullptr)
        return Status::ParamError;
    if(src_w < 32 || src_h < 32)
        return Status::ParamError;
    if(cal->cam_left.fx <= 0.0 || cal->cam_left.fy <= 0.0 ||
        cal->cam_right.fx <= 0.0 || cal->cam_right.fy <= 0.0)
        return Status::ParamError;

    const DistModel model = cal->cam_left.dist_model;
    if(cal->cam_right.dist_model != model)
        return Status::ParamError;   // left/right distortion models differ

    const cv::Mat lK = k_mat(cal->cam_left),  lD = d_mat(cal->cam_left, model);
    const cv::Mat rK = k_mat(cal->cam_right), rD = d_mat(cal->cam_right, model);

    cv::Mat R_r2l, t_r2l;
    relative_pose(cal->cam_left_R, cal->cam_left_T,
                  cal->cam_right_R, cal->cam_right_T, R_r2l, t_r2l);

    const cv::Size src_size(static_cast<int>(src_w), static_cast<int>(src_h));
    cv::Size out_size(static_cast<int>(src_w), static_cast<int>(src_h));

    // Pinhole alpha=0 already applies zoom+shift for maximum framing; no search or frame expansion needed
    double fov_scale = 1.0;
    if(model == DistModel::Fisheye){
        fov_scale = find_best_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, out_size);

        // After touching the border, expand in 32-pixel steps to the maximum frame: grow only in the direction that stays black-border-free
        const cv::Size grow_w(out_size.width + kGridStep, out_size.height);
        const cv::Size grow_h(out_size.width, out_size.height + kGridStep);
        const bool w_ok = try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, grow_w, probe_of(grow_w), fov_scale);
        const bool h_ok = try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                        src_size, grow_h, probe_of(grow_h), fov_scale);

        if(w_ok != h_ok){
            const bool grow_width = w_ok;
            for(;;){
                cv::Size next = out_size;
                if(grow_width)
                    next.width += kGridStep;
                else
                    next.height += kGridStep;
                if(!try_fov_scale(model, lK, lD, rK, rD, R_r2l, t_r2l,
                                   src_size, next, probe_of(next), fov_scale))
                    break;
                out_size = next;
            }
        }
    }

    cv::Mat rect_lR, rect_rR, proj_lP, proj_rP;
    stereo_rectify(model, lK, lD, rK, rD, R_r2l, t_r2l,
                   src_size, out_size, out_size, fov_scale,
                   rect_lR, rect_rR, proj_lP, proj_rP);

    align_focal_and_center(proj_lP, proj_rP, out_size.width, out_size.height);

    cv::Mat lmx, lmy, rmx, rmy;
    undistort_rectify_map(model, lK, lD, rect_lR, proj_lP, out_size, lmx, lmy);
    undistort_rectify_map(model, rK, rD, rect_rR, proj_rP, out_size, rmx, rmy);

    *grid_w = static_cast<uint32_t>(out_size.width);
    *grid_h = static_cast<uint32_t>(out_size.height);
    fill_map(left_map,  lmx, lmy, out_size.width, out_size.height);
    fill_map(right_map, rmx, rmy, out_size.width, out_size.height);

    // In-place write-back of virtual intrinsics/extrinsics (zero distortion, shared focal, centered principal point; T unchanged)
    write_virtual(&cal->cam_left,  cal->cam_left_R,  cal->cam_left_R,  proj_lP, rect_lR);
    write_virtual(&cal->cam_right, cal->cam_right_R, cal->cam_right_R, proj_rP, rect_rR);
    return Status::Ok;
}

} // namespace base
} // namespace gs130
