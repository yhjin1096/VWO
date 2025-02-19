#ifndef TRACKING_MODULE_HPP
#define TRACKING_MODULE_HPP

#include <iostream>

#include <opencv2/opencv.hpp>

#include "VWO/config.hpp"

#include "VWO/camera/camera.hpp"

#include "VWO/data/frame.hpp"

#include "VWO/match/area.hpp"

#include "VWO/initialize/base.h"
#include "VWO/initialize/perspective.h"

class TrackingModule
{
    public:
        TrackingModule() = default;
        TrackingModule(const std::shared_ptr<Config>& cfg, Camera* camera);
        ~TrackingModule();

        std::shared_ptr<Mat44_t> trackFrame(data::Frame curr_frm, data::Frame prev_frm);
    private:
        Camera* camera_ = nullptr;

        std::unique_ptr<initialize::base> initializer_ = nullptr;
        //! max number of iterations of RANSAC
        const unsigned int num_ransac_iters_;
        //! min number of triangulated pts
        const unsigned int min_num_triangulated_pts_;
        //! min number of valid pts
        const unsigned int min_num_valid_pts_;
        //! min parallax
        const float parallax_deg_thr_;
        //! reprojection error threshold
        const float reproj_err_thr_;
        //! Use fixed random seed for RANSAC if true
        const bool use_fixed_seed_;
};

#endif