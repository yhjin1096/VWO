#ifndef TRACKING_MODULE_HPP
#define TRACKING_MODULE_HPP

#include <iostream>

#include <opencv2/opencv.hpp>

#include "VWO/config.hpp"

#include "VWO/camera/camera.hpp"

#include "VWO/data/frame.hpp"

#include "VWO/match/area.hpp"

class TrackingModule
{
    public:
        TrackingModule() = default;
        TrackingModule(const std::shared_ptr<Config>& cfg, Camera* camera);
        ~TrackingModule();

        std::shared_ptr<Mat44_t> trackFrame(data::Frame curr_frm, data::Frame prev_frm);
    private:
        Camera* camera_ = nullptr;
};

#endif