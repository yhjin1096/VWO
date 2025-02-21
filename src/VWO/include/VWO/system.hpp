#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "VWO/config.hpp"

#include "VWO/camera/camera.hpp"
#include "VWO/camera/perspective.hpp"

#include "VWO/feature/feature_extractor.hpp"
#include "VWO/feature/orb_extractor.hpp"
#include "VWO/feature/orb_params.hpp"

#include "VWO/tracking/tracking_module.hpp"

#include "VWO/data/common.hpp"
#include "VWO/data/frame.hpp"
#include "VWO/data/frame_observation.hpp"

#include "VWO/util/yaml.hpp"

class System
{
    public:
        System(const std::shared_ptr<Config>& cfg);
        ~System();
        Camera* camera_;
        FeatureExtractor* feature_extractor_;

        TrackingModule* tracker_;
        std::shared_ptr<Mat44_t> trackFrame(const cv::Mat &image, const double timestamp);

        cv::Mat mask_;

        std::shared_ptr<Mat44_t> trackTwoFrame(const cv::Mat& prev_image, const cv::Mat& curr_image,
                            const double prev_timestamp, const double curr_timestamp);

    private:
        std::shared_ptr<Config> config_;

        orb_params* orb_params_ = nullptr;
        //! number of columns of grid to accelerate reprojection matching
        unsigned int num_grid_cols_ = 64;
        //! number of rows of grid to accelerate reprojection matching
        unsigned int num_grid_rows_ = 48;
        std::vector<cv::KeyPoint> keypts_;

        std::atomic<unsigned int> next_frame_id_{0};
        data::Frame createFrame(const cv::Mat &image, const double timestamp);
        std::shared_ptr<Mat44_t> feed_frame(const data::Frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms);
};

#endif