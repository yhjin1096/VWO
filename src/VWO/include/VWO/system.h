#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "VWO/config.h"

#include "VWO/camera/camera.h"
#include "VWO/camera/perspective.h"

#include "VWO/feature/feature_extractor.h"
#include "VWO/feature/orb_extractor.h"
#include "VWO/feature/orb_params.h"

#include "VWO/tracking/tracking_module.h"

#include "VWO/data/common.h"
#include "VWO/data/frame.h"
#include "VWO/data/frame_observation.h"
#include "VWO/data/bow_vocabulary_fwd.h"

#include "VWO/util/yaml.h"

class System
{
    public:
        System(const std::shared_ptr<Config>& cfg, const std::string& vocab_file_path);
        ~System();
        Camera* camera_;
        FeatureExtractor* feature_extractor_;

        TrackingModule* tracker_;
        std::shared_ptr<Mat44_t> trackFrame(const cv::Mat &image, const double timestamp);
        std::shared_ptr<Mat44_t> trackFrameWithOdom(const cv::Mat &image, const double timestamp, const Mat44_t& curr_cam_tf);

        cv::Mat mask_;

        std::shared_ptr<Mat44_t> trackTwoFrame(const cv::Mat& prev_image, const cv::Mat& curr_image,
                            const double prev_timestamp, const double curr_timestamp);

        data::map_database* getMapDatabase();

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
        
        data::Frame createFrameWithOdom(const cv::Mat &image, const double timestamp, const Mat44_t& curr_cam_tf);
        std::shared_ptr<Mat44_t> feedFrameWithOdom(const data::Frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms);

        //! map database
        data::map_database* map_db_ = nullptr;

        //! BoW vocabulary
        data::bow_vocabulary* bow_vocab_ = nullptr;
};

#endif