#ifndef TRACKING_MODULE_HPP
#define TRACKING_MODULE_HPP

#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "VWO/config.h"

#include "VWO/camera/camera.h"
#include "VWO/camera/perspective.h"

#include "VWO/data/frame.h"
#include "VWO/data/bow_vocabulary.h"
#include "VWO/data/bow_vocabulary_fwd.h"

#include "VWO/match/area.h"

#include "VWO/initialize/base.h"
#include "VWO/initialize/perspective.h"
///////
#include "VWO/module/initializer.h"

enum class tracker_state_t {
    Initializing,
    Tracking,
    Lost
};

class TrackingModule
{
    public:
        TrackingModule() = default;
        TrackingModule(const std::shared_ptr<Config>& cfg, Camera* camera, data::map_database* map_db,
                    data::bow_vocabulary* bow_vocab);
        ~TrackingModule();

        std::shared_ptr<Mat44_t> feed_frame(data::Frame curr_frm);

        data::Frame curr_frm_;

        tracker_state_t tracking_state_ = tracker_state_t::Initializing;

        //! Reset the databases
        void reset();


        std::shared_ptr<Mat44_t> trackFrame(data::Frame curr_frm, data::Frame prev_frm);

        double init_retry_threshold_time_ = 5.0;

    protected:
        bool initialize();
        
        Mat44_t last_cam_pose_from_ref_keyfrm_;
        data::Frame last_frm_;
        
        //! mutex for stop_keyframe_insertion process
        mutable std::mutex mtx_stop_keyframe_insertion_;
        
        //! mutex for pause process
        mutable std::mutex mtx_last_frm_;

        //! ID of latest frame which succeeded in relocalization
        unsigned int last_reloc_frm_id_ = 0;
        //! timestamp of latest frame which succeeded in relocalization
        double last_reloc_frm_timestamp_ = 0.0;

        //! map_database
        data::map_database* map_db_ = nullptr;

        // Bag of Words
        //! BoW vocabulary
        data::bow_vocabulary* bow_vocab_ = nullptr;

    private:
        Camera* camera_ = nullptr;
        
        module::Initializer initializer_;

        double init_frm_stamp_ = 0.0;



        // module::Initializer 에서 구현 예정이므로 삭제 해야됨
        // std::unique_ptr<initialize::base> initializer_ = nullptr;
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
        data::Frame init_frm_;
};

#endif