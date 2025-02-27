#ifndef VWO_MODULE_INITIALIZER_H
#define VWO_MODULE_INITIALIZER_H

#include "VWO/data/frame.h"
#include "VWO/data/map_database.h"

#include "VWO/initialize/base.h"
#include "VWO/initialize/perspective.h"

#include "VWO/match/area.h"

#include "VWO/data/bow_vocabulary_fwd.h"

#include "VWO/optimize/global_bundle_adjuster.h"

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d/features2d.hpp>

class config;

namespace data {
class Frame;
class map_database;
// class bow_database;
} // namespace data

namespace module {

// initializer state
enum class initializer_state_t {
    NotReady,
    Initializing,
    Wrong,
    Succeeded
};

class Initializer {
public:
    Initializer() = delete;

    //! Constructor
    // Initializer(data::map_database* map_db, data::bow_database* bow_db,
    //             const YAML::Node& yaml_node);

    Initializer(data::map_database* map_db, const YAML::Node& yaml_node);

    //! Destructor
    ~Initializer();

    //! Reset initializer
    void reset();

    //! Get initialization state
    initializer_state_t get_state() const;

    //! Get the initial frame ID which succeeded in initialization
    unsigned int get_initial_frame_id() const;

    //! Get the initial frame stamp which succeeded in initialization
    double get_initial_frame_timestamp() const;

    //! Get whether to use a fixed seed for RANSAC
    bool get_use_fixed_seed() const;

    //! Initialize with the current frame
    bool initialize(const Camera::setup_type_t setup_type,
                    data::bow_vocabulary* bow_vocab, data::Frame& curr_frm);
    // bool initialize(const Camera::setup_type_t setup_type, data::Frame& curr_frm);

    //! initial frame
    data::Frame init_frm_;
private:
    // //! map database
    data::map_database* map_db_ = nullptr;
    // //! BoW database
    // data::bow_database* bow_db_ = nullptr;
    //! initializer status
    initializer_state_t state_ = initializer_state_t::NotReady;

    //! ID of frame used for initialization (will be set after succeeded)
    unsigned int init_frm_id_ = 0;
    //! timestamp of frame used for initialization (will be set after succeeded)
    double init_frm_stamp_ = 0.0;

    //-----------------------------------------
    // parameters

    //! max number of iterations of RANSAC (only for monocular initializer)
    const unsigned int num_ransac_iters_;
    //! min number of valid pts (It should be greater than or equal to min_num_triangulated_)
    const unsigned int min_num_valid_pts_;
    //! min number of triangulated pts
    const unsigned int min_num_triangulated_pts_;
    //! min parallax (only for monocular initializer)
    const float parallax_deg_thr_;
    //! reprojection error threshold (only for monocular initializer)
    const float reproj_err_thr_;
    //! max number of iterations of BA (only for monocular initializer)
    const unsigned int num_ba_iters_;
    //! initial scaling factor (only for monocular initializer)
    const float scaling_factor_;
    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;

    //-----------------------------------------
    // for monocular camera model

    //! Create initializer for monocular
    void create_initializer(data::Frame& curr_frm);

    //! Try to initialize a map with monocular camera setup
    bool try_initialize_for_monocular(data::Frame& curr_frm);

    //! Create an initial map with monocular camera setup
    bool create_map_for_monocular(data::bow_vocabulary* bow_vocab, data::Frame& curr_frm);
    // bool create_map_for_monocular(data::Frame& curr_frm);

    //! Scaling up or down a initial map
    // void scale_map(const std::shared_ptr<data::keyframe>& init_keyfrm, const std::shared_ptr<data::keyframe>& curr_keyfrm, const double scale);

    //! initializer for monocular
    std::unique_ptr<initialize::base> initializer_ = nullptr;
    
    //! coordinates of previously matched points to perform area-based matching
    std::vector<cv::Point2f> prev_matched_coords_;
    //! initial matching indices (index: idx of initial frame, value: idx of current frame)
    std::vector<int> init_matches_;

    //-----------------------------------------
    // for stereo or RGBD camera model

    //! Try to initialize a map with stereo or RGBD camera setup
    // bool try_initialize_for_stereo(data::Frame& curr_frm);

    //! Create an initial map with stereo or RGBD camera setup
    // bool create_map_for_stereo(data::bow_vocabulary* bow_vocab, data::Frame& curr_frm);
};

} // namespace module

#endif // STELLA_VSLAM_MODULE_INITIALIZER_H
