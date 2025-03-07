#include "VWO/system.h"

System::System(const std::shared_ptr<Config>& cfg, const std::string& vocab_file_path)
    : config_(cfg)
{
    // camera
    YAML::Node camera_params = config_->yaml_node_["Camera"];
    if(camera_params["model"].as<std::string>() != "perspective")
    {
        std::cout << "invalid camera model: " << camera_params["model"].as<std::string>() << std::endl;
        exit(0);
    }
    camera_ = new Perspective(camera_params);

    // feature - orb
    orb_params_ = new orb_params(config_->yaml_node_["Feature"]);
    YAML::Node preprocessing_params = config_->yaml_node_["Feature"];
    const unsigned int min_size = preprocessing_params["min_size"].as<unsigned int>(800);
    std::vector<std::vector<float>> mask_rectangles = util::get_rectangles(preprocessing_params["mask_rectangles"]);

    if(orb_params_->name_ == "ORB")
    {
        feature_extractor_ = new ORBExtractor(orb_params_, min_size, mask_rectangles);
    }

    // ORB vocabulary
    map_db_ = new data::map_database(config_->yaml_node_["System"]["min_num_shared_lms"].as<unsigned int>(15));
    bow_vocab_ = data::bow_vocabulary_util::load(vocab_file_path);
    
    // tracker
    tracker_ = new TrackingModule(cfg, camera_, map_db_, bow_vocab_);
}

System::~System()
{
    delete orb_params_;
    orb_params_ = nullptr;
    
    delete camera_;
    camera_ = nullptr;

    delete feature_extractor_;
    feature_extractor_ = nullptr;

    delete map_db_;
    map_db_ = nullptr;

    delete bow_vocab_;
    bow_vocab_ = nullptr;

    delete tracker_;
    tracker_ = nullptr;
}

data::map_database* System::getMapDatabase()
{
    return map_db_;
}

std::shared_ptr<Mat44_t> System::trackFrame(const cv::Mat &image, const double timestamp)
{
    const auto start = std::chrono::system_clock::now();
    data::Frame curr_frm = createFrame(image, timestamp);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // return std::make_shared<Mat44_t>(Mat44_t::Identity());
    return feed_frame(curr_frm, image, extraction_time_elapsed_ms);
}

data::Frame System::createFrame(const cv::Mat &image, const double timestamp)
{
     // color conversion
    if (!camera_->is_valid_shape(image)) {
        std::cout << "preprocess: Input image size is invalid" << std::endl;
    }
    cv::Mat image_gray = image;
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    
    data::frame_observation frm_obs;

    // Extract ORB feature
    keypts_.clear();
    feature_extractor_->extractFeatures(image_gray, mask_, keypts_, frm_obs.descriptors_);
    if (keypts_.empty()) {
        std::cout << "preprocess: cannot extract any keypoints" << std::endl;
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);
    
    return data::Frame(next_frame_id_++, timestamp, camera_, orb_params_, frm_obs, image);
}


std::shared_ptr<Mat44_t> System::feed_frame(const data::Frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms)
{
    const auto start = std::chrono::system_clock::now();
    
    const std::shared_ptr<Mat44_t> cam_pose_wc = tracker_->feed_frame(frm);

    const auto end = std::chrono::system_clock::now();
    double tracking_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    //map publisher, frame_publisher -> pangolin viewer

    return cam_pose_wc;
}

////////////////////////////////////////////////////////////////////////

std::shared_ptr<Mat44_t> System::trackFrameWithOdom(const cv::Mat &image, const double timestamp, const Mat44_t& curr_cam_tf)
{
    const auto start = std::chrono::system_clock::now();
    data::Frame curr_frm = createFrameWithOdom(image, timestamp, curr_cam_tf);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    // return std::make_shared<Mat44_t>(Mat44_t::Identity());
    return feedFrameWithOdom(curr_frm, image, extraction_time_elapsed_ms);
}

data::Frame System::createFrameWithOdom(const cv::Mat &image, const double timestamp, const Mat44_t& curr_cam_tf)
{
     // color conversion
    if (!camera_->is_valid_shape(image)) {
        std::cout << "preprocess: Input image size is invalid" << std::endl;
    }
    cv::Mat image_gray = image;
    cv::cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);
    
    data::frame_observation frm_obs;

    // Extract ORB feature
    keypts_.clear();
    feature_extractor_->extractFeatures(image_gray, mask_, keypts_, frm_obs.descriptors_);
    if (keypts_.empty()) {
        std::cout << "preprocess: cannot extract any keypoints" << std::endl;
    }

    // Undistort keypoints
    camera_->undistort_keypoints(keypts_, frm_obs.undist_keypts_);

    // Convert to bearing vector
    camera_->convert_keypoints_to_bearings(frm_obs.undist_keypts_, frm_obs.bearings_);

    // Assign all the keypoints into grid
    frm_obs.num_grid_cols_ = num_grid_cols_;
    frm_obs.num_grid_rows_ = num_grid_rows_;
    data::assign_keypoints_to_grid(camera_, frm_obs.undist_keypts_, frm_obs.keypt_indices_in_cells_,
                                   frm_obs.num_grid_cols_, frm_obs.num_grid_rows_);
    
    return data::Frame(next_frame_id_++, timestamp, camera_, orb_params_, frm_obs, image, curr_cam_tf);
}

std::shared_ptr<Mat44_t> System::feedFrameWithOdom(const data::Frame& frm, const cv::Mat& img, const double extraction_time_elapsed_ms)
{
    const auto start = std::chrono::system_clock::now();
    
    const std::shared_ptr<Mat44_t> cam_pose_wc = tracker_->feed_frame(frm);

    const auto end = std::chrono::system_clock::now();
    double tracking_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    //map publisher, frame_publisher -> pangolin viewer

    return cam_pose_wc;
}

std::shared_ptr<Mat44_t> System::trackTwoFrame(const cv::Mat& prev_image, const cv::Mat& curr_image,
                            const double prev_timestamp, const double curr_timestamp)
{
    data::Frame prev_frm = createFrame(prev_image, prev_timestamp);
    data::Frame curr_frm = createFrame(curr_image, curr_timestamp);
    std::shared_ptr<Mat44_t> pose_cp = tracker_->trackFrame(curr_frm, prev_frm);

    // prev_frm.set_pose_cw(Mat44_t::Identity());
    // curr_frm.set_pose_cw(*pose_cp * prev_frm.get_pose_cw());

    return pose_cp;
}

