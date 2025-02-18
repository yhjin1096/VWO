#include "VWO/system.hpp"

System::System(const std::shared_ptr<Config>& cfg)
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
    
    //tracker
    tracker_ = new TrackingModule(cfg, camera_);
}

System::~System()
{
    delete orb_params_;
    orb_params_ = nullptr;
    
    delete camera_;
    camera_ = nullptr;

    delete feature_extractor_;
    feature_extractor_ = nullptr;
}

std::shared_ptr<Mat44_t> System::trackFrame(const cv::Mat &image, const double timestamp)
{
    const auto start = std::chrono::system_clock::now();
    data::Frame curr_frm = createFrame(image, timestamp);
    const auto end = std::chrono::system_clock::now();
    double extraction_time_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::shared_ptr<Mat44_t> pose_wc;
    if(exist_prev_)
        pose_wc = tracker_->trackFrame(curr_frm, prev_frm_);
    else
    {
        pose_wc = std::make_shared<Mat44_t>(Mat44_t::Identity());
    }
    prev_frm_ = curr_frm;
    exist_prev_ = true;

    return pose_wc;
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