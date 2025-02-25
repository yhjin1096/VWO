#include "VWO/module/initializer.hpp"

namespace module
{

Initializer::Initializer(const YAML::Node& yaml_node)
    : num_ransac_iters_(yaml_node["num_ransac_iterations"].as<unsigned int>(100)),
      min_num_valid_pts_(yaml_node["min_num_valid_pts"].as<unsigned int>(50)),
      min_num_triangulated_pts_(yaml_node["min_num_triangulated_pts"].as<unsigned int>(50)),
      parallax_deg_thr_(yaml_node["parallax_deg_threshold"].as<float>(1.0)),
      reproj_err_thr_(yaml_node["reprojection_error_threshold"].as<float>(4.0)),
      num_ba_iters_(yaml_node["num_ba_iterations"].as<unsigned int>(20)),
      scaling_factor_(yaml_node["scaling_factor"].as<float>(1.0)),
      use_fixed_seed_(yaml_node["use_fixed_seed"].as<bool>(false))
{

}

Initializer::~Initializer()
{

}

void Initializer::reset() {
    initializer_.reset(nullptr);
    state_ = initializer_state_t::NotReady;
    init_frm_id_ = 0;
    init_frm_stamp_ = 0.0;
}

initializer_state_t Initializer::get_state() const {
    return state_;
}

bool Initializer::initialize(const Camera::setup_type_t setup_type, data::Frame& curr_frm)
{
    switch (setup_type) {
        case Camera::setup_type_t::Monocular: {
            // construct an initializer if not constructed
            if (state_ == initializer_state_t::NotReady) {
                create_initializer(curr_frm);
                return false;
            }

            // try to initialize
            if (!try_initialize_for_monocular(curr_frm)) {
                // failed
                return false;
            }
            else
            {
                std::cout << "try_initialize_for_monocular (OK)" << std::endl;
            }

            // create new map if succeeded
            // create_map_for_monocular(bow_vocab, curr_frm);
            create_map_for_monocular(curr_frm);
            break;
        }
        // case camera::setup_type_t::Stereo:
        // case camera::setup_type_t::RGBD: {
        //     state_ = initializer_state_t::Initializing;

        //     // try to initialize
        //     if (!try_initialize_for_stereo(curr_frm)) {
        //         // failed
        //         return false;
        //     }

        //     // create new map if succeeded
        //     create_map_for_stereo(bow_vocab, curr_frm);
        //     break;
        // }
        default: {
            throw std::runtime_error("Undefined camera setup");
        }
    }

    // check the state is succeeded or not
    if (state_ == initializer_state_t::Succeeded) {
        init_frm_id_ = curr_frm.id_;
        init_frm_stamp_ = curr_frm.timestamp_;
        reset(); // initial 결과만 계속 보기위해 작성 -> stella에서는 pangolin에서 요청 오면 reset
        return true;
    }
    else {
        return false;
    }
}

void Initializer::create_initializer(data::Frame& curr_frm) {
    // set the initial frame
    init_frm_ = data::Frame(curr_frm);

    // initialize the previously matched coordinates
    prev_matched_coords_.resize(init_frm_.frm_obs_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.frm_obs_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.frm_obs_.undist_keypts_.at(i).pt;
    }

    // initialize matchings (init_idx -> curr_idx)
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // build a initializer
    initializer_.reset(nullptr);
    switch (init_frm_.camera_->model_type_) {
        case Camera::model_type_t::Perspective:
        case Camera::model_type_t::Fisheye:
        case Camera::model_type_t::RadialDivision: {
            initializer_ = std::unique_ptr<initialize::perspective>(
                new initialize::perspective(
                    init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
            break;
        }
        // case Camera::model_type_t::Equirectangular: {
        //     initializer_ = std::unique_ptr<initialize::bearing_vector>(
        //         new initialize::bearing_vector(
        //             init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
        //             parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_));
        //     break;
        // }
    }

    state_ = initializer_state_t::Initializing;
}

bool Initializer::try_initialize_for_monocular(data::Frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    match::area matcher(0.9, true);
    const auto num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);
    
    if (num_matches < min_num_valid_pts_) {
        // rebuild the initializer with the next frame
        reset();
        return false;
    }

    // visualize matches
    if(1)
    {
        cv::Mat prev_img = init_frm_.image_.clone();
        cv::Mat curr_img = curr_frm.image_.clone();
        
        if (!prev_img.empty() && !curr_img.empty())
        {   
            cv::Mat vis;
            cv::hconcat(prev_img, curr_img, vis);

            for(int i = 0; i < init_matches_.size(); i++)
            {
                int curr_point_idx = init_matches_[i];
                if(curr_point_idx != -1)
                {
                    cv::Point2f pt_prev = init_frm_.frm_obs_.undist_keypts_.at(i).pt;
                    cv::Point2f pt_curr = curr_frm.frm_obs_.undist_keypts_.at(curr_point_idx).pt;
                    pt_curr.x += prev_img.size().width;

                    cv::circle(vis, pt_prev, 4, cv::Scalar(0, 255, 0), -1);
                    cv::circle(vis, pt_curr, 4, cv::Scalar(0, 255, 0), -1);
                    cv::line(vis, pt_prev, pt_curr, cv::Scalar(255, 0, 0), 1);
                }
            }
            cv::resize(vis, vis, vis.size()/2);
            cv::imshow("keypoint matches", vis);
            cv::waitKey(1);
        }
    }

    // try to initialize with the initial frame and the current frame
    assert(initializer_);
    
    // pose 계산, 3d point 계산
    // curr_frm, init_frm 사이의 relative wheel odom 추가
    Mat44_t relative_cam_tf = curr_frm.curr_cam_tf_.inverse() * init_frm_.curr_cam_tf_;
    // std::cout << relative_cam_tf << std::endl;
    return initializer_->initialize_with_odom(curr_frm, init_matches_, relative_cam_tf);
}

bool Initializer::create_map_for_monocular(data::Frame& curr_frm)
{
    eigen_alloc_vector<Vec3_t> init_triangulated_pts;
    {
        assert(initializer_);
        init_triangulated_pts = initializer_->get_triangulated_pts();
        const auto is_triangulated = initializer_->get_triangulated_flags();

        // make invalid the matchings which have not been triangulated
        for (unsigned int i = 0; i < init_matches_.size(); ++i) {
            if (init_matches_.at(i) < 0) {
                continue;
            }
            if (is_triangulated.at(i)) {
                continue;
            }
            init_matches_.at(i) = -1;
        }

        // set the camera poses
        init_frm_.set_pose_cw(Mat44_t::Identity());
        Mat44_t cam_pose_cw = Mat44_t::Identity();
        cam_pose_cw.block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        
        curr_frm.set_pose_cw(cam_pose_cw);

        // destruct the initializer
        initializer_.reset(nullptr);
    }

    state_ = initializer_state_t::Succeeded;
    return true;
}

} // namespace module