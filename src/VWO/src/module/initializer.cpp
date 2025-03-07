#include "VWO/module/initializer.h"

namespace module
{

Initializer::Initializer(data::map_database* map_db, 
                        const YAML::Node& yaml_node)
    : map_db_(map_db), //bow_db
      num_ransac_iters_(yaml_node["num_ransac_iterations"].as<unsigned int>(100)),
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

double Initializer::get_initial_frame_timestamp() const {
    return init_frm_stamp_;
}

bool Initializer::initialize(const Camera::setup_type_t setup_type, data::bow_vocabulary* bow_vocab, data::Frame& curr_frm)
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

            // create new map if succeeded
            create_map_for_monocular(bow_vocab, curr_frm);
            
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
    // curr_frm 기준 init_frm의 위치(wheel odom)
    Mat44_t relative_cam_tf = curr_frm.curr_cam_tf_.inverse() * init_frm_.curr_cam_tf_;
    
    return initializer_->initialize_with_odom(curr_frm, init_matches_, relative_cam_tf);
}

bool Initializer::create_map_for_monocular(data::bow_vocabulary* bow_vocab, data::Frame& curr_frm)
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
        // SLAM이 시작된 위치의 camera 위치로 초기화(tf로 계산됨, odom -> camera_topRGB_link)
        // 추후에는 map -> camera_topRGB_link로 변경될 예정
        init_frm_.set_pose_cw(init_frm_.curr_cam_tf_.inverse());
        Mat44_t cam_pose_cw = Mat44_t::Identity(); // curr 기준 init 위치
        cam_pose_cw.block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
        cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        
        curr_frm.set_pose_cw(cam_pose_cw * init_frm_.get_pose_cw());

        // destruct the initializer
        initializer_.reset(nullptr);
    }
    // std::cout << "최적화 전 camera pose" << std::endl;
    // std::cout << curr_frm.get_pose_cw() << std::endl;
    // create initial keyframes
    auto init_keyfrm = data::keyframe::make_keyframe(map_db_->next_keyframe_id_++, init_frm_);
    auto curr_keyfrm = data::keyframe::make_keyframe(map_db_->next_keyframe_id_++, curr_frm);
    curr_keyfrm->graph_node_->set_spanning_parent(init_keyfrm);
    init_keyfrm->graph_node_->add_spanning_child(curr_keyfrm);
    init_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    curr_keyfrm->graph_node_->set_spanning_root(init_keyfrm);
    map_db_->add_spanning_root(init_keyfrm);

    // compute BoW representations
    init_keyfrm->compute_bow(bow_vocab);
    curr_keyfrm->compute_bow(bow_vocab);

    // add the keyframes to the map DB
    map_db_->add_keyframe(init_keyfrm);
    map_db_->add_keyframe(curr_keyfrm);
    
    // update the frame statistics
    init_frm_.ref_keyfrm_ = init_keyfrm;
    curr_frm.ref_keyfrm_ = curr_keyfrm;
    map_db_->update_frame_statistics(init_frm_, false);
    map_db_->update_frame_statistics(curr_frm, false);

    // assign 2D-3D associations
    std::vector<std::shared_ptr<data::landmark>> lms;
    for(unsigned int init_idx = 0; init_idx < init_matches_.size(); init_idx++)
    {
        const auto curr_idx = init_matches_.at(init_idx);
        if(curr_idx < 0)
            continue;
        
        // construct a landmark
        // 왜 curr 일까?
        std::shared_ptr<data::landmark> lm = std::make_shared<data::landmark>(map_db_->next_landmark_id_++, init_triangulated_pts.at(init_idx), curr_keyfrm);

        //set the associations to the new keyframes
        lm->connect_to_keyframe(init_keyfrm, init_idx);
        lm->connect_to_keyframe(curr_keyfrm, init_idx);

        // update the descriptor - median
        lm->compute_descriptor();
        // update the geometry
        lm->update_mean_normal_and_obs_scale_variance();

        // set the 2D-3D assocications to the current frame
        curr_frm.add_landmark(lm, curr_idx);

        // add the landmark to the map DB
        map_db_->add_landmark(lm);
        lms.push_back(lm);
    }

    // // global bundle adjustment
    const auto global_bundle_adjuster = optimize::global_bundle_adjuster(num_ba_iters_, true);
    std::vector<std::shared_ptr<data::keyframe>> keyfrms{init_keyfrm, curr_keyfrm};
    // global_bundle_adjuster.optimize_for_initialization(keyfrms, lms, markers);
    global_bundle_adjuster.optimize_for_initialization(keyfrms, lms);

    curr_frm.set_pose_cw(curr_keyfrm->get_pose_cw());
    // std::cout << "최적화 후 camera pose" << std::endl;
    // std::cout << curr_frm.get_pose_cw() << std::endl;

    state_ = initializer_state_t::Succeeded;
    return true;
}

} // namespace module