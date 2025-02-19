#include "VWO/tracking/tracking_module.hpp"

TrackingModule::TrackingModule(const std::shared_ptr<Config>& cfg, Camera* camera)
    : camera_(camera), //vocab
      num_ransac_iters_(cfg->yaml_node_["Tracking"]["num_ransac_iterations"].as<unsigned int>(100)),
      min_num_valid_pts_(cfg->yaml_node_["Tracking"]["min_num_valid_pts"].as<unsigned int>(50)),
      min_num_triangulated_pts_(cfg->yaml_node_["Tracking"]["min_num_triangulated_pts"].as<unsigned int>(50)),
      parallax_deg_thr_(cfg->yaml_node_["Tracking"]["parallax_deg_threshold"].as<float>(1.0)),
      reproj_err_thr_(cfg->yaml_node_["Tracking"]["reprojection_error_threshold"].as<float>(4.0)),
      use_fixed_seed_(cfg->yaml_node_["Tracking"]["use_fixed_seed"].as<bool>(false))
{
    
}

TrackingModule::~TrackingModule()
{

}

std::shared_ptr<Mat44_t> TrackingModule::trackFrame(data::Frame curr_frm, data::Frame prev_frm)
{
    //match
    std::vector<cv::Point2f> prev_matched_coords_;
    prev_matched_coords_.resize(prev_frm.frm_obs_.undist_keypts_.size());
    for (unsigned int i = 0; i < prev_frm.frm_obs_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = prev_frm.frm_obs_.undist_keypts_.at(i).pt;
    }
    std::vector<int> init_matches_;

    // init_matches_[i]가 j 일때 -> i는 prev_frm의 keypoint idx, j는 이에 매칭된 curr_frm의 keypoint idx
    match::area matcher(0.9, true);
    const auto num_matches = matcher.match_in_consistent_area(prev_frm, curr_frm, prev_matched_coords_, init_matches_, 100);
    
    // visualize matches
    if(0)
    {
        cv::Mat prev_img = prev_frm.image_.clone();
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
                    cv::Point2f pt_prev = prev_frm.frm_obs_.undist_keypts_.at(i).pt;
                    cv::Point2f pt_curr = curr_frm.frm_obs_.undist_keypts_.at(curr_point_idx).pt;
                    pt_curr.x += camera_->cols_;

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

    //pose
    data::Frame init_frm_= data::Frame(prev_frm);
    initializer_ = std::make_unique<initialize::perspective>(
                    init_frm_, num_ransac_iters_, min_num_triangulated_pts_, min_num_valid_pts_,
                    parallax_deg_thr_, reproj_err_thr_, use_fixed_seed_);
    
    initializer_->initialize(init_frm_, init_matches_);
    std::cout << initializer_->get_rotation_ref_to_cur() << std::endl;
    std::cout << initializer_->get_translation_ref_to_cur().transpose() << std::endl;

    std::shared_ptr<Mat44_t> cam_pose_cw = std::make_shared<Mat44_t>(Mat44_t::Identity());
    cam_pose_cw->block<3, 3>(0, 0) = initializer_->get_rotation_ref_to_cur();
    cam_pose_cw->block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();

    //dummy pose
    return cam_pose_cw;
}