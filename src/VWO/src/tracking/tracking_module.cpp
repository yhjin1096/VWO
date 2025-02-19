#include "VWO/tracking/tracking_module.hpp"

TrackingModule::TrackingModule(const std::shared_ptr<Config>& cfg, Camera* camera)
    : camera_(camera)
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


    //dummy pose
    return std::make_shared<Mat44_t>(Mat44_t::Identity());
}