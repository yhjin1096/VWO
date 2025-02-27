#include "VWO/data/frame.h"

namespace data
{
Frame::Frame(const unsigned int frame_id, const double timestamp, Camera* camera,
                orb_params* orb_params, const frame_observation frm_obs)
        :   id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
            landmarks_(std::vector<std::shared_ptr<landmark>>(frm_obs_.undist_keypts_.size(), nullptr))
{

}

Frame::Frame(const unsigned int frame_id, const double timestamp, Camera* camera,
                orb_params* orb_params, const frame_observation frm_obs, cv::Mat image)
        :   id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
            landmarks_(std::vector<std::shared_ptr<landmark>>(frm_obs_.undist_keypts_.size(), nullptr)),
            image_(image)
{

}

Frame::Frame(const unsigned int frame_id, const double timestamp, Camera* camera,
                orb_params* orb_params, const frame_observation frm_obs, cv::Mat image, const Mat44_t& curr_cam_tf)
        :   id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
            landmarks_(std::vector<std::shared_ptr<landmark>>(frm_obs_.undist_keypts_.size(), nullptr)),
            image_(image), curr_cam_tf_(curr_cam_tf)
{

}

std::vector<unsigned int> Frame::get_keypoints_in_cell(const float ref_x, const float ref_y, const float margin, const int min_level, const int max_level) const
{
    return data::get_keypoints_in_cell(camera_, frm_obs_, ref_x, ref_y, margin, min_level, max_level);
}

void Frame::set_pose_cw(const Mat44_t& pose_cw) {
pose_is_valid_ = true;
pose_cw_ = pose_cw;

rot_cw_ = pose_cw_.block<3, 3>(0, 0);
rot_wc_ = rot_cw_.transpose();
trans_cw_ = pose_cw_.block<3, 1>(0, 3);
trans_wc_ = -rot_cw_.transpose() * trans_cw_;
}

Mat44_t Frame::get_pose_cw() const {
    return pose_cw_;
}

Mat44_t Frame::get_pose_wc() const {
    Mat44_t pose_wc = Mat44_t::Identity();
    pose_wc.block<3, 3>(0, 0) = rot_wc_;
    pose_wc.block<3, 1>(0, 3) = trans_wc_;
    return pose_wc;
}

Vec3_t Frame::get_trans_wc() const {
    return trans_wc_;
}

Mat33_t Frame::get_rot_wc() const {
    return rot_wc_;
}

std::vector<std::shared_ptr<landmark>> Frame::get_landmarks() const
{
    return landmarks_;
}

bool Frame::has_landmark(const std::shared_ptr<landmark>& lm) const
{
    return static_cast<bool>(landmarks_idx_map_.count(lm));
}

void Frame::add_landmark(const std::shared_ptr<landmark>& lm, const unsigned int idx)
{
    assert(!has_landmark(lm));
    landmarks_.at(idx) = lm;
    landmarks_idx_map_[lm] = idx;
}

} // namespace data