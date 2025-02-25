#ifndef STELLA_VSLAM_INITIALIZE_PERSPECTIVE_H
#define STELLA_VSLAM_INITIALIZE_PERSPECTIVE_H

#include "VWO/type.hpp"
#include "VWO/initialize/base.h"

namespace data {
class Frame;
} // namespace data

namespace initialize {

class perspective final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    perspective() = delete;

    //! Constructor
    perspective(const data::Frame& ref_frm,
                const unsigned int num_ransac_iters,
                const unsigned int min_num_triangulated,
                const unsigned int min_num_valid_pts,
                const float parallax_deg_thr,
                const float reproj_err_thr,
                bool use_fixed_seed = false);

    //! Destructor
    ~perspective() override;

    //! Initialize with the current frame
    bool initialize(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;
    bool initialize_with_odom(const data::Frame& cur_frm, const std::vector<int>& ref_matches_with_cur, const Mat44_t& relative_cam_tf) override;

private:
    //! Reconstruct the initial map with the H matrix
    //! (NOTE: the output variables will be set if succeeded)
    bool reconstruct_with_H(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match);
    bool reconstruct_with_H_and_odom(const Mat33_t& H_ref_to_cur, const std::vector<bool>& is_inlier_match, const Mat44_t& relative_cam_tf);

    //! Reconstruct the initial map with the F matrix
    //! (NOTE: the output variables will be set if succeeded)
    bool reconstruct_with_F(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match);
    bool reconstruct_with_F_and_odom(const Mat33_t& F_ref_to_cur, const std::vector<bool>& is_inlier_match, const Mat44_t& relative_cam_tf);

    //! Get the camera matrix from the camera object
    static Mat33_t get_camera_matrix(Camera* camera);

    //! camera matrix of the reference frame
    const Mat33_t ref_cam_matrix_;
    //! camera matrix of the current frame
    Mat33_t cur_cam_matrix_;

    //! Use fixed random seed for RANSAC if true
    const bool use_fixed_seed_;
};

} // namespace initialize

#endif // STELLA_VSLAM_INITIALIZE_PERSPECTIVE_H
