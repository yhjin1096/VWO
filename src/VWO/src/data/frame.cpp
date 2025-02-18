#include "VWO/data/frame.hpp"

namespace data
{
    Frame::Frame(const unsigned int frame_id, const double timestamp, Camera* camera,
                  orb_params* orb_params, const frame_observation frm_obs)
            :   id_(frame_id), timestamp_(timestamp), camera_(camera), orb_params_(orb_params), frm_obs_(frm_obs),
                landmarks_(std::vector<std::shared_ptr<Landmark>>(frm_obs_.undist_keypts_.size(), nullptr))
    {

    }
    
}