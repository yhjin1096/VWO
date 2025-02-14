#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "VWO/config.hpp"

#include "VWO/camera/camera.hpp"
#include "VWO/camera/perspective.hpp"

#include "VWO/feature/feature_extractor.hpp"
#include "VWO/feature/orb_extractor.hpp"
#include "VWO/feature/orb_params.hpp"

#include "util/yaml.hpp"

class System
{
    public:
        System(const std::shared_ptr<Config>& cfg);
        ~System();
        Camera* camera_;
        FeatureExtractor* feature_extractor_;

    private:
        std::shared_ptr<Config> config_;

        orb_params* orb_params_ = nullptr;
        //! number of columns of grid to accelerate reprojection matching
        unsigned int num_grid_cols_ = 64;
        //! number of rows of grid to accelerate reprojection matching
        unsigned int num_grid_rows_ = 48;
};

#endif