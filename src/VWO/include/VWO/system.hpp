#ifndef SYSTEM_HPP
#define SYSTEM_HPP

#include "VWO/config.hpp"
#include "VWO/feature_extractor.hpp"
#include "VWO/orb_extractor.hpp"
#include "VWO/orb_params.hpp"

#include "util/yaml.hpp"

class System
{
    public:
        System(const std::shared_ptr<Config>& cfg);
        std::unique_ptr<FeatureExtractor> feature_extractor_;

    private:
        std::shared_ptr<Config> config_;

        orb_params* orb_params_ = nullptr;

        //! number of columns of grid to accelerate reprojection matching
        unsigned int num_grid_cols_ = 64;
        //! number of rows of grid to accelerate reprojection matching
        unsigned int num_grid_rows_ = 48;
};

#endif