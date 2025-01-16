#include "VWO/system.hpp"

System::System(const std::shared_ptr<Config>& cfg)
    : config_(cfg)
{
    orb_params_ = new orb_params(config_->yaml_node_["Feature"]);
    YAML::Node preprocessing_params = config_->yaml_node_["Feature"];
    const unsigned int min_size = preprocessing_params["min_size"].as<unsigned int>(800);
    std::vector<std::vector<float>> mask_rectangles = util::get_rectangles(preprocessing_params["mask_rectangles"]);

    if(orb_params_->name_ == "ORB")
        feature_extractor_ = std::make_unique<ORBExtractor>(orb_params_, min_size, mask_rectangles);
        // feature_extractor_ = std::make_unique<ORBExtractor>(orb_params_, );

}