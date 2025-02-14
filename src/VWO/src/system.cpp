#include "VWO/system.hpp"

System::System(const std::shared_ptr<Config>& cfg)
    : config_(cfg)
{
    YAML::Node camera_params = config_->yaml_node_["Camera"];
    if(camera_params["model"].as<std::string>() == "perspective")
    {
        camera_ = new Perspective(camera_params);
    }
    else
    {

    }

    orb_params_ = new orb_params(config_->yaml_node_["Feature"]);
    YAML::Node preprocessing_params = config_->yaml_node_["Feature"];
    const unsigned int min_size = preprocessing_params["min_size"].as<unsigned int>(800);
    std::vector<std::vector<float>> mask_rectangles = util::get_rectangles(preprocessing_params["mask_rectangles"]);

    if(orb_params_->name_ == "ORB")
        feature_extractor_ = new ORBExtractor(orb_params_, min_size, mask_rectangles);
    
}

System::~System()
{
    delete orb_params_;
    orb_params_ = nullptr;
    
    delete camera_;
    camera_ = nullptr;

    delete feature_extractor_;
    feature_extractor_ = nullptr;
}