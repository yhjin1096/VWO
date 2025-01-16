#include "VWO/config.hpp"

Config::Config(const std::string& config_file_path)
{
    try
    {
        yaml_node_ = YAML::LoadFile(config_file_path);
    }
    catch (const YAML::Exception &e)
    {
        ROS_ERROR("Failed to get 'config_file_path' from parameter server.");
        ROS_ERROR("Failed to load or parse YAML file: %s", e.what());
    }
}