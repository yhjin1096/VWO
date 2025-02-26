#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <ros/ros.h>

#include <string>
#include <memory>

#include <yaml-cpp/yaml.h> 

class Config
{
    public:
        Config(const std::string& config_file_path);
        // Config(const YAML::Node& yaml_node, const std::string& config_file_path);
        
        YAML::Node yaml_node_;
    private:
};

#endif