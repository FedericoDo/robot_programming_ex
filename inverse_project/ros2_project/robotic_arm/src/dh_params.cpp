#include "robotic_arm/dh_params.h"
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>

std::vector<DHParams> DHParams::loadParams(const std::string &filepath)
{
    std::vector<DHParams> params;
    YAML::Node config = YAML::LoadFile(filepath);
    for (const auto &node : config["joints"])
    {
        DHParams param;
        param.theta = node["theta"].as<double>();
        param.d = node["d"].as<double>();
        param.a = node["a"].as<double>();
        param.alpha = node["alpha"].as<double>();
        params.push_back(param);
    }
    return params;
}

std::string DHParams::toString() const
{
    return "DHParams(theta: " + std::to_string(theta) +
           ", d: " + std::to_string(d) +
           ", a: " + std::to_string(a) +
           ", alpha: " + std::to_string(alpha) + ")";
}