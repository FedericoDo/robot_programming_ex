#include <vector>
#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <cmath>


struct DHParams {
    double theta;  
    double d;     
    double a;     
    double alpha; 

    static std::vector<DHParams> loadParams(const std::string& filepath);

    std::string toString() const;

    static Eigen::Matrix4d computeForwardKinematics(const DHParams &param);

    static Eigen::MatrixXd computeJacobianPseudoInverse(std::vector<DHParams> &dh_params_, std::vector<Eigen::Matrix4d> &transforms);
};