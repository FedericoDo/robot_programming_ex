#include "robotic_arm/dh_params.h"

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

Eigen::Matrix4d DHParams::computeForwardKinematics(const DHParams &param)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    double theta = param.theta;
    double d = param.d;
    double a = param.a;
    double alpha = param.alpha;

    Eigen::Matrix4d A;
    T << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta),
        sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta),
        0, sin(alpha), cos(alpha), d,
        0, 0, 0, 1;

    return T;
}

Eigen::MatrixXd DHParams::computeJacobianPseudoInverse(std::vector<DHParams> &dh_params_, std::vector<Eigen::Matrix4d> &transforms){
    int n = dh_params_.size();
    double delta = 1e-3;
    Eigen::MatrixXd J(3,n);
    
    Eigen::Matrix4d T_end = Eigen::Matrix4d::Identity();
    for (const auto &T : transforms) {
        T_end *= T;
    }
    Eigen::Vector3d end_effector_position(T_end(0, 3), T_end(1, 3), T_end(2, 3));
    
    for(int i=0; i<n; i++){
        double original_theta = dh_params_[i].theta;
        dh_params_[i].theta += delta;
        transforms[i] = DHParams::computeForwardKinematics(dh_params_[i]);
        Eigen::Matrix4d T_modified = Eigen::Matrix4d::Identity();
        for (const auto &T : transforms) {
            T_modified *= T;
        }
        Eigen::Vector3d modified_position(T_modified(0, 3), T_modified(1, 3), T_modified(2, 3));
        Eigen::Vector3d variation = (modified_position - end_effector_position) / delta;
        
        J(0, i) = variation(0); 
        J(1, i) = variation(1); 
        J(2, i) = variation(2);

        dh_params_[i].theta = original_theta;
        transforms[i] = DHParams::computeForwardKinematics(dh_params_[i]);
    }
    J = J.completeOrthogonalDecomposition().pseudoInverse();
    
    return J;
}