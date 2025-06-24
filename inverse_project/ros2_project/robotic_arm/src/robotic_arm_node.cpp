#include <memory>

#include "robotic_arm/dh_params.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <Eigen/Dense>
#include <cmath>

class RoboticArmNode : public rclcpp::Node
{
public:
    RoboticArmNode(): Node("robotic_arm_node"){
        //LOAD DH PARAMS
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robotic_arm");
        std::string yaml_file_path = package_share_directory + "/config/dh_params.yaml";
        dh_params_ = DHParams::loadParams(yaml_file_path);

        //INITIALIZE PUBLISHERS
        pointsPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("joints", 10);
        linesPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("links", 10);
        pointTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&RoboticArmNode::publishJoints, this));
        lineTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&RoboticArmNode::publishLinks, this));

        //INITIALIZE DATAS
        transforms.clear();
        for (std::size_t i = 0; i< dh_params_.size(); i++){
            transforms.push_back(DHParams::computeForwardKinematics(dh_params_[i]));
        }

        //INITIALIZE SUBSCRIBER
        subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("coordinates", 10,
            std::bind(&RoboticArmNode::coordinatesCallback, this, std::placeholders::_1));
    }

private:
    std::vector<DHParams> dh_params_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pointsPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr linesPublisher_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscriber_;
    visualization_msgs::msg::MarkerArray joints;
    visualization_msgs::msg::MarkerArray links;
    rclcpp::TimerBase::SharedPtr pointTimer_;
    rclcpp::TimerBase::SharedPtr lineTimer_;
    std::vector<Eigen::Matrix4d> transforms;


    void publishJoints()
    {
        joints.markers.clear();

        visualization_msgs::msg::Marker joint;

        joint.header.frame_id = "map";
        joint.header.stamp = this->get_clock()->now();

        joint.ns = "robotic_joint_"+std::to_string(0);
        joint.id = 0;
        joint.type = visualization_msgs::msg::Marker::SPHERE;
        joint.action = visualization_msgs::msg::Marker::ADD;

        joint.pose.position.x = 0.0;
        joint.pose.position.y = 0.0;
        joint.pose.position.z = 0.0;

        joint.pose.orientation.x = 0.0;
        joint.pose.orientation.y = 0.0;
        joint.pose.orientation.z = 0.0;
        joint.pose.orientation.w = 1.0;

        joint.scale.x = 0.2;
        joint.scale.y = 0.2;
        joint.scale.z = 0.2;

        joint.color.r = 1.0;
        joint.color.g = 0.0;
        joint.color.b = 0.0;
        joint.color.a = 1.0;
        joints.markers.push_back(joint);

        for(std::size_t i = 0; i<transforms.size(); i++){
            joint.header.frame_id = "map";
            joint.header.stamp = this->get_clock()->now();

            joint.ns = "robotic_joint_"+std::to_string(i+1);
            joint.id = i+1;
            joint.type = visualization_msgs::msg::Marker::SPHERE;
            if(i == transforms.size()-1){
                joint.color.r = 0.0;
                joint.color.g = 0.0;
                joint.color.b = 1.0;
                joint.color.a = 1.0;
            } else {
                joint.color.r = 1.0;
                joint.color.g = 0.0;
                joint.color.b = 0.0;
                joint.color.a = 1.0;
            }
            joint.action = visualization_msgs::msg::Marker::ADD;

            Eigen::Matrix4d T1 = transforms[0];

            for(std::size_t k = 1; k < i+1; k++){
                T1 *= transforms[k];
            }
            joint.pose.position.x = T1(0, 3); 
            joint.pose.position.y = T1(1, 3);
            joint.pose.position.z = T1(2, 3);

            joint.pose.orientation.x = 0.0;
            joint.pose.orientation.y = 0.0;
            joint.pose.orientation.z = 0.0;
            joint.pose.orientation.w = 1.0;

            joint.scale.x = 0.2;
            joint.scale.y = 0.2;
            joint.scale.z = 0.2;

            joints.markers.push_back(joint);
        }

        pointsPublisher_->publish(joints);
    }
    void publishLinks()
    {
        links.markers.clear();

        if (joints.markers.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough markers to form lines.");
            return;
        }

        for (std::size_t o = 0; o < joints.markers.size()-1; o++){
            visualization_msgs::msg::Marker link;
            link.header.frame_id = "map";
            link.header.stamp = this->get_clock()->now();
    
            link.ns = "robotic_link:_"+std::to_string(o);
            link.id = o+joints.markers.size();
            link.type = visualization_msgs::msg::Marker::LINE_STRIP;
            link.action = visualization_msgs::msg::Marker::ADD;
    
            link.pose.orientation.w = 1.0;
    
            link.scale.x = 0.05;
    
            link.color.r = 0.0;
            link.color.g = 1.0;
            link.color.b = 0.0;
            link.color.a = 1.0;
    
            geometry_msgs::msg::Point p1, p2;
            p1.x = joints.markers[o].pose.position.x;
            p1.y = joints.markers[o].pose.position.y;
            p1.z = joints.markers[o].pose.position.z;
    
            p2.x = joints.markers[o+1].pose.position.x; 
            p2.y = joints.markers[o+1].pose.position.y;
            p2.z = joints.markers[o+1].pose.position.z;
            link.points.push_back(p1);
            link.points.push_back(p2);
            links.markers.push_back(link);
        }
        linesPublisher_->publish(links);
    }
    void coordinatesCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%.2f, y=%.2f, z=%.2f", msg->x, msg->y, msg->z);
        Eigen::Vector3d desiredPos(msg->x, msg->y, msg->z);
        
        solveInverseKinematics(desiredPos);
    }
    void solveInverseKinematics(const Eigen::Vector3d &desired_position){
        const double tolerance = 1e-2; 
        const int max_iterations = 100;
        int iteration = 0;

        while (iteration < max_iterations)
        {
            Eigen::Matrix4d T_end = Eigen::Matrix4d::Identity();
            for (const auto &T : transforms) {
                T_end *= T;
            }
            Eigen::Vector3d current_position(T_end(0, 3), T_end(1, 3), T_end(2, 3));

            Eigen::Vector3d error = desired_position - current_position;

            if (error.norm() < tolerance) {
                RCLCPP_INFO(this->get_logger(), "Inverse kinematics converged after %d iterations.", iteration);
                transforms.clear();
                for (std::size_t i = 0; i< dh_params_.size(); i++){
                    transforms.push_back(DHParams::computeForwardKinematics(dh_params_[i]));
                }
                return;
            }

            Eigen::MatrixXd J_pseudo = DHParams::computeJacobianPseudoInverse(dh_params_, transforms);

            Eigen::VectorXd delta_theta = J_pseudo * error *0.1;

            for (std::size_t  i = 0; i < dh_params_.size(); i++) {
                dh_params_[i].theta += delta_theta(i);
                transforms[i] = DHParams::computeForwardKinematics(dh_params_[i]);
            }
            publishJoints();
            publishLinks();
            rclcpp::sleep_for(std::chrono::milliseconds(100));

            iteration++;
        }
    RCLCPP_WARN(this->get_logger(), "Inverse kinematics did not converge within the maximum number of iterations. %d", iteration);
    }
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoboticArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
