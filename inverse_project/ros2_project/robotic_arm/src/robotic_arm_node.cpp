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
    RoboticArmNode()
        : Node("robotic_arm_node")
    {
        // Load DH parameters from a YAML file
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robotic_arm");
        std::string yaml_file_path = package_share_directory + "/config/dh_params.yaml";
        dh_params_ = DHParams::loadParams(yaml_file_path);
        // Initialize publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("arm_status", 10);
        pointsPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("joints", 10);
        linesPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("links", 10);
        pointTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RoboticArmNode::publishPoints, this));
        lineTimer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RoboticArmNode::publishLines, this));
        transforms.clear();

        for (auto i = 0; i< dh_params_.size(); i++){
            transforms.push_back(computeForwardKinematics(dh_params_[i]));
        }
        // Example of publishing a message
        auto message = std_msgs::msg::String();

        message.data = "size" + std::to_string(transforms.size()) + " joints:";
        for (const auto &param : dh_params_)
        {
            message.data += "\n" + param.toString();
        }
        RCLCPP_INFO(this->get_logger(), "size: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

private:
    std::vector<DHParams> dh_params_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pointsPublisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr linesPublisher_;
    visualization_msgs::msg::MarkerArray joints;
    visualization_msgs::msg::MarkerArray lines;
    rclcpp::TimerBase::SharedPtr pointTimer_;
    rclcpp::TimerBase::SharedPtr lineTimer_;
    Eigen::Matrix4d T;
    std::vector<Eigen::Matrix4d> transforms;

    void publishPoints()
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

        for(auto i = 0; i<transforms.size(); i++){
            joint.header.frame_id = "map";
            joint.header.stamp = this->get_clock()->now();

            joint.ns = "robotic_joint_"+std::to_string(i+1);
            joint.id = i+1;
            joint.type = visualization_msgs::msg::Marker::SPHERE;
            joint.action = visualization_msgs::msg::Marker::ADD;

            Eigen::Matrix4d T1 = transforms[0];

            for(auto k = 1; k < i+1; k++){
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

            joint.color.r = 1.0;
            joint.color.g = 0.0;
            joint.color.b = 0.0;
            joint.color.a = 1.0;
            joints.markers.push_back(joint);
        }

        pointsPublisher_->publish(joints);
    }
    void publishLines()
    {
        lines.markers.clear();

        if (joints.markers.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Not enough markers to form lines.");
            return;
        }

        for (auto o = 0; o < joints.markers.size()-1; o++){
            visualization_msgs::msg::Marker line;
            line.header.frame_id = "map";
            line.header.stamp = this->get_clock()->now();
    
            line.ns = "robotic_link:_"+std::to_string(o);
            line.id = o+joints.markers.size();
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
    
            line.pose.orientation.w = 1.0;
    
            line.scale.x = 0.05;
    
            line.color.r = 0.0;
            line.color.g = 1.0;
            line.color.b = 0.0;
            line.color.a = 1.0;
    
            geometry_msgs::msg::Point p1, p2;
            p1.x = joints.markers[o].pose.position.x;
            p1.y = joints.markers[o].pose.position.y;
            p1.z = joints.markers[o].pose.position.z;
    
            p2.x = joints.markers[o+1].pose.position.x; 
            p2.y = joints.markers[o+1].pose.position.y;
            p2.z = joints.markers[o+1].pose.position.z;
            line.points.push_back(p1);
            line.points.push_back(p2);
            lines.markers.push_back(line);
        }
        linesPublisher_->publish(lines);
    }
    Eigen::Matrix4d computeForwardKinematics(const DHParams &param)
    {
        T = Eigen::Matrix4d::Identity();

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
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RoboticArmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
