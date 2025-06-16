#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <iostream>


class CoordinatesPublisher : public rclcpp::Node
{
    public:
    CoordinatesPublisher(): Node("coordinates_publisher"){
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("coordinates", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CoordinatesPublisher::publishCoordinates, this));
    }
    private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    void publishCoordinates()
    {
        double x,y,z;
        std::cout << "Enter coordinates (x, y, z): ";
        if (!(std::cin >> x >> y >> z)) {
            std::cout << "Invalid input. Please enter three valid coordinates." ;
            rclcpp::shutdown();
            return;
        }
        geometry_msgs::msg::Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        publisher_->publish(point);
        RCLCPP_INFO(this->get_logger(), "Published coordinates: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinatesPublisher>());
    rclcpp::shutdown();
    return 0;
}