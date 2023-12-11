#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TeleopTurtle : public rclcpp::Node
{
public:
    TeleopTurtle();

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    int axis_linear_, axis_angular_;
};

TeleopTurtle::TeleopTurtle() : Node("teleop_turtle")
{
    this->declare_parameter<int>("axis_linear", 1);
    this->declare_parameter<int>("axis_angular", 2);
    this->get_parameter("axis_linear", axis_linear_);
    this->get_parameter("axis_angular", axis_angular_);

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&TeleopTurtle::joyCallback, this, std::placeholders::_1));
}

void TeleopTurtle::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    auto vel = geometry_msgs::msg::Twist();
    vel.linear.x = joy->axes[axis_linear_];
    vel.angular.z = joy->axes[axis_angular_];
    RCLCPP_INFO(this->get_logger(), "current linear velocity: %.3lf ; current angle velocity: %.3lf", vel.linear.x, vel.angular.z);
    pub_->publish(vel);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopTurtle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
