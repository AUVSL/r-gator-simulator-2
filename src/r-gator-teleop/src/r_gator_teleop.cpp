#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

class TeleopTurtle : public rclcpp::Node
{
public:
    TeleopTurtle();

private:
    void callback(const sensor_msgs::msg::Joy::SharedPtr joy);
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    int axis_linear, axis_angular;
};

TeleopTurtle::TeleopTurtle() : Node("teleop_turtle")
{
    declare_parameter<int>("axis_linear", 1);
    declare_parameter<int>("axis_angular", 2);

    get_parameter("axis_linear", axis_linear);
    get_parameter("axis_angular", axis_angular);

    pub = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
    sub = create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&TeleopTurtle::callback, this, std::placeholders::_1));
}

void TeleopTurtle::callback(const sensor_msgs::msg::Joy::SharedPtr joy)
{
    auto vel = std::make_unique<geometry_msgs::msg::Twist>();
    vel->linear.x = joy->axes[axis_linear];
    vel->angular.z = joy->axes[axis_angular];
    RCLCPP_INFO(get_logger(), "current linear velocity: %.3lf ; current angular velocity: %.3lf", vel->linear.x, vel->angular.z);
    pub->publish(std::move(vel));
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);
    auto teleopTurtle = std::make_shared<TeleopTurtle>();
    rclcpp::spin(teleopTurtle);
    rclcpp::shutdown();
    return 0;
}
