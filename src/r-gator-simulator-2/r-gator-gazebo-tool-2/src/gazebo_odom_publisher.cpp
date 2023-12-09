#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <gazebo_msgs/srv/get_model_state.hpp>
#include <gazebo_msgs/msg/model_states.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using std::placeholders::_1;

class GazeboOdometry : public rclcpp::Node
{
public:
    GazeboOdometry();
    bool check_robot_name();
    void ModelStatecallback(const gazebo_msgs::msg::ModelStates::SharedPtr box_state_current);
    void pub_odom();

private:
    string robot_name;
    string odom_topic;
    string robot_frame_id;
    int hz;

    bool robot_exist;

    rclcpp::Client<gazebo_msgs::srv::GetModelState>::SharedPtr states_client;
    gazebo_msgs::srv::GetModelState::Request::SharedPtr model_states_request;
    nav_msgs::msg::Odometry odom;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;

    tf2::Transform transform;
    tf2::Quaternion quaternion;
};

GazeboOdometry::GazeboOdometry() : Node("gazebo_odom_publisher")
{
    this->declare_parameter<string>("robot_name", "gem");
    this->declare_parameter<string>("odom_topic", "/odom");
    this->declare_parameter<string>("robot_frame_id", "/base_footprint");
    this->declare_parameter<int>("Frequency_of_odom_publisher", 10);

    this->get_parameter("robot_name", robot_name);
    this->get_parameter("odom_topic", odom_topic);
    this->get_parameter("robot_frame_id", robot_frame_id);
    this->get_parameter("Frequency_of_odom_publisher", hz);

    robot_exist = false;

    states_client = this->create_client<gazebo_msgs::srv::GetModelState>("/gazebo/get_model_state");
    model_states_request = std::make_shared<gazebo_msgs::srv::GetModelState::Request>();
    odometry_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, hz);

    odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void GazeboOdometry::ModelStatecallback(const gazebo_msgs::msg::ModelStates::SharedPtr model_state_current)
{
    if(model_state_current->name.size() == 0)
    {
        return;
    }
    for(size_t i = 0; i < model_state_current->name.size(); i++)
    {
        if(model_state_current->name[i] == robot_name)
        {
            robot_exist = true;
        }
    }
}

bool GazeboOdometry::check_robot_name()
{
    return robot_exist;
}

void GazeboOdometry::pub_odom()
{
    model_states_request->model_name = robot_name;
    model_states_request->relative_entity_name = "world";

    auto result = states_client->async_send_request(model_states_request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == 
        rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = result.get();
        odom.pose.pose = response->pose;
        odom.twist.twist = response->twist;

        geometry_msgs::msg::TransformStamped transformStamped;

        transformStamped.header.stamp = this->get_clock()->now();
        transformStamped.header.frame_id = "odom";
        transformStamped.child_frame_id = robot_frame_id;
        transformStamped.transform.translation.x = odom.pose.pose.position.x;
        transformStamped.transform.translation.y = odom.pose.pose.position.y;
        transformStamped.transform.translation.z = odom.pose.pose.position.z;
        transformStamped.transform.rotation = odom.pose.pose.orientation;

        odom_broadcaster->sendTransform(transformStamped);

        odometry_pub->publish(odom);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboOdometry>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
