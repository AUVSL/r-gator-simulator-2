#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/srv/get_model_state.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace std;
using namespace rclcpp;
using std::placeholders::_1;

class GazeboOdometry : public Node
{
public:
    GazeboOdometry();

private:
    void ModelStateCallback(const gazebo_msgs::msg::ModelStates::SharedPtr model_state_current);
    void PublishOdometry();
    bool CheckRobotName();

    string robot_name;
    string odom_topic;
    string robot_frame_id;
    string hz;
    bool robot_exist;

    Client<gazebo_msgs::srv::GetModelState>::SharedPtr states_client;
    gazebo_msgs::srv::GetModelState::Request model_states_request;
    gazebo_msgs::srv::GetModelState::Response model_states_response;
    nav_msgs::msg::Odometry odom;
    Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2::Transform transform;
    tf2::Quaternion quaternion;
};

GazeboOdometry::GazeboOdometry() : Node("gazebo_odom_publisher")
{
    robot_exist = false;
    get_parameter("robot_name", robot_name, "gem");
    get_parameter("odom_topic", odom_topic, "/odom");
    get_parameter("robot_frame_id", robot_frame_id, "/base_footprint");
    get_parameter("Frequency_of_odom_publisher", hz, "10");

    states_client = create_client<gazebo_msgs::srv::GetModelState>("/gazebo/get_model_state");
    odometry_pub = create_publisher<nav_msgs::msg::Odometry>(odom_topic, std::atoi(hz.c_str()));

    auto timer_callback = [this]() -> void
    {
        if (robot_exist)
        {
            PublishOdometry();
        }
        else
        {
            if (robot_exist)
            {
                robot_exist = true;
                RCLCPP_INFO(get_logger(), "Robot is not fully loaded. Please wait or check if the robot_name exists in Gazebo.");
            }
        }
    };

    create_wall_timer(10ms, timer_callback);

    RCLCPP_INFO(get_logger(), "gazebo_odom_publisher node has been initialized.");
}

void GazeboOdometry::ModelStateCallback(const gazebo_msgs::msg::ModelStates::SharedPtr model_state_current)
{
    if (model_state_current->name.size() == 0)
    {
        return;
    }

    for (size_t i = 0; i < model_state_current->name.size(); ++i)
    {
        if (model_state_current->name[i] == robot_name)
        {
            robot_exist = true;
        }
    }
}

void GazeboOdometry::PublishOdometry()
{
    states_client->async_send_request(model_states_request);

    if (states_client->async_get_result(model_states_response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        odom.pose.pose = model_states_response.pose;
        odom.twist.twist = model_states_response.twist;

        odometry_pub->publish(odom);

        tf2::Quaternion quaternion;
        tf2::fromMsg(odom.pose.pose.orientation, quaternion);

        transform.setOrigin(tf2::Vector3(
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z));
        transform.setRotation(quaternion);

        odom_broadcaster.sendTransform(tf2::StampedTransform(
            transform, now(), odom_topic.c_str(), robot_frame_id.c_str()));
    }
}

bool GazeboOdometry::CheckRobotName()
{
    return robot_exist;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GazeboOdometry>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
