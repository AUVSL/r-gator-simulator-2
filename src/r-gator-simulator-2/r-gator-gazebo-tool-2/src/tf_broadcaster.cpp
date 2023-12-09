/**
 * @file tf_broadcaster.cpp
 * @brief This file contains the implementation of the TfBroadcasterNode class, which is responsible for broadcasting transforms using tf2_ros.
 */

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;

/**
 * @class TfBroadcasterNode
 * @brief The TfBroadcasterNode class is a ROS2 node that broadcasts transforms using tf2_ros.
 */
class TfBroadcasterNode : public rclcpp::Node
{
public:
    /**
     * @brief Constructor of the TfBroadcasterNode class.
     */
    TfBroadcasterNode() : Node("tf_broadcaster_node")
    {
        // Subscribe to map updates
        sub_map_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/occ_map", 1, std::bind(&TfBroadcasterNode::setMap, this, std::placeholders::_1));
    }

    /**
     * @brief Callback function to set the map.
     * @param map The occupancy grid map.
     */
    void setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        std::cout << "Creating transform for map..." << std::endl;
        grid_ = map;
    }

    /**
     * @brief Publishes the transforms.
     */
    void publishTransforms()
    {
        if (grid_ != nullptr)
        {
            // Transform from geometry msg to TF2
            geometry_msgs::msg::TransformStamped odom_to_map;
            tf2::Transform tfTransform;
            tf2::convert(grid_->info.origin, tfTransform);

            odom_to_map.header.stamp = this->now();
            odom_to_map.header.frame_id = "odom";
            odom_to_map.child_frame_id = "map";
            tf2::convert(tfTransform, odom_to_map.transform);

            tf_broadcaster_->sendTransform(odom_to_map);

            // Map to path (example, might be adjusted as needed)
            geometry_msgs::msg::TransformStamped map_to_path;
            map_to_path.header.stamp = this->now();
            map_to_path.header.frame_id = "map";
            map_to_path.child_frame_id = "path";
            map_to_path.transform.rotation.w = 1.0; // Identity rotation

            tf_broadcaster_->sendTransform(map_to_path);
        }
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_; ///< Subscription to the occupancy grid map.
    nav_msgs::msg::OccupancyGrid::SharedPtr grid_; ///< Pointer to the occupancy grid map.
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_ = 
        std::make_unique<tf2_ros::TransformBroadcaster>(*this); ///< Pointer to the tf2_ros::TransformBroadcaster.
};

/**
 * @brief The main function of the tf_broadcaster_node.
 * @param argc The number of command line arguments.
 * @param argv The command line arguments.
 * @return The exit code.
 */
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfBroadcasterNode>();

    rclcpp::Rate rate(100ms);
    while (rclcpp::ok())
    {
        node->publishTransforms();
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
