#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class TFBroadcaster : public rclcpp::Node
{
public:
  TFBroadcaster() : Node("tf_broadcaster")
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    grid_ = nullptr;

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "occ_map", 1, std::bind(&TFBroadcaster::map_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10), std::bind(&TFBroadcaster::broadcast_transforms, this));
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Creating transform for map...");
    grid_ = msg;
  }

  void broadcast_transforms()
  {
    if (grid_) {
      auto tf_pose = grid_->info.origin;

      tf2_ros::Transform map_to_path;
      map_to_path.setIdentity();

      geometry_msgs::msg::TransformStamped map_to_path_tf;
      map_to_path_tf.header = grid_->header;
      map_to_path_tf.child_frame_id = "path";
      map_to_path_tf.transform = tf2_ros::toMsg(map_to_path);

      tf2_ros::Transform map_to_odom;
      tf2_ros::Transform map_to_odom_tf;
      map_to_odom.setOrigin(tf2_ros::Vector3(tf_pose.position.x, tf_pose.position.y, tf_pose.position.z));
      map_to_odom.setRotation(tf2_ros::Quaternion(0, 0, 0, 1));

      map_to_odom_tf = map_to_path.inverseTimes(map_to_odom);

      geometry_msgs::msg::TransformStamped map_to_odom_msg;
      map_to_odom_msg.header = grid_->header;
      map_to_odom_msg.child_frame_id = "odom";
      map_to_odom_msg.transform = tf2_ros::toMsg(map_to_odom_tf);

      tf_broadcaster_->sendTransform(map_to_path_tf);
      tf_broadcaster_->sendTransform(map_to_odom_msg);
    }
  }

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
