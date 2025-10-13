#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


using namespace std::chrono_literals;

class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
      : Node("trajectory_preprocessor")
  {
    // Declare and acquire `planning_frame` parameter
    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "world");

    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    using std::placeholders::_1;

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints_transformed", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/waypoints", 10, std::bind(&FrameListener::waypoint_callback, this, _1));
  }

private:
  void waypoint_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    // Store frame names in variables that will be used to
    // compute transformations
    std::string toFrameRel = planning_frame_.c_str();
    std::string fromFrameRel = msg->header.frame_id;

    try
    {
      geometry_msgs::msg::PoseArray transformed_msg;
      transformed_msg.header.frame_id = toFrameRel;
      transformed_msg.header.stamp = this->get_clock()->now();

      size_t n = msg->poses.size();

      for (size_t i = 0; i < n; i++) {
        geometry_msgs::msg::PoseStamped pose_in;
        pose_in.header = msg->header;
        pose_in.pose = msg->poses[i];

        geometry_msgs::msg::PoseStamped pose_out;
        pose_out = tf_buffer_->transform(pose_in, toFrameRel);
        transformed_msg.poses.push_back(pose_out.pose);
      }

      // Orient the previous waypoint to face the next waypoint in 3D space
      if (n > 1) {
        for (size_t i = 0; i < n-1; i++) {
          geometry_msgs::msg::Pose pose_1 = transformed_msg.poses[i];
          geometry_msgs::msg::Pose pose_2 = transformed_msg.poses[i + 1];

          // New logic: Z-axis down, X-axis towards next waypoint
          tf2::Vector3 x_dir(
              pose_2.position.x - pose_1.position.x,
              pose_2.position.y - pose_1.position.y,
              pose_2.position.z - pose_1.position.z
          );

          if (x_dir.length2() < 1e-9) {
              continue; // Poses are coincident, skip
          }
          x_dir.normalize();

          tf2::Vector3 z_down(0.0, 0.0, -1.0);
          tf2::Vector3 y_dir = z_down.cross(x_dir).normalized();
          tf2::Vector3 x_new = y_dir.cross(z_down).normalized();

          tf2::Matrix3x3 rot_matrix;
          rot_matrix.setValue(
              x_new.x(), y_dir.x(), z_down.x(),
              x_new.y(), y_dir.y(), z_down.y(),
              x_new.z(), y_dir.z(), z_down.z()
          );

          tf2::Quaternion q;
          rot_matrix.getRotation(q);
          q.normalize();

          transformed_msg.poses[i].orientation.x = q.x();
          transformed_msg.poses[i].orientation.y = q.y();
          transformed_msg.poses[i].orientation.z = q.z();
          transformed_msg.poses[i].orientation.w = q.w();
        }

        // For the last waypoint, just copy the orientation of the previous one
        transformed_msg.poses[n-1].orientation = transformed_msg.poses[n-2].orientation;
      }

      publisher_->publish(transformed_msg);
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          fromFrameRel.c_str(), toFrameRel.c_str(), ex.what());
      return;
    }
  }

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
  std::string planning_frame_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrameListener>());
  rclcpp::shutdown();
  return 0;
}