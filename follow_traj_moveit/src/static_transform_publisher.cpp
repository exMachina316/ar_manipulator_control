#include <memory>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

class StaticFramePublisher : public rclcpp::Node
{
public:
  explicit StaticFramePublisher()
  : Node("static_tf2_broadcaster")
  {
    // Declare parameters with default values
    this->declare_parameter("world_to_camera.parent_frame", "world");
    this->declare_parameter("world_to_camera.child_frame", "camera_frame");
    this->declare_parameter("world_to_camera.translation", std::vector<double>{2.0, 0.0, 0.5});
    this->declare_parameter("world_to_camera.rotation", std::vector<double>{0.0, 0.0, 3.14});

    this->declare_parameter("camera_to_optical.parent_frame", "camera_frame");
    this->declare_parameter("camera_to_optical.child_frame", "camera_optical_frame");
    this->declare_parameter("camera_to_optical.translation", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter("camera_to_optical.rotation", std::vector<double>{-1.57, 0.0, -1.57});

    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Publish transforms
    publish_world_to_camera_transform();
    publish_camera_to_optical_transform();
  }

private:
  void publish_world_to_camera_transform()
  {
    auto parent_frame = this->get_parameter("world_to_camera.parent_frame").as_string();
    auto child_frame = this->get_parameter("world_to_camera.child_frame").as_string();
    auto translation = this->get_parameter("world_to_camera.translation").as_double_array();
    auto rotation = this->get_parameter("world_to_camera.rotation").as_double_array();

    publish_transform(parent_frame, child_frame, translation, rotation);
  }

  void publish_camera_to_optical_transform()
  {
    auto parent_frame = this->get_parameter("camera_to_optical.parent_frame").as_string();
    auto child_frame = this->get_parameter("camera_to_optical.child_frame").as_string();
    auto translation = this->get_parameter("camera_to_optical.translation").as_double_array();
    auto rotation = this->get_parameter("camera_to_optical.rotation").as_double_array();

    publish_transform(parent_frame, child_frame, translation, rotation);
  }

  void publish_transform(
    const std::string& parent_frame,
    const std::string& child_frame,
    const std::vector<double>& translation,
    const std::vector<double>& rotation)
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = parent_frame;
    t.child_frame_id = child_frame;

    t.transform.translation.x = translation[0];
    t.transform.translation.y = translation[1];
    t.transform.translation.z = translation[2];

    tf2::Quaternion q;
    q.setRPY(rotation[0], rotation[1], rotation[2]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_static_broadcaster_->sendTransform(t);
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing static transform from '%s' to '%s'",
      parent_frame.c_str(),
      child_frame.c_str());
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
};

int main(int argc, char * argv[])
{
  auto logger = rclcpp::get_logger("logger");

  // Pass parameters and initialize node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticFramePublisher>());
  rclcpp::shutdown();
  return 0;
}