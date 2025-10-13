#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>

class TfInverterNode : public rclcpp::Node
{
public:
    TfInverterNode()
        : Node("tf_inverter_node")
    {
        this->declare_parameter<std::vector<std::string>>("camera_names", {"oak1", "oak2", "oak3"});
        this->get_parameter("camera_names", camera_names_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TfInverterNode::on_timer, this));
    }

private:
    void on_timer()
    {
        for (const auto& camera_name : camera_names_)
        {
            std::string source_frame = camera_name;
            std::string target_frame = "table2_from_" + camera_name;
            std::string new_source_frame = "table2";
            std::string new_target_frame = camera_name;

            geometry_msgs::msg::TransformStamped t;

            try
            {
                t = tf_buffer_->lookupTransform(
                    source_frame, target_frame,
                    tf2::TimePointZero);
            }
            catch (const tf2::TransformException & ex)
            {
                RCLCPP_DEBUG(this->get_logger(), "Could not transform %s to %s: %s",
                    source_frame.c_str(), target_frame.c_str(), ex.what());
                continue;
            }

            tf2::Transform transform;
            tf2::fromMsg(t.transform, transform);

            tf2::Transform inverse_transform = transform.inverse();

            geometry_msgs::msg::TransformStamped t_inv;
            t_inv.header.stamp = this->get_clock()->now();
            t_inv.header.frame_id = new_source_frame;
            t_inv.child_frame_id = new_target_frame;
            t_inv.transform = tf2::toMsg(inverse_transform);

            tf_broadcaster_->sendTransform(t_inv);
        }
    }

    std::vector<std::string> camera_names_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TfInverterNode>());
    rclcpp::shutdown();
    return 0;
}
