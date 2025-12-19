#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <string>
#include <fstream>
#include <map>
#include <sstream>

using namespace std;

struct TransformAccumulator
{
    tf2::Vector3 translation_sum{0, 0, 0};
    tf2::Quaternion rotation_sum{0, 0, 0, 0};
    int count = 0;
};

class TfInverterNode : public rclcpp::Node
{
public:
    TfInverterNode()
        : Node("extrinsics_calibrator")
    {
        this->declare_parameter<std::vector<std::string>>("camera_names", {"oak1", "oak2", "oak3"});
        this->get_parameter("camera_names", camera_names_);

        RCLCPP_INFO(this->get_logger(), "Extrinsics Calibrator Node Initialized");

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        lookup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TfInverterNode::on_lookup_timer, this));

        log_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&TfInverterNode::on_log_timer, this));
    }

private:
    void on_lookup_timer()
    {
        for (const auto& camera_name : camera_names_)
        {
            std::string source_frame = "base_link";
            std::string target_frame = camera_name;

            geometry_msgs::msg::TransformStamped t_stamped;

            try
            {
                t_stamped = tf_buffer_->lookupTransform(
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
            tf2::fromMsg(t_stamped.transform, transform);

            auto& accumulator = transform_accumulators_[camera_name];
            accumulator.translation_sum += transform.getOrigin();
            accumulator.rotation_sum += transform.getRotation();
            accumulator.count++;
        }
    }

    void on_log_timer()
    {
        for (const auto& pair : transform_accumulators_)
        {
            const auto& camera_name = pair.first;
            const auto& accumulator = pair.second;

            if (accumulator.count == 0)
            {
                continue;
            }

            tf2::Vector3 avg_translation = accumulator.translation_sum / accumulator.count;
            tf2::Quaternion avg_rotation = accumulator.rotation_sum.normalized();

            RCLCPP_INFO(this->get_logger(),
                "Average transform for %s (samples: %d): "
                "Translation: [%.3f, %.3f, %.3f], "
                "Rotation: [%.3f, %.3f, %.3f, %.3f]",
                camera_name.c_str(),
                accumulator.count,
                avg_translation.x(), avg_translation.y(), avg_translation.z(),
                avg_rotation.x(), avg_rotation.y(), avg_rotation.z(), avg_rotation.w());
        }
    }

    std::vector<std::string> camera_names_;
    std::map<std::string, TransformAccumulator> transform_accumulators_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr lookup_timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TfInverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

