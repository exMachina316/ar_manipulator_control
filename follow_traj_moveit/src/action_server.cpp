#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ur_interfaces/action/execute_way_points_with_types.hpp"
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>
#include <string>

class PlanExecutePilzPathServer : public rclcpp::Node
{
public:
  using PlanExecutePilzPath = ur_interfaces::action::ExecuteWayPointsWithTypes;
  using GoalHandlePlanExecute = rclcpp_action::ServerGoalHandle<PlanExecutePilzPath>;

  PlanExecutePilzPathServer() : Node("plan_execute_pilz_path_server")
  {
    this->action_server_ = rclcpp_action::create_server<PlanExecutePilzPath>(
        this,
        "plan_execute_pilz_path",
        std::bind(&PlanExecutePilzPathServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlanExecutePilzPathServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PlanExecutePilzPathServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server started: plan_execute_pilz_path");
  }

private:
  rclcpp_action::Server<PlanExecutePilzPath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PlanExecutePilzPath::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to execute path with %ld waypoints and %ld trajectory types",
                goal->waypoints.poses.size(), goal->trajectory_types.size());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandlePlanExecute> goal_handle)
  {
    RCLCPP_WARN(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandlePlanExecute> goal_handle)
  {
    std::thread{std::bind(&PlanExecutePilzPathServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlanExecute> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanExecutePilzPath::Feedback>();
    auto result = std::make_shared<PlanExecutePilzPath::Result>();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(this->shared_from_this(), "ur_on_rail_manipulator");

    // Set Pilz Industrial Motion Planner as the planning pipeline
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlanningTime(10.0);

    std::vector<geometry_msgs::msg::Pose> waypoints = goal->waypoints.poses;
    std::vector<std::string> trajectory_types = goal->trajectory_types;
    std::vector<geometry_msgs::msg::Pose> interim_points = goal->interim_points;
    std::vector<int32_t> interim_point_indices = goal->interim_point_indices;

    if (waypoints.size() < 2)
    {
      result->success = false;
      result->message = "Need at least 2 waypoints to execute a trajectory";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return;
    }

    // Fill in default trajectory types if not provided
    size_t num_segments = waypoints.size() - 1;
    if (trajectory_types.size() < num_segments)
    {
      RCLCPP_WARN(this->get_logger(), "Trajectory types array is shorter than expected. Filling with PTP.");
      while (trajectory_types.size() < num_segments)
      {
        trajectory_types.push_back("PTP");
      }
    }

    // Initialize result arrays
    result->segment_success.resize(num_segments, false);
    result->segment_messages.resize(num_segments, "");

    RCLCPP_INFO(this->get_logger(), "Starting trajectory execution with %ld segments using Pilz planner", num_segments);

    // ====================
    // PHASE 1: PLANNING
    // ====================
    feedback->current_phase = "planning";
    feedback->status = "Planning all segments";
    feedback->progress = 0.0;
    feedback->current_segment = -1;
    goal_handle->publish_feedback(feedback);

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> segment_plans;
    segment_plans.reserve(num_segments);

    // Plan to first waypoint using PTP
    RCLCPP_INFO(this->get_logger(), "Planning move to first waypoint using PTP");
    move_group.setPlannerId("PTP");
    move_group.setPoseTarget(waypoints[0]);
    move_group.clearPathConstraints();

    auto const [start_success, start_plan] = [&move_group]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group.plan(msg));
      return std::make_pair(ok, msg);
    }();

    if (!start_success)
    {
      result->success = false;
      result->message = "Failed to plan move to first waypoint";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return;
    }

    segment_plans.push_back(start_plan);
    RCLCPP_INFO(this->get_logger(), "Successfully planned move to first waypoint");

    // Plan each segment
    for (size_t i = 0; i < num_segments; ++i)
    {
      feedback->current_segment = static_cast<int32_t>(i);
      feedback->status = "Planning segment " + std::to_string(i + 1) + "/" + std::to_string(num_segments);
      feedback->progress = 0.3f * (static_cast<float>(i) / static_cast<float>(num_segments));
      goal_handle->publish_feedback(feedback);

      std::string traj_type = trajectory_types[i];
      RCLCPP_INFO(this->get_logger(), "Planning segment %ld: waypoint %ld -> %ld using %s",
                  i + 1, i + 1, i + 2, traj_type.c_str());

      // Validate trajectory type
      if (traj_type != "PTP" && traj_type != "LIN" && traj_type != "CIRC")
      {
        RCLCPP_WARN(this->get_logger(), "Unknown trajectory type '%s', defaulting to PTP", traj_type.c_str());
        traj_type = "PTP";
      }

      move_group.setPlannerId(traj_type);
      move_group.setPoseTarget(waypoints[i + 1]);
      move_group.clearPathConstraints();

      // Handle CIRC trajectory with interim point
      if (traj_type == "CIRC")
      {
        // Find interim point for this segment
        auto it = std::find(interim_point_indices.begin(), interim_point_indices.end(), static_cast<int32_t>(i));
        if (it != interim_point_indices.end())
        {
          size_t interim_idx = std::distance(interim_point_indices.begin(), it);
          if (interim_idx < interim_points.size())
          {
            geometry_msgs::msg::Pose interim_pose = interim_points[interim_idx];
            RCLCPP_INFO(this->get_logger(), "Using interim point at [%.3f, %.3f, %.3f] for CIRC segment",
                        interim_pose.position.x, interim_pose.position.y, interim_pose.position.z);

            // Set path constraints for CIRC planner
            moveit_msgs::msg::Constraints constraints;
            constraints.name = "interim";  // Can use "center" for center of arc
            moveit_msgs::msg::PositionConstraint pos_constraint;
            pos_constraint.header.frame_id = "world";  // Assuming world frame
            pos_constraint.link_name = "tool0";
            pos_constraint.constraint_region.primitive_poses.push_back(interim_pose);
            pos_constraint.weight = 1.0;
            constraints.position_constraints.push_back(pos_constraint);
            move_group.setPathConstraints(constraints);
          }
          else
          {
            RCLCPP_ERROR(this->get_logger(), "Interim point index out of bounds for segment %ld", i);
            result->segment_success[i] = false;
            result->segment_messages[i] = "Interim point index out of bounds";
            result->success = false;
            result->message = "Failed at segment " + std::to_string(i + 1) + ": " + result->segment_messages[i];
            goal_handle->abort(result);
            return;
          }
        }
        else
        {
          RCLCPP_ERROR(this->get_logger(), "No interim point provided for CIRC segment %ld", i);
          result->segment_success[i] = false;
          result->segment_messages[i] = "No interim point for CIRC trajectory";
          result->success = false;
          result->message = "Failed at segment " + std::to_string(i + 1) + ": " + result->segment_messages[i];
          goal_handle->abort(result);
          return;
        }
      }

      // Plan the segment
      auto const [segment_success, segment_plan] = [&move_group]
      {
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group.plan(msg));
        return std::make_pair(ok, msg);
      }();

      if (!segment_success)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to plan segment %ld using %s", i + 1, traj_type.c_str());
        result->segment_success[i] = false;
        result->segment_messages[i] = "Planning failed for " + traj_type;
        result->success = false;
        result->message = "Failed at segment " + std::to_string(i + 1) + ": " + result->segment_messages[i];
        goal_handle->abort(result);
        return;
      }

      segment_plans.push_back(segment_plan);
      result->segment_success[i] = true;
      result->segment_messages[i] = "Planned successfully with " + traj_type;
      RCLCPP_INFO(this->get_logger(), "Successfully planned segment %ld with %s", i + 1, traj_type.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "All segments planned successfully");

    // ====================
    // PHASE 2: EXECUTION
    // ====================
    feedback->current_phase = "executing";
    feedback->status = "Executing all segments";
    feedback->progress = 0.3;
    feedback->current_segment = -1;
    goal_handle->publish_feedback(feedback);

    // Execute move to first waypoint
    RCLCPP_INFO(this->get_logger(), "Executing move to first waypoint");
    if (!move_group.execute(segment_plans[0]))
    {
      result->success = false;
      result->message = "Failed to execute move to first waypoint";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return;
    }

    // Execute each segment
    for (size_t i = 0; i < num_segments; ++i)
    {
      feedback->current_segment = static_cast<int32_t>(i);
      feedback->status = "Executing segment " + std::to_string(i + 1) + "/" + std::to_string(num_segments);
      feedback->progress = 0.3f + 0.5f * (static_cast<float>(i) / static_cast<float>(num_segments));
      goal_handle->publish_feedback(feedback);

      RCLCPP_INFO(this->get_logger(), "Executing segment %ld with %s", i + 1, trajectory_types[i].c_str());

      if (!move_group.execute(segment_plans[i + 1]))
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute segment %ld", i + 1);
        result->segment_success[i] = false;
        result->segment_messages[i] = "Execution failed";
        result->success = false;
        result->message = "Failed to execute segment " + std::to_string(i + 1);
        goal_handle->abort(result);
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Successfully executed segment %ld", i + 1);
    }

    RCLCPP_INFO(this->get_logger(), "All segments executed successfully");

    // ====================
    // PHASE 3: VERIFICATION
    // ====================
    feedback->current_phase = "verifying";
    feedback->status = "Verifying final position";
    feedback->progress = 0.9;
    feedback->current_segment = -1;
    goal_handle->publish_feedback(feedback);

    // Get current pose and compare with final waypoint
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    geometry_msgs::msg::Pose target_pose = waypoints.back();

    double position_error = std::sqrt(
        std::pow(current_pose.pose.position.x - target_pose.position.x, 2) +
        std::pow(current_pose.pose.position.y - target_pose.position.y, 2) +
        std::pow(current_pose.pose.position.z - target_pose.position.z, 2));

    RCLCPP_INFO(this->get_logger(), "Final position error: %.4f m", position_error);

    const double position_tolerance = 0.01;  // 1 cm tolerance
    if (position_error > position_tolerance)
    {
      RCLCPP_WARN(this->get_logger(), "Position error (%.4f m) exceeds tolerance (%.4f m)",
                  position_error, position_tolerance);
    }

    // Success!
    feedback->status = "Trajectory execution complete";
    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Successfully executed all " + std::to_string(num_segments) + 
                      " segments with position error: " + std::to_string(position_error) + " m";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), result->message.c_str());
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanExecutePilzPathServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
