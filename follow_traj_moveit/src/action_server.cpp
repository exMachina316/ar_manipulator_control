#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ur_interfaces/action/execute_way_points.hpp"
#include <moveit_msgs/msg/position_constraint.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

class PlanExecuteCartesianPathServer : public rclcpp::Node
{
public:
  using PlanExecuteCartesianPath = ur_interfaces::action::ExecuteWayPoints;
  using GoalHandlePlanExecute = rclcpp_action::ServerGoalHandle<PlanExecuteCartesianPath>;

  PlanExecuteCartesianPathServer() : Node("plan_execute_cartesian_path_server")
  {
    this->action_server_ = rclcpp_action::create_server<PlanExecuteCartesianPath>(
        this,
        "plan_execute_cartesian_path",
        std::bind(&PlanExecuteCartesianPathServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlanExecuteCartesianPathServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PlanExecuteCartesianPathServer::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Action server started: plan_execute_cartesian_path");
  }

private:
  rclcpp_action::Server<PlanExecuteCartesianPath>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const PlanExecuteCartesianPath::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to execute Cartesian path with %ld waypoints", goal->waypoints.poses.size());
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
    std::thread{std::bind(&PlanExecuteCartesianPathServer::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandlePlanExecute> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<PlanExecuteCartesianPath::Feedback>();
    auto result = std::make_shared<PlanExecuteCartesianPath::Result>();

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(this->shared_from_this(), "ur_on_rail_manipulator");

    move_group.setMaxVelocityScalingFactor(0.2);
    RCLCPP_INFO(this->get_logger(), "Planning Path to Start Pose");

    feedback->status = "Planning in Joint space";
    feedback->progress = 0.10;
    goal_handle->publish_feedback(feedback);

    std::vector<geometry_msgs::msg::Pose> waypoints = goal->waypoints.poses;
    moveit_msgs::msg::RobotTrajectory trajectory;

    move_group.setPoseTarget(waypoints[0]);
    move_group.setPlanningTime(10.0);

    auto const [success, plan] = [&move_group]
    {
      moveit::planning_interface::MoveGroupInterface::Plan msg;
      auto const ok = static_cast<bool>(move_group.plan(msg));
      return std::make_pair(ok, msg);
    }();

    feedback->status = "Exectuing Start Pose Plan";
    feedback->progress = 0.15;
    goal_handle->publish_feedback(feedback);

    if (success)
    {
      // Execute the trajectory if planning was successful
      if (move_group.execute(plan))
      {
        RCLCPP_INFO(this->get_logger(), "Trajectory executed successfully.");
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to execute the trajectory.");
        result->success = false;
        result->message = "Failed to execute the trajectory.";
        goal_handle->abort(result);
      }
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to plan a Start Pose manuver.");
      result->success = false;
      result->message = "Failed to plan a Start Pose manuver.";
      goal_handle->abort(result);
      return;
    }

    feedback->status = "Planning in Cartesian space";
    feedback->progress = 0.25;
    goal_handle->publish_feedback(feedback);

    moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
    const double jump_threshold = 0.0;    // Disable jumping
    const double eef_step = 0.01;         // Interpolation

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_trajectory);
    // int retries = 0;

    // while (fraction < 0.9 && retries < 10)
    // {
    //   RCLCPP_WARN(this->get_logger(), "Failed to find a Cartesian path %.2f success. Retrying with smaller step size.", fraction * 100);
    //   fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, cartesian_trajectory);
    //   retries++;
    //   // Wait for a bit before retrying
    //   std::this_thread::sleep_for(std::chrono::seconds(1));
    // }

    if (fraction < 0.9)
    {
      result->success = false;
      result->message = "Cartesian path planning failed with only " + std::to_string(fraction * 100) + "% success";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return;
    }

    feedback->status = "Executing Cartesian path";
    feedback->progress = 0.50;
    goal_handle->publish_feedback(feedback);

    if (!move_group.execute(cartesian_trajectory))
    {
      result->success = false;
      result->message = "Failed to execute Cartesian path";
      goal_handle->abort(result);
      RCLCPP_ERROR(this->get_logger(), result->message.c_str());
      return;
    }

    feedback->status = "Execution complete";
    feedback->progress = 1.0;
    goal_handle->publish_feedback(feedback);

    result->success = true;
    result->message = "Successfully executed Cartesian path";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), result->message.c_str());
  }
};

// RCLCPP_COMPONENTS_REGISTER_NODE(follow_traj_moveit::PlanExecuteCartesianPathServer)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanExecuteCartesianPathServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
