import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_srvs.srv import Trigger
from ur_interfaces.action import ExecuteWayPoints

class ExecuteWayPointsClient(Node):
    def __init__(self):
        super().__init__('execute_waypoints_client')

        self.waypoints = None
        self.servicing = False
        
        # Create a subscription 
        sensor_qos = rclpy.qos.qos_profile_sensor_data
        self._waypoint_sub = self.create_subscription(
            PoseArray,
            'waypoints_transformed',
            self.waypoint_callback,
            sensor_qos
        )

        # Create service
        self.srv = self.create_service(
            Trigger,
            'execute_waypoints',
            self.execute_waypoints_callback
        )

        # Initialize Action Client
        self._action_client = ActionClient(self, ExecuteWayPoints, 'plan_execute_cartesian_path')

    def execute_waypoints_callback(self, request, response):
        """Service callback to execute stored waypoints"""
        if self.waypoints is None or len(self.waypoints.poses) == 0:
            response.success = False
            response.message = "No waypoints available to execute"
            return response

        elif self.servicing:
            response.success = False
            response.message = "Already executing waypoints"
            return response

        self.send_goal(self.waypoints)
        response.success = True
        self.servicing = True
        response.message = f"Executing {len(self.waypoints.poses)} waypoints"
        return response

    def waypoint_callback(self, msg):
        self.get_logger().debug(f"Received {len(msg.poses)} waypoints")
        self.waypoints = msg

    def send_goal(self, msg):
        goal_msg = ExecuteWayPoints.Goal()
        goal_msg.waypoints = msg

        self.get_logger().info(f"Sending goal with {len(goal_msg.waypoints.poses)} waypoints...")

        # Send goal to the action server
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the server')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.status} | Progress: {feedback.progress * 100:.2f}%")

    def result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info(f"Goal succeeded! Message: {result.message}")
        else:
            self.get_logger().error(f"Goal failed: {result.message}")
        
        self.servicing = False


def main(args=None):
    rclpy.init(args=args)

    client = ExecuteWayPointsClient()

    rclpy.spin(client)


if __name__ == '__main__':
    main()
