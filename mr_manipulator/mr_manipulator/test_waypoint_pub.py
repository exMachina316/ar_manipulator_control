import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from std_srvs.srv import Trigger
import numpy as np

class WaypointPublisherNode(Node):
    def __init__(self):
        super().__init__('waypoint_publisher_node')

        # Create ROS 2 publisher for waypoints as PoseArray
        self.waypoints_publisher = self.create_publisher(PoseArray, 'waypoints_transformed', 10)

        # Create ROS 2 client for executing waypoints
        self.execute_client = self.create_client(Trigger, 'execute_waypoints')

        # Initialize waypoints
        self.waypoints = []

        # Timer to publish waypoints periodically
        self.create_timer(5.0, self.publish_waypoints)

    def publish_waypoints(self):
        pose_array_msg = PoseArray()
        pose_array_msg.header.stamp = self.get_clock().now().to_msg()
        pose_array_msg.header.frame_id = 'world'

        for x in np.arange(-0.2, 0.2, 0.001):
            pose = Pose()
            pose.position.x = x
            pose.position.y = 0.3
            pose.position.z = 1.0
            pose.orientation.w = 1.0
            pose_array_msg.poses.append(pose)
        
        self.waypoints_publisher.publish(pose_array_msg)
        self.get_logger().info(f'Published {len(pose_array_msg.poses)} waypoints.')

        # Call the trigger service
        if self.execute_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            future = self.execute_client.call_async(request)
            future.add_done_callback(self.execute_callback)
        else:
            self.get_logger().error('Service not available, cannot call trigger.')

    def execute_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()