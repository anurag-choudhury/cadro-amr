#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import math
from time import sleep
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


qos_profile = QoSProfile(
    depth=100,  # Queue size
    durability=DurabilityPolicy.TRANSIENT_LOCAL
)

class CoveragePathPublisher(Node):
    def __init__(self):
        super().__init__('coverage_path_publisher')
        self.get_logger().info('Initializing CoveragePathPublisher Node...')

        self.declare_parameter('coverage_width', 1.0)
        self.waypoints = []
        self.current_waypoint_index = 0
        self.map_received = False
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, qos_profile)
        self.get_logger().info('Subscribed to /map topic.')

        # Create a publisher for the path
        self.path_pub = self.create_publisher(Path, '/coverage_path', 10)

    def publish_coverage_path(self, waypoints):
        path = Path()
        path.header.frame_id = "map"  # Set the frame ID to match your map or odom
        path.header.stamp = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0  # Assuming a flat map
            pose.pose.orientation.w = 1.0  # Identity quaternion (no rotation)
            path.poses.append(pose)

        # Publish the path
        self.path_pub.publish(path)
        self.get_logger().info("Published coverage path")

    def map_callback(self, msg):
        print("inside map callback")
        self.get_logger().info('Received map data.')

        if not self.map_received:
            self.generate_parallel_waypoints(msg)
            self.map_received = True

            # After generating waypoints, publish them for visualization
            waypoints = [(wp.pose.position.x, wp.pose.position.y) for wp in self.waypoints]
            self.publish_coverage_path(waypoints)  # Visualize waypoints in RViz

            self.send_next_goal()

    def generate_parallel_waypoints(self, map_msg):
        self.get_logger().info('Generating waypoints...')
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin = map_msg.info.origin.position

        strip_width = self.get_parameter('coverage_width').get_parameter_value().double_value
        num_strips = int(math.ceil((width * resolution) / strip_width))

        for i in range(num_strips):
            x_start = origin.x + i * strip_width
            y_start = origin.y
            x_end = x_start
            y_end = origin.y + height * resolution

            waypoint1 = PoseStamped()
            waypoint1.header.frame_id = 'map'
            waypoint1.pose.position.x = x_start
            waypoint1.pose.position.y = y_start
            waypoint1.pose.orientation.w = 1.0

            waypoint2 = PoseStamped()
            waypoint2.header.frame_id = 'map'
            waypoint2.pose.position.x = x_end
            waypoint2.pose.position.y = y_end
            waypoint2.pose.orientation.w = 1.0

            if i % 2 == 1:
                waypoint1.pose.position.y = y_end
                waypoint2.pose.position.y = y_start

            self.waypoints.append(waypoint1)
            self.waypoints.append(waypoint2)

        self.get_logger().info(f'Generated {len(self.waypoints)} waypoints.')

    def send_next_goal(self):
        if self.current_waypoint_index < len(self.waypoints):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = self.waypoints[self.current_waypoint_index]

            self.get_logger().info(f'Sending waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                                   f'({goal_msg.pose.pose.position.x}, {goal_msg.pose.pose.position.y})')
            self.nav_action_client.wait_for_server()
            self._send_goal_future = self.nav_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info('All waypoints have been covered.')

    def feedback_callback(self, feedback):
        pass
        # self.get_logger().info(f'Navigating...')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Reached waypoint {self.current_waypoint_index + 1} with result: {result}')
        self.current_waypoint_index += 1
        sleep(1)
        self.send_next_goal()


def main(args=None):
    rclpy.init(args=args)
    node = CoveragePathPublisher()
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
