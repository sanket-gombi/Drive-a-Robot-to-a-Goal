#!/usr/bin/env python3
"""
Navigate to Goal Node.

A ROS 2 Python node commanding autonomous navigation to a target pose
using the Nav2 NavigateToPose action client.

Author: Sanket
Task: Drive a Robot to a Goal (ROS 2 + Nav2)
"""

import math
import sys

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class Nav2GoalNavigator(Node):
    """ROS 2 Node managing communication with Nav2's NavigateToPose action server."""

    def __init__(self):
        """Initialize parameters, action client, and publisher."""
        super().__init__('nav2_goal_navigator')

        # Declare configurable parameters with sensible default values
        self.declare_parameter('goal_x', 1.8)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('server_timeout_sec', 10.0)
        self.declare_parameter('publish_initial_pose', False)
        self.declare_parameter('initial_x', -2.0)
        self.declare_parameter('initial_y', -0.5)
        self.declare_parameter('initial_yaw', 0.0)

        # Retrieve parameter values
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_yaw = self.get_parameter('goal_yaw').get_parameter_value().double_value
        timeout_param = self.get_parameter('server_timeout_sec').get_parameter_value()
        self.timeout_sec = timeout_param.double_value
        init_param = self.get_parameter('publish_initial_pose').get_parameter_value()
        self.publish_init = init_param.bool_value

        # Status tracking flags
        self.navigation_done = False
        self.navigation_success = False
        self._goal_handle = None

        # Create the action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Optional initial pose publisher (for headless/automated testing)
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.get_logger().info('Nav2GoalNavigator node initialized.')

    @staticmethod
    def quaternion_from_yaw(yaw: float):
        """
        Convert a planar rotation angle (yaw in radians) into a normalized quaternion.

        For 2D planar navigation: roll=0, pitch=0.
        """
        half_yaw = yaw * 0.5
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)
        return 0.0, 0.0, qz, qw

    def publish_initial_pose(self):
        """Publish initial pose estimate to /initialpose to localize the robot in AMCL."""
        init_x = self.get_parameter('initial_x').get_parameter_value().double_value
        init_y = self.get_parameter('initial_y').get_parameter_value().double_value
        init_yaw = self.get_parameter('initial_yaw').get_parameter_value().double_value
        _, _, qz, qw = self.quaternion_from_yaw(init_yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = init_x
        msg.pose.pose.position.y = init_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = 0.25   # x covariance
        msg.pose.covariance[7] = 0.25   # y covariance
        msg.pose.covariance[35] = 0.06  # yaw covariance

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f'Published initial pose: x={init_x:.2f}, y={init_y:.2f}, yaw={init_yaw:.2f} rad'
        )

    def send_navigation_goal(self):
        """Wait for the Nav2 action server, construct the PoseStamped goal, and send it."""
        self.get_logger().info(
            f'Waiting up to {self.timeout_sec:.1f}s for /navigate_to_pose action server...'
        )
        server_ready = self._action_client.wait_for_server(timeout_sec=self.timeout_sec)
        if not server_ready:
            self.get_logger().error('Action server /navigate_to_pose timed out or unavailable!')
            print('GOAL FAILED (status: SERVER_TIMEOUT)')
            self.navigation_done = True
            self.navigation_success = False
            return False

        # If flag is set, publish initial pose before sending goal
        if self.publish_init:
            self.publish_initial_pose()

        # Build goal PoseStamped in the 'map' frame
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        # Target position
        goal_msg.pose.pose.position.x = self.goal_x
        goal_msg.pose.pose.position.y = self.goal_y
        goal_msg.pose.pose.position.z = 0.0

        # Target orientation from yaw
        _, _, qz, qw = self.quaternion_from_yaw(self.goal_yaw)
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f'Sending goal -> x: {self.goal_x:.2f}, y: {self.goal_y:.2f}, '
            f'yaw: {self.goal_yaw:.2f} rad'
        )

        # Send goal asynchronously with feedback callback
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def feedback_callback(self, feedback_msg):
        """Receive real-time progress feedback from the navigation action server."""
        feedback = feedback_msg.feedback
        dist_remaining = feedback.distance_remaining
        time_elapsed = feedback.navigation_time.sec

        self.get_logger().info(
            f'[FEEDBACK] Distance remaining: {dist_remaining:.2f} m | '
            f'Time elapsed: {time_elapsed} s'
        )

    def goal_response_callback(self, future):
        """Handle server acceptance or rejection of the navigation goal."""
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the Nav2 action server!')
            print('GOAL FAILED (status: REJECTED)')
            self.navigation_done = True
            self.navigation_success = False
            return

        self.get_logger().info('Goal accepted by Nav2. Robot is navigating...')
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle the final action result and status upon goal completion or failure."""
        status = future.result().status

        # Status code mappings from action_msgs/msg/GoalStatus
        status_names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        status_str = status_names.get(status, f'STATUS_CODE_{status}')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Navigation finished with status: {status_str}')
            print('GOAL SUCCEEDED')
            self.navigation_success = True
        else:
            self.get_logger().warn(f'Navigation failed with status: {status_str}')
            print(f'GOAL FAILED (status: {status_str})')
            self.navigation_success = False

        self.navigation_done = True


def main(args=None):
    """Execute main ROS 2 node life cycle and spin until navigation completes."""
    rclpy.init(args=args)
    navigator = Nav2GoalNavigator()

    try:
        started = navigator.send_navigation_goal()
        if started:
            while rclpy.ok() and not navigator.navigation_done:
                rclpy.spin_once(navigator, timeout_sec=0.1)

    except KeyboardInterrupt:
        navigator.get_logger().warn('Ctrl+C detected! Canceling navigation goal...')
        if navigator._goal_handle is not None:
            cancel_future = navigator._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(navigator, cancel_future, timeout_sec=2.0)
        print('GOAL FAILED (status: INTERRUPTED)')
        navigator.navigation_success = False

    finally:
        success = navigator.navigation_success
        navigator.destroy_node()
        rclpy.shutdown()
        # Return exit code 0 on success, 1 on failure
        sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
