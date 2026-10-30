#!/usr/bin/env python3
"""
Dynamic Obstacle Spawner / Remover for Gazebo Classic.

Spawns or removes an unmapped obstacle in Gazebo to demonstrate
Nav2 local costmap marking and dynamic path replanning.

Author: Sanket
Task: Drive a Robot to a Goal (ROS 2 + Nav2)
"""

import argparse
import os
import sys

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
import rclpy
from rclpy.node import Node


class ObstacleManager(Node):
    """ROS 2 Node managing Gazebo entity spawning and deletion."""

    def __init__(self):
        super().__init__('obstacle_manager')
        self._spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self._delete_client = self.create_client(DeleteEntity, '/delete_entity')

    def spawn(self, name: str, sdf_path: str, x: float, y: float, z: float = 0.0):
        """Spawn an obstacle model into Gazebo."""
        self.get_logger().info(f'Waiting for /spawn_entity service...')
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity service not available!')
            return False

        if not os.path.isfile(sdf_path):
            self.get_logger().error(f'SDF file not found: {sdf_path}')
            return False

        with open(sdf_path, 'r', encoding='utf-8') as f:
            xml_content = f.read()

        request = SpawnEntity.Request()
        request.name = name
        request.xml = xml_content
        request.initial_pose.position.x = float(x)
        request.initial_pose.position.y = float(y)
        request.initial_pose.position.z = float(z)

        self.get_logger().info(f'Spawning entity "{name}" at x={x:.2f}, y={y:.2f}, z={z:.2f}...')
        future = self._spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        response = future.result()
        if response and response.success:
            self.get_logger().info(f'Successfully spawned "{name}": {response.status_message}')
            return True
        else:
            msg = response.status_message if response else 'Call timed out'
            self.get_logger().error(f'Failed to spawn "{name}": {msg}')
            return False

    def remove(self, name: str):
        """Remove an entity from Gazebo."""
        self.get_logger().info(f'Waiting for /delete_entity service...')
        if not self._delete_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity service not available!')
            return False

        request = DeleteEntity.Request()
        request.name = name

        self.get_logger().info(f'Deleting entity "{name}"...')
        future = self._delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        response = future.result()
        if response and response.success:
            self.get_logger().info(f'Successfully deleted "{name}": {response.status_message}')
            return True
        else:
            msg = response.status_message if response else 'Call timed out'
            self.get_logger().error(f'Failed to delete "{name}": {msg}')
            return False


def main():
    parser = argparse.ArgumentParser(description='Spawn or remove dynamic obstacle in Gazebo')
    subparsers = parser.add_subparsers(dest='action', required=True)

    # Spawn command
    spawn_cmd = subparsers.add_parser('spawn', help='Spawn an obstacle')
    spawn_cmd.add_argument('--name', default='dynamic_box', help='Model entity name')
    spawn_cmd.add_argument('--x', type=float, default=1.0, help='X coordinate in map frame')
    spawn_cmd.add_argument('--y', type=float, default=0.0, help='Y coordinate in map frame')
    spawn_cmd.add_argument('--z', type=float, default=0.0, help='Z coordinate in map frame')
    spawn_cmd.add_argument('--sdf', default='', help='Path to SDF file')

    # Remove command
    remove_cmd = subparsers.add_parser('remove', help='Remove an obstacle')
    remove_cmd.add_argument('--name', default='dynamic_box', help='Model entity name')

    args = parser.parse_args()

    rclpy.init()
    node = ObstacleManager()

    success = False
    try:
        if args.action == 'spawn':
            sdf_path = args.sdf
            if not sdf_path:
                # Default to package model path
                pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
                sdf_path = os.path.join(pkg_dir, 'models', 'obstacle_box', 'model.sdf')
            success = node.spawn(args.name, sdf_path, args.x, args.y, args.z)
        elif args.action == 'remove':
            success = node.remove(args.name)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
