#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Spawn dynamic obstacles using Gazebo ACTORS.
Actors support built-in animations and ALWAYS move in Gazebo Classic.
No ROS services required for movement.
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time
import sys
from textwrap import dedent

# name, x_start, x_end, y, z, period_seconds
OBSTACLES = [
    ('obstacle_1', 0.4, 1.6, 0.0, 0.20, 6.0),
    ('obstacle_2', 1.4, 2.6, 1.0, 0.20, 8.0),
    ('obstacle_3', 0.7, 2.3, -1.0, 0.20, 5.0),
]


def build_actor_sdf(name, x0, x1, y, z, period):
    """
    Create SDF actor with a linear looping trajectory.
    Uses skinless actor (simple box visual).
    """

    sdf = dedent(f"""
    <?xml version="1.0" ?>
    <sdf version="1.7">
      <actor name="{name}">
        <pose>{x0} {y} {z} 0 0 0</pose>

        <skin>
          <filename>__default__</filename>
        </skin>

        <script>
          <loop>true</loop>
          <delay_start>0.0</delay_start>
          <auto_start>true</auto_start>
          <trajectory id="0" type="linear">
            <waypoint>
              <time>0</time>
              <pose>{x0} {y} {z} 0 0 0</pose>
            </waypoint>
            <waypoint>
              <time>{period/2}</time>
              <pose>{x1} {y} {z} 0 0 0</pose>
            </waypoint>
            <waypoint>
              <time>{period}</time>
              <pose>{x0} {y} {z} 0 0 0</pose>
            </waypoint>
          </trajectory>
        </script>

        <visual name="visual">
          <geometry>
            <box>
              <size>0.35 0.35 0.5</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>

      </actor>
    </sdf>
    """)

    return sdf


class ActorSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_manager')
        self.get_logger().info("Actor-based obstacle spawner starting...")

        # connect to Gazebo spawn_entity
        client = self.create_client(SpawnEntity, '/spawn_entity')
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Cannot find /spawn_entity service.")
            self.spawn_client = None
        else:
            self.spawn_client = client
            self.get_logger().info("Connected to /spawn_entity")

    def spawn_actor(self, name, sdf, x, y, z):
        req = SpawnEntity.Request()
        req.name = name
        req.xml = sdf

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        req.initial_pose = pose

        fut = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=4.0)

        self.get_logger().info(f"Spawned actor: {name}")

    def spawn_all(self):
        for (name, x0, x1, y, z, period) in OBSTACLES:
            sdf = build_actor_sdf(name, x0, x1, y, z, period)
            self.spawn_actor(name, sdf, x0, y, z)
            time.sleep(0.1)

        self.get_logger().info("All actors spawned. Movement will continue forever.")


def main(args=None):
    rclpy.init(args=args)
    node = ActorSpawner()
    if node.spawn_client:
        node.spawn_all()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

