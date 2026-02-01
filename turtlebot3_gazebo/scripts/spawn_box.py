#!/usr/bin/env python3

import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity

def main():
    rclpy.init()
    node = rclpy.create_node('spawn_box_node')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    if not client.service_is_ready():
        node.get_logger().info("Waiting for /spawn_entity service...")
        client.wait_for_service()

    # Get coordinates from command line or use defaults
    x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
    y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    name = sys.argv[3] if len(sys.argv) > 3 else "dynamic_box"

    sdf_xml = f"""
    <?xml version='1.0'?>
    <sdf version="1.4">
    <model name="{name}">
      <pose>{x} {y} 0.25 0 0 0</pose>
      <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry>
              <box>
                <size>2.0 2.0 0.5</size>
              </box>
            </geometry>
          </collision>
          <visual name="visual">
            <geometry>
              <box>
                <size>2.0 2.0 0.5</size>
              </box>
            </geometry>
            <material>
              <script>
                <uri>file://media/materials/scripts/gazebo.material</uri>
                <name>Gazebo/Red</name>
              </script>
            </material>
          </visual>
        </link>
      </model>
    </sdf>
    """

    request = SpawnEntity.Request()
    request.name = name
    request.xml = sdf_xml
    # request.robot_namespace = "box_ns"
    # request.initial_pose = ... # Already in SDF

    node.get_logger().info(f"Spawning box '{name}' at ({x}, {y})...")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(f"Result: {future.result().status_message}")
    else:
        node.get_logger().error("Service call failed")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
