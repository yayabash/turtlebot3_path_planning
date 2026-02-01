#!/usr/bin/env python3

import sys
import rclpy
from gazebo_msgs.srv import DeleteEntity

def main():
    rclpy.init()
    node = rclpy.create_node('delete_box_node')
    client = node.create_client(DeleteEntity, '/delete_entity')

    if not client.service_is_ready():
        node.get_logger().info("Waiting for /delete_entity service...")
        client.wait_for_service()

    name = sys.argv[1] if len(sys.argv) > 1 else "dynamic_box"

    request = DeleteEntity.Request()
    request.name = name

    node.get_logger().info(f"Deleting entity '{name}'...")
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
