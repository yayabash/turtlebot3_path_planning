#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math
import time

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        
        # Try different service names for Gazebo Harmonic
        self.service_names = [
            '/set_model_state',
            '/gazebo/set_model_state'
        ]
        
        self.client = None
        for service_name in self.service_names:
            self.client = self.create_client(SetModelState, service_name)
            if self.client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(f'Connected to {service_name}')
                break
        
        if self.client is None or not self.client.service_is_ready():
            self.get_logger().error('Could not connect to Gazebo SetModelState service!')
            self.get_logger().info('Make sure:')
            self.get_logger().info('1. Gazebo is running')
            self.get_logger().info('2. You have the gazebo_ros_pkgs installed')
            return
        
        self.start_time = time.time()
        self.get_logger().info('Obstacle mover started successfully!')
        
        # Create timer for updates (10 Hz)
        self.timer = self.create_timer(0.1, self.update_obstacles)
    
    def update_obstacles(self):
        """Update all obstacle positions"""
        current_time = time.time() - self.start_time
        
        # Move obstacle_1 in a circle
        radius = 0.8
        speed = 0.5
        angular_speed = speed / radius
        angle = angular_speed * current_time
        
        self.move_obstacle('obstacle_1', 
                          1.0 + radius * math.cos(angle),
                          0.0 + radius * math.sin(angle),
                          0.25)
        
        # Move obstacle_2 in a smaller circle
        radius2 = 0.6
        speed2 = 0.3
        angle2 = (speed2 / radius2) * current_time
        
        self.move_obstacle('obstacle_2',
                          2.0 + radius2 * math.cos(angle2),
                          1.0 + radius2 * math.sin(angle2),
                          0.25)
        
        # Move obstacle_3 back and forth
        range_x = 0.8
        period = 8.0  # seconds for one full cycle
        pos_x = 1.5 + range_x * math.sin(2 * math.pi * current_time / period)
        
        self.move_obstacle('obstacle_3',
                          pos_x,
                          -1.0,
                          0.25)
    
    def move_obstacle(self, name, x, y, z):
        """Move a single obstacle to new position"""
        if not self.client or not self.client.service_is_ready():
            return
        
        # Create model state
        model_state = ModelState()
        model_state.model_name = name
        model_state.reference_frame = 'world'
        
        # Set position
        model_state.pose.position.x = float(x)
        model_state.pose.position.y = float(y)
        model_state.pose.position.z = float(z)
        
        # Keep orientation neutral
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0
        
        # Send request (async for performance)
        req = SetModelState.Request()
        req.model_state = model_state
        
        try:
            future = self.client.call_async(req)
            # Don't wait for response to avoid blocking
        except Exception as e:
            self.get_logger().debug(f'Error moving {name}: {str(e)[:50]}')

def main(args=None):
    rclpy.init(args=args)
    mover = ObstacleMover()
    
    # Only spin if we successfully connected
    if mover.client and mover.client.service_is_ready():
        try:
            rclpy.spin(mover)
        except KeyboardInterrupt:
            mover.get_logger().info('Obstacle mover stopped')
    else:
        mover.get_logger().error('Failed to start obstacle mover')
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
