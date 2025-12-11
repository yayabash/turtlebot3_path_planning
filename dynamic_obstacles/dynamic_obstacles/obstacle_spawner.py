#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import math
import time

class ObstacleSpawner(Node):
    def __init__(self):
        super().__init__('obstacle_spawner')
        
        # Create client for spawning
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        
        # Try multiple service names (Gazebo Harmonic uses different naming)
        self.service_names = [
            '/spawn_entity',
            '/gazebo/spawn_entity'
        ]
        
        # Wait for service (with retry logic)
        service_ready = False
        for service_name in self.service_names:
            self.get_logger().info(f'Trying service: {service_name}')
            client = self.create_client(SpawnEntity, service_name)
            if client.wait_for_service(timeout_sec=2.0):
                self.spawn_client = client
                self.get_logger().info(f'Connected to service: {service_name}')
                service_ready = True
                break
        
        if not service_ready:
            self.get_logger().warn('Gazebo spawn service not available. Make sure Gazebo is running.')
            return
        
        # Spawn obstacles
        self.spawn_obstacles()
    
    def spawn_box(self, name, x, y, z, size=0.3, color='Red'):
        """Create SDF for a simple box"""
        sdf = f"""<?xml version="1.0"?>
<sdf version="1.7">
  <model name="{name}">
    <pose>{x} {y} {z} 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{size} {size} 0.5</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size} {size} 0.5</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""
        
        return sdf
    
    def spawn_obstacles(self):
        """Spawn 3 obstacles with different paths"""
        obstacles = [
            {'name': 'obstacle_1', 'x': 1.0, 'y': 0.0, 'z': 0.25, 'color': 'Red'},
            {'name': 'obstacle_2', 'x': 2.0, 'y': 1.0, 'z': 0.25, 'color': 'Green'},
            {'name': 'obstacle_3', 'x': 1.5, 'y': -1.0, 'z': 0.25, 'color': 'Blue'}
        ]
        
        for obs in obstacles:
            sdf = self.spawn_box(obs['name'], obs['x'], obs['y'], obs['z'])
            
            req = SpawnEntity.Request()
            req.name = obs['name']
            req.xml = sdf
            req.initial_pose = Pose()
            req.initial_pose.position.x = obs['x']
            req.initial_pose.position.y = obs['y']
            req.initial_pose.position.z = obs['z']
            
            future = self.spawn_client.call_async(req)
            self.get_logger().info(f'Spawning {obs["name"]}...')
            
        self.get_logger().info('All obstacles spawned. Shutting down spawner.')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    spawner = ObstacleSpawner()
    rclpy.spin(spawner)

if __name__ == '__main__':
    main()
