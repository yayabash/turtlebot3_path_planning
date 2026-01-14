#! /usr/bin/env python3
import time
import math
import os
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import matplotlib.pyplot as plt
import csv
import threading

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        self.start_time = None
        self.path_x = []
        self.path_y = []
        self.velocities_v = []
        self.velocities_w = []
        self.timestamps = []
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.is_logging = False

    def start_logging(self):
        self.is_logging = True
        self.start_time = time.time()
        self.get_logger().info("Started logging metrics...")

    def stop_logging(self):
        self.is_logging = False
        self.get_logger().info("Stopped logging metrics.")

    def odom_callback(self, msg):
        if not self.is_logging:
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        
        current_time = time.time() - self.start_time

        if self.last_x is not None:
            dist = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2)
            self.total_distance += dist

        self.last_x = x
        self.last_y = y

        self.path_x.append(x)
        self.path_y.append(y)
        self.velocities_v.append(v)
        self.velocities_w.append(w)
        self.timestamps.append(current_time)

    def save_plots(self, filename_prefix):
        # 1. Trajectory Plot
        plt.figure(figsize=(10, 6))
        plt.plot(self.path_x, self.path_y, label='Actual Path', color='blue', linewidth=2)
        plt.title(f'Robot Trajectory - {filename_prefix}')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.grid(True)
        plt.axis('equal')
        plt.legend()
        plt.savefig(f'{filename_prefix}_trajectory.png')
        plt.close()

        # 2. Velocity Profile
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(self.timestamps, self.velocities_v, label='Linear Velocity (v)', color='green')
        plt.title(f'Velocity Profile - {filename_prefix}')
        plt.ylabel('v [m/s]')
        plt.grid(True)
        plt.legend()

        plt.subplot(2, 1, 2)
        plt.plot(self.timestamps, self.velocities_w, label='Angular Velocity (w)', color='red')
        plt.xlabel('Time [s]')
        plt.ylabel('w [rad/s]')
        plt.grid(True)
        plt.legend()
        
        plt.savefig(f'{filename_prefix}_velocity.png')
        plt.close()
        
        return self.total_distance, self.timestamps[-1] if self.timestamps else 0

def spawn_obstacle(node, x, y, name="test_box"):
    client = node.create_client(SpawnEntity, '/spawn_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Spawn service not available, waiting...')
    
    sdf_xml = f"""<?xml version='1.0'?>
    <sdf version="1.4">
    <model name="{name}">
      <pose>{x} {y} 0.25 0 0 0</pose>
      <static>true</static>
        <link name="link">
          <collision name="collision">
            <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
          </collision>
          <visual name="visual">
            <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
            <material><script><name>Gazebo/Red</name></script></material>
          </visual>
        </link>
      </model>
    </sdf>"""
    
    req = SpawnEntity.Request()
    req.name = name
    req.xml = sdf_xml
    future = client.call_async(req)
    # rclpy.spin_until_future_complete(node, future) # DO NOT SPIN, background thread processes it
    while not future.done():
        time.sleep(0.1)

def delete_obstacle(node, name="test_box"):
    client = node.create_client(DeleteEntity, '/delete_entity')
    while not client.wait_for_service(timeout_sec=1.0):
        pass
    req = DeleteEntity.Request()
    req.name = name
    future = client.call_async(req)
    # rclpy.spin_until_future_complete(node, future) # DO NOT SPIN
    while not future.done():
        time.sleep(0.1)

def main():
    rclpy.init()

    # Determine analysis directory relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    analysis_dir = os.path.join(script_dir, 'analysis')
    if not os.path.exists(analysis_dir):
        os.makedirs(analysis_dir)

    # Navigator setup
    navigator = BasicNavigator()
    
    # Logger node setup
    logger_node = MetricsLogger()
    
    # Run logger in separate thread so it can subscribe while navigator works
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(logger_node)
    spinner_thread = threading.Thread(target=executor.spin, daemon=True)
    spinner_thread.start()

    # --- Scenario 1: Static Environment ---
    print("--- Starting Scenario 1: Static Environment ---")
    
    # Set initial pose (approximate start)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -0.0 # Adjust to map start
    initial_pose.pose.position.y = -0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)
    
    # Wait for Nav2
    navigator.waitUntilNav2Active()

    # Set Goal
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0 # Adjust far goal
    goal_pose.pose.position.y = 8.0
    goal_pose.pose.orientation.w = 1.0

    logger_node.start_logging()
    navigator.goToPose(goal_pose)

    while not navigator.isTaskComplete():
        time.sleep(0.1)

    result = navigator.getResult()
    logger_node.stop_logging()
    
    dist_static, time_static = logger_node.save_plots(os.path.join(analysis_dir, "scenario_static"))
    print(f"Static Test: Distance={dist_static:.2f}m, Time={time_static:.2f}s")
    
    # Reset for next test
    # Ideally should reset robot pose, but for simplicity let's drive back or assume user resets simulation.
    # We will just carry on for now, assume robot is at goal.
    # Actually, let's reverse the path for Dynamic test to save reset time? 
    # Or navigate back to start without logging.
    
    print(" returning to start...")
    back_pose = initial_pose
    navigator.goToPose(back_pose)
    while not navigator.isTaskComplete():
        time.sleep(0.1)

    # Reset metrics
    logger_node.path_x = []
    logger_node.path_y = []
    logger_node.velocities_v = []
    logger_node.velocities_w = []
    logger_node.timestamps = []
    logger_node.total_distance = 0.0
    logger_node.last_x = None
    logger_node.last_y = None

    # --- Scenario 2: Dynamic Obstacle ---
    print("--- Starting Scenario 2: Dynamic Obstacle ---")
    
    logger_node.start_logging()
    navigator.goToPose(goal_pose)
    
    # Wait a bit then spawn obstacle
    start_t = time.time()
    spawned = False
    
    while not navigator.isTaskComplete():
        if not spawned and (time.time() - start_t > 5.0): # Spawn after 5 seconds
            spawn_obstacle(logger_node, 0.0, 0.0, "dynamic_box_test") # Spawn in middle
            print(">>> Obstacle Spawned! <<<")
            spawned = True
        time.sleep(0.1)

    logger_node.stop_logging()
    dist_dyn, time_dyn = logger_node.save_plots(os.path.join(analysis_dir, "scenario_dynamic"))
    print(f"Dynamic Test: Distance={dist_dyn:.2f}m, Time={time_dyn:.2f}s")
    
    delete_obstacle(logger_node, "dynamic_box_test")

    # Generate Report Table
    with open(os.path.join(analysis_dir, 'results_summary.csv'), 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['Scenario', 'Total Time (s)', 'Total Distance (m)'])
        writer.writerow(['Static', f'{time_static:.2f}', f'{dist_static:.2f}'])
        writer.writerow(['Dynamic', f'{time_dyn:.2f}', f'{dist_dyn:.2f}'])

    rclpy.shutdown()

if __name__ == '__main__':
    main()
