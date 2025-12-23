# TurtleBot3 Path Planning Demo

This project demonstrates path planning for TurtleBot3 in simulation using the A* and DWA (Dynamic Window Approach) algorithms. The demo uses ROS 2 and Gazebo, with a custom map and navigation parameters.

## Features

- **A\***: Used for global path planning.
- **DWA**: Used for local path planning and obstacle avoidance.
- Custom map and navigation parameters for TurtleBot3.
- Launch files for easy simulation and navigation startup.

## Requirements

- **Ubuntu**: 22.04
- **ROS 2**:  humbel
- **Python**: 3.10+
- **Gazebo**: 11+
- **TurtleBot3 packages**: You must have the official TurtleBot3 packages installed on your system (not included in this repo).
	- `turtlebot3`
	- `turtlebot3_msgs`
	- `turtlebot3_navigation2`
	- `turtlebot3_gazebo`
- **Other dependencies** (install via apt if missing):
	- `ros-foxy-nav2-bringup`
	- `ros-foxy-gazebo-ros-pkgs`
	- `ros-foxy-robot-state-publisher`
	- `ros-foxy-map-server`
	- `ros-foxy-amcl`
	- `ros-foxy-dwb-plugins`

## How It Works

- The simulation is launched in Gazebo with a custom world.
- The navigation stack uses A* for global planning and DWA for local planning.
- The robot navigates from a start to a goal position using the provided map and parameters.

## Usage

1. **Build the workspace:**
	```bash
	cd ~/turtlebot3_ws
	colcon build
	source install/setup.bash
	```

2. **Launch the simulation (Terminal 1):**
	```bash
	ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py use_sim_time:=true
	```

3. **Launch navigation with custom map and parameters (Terminal 2):**
	```bash
	ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=/absolute/path/to/maps/custom_map.yaml param:=/absolute/path/to/turtlebot3_navigation2/param/waffle_astar.yaml use_sim_time:=true
	```

	Replace `/absolute/path/to/` with the actual path to your workspace.

4. **Set an initial pose and goal in RViz to start navigation.**

## Project Structure

- `turtlebot3_gazebo/`: Gazebo simulation files and launch scripts.
- `turtlebot3_navigation2/`: Navigation launch and parameter files (A* and DWA).
- `maps/`: Custom map files (YAML and PGM).

## Notes

- This repo does not include the full TurtleBot3 packages. Install them from the official sources.
- The navigation parameters are tuned for the provided map and simulation environment.
