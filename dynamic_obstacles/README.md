# Dynamic Obstacles for TurtleBot3 Navigation

A ROS2 package for spawning and controlling moving obstacles in Gazebo simulation. Designed to test navigation algorithms (like A* path planning) in dynamic environments with the TurtleBot3.

## Features

- **Obstacle Spawning:** Dynamically spawn box-shaped obstacles in Gazebo at runtime
- **Multiple Trajectory Types:**
  - **Circular:** Obstacles move in circles at specified speed and radius
  - **Linear:** Obstacles move back-and-forth along a line
- **ROS2 Native:** Full ROS2 integration using Python nodes
- **Configurable:** Easy to customize obstacle count, size, speed, color, and trajectories
- **Real-time Updates:** Obstacles update position at 10 Hz for smooth motion

## Architecture

### Nodes

1. **obstacle_spawner** (`obstacle_spawner.py`)
   - Spawns obstacles into Gazebo using `/spawn_entity` service
   - Creates box models with configurable dimensions, mass, and color
   - Manages list of spawned obstacles

2. **obstacle_controller** (`obstacle_controller.py`)
   - Reads obstacle trajectory configurations
   - Updates obstacle positions using `/gazebo/set_model_state` service
   - Supports circular and linear motion patterns

## Installation & Build

1. Clone/update your workspace:
```bash
cd ~/turtlebot3_ws/src
# Package should already be created: dynamic_obstacles/
```

2. Build the package:
```bash
cd ~/turtlebot3_ws
colcon build --packages-select dynamic_obstacles
source install/setup.bash
```

3. Verify build:
```bash
ros2 pkg list | grep dynamic_obstacles
ros2 run dynamic_obstacles obstacle_spawner --help  # Should not error
```

## Usage

### Option 1: Launch Both Nodes Together (Recommended)

```bash
# Terminal 1: Start Gazebo with TurtleBot3
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py

# Terminal 2: Start Nav2 stack
source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:=/home/yaxye2200/maps/custom_map.yaml \
  params_file:=/home/yaxye2200/turtlebot3_ws/src/turtlebot3_navigation2/param/waffle_astar.yaml

# Terminal 3: Spawn and control obstacles
source ~/turtlebot3_ws/install/setup.bash
ros2 launch dynamic_obstacles dynamic_obstacles.launch.py
```

### Option 2: Run Nodes Separately

```bash
# Spawn obstacles first
ros2 run dynamic_obstacles obstacle_spawner

# In another terminal, start controlling them
ros2 run dynamic_obstacles obstacle_controller
```

## Customization

### Modifying Default Obstacles

Edit `obstacle_spawner.py` `main()` function to change spawn parameters:

```python
# In main() function:
spawner.spawn_box(
    name='obstacle_1',
    x=1.0, y=0.0, z=0.25,           # Position (z=height above ground)
    size_x=0.3, size_y=0.3, size_z=0.5,  # Dimensions
    mass=1.0,                         # Mass in kg
    color=[1.0, 0.0, 0.0]            # RGB color (red)
)
```

### Modifying Obstacle Trajectories

Edit `obstacle_controller.py` `main()` function:

```python
# Circular motion: moves in a circle
controller.add_circular_trajectory(
    obstacle_name='obstacle_1',
    center_x=1.0, center_y=0.0,      # Center of circle
    radius=0.8,                       # Radius in meters
    speed=0.5,                        # Linear speed in m/s
    start_z=0.25                      # Height (meters)
)

# Linear motion: moves back-and-forth
controller.add_linear_trajectory(
    obstacle_name='obstacle_3',
    start_x=1.5, start_y=-1.5,       # Start position
    end_x=1.5, end_y=1.5,            # End position
    speed=0.4,                        # Linear speed in m/s
    start_z=0.25                      # Height (meters)
)
```

## Integration with A* Navigation

1. **Start all three systems** (Gazebo, Nav2 with A*, Dynamic Obstacles) as shown above
2. **Set initial pose** in RViz (2D Pose Estimate button)
3. **Set goal** in RViz (Nav2 Goal button)
4. **Watch A* plan** around moving obstacles and robot navigate smoothly

The A* planner will:
- Compute an initial path around static obstacles
- Handle dynamic obstacles via the costmap layers (obstacle_layer, voxel_layer)
- Robot tracks the path with DWA local planner, avoiding moving obstacles

## Diagnostics

Check if obstacles are spawned:
```bash
ros2 service list | grep spawn
ros2 service list | grep /gazebo/set_model_state
```

Monitor obstacle positions:
```bash
# Check /gazebo/model_states topic for all model positions
ros2 topic echo /gazebo/model_states
```

Verify trajectories are updating:
```bash
# Should show obstacle positions changing over time
ros2 topic hz /gazebo/model_states
```

## Extending the Package

### Adding Custom Trajectory Types

1. Add a new method in `ObstacleController`:
```python
def _compute_sinusoidal_position(self, config, time_elapsed):
    """Your custom trajectory math"""
    x = config['center_x'] + config['amplitude'] * math.sin(time_elapsed * config['frequency'])
    y = config['center_y']
    return x, y
```

2. Add support in `add_*_trajectory()` and `update_obstacles()`:
```python
def add_sinusoidal_trajectory(self, obstacle_name, center_x, center_y, amplitude, frequency, start_z=0.25):
    self.obstacles_config[obstacle_name] = {
        'type': 'sinusoidal',
        'center_x': center_x,
        'center_y': center_y,
        'amplitude': amplitude,
        'frequency': frequency,
        'z': start_z
    }
```

### Creating Custom Obstacle Shapes

Modify `_create_box_sdf()` in `obstacle_spawner.py` to support cylinders, spheres, etc.:
```python
def _create_cylinder_sdf(self, name, radius, height, mass, color):
    # Create SDF with <cylinder> instead of <box>
    ...
```

## Known Limitations

- Obstacles are kinematic (no physics-based collision response with each other)
- Motion patterns are deterministic (for reproducible testing)
- Obstacle update rate is 10 Hz (sufficient for most navigation algorithms)

## Future Enhancements

- [ ] ROS2 parameters for dynamic reconfiguration of trajectories
- [ ] Service to add/remove obstacles at runtime
- [ ] Support for actor models (humanoid motion)
- [ ] Random trajectory generation
- [ ] Velocity tracking (obstacles can be controlled like the robot)

## License

Apache 2.0

## Questions?

For issues or enhancements, refer to the implementation in:
- `dynamic_obstacles/obstacle_spawner.py` - Spawning logic
- `dynamic_obstacles/obstacle_controller.py` - Motion control
