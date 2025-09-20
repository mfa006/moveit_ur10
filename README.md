# MoveIt UR10 Package

A ROS2 MoveIt configuration package for the Universal Robots UR10 robotic arm with gripper integration. This package provides motion planning capabilities for the UR10 robot using the MoveIt Motion Planning Framework.

## Package Information

- **Package Name**: `moveit_ur10`
- **Version**: 0.3.0
- **Maintainer**: Muhammad Faran Akram (faran1218@gmail.com)
- **License**: BSD
- **Description**: An automatically generated package with all the configuration and launch files for using the ur10_with_gripper with the MoveIt Motion Planning Framework

## Prerequisites

Before using this package, ensure you have the following installed:

- ROS2 Humble
- MoveIt2
- Universal Robots UR description package (`ur_description`)
- RViz2
- Controller Manager
- Robot State Publisher
- Joint State Publisher

## Package Structure

```
moveit_ur10/
├── config/                     # Configuration files
│   ├── ur10_with_gripper.urdf.xacro      # Main URDF with gripper
│   ├── ur10_with_gripper_isaac.urdf.xacro # Isaac Sim specific URDF
│   ├── ur10_with_gripper.srdf            # Semantic Robot Description
│   ├── moveit_controllers.yaml          # MoveIt controller configuration
│   ├── ros2_controllers.yaml            # ROS2 controller configuration
│   ├── kinematics.yaml                  # Kinematic solver configuration
│   ├── joint_limits.yaml               # Joint limits configuration
│   ├── pilz_cartesian_limits.yaml       # Pilz planner limits
│   ├── initial_positions.yaml           # Initial joint positions
│   └── moveit.rviz                     # RViz configuration
├── launch/                     # Launch files
│   ├── demo.launch.py                   # Complete demo with RViz
│   ├── isaac_moveit.launch.py          # Isaac Sim integration
│   ├── move_group.launch.py             # MoveIt move group only
│   ├── moveit_rviz.launch.py            # MoveIt with RViz
│   ├── setup_assistant.launch.py        # MoveIt Setup Assistant
│   ├── rsp.launch.py                    # Robot State Publisher
│   ├── spawn_controllers.launch.py      # Controller spawning
│   ├── static_virtual_joint_tfs.launch.py # Static transforms
│   └── warehouse_db.launch.py           # MoveIt warehouse database
└── CMakeLists.txt              # Build configuration
```

## Building the Package

1. Navigate to your ROS2 workspace:
   ```bash
   cd /path/to/your/ros2_workspace
   ```

2. Build the package:
   ```bash
   colcon build --packages-select moveit_ur10
   ```

3. Source the workspace:
   ```bash
   source install/setup.zsh
   ```

## Usage

### 1. Complete Demo 

Launch the complete demo with RViz visualization:

```bash
ros2 launch moveit_ur10 isaac_moveit.launch.py ros2_control_hardware_type:=isaac
```

This will start:
- MoveIt move group
- RViz with MoveIt motion planning plugin
- Robot state publisher
- Joint state publisher GUI

### 2. Isaac Sim Integration

For Isaac Sim simulation:
run sim/ur10_ros2.usd file in isaacsim 
 

### 3. MoveIt  

Change move group to 'arm' and give comands to move the arm.
 
 
### 5. MoveIt Setup Assistant

To reconfigure the robot or modify planning groups:

```bash
ros2 launch moveit_ur10 setup_assistant.launch.py
```
 
## Robot Configuration

### Planning Groups

The robot is configured with the following planning groups:
- **ur10_arm**: Main UR10 arm (6 DOF)
- **gripper**: Gripper control

### Available Controllers

- `arm_controller`: Controls the UR10 arm joints
- `gripper_controller`: Controls the gripper
- `joint_state_broadcaster`: Publishes joint states

### Hardware Types

The package supports multiple hardware interfaces:
- `mock_components`: For simulation/testing
- `isaac`: For Isaac Sim integration

## Motion Planning

### Planning Pipelines

The package includes two planning pipelines:
1. **OMPL**: Open Motion Planning Library with various planners
2. **Pilz Industrial Motion Planner**: For industrial applications

### Available Planners

- RRT Connect
- RRT Star
- PRM
- PRM Star
- Pilz Industrial Motion Planner

## Interactive Motion Planning

Once the demo is running:

1. **In RViz**:
   - Use the "Planning" tab in the Motion Planning plugin
   - Set start and goal states by dragging the interactive markers
   - Click "Plan" to generate a motion plan
   - Click "Execute" to execute the planned trajectory

2. **Using Joint State Publisher GUI**:
   - Adjust joint positions manually
   - Use sliders to move individual joints

## Configuration Files

### Key Configuration Files

- **`ur10_with_gripper.srdf`**: Defines planning groups, collision objects, and end-effectors
- **`kinematics.yaml`**: Configures kinematic solvers (KDL, IKFast, etc.)
- **`joint_limits.yaml`**: Defines joint position, velocity, and acceleration limits
- **`moveit_controllers.yaml`**: Configures MoveIt controller interfaces
- **`ros2_controllers.yaml`**: Defines ROS2 controller configurations

### Customization

To customize the robot configuration:

1. Use the MoveIt Setup Assistant:
   ```bash
   ros2 launch moveit_ur10 setup_assistant.launch.py
   ```

2. Modify configuration files directly in the `config/` directory

3. Update URDF files for physical robot modifications

## Troubleshooting

### Common Issues

1. **Build Errors**: Ensure all dependencies are installed
2. **Launch Failures**: Check that all required packages are sourced
3. **Planning Failures**: Verify joint limits and collision objects
4. **Controller Issues**: Ensure proper hardware interface configuration

### Debug Commands

Check available topics:
```bash
ros2 topic list
```

Monitor joint states:
```bash
ros2 topic echo /joint_states
```

Check MoveIt services:
```bash
ros2 service list | grep move_group
```

## Dependencies

### Required Packages
- `moveit_ros_move_group`
- `moveit_kinematics`
- `moveit_planners`
- `moveit_simple_controller_manager`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `tf2_ros`
- `xacro`
- `controller_manager`
- `moveit_configs_utils`
- `moveit_ros_visualization`
- `moveit_ros_warehouse`
- `moveit_setup_assistant`
- `robot_state_publisher`
- `rviz2`
- `ur_description`
- `warehouse_ros_mongo`
 

 