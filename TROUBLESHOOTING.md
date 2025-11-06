# Troubleshooting Guide: Trajectory Execution Failure in Isaac Sim

## Issue Summary

The UR10 robotic arm in Isaac Sim was not moving when commanded through MoveIt, despite successful trajectory planning. The controller was rejecting all trajectories with the error:

```
[ERROR] [arm_controller]: Time between points 0 and 1 is not strictly increasing, it is 0.000000 and 0.000000 respectively
```

## Root Cause Analysis

### Problem Identification

1. **Trajectory Planning Succeeded**: MoveIt was successfully generating motion plans using OMPL planners
2. **CHOMP Optimization Issue**: CHOMP trajectory optimizer was being automatically used as a post-processing step
3. **Timestamp Stripping**: CHOMP optimizer was creating optimized trajectories but **stripping all `time_from_start` timestamps**, leaving them as `0.000000` for all trajectory points
4. **Controller Rejection**: The `joint_trajectory_controller` requires strictly increasing timestamps and rejects trajectories with zero or duplicate timestamps

### Why This Happened

- MoveIt2's default configuration includes CHOMP as a trajectory optimizer
- CHOMP optimizer focuses on collision avoidance and smoothness but doesn't preserve timing information
- The trajectory execution manager wasn't recomputing timestamps after CHOMP optimization
- Without proper timestamps, the ROS2 controller cannot interpolate between waypoints and rejects the trajectory

## Solution

### Changes Made

#### 1. Created OMPL Planning Configuration (`config/ompl_planning.yaml`)

**Purpose**: Ensure proper request adapters are used, including `AddTimeOptimalParameterization` which computes timestamps.

```yaml
planning_plugin: ompl_interface/OMPLPlanner
start_state_max_bounds_error: 0.1
jiggle_fraction: 0.05
request_adapters: >-
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/ResolveConstraintFrames
    default_planner_request_adapters/FixWorkspaceBounds
    default_planner_request_adapters/FixStartStateBounds
    default_planner_request_adapters/FixStartStateCollision
    default_planner_request_adapters/FixStartStatePathConstraints
```

**Key Point**: `AddTimeOptimalParameterization` adapter computes `time_from_start` values based on joint velocity and acceleration limits.

#### 2. Disabled CHOMP Trajectory Optimization (`launch/isaac_moveit.launch.py`)

**Purpose**: Prevent CHOMP from stripping timestamps after planning.

```python
# Disable CHOMP trajectory optimization to prevent timestamp issues
{"planning_pipelines.ompl.enable_trajectory_optimization": False},
```

**Key Point**: This parameter disables the automatic CHOMP optimization that was removing timestamps.

#### 3. Enhanced Trajectory Execution Configuration (`config/moveit_controllers.yaml`)

**Purpose**: Ensure proper trajectory execution parameters are set.

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.2
  allowed_goal_duration_margin: 0.5
  allowed_start_tolerance: 0.1
  execution_duration_monitoring: true
  max_safe_path_cost: 1.0
  wait_for_trajectory_execution: true
  controller_connection_timeout: 60.0
  controller_action_timeout: 30.0
  default_velocity_scaling_factor: 0.1
  default_acceleration_scaling_factor: 0.1
```

**Key Point**: These parameters ensure trajectories are validated and executed with proper timing constraints.

#### 4. Enhanced Controller Configuration (`config/ros2_controllers.yaml`)

**Purpose**: Add controller-specific parameters for better trajectory handling.

```yaml
arm_controller:
  ros__parameters:
    # ... joint configuration ...
    state_publish_rate: 50.0
    action_monitor_rate: 20.0
    allow_partial_joints_goal: false
    constraints:
      stopped_velocity_tolerance: 0.01
      goal_time: 0.6
```

**Key Point**: These parameters help the controller properly validate and execute trajectories.

## Verification

### Success Indicators

After applying the fixes, the following confirms successful execution:

1. **No CHOMP Messages**: Logs show OMPL planner being used directly without CHOMP optimization
   ```
   [INFO] [moveit.ompl_planning.model_based_planning_context]: Planner configuration 'arm' will use planner 'geometric::RRTConnect'
   ```

2. **Trajectory Accepted**: Controller accepts the trajectory
   ```
   [INFO] [arm_controller]: Accepted new action goal
   [INFO] [follow_joint_trajectory_controller_handle]: Goal request accepted!
   ```

3. **No Timestamp Errors**: No more "Time between points is not strictly increasing" errors

4. **Robot Movement**: The arm successfully executes planned trajectories

## Technical Details

### How Trajectory Timestamps Work

1. **Planning Phase**: OMPL planner generates a path (sequence of joint positions)
2. **Parameterization**: `AddTimeOptimalParameterization` adapter computes `time_from_start` for each waypoint based on:
   - Joint velocity limits
   - Joint acceleration limits
   - Distance between waypoints
3. **Execution**: Controller uses timestamps to interpolate between waypoints and execute smooth motion

### Why CHOMP Caused Issues

- CHOMP optimizer creates a new trajectory optimized for collision avoidance
- The optimized trajectory contains only joint positions (waypoints)
- Timing information (`time_from_start`) is not preserved
- Without timestamps, the controller cannot execute the trajectory

### Why Disabling CHOMP Works

- OMPL planners generate valid trajectories
- `AddTimeOptimalParameterization` ensures timestamps are computed
- Timestamps are preserved throughout the execution pipeline
- Controller receives trajectories with proper timing information

## Prevention

### Best Practices

1. **Always Configure Request Adapters**: Ensure `AddTimeOptimalParameterization` is in your OMPL config
2. **Monitor Trajectory Optimization**: If using trajectory optimizers, ensure they preserve timestamps
3. **Validate Trajectories**: Check that trajectories have proper timestamps before execution
4. **Controller Configuration**: Ensure controller parameters match your trajectory execution settings

### Debugging Commands

If you encounter similar issues:

```bash
# Check if trajectories have timestamps
ros2 topic echo /arm_controller/follow_joint_trajectory/goal --once

# Monitor trajectory execution
ros2 topic echo /arm_controller/follow_joint_trajectory/feedback

# Check MoveIt parameters
ros2 param list /move_group | grep -i trajectory

# Verify planning pipeline
ros2 param get /move_group planning_pipelines.ompl.enable_trajectory_optimization
```

## Related Files

- `config/ompl_planning.yaml` - OMPL planning configuration
- `config/moveit_controllers.yaml` - MoveIt controller and trajectory execution configuration
- `config/ros2_controllers.yaml` - ROS2 controller configuration
- `launch/isaac_moveit.launch.py` - Main launch file with optimization disabled

## References

- [MoveIt2 Trajectory Execution Documentation](https://moveit.picknik.ai/main/doc/concepts/trajectory_execution.html)
- [OMPL Planning Request Adapters](https://moveit.picknik.ai/main/doc/concepts/planning_request_adapter.html)
- [ROS2 Control Joint Trajectory Controller](https://control.ros.org/master/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)

