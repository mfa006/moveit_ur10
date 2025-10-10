#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math
import time

def get_goal_from_user():
    """Get goal position (m) and orientation (rpy in degrees) from user input"""
    print("Enter goal pose (position in meters, orientation in degrees):")
    try:
        x = float(input("X position: "))
        y = float(input("Y position: "))
        z = float(input("Z position: "))
        roll_deg = float(input("Roll (deg): ") or 0)
        pitch_deg = float(input("Pitch (deg): ") or 0)
        yaw_deg = float(input("Yaw (deg): ") or 0)
        return x, y, z, roll_deg, pitch_deg, yaw_deg
    except ValueError:
        print("Invalid input. Using defaults (0.5, 0.0, 0.5, rpy=0,0,0).")
        return 0.5, 0.0, 0.5, 0.0, 0.0, 0.0


def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (in radians) to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return qx, qy, qz, qw


class UR10MoveItClient(Node):
    def __init__(self):
        super().__init__('ur10_moveit_client')
        
        # Action client for MoveGroup
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # Publisher for planning scene updates (to add floor)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        # Wait for MoveGroup server
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("MoveGroup action server available!")

        # Add floor to the planning scene
        self.add_floor_to_scene()
        time.sleep(1.0)

    def add_floor_to_scene(self):
        """Publish a collision object representing the floor"""
        self.get_logger().info("Adding floor collision object to the planning scene...")

        floor = CollisionObject()
        floor.id = "floor"
        floor.header.frame_id = "world"  # Use 'world' if robot base is at world origin

        # Define a box primitive as floor (2m x 2m x 0.05m)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.05]

        # Set its pose slightly below the robot base
        pose = Pose()
        pose.position.z = -0.025  # half thickness below base (floor is at z = 0)
        pose.orientation.w = 1.0

        # Add primitive to floor object
        floor.primitives.append(primitive)
        floor.primitive_poses.append(pose)
        floor.operation = CollisionObject.ADD

        # Add to PlanningScene
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(floor)

        # Publish once
        self.scene_pub.publish(scene)
        self.get_logger().info("Floor collision object published to planning scene!")

    def move_to_pose(self, pose, group_name="arm"):
        """Move robot to a specific pose using MoveGroup action"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        
        # Define constraints
        # pos_constraint = PositionConstraint()
        # pos_constraint.header.frame_id = "base_link"
        # pos_constraint.link_name = "flange"
        # pos_constraint.target_point_offset.x = pose.position.x
        # pos_constraint.target_point_offset.y = pose.position.y
        # pos_constraint.target_point_offset.z = pose.position.z
        # pos_constraint.weight = 1.0
        
        # orient_constraint = OrientationConstraint()
        # orient_constraint.header.frame_id = "base_link"
        # orient_constraint.link_name = "flange"
        # orient_constraint.orientation = pose.orientation
        # orient_constraint.absolute_x_axis_tolerance = 0.01
        # orient_constraint.absolute_y_axis_tolerance = 0.01
        # orient_constraint.absolute_z_axis_tolerance = 0.01
        # orient_constraint.weight = 1.0
        
        # constraints = Constraints()
        # constraints.name = "pose_goal"
        # constraints.position_constraints = [pos_constraint]
        # constraints.orientation_constraints = [orient_constraint]
        # goal_msg.request.goal_constraints = [constraints]
        
        # Send goal
        self.get_logger().info(f"Sending goal to MoveGroup for group '{group_name}'...")
        future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup server.")
            return False
        
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        if result.error_code.val == result.error_code.SUCCESS:
            self.get_logger().info("Movement completed successfully!")
            return True
        else:
            self.get_logger().error(f"Movement failed with error code: {result.error_code.val}")
            return 
        
    def visualize_goal_pose(self, pose):
        """Publish a PoseStamped and Marker for visualization in RViz"""
        # PoseStamped for TF visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        self.goal_pub.publish(pose_stamped)

        # Marker  
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose = pose
        marker.id = 0
        self.marker_pub.publish(marker)

        self.get_logger().info("Published goal pose to RViz topics: /goal_pose and /goal_marker")


def main():
    rclpy.init()
    try:
        client = UR10MoveItClient()
        
        # Get goal from user
        x, y, z, roll_deg, pitch_deg, yaw_deg = get_goal_from_user()
        roll, pitch, yaw = [math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg)]
        
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z
        target_pose.orientation.x = qx
        target_pose.orientation.y = qy
        target_pose.orientation.z = qz
        target_pose.orientation.w = qw
        
        print(f"Planning to pose: x={x}, y={y}, z={z}, rpy(deg)=({roll_deg}, {pitch_deg}, {yaw_deg})")
        client.visualize_goal_pose(target_pose)
        success = client.move_to_pose(target_pose)
        
        if success:
            print("Movement completed successfully!")
        else:
            print("Movement failed! Possibly in collision or unreachable.")
            
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
