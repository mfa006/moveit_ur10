#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, PlanningScene, CollisionObject
from moveit_msgs.msg import BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker
from moveit_msgs.msg import RobotTrajectory

import math
import time

import os
import sys
sys.path.append(os.path.dirname(__file__))
from collision_objects import CollisionObjects  # type: ignore

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

        # Publishers
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.marker_pub = self.create_publisher(Marker, '/goal_marker', 10)
        self.traj_pub = self.create_publisher(RobotTrajectory, '/display_planned_path', 10)
        # Wait for MoveGroup server
        self.get_logger().info("Waiting for MoveGroup action server...")
        self.move_group_client.wait_for_server(timeout_sec=10.0)
        self.get_logger().info("MoveGroup action server available!")

        # Collision objects helper
        self.collision_objects = CollisionObjects(self)

        # Add rectangular floor where the robot base (at world origin) is at the left corner
        # Choose rectangle dimensions (meters)
        rect_len_x = 2.0  # length along +X from robot
        rect_len_y = 1.2  # width along +Y from robot
        thickness = 0.05
        center_z = -0.2  # top surface at z=0
        self.collision_objects.add_floor(
            size_x=rect_len_x,
            size_y=rect_len_y,
            thickness=thickness,
            frame_id='world',
            center_z=center_z,
            object_id='floor',
            anchor_at_origin_corner=True,
        )
        # Add pads at all four corners  
        self.collision_objects.add_corner_pads(
            rect_len_x=rect_len_x,
            rect_len_y=rect_len_y,
            pad_size_x=0.6,
            pad_size_y=0.6,
            thickness=0.5,
            frame_id='world',
            top_z=0.0,
            id_prefix='pad_corner',
        )
        time.sleep(1.0)

    

    def move_to_pose(self, pose, group_name="arm"):
        """Move robot to a specific pose using MoveGroup action"""
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = group_name
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "flange"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]  # small tolerance box

        bv = BoundingVolume()
        bv.primitives.append(primitive)
        bv.primitive_poses.append(pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "flange"
        orient_constraint.orientation = pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.05
        orient_constraint.absolute_y_axis_tolerance = 0.05
        orient_constraint.absolute_z_axis_tolerance = 0.05
        orient_constraint.weight = 1.0

        # Combine into goal constraints
        constraints = Constraints()
        constraints.name = "pose_goal"
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [orient_constraint]
        goal_msg.request.goal_constraints = [constraints]

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

        get_result_response = result_future.result()
        # In ROS 2 actions, get_result() returns a response wrapper with a `.result` field
        action_result = getattr(get_result_response, 'result', None)
        if action_result is None:
            self.get_logger().error("MoveGroup result is empty or invalid.")
            return False

        if action_result.error_code.val == action_result.error_code.SUCCESS:
            traj = action_result.planned_trajectory  # RobotTrajectory
            self.traj_pub.publish(traj)
            self.get_logger().info("Movement completed successfully!")
            return True
        else:
            self.get_logger().error(f"Movement failed with error code: {action_result.error_code.val}")
            return False

    def visualize_goal_pose(self, pose):
        """Publish a PoseStamped and Marker for visualization in RViz"""
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "world"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose
        self.goal_pub.publish(pose_stamped)

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

        self.get_logger().info("Published goal pose to /goal_pose and /goal_marker")



def main():
    rclpy.init()
    try:
        client = UR10MoveItClient()

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
