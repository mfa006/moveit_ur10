#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject, BoundingVolume
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class CollisionObjects:
    def __init__(self, node: Node):
        self._node = node
        self._scene_pub = node.create_publisher(PlanningScene, '/planning_scene', 10)

    def add_floor(self, 
                  size_x: float = 2.0, 
                  size_y: float = 2.0, 
                  thickness: float = 0.05, 
                  frame_id: str = 'world', 
                  center_z: float = -0.025,
                  object_id: str = 'floor',
                  anchor_at_origin_corner: bool = False) -> None:
        self._node.get_logger().info("Adding floor collision object to the planning scene...")

        floor = CollisionObject()
        floor.id = object_id
        floor.header.frame_id = frame_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, thickness]

        pose = Pose()
        # If anchoring at origin corner, shift the box so that (0,0) is a min corner
        if anchor_at_origin_corner:
            pose.position.x = size_x * 0.5
            pose.position.y = size_y * 0.5
        pose.position.z = center_z
        pose.orientation.w = 1.0

        floor.primitives.append(primitive)
        floor.primitive_poses.append(pose)
        floor.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(floor)

        self._scene_pub.publish(scene)
        self._node.get_logger().info("Floor collision object published to planning scene.")


    def add_pad(self,
                center_x: float,
                center_y: float,
                size_x: float = 0.6,
                size_y: float = 0.6,
                thickness: float = 0.05,
                frame_id: str = 'world',
                top_z: float = 0.0,
                object_id: str = 'pad') -> None:
        self._node.get_logger().info(f"Adding pad '{object_id}' at ({center_x}, {center_y}) to the planning scene...")

        pad = CollisionObject()
        pad.id = object_id
        pad.header.frame_id = frame_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size_x, size_y, thickness]

        pose = Pose()
        pose.position.x = center_x
        pose.position.y = center_y
        pose.position.z = top_z - thickness * 0.5
        pose.orientation.w = 1.0

        pad.primitives.append(primitive)
        pad.primitive_poses.append(pose)
        pad.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(pad)

        self._scene_pub.publish(scene)
        self._node.get_logger().info(f"Pad '{object_id}' collision object published to planning scene.")

    def add_corner_pads(self,
                        rect_len_x: float,
                        rect_len_y: float,
                        pad_size_x: float = 0.6,
                        pad_size_y: float = 0.6,
                        thickness: float = 0.05,
                        frame_id: str = 'world',
                        top_z: float = 0.0,
                        id_prefix: str = 'pad_corner',
                        floor_centered: bool = False) -> None:
        if floor_centered:
            # Floor is centered at origin, so corners are at ±half dimensions
            half_x = rect_len_x * 0.5
            half_y = rect_len_y * 0.5
            corners = [
                (-half_x, -half_y, f"{id_prefix}_00"),
                (half_x, -half_y, f"{id_prefix}_x0"),
                (-half_x, half_y, f"{id_prefix}_0y"),
                (half_x, half_y, f"{id_prefix}_xy"),
            ]
        else:
            # Floor is anchored at origin corner
            corners = [
                (0.0, 0.0, f"{id_prefix}_00"),
                (rect_len_x, 0.0, f"{id_prefix}_x0"),
                (0.0, rect_len_y, f"{id_prefix}_0y"),
                (rect_len_x, rect_len_y, f"{id_prefix}_xy"),
            ]
        for cx, cy, oid in corners:
            self.add_pad(
                center_x=cx,
                center_y=cy,
                size_x=pad_size_x,
                size_y=pad_size_y,
                thickness=thickness,
                frame_id=frame_id,
                top_z=top_z,
                object_id=oid,
            )


if __name__ == "__main__":
    rclpy.init()
    node = Node("collision_objects")
    collision_objects = CollisionObjects(node)
    rect_len_x = 2.0  # length along +X from robot
    rect_len_y = 1.2  # width along +Y from robot
    thickness = 0.05
    center_z = 0.0  # top surface at z=0
    collision_objects.add_floor(
        size_x=rect_len_x,
        size_y=rect_len_y,
        thickness=thickness,
        frame_id='world',
        center_z=center_z,
        object_id='floor',
        anchor_at_origin_corner=False,
    )
    # Add pads at all four corners  
    collision_objects.add_corner_pads(
        rect_len_x=rect_len_x,
        rect_len_y=rect_len_y,
        pad_size_x=0.6,
        pad_size_y=0.6,
        thickness=0.5,
        frame_id='world',
        top_z=0.0,
        id_prefix='pad_corner',
        floor_centered=True,
    )
    time.sleep(1.0)

    rclpy.spin(node)
    rclpy.shutdown()