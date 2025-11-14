#!/usr/bin/env python3
import math
import time
from pathlib import Path 
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
import trimesh


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Return quaternion as (x, y, z, w) from Euler angles."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class CollisionObjects:
    DEFAULT_BODY_MESH_PATH = "/home/muhammad/IsaacSim-ros_workspaces/humble_ws/src/moveit_ur10/resource/body_map_filt.ply"

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

    def publish_body_mesh(self,
                          roll: float = 0.0,
                          pitch: float = 0.0,
                          yaw: float = 0.0,
                          mesh_path: str = None,
                          frame_id: str = 'world',
                          object_id: str = 'body_mesh',
                          position_x: float = 0.0,
                          position_y: float = 0.0,
                          position_z: float = 0.0) -> None:
        mesh_path = mesh_path or self.DEFAULT_BODY_MESH_PATH
        mesh_file = Path(mesh_path)
        if not mesh_file.exists():
            self._node.get_logger().error(f"Mesh file '{mesh_path}' does not exist. Skipping body mesh publication.")
            return

        self._node.get_logger().info(
            f"Publishing body mesh collision object '{object_id}' using mesh '{mesh_path}' "
            f"in frame '{frame_id}' at ({position_x}, {position_y}, {position_z}) "
            f"with roll={roll}, pitch={pitch}, yaw={yaw}."
        )

        tm = trimesh.load(mesh_path)

        mesh_msg = Mesh()
        for face in tm.faces:
            tri = MeshTriangle()
            tri.vertex_indices = [int(i) for i in face]
            mesh_msg.triangles.append(tri)

        for vertex in tm.vertices:
            pt = Point()
            pt.x, pt.y, pt.z = (float(v) for v in vertex)
            mesh_msg.vertices.append(pt)

        col_obj = CollisionObject()
        col_obj.header.frame_id = frame_id
        col_obj.id = object_id
        col_obj.meshes.append(mesh_msg)

        pose = Pose()
        pose.position.x = position_x
        pose.position.y = position_y
        pose.position.z = position_z
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        col_obj.mesh_poses.append(pose)
        col_obj.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(col_obj)

        self._scene_pub.publish(scene)
        self._node.get_logger().info("Body mesh collision object published to planning scene.")


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
    collision_objects.publish_body_mesh(roll=0, pitch=3.14, yaw= -1.57, position_x=-1.0, position_y=-0.5, position_z=0.0)
    time.sleep(1.0)

    rclpy.spin(node)
    rclpy.shutdown()