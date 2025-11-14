#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from moveit_msgs.msg import PlanningScene
import trimesh

MESH_PATH = "/home/muhammad/IsaacSim-ros_workspaces/humble_ws/src/moveit_ur10/resource/body_map_filt.ply"

class CollisionMeshPublisher(Node):
    def __init__(self):
        super().__init__("collision_mesh_publisher")

        self.mesh_frame = self.declare_parameter("mesh_frame", "base_link").value
        self.mesh_position_x = self.declare_parameter("mesh_position_x", 0.0).value
        self.mesh_position_y = self.declare_parameter("mesh_position_y", 0.0).value
        self.mesh_position_z = self.declare_parameter("mesh_position_z", -0.5).value
        self.mesh_roll = self.declare_parameter("mesh_roll", 0.0).value
        self.mesh_pitch = self.declare_parameter("mesh_pitch", 0.0).value
        self.mesh_yaw = self.declare_parameter("mesh_yaw", 0.0).value

        self.publisher_ = self.create_publisher(
            PlanningScene, "planning_scene", 10
        )

        self.timer = self.create_timer(1.0, self.publish_collision_mesh)
        self.sent_once = False

    def publish_collision_mesh(self):
        if self.sent_once:
            return

        # Load mesh with trimesh
        tm = trimesh.load(MESH_PATH)

        mesh_msg = Mesh()
        for face in tm.faces:
            tri = MeshTriangle()
            tri.vertex_indices = [int(i) for i in face]
            mesh_msg.triangles.append(tri)

        for vertex in tm.vertices:
            pt = Point()
            pt.x, pt.y, pt.z = (float(v) for v in vertex)
            mesh_msg.vertices.append(pt)

        # CollisionObject
        col_obj = CollisionObject()
        col_obj.header.frame_id = self.mesh_frame
        col_obj.id = "body_mesh"

        col_obj.meshes.append(mesh_msg)

        pose = Pose()
        pose.position.x = self.mesh_position_x
        pose.position.y = self.mesh_position_y
        pose.position.z = self.mesh_position_z
        qx, qy, qz, qw = quaternion_from_euler(
            self.mesh_roll, self.mesh_pitch, self.mesh_yaw
        )
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        col_obj.mesh_poses.append(pose)

        col_obj.operation = CollisionObject.ADD

        # PlanningScene
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(col_obj)

        self.publisher_.publish(scene)
        self.get_logger().info(
            "Published collision mesh to MoveIt planning scene "
            f"(frame='{self.mesh_frame}', position=({self.mesh_position_x}, "
            f"{self.mesh_position_y}, {self.mesh_position_z}))"
        )

        self.sent_once = True

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Return quaternion (x, y, z, w) from Euler angles."""
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


def main():
    rclpy.init()
    node = CollisionMeshPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

