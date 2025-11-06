#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker

class MeshPublisher(Node):
    def __init__(self):
        super().__init__('ply_mesh_publisher')
        self.publisher_ = self.create_publisher(Marker, 'visualization_marker', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_marker)
        self._logged_once = False
        self.get_logger().info('PLY Mesh Publisher node started')

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "ply_mesh"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        marker.mesh_resource = "file:///home/muhammad/IsaacSim-ros_workspaces/humble_ws/src/moveit_ur10/resource/body_map_filt.ply"

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
        marker.color.a = 1.0  # must be > 0

        self.publisher_.publish(marker)
        if not self._logged_once:
            self.get_logger().info('Publishing mesh marker...')
            self._logged_once = True


def main(args=None):
    rclpy.init(args=args)
    node = MeshPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
