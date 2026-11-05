"""Project a depth image into a PointCloud2 using camera intrinsics.

Works around a bug in ros-foxy-gazebo-plugins 3.5.3 (libgazebo_ros_camera.so):
the depth point cloud it publishes has every point's x frozen at the first
column's ray angle (x ~= -0.9497 * depth), i.e. the cloud is collapsed onto
one plane. The published depth IMAGE is correct, so this node reprojects it:

  /camera/depth/image_rect_raw (32FC1) + /camera/depth/camera_info
      -> /camera/depth/color/points (sensor_msgs/PointCloud2, xyz)

Projection (optical frame convention):
  x = (u - cx) * d / fx
  y = (v - cy) * d / fy
  z = d
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField


class DepthProjectorNode(Node):

    def __init__(self):
        super().__init__('depth_projector')

        self.declare_parameter('depth_topic', '/camera/depth/image_rect_raw')
        self.declare_parameter('info_topic', '/camera/depth/camera_info')
        self.declare_parameter('cloud_topic', '/camera/depth/color/points')
        self.declare_parameter('max_depth', 10.0)

        p = lambda n: self.get_parameter(n).value

        self.fx = self.fy = self.cx = self.cy = None
        self.u_grid = self.v_grid = None

        # camera_info from the gazebo plugin is published RELIABLE/volatile
        info_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.info_sub = self.create_subscription(
            CameraInfo, p('info_topic'), self.info_callback, info_qos)
        self.depth_sub = self.create_subscription(
            Image, p('depth_topic'), self.depth_callback,
            qos_profile_sensor_data)
        self.cloud_pub = self.create_publisher(
            PointCloud2, p('cloud_topic'), qos_profile_sensor_data)

        self.get_logger().info('Projecting %s -> %s'
                               % (p('depth_topic'), p('cloud_topic')))

    def info_callback(self, msg):
        if self.fx is not None:
            return
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        u = np.arange(msg.width, dtype=np.float32)
        v = np.arange(msg.height, dtype=np.float32)
        self.u_grid, self.v_grid = np.meshgrid(u, v)
        self.get_logger().info(
            'Intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f (%dx%d)'
            % (self.fx, self.fy, self.cx, self.cy, msg.width, msg.height))

    def depth_callback(self, msg):
        if self.fx is None:
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
            msg.height, msg.width)
        max_depth = self.get_parameter('max_depth').value
        valid = np.isfinite(depth) & (depth > 0.0) & (depth < max_depth)
        if not valid.any():
            return

        d = depth[valid]
        x = (self.u_grid[valid] - self.cx) * d / self.fx
        y = (self.v_grid[valid] - self.cy) * d / self.fy

        cloud = np.empty(x.shape[0], dtype=[
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('pad', np.float32)])
        cloud['x'] = x
        cloud['y'] = y
        cloud['z'] = d
        cloud['pad'] = 0.0

        out = PointCloud2()
        out.header = msg.header
        out.height = 1
        out.width = x.shape[0]
        out.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = out.point_step * out.width
        out.data = cloud.tobytes()
        out.is_dense = True
        self.cloud_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DepthProjectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
