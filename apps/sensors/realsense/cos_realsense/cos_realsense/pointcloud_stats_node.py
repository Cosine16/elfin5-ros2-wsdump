"""Subscribe to a PointCloud2 topic, print statistics, and publish a centroid marker.

Demonstrates:
  - rclpy subscription with sensor-data QoS (best effort), which both the
    realsense2_camera driver and the gazebo_ros camera plugin publish with;
  - manual parsing of sensor_msgs/PointCloud2 with numpy (the structure that
    sensor_msgs_py.point_cloud2 wraps);
  - publishing a visualization_msgs/Marker to display a result in RViz.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker

# PointField datatype code -> numpy type
_DATATYPES = {
    1: np.int8,   # INT8
    2: np.uint8,  # UINT8
    3: np.int16,  # INT16
    4: np.uint16, # UINT16
    5: np.int32,  # INT32
    6: np.uint32, # UINT32
    7: np.float32,# FLOAT32
    8: np.float64,# FLOAT64
}


def cloud_to_xyz(msg):
    """Extract the Nx3 xyz array from a PointCloud2 message using its field layout."""
    dtype = np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [_DATATYPES[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })
    data = np.frombuffer(msg.data, dtype=dtype)
    return np.column_stack([data['x'], data['y'], data['z']])


class PointCloudStatsNode(Node):

    def __init__(self):
        super().__init__('pointcloud_stats')
        self.declare_parameter('points_topic', '/camera/depth/color/points')
        topic = self.get_parameter('points_topic').get_parameter_value().string_value

        self.sub = self.create_subscription(
            PointCloud2, topic, self.cloud_callback, qos_profile_sensor_data)
        self.marker_pub = self.create_publisher(Marker, '~/centroid_marker', 10)
        self.frame_count = 0
        self.get_logger().info('Waiting for point cloud on: %s' % topic)

    def cloud_callback(self, msg):
        xyz = cloud_to_xyz(msg)
        valid = xyz[np.isfinite(xyz).all(axis=1)]
        if valid.size == 0:
            return

        centroid = valid.mean(axis=0)
        nearest = np.linalg.norm(valid, axis=1).min()
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                'frames=%d  points=%d/%d  centroid=(%.3f, %.3f, %.3f) m  nearest=%.3f m'
                % (self.frame_count, valid.shape[0], xyz.shape[0],
                   centroid[0], centroid[1], centroid[2], nearest))

        self.publish_centroid_marker(msg, centroid)

    def publish_centroid_marker(self, msg, centroid):
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'centroid'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(centroid[0])
        marker.pose.position.y = float(centroid[1])
        marker.pose.position.z = float(centroid[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.1
        marker.color.a = 1.0
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudStatsNode()
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
