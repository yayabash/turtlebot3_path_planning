#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2
import tf2_ros
from sklearn.cluster import DBSCAN


class DynamicMotionSegmentation(Node):
    def __init__(self):
        super().__init__('dynamic_motion_segmentation')

        # Subscribe to depth-camera point cloud
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.cloud_callback,
            10
        )

        # Publish clustered dynamic regions as markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/dynamic_markers',
            10
        )

        self.prev_cloud = None
        self.prev_time = None

        # TF buffer kept for future ego-motion compensation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('Dynamic motion segmentation node started')

    def cloud_callback(self, msg: PointCloud2) -> None:
        # Convert PointCloud2 to Nx3 numpy array
        points = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(msg, skip_nans=True)
        ])

        if points.size == 0:
            return

        # First frame: just store and return
        if self.prev_cloud is None:
            self.prev_cloud = points
            self.prev_time = msg.header.stamp
            return

        # --- Cloud differencing: naive nearest-neighbor threshold ---
        prev = self.prev_cloud
        dynamic = []

        # For each current point, check if there was a close point before
        for p in points:
            dists = np.linalg.norm(prev - p, axis=1)
            if np.min(dists) > 0.15:  # motion threshold (meters)
                dynamic.append(p)

        dynamic = np.array(dynamic)
        self.prev_cloud = points
        self.prev_time = msg.header.stamp

        if dynamic.size == 0 or dynamic.shape[0] < 30:
            return

        # --- Clustering dynamic points into moving volumes ---
        clustering = DBSCAN(eps=0.4, min_samples=20).fit(dynamic)
        labels = clustering.labels_

        self.publish_markers(dynamic, labels, msg.header)

    def publish_markers(self, points: np.ndarray, labels: np.ndarray, header) -> None:
        markers = MarkerArray()
        unique_labels = set(labels)

        marker_id = 0
        for label in unique_labels:
            if label == -1:
                continue  # noise cluster

            cluster = points[labels == label]
            center = cluster.mean(axis=0)

            marker = Marker()
            marker.header = header
            marker.ns = 'dynamic_objects'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = float(center[0])
            marker.pose.position.y = float(center[1])
            marker.pose.position.z = float(center[2])

            marker.scale.x = 0.6
            marker.scale.y = 0.6
            marker.scale.z = 1.6

            marker.color.r = 1.0
            marker.color.g = 0.2
            marker.color.b = 0.2
            marker.color.a = 0.8

            markers.markers.append(marker)
            marker_id += 1

        if markers.markers:
            self.marker_pub.publish(markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicMotionSegmentation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
