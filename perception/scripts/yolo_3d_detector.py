#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters
from ultralytics import YOLO

class Yolo3DDetector(Node):
    def __init__(self):
        super().__init__('yolo_3d_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()
        self.camera_info = None

        # Subscribers
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera_info', self.info_callback, 10)
        
        # Synchronized subscribers for RGB and Depth
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 10, 0.1)
        self.ts.registerCallback(self.rgb_depth_callback)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/yolo_3d/markers', 10)
        self.debug_image_pub = self.create_publisher(Image, '/yolo_3d/debug_image', 10)

        self.get_logger().info('YOLO 3D Detector Node Started')

    def info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info('Camera Info Received')

    def rgb_depth_callback(self, rgb_msg, depth_msg):
        if self.camera_info is None:
            return

        try:
            # Convert images to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting images: {e}')
            return

        # Run YOLO inference
        results = self.model(cv_image, verbose=False)
        
        marker_array = MarkerArray()
        
        # Process detections
        result = results[0]
        boxes = result.boxes.cpu().numpy()
        
        for i, box in enumerate(boxes):
            r = box.xyxy[0].astype(int) # x1, y1, x2, y2
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            label = result.names[cls]

            # 2D Bounding Box Center
            u_center = int((r[0] + r[2]) / 2)
            v_center = int((r[1] + r[3]) / 2)

            # Get Depth at center (or median of ROI)
            # Ensure coordinates are within bounds
            h, w = depth_image.shape
            u_center = max(0, min(u_center, w-1))
            v_center = max(0, min(v_center, h-1))

            # Use a small window to get median depth to start robust
            d_u_min = max(0, u_center - 5)
            d_u_max = min(w, u_center + 5)
            d_v_min = max(0, v_center - 5)
            d_v_max = min(h, v_center + 5)
            
            depth_roi = depth_image[d_v_min:d_v_max, d_u_min:d_u_max]
            
            # Filter NaNs and zeros
            valid_depths = depth_roi[depth_roi > 0]
            valid_depths = valid_depths[~np.isnan(valid_depths)]
            
            if len(valid_depths) == 0:
                continue

            z = np.median(valid_depths)
            
            # Simple Pinhole Model Deprojection
            # x = (u - cx) * z / fx
            # y = (v - cy) * z / fy
            x = (u_center - self.cx) * z / self.fx
            y = (v_center - self.cy) * z / self.fy
            
            # Create Marker
            marker = Marker()
            marker.header.frame_id = self.camera_info.header.frame_id # usually camera_rgb_optical_frame or similar
            marker.header.stamp = rgb_msg.header.stamp
            marker.ns = "yolo_3d"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            
            marker.pose.position.x = float(x)
            marker.pose.position.y = float(y)
            marker.pose.position.z = float(z)
            marker.pose.orientation.w = 1.0
            
            # Approximate size based on 2D box size at that depth? 
            # For now, fixed size or scale based on box width/height ratio
            # Let's say 1 pixel at depth Z is Z/f
            box_w_px = r[2] - r[0]
            box_h_px = r[3] - r[1]
            world_w = box_w_px * z / self.fx
            world_h = box_h_px * z / self.fy
            
            marker.scale.x = float(world_w)
            marker.scale.y = float(world_h)
            marker.scale.z = float(min(world_w, world_h)) # estimate depth thickness roughly
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            marker.text = f"{label} {z:.2f}m"
            
            marker_array.markers.append(marker)
            
            # Create Text Marker above
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = "yolo_3d_text"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(x)
            text_marker.pose.position.y = float(y - world_h/2 - 0.1) # slightly above
            text_marker.pose.position.z = float(z)
            text_marker.scale.z = 0.1
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{label}"
            marker_array.markers.append(text_marker)

            # Draw on 2D image
            cv2.rectangle(cv_image, (r[0], r[1]), (r[2], r[3]), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{label} {z:.2f}m", (r[0], r[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.marker_pub.publish(marker_array)
        
        # Publish Debug Image
        self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8'))
        # Publish Masked Depth
        self.masked_depth_pub.publish(self.bridge.cv2_to_imgmsg(masked_depth_image, encoding='passthrough'))

def main(args=None):
    rclpy.init(args=args)
    node = Yolo3DDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':