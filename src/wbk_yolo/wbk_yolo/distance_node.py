#!/usr/bin/env python3
"""
Distance Estimation Node for Wolverbot Kickers Vision Stack.

[Write your own description here]
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CameraInfo
# Try to import vision_msgs, fallback to custom messages
try:
    from vision_msgs.msg import Detection2DArray, Detection2D
    VISION_MSGS_AVAILABLE = True
except ImportError:
    from wbk_yolo.msg import Detection2DArray, Detection2D
    VISION_MSGS_AVAILABLE = False
from std_msgs.msg import Float32
from geometry_msgs.msg import Point, PointStamped

# TODO: Import necessary ROS2 message types
# - geometry_msgs.msg (Point, PointStamped)
# - std_msgs.msg (Header, Float32)
# - sensor_msgs.msg (CameraInfo)
# - Detection messages (from vision_msgs or wbk_yolo.msg)

# TODO: Import DistanceEstimator
from wbk_yolo.distance_estimator import DistanceEstimator


class DistanceEstimationNode(Node):
    """[Write your docstring here]"""
    
    def __init__(self):
        super().__init__('distance_estimation_node')
        
        # TODO: Declare parameters
        # - camera_info_topic (default: '/camera/camera_info')
        # - ball_detections_topic (default: '/vision/ball_dets')
        # - frame_id (default: 'camera_optical_frame')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('ball_detections_topic', '/vision/ball_dets')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        
        # TODO: Get parameter values
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.ball_detections_topic = self.get_parameter('ball_detections_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # TODO: Initialize DistanceEstimator with default values
        #   DistanceEstimator(focal_length=525.0, image_width=640, image_height=480)
        self.distance_estimator = DistanceEstimator(focal_length=525.0, image_width=640, image_height=480)
        # TODO: Setup QoS profile (BEST_EFFORT, KEEP_LAST, depth=1)
        qos_profile = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        
        # TODO: Create subscribers
        # - CameraInfo subscriber
        # - Detection2DArray subscriber
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, qos_profile)
        self.ball_detections_sub = self.create_subscription(Detection2DArray, self.ball_detections_topic, self.ball_detections_callback, qos_profile)
        # TODO: Create publishers
        # - Float32 publisher for distance (/vision/ball_distance)
        # - PointStamped publisher for position (/vision/ball_position)
        self.distance_publisher = self.create_publisher(Float32, '/vision/ball_distance', 10)
        self.position_publisher = self.create_publisher(PointStamped, '/vision/ball_position', 10)
        self.get_logger().info("Distance estimation node initialized")
    
    def camera_info_callback(self, msg):
        """
        Update camera parameters when camera_info is received.
        """
        if len(msg.k) >= 4:
            # Camera matrix K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx = msg.k[0]  # Focal length in x direction
            fy = msg.k[4]  # Focal length in y direction (usually same as fx)
            
            # Use average if fx and fy differ slightly
            focal_length = (fx + fy) / 2.0 if abs(fx - fy) < 10 else fx
            
            image_width = msg.width
            image_height = msg.height
            
            self.distance_estimator.update_camera_params(focal_length, image_width, image_height)
    
    def ball_detections_callback(self, msg):
        """
        Process ball detections and estimate distances.
        """
        if len(msg.detections) == 0:
            # No detections - publish zero distance
            self.distance_publisher.publish(Float32(data=0.0))
            return  # Exit early, don't continue
        
        # Find best detection (highest confidence score)
        best_detection = max(msg.detections, key=lambda d: d.score)
        
        # Extract bounding box [x1, y1, x2, y2]
        bbox = best_detection.bbox_xyxy
        x1, y1, x2, y2 = bbox[0], bbox[1], bbox[2], bbox[3]
        
        # Estimate distance
        distance = self.distance_estimator.estimate_ball_distance((x1, y1, x2, y2))
        
        if distance is not None and distance > 0:
            # Publish distance
            distance_msg = Float32()
            distance_msg.data = float(distance)
            self.distance_publisher.publish(distance_msg)
            
            # Calculate 3D position
            calculated_position = self.distance_estimator.calculate_3d_position((x1, y1, x2, y2), distance)
            
            # Create and publish position message
            position_msg = PointStamped()
            position_msg.header = msg.header
            position_msg.header.frame_id = self.frame_id
            position_msg.point = Point(
                x=float(calculated_position[0]), 
                y=float(calculated_position[1]), 
                z=float(calculated_position[2])
            )
            self.position_publisher.publish(position_msg)
        else:
            # Invalid distance - publish zero
            self.distance_publisher.publish(Float32(data=0.0))


def main(args=None):
    """Main entry point for distance estimation node."""
    rclpy.init(args=args)
    
    try:
        node = DistanceEstimationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in distance estimation node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()