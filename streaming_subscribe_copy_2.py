import argparse
import sys
import time

import aria.sdk as aria

import cv2
import numpy as np
from common import quit_keypress, update_iptables
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from projectaria_tools.core.sensor_data import ImageDataRecord
from projectaria_tools.core.calibration import (
    device_calibration_from_json_string,
    distort_by_calibration,
    get_linear_camera_calibration,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--update_iptables",
        default=False,
        action="store_true",
        help="Update iptables to enable receiving the data stream, only for Linux.",
    )
    return parser.parse_args()


class AriaStreamPublisher(Node):
    def __init__(self):
        super().__init__('aria_stream_publisher')
        self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/aria/camera_info', 10)
        self.timer = self.create_timer(1/30, self.publish_image)
        self.bridge = CvBridge()
        self.images = {}
        self.records = {}

        # Camera Calibration
        self.calibration = None
        self.distortion_calibration = None

    def on_image_received(self, image: np.array, record: ImageDataRecord):
        self.images[record.camera_id] = image
        self.records[record.camera_id] = record
        #print("capture_timestamp_s", record.capture_timestamp_ns/1.0e9)

    def set_camera_calibration(self, calibration_data):
        self.calibration = calibration_data.get_camera_calib("camera-rgb")
        print("Camera Calibration Object:", self.calibration)
        self.distortion_calibration = get_linear_camera_calibration(
            640, 640, 278, "camera-rgb"
        )
        print("Destination Calibration:", self.distortion_calibration)
        
        # Create and publish camera info message
        camera_info = CameraInfo()
        camera_info.header.frame_id = "camera-rgb"
        
        # Get the actual projection parameters
        focal_lengths = self.calibration.get_focal_lengths()
        principal_point = self.calibration.get_principal_point()
        
        # Convert values to float explicitly
        fx = float(focal_lengths[0])
        fy = float(focal_lengths[1])
        cx = float(principal_point[0])
        cy = float(principal_point[1])
        
        # Set camera matrix
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Set distortion parameters - using empty list for now since we can't get them
        camera_info.d = []
        
        # Set rectification matrix (identity matrix)
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # Set projection matrix
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Set image dimensions
        image_size = self.calibration.get_image_size()
        camera_info.width = int(image_size[0])
        camera_info.height = int(image_size[1])
        
        self.camera_info = camera_info

    def publish_image(self):
        if aria.CameraId.Rgb in self.images and aria.CameraId.Rgb in self.records:
            rgb_image = self.images[aria.CameraId.Rgb]
            rgb_image = np.rot90(rgb_image, -1)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)

            # Apply undistortion
            if self.calibration and self.distortion_calibration:
                rgb_image = distort_by_calibration(
                    rgb_image, self.distortion_calibration, self.calibration
                )

            # Resize image
            #rgb_image = cv2.resize(rgb_image, (640, 640))
            record = self.records[aria.CameraId.Rgb]

            ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")

            header = Header()
            timestamp = Time()
            timestamp.sec = int(record.capture_timestamp_ns / 1.0e9)  # seconds
            timestamp.nanosec = int(record.capture_timestamp_ns % 1.0e9)  # nanoseconds

            header.stamp = timestamp
            ros_rgb_image.header = header
            self.image_pub_rgb.publish(ros_rgb_image)
            
            # Publish camera info with the same timestamp
            self.camera_info.header.stamp = timestamp
            self.camera_info_pub.publish(self.camera_info)


def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

    rclpy.init()
    observer_node = AriaStreamPublisher()

    # Set log level
    aria.set_log_level(aria.Level.Info)

    # Create streaming client
    streaming_client = aria.StreamingClient()

    # Subscription configuration
    config = streaming_client.subscription_config
    config.subscriber_data_type = aria.StreamingDataType.Rgb
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1

    # Security options
    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config

    # Create and attach observer
    streaming_client.set_streaming_client_observer(observer_node)

    # Connect to device and get calibration data
    device_client = aria.DeviceClient()
    device = device_client.connect()
    sensors_calib_json = device.streaming_manager.sensors_calibration()
    sensors_calib = device_calibration_from_json_string(sensors_calib_json)
    print("Sensors Calibration Object:", sensors_calib) 
    observer_node.set_camera_calibration(sensors_calib)

    # Start streaming
    print("Start listening to image data")
    streaming_client.subscribe()

    rclpy.spin(observer_node)

    # Unsubscribe and shutdown
    print("Stop listening to image data")
    streaming_client.unsubscribe()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
