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
from sensor_msgs.msg import Image
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
        self.image_pub_undistorted = self.create_publisher(Image, '/aria/rgb_image', 10)
        self.timer = self.create_timer(1/30, self.publish_image)
        self.bridge = CvBridge()
        self.images = {}
        self.records = {}
        
        # Lấy thông tin hiệu chuẩn (calibration) từ thiết bị Aria
        self.streaming_client = aria.StreamingClient()
        # Tạo calibration cho ảnh đích (đã được hiệu chỉnh)
        self.dst_calib = get_linear_camera_calibration(512, 512, 150, "camera-rgb")
        
        # Lưu trữ thông tin calibration gốc từ camera
        self.rgb_calib = None
        
    def set_camera_calibration(self, sensors_calib_json):
        if sensors_calib_json:
            sensors_calib = device_calibration_from_json_string(sensors_calib_json)
            self.rgb_calib = sensors_calib.get_camera_calib("camera-rgb")
            self.get_logger().info("Camera calibration loaded successfully")

    def on_image_received(self, image: np.array, record: ImageDataRecord):
        self.images[record.camera_id] = image
        self.records[record.camera_id] = record

    def publish_image(self):
        if aria.CameraId.Rgb in self.images and aria.CameraId.Rgb in self.records:
            rgb_image = self.images[aria.CameraId.Rgb]
            rgb_image = np.rot90(rgb_image, -1)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            rgb_image = cv2.resize(rgb_image, (640, 640))
            record = self.records[aria.CameraId.Rgb]

            header = Header()
            timestamp = Time() 
            timestamp.sec = int(record.capture_timestamp_ns / 1.0e9)  # seconds
            timestamp.nanosec = int(record.capture_timestamp_ns % 1.0e9)  # nanoseconds
            header.stamp = timestamp
            
            # Chỉ xử lý và xuất bản ảnh đã hiệu chỉnh
            if self.rgb_calib is not None:
                # Áp dụng hiệu chỉnh ảnh
                undistorted_rgb_image = distort_by_calibration(
                    cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR), 
                    self.dst_calib, 
                    self.rgb_calib
                )
                
                # Chuyển ảnh đã hiệu chỉnh sang định dạng ROS
                undistorted_rgb_image = cv2.cvtColor(undistorted_rgb_image, cv2.COLOR_BGR2RGB)
                ros_undistorted_image = self.bridge.cv2_to_imgmsg(undistorted_rgb_image, "rgb8")
                ros_undistorted_image.header = header
                
                # Xuất bản ảnh đã hiệu chỉnh
                self.image_pub_undistorted.publish(ros_undistorted_image)
            else:
                self.get_logger().warning("Chưa có thông tin hiệu chuẩn, không thể hiệu chỉnh ảnh")


def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

    rclpy.init()
    observer_node = AriaStreamPublisher()

    aria.set_log_level(aria.Level.Info)

    streaming_client = aria.StreamingClient()

    config = streaming_client.subscription_config
    config.subscriber_data_type = (
        aria.StreamingDataType.Rgb
    )

    config.message_queue_size[aria.StreamingDataType.Rgb] = 1

    # Set the security options
    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config
    
    # Lấy thông tin hiệu chuẩn từ thiết bị (nếu có thể)
    try:
        # Kết nối với thiết bị để lấy thông tin hiệu chuẩn
        device_client = aria.DeviceClient()
        device = device_client.connect()
        streaming_manager = device.streaming_manager
        sensors_calib_json = streaming_manager.sensors_calibration()
        observer_node.set_camera_calibration(sensors_calib_json)
        device_client.disconnect(device)
    except Exception as e:
        print(f"Không thể lấy thông tin hiệu chuẩn: {e}")
        print("Ảnh sẽ không được hiệu chỉnh")

    # 3. Create and attach observer
    streaming_client.set_streaming_client_observer(observer_node)

    # 4. Start listening
    print("Start listening to image data")
    streaming_client.subscribe()
    rclpy.spin(observer_node)

    # 5. Unsubscribe to clean up resources
    print("Stop listening to image data")
    streaming_client.unsubscribe()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
