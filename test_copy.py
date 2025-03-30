import argparse
import sys
import time

import aria.sdk as aria

import cv2
import numpy as np
from common import quit_keypress, update_iptables

from projectaria_tools.core.sensor_data import ImageDataRecord
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

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
        self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 1000)
        self.timer = self.create_timer(1/30, self.publish_image)
        self.bridge = CvBridge()
        self.images = {}
        self.record = None  # We'll set this in the callback

    def on_image_received(self, image: np.array, record: ImageDataRecord):
        self.images[record.camera_id] = image, record
        self.records[record.camera_id] = record
        image = np.rot90(image, -1)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (640, 480))
        self.publish_image(image)

    def publish_image(self, rgb_image=None):
        if rgb_image is not None and self.record is not None:
            ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            header = Header()
            timestamp = Time()
            timestamp.sec = int(self.record.capture_timestamp_ns / 1.0e9)  # seconds
            timestamp.nanosec = int(self.record.capture_timestamp_ns % 1.0e9)  # nanoseconds

            header.stamp = timestamp
            ros_rgb_image.header = header
            self.image_pub_rgb.publish(ros_rgb_image)
            #self.get_logger().info(f"Published image with timestamp: {timestamp.sec}.{timestamp.nanosec}")


def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

    rclpy.init()
    node = AriaStreamPublisher()

    aria.set_log_level(aria.Level.Info)

    streaming_client = aria.StreamingClient()

    config = streaming_client.subscription_config
    config.subscriber_data_type = aria.StreamingDataType.Rgb
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1

    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config

    # Set the streaming client observer (callback)
    streaming_client.set_streaming_client_observer(node)

    print("Start listening to image data")
    streaming_client.subscribe()

    rclpy.spin(node)

    print("Stop listening to image data")
    streaming_client.unsubscribe()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
