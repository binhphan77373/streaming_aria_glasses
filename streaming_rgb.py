import argparse
import sys

import aria.sdk as aria

import cv2
import numpy as np
from common import quit_keypress, update_iptables
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from projectaria_tools.core.sensor_data import ImageDataRecord

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
        self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 10)
        self.timer = self.create_timer(1/30, self.publish_image)
        self.bridge = CvBridge()
        self.images = {}
        self.records = {}

    def on_image_received(self, image: np.array, record: ImageDataRecord):
        self.images[record.camera_id] = image
        self.records[record.camera_id] = record
        #print("capture_timestamp_s", record.capture_timestamp_ns/1.0e9)

    def publish_image(self):
        if aria.CameraId.Rgb in self.images and aria.CameraId.Rgb in self.records:
            rgb_image = self.images[aria.CameraId.Rgb]
            rgb_image = np.rot90(rgb_image, -1)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            #rgb_image = cv2.resize(rgb_image, (640, 480))
            rgb_image = cv2.resize(rgb_image, (640, 640))
            record = self.records[aria.CameraId.Rgb]

            ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")

            header = Header()
            timestamp = Time() 
            timestamp.sec = int(record.capture_timestamp_ns / 1.0e9)  # seconds
            timestamp.nanosec = int(record.capture_timestamp_ns % 1.0e9)  # nanoseconds

            header.stamp = timestamp
            ros_rgb_image.header = header
            #print("capture_timestamp_s", record.capture_timestamp_ns / 1.0e9)
            self.image_pub_rgb.publish(ros_rgb_image)


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