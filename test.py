# import argparse
# import sys
# import time

# import aria.sdk as aria

# import cv2
# import numpy as np
# from common import quit_keypress, update_iptables

# from projectaria_tools.core.sensor_data import ImageDataRecord

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# def parse_args() -> argparse.Namespace:
#     parser = argparse.ArgumentParser()
#     parser.add_argument(
#         "--update_iptables",
#         default=False,
#         action="store_true",
#         help="Update iptables to enable receiving the data stream, only for Linux.",
#     )
#     return parser.parse_args()

# class AriaStreamPublisher(Node):
#     def __init__(self):
#         super().__init__('aria_stream_publisher')
#         self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 1000)
#         self.timer = self.create_timer(1/30, self.publish_image)  # 30 Hz
#         #self.image_pub_slam = self.create_publisher(Image, '/aria/slam_image', 10)
#         #self.image_pub_eye_camera = self.create_publisher(Image, '/aria/eye_camera_image', 10)
#         self.bridge = CvBridge()
#         self.images = {}
#         self.last_time_rgb = time.time()
#         # self.last_time_slam = time.time()
#         # self.last_time_eye = time.time()
#         self.frame_count_rgb = 0
#         # self.frame_count_slam = 0
#         # self.frame_count_eye = 0

#     def on_image_received(self, image: np.array, record: ImageDataRecord):
#         self.images[record.camera_id] = image
#         # print("cam id:", record.camera_id)
#         # print("capture_timestamp_s", record.capture_timestamp_ns/1.0e9)
#         # print("arrival_timestamp_s", record.arrival_timestamp_ns/1.0e9)
#         # print("\n")
#         self.publish_image(image)

#     def publish_image(self, rgb_image=None, slam_image=None, eye_camera_image=None):
#         # RGB image
#         if rgb_image is not None:
#             ros_rgb_image = cv2.resize(rgb_image, (640, 480))
#             ros_rgb_imgmsg = self.bridge.cv2_to_imgmsg(ros_rgb_image, "rgb8")
#             self.image_pub_rgb.publish(ros_rgb_imgmsg)
#         # SLAM image
#         # if slam_image is not None:
#         #     ros_slam_image = self.bridge.cv2_to_imgmsg(slam_image, "mono8")
#         #     self.image_pub_slam.publish(ros_slam_image)

#         # Eye camera image
#         # if eye_camera_image is not None:
#         #     ros_eye_camera_image = self.bridge.cv2_to_imgmsg(eye_camera_image, "mono8")
#         #     self.image_pub_eye_camera.publish(ros_eye_camera_image)
    
#     def calculate_fps(self, camera_id):
#         current_time = time.time()
#         elapsed_time = current_time - getattr(self, f'last_time_{camera_id}')
#         if elapsed_time >= 1.0:
#             frame_count = getattr(self, f'frame_count_{camera_id}')
#             fps = frame_count / elapsed_time
#             print(f"{camera_id.upper()} FPS: {fps:.2f}")
#             setattr(self, f'frame_count_{camera_id}', 0)
#             setattr(self, f'last_time_{camera_id}', current_time)
#         else:
#             setattr(self, f'frame_count_{camera_id}', getattr(self, f'frame_count_{camera_id}') + 1)

#     def update_fps(self):
#         # Update FPS for each camera type
#         if aria.CameraId.Rgb in self.images:
#             self.calculate_fps('rgb')

#         # if aria.CameraId.Slam1 in self.images or aria.CameraId.Slam2 in self.images:
#         #     self.calculate_fps('slam')

#         # if aria.CameraId.EyeTrack in self.images:
#         #     self.calculate_fps('eye')


# def main():
#     args = parse_args()
#     if args.update_iptables and sys.platform.startswith("linux"):
#         update_iptables()

#     # Initialize ROS
#     rclpy.init()
#     observer_node = AriaStreamPublisher()

#     # Optional: Set SDK's log level to Trace or Debug for more verbose logs. Defaults to Info
#     aria.set_log_level(aria.Level.Info)

#     # 1. Create StreamingClient instance
#     streaming_client = aria.StreamingClient()

#     #  2. Configure subscription to listen to Aria's RGB and SLAM streams.
#     config = streaming_client.subscription_config
#     config.subscriber_data_type = (
#         aria.StreamingDataType.Rgb #| aria.StreamingDataType.Slam #| aria.StreamingDataType.EyeTrack
#     )

#     # A shorter queue size may be useful if the processing callback is always slow and you wish to process more recent data
#     # For visualizing the images, we only need the most recent frame so set the queue size to 1
#     config.message_queue_size[aria.StreamingDataType.Rgb] = 1
#     #config.message_queue_size[aria.StreamingDataType.Slam] = 1
#     #config.message_queue_size[aria.StreamingDataType.EyeTrack] = 1

#     # Set the security options
#     options = aria.StreamingSecurityOptions()
#     options.use_ephemeral_certs = True
#     config.security_options = options
#     streaming_client.subscription_config = config

#     # 3. Create and attach observer
#     streaming_client.set_streaming_client_observer(observer_node)

#     # 4. Start listening
#     print("Start listening to image data")
#     streaming_client.subscribe()

#     rclpy.spin(observer_node)

#     # while not quit_keypress() and rclpy.ok():
#         # rclpy.spin_once(observer_node, timeout_sec=0.1)
        
#         # # Render the RGB image
#         # if aria.CameraId.Rgb in observer_node.images:
#         #     rgb_image = np.rot90(observer_node.images[aria.CameraId.Rgb], -1)
#         #     rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
#         #     rgb_image = cv2.resize(rgb_image, (640, 480))
#         #     observer_node.publish_image(rgb_image)
#         #     observer_node.update_fps()
#         #     del observer_node.images[aria.CameraId.Rgb]

#         # # Render the SLAM images
#         # if (
#         #     aria.CameraId.Slam1 in observer.images
#         #     and aria.CameraId.Slam2 in observer.images
#         # ):
#         #     slam1_image = np.rot90(observer.images[aria.CameraId.Slam1], -1)
#         #     slam2_image = np.rot90(observer.images[aria.CameraId.Slam2], -1)

#         #     observer.publish_image(rgb_image=None, slam_image=np.hstack((slam1_image, slam2_image)))
#         #     observer.update_fps()
#         #     del observer.images[aria.CameraId.Slam1]
#         #     del observer.images[aria.CameraId.Slam2]
        
#         # Render the Eye camera image
#         # if aria.CameraId.EyeTrack in observer.images:
#         #     eye_camera_image = np.rot90(observer.images[aria.CameraId.EyeTrack], -1)

#         #     observer.publish_image(rgb_image=None, slam_image=None, eye_camera_image=eye_camera_image)
#         #     observer.update_fps()
#         #     del observer.images[aria.CameraId.EyeTrack]

#     # 5. Unsubscribe to clean up resources
#     print("Stop listening to image data")
#     streaming_client.unsubscribe()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()























import argparse
import sys
import time

import aria.sdk as aria

import cv2
import numpy as np
from common import quit_keypress, update_iptables

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
        self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 1000)
        self.timer = self.create_timer(1/30, self.publish_image)
        #self.image_pub_slam = self.create_publisher(Image, '/aria/slam_image', 10)
        #self.image_pub_eye_camera = self.create_publisher(Image, '/aria/eye_camera_image', 10)
        self.bridge = CvBridge()
        self.images = {}
        self.last_time_rgb = time.time()
        # self.last_time_slam = time.time()
        # self.last_time_eye = time.time()
        self.frame_count_rgb = 0
        # self.frame_count_slam = 0
        # self.frame_count_eye = 0

    def on_image_received(self, image: np.array, record: ImageDataRecord):
        self.images[record.camera_id] = image
        image = cv2.resize(image, (640, 480))
        self.publish_image(image)


    def publish_image(self, rgb_image=None, slam_image=None, eye_camera_image=None):
        # RGB image
        if rgb_image is not None:
            ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
            self.image_pub_rgb.publish(ros_rgb_image)
        # SLAM image
        # if slam_image is not None:
        #     ros_slam_image = self.bridge.cv2_to_imgmsg(slam_image, "mono8")
        #     self.image_pub_slam.publish(ros_slam_image)

        # Eye camera image
        # if eye_camera_image is not None:
        #     ros_eye_camera_image = self.bridge.cv2_to_imgmsg(eye_camera_image, "mono8")
        #     self.image_pub_eye_camera.publish(ros_eye_camera_image)
    
    def calculate_fps(self, camera_id):
        current_time = time.time()
        elapsed_time = current_time - getattr(self, f'last_time_{camera_id}')
        if elapsed_time >= 1.0:
            frame_count = getattr(self, f'frame_count_{camera_id}')
            fps = frame_count / elapsed_time
            print(f"{camera_id.upper()} FPS: {fps:.2f}")
            setattr(self, f'frame_count_{camera_id}', 0)
            setattr(self, f'last_time_{camera_id}', current_time)
        else:
            setattr(self, f'frame_count_{camera_id}', getattr(self, f'frame_count_{camera_id}') + 1)

    def update_fps(self):
        # Update FPS for each camera type
        if aria.CameraId.Rgb in self.images:
            self.calculate_fps('rgb')

        # if aria.CameraId.Slam1 in self.images or aria.CameraId.Slam2 in self.images:
        #     self.calculate_fps('slam')

        # if aria.CameraId.EyeTrack in self.images:
        #     self.calculate_fps('eye')


def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

    # Initialize ROS
    rclpy.init()
    node = AriaStreamPublisher()

    # Optional: Set SDK's log level to Trace or Debug for more verbose logs. Defaults to Info
    aria.set_log_level(aria.Level.Info)

    # 1. Create StreamingClient instance
    streaming_client = aria.StreamingClient()

    #  2. Configure subscription to listen to Aria's RGB and SLAM streams.
    config = streaming_client.subscription_config
    config.subscriber_data_type = (
        aria.StreamingDataType.Rgb #| aria.StreamingDataType.Slam #| aria.StreamingDataType.EyeTrack
    )

    # A shorter queue size may be useful if the processing callback is always slow and you wish to process more recent data
    # For visualizing the images, we only need the most recent frame so set the queue size to 1
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1
    #config.message_queue_size[aria.StreamingDataType.Slam] = 1
    #config.message_queue_size[aria.StreamingDataType.EyeTrack] = 1

    # Set the security options
    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config

    # 3. Create and attach observer
    observer = node
    streaming_client.set_streaming_client_observer(observer)

    # 4. Start listening
    print("Start listening to image data")
    streaming_client.subscribe()

    rclpy.spin(observer)

    # while not quit_keypress() and rclpy.ok():
    #     rclpy.spin_once(node, timeout_sec=0.1)
        
    #     # Render the RGB image
    #     if aria.CameraId.Rgb in observer.images:
    #         rgb_image = np.rot90(observer.images[aria.CameraId.Rgb], -1)
    #         rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
    #         rgb_image = cv2.resize(rgb_image, (640, 480))
    #         observer.publish_image(rgb_image)
    #         observer.update_fps()
    #         del observer.images[aria.CameraId.Rgb]

    #     # # Render the SLAM images
    #     # if (
    #     #     aria.CameraId.Slam1 in observer.images
    #     #     and aria.CameraId.Slam2 in observer.images
    #     # ):
    #     #     slam1_image = np.rot90(observer.images[aria.CameraId.Slam1], -1)
    #     #     slam2_image = np.rot90(observer.images[aria.CameraId.Slam2], -1)

    #     #     observer.publish_image(rgb_image=None, slam_image=np.hstack((slam1_image, slam2_image)))
    #     #     observer.update_fps()
    #     #     del observer.images[aria.CameraId.Slam1]
    #     #     del observer.images[aria.CameraId.Slam2]
        
    #     # Render the Eye camera image
    #     # if aria.CameraId.EyeTrack in observer.images:
    #     #     eye_camera_image = np.rot90(observer.images[aria.CameraId.EyeTrack], -1)

    #     #     observer.publish_image(rgb_image=None, slam_image=None, eye_camera_image=eye_camera_image)
    #     #     observer.update_fps()
    #     #     del observer.images[aria.CameraId.EyeTrack]

    # 5. Unsubscribe to clean up resources
    print("Stop listening to image data")
    streaming_client.unsubscribe()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



# import argparse
# import sys
# import time

# import aria.sdk as aria

# import cv2
# import numpy as np
# from common import quit_keypress, update_iptables

# from projectaria_tools.core.sensor_data import ImageDataRecord

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# def parse_args() -> argparse.Namespace:
#     parser = argparse.ArgumentParser()
#     parser.add_argument(
#         "--update_iptables",
#         default=False,
#         action="store_true",
#         help="Update iptables to enable receiving the data stream, only for Linux.",
#     )
#     return parser.parse_args()

# class AriaStreamPublisher(Node):
#     def __init__(self):
#         super().__init__('aria_stream_publisher')
#         self.image_pub_rgb = self.create_publisher(Image, '/aria/rgb_image', 10)
#         self.timer = self.create_timer(1/30, self.publish_image)
#         self.bridge = CvBridge()
#         self.images = {}


#     def on_image_received(self, image: np.array, record: ImageDataRecord):
#         self.images[record.camera_id] = image

#     def publish_image(self, rgb_image=None, slam_image=None, eye_camera_image=None):
#         # RGB image
#         if rgb_image is not None:
#             ros_rgb_image = self.bridge.cv2_to_imgmsg(rgb_image, "rgb8")
#             self.image_pub_rgb.publish(ros_rgb_image)


# def main():
#     args = parse_args()
#     if args.update_iptables and sys.platform.startswith("linux"):
#         update_iptables()

#     rclpy.init()
#     node = AriaStreamPublisher()

#     aria.set_log_level(aria.Level.Info)

#     streaming_client = aria.StreamingClient()

#     config = streaming_client.subscription_config
#     config.subscriber_data_type = (
#         aria.StreamingDataType.Rgb 
#     )

#     config.message_queue_size[aria.StreamingDataType.Rgb] = 1

#     # Set the security options
#     options = aria.StreamingSecurityOptions()
#     options.use_ephemeral_certs = True
#     config.security_options = options
#     streaming_client.subscription_config = config

#     # 3. Create and attach observer
#     observer = node
#     streaming_client.set_streaming_client_observer(observer)

#     # 4. Start listening
#     print("Start listening to image data")
#     streaming_client.subscribe()

#     while not quit_keypress() and rclpy.ok():
#         rclpy.spin_once(node, timeout_sec=0.1)
        
#         # Render the RGB image
#         if aria.CameraId.Rgb in observer.images:
#             rgb_image = np.rot90(observer.images[aria.CameraId.Rgb], -1)
#             rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
#             rgb_image = cv2.resize(rgb_image, (640, 480))
#             observer.publish_image(rgb_image)
#             del observer.images[aria.CameraId.Rgb]

#     # 5. Unsubscribe to clean up resources
#     print("Stop listening to image data")
#     streaming_client.unsubscribe()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()