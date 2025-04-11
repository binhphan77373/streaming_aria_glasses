import argparse
import sys
import os
import time
from datetime import datetime
import numpy as np
import cv2
import soundfile as sf
from queue import Queue
import threading
from common import quit_keypress, update_iptables
import aria.sdk as aria
from projectaria_tools.core.sensor_data import ImageDataRecord

class AudioRecorder:
    def __init__(self, output_dir='recordings', channel=0):
        self.output_dir = output_dir
        self.channel = channel
        os.makedirs(output_dir, exist_ok=True)
        self.recording = False
        self.audio_buffer = []
        self.start_time = None

    def start_recording(self):
        self.recording = True
        self.audio_buffer = []
        self.start_time = time.time()
        print(f"Starting recording for channel {self.channel}...")

    def stop_recording(self):
        self.recording = False
        if self.audio_buffer:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'audio_channel{self.channel}_{timestamp}.wav')
            audio_data = np.vstack(self.audio_buffer)[:, self.channel]
            sf.write(filename, audio_data, 48000)
            print(f"Saved recording to {filename}")
            return True
        return False

    def add_audio(self, audio_matrix):
        if self.recording:
            self.audio_buffer.append(audio_matrix)

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--update_iptables",
        default=False,
        action="store_true",
        help="Update iptables to enable receiving the data stream, only for Linux.",
    )
    parser.add_argument(
        "--output_dir",
        default="recordings",
        help="Directory to save audio recordings",
    )
    parser.add_argument(
        "--channel",
        type=int,
        default=0,
        help="Microphone channel to process (0-6)",
    )
    return parser.parse_args()

def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        update_iptables()

    # Initialize audio recorder with selected channel
    channel = min(max(args.channel, 0), 6)  # Ensure channel is between 0 and 6
    recorder = AudioRecorder(args.output_dir, channel=channel)

    #  Optional: Set SDK's log level to Trace or Debug for more verbose logs. Defaults to Info
    aria.set_log_level(aria.Level.Info)

    # 1. Create StreamingClient instance
    streaming_client = aria.StreamingClient()

    #  2. Configure subscription to listen to Aria's RGB, SLAM and Audio streams.
    config = streaming_client.subscription_config
    config.subscriber_data_type = (
        aria.StreamingDataType.Rgb | aria.StreamingDataType.Slam | aria.StreamingDataType.Audio
    )

    # A shorter queue size may be useful if the processing callback is always slow and you wish to process more recent data
    # For visualizing the images, we only need the most recent frame so set the queue size to 1
    config.message_queue_size[aria.StreamingDataType.Rgb] = 1
    config.message_queue_size[aria.StreamingDataType.Slam] = 1
    config.message_queue_size[aria.StreamingDataType.Audio] = 1

    # Set the security options
    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config

    # 3. Create and attach observer
    class StreamingClientObserver:
        def __init__(self):
            self.images = {}
            self.audio_data = None
            self.audio_record = None
            self.num_channels = 7
            self.sample_rate = 48000

        def on_image_received(self, image: np.array, record: ImageDataRecord):
            self.images[record.camera_id] = image

        def on_audio_received(self, audio, record):
            self.audio_data = audio
            self.audio_record = record
            if self.audio_data is not None:
                # Convert and reshape audio data
                audio_array = np.array(self.audio_data.data, dtype=np.int32)
                num_samples = len(audio_array) // self.num_channels
                audio_matrix = audio_array.reshape(num_samples, self.num_channels)
                
                max_value = np.iinfo(np.int32).max
                audio_matrix = np.clip(audio_matrix * 10, -max_value, max_value)
                
                # Add to recording if active
                recorder.add_audio(audio_matrix)
                
                # Print basic information
                print(f"\nReceived audio data with {len(audio_matrix)} samples")
                print(f"Audio data shape: {audio_matrix.shape}, dtype: {audio_matrix.dtype}")
                print(f"Sample rate: {self.sample_rate}Hz, Channels: {self.num_channels}")
                
                if hasattr(record, 'capture_timestamps_ns'):
                    print(f"Audio timestamps (first 5): {record.capture_timestamps_ns[:5]}")
                if hasattr(record, 'audio_muted'):
                    print(f"Audio muted: {record.audio_muted}")

    observer = StreamingClientObserver()
    streaming_client.set_streaming_client_observer(observer)

    # 4. Start listening
    print("Start listening to image and audio data")
    streaming_client.subscribe()

    # 5. Visualize the streaming data until we close the window
    rgb_window = "Aria RGB"
    slam_window = "Aria SLAM"

    cv2.namedWindow(rgb_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(rgb_window, 1024, 1024)
    cv2.setWindowProperty(rgb_window, cv2.WND_PROP_TOPMOST, 1)
    cv2.moveWindow(rgb_window, 50, 50)

    cv2.namedWindow(slam_window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(slam_window, 480 * 2, 640)
    cv2.setWindowProperty(slam_window, cv2.WND_PROP_TOPMOST, 1)
    cv2.moveWindow(slam_window, 1100, 50)

    print("\nControls:")
    print("Press 'r' to start/stop recording")
    print("Press 'q' to quit")

    while not quit_keypress():
        # Render the RGB image
        if aria.CameraId.Rgb in observer.images:
            rgb_image = np.rot90(observer.images[aria.CameraId.Rgb], -1)
            rgb_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2RGB)
            cv2.imshow(rgb_window, rgb_image)
            del observer.images[aria.CameraId.Rgb]

        # Stack and display the SLAM images
        if (
            aria.CameraId.Slam1 in observer.images
            and aria.CameraId.Slam2 in observer.images
        ):
            slam1_image = np.rot90(observer.images[aria.CameraId.Slam1], -1)
            slam2_image = np.rot90(observer.images[aria.CameraId.Slam2], -1)
            cv2.imshow(slam_window, np.hstack((slam1_image, slam2_image)))
            del observer.images[aria.CameraId.Slam1]
            del observer.images[aria.CameraId.Slam2]

        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('r'):
            if not recorder.recording:
                recorder.start_recording()
            else:
                recorder.stop_recording()

    # 6. Unsubscribe to clean up resources
    print("Stop listening to image data")
    streaming_client.unsubscribe()

if __name__ == "__main__":
    main()
