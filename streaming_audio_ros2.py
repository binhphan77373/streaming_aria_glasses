import argparse
import sys
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import aria.sdk as aria
import webrtcvad
import soundfile as sf
from datetime import datetime

class AriaAudioPublisher(Node):
    def __init__(self):
        super().__init__('aria_audio_publisher')
        self.audio_pub = self.create_publisher(Int32MultiArray, '/aria/audio', 10)
        self.audio_data = None
        self.audio_record = None
        self.num_channels = 7
        self.sample_rate = 48000
        
        # Initialize VAD
        self.vad = webrtcvad.Vad(3)  # Aggressiveness mode 3 (most aggressive)
        self.frame_duration = 30  # ms
        # VAD requires frame size to be 10, 20, or 30 ms
        self.frame_size = int(self.sample_rate * self.frame_duration / 1000)
        self.voice_detected = False
        self.silence_frames = 0
        self.silence_threshold = 10  # Number of silent frames before stopping
        
        # Audio buffering parameters
        self.target_duration = 5.0  # Target duration in seconds
        self.audio_buffer = []
        self.current_duration = 0.0
        
        # Audio recording
        self.recording_buffer = []
        self.recording_start_time = None
        self.output_dir = "recordings"
        os.makedirs(self.output_dir, exist_ok=True)

    def is_voice_activity(self, audio_frame):
        try:
            # Ensure frame size is correct for VAD
            if len(audio_frame) != self.frame_size:
                # Resize frame to match required size
                audio_frame = audio_frame[:self.frame_size]
                if len(audio_frame) < self.frame_size:
                    # Pad with zeros if too short
                    audio_frame = np.pad(audio_frame, (0, self.frame_size - len(audio_frame)))
            
            # Convert to 16-bit PCM for VAD
            audio_16bit = np.clip(audio_frame, -32768, 32767).astype(np.int16)
            # VAD expects 16-bit PCM data
            return self.vad.is_speech(audio_16bit.tobytes(), self.sample_rate)
        except Exception as e:
            self.get_logger().error(f'Error in VAD processing: {str(e)}')
            return False

    def save_audio(self):
        if self.recording_buffer:
            # Convert buffer to numpy array
            audio_data = np.concatenate(self.recording_buffer)
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = os.path.join(self.output_dir, f'audio_{timestamp}.wav')
            
            # Save audio file
            sf.write(filename, audio_data, self.sample_rate)
            self.get_logger().info(f'Saved audio recording to {filename}')
            
            # Clear buffer
            self.recording_buffer = []
            self.recording_start_time = None

    def on_audio_received(self, audio, record):
        self.audio_data = audio
        self.audio_record = record
        if self.audio_data is not None:
            try:
                # Convert and reshape audio data
                audio_array = np.array(self.audio_data.data, dtype=np.int32)
                num_samples = len(audio_array) // self.num_channels
                audio_matrix = audio_array.reshape(num_samples, self.num_channels)
                
                # Take first channel for VAD
                audio_mono = audio_matrix[:, 0]
                
                # Process audio in frames
                is_speech = False
                for i in range(0, len(audio_mono), self.frame_size):
                    frame = audio_mono[i:i + self.frame_size]
                    if self.is_voice_activity(frame):
                        is_speech = True
                        break
                
                if is_speech:
                    if not self.voice_detected:
                        self.voice_detected = True
                        self.silence_frames = 0
                        self.recording_start_time = time.time()
                        self.get_logger().info('Starting new audio recording')
                    
                    # Calculate duration of this chunk
                    chunk_duration = len(audio_mono) / self.sample_rate
                    self.current_duration += chunk_duration
                    
                    # Add to buffer
                    self.audio_buffer.append(audio_matrix)
                    
                    # Check if we have enough audio
                    if self.current_duration >= self.target_duration:
                        # Concatenate all buffered audio
                        audio_concatenated = np.concatenate(self.audio_buffer)
                        
                        # Create ROS2 message
                        audio_msg = Int32MultiArray()
                        audio_msg.data = audio_concatenated.flatten().tolist()
                        
                        # Publish audio data
                        self.audio_pub.publish(audio_msg)
                        self.get_logger().info(f'Published {self.current_duration:.2f} seconds of audio')
                        
                        # Clear buffer
                        self.audio_buffer = []
                        self.current_duration = 0.0
                    
                    # Add to recording buffer
                    self.recording_buffer.append(audio_matrix)
                    
                else:
                    if self.voice_detected:
                        self.silence_frames += 1
                        if self.silence_frames >= self.silence_threshold:
                            self.voice_detected = False
                            self.get_logger().info('Silence detected - Stopping audio publishing')
                            
                            # If we have buffered audio, send it
                            if self.audio_buffer:
                                audio_concatenated = np.concatenate(self.audio_buffer)
                                audio_msg = Int32MultiArray()
                                audio_msg.data = audio_concatenated.flatten().tolist()
                                self.audio_pub.publish(audio_msg)
                                self.get_logger().info(f'Published final {self.current_duration:.2f} seconds of audio')
                            
                            # Clear buffers
                            self.audio_buffer = []
                            self.current_duration = 0.0
                            
                            # Save the recording
                            self.save_audio()
            except Exception as e:
                self.get_logger().error(f'Error processing audio: {str(e)}')

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
    return parser.parse_args()

def main():
    args = parse_args()
    if args.update_iptables and sys.platform.startswith("linux"):
        from common import update_iptables
        update_iptables()

    # Initialize ROS2
    rclpy.init()
    node = AriaAudioPublisher()
    node.output_dir = args.output_dir
    os.makedirs(node.output_dir, exist_ok=True)

    # Set SDK's log level
    aria.set_log_level(aria.Level.Info)

    # Create StreamingClient instance
    streaming_client = aria.StreamingClient()

    # Configure subscription to listen only to Audio stream
    config = streaming_client.subscription_config
    config.subscriber_data_type = aria.StreamingDataType.Audio
    config.message_queue_size[aria.StreamingDataType.Audio] = 1

    # Set security options
    options = aria.StreamingSecurityOptions()
    options.use_ephemeral_certs = True
    config.security_options = options
    streaming_client.subscription_config = config

    # Create and attach observer
    streaming_client.set_streaming_client_observer(node)

    # Start listening
    print("Start listening to audio data")
    streaming_client.subscribe()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nStopping audio streaming...")
        node.save_audio()
    finally:
        print("Stop listening to audio data")
        streaming_client.unsubscribe()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main() 