import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import numpy as np
import torch
from transformers import WhisperForConditionalGeneration, WhisperProcessor
from scipy.signal import resample_poly

class AudioTranscriptionNode(Node):
    def __init__(self):
        super().__init__('audio_transcription_node')
        
        # Initialize Whisper model and processor
        self.processor = WhisperProcessor.from_pretrained("openai/whisper-tiny")
        self.model = WhisperForConditionalGeneration.from_pretrained("openai/whisper-tiny").to("cuda")
        
        # Audio parameters
        self.sample_rate = 48000  # Aria's sample rate
        self.target_sample_rate = 16000  # Whisper's required sample rate
        self.num_channels = 7  # Aria's number of channels
        
        # Create subscriber for audio data
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/aria/audio',
            self.audio_callback,
            10)
        self.subscription  # prevent unused variable warning
        
        # Create publisher for transcriptions
        self.transcription_publisher = self.create_publisher(
            String,
            '/aria/transcription',
            10
        )
        
        self.get_logger().info('Audio transcription node initialized')

    def process_audio_data(self, audio_data):
        # Convert to numpy array and reshape
        audio_array = np.array(audio_data, dtype=np.int32)
        num_samples = len(audio_array) // self.num_channels
        audio_matrix = audio_array.reshape(num_samples, self.num_channels)
        
        # Take only the first channel (mono)
        audio_mono = audio_matrix[:, 0]
        
        # Convert to float32 and normalize
        audio_float = audio_mono.astype(np.float32) / np.iinfo(np.int32).max
        
        # Resample to 16kHz using scipy
        num_samples_resampled = int(len(audio_float) * self.target_sample_rate / self.sample_rate)
        audio_resampled = resample_poly(audio_float, self.target_sample_rate, self.sample_rate)
        
        # Process with Whisper
        input_features = self.processor(
            audio_resampled,
            sampling_rate=self.target_sample_rate,
            return_tensors="pt",
            language="en"
        ).input_features
        
        with torch.no_grad():
            predicted_ids = self.model.generate(input_features.to("cuda"))[0]
        
        transcription = self.processor.decode(predicted_ids)
        return transcription

    def audio_callback(self, msg):
        try:
            # Process the audio data
            transcription = self.process_audio_data(msg.data)
            
            # Publish the transcription
            transcription_msg = String()
            transcription_msg.data = transcription
            self.transcription_publisher.publish(transcription_msg)
            
            self.get_logger().info(f'Transcription: {transcription}')
        except Exception as e:
            self.get_logger().error(f'Error during transcription: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AudioTranscriptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main() 