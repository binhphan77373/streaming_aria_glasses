# Aria Client SDK For Python

The Aria Client SDK with CLI provides robust capabilities for creating computer vision
and machine learning applications with Project Aria glasses.

## Documentation

- [Project Aria Client SDK](https://facebookresearch.github.io/projectaria_tools/docs/sdk/sdk.mdx)
- [Setup Guide](https://facebookresearch.github.io/projectaria_tools/docs/sdk/setup.mdx)
- [Code Sample Examples and Walkthroughs](https://facebookresearch.github.io/projectaria_tools/docs/sdk/samples/samples.mdx)

To run the Project Aria Client SDK and set up the streaming from the Project Aria glasses to a Jetson Orin NX using ROS2 Humble, follow these steps:

### 1. Connect to NUC (Network Unit Computer)
First, establish a connection to the NUC (Network Unit Computer) using either Wi-Fi or USB.

- **Wi-Fi connection:**
  - Ensure that your NUC and the Project Aria glasses are connected to the same Wi-Fi network.
  
- **USB connection:**
  - Connect the NUC to the Project Aria glasses via a USB cable.

### 2. Start Aria Streaming
Once the connection is established, run the following command to start the streaming from the Project Aria glasses:

```bash
aria streaming start --interface usb --use-ephemeral-certs --profile profile10
```

Explanation:
- `--interface usb`: Specifies the connection interface (USB in this case).
- `--use-ephemeral-certs`: Uses ephemeral certificates for a secure connection.
- `--profile profile10`: Specifies the profile to use, such as `profile10` for the camera setup.

### 3. Stream Camera Data to Jetson Orin NX using ROS2
After starting the streaming, you can subscribe to the streaming data on the Jetson Orin NX via ROS2 by running the following command:

```bash
python -m streaming_subscribe
```

This command will set up a ROS2 node that subscribes to the RGB camera data streamed from the Project Aria glasses to the Jetson Orin NX.

Ensure that you have ROS2 Humble installed on the Jetson Orin NX, and the necessary packages for subscribing to the camera stream are available.

### Additional Notes:
- Ensure that your environment is properly set up with all required dependencies for Project Aria and ROS2.
- If you encounter any issues during setup, refer to the [Project Aria Client SDK Documentation](https://facebookresearch.github.io/projectaria_tools/docs/sdk/sdk.mdx) for troubleshooting and additional setup details.

By following these steps, you should be able to stream camera data from Project Aria glasses to a Jetson Orin NX running ROS2.
