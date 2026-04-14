# cyberwave-edge-d500-lidar-driver

A Cyberwave driver for the Waveshare D500 kit aka LDRobot LD19 Lidar DTOF sensor

This driver connects to the [Cyberwave](https://cyberwave.com) platform as a digital twin,
allowing you to monitor and control the device from the Cyberwave dashboard and API.

[![License](https://img.shields.io/badge/License-Apache%202.0-orange.svg)](https://opensource.org/licenses/Apache-2.0)

## Prerequisites

- [Docker](https://docs.docker.com/get-docker/)
- A Cyberwave account — [sign up](https://cyberwave.com)
- Any hardware-specific requirements (cables, SDKs, drivers)

## Cyberwave Telemetry & Output Format

This driver automatically bridges Lidar data to your Cyberwave Digital Twin.

*   **State Updates:** Information about the physical state of the hardware is written to the Twin's metadata. This acts as the source-of-truth for the dashboard.
    *   `lidar_status`: Set to `"streaming"` when functioning nominally.
    *   `points_count`: The number of scanned points in the last successful rotation.
*   **Telemetry Pub/Sub:** Raw measurement data is published aggressively (via Zenoh/MQTT) on the `"telemetry"` topic, using a planar JSON structure isomorphic to a ROS `sensor_msgs/LaserScan`:
```json
{
  "ts": 172900223,
  "angle_min": 0.0,
  "angle_max": 6.28318,
  "angle_increment": 0.01026,
  "time_increment": 0.0,
  "scan_time": 0.005,
  "range_min": 0.02,
  "range_max": 12.0,
  "ranges": [1.2, 1.25, ...],
  "intensities": [200, 204, ...]
}
```

## Native ROS 2 Publishing

In addition to publishing up to the Cyberwave platform, this driver is fully multi-lingual on the physical edge device. If enabled, the driver initializes an internal `rclpy` node and co-publishes actual `sensor_msgs.msg.LaserScan` standard messages straight to the physical host's local ROS Data distribution service.

You can configure this by placing an `edge_configs` block inside your local `/tmp/cyberwave-twin.json` OR passing `ENABLE_ROS="true"` on the Docker environment configuration.

Supported configuration keys:

| Config Key | Default | Description |
|---|---|---|
| `enable_ros` | `false` | Enable or disable the secondary `rclpy` publisher (`ENABLE_ROS=true` overrides this). |
| `ros_topic` | `/scan` | The native ROS 2 topic name to publish the `.msg`s down. |
| `ros_frame_id` | `lidar_link` | The TF2 optical link ID to declare as the origin for the `/scan`. |

## Deploying to Cyberwave Edge

Push your image to a registry and deploy via the Cyberwave dashboard.
See the [Edge documentation](https://docs.cyberwave.com/edge/drivers/writing-compatible-drivers) for details.

## Reference drivers

- [cyberwave-edge-camera-driver](https://github.com/cyberwave-os/cyberwave-edge-camera-driver)
- [cyberwave-edge-so101](https://github.com/cyberwave-os/cyberwave-edge-so101)

## License

Apache 2.0 — Sam Wilcock
