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

## Configuration (Edge Configs)

This driver is configured via the **Digital Twin metadata** on the Cyberwave platform. You can override these by adding an `edge_configs` object to your Twin's metadata in the dashboard.

| Config Key | Default | Description |
|---|---|---|
| `serial_port` | `/dev/ttyACM0` | The device path for the Lidar (e.g., `/dev/ttyUSB0`). Overridden by `LIDAR_PORT` env var. |
| `product_name` | `LD19` | The LDRobot/Waveshare product model. Overridden by `LIDAR_PRODUCT` env var. |
| `enable_ros` | `false` | Enable/disable the secondary `rclpy` publisher. Overridden by `ENABLE_ROS=true` env var. |
| `ros_topic` | `/scan` | The native ROS 2 topic name for local publishing. |
| `ros_frame_id` | `lidar_optical_link` | The TF2 optical link ID for the `/scan` message header. |

## Native ROS 2 Publishing

In addition to publishing up to the Cyberwave platform, this driver is fully multi-lingual on the physical edge device. If enabled, the driver initializes an internal `rclpy` node and co-publishes actual `sensor_msgs.msg.LaserScan` standard messages straight to the physical host's local ROS Data distribution service.

## License

Apache 2.0 — Sam Wilcock
