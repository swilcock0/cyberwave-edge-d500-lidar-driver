import json
import logging
import os
import time
from pathlib import Path
from typing import Any, Optional

import numpy as np
from cyberwave import Cyberwave

# Optional ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

logger = logging.getLogger(__name__)

POLL_INTERVAL_SECONDS = 0.005  # Increased polling frequency for high-speed Lidar


class CyberwaveEdgeD500LidarDriver:
    """
    A driver for the Waveshare D500 kit aka LDRobot LD19 Lidar DTOF sensor

    Bridges the hardware API and the Cyberwave platform via the digital twin model,
    with optional native ROS2 publishing.
    """

    def __init__(
        self,
        twin_uuid: str,
        api_key: str,
        twin_json_file: str,
        child_uuids: list[str],
    ) -> None:
        self.twin_uuid = twin_uuid
        self.api_key = api_key
        self.twin_json_file = Path(twin_json_file)
        self.child_uuids = child_uuids
        self._twin_data: dict[str, Any] = self._load_twin_json()

        # Read per-device runtime config from edge_configs
        metadata = self._twin_data.get("metadata", {})
        self.edge_configs: dict[str, Any] = metadata.get("edge_configs", {})
        logger.info("Edge configs: %s", self.edge_configs)

        # ROS2 setup
        self.ros_node: Optional[Node] = None
        self.ros_pub: Optional[Any] = None
        self._init_ros()

        # Initialize Cyberwave SDK for data publishing
        self.cw = Cyberwave(api_key=api_key, source_type="edge")

        self._hardware = self._connect_hardware()
        self._last_cw_publish_time = 0.0

    def _init_ros(self) -> None:
        """Initialize ROS2 node and publisher if enabled and available."""
        if not ROS2_AVAILABLE:
            logger.info("ROS2 not available in environment, skipping native ROS publishing")
            return

        enable_ros = self.edge_configs.get("enable_ros", os.getenv("ENABLE_ROS", "true").lower() == "true")
        if not enable_ros:
            logger.info("ROS2 available but disabled via config (enable_ros=false)")
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            
            node_name = f"lidar_driver_{self.twin_uuid[:8]}"
            topic_name = self.edge_configs.get("ros_topic", "/scan")
            
            self.ros_node = rclpy.create_node(node_name)
            self.ros_pub = self.ros_node.create_publisher(LaserScan, topic_name, 10)
            logger.info("Native ROS2 publishing enabled on topic: %s", topic_name)
        except Exception:
            logger.exception("Failed to initialize ROS2 node")

    # ------------------------------------------------------------------
    # Twin JSON helpers
    # ------------------------------------------------------------------

    def _load_twin_json(self) -> dict[str, Any]:
        try:
            return json.loads(self.twin_json_file.read_text())
        except Exception:
            logger.exception("Failed to read twin JSON file at %s", self.twin_json_file)
            raise

    def _save_twin_json(self) -> None:
        self.twin_json_file.write_text(json.dumps(self._twin_data, indent=2))

    def _update_twin_state(self, updates: dict[str, Any]) -> None:
        """Merge updates into twin metadata and persist to disk for Edge Core to sync."""
        self._twin_data.setdefault("metadata", {}).update(updates)
        self._save_twin_json()

    # ------------------------------------------------------------------
    # Hardware
    # ------------------------------------------------------------------

    def _connect_hardware(self):
        """Instantiate and return the hardware client. Exits non-zero if unavailable."""
        from cyberwave_edge_d500_lidar_driver.hardware import HardwareClient
        try:
            client = HardwareClient(config=self.edge_configs)
            client.connect()
            logger.info("Hardware connected")
            return client
        except Exception:
            logger.exception("Cannot connect to hardware — exiting so Edge Core restarts the driver")
            raise

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------

    def run(self) -> None:
        logger.info("Driver running (twin=%s)", self.twin_uuid)
        while True:
            try:
                # Use read_scan from the updated HardwareClient
                scan_data = self._hardware.read_scan()
                if scan_data:
                    ts = scan_data["ts"]
                    raw_points = scan_data["points"]

                    if not raw_points:
                        continue

                    # 2. Prepare LaserScan data
                    sorted_points = sorted(raw_points, key=lambda x: x["angle"])
                    ranges = [p["distance"] / 1000.0 for p in sorted_points]
                    intensities = [float(p["intensity"]) for p in sorted_points]
                    
                    angle_min = np.deg2rad(sorted_points[0]["angle"])
                    angle_max = np.deg2rad(sorted_points[-1]["angle"])
                    angle_increment = (angle_max - angle_min) / (len(sorted_points) - 1) if len(sorted_points) > 1 else 0

                    current_time = time.time()
                    if current_time - self._last_cw_publish_time >= 1.0:
                        # 1. Update twin state with telemetry (e.g. status)
                        self._update_twin_state({"lidar_status": "streaming", "points_count": len(raw_points)})

                        # 3. Publish to Cyberwave (Zenoh/MQTT)
                        laser_scan_json = {
                            "ts": ts,
                            "angle_min": float(angle_min),
                            "angle_max": float(angle_max),
                            "angle_increment": float(angle_increment),
                            "time_increment": 0.0,
                            "scan_time": POLL_INTERVAL_SECONDS,
                            "range_min": 0.02,
                            "range_max": 12.0,
                            "ranges": ranges,
                            "intensities": intensities
                        }
                        self.cw.data.publish("telemetry", laser_scan_json)
                        self._last_cw_publish_time = current_time

                    # 4. Optional: Publish directly to ROS2 if initialized
                    if self.ros_node and self.ros_pub:
                        msg = LaserScan()
                        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
                        msg.header.frame_id = self.edge_configs.get("ros_frame_id", "lidar_optical_link")
                        msg.angle_min = float(angle_min)
                        msg.angle_max = float(angle_max)
                        msg.angle_increment = float(angle_increment)
                        msg.time_increment = 0.0
                        msg.scan_time = POLL_INTERVAL_SECONDS
                        msg.range_min = 0.02
                        msg.range_max = 12.0
                        msg.ranges = ranges
                        msg.intensities = intensities
                        self.ros_pub.publish(msg)

                else:
                    # Optional: wait slightly if no scan data is ready
                    pass

            except Exception:
                logger.exception("Error in main loop")
            
            time.sleep(POLL_INTERVAL_SECONDS)
