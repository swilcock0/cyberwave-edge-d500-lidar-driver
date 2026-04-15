import ctypes
import logging
import os
from typing import Any, List, Optional

logger = logging.getLogger(__name__)

# Define structure for Point and ScanResult to match C++
class Point(ctypes.Structure):
    _fields_ = [
        ("angle", ctypes.c_float),
        ("distance", ctypes.c_uint16),
        ("intensity", ctypes.c_uint8),
        ("x", ctypes.c_double),
        ("y", ctypes.c_double),
    ]

class ScanResult(ctypes.Structure):
    _fields_ = [
        ("points", ctypes.POINTER(Point)),
        ("count", ctypes.c_int),
        ("stamp", ctypes.c_double),
    ]

class HardwareClient:
    """
    A driver for the Waveshare D500 kit aka LDRobot LD19 Lidar DTOF sensor.
    Bridges the LDRobot C++ SDK via a shared library.
    """

    def __init__(self, config: dict[str, Any]) -> None:
        self.config = config
        self.port = config.get("serial_port", os.getenv("LIDAR_PORT", "/dev/ttyACM0")) # Use /dev/ttyUSB0 for USB connection if using tty-usb board
        self.product = config.get("product_name", os.getenv("LIDAR_PRODUCT", "LD19"))
        self._lib: Optional[ctypes.CDLL] = None

        # Resolve the shared library path
        lib_path = os.getenv("LIDAR_SDK_LIB", "/usr/local/lib/liblidar_wrapper.so")
        if os.path.exists(lib_path):
            self._lib = ctypes.CDLL(lib_path)
            self._setup_ctypes()
        else:
            logger.warning("Lidar wrapper shared library not found at %s. Hardware will be unavailable.", lib_path)

    def _setup_ctypes(self) -> None:
        if not self._lib:
            return
        
        self._lib.lidar_start.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        self._lib.lidar_start.restype = ctypes.c_bool

        self._lib.lidar_get_scan.restype = ScanResult
        self._lib.lidar_free_scan.argtypes = [ScanResult]
        self._lib.lidar_stop.restype = None

    def connect(self) -> None:
        if not self._lib:
            raise RuntimeError("Lidar SDK shared library not loaded")
        
        success = self._lib.lidar_start(
            self.product.encode("utf-8"), 
            self.port.encode("utf-8")
        )
        if not success:
            raise RuntimeError(f"Failed to start lidar on {self.port} ({self.product})")
        logger.info("Connected to Lidar on %s", self.port)

    def read_scan(self) -> Optional[dict[str, Any]]:
        """Polls the lidar for a new scan."""
        if not self._lib:
            return None
        
        res = self._lib.lidar_get_scan()
        if res.count <= 0:
            return None

        try:
            points = []
            for i in range(res.count):
                p = res.points[i]
                points.append({
                    "angle": p.angle,
                    "distance": p.distance,
                    "intensity": p.intensity,
                    "x": p.x,
                    "y": p.y
                })
            
            return {
                "ts": res.stamp,
                "points": points
            }
        finally:
            self._lib.lidar_free_scan(res)

    def disconnect(self) -> None:
        if self._lib:
            self._lib.lidar_stop()
            logger.info("Disconnected from Lidar")
