# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Provides the ROSCamera class for capturing frames from ROS topics.
"""

import logging
from typing import Any
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge
from PIL import Image
import cv2
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError

from ..camera import Camera
from ..configs import ColorMode
from .configuration_ros import ROSCameraConfig

logger = logging.getLogger(__name__)


class ROSCamera(Camera):
    """Camera that subscribes to ROS topics for image data."""

    def __init__(self, config: ROSCameraConfig):
        super().__init__(config)  # Just pass config, not individual parameters
        self.config = config
        self.bridge = CvBridge()
        self.latest_image = None
        self.node = None
        self.subscription = None
        self._is_connected = False

    def _image_callback(self, msg):
        """Callback for receiving ROS images."""
        try:
            # Convert ROS image to numpy array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.config.encoding)

            # Convert depth (2D) to 3-channel FIRST, before resizing
            if len(cv_image.shape) == 2:
                # Normalize depth to 0-255 range for visualization
                cv_image_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                # Convert to 3-channel by applying colormap
                cv_image = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)

            # Resize if needed (after converting to 3-channel)
            if cv_image.shape[:2] != (self.config.height, self.config.width):
                cv_image = cv2.resize(cv_image, (self.config.width, self.config.height))

            self.latest_image = cv_image
        except Exception as e:
            logger.error(f"Error converting ROS image from {self.config.topic}: {e}")


    def is_connected(self) -> bool:
        """Check if the camera is currently connected."""
        return self._is_connected

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Detects available ROS camera topics.

        Note: This requires ROS to be initialized and topics to be published.
        Returns an empty list as ROS topic discovery is environment-specific.
        """
        logger.warning("ROSCamera.find_cameras() not implemented - ROS topics must be manually specified")
        return []

    def connect(self, warmup: bool = True) -> None:
        """Connect to the ROS topic."""
        if self._is_connected:
            raise DeviceAlreadyConnectedError(f"ROSCamera({self.config.topic}) is already connected.")

        # Initialize ROS if not already initialized
        if not rclpy.ok():
            rclpy.init()

        # Create ROS node
        node_name = f'camera_{self.config.topic.replace("/", "_").replace("-", "_")}'
        self.node = rclpy.create_node(node_name)

        # Subscribe to the ROS topic
        self.subscription = self.node.create_subscription(
            ROSImage,
            self.config.topic,
            self._image_callback,
            10
        )

        self._is_connected = True
        logger.info(f"ROSCamera({self.config.topic}) connected.")

        # Warmup: wait for first image
        if warmup:
            import time
            logger.info(f"Waiting for first image from {self.config.topic}...")
            timeout = 5.0
            start = time.time()
            while self.latest_image is None and (time.time() - start) < timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if self.latest_image is None:
                # Return a black frame if no image received yet
                logger.warning(f"No image received yet from {self.config.topic}, returning black frame")
                return np.zeros((self.config.height, self.config.width, 3), dtype=np.uint8)

    def read(self, color_mode: ColorMode | None = None) -> NDArray[Any]:
        """Read a frame from the camera."""
        if not self._is_connected:
            raise DeviceNotConnectedError(
                f"ROSCamera('{self.config.topic}') is not connected. You need to run `camera.connect()`."
            )

        # Spin ROS to process callbacks
        rclpy.spin_once(self.node, timeout_sec=0.01)

        if self.latest_image is None:
            # Always return 3-channel black frame
            logger.warning(f"No image received yet from {self.config.topic}, returning black frame")
            return np.zeros((self.config.height, self.config.width, 3), dtype=np.uint8)

        return self.latest_image.copy()

    def async_read(self, timeout_ms: float = 1000) -> NDArray[Any]:
        """Asynchronously capture and return a single frame from the camera.

        For ROS cameras, this is the same as read() since ROS handles async internally.
        """
        return self.read()

    def disconnect(self) -> None:
        """Disconnect from the ROS topic."""
        if not self._is_connected:
            raise DeviceNotConnectedError(
                f"ROSCamera('{self.config.topic}') is not connected. You need to run `camera.connect()` before disconnecting."
            )

        if self.node is not None:
            self.node.destroy_node()
            self.node = None

        self._is_connected = False
        logger.info(f"ROSCamera({self.config.topic}) disconnected.")

    def __del__(self):
        """Cleanup on deletion."""
        if hasattr(self, '_is_connected') and self._is_connected:
            try:
                self.disconnect()
            except:
                pass