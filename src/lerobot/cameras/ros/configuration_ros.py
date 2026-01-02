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

from dataclasses import dataclass

from ..configs import CameraConfig

__all__ = ["ROSCameraConfig"]


@CameraConfig.register_subclass("ros")
@dataclass
class ROSCameraConfig(CameraConfig):
    """Configuration class for ROS-based cameras (e.g., Aurora930).

    This class provides configuration options for cameras that publish
    images via ROS topics, such as depth cameras or RGB cameras.

    Example configurations:
```python
    # RGB camera from Aurora930
    ROSCameraConfig(topic="/rgb/image_raw", width=320, height=200, fps=30)

    # Depth camera from Aurora930
    ROSCameraConfig(topic="/depth/image_raw", width=320, height=200, fps=30, encoding="passthrough")
```

    Attributes:
        topic: ROS topic name to subscribe to (e.g., '/rgb/image_raw')
        fps: Requested frames per second
        width: Requested frame width in pixels
        height: Requested frame height in pixels
        encoding: Image encoding ('bgr8' for RGB, 'passthrough' for depth)
    """

    topic: str
    encoding: str = 'bgr8'

    def __post_init__(self) -> None:
        if not self.topic.startswith('/'):
            raise ValueError(f"ROS topic must start with '/', but '{self.topic}' is provided.")