import numpy as np
import rclpy
from rclpy.node import Node
from typing import Any, Callable, Optional, Tuple

from ament_index_python.packages import get_package_share_directory
from kit_interfaces.msg import DetectedObject.msg
from kit_interfaces.msg import DetectionArray.msg
