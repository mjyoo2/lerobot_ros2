"""
LeRobot ROS2 Bridge

A complete integration bridge between ROS2 and LeRobot for robot learning.

Modules:
    - data_collection: Tools for collecting robot demonstrations via ROS2
    - training: Convert rosbag data and train policies
    - deployment: Deploy trained policies to ROS2 robots
    - utils: Utility functions and helpers
    - configs: Configuration schemas and validators
"""

__version__ = "0.1.0"
__author__ = "Your Name"

# Import only what exists (avoid import errors for incomplete modules)
__all__ = ["__version__"]

try:
    from . import hardware
    __all__.append("hardware")
except ImportError:
    pass

try:
    from . import data_collection
    __all__.append("data_collection")
except ImportError:
    pass

try:
    from . import deployment
    __all__.append("deployment")
except ImportError:
    pass

try:
    from . import training
    __all__.append("training")
except ImportError:
    pass

try:
    from . import utils
    __all__.append("utils")
except ImportError:
    pass
