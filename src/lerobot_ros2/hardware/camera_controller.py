"""
Camera Controller (Direct implementation without LeRobot)

Directly uses OpenCV and RealSense to control cameras.
"""

from typing import Dict, Optional
import numpy as np
try:
    import cv2
except ImportError:
    cv2 = None


class CameraController:
    """
    Camera control using OpenCV

    Args:
        camera_configs: Camera configuration dictionary
            Example: {
                "top": {"type": "opencv", "index": 0, "width": 640, "height": 480},
                "wrist": {"type": "opencv", "index": 1, "width": 320, "height": 240}
            }
    """

    def __init__(self, camera_configs: Dict[str, Dict]):
        self.camera_configs = camera_configs
        self.cameras = {}
        self.is_connected = False

    def connect(self) -> bool:
        """Camera connection"""
        try:
            for camera_name, config in self.camera_configs.items():
                camera_type = config.get("type", "opencv")

                if camera_type == "opencv":
                    camera = self._init_opencv_camera(camera_name, config)
                elif camera_type == "realsense":
                    camera = self._init_realsense_camera(camera_name, config)
                else:
                    raise ValueError(f"Unsupported camera type: {camera_type}")

                self.cameras[camera_name] = camera
                print(f"✓ Camera '{camera_name}' connected successfully")

            self.is_connected = True
            print(f"✓ Initialized all cameras")
            return True

        except Exception as e:
            print(f"✗ Camera connection failed: {e}")
            self.is_connected = False
            return False

    def _init_opencv_camera(self, name: str, config: Dict):
        """Initialize OpenCV camera"""
        index = config.get("index", 0)
        width = config.get("width", 640)
        height = config.get("height", 480)
        fps = config.get("fps", 30)

        cap = cv2.VideoCapture(index)

        if not cap.isOpened():
            raise Exception(f"Failed to open camera {index}")

        # Set resolution
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)

        # Verify settings
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = int(cap.get(cv2.CAP_PROP_FPS))

        print(f"  Camera {name}: {actual_width}x{actual_height} @ {actual_fps}fps")

        return cap

    def _init_realsense_camera(self, name: str, config: Dict):
        """Initialize RealSense camera"""
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise ImportError(
                "For RealSense support, install pyrealsense2:\n"
                "pip install pyrealsense2"
            )

        width = config.get("width", 640)
        height = config.get("height", 480)
        fps = config.get("fps", 30)
        enable_depth = config.get("enable_depth", False)

        pipeline = rs.pipeline()
        config_rs = rs.config()

        # RGB stream
        config_rs.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)

        # Depth stream (optional)
        if enable_depth:
            config_rs.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)

        pipeline.start(config_rs)

        print(f"  RealSense {name}: {width}x{height} @ {fps}fps")

        return pipeline

    def disconnect(self):
        """Disconnect cameras"""
        if not self.is_connected:
            return

        for name, camera in self.cameras.items():
            try:
                if isinstance(camera, cv2.VideoCapture):
                    camera.release()
                else:
                    # RealSense pipeline
                    camera.stop()
                print(f"✓ Camera '{name}' disconnected")
            except Exception as e:
                print(f"✗ Failed to disconnect camera '{name}': {e}")

        self.cameras.clear()
        self.is_connected = False

    def read_images(self) -> Dict[str, np.ndarray]:
        """
        Read images from all cameras

        Returns:
            {"observation.images.{camera_name}": image_array}
        """
        if not self.is_connected:
            return {}

        images = {}

        for camera_name, camera in self.cameras.items():
            try:
                image = self._read_single_camera(camera_name, camera)
                if image is not None:
                    key = f"observation.images.{camera_name}"
                    images[key] = image
            except Exception as e:
                print(f"✗ Failed to read camera '{camera_name}': {e}")

        return images

    def _read_single_camera(self, name: str, camera) -> Optional[np.ndarray]:
        """Read image from single camera"""
        if isinstance(camera, cv2.VideoCapture):
            # OpenCV camera
            ret, frame = camera.read()
            if not ret:
                return None

            # BGR -> RGB conversion
            config = self.camera_configs[name]
            if config.get("color_mode", "rgb") == "rgb":
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            return frame

        else:
            # RealSense camera
            import pyrealsense2 as rs

            frames = camera.wait_for_frames()
            color_frame = frames.get_color_frame()

            if not color_frame:
                return None

            # Convert to numpy array
            image = np.asanyarray(color_frame.get_data())

            # BGR -> RGB conversion
            config = self.camera_configs[name]
            if config.get("color_mode", "rgb") == "rgb":
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

            return image

    def get_observation(self) -> Dict[str, np.ndarray]:
        """
        Get current camera observations (LeRobot-compatible format)

        Returns:
            {"observation.images.{camera_name}": image}
        """
        return self.read_images()

    def __del__(self):
        """Destructor"""
        self.disconnect()


def test_camera():
    """Test code"""
    print("Camera test starting...\n")

    # Configuration
    camera_configs = {
        "webcam": {
            "type": "opencv",
            "index": 0,
            "width": 640,
            "height": 480,
            "fps": 30,
            "color_mode": "rgb"
        }
    }

    # Create controller
    controller = CameraController(camera_configs)

    # Connect
    if not controller.connect():
        print("Camera connection failed")
        return

    # Read images
    print("\nReading images... (5 seconds)")
    import time

    for i in range(5):
        images = controller.read_images()

        for key, image in images.items():
            print(f"{key}: shape={image.shape}, dtype={image.dtype}")

            # Display image (optional)
            cv2.imshow("Test", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
            cv2.waitKey(1)

        time.sleep(1)

    cv2.destroyAllWindows()

    # Disconnect
    controller.disconnect()
    print("\nTest complete")


if __name__ == "__main__":
    test_camera()
