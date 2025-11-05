"""
RosbagRecorder: Record ROS2 topics to rosbag2 files.

This module provides a wrapper around rosbag2 recording functionality
for collecting robot demonstration data.
"""

import subprocess
import time
from pathlib import Path
from typing import List, Optional
import signal
import os


class RosbagRecorder:
    """
    Wrapper for rosbag2 recording.

    Provides convenient interface for recording robot demonstrations
    to rosbag2 files for later conversion to LeRobot dataset format.

    Args:
        topics: List of ROS2 topics to record
        output_dir: Directory to save rosbag files (default: "data/rosbags")
        storage: Storage plugin to use (default: "sqlite3")
        serialization: Serialization format (default: "cdr")
    """

    def __init__(
        self,
        topics: List[str],
        output_dir: str = "data/rosbags",
        storage: str = "sqlite3",
        serialization: str = "cdr",
    ):
        self.topics = topics
        self.output_dir = Path(output_dir)
        self.storage = storage
        self.serialization = serialization

        # Recording state
        self.recording_process: Optional[subprocess.Popen] = None
        self.is_recording = False
        self.current_bag_path: Optional[Path] = None

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def start_recording(
        self,
        bag_name: str,
        duration: Optional[float] = None,
        compression: Optional[str] = None,
    ) -> bool:
        """
        Start recording to a new rosbag file.

        Args:
            bag_name: Name for the rosbag (without extension)
            duration: Optional duration in seconds (None = infinite)
            compression: Optional compression mode ("zstd", "lz4", None)

        Returns:
            True if recording started successfully, False otherwise
        """
        if self.is_recording:
            print("❌ Already recording. Stop current recording first.")
            return False

        # Build rosbag2 record command
        cmd = [
            "ros2", "bag", "record",
            *self.topics,
            "-o", str(self.output_dir / bag_name),
            "-s", self.storage,
            "--serialization-format", self.serialization,
        ]

        # Add optional parameters
        if duration is not None:
            cmd.extend(["-d", str(int(duration))])

        if compression is not None:
            cmd.extend(["--compression-mode", compression])

        try:
            # Start recording process
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid,  # Create new process group for clean termination
            )

            self.is_recording = True
            self.current_bag_path = self.output_dir / bag_name

            print(f"🔴 Started recording to: {self.current_bag_path}")
            print(f"   Topics: {', '.join(self.topics)}")
            if duration:
                print(f"   Duration: {duration}s")

            return True

        except Exception as e:
            print(f"❌ Failed to start recording: {e}")
            return False

    def stop_recording(self) -> bool:
        """
        Stop current recording.

        Returns:
            True if stopped successfully, False otherwise
        """
        if not self.is_recording or self.recording_process is None:
            print("❌ Not currently recording")
            return False

        try:
            # Send SIGINT to process group (like Ctrl+C)
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)

            # Wait for process to terminate (with timeout)
            self.recording_process.wait(timeout=5.0)

            self.is_recording = False
            print(f"⏹️  Stopped recording: {self.current_bag_path}")

            self.recording_process = None
            return True

        except subprocess.TimeoutExpired:
            # Force kill if doesn't respond
            print("⚠️  Recording didn't stop gracefully, forcing termination...")
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGKILL)
            self.recording_process.wait()
            self.is_recording = False
            self.recording_process = None
            return True

        except Exception as e:
            print(f"❌ Error stopping recording: {e}")
            return False

    def record_episode(
        self,
        episode_name: str,
        duration: float,
        countdown: int = 3,
        compression: Optional[str] = None,
    ) -> bool:
        """
        Record a single episode with countdown.

        Args:
            episode_name: Name for the episode
            duration: Recording duration in seconds
            countdown: Countdown before starting (default: 3 seconds)
            compression: Optional compression mode

        Returns:
            True if episode recorded successfully, False otherwise
        """
        # Countdown
        if countdown > 0:
            print(f"\n{'='*60}")
            print(f"Recording: {episode_name}")
            print(f"Duration: {duration}s")
            print(f"{'='*60}")

            for i in range(countdown, 0, -1):
                print(f"Starting in {i}...", end="\r")
                time.sleep(1)
            print("🔴 RECORDING NOW!" + " " * 20)

        # Start recording with duration
        if not self.start_recording(episode_name, duration=duration, compression=compression):
            return False

        # Wait for duration
        time.sleep(duration)

        # rosbag2 should auto-stop after duration, but ensure it's stopped
        time.sleep(0.5)  # Small buffer
        if self.is_recording:
            self.stop_recording()

        print(f"✅ Episode '{episode_name}' recorded successfully!\n")
        return True

    def record_multiple_episodes(
        self,
        num_episodes: int,
        episode_prefix: str = "episode",
        episode_duration: float = 30.0,
        reset_time: float = 10.0,
        start_index: int = 0,
        compression: Optional[str] = None,
    ) -> List[Path]:
        """
        Record multiple episodes with reset time between them.

        Args:
            num_episodes: Number of episodes to record
            episode_prefix: Prefix for episode names (default: "episode")
            episode_duration: Duration of each episode in seconds (default: 30)
            reset_time: Time between episodes for environment reset (default: 10)
            start_index: Starting index for episode numbering (default: 0)
            compression: Optional compression mode

        Returns:
            List of paths to recorded bag files
        """
        recorded_bags = []

        for i in range(num_episodes):
            episode_idx = start_index + i
            episode_name = f"{episode_prefix}_{episode_idx:03d}"

            print(f"\n{'='*60}")
            print(f"Episode {i+1}/{num_episodes}")
            print(f"{'='*60}")

            # Reset time
            if i > 0:  # Skip reset before first episode
                print(f"\n⏸️  Reset environment ({reset_time}s)")
                print("   Position robot and objects for next episode...")
                for t in range(int(reset_time), 0, -1):
                    print(f"   Next recording in {t}s...", end="\r")
                    time.sleep(1)
                print()

            # Record episode
            if self.record_episode(
                episode_name,
                duration=episode_duration,
                countdown=3,
                compression=compression,
            ):
                recorded_bags.append(self.current_bag_path)
            else:
                print(f"❌ Failed to record episode {episode_name}")

        print(f"\n{'='*60}")
        print(f"✅ Completed! Recorded {len(recorded_bags)}/{num_episodes} episodes")
        print(f"   Saved to: {self.output_dir}")
        print(f"{'='*60}\n")

        return recorded_bags

    def get_bag_info(self, bag_path: Path) -> Optional[dict]:
        """
        Get information about a recorded bag.

        Args:
            bag_path: Path to the bag directory

        Returns:
            Dictionary with bag info or None if failed
        """
        try:
            result = subprocess.run(
                ["ros2", "bag", "info", str(bag_path)],
                capture_output=True,
                text=True,
                check=True,
            )
            # TODO: Parse output into structured dict
            return {"raw_info": result.stdout}
        except Exception as e:
            print(f"❌ Failed to get bag info: {e}")
            return None

    def list_recorded_bags(self) -> List[Path]:
        """
        List all recorded bags in output directory.

        Returns:
            List of paths to bag directories
        """
        if not self.output_dir.exists():
            return []

        # rosbag2 creates directories for each bag
        bags = [
            p for p in self.output_dir.iterdir()
            if p.is_dir() and (p / "metadata.yaml").exists()
        ]

        return sorted(bags)

    def __del__(self):
        """Ensure recording is stopped on cleanup."""
        if self.is_recording:
            self.stop_recording()


def main():
    """Example usage of RosbagRecorder."""
    import argparse

    parser = argparse.ArgumentParser(description="Record ROS2 topics to rosbag")
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/joint_states", "/camera/top/image_raw", "/joint_commands"],
        help="Topics to record",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="data/rosbags",
        help="Output directory for bags",
    )
    parser.add_argument(
        "--episodes",
        type=int,
        default=1,
        help="Number of episodes to record",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=30.0,
        help="Duration per episode (seconds)",
    )
    parser.add_argument(
        "--reset-time",
        type=float,
        default=10.0,
        help="Time between episodes (seconds)",
    )
    parser.add_argument(
        "--prefix",
        type=str,
        default="episode",
        help="Episode name prefix",
    )
    parser.add_argument(
        "--compression",
        type=str,
        choices=["zstd", "lz4", None],
        default=None,
        help="Compression mode",
    )

    args = parser.parse_args()

    # Create recorder
    recorder = RosbagRecorder(
        topics=args.topics,
        output_dir=args.output_dir,
    )

    # Record episodes
    if args.episodes == 1:
        # Single episode
        recorder.record_episode(
            episode_name=f"{args.prefix}_001",
            duration=args.duration,
            compression=args.compression,
        )
    else:
        # Multiple episodes
        recorder.record_multiple_episodes(
            num_episodes=args.episodes,
            episode_prefix=args.prefix,
            episode_duration=args.duration,
            reset_time=args.reset_time,
            compression=args.compression,
        )


if __name__ == "__main__":
    main()
