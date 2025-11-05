#!/usr/bin/env python3
"""
MoveIt Trajectory Executor - Action Server

Handles RViz Execute button to execute trajectories on real robot.

Usage:
    python scripts/execute_moveit_trajectory.py

How it works:
1. Set goal position with Interactive Marker in RViz
2. Click "Plan & Execute" button
3. This action server automatically sends to real robot!

Note: robot bridge must be running.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import time


class TrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('trajectory_executor')

        # QoS configuration
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publish joint commands (send to robot bridge)
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            qos
        )

        # FollowJointTrajectory action server (called by MoveIt)
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/robot_controller/follow_joint_trajectory',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Robot bridge standard joint order (joint_names order from so101_scanned.yaml)
        # MoveIt uses alphabetical order so reordering is needed
        self.robot_joint_order = [
            'shoulder_pan',
            'shoulder_lift',
            'elbow_flex',
            'wrist_flex',
            'wrist_roll',
            'gripper'
        ]

        self.get_logger().info('✅ Trajectory Executor Action Server ready!')
        self.get_logger().info('   Listening on: /robot_controller/follow_joint_trajectory')
        self.get_logger().info('   Publishing to: /joint_commands')
        self.get_logger().info(f'   Robot joint order: {self.robot_joint_order}')
        self.get_logger().info('')
        self.get_logger().info('💡 Now you can use "Plan & Execute" in RViz!')

        self.executing = False
        self.cancel_requested = False

    def goal_callback(self, goal_request):
        """Handle new goal request"""
        if self.executing:
            self.get_logger().warn('Already executing, rejecting new goal')
            return GoalResponse.REJECT

        self.get_logger().info('Received new trajectory execution goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Handle cancel request"""
        self.get_logger().info('Received cancel request')
        self.cancel_requested = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute trajectory (async)"""
        self.executing = True
        self.cancel_requested = False

        trajectory = goal_handle.request.trajectory

        self.get_logger().info("=" * 80)
        self.get_logger().info(f'📍 Executing trajectory: {len(trajectory.points)} waypoints')
        self.get_logger().info(f'   MoveIt joint order: {trajectory.joint_names}')

        # Check first and last points
        if len(trajectory.points) > 0:
            first_point = trajectory.points[0]
            last_point = trajectory.points[-1]
            self.get_logger().info(f'   First point positions: {[f"{p:.4f}" for p in first_point.positions]}')
            self.get_logger().info(f'   Last point positions: {[f"{p:.4f}" for p in last_point.positions]}')
        self.get_logger().info("=" * 80)

        try:
            start_time = self.get_clock().now()

            for i, point in enumerate(trajectory.points):
                # Check for cancellation
                if self.cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().warn('Trajectory execution canceled')
                    self.executing = False
                    return FollowJointTrajectory.Result()

                # Calculate target time
                target_time_sec = point.time_from_start.sec + point.time_from_start.nanosec / 1e9

                # Current elapsed time
                elapsed = (self.get_clock().now() - start_time).nanoseconds / 1e9

                # Wait until target time
                sleep_time = target_time_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

                # Reorder from MoveIt joint order to robot joint order
                # MoveIt uses alphabetical order, but robot_bridge expects physical order
                reordered_positions = [0.0] * len(self.robot_joint_order)

                for moveit_idx, joint_name in enumerate(trajectory.joint_names):
                    if joint_name in self.robot_joint_order:
                        robot_idx = self.robot_joint_order.index(joint_name)
                        reordered_positions[robot_idx] = point.positions[moveit_idx]
                    else:
                        self.get_logger().warning(f'Unknown joint: {joint_name}')

                # Publish joint command
                cmd_msg = Float64MultiArray()
                cmd_msg.data = reordered_positions

                # Debug: Log first and last points with detailed info
                if i == 0 or i == len(trajectory.points) - 1:
                    self.get_logger().info(f'📤 Sending Point {i} to /joint_commands:')
                    self.get_logger().info(f'   MoveIt joint order: {trajectory.joint_names}')
                    self.get_logger().info(f'   MoveIt positions: {[f"{p:.4f}" for p in point.positions]}')
                    self.get_logger().info(f'   ⚙️  Reordering to robot joint order...')
                    self.get_logger().info(f'   Robot joint order: {self.robot_joint_order}')
                    self.get_logger().info(f'   Reordered positions: {[f"{p:.4f}" for p in reordered_positions]}')
                    for j, (joint_name, pos) in enumerate(zip(self.robot_joint_order, reordered_positions)):
                        self.get_logger().info(f'     [{j}] {joint_name} = {pos:.4f} rad')

                self.joint_cmd_pub.publish(cmd_msg)

                # Progress feedback
                if i % 10 == 0 or i == len(trajectory.points) - 1:
                    progress = (i + 1) / len(trajectory.points)
                    feedback_msg = FollowJointTrajectory.Feedback()
                    goal_handle.publish_feedback(feedback_msg)
                    self.get_logger().info(f'Progress: {progress*100:.1f}%')

            # Success
            goal_handle.succeed()
            self.get_logger().info('✅ Trajectory execution complete!')

        except Exception as e:
            self.get_logger().error(f'Error during execution: {e}')
            goal_handle.abort()

        finally:
            self.executing = False

        return FollowJointTrajectory.Result()


def main():
    rclpy.init()

    executor = TrajectoryExecutor()

    print("\n" + "="*60)
    print("MoveIt Trajectory Executor - Action Server")
    print("="*60)
    print("\n✅ Ready to execute trajectories from RViz!")
    print("\nInstructions:")
    print("1. In RViz, drag the Interactive Marker to desired position")
    print("2. Click 'Plan & Execute' button")
    print("3. Watch your real robot move! 🚀")
    print("\nPress Ctrl+C to exit")
    print("="*60 + "\n")

    try:
        rclpy.spin(executor)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        executor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
