#!/usr/bin/env python3
"""
실시간으로 실제 로봇과 Isaac Sim의 joint 차이 확인
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointDiffChecker(Node):
    def __init__(self):
        super().__init__('joint_diff_checker')

        self.real_state = None
        self.isaac_state = None

        # Subscribe to both topics
        self.real_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.real_callback,
            10
        )

        self.isaac_sub = self.create_subscription(
            JointState,
            '/isaac_joint_states',
            self.isaac_callback,
            10
        )

        # Timer to compare
        self.timer = self.create_timer(0.5, self.compare_states)

        print("=" * 80)
        print("실시간 Joint 차이 모니터링")
        print("=" * 80)
        print("로봇을 움직여보세요! (특히 wrist_roll과 gripper)")
        print("=" * 80)

    def real_callback(self, msg):
        self.real_state = msg

    def isaac_callback(self, msg):
        self.isaac_state = msg

    def compare_states(self):
        if self.real_state is None or self.isaac_state is None:
            print("토픽 대기 중...")
            return

        print("\n" + "=" * 80)

        for i, name in enumerate(self.real_state.name):
            real_pos = self.real_state.position[i]
            isaac_pos = self.isaac_state.position[i]
            diff_rad = real_pos - isaac_pos
            diff_deg = math.degrees(diff_rad)

            # 차이가 큰 joint 강조
            marker = ""
            if abs(diff_deg) > 30:
                marker = " ⚠️  큰 차이!"

            print(f"[{i}] {name:15s}: Real={real_pos:7.3f}, Isaac={isaac_pos:7.3f}, "
                  f"Diff={diff_rad:7.3f} rad ({diff_deg:6.1f}°){marker}")

        print("=" * 80)

def main():
    rclpy.init()
    checker = JointDiffChecker()

    try:
        rclpy.spin(checker)
    except KeyboardInterrupt:
        print("\n종료")
    finally:
        checker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
