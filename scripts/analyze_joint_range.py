#!/usr/bin/env python3
"""
Joint 범위를 실시간으로 분석하여 URDF/calibration 매칭 확인
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class JointRangeAnalyzer(Node):
    def __init__(self):
        super().__init__('joint_range_analyzer')

        # Track min/max for each joint
        self.real_ranges = {}
        self.isaac_ranges = {}

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

        # Timer to show summary
        self.timer = self.create_timer(2.0, self.show_summary)

        print("=" * 80)
        print("Joint 범위 분석기")
        print("=" * 80)
        print("지금부터 로봇의 각 joint를 천천히 최대 범위로 움직여주세요!")
        print("특히 wrist_roll(모터5)과 gripper(모터6)를 집중적으로!")
        print("=" * 80)

    def real_callback(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i]

            if name not in self.real_ranges:
                self.real_ranges[name] = {'min': pos, 'max': pos}
            else:
                self.real_ranges[name]['min'] = min(self.real_ranges[name]['min'], pos)
                self.real_ranges[name]['max'] = max(self.real_ranges[name]['max'], pos)

    def isaac_callback(self, msg):
        for i, name in enumerate(msg.name):
            pos = msg.position[i]

            if name not in self.isaac_ranges:
                self.isaac_ranges[name] = {'min': pos, 'max': pos}
            else:
                self.isaac_ranges[name]['min'] = min(self.isaac_ranges[name]['min'], pos)
                self.isaac_ranges[name]['max'] = max(self.isaac_ranges[name]['max'], pos)

    def show_summary(self):
        if not self.real_ranges or not self.isaac_ranges:
            print("데이터 수집 중...")
            return

        print("\n" + "=" * 80)
        print("현재까지 관찰된 Joint 범위:")
        print("=" * 80)

        # URDF limits (from so101_new_calib.urdf)
        urdf_limits = {
            "shoulder_pan": (-1.91986, 1.91986),
            "shoulder_lift": (-1.74533, 1.74533),
            "elbow_flex": (-1.69, 1.69),
            "wrist_flex": (-1.65806, 1.65806),
            "wrist_roll": (-2.74385, 2.84121),
            "gripper": (-1.74533, 1.74533),  # Updated limit
        }

        for name in sorted(self.real_ranges.keys()):
            real_min = self.real_ranges[name]['min']
            real_max = self.real_ranges[name]['max']
            real_range = real_max - real_min

            isaac_min = self.isaac_ranges.get(name, {}).get('min', 0)
            isaac_max = self.isaac_ranges.get(name, {}).get('max', 0)
            isaac_range = isaac_max - isaac_min

            urdf_min, urdf_max = urdf_limits.get(name, (0, 0))

            print(f"\n{name}:")
            print(f"  Real:  [{real_min:7.3f}, {real_max:7.3f}]  range={real_range:6.3f} rad ({math.degrees(real_range):6.1f}°)")
            print(f"  Isaac: [{isaac_min:7.3f}, {isaac_max:7.3f}]  range={isaac_range:6.3f} rad ({math.degrees(isaac_range):6.1f}°)")
            print(f"  URDF:  [{urdf_min:7.3f}, {urdf_max:7.3f}]  (limit)")

            # Check if hitting URDF limits
            if abs(isaac_min - urdf_min) < 0.01:
                print(f"  ⚠️  Isaac이 URDF lower limit에 걸림!")
            if abs(isaac_max - urdf_max) < 0.01:
                print(f"  ⚠️  Isaac이 URDF upper limit에 걸림!")

            # Check if Real exceeds URDF
            if real_min < urdf_min:
                print(f"  ⚠️  실제 로봇이 URDF lower limit 초과! ({real_min:.3f} < {urdf_min:.3f})")
            if real_max > urdf_max:
                print(f"  ⚠️  실제 로봇이 URDF upper limit 초과! ({real_max:.3f} > {urdf_max:.3f})")

        print("=" * 80)

def main():
    rclpy.init()
    analyzer = JointRangeAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        print("\n\n최종 분석 완료!")
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
