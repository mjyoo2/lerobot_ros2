#!/usr/bin/env python3
"""
Isaac Sim Teleoperation Relay

Relays real robot joint states to Isaac Sim joint commands.
This enables teleoperation: move real robot → Isaac Sim mirrors the movement.

Flow:
  Real Robot → /joint_states → [THIS RELAY] → /isaac_joint_commands → Isaac Sim

Usage:
    python scripts/isaac_teleop_relay.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class IsaacTeleopRelay(Node):
    """Relay real robot joint states to Isaac Sim joint commands"""

    def __init__(self):
        super().__init__('isaac_teleop_relay')

        # Subscribe to real robot joint states
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publish to Isaac Sim joint commands (use JointState type for Isaac Sim Action Graph)
        self.pub = self.create_publisher(
            JointState,
            '/isaac_joint_commands',
            10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('Isaac Sim Teleoperation Relay Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Subscribing to: /joint_states (real robot)')
        self.get_logger().info('Publishing to:  /isaac_joint_commands (Isaac Sim)')
        self.get_logger().info('Message type:   sensor_msgs/JointState')
        self.get_logger().info('')
        self.get_logger().info('Move the real robot to see Isaac Sim follow!')
        self.get_logger().info('=' * 60)

        self.message_count = 0

    def joint_state_callback(self, msg: JointState):
        """
        Receive joint states from real robot and forward to Isaac Sim

        Args:
            msg: JointState from real robot (radians)
        """
        try:
            # Forward JointState message directly to Isaac Sim
            # Isaac Sim Action Graph expects JointState type
            self.pub.publish(msg)

            # Log every 30 messages (1 Hz at 30Hz rate)
            self.message_count += 1
            if self.message_count % 30 == 0:
                self.get_logger().info(f'Relaying: {len(msg.position)} joints')
                for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                    self.get_logger().info(f'  [{i}] {name}: {pos:.4f} rad')

        except Exception as e:
            self.get_logger().error(f'Relay error: {e}')


def main():
    """Main function"""
    rclpy.init()

    relay = IsaacTeleopRelay()

    try:
        rclpy.spin(relay)
    except KeyboardInterrupt:
        relay.get_logger().info('Shutting down...')
    finally:
        relay.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
