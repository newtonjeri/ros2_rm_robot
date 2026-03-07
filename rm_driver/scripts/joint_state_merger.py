#!/usr/bin/env python3
"""
Joint State Merger Node

Subscribes to individual arm joint state topics and publishes a merged
/joint_states message containing all joints from both arms.

Order: Right arm joints first (indices 0-6), then Left arm joints (indices 7-13).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMerger(Node):
    def __init__(self):
        super().__init__('joint_state_merger')

        # Declare parameters for topic names (configurable via launch)
        self.declare_parameter('right_arm_topic', '/right_arm/arm_joint_states')
        self.declare_parameter('left_arm_topic', '/left_arm/arm_joint_states')
        self.declare_parameter('merged_topic', '/joint_states')

        right_topic = self.get_parameter('right_arm_topic').get_parameter_value().string_value
        left_topic = self.get_parameter('left_arm_topic').get_parameter_value().string_value
        merged_topic = self.get_parameter('merged_topic').get_parameter_value().string_value

        # Cache for latest states
        self._right_state = None
        self._left_state = None

        # Subscribers (absolute topic names)
        self._right_sub = self.create_subscription(
            JointState, right_topic, self._right_callback, 10)
        self._left_sub = self.create_subscription(
            JointState, left_topic, self._left_callback, 10)

        # Publisher
        self._pub = self.create_publisher(JointState, merged_topic, 10)

        self.get_logger().info(
            f'Merging [{right_topic}] + [{left_topic}] -> [{merged_topic}]')

    def _right_callback(self, msg: JointState):
        self._right_state = msg
        self._publish_merged()

    def _left_callback(self, msg: JointState):
        self._left_state = msg
        self._publish_merged()

    def _publish_merged(self):
        # Only publish once we have data from both arms
        if self._right_state is None or self._left_state is None:
            return

        merged = JointState()
        # Use the latest timestamp
        if (self._right_state.header.stamp.sec > self._left_state.header.stamp.sec or
            (self._right_state.header.stamp.sec == self._left_state.header.stamp.sec and
             self._right_state.header.stamp.nanosec >= self._left_state.header.stamp.nanosec)):
            merged.header.stamp = self._right_state.header.stamp
        else:
            merged.header.stamp = self._left_state.header.stamp

        # Right arm first (indices 0-6), then Left arm (indices 7-13)
        merged.name = list(self._right_state.name) + list(self._left_state.name)
        merged.position = list(self._right_state.position) + list(self._left_state.position)
        merged.velocity = list(self._right_state.velocity) + list(self._left_state.velocity)
        merged.effort = list(self._right_state.effort) + list(self._left_state.effort)

        self._pub.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
