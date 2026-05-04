from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class GoalPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__('goal_pose_publisher')

        goal_topic = self.declare_parameter(
            'goal_topic',
            '/hmi/control/goal_pose',
            ParameterDescriptor(description='Goal topic consumed by the control-side behavior node.'),
        ).value
        frame_id = self.declare_parameter(
            'frame_id',
            'world',
            ParameterDescriptor(description='Frame id used for the published goal pose.'),
        ).value
        goal_x = float(
            self.declare_parameter(
                'goal_x',
                0.0,
                ParameterDescriptor(description='Goal x coordinate in world frame.'),
            ).value
        )
        goal_y = float(
            self.declare_parameter(
                'goal_y',
                2.95,
                ParameterDescriptor(description='Goal y coordinate in world frame.'),
            ).value
        )
        goal_yaw = float(
            self.declare_parameter(
                'goal_yaw',
                1.570796,
                ParameterDescriptor(description='Goal yaw in radians.'),
            ).value
        )
        publish_rate_hz = float(
            self.declare_parameter(
                'publish_rate_hz',
                1.0,
                ParameterDescriptor(description='How often to republish the fixed goal pose.'),
            ).value
        )

        self._goal_topic = str(goal_topic)
        self._frame_id = str(frame_id)
        self._goal_x = goal_x
        self._goal_y = goal_y
        self._goal_yaw = goal_yaw
        self._publisher = self.create_publisher(PoseStamped, self._goal_topic, 10)
        self.create_timer(1.0 / max(publish_rate_hz, 0.1), self._publish_goal)

        self.get_logger().info(
            'Goal pose publisher is ready. '
            f'Publishing fixed goal ({self._goal_x:.2f}, {self._goal_y:.2f}, yaw={self._goal_yaw:.2f}) '
            f'on {self._goal_topic}.'
        )

    def _publish_goal(self) -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = self._goal_x
        msg.pose.position.y = self._goal_y

        half_yaw = self._goal_yaw * 0.5
        msg.pose.orientation.z = math.sin(half_yaw)
        msg.pose.orientation.w = math.cos(half_yaw)
        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GoalPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
